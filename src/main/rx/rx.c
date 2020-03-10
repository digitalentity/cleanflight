/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "scheduler/scheduler.h"

#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/crsf.h"
#include "rx/eleres.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/fport.h"
#include "rx/msp.h"
#include "rx/msp_override.h"
#include "rx/pwm.h"
#include "rx/rx_spi.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/uib_rx.h"
#include "rx/xbus.h"


#define FILTERING_SAMPLE_COUNT  5

typedef struct {
    int16_t samples[MAX_SUPPORTED_RC_CHANNEL_COUNT][FILTERING_SAMPLE_COUNT];
    int     totalSamplesCollected;
    bool    enoughSamplesCollected;
} rxFilterState_t;

typedef struct {
    rxReceiverStatus_t  rcStatus;
    rcChannel_t         rcChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxFilterState_t     rcFilterState;
    timeUs_t            lastDriverUpdateTimeUs;        // Last time we got RX_FRAME_COMPLETE from the driver
    bool                auxiliaryProcessingRequired;   // Driver asks us to call rcProcessFrameFn()
    bool                dataProcessingRequired;
    bool                flightChannelsValid;
} rxState_t;

static rxState_t        rxState;
static schdSemaphore_t  rxDataAvailableSemaphore;


const char rcChannelLetters[] = "AERT";

static uint16_t rssi = 0;                  // range: [0;1023]
static timeUs_t lastMspRssiUpdateUs = 0;

#define MSP_RSSI_TIMEOUT_US     1500000   // 1.5 sec
#define RX_LQ_INTERVAL_MS       200
#define RX_LQ_TIMEOUT_MS        1000

static rxLinkQualityTracker_e rxLQTracker;
static rssiSource_e activeRssiSource;

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
static bool mspOverrideDataProcessingRequired = false;
#endif

static timeUs_t suspendRxSignalUntil = 0;
static uint8_t skipRxSamples = 0;

#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeConfig_t rxRuntimeConfig;

PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 9);

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#ifndef DEFAULT_RX_TYPE
#define DEFAULT_RX_TYPE   RX_TYPE_NONE
#endif

#define RX_MIN_USEX 885
PG_RESET_TEMPLATE(rxConfig_t, rxConfig,
    .receiverType = DEFAULT_RX_TYPE,
    .rcmap = {0, 1, 3, 2},      // Default to AETR map
    .halfDuplex = 0,
    .serialrx_provider = SERIALRX_PROVIDER,
    .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
    .spektrum_sat_bind = 0,
    .serialrx_inverted = 0,
    .mincheck = 1100,
    .maxcheck = 1900,
    .rx_min_usec = RX_MIN_USEX,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = 2115,         // any of first 4 channels above this value will trigger rx loss detection
    .rssi_channel = 0,
    .rssiMin = RSSI_VISIBLE_VALUE_MIN,
    .rssiMax = RSSI_VISIBLE_VALUE_MAX,
    .sbusSyncInterval = SBUS_DEFAULT_INTERFRAME_DELAY_US,
    .rcFilterFrequency = 50,
#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    .mspOverrideChannels = 15,
#endif
    .rssi_source = RSSI_SOURCE_AUTO,
);

void resetAllRxChannelRangeConfigurations(void)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigsMutable(i)->min = PWM_RANGE_MIN;
        rxChannelRangeConfigsMutable(i)->max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);

void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);
    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    return RX_FRAME_PENDING;
}

bool isRxPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeConfig);
        break;
    case SERIALRX_SBUS_FAST:
        enabled = sbusInitFast(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    lqTrackerReset(&rxLQTracker);

    rxRuntimeConfig.lqTracker = &rxLQTracker;
    rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
    rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeConfig.rxSignalTimeout = DELAY_10_HZ;
    rxRuntimeConfig.requireFiltering = false;
    
    timeMs_t nowMs = millis();

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxState.rcChannels[i].raw = PWM_RANGE_MIDDLE;
        rxState.rcChannels[i].data = PWM_RANGE_MIDDLE;
        rxState.rcChannels[i].lastUpdatedMs = nowMs;
    }

    rxState.rcChannels[THROTTLE].raw = (feature(FEATURE_REVERSIBLE_MOTORS)) ? PWM_RANGE_MIDDLE : rxConfig()->rx_min_usec;
    rxState.rcChannels[THROTTLE].data = rxState.rcChannels[THROTTLE].raw;

    // Initialize ARM switch to OFF position when arming via switch is defined
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        if (modeActivationConditions(i)->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationConditions(i)->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationConditions(i)->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rxState.rcChannels[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT].raw = value;
            rxState.rcChannels[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT].data = value;
        }
    }

    switch (rxConfig()->receiverType) {
#if defined(USE_RX_PPM)
        case RX_TYPE_PPM:
            if (!rxPpmInit(&rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

#ifdef USE_SERIAL_RX
        case RX_TYPE_SERIAL:
            if (!serialRxInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

#ifdef USE_RX_MSP
        case RX_TYPE_MSP:
            rxMspInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_UIB
        case RX_TYPE_UIB:
            rxUIBInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_SPI
        case RX_TYPE_SPI:
            if (!rxSpiInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

        default:
        case RX_TYPE_NONE:
            rxConfigMutable()->receiverType = RX_TYPE_NONE;
            rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
            rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            break;
    }

    rxUpdateRSSISource();

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    if (rxConfig()->receiverType != RX_TYPE_MSP) {
        mspOverrideInit();
    }
#endif
}

void rxUpdateRSSISource(void)
{
    activeRssiSource = RSSI_SOURCE_NONE;

    if (rxConfig()->rssi_source == RSSI_SOURCE_NONE) {
        return;
    }

#if defined(USE_ADC)
    if (rxConfig()->rssi_source == RSSI_SOURCE_ADC || rxConfig()->rssi_source == RSSI_SOURCE_AUTO) {
        if (feature(FEATURE_RSSI_ADC)) {
            activeRssiSource = RSSI_SOURCE_ADC;
            return;
        }
    }
#endif

    if (rxConfig()->rssi_source == RSSI_SOURCE_RX_CHANNEL || rxConfig()->rssi_source == RSSI_SOURCE_AUTO) {
        if (rxConfig()->rssi_channel > 0) {
            activeRssiSource = RSSI_SOURCE_RX_CHANNEL;
            return;
        }
    }

    if (rxConfig()->rssi_source == RSSI_SOURCE_RX_PROTOCOL || rxConfig()->rssi_source == RSSI_SOURCE_AUTO) {
        activeRssiSource = RSSI_SOURCE_RX_PROTOCOL;
        return;
    }
}

uint8_t calculateChannelRemapping(const uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
{
    if (channelToRemap < channelMapEntryCount) {
        return channelMap[channelToRemap];
    }
    return channelToRemap;
}

bool rxIsReceivingSignal(void)
{
    return rxState.rcStatus == RX_RC_STATUS_OK;
}

bool rxAreFlightChannelsValid(void)
{
    return rxState.flightChannelsValid;
}

void suspendRxSignal(void)
{
    failsafeOnRxSuspend();
    suspendRxSignalUntil = micros() + SKIP_RC_ON_SUSPEND_PERIOD;
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
}

void resumeRxSignal(void)
{
    suspendRxSignalUntil = micros();
    skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    failsafeOnRxResume();
}

void parseRcChannels(const char *input)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
            rxConfigMutable()->rcmap[s - rcChannelLetters] = c - input;
    }
}

#define RSSI_SAMPLE_COUNT 16

static void setRSSIValue(uint16_t rssiValue, rssiSource_e source, bool filtered)
{
    if (source != activeRssiSource) {
        return;
    }

    static uint16_t rssiSamples[RSSI_SAMPLE_COUNT];
    static uint8_t rssiSampleIndex = 0;
    static unsigned sum = 0;

    if (filtered) {
        // Value is already filtered
        rssi = rssiValue;

    } else {
        sum = sum + rssiValue;
        sum = sum - rssiSamples[rssiSampleIndex];
        rssiSamples[rssiSampleIndex] = rssiValue;
        rssiSampleIndex = (rssiSampleIndex + 1) % RSSI_SAMPLE_COUNT;

        int16_t rssiMean = sum / RSSI_SAMPLE_COUNT;

        rssi = rssiMean;
    }

    // Apply min/max values
    int rssiMin = rxConfig()->rssiMin * RSSI_VISIBLE_FACTOR;
    int rssiMax = rxConfig()->rssiMax * RSSI_VISIBLE_FACTOR;
    if (rssiMin > rssiMax) {
        int tmp = rssiMax;
        rssiMax = rssiMin;
        rssiMin = tmp;
        int delta = rssi >= rssiMin ? rssi - rssiMin : 0;
        rssi = rssiMax >= delta ? rssiMax - delta : 0;
    }
    rssi = constrain(scaleRange(rssi, rssiMin, rssiMax, 0, RSSI_MAX_VALUE), 0, RSSI_MAX_VALUE);
}

void setRSSIFromMSP(uint8_t newMspRssi)
{
    if (activeRssiSource == RSSI_SOURCE_NONE && (rxConfig()->rssi_source == RSSI_SOURCE_MSP || rxConfig()->rssi_source == RSSI_SOURCE_AUTO)) {
        activeRssiSource = RSSI_SOURCE_MSP;
    }

    if (activeRssiSource == RSSI_SOURCE_MSP) {
        rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIFromChannel(void)
{
    if (rxConfig()->rssi_channel > 0) {
        int pwmRssi = rxState.rcChannels[rxConfig()->rssi_channel - 1].raw;
        int rawRSSI = (uint16_t)((constrain(pwmRssi - 1000, 0, 1000) / 1000.0f) * (RSSI_MAX_VALUE * 1.0f));
        setRSSIValue(rawRSSI, RSSI_SOURCE_RX_CHANNEL, false);
    }
}

static void updateRSSIFromADC(void)
{
#ifdef USE_ADC
    uint16_t rawRSSI = adcGetChannel(ADC_RSSI) / 4;    // Reduce to [0;1023]
    setRSSIValue(rawRSSI, RSSI_SOURCE_ADC, false);
#else
    setRSSIValue(0, RSSI_SOURCE_ADC, false);
#endif
}

static void updateRSSIFromProtocol(void)
{
    setRSSIValue(lqTrackerGet(&rxLQTracker), RSSI_SOURCE_RX_PROTOCOL, false);
}

void updateRSSI(timeUs_t currentTimeUs)
{
    // Read RSSI
    switch (activeRssiSource) {
    case RSSI_SOURCE_ADC:
        updateRSSIFromADC();
        break;
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIFromChannel();
        break;
    case RSSI_SOURCE_RX_PROTOCOL:
        updateRSSIFromProtocol();
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(currentTimeUs, lastMspRssiUpdateUs) > MSP_RSSI_TIMEOUT_US) {
            rssi = 0;
        }
        break;
    default:
        rssi = 0;
        break;
    }
}

uint16_t getRSSI(void)
{
    return rssi;
}

rssiSource_e getRSSISource(void)
{
    return activeRssiSource;
}

uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}

int16_t rxGetChannelValue(unsigned channelNumber)
{
    return rxState.rcChannels[channelNumber].data;
}

int16_t rxGetRawChannelValue(unsigned channelNumber)
{
    return rxState.rcChannels[channelNumber].raw;
}

void lqTrackerReset(rxLinkQualityTracker_e * lqTracker)
{
    lqTracker->lastUpdatedMs = millis();
    lqTracker->lqAccumulator = 0;
    lqTracker->lqCount = 0;
    lqTracker->lqValue = 0;
}

void lqTrackerAccumulate(rxLinkQualityTracker_e * lqTracker, uint16_t rawValue)
{
    const timeMs_t currentTimeMs = millis();

    if (((currentTimeMs - lqTracker->lastUpdatedMs) > RX_LQ_INTERVAL_MS) && lqTracker->lqCount) {
        lqTrackerSet(lqTracker, lqTracker->lqAccumulator / lqTracker->lqCount);
        lqTracker->lqAccumulator = 0;
        lqTracker->lqCount = 0;
    }

    lqTracker->lqAccumulator += rawValue;
    lqTracker->lqCount += 1;
}

void lqTrackerSet(rxLinkQualityTracker_e * lqTracker, uint16_t rawValue)
{
    lqTracker->lqValue = rawValue;
    lqTracker->lastUpdatedMs = millis();
}

uint16_t lqTrackerGet(rxLinkQualityTracker_e * lqTracker)
{
    if ((millis() - lqTracker->lastUpdatedMs) > RX_LQ_TIMEOUT_MS) {
        lqTracker->lqValue = 0;
    }

    return lqTracker->lqValue;
}

/*
bool taskUpdateRxCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    return rxUpdateCheck(currentTimeUs, currentDeltaTime);
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    processRx(currentTimeUs);
    isRXDataNew = true;
}
*/

static uint16_t applyChannelFiltering(rxFilterState_t * state, uint8_t chan, uint16_t sample)
{
    // Update the recent samples
    state->samples[chan][state->totalSamplesCollected % FILTERING_SAMPLE_COUNT] = sample;

    // Until we have enough data - return unfiltered samples
    if (!state->enoughSamplesCollected) {
        if (state->totalSamplesCollected < FILTERING_SAMPLE_COUNT) {
            return sample;
        }
        state->enoughSamplesCollected = true;
    }

    // Assuming a step transition from 1000 -> 2000 different filters will yield the following output:
    //  No filter:              1000, 2000, 2000, 2000, 2000        - 0 samples delay
    //  3-point moving average: 1000, 1333, 1667, 2000, 2000        - 2 samples delay
    //  3-point median:         1000, 1000, 2000, 2000, 2000        - 1 sample delay
    //  5-point median:         1000, 1000, 1000, 2000, 2000        - 2 sample delay

    // For the same filters - noise rejection capabilities (2 out of 5 outliers
    //  No filter:              1000, 2000, 1000, 2000, 1000, 1000, 1000
    //  3-point MA:             1000, 1333, 1333, 1667, 1333, 1333, 1000    - noise has reduced magnitude, but spread over more samples
    //  3-point median:         1000, 1000, 1000, 2000, 1000, 1000, 1000    - high density noise is not removed
    //  5-point median:         1000, 1000, 1000, 1000, 1000, 1000, 1000    - only 3 out of 5 outlier noise will get through

    // Apply 5-point median filtering. This filter has the same delay as 3-point moving average, but better noise rejection
    return quickMedianFilter5_16(state->samples[chan]);
}

TASK(taskUpdateRx)
{
    taskBegin();

    // Protothread has no permanent stack, all variables that should survive yield should be static
    rxState.rcStatus = RX_RC_STATUS_NO_SIGNAL;
    rxState.lastDriverUpdateTimeUs = 0;
    rxState.auxiliaryProcessingRequired = false;
    rxState.dataProcessingRequired = false;
    rxState.rcFilterState.totalSamplesCollected = 0;
    rxState.rcFilterState.enoughSamplesCollected = false;

    taskSemaphoreInit(&rxDataAvailableSemaphore);

    while (1) {
        // Check driver timeout
        if ((currentTimeUs - rxState.lastDriverUpdateTimeUs) >= rxRuntimeConfig.rxSignalTimeout) {
            rxState.rcStatus = RX_RC_STATUS_NO_SIGNAL;
        }

        // Call driver status function
        const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig);

        if (frameStatus & RX_FRAME_COMPLETE) {
            rxState.lastDriverUpdateTimeUs = currentTimeUs;
            rxState.dataProcessingRequired = true;
            rxState.rcStatus = ((frameStatus & RX_FRAME_FAILSAFE) != 0) ? RX_RC_STATUS_FAILSAFE : RX_RC_STATUS_OK;
        }

        if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
            rxState.auxiliaryProcessingRequired = true;
        }

        // Give up time quant - processing can wait for next wakeup
        taskYield();

        // Extra frame processing if necessary
        if (rxState.auxiliaryProcessingRequired) {
            rxState.auxiliaryProcessingRequired = !rxRuntimeConfig.rcProcessFrameFn(&rxRuntimeConfig);
        }

        // Process the frame
        if (rxState.dataProcessingRequired) {
            int rcStaging[MAX_SUPPORTED_RC_CHANNEL_COUNT];
            rxState.flightChannelsValid = true;
            timeMs_t currentTimeMs = millis();

            // Read and process channel data
            for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
                const uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

                // sample the channel
                uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

                // apply the rx calibration to flight channel
                if (channel < NON_AUX_CHANNEL_COUNT && sample != PPM_RCVR_TIMEOUT) {
                    sample = scaleRange(sample, rxChannelRangeConfigs(channel)->min, rxChannelRangeConfigs(channel)->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
                    sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);
                }

                // Store as rxRaw
                rxState.rcChannels[channel].raw = sample;

                // Apply invalid pulse value logic
                if (!isRxPulseValid(sample)) {
                    sample = rxState.rcChannels[channel].data;   // hold channel, replace with old value
                    if (((currentTimeMs - rxState.rcChannels[channel].lastUpdatedMs) > MAX_INVALID_RX_PULSE_TIME) && (channel < NON_AUX_CHANNEL_COUNT)) {
                        rxState.flightChannelsValid = false;
                    }
                } else {
                    rxState.rcChannels[channel].lastUpdatedMs = currentTimeMs;
                }

                // Save channel value
                rcStaging[channel] = sample;
            }

            // If channels are out of range - force 
            if (!rxState.flightChannelsValid) {
                rxState.rcStatus = RX_RC_STATUS_FAILSAFE;
            }

            // Update channel input value if receiver is not in failsafe mode
            // If receiver is in failsafe (not receiving signal or sending invalid channel values) - last good input values are retained
            if (rxState.rcStatus == RX_RC_STATUS_OK) {
                if (rxRuntimeConfig.requireFiltering) {
                    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
                        rxState.rcChannels[channel].data = applyChannelFiltering(&rxState.rcFilterState, channel, rcStaging[channel]);
                    }
                    rxState.rcFilterState.totalSamplesCollected++;
                } else {
                    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
                        rxState.rcChannels[channel].data = rcStaging[channel];
                    }
                }
            }

            rxState.dataProcessingRequired = false;

            // Signal new update to taskProcessRx thread
            taskSemaphoreSignal(&rxDataAvailableSemaphore);
        }

        taskYield();
    }

    taskEnd();
}

extern void signalRxDataNew(void);
extern void processRx(timeUs_t currentTimeUs);

TASK(taskProcessRx)
{
    taskBegin();

    while (1) {
        // Executed every RX update or at 10HZ
        taskWaitSemaphoreWithTimeoutUs(&rxDataAvailableSemaphore, HZ2US(10), NULL);

        // Update failsafe system
        if (rxState.rcStatus == RX_RC_STATUS_OK) {
            failsafeOnValidDataReceived();
        } else {
            failsafeOnValidDataFailed();
        }

        // Update RSSI

        // Process stick hooks

        // Process box modes

        // Process FLM selection

        // Mark RX data new for RC smoothing

        //processRx(currentTimeUs);
        //signalRxDataNew();
    }

    taskEnd();
}
