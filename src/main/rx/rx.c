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

#include "programming/logic_condition.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"


#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/settings.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/crsf.h"
#include "rx/eleres.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/fport.h"
#include "rx/fport2.h"
#include "rx/msp.h"
#include "rx/msp_override.h"
#include "rx/pwm.h"
#include "rx/rx_spi.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/srxl2.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/xbus.h"
#include "rx/ghst.h"
#include "rx/mavlink.h"

//#define DEBUG_RX_SIGNAL_LOSS

const char rcChannelLetters[] = "AERT";

static uint16_t rssi = 0;                  // range: [0;1023]
static timeUs_t lastMspRssiUpdateUs = 0;

#define MSP_RSSI_TIMEOUT_US     1500000   // 1.5 sec
#define RX_LQ_INTERVAL_MS       200
#define RX_LQ_TIMEOUT_MS        1000

static rxLinkQualityTracker_e rxLQTracker;
static rssiSource_e activeRssiSource;

static bool rxDataProcessingRequired = false;
static bool auxiliaryProcessingRequired = false;

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
static bool mspOverrideDataProcessingRequired = false;
#endif

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static uint8_t rxChannelCount;

static timeUs_t rxNextUpdateAtUs = 0;
static timeUs_t needRxSignalBefore = 0;
static timeUs_t suspendRxSignalUntil = 0;
static uint8_t skipRxSamples = 0;

static rcChannel_t rcChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxLinkStatistics_t rxLinkStatistics;
rxRuntimeConfig_t rxRuntimeConfig;
static uint8_t rcSampleIndex = 0;

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
    .halfDuplex = SETTING_SERIALRX_HALFDUPLEX_DEFAULT,
    .serialrx_provider = SERIALRX_PROVIDER,
#ifdef USE_RX_SPI
    .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
#endif
#ifdef USE_SPEKTRUM_BIND
    .spektrum_sat_bind = SETTING_SPEKTRUM_SAT_BIND_DEFAULT,
#endif
    .serialrx_inverted = SETTING_SERIALRX_INVERTED_DEFAULT,
    .mincheck = SETTING_MIN_CHECK_DEFAULT,
    .maxcheck = SETTING_MAX_CHECK_DEFAULT,
    .rx_min_usec = SETTING_RX_MIN_USEC_DEFAULT,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = SETTING_RX_MAX_USEC_DEFAULT,          // any of first 4 channels above this value will trigger rx loss detection
    .rssi_channel = SETTING_RSSI_CHANNEL_DEFAULT,
    .rssiMin = SETTING_RSSI_MIN_DEFAULT,
    .rssiMax = SETTING_RSSI_MAX_DEFAULT,
    .sbusSyncInterval = SETTING_SBUS_SYNC_INTERVAL_DEFAULT,
    .rcFilterFrequency = SETTING_RC_FILTER_FREQUENCY_DEFAULT,
#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    .mspOverrideChannels = SETTING_MSP_OVERRIDE_CHANNELS_DEFAULT,
#endif
    .rssi_source = SETTING_RSSI_SOURCE_DEFAULT,
#ifdef USE_SERIALRX_SRXL2
    .srxl2_unit_id = SETTING_SRXL2_UNIT_ID_DEFAULT,
    .srxl2_baud_fast = SETTING_SRXL2_BAUD_FAST_DEFAULT,
#endif
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
#ifdef USE_SERIALRX_SRXL2
    case SERIALRX_SRXL2:
        enabled = srxl2RxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
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
#ifdef USE_SERIALRX_FPORT2
    case SERIALRX_FPORT2:
        enabled = fport2RxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_GHST
    case SERIALRX_GHST:
        enabled = ghstRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_MAVLINK
    case SERIALRX_MAVLINK:
        enabled = mavlinkRxInit(rxConfig, rxRuntimeConfig);
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
    rcSampleIndex = 0;

    timeMs_t nowMs = millis();

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcChannels[i].raw = PWM_RANGE_MIDDLE;
        rcChannels[i].data = PWM_RANGE_MIDDLE;
        rcChannels[i].expiresAt = nowMs + MAX_INVALID_RX_PULSE_TIME;
    }

    rcChannels[THROTTLE].raw = (feature(FEATURE_REVERSIBLE_MOTORS)) ? PWM_RANGE_MIDDLE : rxConfig()->rx_min_usec;
    rcChannels[THROTTLE].data = rcChannels[THROTTLE].raw;

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
            rcChannel_t *armChannel = &rcChannels[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT];
            armChannel->raw = value;
            armChannel->data = value;
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

    rxChannelCount = MIN(MAX_SUPPORTED_RC_CHANNEL_COUNT, rxRuntimeConfig.channelCount);
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
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
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

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    if (rxSignalReceived) {
        if (currentTimeUs >= needRxSignalBefore) {
            rxSignalReceived = false;
        }
    }

    const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn(&rxRuntimeConfig);

    if (frameStatus & RX_FRAME_COMPLETE) {
        // RX_FRAME_COMPLETE updated the failsafe status regardless
        rxSignalReceived = (frameStatus & RX_FRAME_FAILSAFE) == 0;
        needRxSignalBefore = currentTimeUs + rxRuntimeConfig.rxSignalTimeout;
        rxDataProcessingRequired = true;
    }
    else if ((frameStatus & RX_FRAME_FAILSAFE) && rxSignalReceived) {
        // All other receiver statuses are allowed to report failsafe, but not allowed to leave it
        rxSignalReceived = false;
    }

    if (frameStatus & RX_FRAME_PROCESSING_REQUIRED) {
        auxiliaryProcessingRequired = true;
    }

    if (cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0) {
        rxDataProcessingRequired = true;
    }

    bool result = rxDataProcessingRequired || auxiliaryProcessingRequired;

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    if (rxConfig()->receiverType != RX_TYPE_MSP) {
        mspOverrideDataProcessingRequired = mspOverrideUpdateCheck(currentTimeUs, currentDeltaTime);
        result = result || mspOverrideDataProcessingRequired;
    }
#endif

    return result;
}

#define FILTERING_SAMPLE_COUNT  5
static uint16_t applyChannelFiltering(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RC_CHANNEL_COUNT][FILTERING_SAMPLE_COUNT];
    static bool rxSamplesCollected = false;

    // Update the recent samples
    rcSamples[chan][rcSampleIndex % FILTERING_SAMPLE_COUNT] = sample;

    // Until we have enough data - return unfiltered samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < FILTERING_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
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
    return quickMedianFilter5_16(rcSamples[chan]);
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    int16_t rcStaging[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    const timeMs_t currentTimeMs = millis();

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    if ((rxConfig()->receiverType != RX_TYPE_MSP) && mspOverrideDataProcessingRequired) {
        mspOverrideCalculateChannels(currentTimeUs);
    }
#endif

    if (auxiliaryProcessingRequired) {
        auxiliaryProcessingRequired = !rxRuntimeConfig.rcProcessFrameFn(&rxRuntimeConfig);
    }

    if (!rxDataProcessingRequired) {
        return false;
    }

    rxDataProcessingRequired = false;
    rxNextUpdateAtUs = currentTimeUs + DELAY_50_HZ;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    rxFlightChannelsValid = true;

    // Read and process channel data
    for (int channel = 0; channel < rxChannelCount; channel++) {
        const uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration to flight channel
        if (channel < NON_AUX_CHANNEL_COUNT && sample != PPM_RCVR_TIMEOUT) {
            sample = scaleRange(sample, rxChannelRangeConfigs(channel)->min, rxChannelRangeConfigs(channel)->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
            sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);
        }

        // Store as rxRaw
        rcChannels[channel].raw = sample;

        // Apply invalid pulse value logic
        if (!isRxPulseValid(sample)) {
            sample = rcChannels[channel].data;   // hold channel, replace with old value
            if ((currentTimeMs > rcChannels[channel].expiresAt) && (channel < NON_AUX_CHANNEL_COUNT)) {
                rxFlightChannelsValid = false;
            }
        } else {
            rcChannels[channel].expiresAt = currentTimeMs + MAX_INVALID_RX_PULSE_TIME;
        }

        // Save channel value
        rcStaging[channel] = sample;
    }

    // Update channel input value if receiver is not in failsafe mode
    // If receiver is in failsafe (not receiving signal or sending invalid channel values) - last good input values are retained
    if (rxFlightChannelsValid && rxSignalReceived) {
        if (rxRuntimeConfig.requireFiltering) {
            for (int channel = 0; channel < rxChannelCount; channel++) {
                rcChannels[channel].data = applyChannelFiltering(channel, rcStaging[channel]);
            }
        } else {
            for (int channel = 0; channel < rxChannelCount; channel++) {
                rcChannels[channel].data = rcStaging[channel];
            }
        }
    }

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe()) {
        mspOverrideChannels(rcChannels);
    }
#endif

    // Update failsafe
    if (rxFlightChannelsValid && rxSignalReceived) {
        failsafeOnValidDataReceived();
    } else {
        failsafeOnValidDataFailed();
    }

    rcSampleIndex++;
    return true;
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
        int pwmRssi = rcChannels[rxConfig()->rssi_channel - 1].raw;
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
    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL)) {
        return getRcChannelOverride(channelNumber, rcChannels[channelNumber].data);
    } else {
        return rcChannels[channelNumber].data;
    }
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
