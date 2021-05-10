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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"
#include "drivers/rx_spi.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/bus_i2c.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/rangefinder.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "navigation/navigation.h"

#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES 0
#endif
#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

#if !defined(VBAT_ADC_CHANNEL)
#define VBAT_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(RSSI_ADC_CHANNEL)
#define RSSI_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(CURRENT_METER_ADC_CHANNEL)
#define CURRENT_METER_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(AIRSPEED_ADC_CHANNEL)
#define AIRSPEED_ADC_CHANNEL ADC_CHN_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | COMMON_DEFAULT_FEATURES
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 3);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .current_profile_index = 0,
    .current_battery_profile_index = 0,
    .debug_mode = SETTING_DEBUG_MODE_DEFAULT,
#ifdef USE_I2C
    .i2c_speed = SETTING_I2C_SPEED_DEFAULT,
#endif
#ifdef USE_UNDERCLOCK
    .cpuUnderclock = SETTING_CPU_UNDERCLOCK_DEFAULT,
#endif
    .throttle_tilt_compensation_strength = SETTING_THROTTLE_TILT_COMP_STR_DEFAULT,      // 0-100, 0 - disabled
    .name = SETTING_NAME_DEFAULT
);

PG_REGISTER_WITH_RESET_TEMPLATE(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 1);

PG_RESET_TEMPLATE(beeperConfig_t, beeperConfig,
                  .beeper_off_flags = 0,
                  .preferred_beeper_off_flags = 0,
                  .dshot_beeper_enabled = SETTING_DSHOT_BEEPER_ENABLED_DEFAULT,
                  .dshot_beeper_tone = SETTING_DSHOT_BEEPER_TONE_DEFAULT,
);

PG_REGISTER_WITH_RESET_TEMPLATE(adcChannelConfig_t, adcChannelConfig, PG_ADC_CHANNEL_CONFIG, 0);

PG_RESET_TEMPLATE(adcChannelConfig_t, adcChannelConfig,
    .adcFunctionChannel = {
        [ADC_BATTERY]   = VBAT_ADC_CHANNEL,
        [ADC_RSSI]      = RSSI_ADC_CHANNEL,
        [ADC_CURRENT]   = CURRENT_METER_ADC_CHANNEL,
        [ADC_AIRSPEED]  = AIRSPEED_ADC_CHANNEL,
    }
);

#ifdef USE_NAV
void validateNavConfig(void)
{
    // Make sure minAlt is not more than maxAlt, maxAlt cannot be set lower than 500.
    navConfigMutable()->general.land_slowdown_minalt = MIN(navConfig()->general.land_slowdown_minalt, navConfig()->general.land_slowdown_maxalt - 100);
}
#endif


// Stubs to handle target-specific configs
__attribute__((weak)) void validateAndFixTargetConfig(void)
{
    __NOP();
}

__attribute__((weak)) void targetConfiguration(void)
{
    __NOP();
}


#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

uint32_t getLooptime(void) {
    return gyroConfig()->looptime;
}

uint32_t getGyroLooptime(void) {
    return gyro.targetLooptime;
}

void validateAndFixConfig(void)
{
    if (gyroConfig()->gyro_notch_cutoff >= gyroConfig()->gyro_notch_hz) {
        gyroConfigMutable()->gyro_notch_hz = 0;
    }
    if (accelerometerConfig()->acc_notch_cutoff >= accelerometerConfig()->acc_notch_hz) {
        accelerometerConfigMutable()->acc_notch_hz = 0;
    }

    // Disable unused features
    featureClear(FEATURE_UNUSED_1 | FEATURE_UNUSED_3 | FEATURE_UNUSED_4 | FEATURE_UNUSED_5 | FEATURE_UNUSED_6 | FEATURE_UNUSED_7 | FEATURE_UNUSED_8 | FEATURE_UNUSED_9 | FEATURE_UNUSED_10);

#if !defined(USE_RX_PPM)
    if (rxConfig()->receiverType == RX_TYPE_PPM) {
        rxConfigMutable()->receiverType = RX_TYPE_NONE;
    }
#endif


#if defined(USE_LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && featureConfigured(FEATURE_LED_STRIP)) {
        const timerHardware_t *ledTimerHardware = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);
        if (ledTimerHardware != NULL) {
            bool sameTimerUsed = false;

#if defined(USE_SOFTSERIAL1)
            const timerHardware_t *ss1TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_1_RX_PIN), TIM_USE_ANY);
            if (ss1TimerHardware != NULL && ss1TimerHardware->tim == ledTimerHardware->tim) {
                sameTimerUsed = true;
            }
#endif
#if defined(USE_SOFTSERIAL2)
            const timerHardware_t *ss2TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_2_RX_PIN), TIM_USE_ANY);
            if (ss2TimerHardware != NULL && ss2TimerHardware->tim == ledTimerHardware->tim) {
                sameTimerUsed = true;
            }
#endif
            if (sameTimerUsed) {
                // led strip needs the same timer as softserial
                featureClear(FEATURE_LED_STRIP);
            }
        }
    }
#endif

#ifndef USE_PWM_SERVO_DRIVER
    if (servoConfig()->servo_protocol == SERVO_TYPE_SERVO_DRIVER) {
        servoConfigMutable()->servo_protocol = SERVO_TYPE_PWM;
    }
#endif

#ifndef USE_SERVO_SBUS
    if (servoConfig()->servo_protocol == SERVO_TYPE_SBUS || servoConfig()->servo_protocol == SERVO_TYPE_SBUS_PWM) {
        servoConfigMutable()->servo_protocol = SERVO_TYPE_PWM;
    }
#endif

    if (!isSerialConfigValid(serialConfigMutable())) {
        pgResetCopy(serialConfigMutable(), PG_SERIAL_CONFIG);
    }

#if defined(USE_NAV)
    // Ensure sane values of navConfig settings
    validateNavConfig();
#endif

    // Limitations of different protocols
#if !defined(USE_DSHOT)
    if (motorConfig()->motorPwmProtocol > PWM_TYPE_BRUSHED) {
        motorConfigMutable()->motorPwmProtocol = PWM_TYPE_STANDARD;
    }
#endif

#ifdef BRUSHED_MOTORS
    motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
#else
    switch (motorConfig()->motorPwmProtocol) {
    default:
    case PWM_TYPE_STANDARD: // Limited to 490 Hz
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 490);
        break;
    case PWM_TYPE_ONESHOT125:   // Limited to 3900 Hz
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 3900);
        break;
    case PWM_TYPE_ONESHOT42:    // 2-8 kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 8000);
        break;
    case PWM_TYPE_MULTISHOT:    // 2-16 kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 16000);
        break;
    case PWM_TYPE_BRUSHED:      // 500Hz - 32kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
        break;
#ifdef USE_DSHOT
    // One DSHOT packet takes 16 bits x 19 ticks + 2uS = 304 timer ticks + 2uS
    case PWM_TYPE_DSHOT150:
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 4000);
        break;
    case PWM_TYPE_DSHOT300:
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 8000);
        break;
    // Although DSHOT 600+ support >16kHz update rate it's not practical because of increased CPU load
    // It's more reasonable to use slower-speed DSHOT at higher rate for better reliability
    case PWM_TYPE_DSHOT600:
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 16000);
        break;
    case PWM_TYPE_DSHOT1200:
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 32000);
        break;
#endif
#ifdef USE_SERIALSHOT
    case PWM_TYPE_SERIALSHOT:   // 2-4 kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 4000);
        break;
#endif
    }
#endif

    // Call target-specific validation function
    validateAndFixTargetConfig();

#ifdef USE_MAG
    if (compassConfig()->mag_align == ALIGN_DEFAULT) {
        compassConfigMutable()->mag_align = CW270_DEG_FLIP;
    }
#endif

    if (settingsValidate(NULL)) {
        DISABLE_ARMING_FLAG(ARMING_DISABLED_INVALID_SETTING);
    } else {
        ENABLE_ARMING_FLAG(ARMING_DISABLED_INVALID_SETTING);
    }
}

void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch)
{
    updateBoardAlignment(roll, pitch);
    saveConfigAndNotify();
}

// Default settings
void createDefaultConfig(void)
{
    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234");
#else
    parseRcChannels("AETR1234");
#endif

#ifdef USE_BLACKBOX
#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
    featureSet(FEATURE_BLACKBOX);
#endif
#endif

    featureSet(FEATURE_AIRMODE);

    targetConfiguration();
}

void resetConfigs(void)
{
    pgResetAll(MAX_PROFILE_COUNT);
    pgActivateProfile(0);

    createDefaultConfig();

    setConfigProfile(getConfigProfile());
#ifdef USE_LED_STRIP
    reevaluateLedConfig();
#endif
}

static void activateConfig(void)
{
    activateControlRateConfig();
    activateBatteryProfile();

    resetAdjustmentStates();

    updateUsedModeActivationConditionFlags();

    failsafeReset();

    accSetCalibrationValues();
    accInitFilters();

    imuConfigure();

    pidInit();

#ifdef USE_NAV
    navigationUsePIDs();
#endif
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    setConfigProfile(getConfigProfile());
    setConfigBatteryProfile(getConfigBatteryProfile());

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void writeEEPROM(void)
{
    suspendRxSignal();

    writeConfigToEEPROM();

    resumeRxSignal();
}

void resetEEPROM(void)
{
    resetConfigs();
    writeEEPROM();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

uint8_t getConfigProfile(void)
{
    return systemConfig()->current_profile_index;
}

bool setConfigProfile(uint8_t profileIndex)
{
    bool ret = true; // return true if current_profile_index has changed
    if (systemConfig()->current_profile_index == profileIndex) {
        ret =  false;
    }
    if (profileIndex >= MAX_PROFILE_COUNT) {// sanity check
        profileIndex = 0;
    }
    pgActivateProfile(profileIndex);
    systemConfigMutable()->current_profile_index = profileIndex;
    // set the control rate profile to match
    setControlRateProfile(profileIndex);
    return ret;
}

void setConfigProfileAndWriteEEPROM(uint8_t profileIndex)
{
    if (setConfigProfile(profileIndex)) {
        // profile has changed, so ensure current values saved before new profile is loaded
        writeEEPROM();
        readEEPROM();
    }
    beeperConfirmationBeeps(profileIndex + 1);
}

uint8_t getConfigBatteryProfile(void)
{
    return systemConfig()->current_battery_profile_index;
}

bool setConfigBatteryProfile(uint8_t profileIndex)
{
    bool ret = true; // return true if current_battery_profile_index has changed
    if (systemConfig()->current_battery_profile_index == profileIndex) {
        ret =  false;
    }
    if (profileIndex >= MAX_BATTERY_PROFILE_COUNT) {// sanity check
        profileIndex = 0;
    }
    systemConfigMutable()->current_battery_profile_index = profileIndex;
    setBatteryProfile(profileIndex);
    return ret;
}

void setConfigBatteryProfileAndWriteEEPROM(uint8_t profileIndex)
{
    if (setConfigBatteryProfile(profileIndex)) {
        // profile has changed, so ensure current values saved before new profile is loaded
        writeEEPROM();
        readEEPROM();
    }
    beeperConfirmationBeeps(profileIndex + 1);
}

void beeperOffSet(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
    beeperConfigMutable()->beeper_off_flags = (1 << beeperCount) -1;
}

void beeperOffClear(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
    beeperConfigMutable()->beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
    return beeperConfig()->beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
    return beeperConfig()->preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
    beeperConfigMutable()->preferred_beeper_off_flags = mask;
}
