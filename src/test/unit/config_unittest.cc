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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include <platform.h>

    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"

    #include "config/parameter_group.h"

    #include "config/config.h"

    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"

    #include "io/escservo.h"
    #include "io/gimbal.h"
    #include "io/gps.h"
    #include "io/ledstrip.h"
    #include "io/rc_controls.h"
    #include "io/serial.h"
    #include "io/transponder_ir.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"
    #include "telemetry/hott.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/boardalignment.h"
    #include "sensors/barometer.h"
    #include "sensors/battery.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"

    #include "flight/pid.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/navigation.h"

    #include "rx/rx.h"

    #include "config/config_profile.h"
    #include "config/config_system.h"
    #include "config/feature.h"
    #include "config/profile.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void resetConf(void);

    profile_t testProfile;
    profile_t *currentProfile = &testProfile;

    rcControlsConfig_t testRcControlsConfig;
    rcControlsConfig_t *rcControlsConfig = &testRcControlsConfig;

    failsafeConfig_t failsafeConfig;
    boardAlignment_t boardAlignment;
    escAndServoConfig_t escAndServoConfig;
    sensorSelectionConfig_t sensorSelectionConfig;
    sensorAlignmentConfig_t sensorAlignmentConfig;
    sensorTrims_t sensorTrims;
    gyroConfig_t gyroConfig;
    batteryConfig_t batteryConfig;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];
    serialConfig_t serialConfig;
    pwmRxConfig_t pwmRxConfig;
    armingConfig_t armingConfig;
    transponderConfig_t transponderConfig;
    systemConfig_t systemConfig;
    mixerConfig_t mixerConfig;
    imuConfig_t imuConfig;
    profileSelection_t profileSelection;
    rxConfig_t rxConfig;
<<<<<<< HEAD
=======
    airplaneConfig_t airplaneConfig;
    gpsConfig_t gpsConfig;
<<<<<<< HEAD
>>>>>>> 3a52e51... extract gpsConfig_t from master_t.
=======
    telemetryConfig_t telemetryConfig;
    frskyTelemetryConfig_t frskyTelemetryConfig;
    hottTelemetryConfig_t hottTelemetryConfig;
>>>>>>> 206bbee... extract telemetryConfig_t from master_t and split into

    motor3DConfig_t motor3DConfig;

    gimbalConfig_t testGimbalConfig[MAX_PROFILE_COUNT];
    gimbalConfig_t *gimbalConfig = &testGimbalConfig[0];

    pidProfile_t testPidProfile[MAX_PROFILE_COUNT];
    pidProfile_t *pidProfile = &testPidProfile[0];

    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];

    void pgResetAll(uint8_t) {
        memset(&testProfile, 0x00, sizeof(testProfile));
        memset(&testRcControlsConfig, 0x00, sizeof(testRcControlsConfig));

        memset(&boardAlignment, 0x00, sizeof(boardAlignment));
        memset(&customMotorMixer, 0x00, sizeof(customMotorMixer));
        memset(&gyroConfig, 0x00, sizeof(gyroConfig));
        memset(&sensorTrims, 0x00, sizeof(sensorTrims));
        memset(&sensorAlignmentConfig, 0x00, sizeof(sensorAlignmentConfig));
        memset(&sensorSelectionConfig, 0x00, sizeof(sensorSelectionConfig));
        memset(&batteryConfig, 0x00, sizeof(batteryConfig));
        memset(&controlRateProfiles, 0x00, sizeof(controlRateProfiles));
        memset(&serialConfig, 0x00, sizeof(serialConfig));
        memset(&pwmRxConfig, 0x00, sizeof(pwmRxConfig));
        memset(&armingConfig, 0x00, sizeof(armingConfig));
        memset(&transponderConfig, 0x00, sizeof(transponderConfig));
        memset(&systemConfig, 0x00, sizeof(systemConfig));
        memset(&mixerConfig, 0x00, sizeof(mixerConfig));
        memset(&imuConfig, 0x00, sizeof(imuConfig));
        memset(&profileSelection, 0x00, sizeof(profileSelection));
        memset(&rxConfig, 0x00, sizeof(rxConfig));
<<<<<<< HEAD
=======
        memset(&airplaneConfig, 0x00, sizeof(airplaneConfig));
        memset(&gpsConfig, 0x00, sizeof(gpsConfig));
<<<<<<< HEAD
>>>>>>> 3a52e51... extract gpsConfig_t from master_t.
=======
        memset(&telemetryConfig, 0x00, sizeof(telemetryConfig));
        memset(&frskyTelemetryConfig, 0x00, sizeof(frskyTelemetryConfig));
        memset(&hottTelemetryConfig, 0x00, sizeof(hottTelemetryConfig));
>>>>>>> 206bbee... extract telemetryConfig_t from master_t and split into
    }

    void pgActivateProfile(uint8_t) {
    };


}

/*
 * Test that the config items whose default values are zero are indeed set to zero by resetConf().
 *
 */
TEST(ConfigUnittest, TestResetConfigZeroValues)
{
    //FIXME this needs updating since pgResetAll() is now what does the memset(), not resetConf()

    resetConf();

    EXPECT_EQ(0, profileSelection.current_profile_index); // default profile
    EXPECT_EQ(0, imuConfig.dcm_ki); // 0.003 * 10000

    EXPECT_EQ(0, sensorTrims.accZero.values.pitch);
    EXPECT_EQ(0, sensorTrims.accZero.values.roll);
    EXPECT_EQ(0, sensorTrims.accZero.values.yaw);

    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig.gyro_align);
    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig.acc_align);
    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig.mag_align);

    EXPECT_EQ(0, boardAlignment.rollDegrees);
    EXPECT_EQ(0, boardAlignment.pitchDegrees);
    EXPECT_EQ(0, boardAlignment.yawDegrees);

    EXPECT_EQ(ACC_DEFAULT, sensorSelectionConfig.acc_hardware);   // default/autodetect
    EXPECT_EQ(MAG_DEFAULT, sensorSelectionConfig.mag_hardware);   // default/autodetect
    EXPECT_EQ(BARO_DEFAULT, sensorSelectionConfig.baro_hardware); // default/autodetect

    EXPECT_EQ(0, batteryConfig.currentMeterOffset);
    EXPECT_EQ(0, batteryConfig.batteryCapacity);

    EXPECT_EQ(0, telemetryConfig.telemetry_inversion);
    EXPECT_EQ(0, telemetryConfig.telemetry_switch);

    EXPECT_EQ(0, frskyTelemetryConfig.gpsNoFixLatitude);
    EXPECT_EQ(0, frskyTelemetryConfig.gpsNoFixLongitude);
    EXPECT_EQ(FRSKY_FORMAT_DMS, frskyTelemetryConfig.frsky_coordinate_format);
    EXPECT_EQ(FRSKY_UNIT_METRICS, frskyTelemetryConfig.frsky_unit);
    EXPECT_EQ(0, frskyTelemetryConfig.frsky_vfas_precision);

    EXPECT_EQ(0, rxConfig.serialrx_provider);
    EXPECT_EQ(0, rxConfig.spektrum_sat_bind);

    EXPECT_EQ(0, rxConfig.rssi_channel);
    EXPECT_EQ(0, rxConfig.rssi_ppm_invert);
    EXPECT_EQ(0, rxConfig.rcSmoothing);

    EXPECT_EQ(INPUT_FILTERING_DISABLED, pwmRxConfig.inputFilteringMode);

    EXPECT_EQ(0, armingConfig.retarded_arm);

    EXPECT_EQ(0, mixerConfig.servo_lowpass_enable);

    EXPECT_EQ(GPS_NMEA, gpsConfig.provider);
    EXPECT_EQ(SBAS_AUTO, gpsConfig.sbasMode);
    EXPECT_EQ(GPS_AUTOBAUD_OFF, gpsConfig.autoBaud);

    EXPECT_EQ(0, systemConfig.emf_avoidance);

    EXPECT_EQ(0, controlRateProfiles[0].thrExpo8);
    EXPECT_EQ(0, controlRateProfiles[0].dynThrPID);
    EXPECT_EQ(0, controlRateProfiles[0].rcYawExpo8);
    for (uint8_t axis = 0; axis < FD_INDEX_COUNT; axis++) {
        EXPECT_EQ(0, controlRateProfiles[0].rates[axis]);
    }

    EXPECT_EQ(0, failsafeConfig.failsafe_kill_switch); // default failsafe switch action is identical to rc link loss
    EXPECT_EQ(0, failsafeConfig.failsafe_procedure);   // default full failsafe procedure is 0: auto-landing

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        EXPECT_EQ(0.0f, customMotorMixer[i].throttle);
    }
}

// STUBS
extern "C" {

void applyDefaultLedStripConfig(void) {}
void applyDefaultColors(void) {}
void beeperConfirmationBeeps(uint8_t) {}
void StopPwmAllMotors(void) {}
void useRxConfig(rxConfig_t *) {}
void useRcControlsConfig(modeActivationCondition_t *) {}
void useFailsafeConfig(void) {}
void useBarometerConfig(barometerConfig_t *) {}
void telemetryUseConfig(telemetryConfig_t *) {}
void suspendRxSignal(void) {}
void setAccelerationTrims(flightDynamicsTrims_t *) {}
void resumeRxSignal(void) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *) {}
void resetAdjustmentStates(void) {}
void pidSetController(pidControllerType_e) {}
void parseRcChannels(const char *, rxConfig_t *) {}
#ifdef USE_SERVOS
void mixerUseConfigs(servoParam_t *, airplaneConfig_t *) {}
#else
void mixerUseConfigs(escAndServoConfig_t *, airplaneConfig_t *) {}
#endif
bool isSerialConfigValid(serialConfig_t *) {return true;}
void imuConfigure(imuRuntimeConfig_t *, accDeadband_t *,float ,uint16_t) {}
void gpsUseProfile(gpsProfile_t *) {}
void gpsUsePIDs(pidProfile_t *) {}
void generateYawCurve(controlRateConfig_t *) {}
void generatePitchRollCurve(controlRateConfig_t *) {}
void generateThrottleCurve(controlRateConfig_t *) {}
void delay(uint32_t) {}
void configureAltitudeHold(barometerConfig_t *) {}
void failureMode(uint8_t) {}
bool scanEEPROM(bool) { return true; }
void writeConfigToEEPROM(void) {}
bool isEEPROMContentValid(void) { return true; }

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART1
    SERIAL_PORT_UART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_UART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_UART3,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};
}