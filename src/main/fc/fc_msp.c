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

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/bitarray.h"
#include "common/time.h"
#include "common/utils.h"
#include "programming/global_variables.h"
#include "programming/pid.h"

#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_msp.h"
#include "drivers/barometer/barometer_msp.h"
#include "drivers/pitotmeter/pitotmeter_msp.h"
#include "drivers/bus_i2c.h"
#include "drivers/display.h"
#include "drivers/flash.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sdcard/sdcard.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/vtx_common.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"
#include "fc/fc_msp_box.h"
#include "fc/firmware_update.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/hil.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/opflow.h"
#include "io/rangefinder.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/vtx.h"
#include "io/vtx_string.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/diagnostics.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/opflow.h"
#include "sensors/temperature.h"

#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

extern timeDelta_t cycleTime; // FIXME dependency on mw.c

static const char * const flightControllerIdentifier = INAV_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

// from mixer.c
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTTED   = 1
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_BIT_READY        = 1,
    MSP_FLASHFS_BIT_SUPPORTED    = 2
} mspFlashfsFlags_e;

typedef enum {
    MSP_PASSTHROUGH_SERIAL_ID = 0xFD,
    MSP_PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE,

    MSP_PASSTHROUGH_ESC_4WAY = 0xFF,
 } mspPassthroughType_e;

static uint8_t mspPassthroughMode;
static uint8_t mspPassthroughArgument;

static serialPort_t *mspFindPassthroughSerialPort(void)
 {
    serialPortUsage_t *portUsage = NULL;

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    {
        portUsage = findSerialPortUsageByIdentifier(mspPassthroughArgument);
        break;
    }
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
    {
        const serialPortConfig_t *portConfig = findSerialPortConfig(1 << mspPassthroughArgument);
        if (portConfig) {
            portUsage = findSerialPortUsageByIdentifier(portConfig->identifier);
        }
        break;
    }
    }
    return portUsage ? portUsage->serialPort : NULL;
}

static void mspSerialPassthroughFn(serialPort_t *serialPort)
{
    serialPort_t *passthroughPort = mspFindPassthroughSerialPort();
    if (passthroughPort && serialPort) {
        serialPassthrough(passthroughPort, serialPort, NULL, NULL);
    }
}

static void mspFcSetPassthroughCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);

    if (dataSize == 0) {
        // Legacy format
        mspPassthroughMode = MSP_PASSTHROUGH_ESC_4WAY;
    } else {
        mspPassthroughMode = sbufReadU8(src);
        if (!sbufReadU8Safe(&mspPassthroughArgument, src)) {
            mspPassthroughArgument = 0;
        }
    }

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
         if (mspFindPassthroughSerialPort()) {
             if (mspPostProcessFn) {
                 *mspPostProcessFn = mspSerialPassthroughFn;
             }
             sbufWriteU8(dst, 1);
         } else {
             sbufWriteU8(dst, 0);
         }
         break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;
#endif
    default:
        sbufWriteU8(dst, 0);
    }
}

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);
    fcReboot(false);
}

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
#ifdef USE_SDCARD
    uint8_t flags = MSP_SDCARD_FLAG_SUPPORTTED;
    uint8_t state;

    sbufWriteU8(dst, flags);

    // Merge the card and filesystem states together
    if (!sdcard_isInserted()) {
        state = MSP_SDCARD_STATE_NOT_PRESENT;
    } else if (!sdcard_isFunctional()) {
        state = MSP_SDCARD_STATE_FATAL;
    } else {
        switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                if (sdcard_isInitialized()) {
                    state = MSP_SDCARD_STATE_FS_INIT;
                } else {
                    state = MSP_SDCARD_STATE_CARD_INIT;
                }
                break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
        }
    }

    sbufWriteU8(dst, state);
    sbufWriteU8(dst, afatfs_getLastError());
    // Write free space and total space in kilobytes
    sbufWriteU32(dst, afatfs_getContiguousFreeSpace() / 1024);
    sbufWriteU32(dst, sdcard_getMetadata()->numBlocks / 2); // Block size is half a kilobyte
#else
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    const flashGeometry_t *geometry = flashGetGeometry();
    sbufWriteU8(dst, flashIsReady() ? 1 : 0);
    sbufWriteU32(dst, geometry->sectors);
    sbufWriteU32(dst, geometry->totalSize);
    sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
#else
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
#endif
}

#ifdef USE_FLASHFS
static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, uint16_t size)
{
    // Check how much bytes we can read
    const int bytesRemainingInBuf = sbufBytesRemaining(dst);
    uint16_t readLen = (size > bytesRemainingInBuf) ? bytesRemainingInBuf : size;

    // size will be lower than that requested if we reach end of volume
    const uint32_t flashfsSize = flashfsGetSize();
    if (readLen > flashfsSize - address) {
        // truncate the request
        readLen = flashfsSize - address;
    }

    // Write address
    sbufWriteU32(dst, address);

    // Read into streambuf directly
    const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);
    sbufAdvance(dst, bytesRead);
}
#endif

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspFcProcessOutCommand(uint16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
        // OSD support (for BF compatibility):
        // 0 = no OSD
        // 1 = OSD slave (not supported in INAV)
        // 2 = OSD chip on board
#if defined(USE_OSD)
        sbufWriteU8(dst, 2);
#else
        sbufWriteU8(dst, 0);
#endif
        // Board communication capabilities (uint8)
        // Bit 0: 1 iff the board has VCP
        // Bit 1: 1 iff the board supports software serial
        uint8_t commCapabilities = 0;
#ifdef USE_VCP
        commCapabilities |= 1 << 0;
#endif
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
        commCapabilities |= 1 << 1;
#endif
        sbufWriteU8(dst, commCapabilities);

        sbufWriteU8(dst, strlen(targetName));
        sbufWriteData(dst, targetName, strlen(targetName));
        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        break;

    // DEPRECATED - Use MSP_API_VERSION
    case MSP_IDENT:
        sbufWriteU8(dst, MW_VERSION);
        sbufWriteU8(dst, 3); //We no longer have mixerMode, just sent 3 (QuadX) as fallback
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU32(dst, CAP_PLATFORM_32BIT | CAP_DYNBALANCE | CAP_FLAPS | CAP_NAVCAP | CAP_EXTAUX); // "capability"
        break;

#ifdef HIL
    case MSP_HIL_STATE:
        sbufWriteU16(dst, hilToSIM.pidCommand[ROLL]);
        sbufWriteU16(dst, hilToSIM.pidCommand[PITCH]);
        sbufWriteU16(dst, hilToSIM.pidCommand[YAW]);
        sbufWriteU16(dst, hilToSIM.pidCommand[THROTTLE]);
        break;
#endif

    case MSP_SENSOR_STATUS:
        sbufWriteU8(dst, isHardwareHealthy() ? 1 : 0);
        sbufWriteU8(dst, getHwGyroStatus());
        sbufWriteU8(dst, getHwAccelerometerStatus());
        sbufWriteU8(dst, getHwCompassStatus());
        sbufWriteU8(dst, getHwBarometerStatus());
        sbufWriteU8(dst, getHwGPSStatus());
        sbufWriteU8(dst, getHwRangefinderStatus());
        sbufWriteU8(dst, getHwPitotmeterStatus());
        sbufWriteU8(dst, getHwOpticalFlowStatus());
        break;

    case MSP_ACTIVEBOXES:
        {
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);
            sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
        }
        break;

    case MSP_STATUS_EX:
    case MSP_STATUS:
        {
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);

            sbufWriteU16(dst, (uint16_t)cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, packSensorStatus());
            sbufWriteData(dst, &mspBoxModeFlags, 4);
            sbufWriteU8(dst, getConfigProfile());

            if (cmdMSP == MSP_STATUS_EX) {
                sbufWriteU16(dst, averageSystemLoadPercent);
                sbufWriteU16(dst, armingFlags);
                sbufWriteU8(dst, accGetCalibrationAxisFlags());
            }
        }
        break;

        case MSP2_INAV_STATUS:
        {
            // Preserves full arming flags and box modes
            boxBitmask_t mspBoxModeFlags;
            packBoxModeFlags(&mspBoxModeFlags);

            sbufWriteU16(dst, (uint16_t)cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, packSensorStatus());
            sbufWriteU16(dst, averageSystemLoadPercent);
            sbufWriteU8(dst, (getConfigBatteryProfile() << 4) | getConfigProfile());
            sbufWriteU32(dst, armingFlags);
            sbufWriteData(dst, &mspBoxModeFlags, sizeof(mspBoxModeFlags));
        }
        break;

    case MSP_RAW_IMU:
        {
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, (int16_t)lrintf(acc.accADCf[i] * 512));
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
#ifdef USE_MAG
                sbufWriteU16(dst, mag.magADC[i]);
#else
                sbufWriteU16(dst, 0);
#endif
            }
        }
        break;

    case MSP_SERVO:
        sbufWriteData(dst, &servo, MAX_SUPPORTED_SERVOS * 2);
        break;
    case MSP_SERVO_CONFIGURATIONS:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, servoParams(i)->min);
            sbufWriteU16(dst, servoParams(i)->max);
            sbufWriteU16(dst, servoParams(i)->middle);
            sbufWriteU8(dst, servoParams(i)->rate);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 255); // used to be forwardFromChannel, not used anymore, send 0xff for compatibility reasons
            sbufWriteU32(dst, 0); //Input reversing is not required since it can be done on mixer level
        }
        break;
    case MSP_SERVO_MIX_RULES:
        for (int i = 0; i < MAX_SERVO_RULES; i++) {
            sbufWriteU8(dst, customServoMixers(i)->targetChannel);
            sbufWriteU8(dst, customServoMixers(i)->inputSource);
            sbufWriteU16(dst, customServoMixers(i)->rate);
            sbufWriteU8(dst, customServoMixers(i)->speed);
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, 100);
            sbufWriteU8(dst, 0);
        }
        break;
    case MSP2_INAV_SERVO_MIXER:
        for (int i = 0; i < MAX_SERVO_RULES; i++) {
            sbufWriteU8(dst, customServoMixers(i)->targetChannel);
            sbufWriteU8(dst, customServoMixers(i)->inputSource);
            sbufWriteU16(dst, customServoMixers(i)->rate);
            sbufWriteU8(dst, customServoMixers(i)->speed);
        #ifdef USE_PROGRAMMING_FRAMEWORK
            sbufWriteU8(dst, customServoMixers(i)->conditionId);
        #else
            sbufWriteU8(dst, -1);
        #endif
        }
        break;
#ifdef USE_PROGRAMMING_FRAMEWORK
    case MSP2_INAV_LOGIC_CONDITIONS:
        for (int i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
            sbufWriteU8(dst, logicConditions(i)->enabled);
            sbufWriteU8(dst, logicConditions(i)->activatorId);
            sbufWriteU8(dst, logicConditions(i)->operation);
            sbufWriteU8(dst, logicConditions(i)->operandA.type);
            sbufWriteU32(dst, logicConditions(i)->operandA.value);
            sbufWriteU8(dst, logicConditions(i)->operandB.type);
            sbufWriteU32(dst, logicConditions(i)->operandB.value);
            sbufWriteU8(dst, logicConditions(i)->flags);
        }
        break;
    case MSP2_INAV_LOGIC_CONDITIONS_STATUS:
        for (int i = 0; i < MAX_LOGIC_CONDITIONS; i++) {
            sbufWriteU32(dst, logicConditionGetValue(i));
        }
        break;
    case MSP2_INAV_GVAR_STATUS:
        for (int i = 0; i < MAX_GLOBAL_VARIABLES; i++) {
            sbufWriteU32(dst, gvGet(i));
        }
        break;
    case MSP2_INAV_PROGRAMMING_PID:
        for (int i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
            sbufWriteU8(dst, programmingPids(i)->enabled);
            sbufWriteU8(dst, programmingPids(i)->setpoint.type);
            sbufWriteU32(dst, programmingPids(i)->setpoint.value);
            sbufWriteU8(dst, programmingPids(i)->measurement.type);
            sbufWriteU32(dst, programmingPids(i)->measurement.value);
            sbufWriteU16(dst, programmingPids(i)->gains.P);
            sbufWriteU16(dst, programmingPids(i)->gains.I);
            sbufWriteU16(dst, programmingPids(i)->gains.D);
            sbufWriteU16(dst, programmingPids(i)->gains.FF);
        }
        break;
    case MSP2_INAV_PROGRAMMING_PID_STATUS:
        for (int i = 0; i < MAX_PROGRAMMING_PID_COUNT; i++) {
            sbufWriteU32(dst, programmingPidGetOutput(i));
        }
        break;
#endif
    case MSP2_COMMON_MOTOR_MIXER:
        for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            sbufWriteU16(dst, primaryMotorMixer(i)->throttle * 1000);
            sbufWriteU16(dst, constrainf(primaryMotorMixer(i)->roll + 2.0f, 0.0f, 4.0f) * 1000);
            sbufWriteU16(dst, constrainf(primaryMotorMixer(i)->pitch + 2.0f, 0.0f, 4.0f) * 1000);
            sbufWriteU16(dst, constrainf(primaryMotorMixer(i)->yaw + 2.0f, 0.0f, 4.0f) * 1000);
        }
        break;

    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++) {
            sbufWriteU16(dst, i < MAX_SUPPORTED_MOTORS ? motor[i] : 0);
        }
        break;

    case MSP_RC:
        for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
            sbufWriteU16(dst, rxGetChannelValue(i));
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
#if defined(USE_NAV)
        sbufWriteU32(dst, lrintf(getEstimatedActualPosition(Z)));
        sbufWriteU16(dst, lrintf(getEstimatedActualVelocity(Z)));
#else
        sbufWriteU32(dst, 0);
        sbufWriteU16(dst, 0);
#endif
#if defined(USE_BARO)
        sbufWriteU32(dst, baroGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP_SONAR_ALTITUDE:
#ifdef USE_RANGEFINDER
        sbufWriteU32(dst, rangefinderGetLatestAltitude());
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP2_INAV_OPTICAL_FLOW:
#ifdef USE_OPFLOW
        sbufWriteU8(dst, opflow.rawQuality);
        sbufWriteU16(dst, RADIANS_TO_DEGREES(opflow.flowRate[X]));
        sbufWriteU16(dst, RADIANS_TO_DEGREES(opflow.flowRate[Y]));
        sbufWriteU16(dst, RADIANS_TO_DEGREES(opflow.bodyRate[X]));
        sbufWriteU16(dst, RADIANS_TO_DEGREES(opflow.bodyRate[Y]));
#else
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(getBatteryVoltage() / 10, 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, getRSSI());
        sbufWriteU16(dst, (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
        break;

    case MSP2_INAV_ANALOG:
        // Bit 1: battery full, Bit 2: use capacity threshold, Bit 3-4: battery state, Bit 5-8: battery cell count
        sbufWriteU8(dst, batteryWasFullWhenPluggedIn() | (batteryUsesCapacityThresholds() << 1) | (getBatteryState() << 2) | (getBatteryCellCount() << 4));
        sbufWriteU16(dst, getBatteryVoltage());
        sbufWriteU16(dst, getAmperage()); // send amperage in 0.01 A steps
        sbufWriteU32(dst, getPower());    // power draw
        sbufWriteU32(dst, getMAhDrawn()); // milliamp hours drawn from battery
        sbufWriteU32(dst, getMWhDrawn()); // milliWatt hours drawn from battery
        sbufWriteU32(dst, getBatteryRemainingCapacity());
        sbufWriteU8(dst, calculateBatteryPercentage());
        sbufWriteU16(dst, getRSSI());
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, armingConfig()->disarm_kill_switch);
        break;

    case MSP_LOOP_TIME:
        sbufWriteU16(dst, gyroConfig()->looptime);
        break;

    case MSP_RC_TUNING:
        sbufWriteU8(dst, 100); //rcRate8 kept for compatibity reasons, this setting is no longer used
        sbufWriteU8(dst, currentControlRateProfile->stabilized.rcExpo8);
        for (int i = 0 ; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->stabilized.rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        sbufWriteU8(dst, currentControlRateProfile->throttle.dynPID);
        sbufWriteU8(dst, currentControlRateProfile->throttle.rcMid8);
        sbufWriteU8(dst, currentControlRateProfile->throttle.rcExpo8);
        sbufWriteU16(dst, currentControlRateProfile->throttle.pa_breakpoint);
        sbufWriteU8(dst, currentControlRateProfile->stabilized.rcYawExpo8);
        break;

    case MSP2_INAV_RATE_PROFILE:
        // throttle
        sbufWriteU8(dst, currentControlRateProfile->throttle.rcMid8);
        sbufWriteU8(dst, currentControlRateProfile->throttle.rcExpo8);
        sbufWriteU8(dst, currentControlRateProfile->throttle.dynPID);
        sbufWriteU16(dst, currentControlRateProfile->throttle.pa_breakpoint);

        // stabilized
        sbufWriteU8(dst, currentControlRateProfile->stabilized.rcExpo8);
        sbufWriteU8(dst, currentControlRateProfile->stabilized.rcYawExpo8);
        for (uint8_t i = 0 ; i < 3; ++i) {
            sbufWriteU8(dst, currentControlRateProfile->stabilized.rates[i]); // R,P,Y see flight_dynamics_index_t
        }

        // manual
        sbufWriteU8(dst, currentControlRateProfile->manual.rcExpo8);
        sbufWriteU8(dst, currentControlRateProfile->manual.rcYawExpo8);
        for (uint8_t i = 0 ; i < 3; ++i) {
            sbufWriteU8(dst, currentControlRateProfile->manual.rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        break;

    case MSP_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU8(dst, constrain(pidBank()->pid[i].P, 0, 255));
            sbufWriteU8(dst, constrain(pidBank()->pid[i].I, 0, 255));
            sbufWriteU8(dst, constrain(pidBank()->pid[i].D, 0, 255));
        }
        break;

    case MSP2_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU8(dst, constrain(pidBank()->pid[i].P, 0, 255));
            sbufWriteU8(dst, constrain(pidBank()->pid[i].I, 0, 255));
            sbufWriteU8(dst, constrain(pidBank()->pid[i].D, 0, 255));
            sbufWriteU8(dst, constrain(pidBank()->pid[i].FF, 0, 255));
        }
        break;

    case MSP_PIDNAMES:
        for (const char *c = pidnames; *c; c++) {
            sbufWriteU8(dst, *c);
        }
        break;

    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, 2);      // FIXME: Report as LuxFloat
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByActiveBoxId(mac->modeId);
            sbufWriteU8(dst, box ? box->permanentId : 0);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, adjRange->adjustmentIndex);
            sbufWriteU8(dst, adjRange->auxChannelIndex);
            sbufWriteU8(dst, adjRange->range.startStep);
            sbufWriteU8(dst, adjRange->range.endStep);
            sbufWriteU8(dst, adjRange->adjustmentFunction);
            sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
        }
        break;

    case MSP_BOXNAMES:
        if (!serializeBoxNamesReply(dst)) {
            return false;
        }
        break;

    case MSP_BOXIDS:
        serializeBoxReply(dst);
        break;

    case MSP_MISC:
        sbufWriteU16(dst, PWM_RANGE_MIDDLE);

        sbufWriteU16(dst, 0); // Was min_throttle
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);

#ifdef USE_GPS
        sbufWriteU8(dst, gpsConfig()->provider); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, gpsConfig()->sbasMode); // gps_ubx_sbas
#else
        sbufWriteU8(dst, 0); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, 0); // gps_ubx_sbas
#endif
        sbufWriteU8(dst, 0); // multiwiiCurrentMeterOutput
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        sbufWriteU8(dst, 0);

#ifdef USE_MAG
        sbufWriteU16(dst, compassConfig()->mag_declination / 10);
#else
        sbufWriteU16(dst, 0);
#endif

#ifdef USE_ADC
        sbufWriteU8(dst, batteryMetersConfig()->voltage.scale / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellMin / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellMax / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellWarning / 10);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP2_INAV_MISC:
        sbufWriteU16(dst, PWM_RANGE_MIDDLE);

        sbufWriteU16(dst, 0); //Was min_throttle
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);

#ifdef USE_GPS
        sbufWriteU8(dst, gpsConfig()->provider); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, gpsConfig()->sbasMode); // gps_ubx_sbas
#else
        sbufWriteU8(dst, 0); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, 0); // gps_ubx_sbas
#endif
        sbufWriteU8(dst, rxConfig()->rssi_channel);

#ifdef USE_MAG
        sbufWriteU16(dst, compassConfig()->mag_declination / 10);
#else
        sbufWriteU16(dst, 0);
#endif

#ifdef USE_ADC
        sbufWriteU16(dst, batteryMetersConfig()->voltage.scale);
        sbufWriteU8(dst, batteryMetersConfig()->voltageSource);
        sbufWriteU8(dst, currentBatteryProfile->cells);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellDetect);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellMin);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellMax);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellWarning);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif

        sbufWriteU32(dst, currentBatteryProfile->capacity.value);
        sbufWriteU32(dst, currentBatteryProfile->capacity.warning);
        sbufWriteU32(dst, currentBatteryProfile->capacity.critical);
        sbufWriteU8(dst, currentBatteryProfile->capacity.unit);
        break;

    case MSP2_INAV_MISC2:
        // Timers
        sbufWriteU32(dst, micros() / 1000000); // On time (seconds)
        sbufWriteU32(dst, getFlightTime()); // Flight time (seconds)

        // Throttle
        sbufWriteU8(dst, getThrottlePercent()); // Throttle Percent
        sbufWriteU8(dst, navigationIsControllingThrottle() ? 1 : 0); // Auto Throttle Flag (0 or 1)

        break;

    case MSP2_INAV_BATTERY_CONFIG:
#ifdef USE_ADC
        sbufWriteU16(dst, batteryMetersConfig()->voltage.scale);
        sbufWriteU8(dst, batteryMetersConfig()->voltageSource);
        sbufWriteU8(dst, currentBatteryProfile->cells);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellDetect);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellMin);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellMax);
        sbufWriteU16(dst, currentBatteryProfile->voltage.cellWarning);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif

        sbufWriteU16(dst, batteryMetersConfig()->current.offset);
        sbufWriteU16(dst, batteryMetersConfig()->current.scale);

        sbufWriteU32(dst, currentBatteryProfile->capacity.value);
        sbufWriteU32(dst, currentBatteryProfile->capacity.warning);
        sbufWriteU32(dst, currentBatteryProfile->capacity.critical);
        sbufWriteU8(dst, currentBatteryProfile->capacity.unit);
        break;

    case MSP_MOTOR_PINS:
        // FIXME This is hardcoded and should not be.
        for (int i = 0; i < 8; i++) {
            sbufWriteU8(dst, i + 1);
        }
        break;

#ifdef USE_GPS
    case MSP_RAW_GPS:
        sbufWriteU8(dst, gpsSol.fixType);
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, gpsSol.llh.alt/100); // meters
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        sbufWriteU16(dst, gpsSol.hdop);
        break;

    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, gpsSol.flags.gpsHeartbeat ? 1 : 0);
        break;
#ifdef USE_NAV
    case MSP_NAV_STATUS:
        sbufWriteU8(dst, NAV_Status.mode);
        sbufWriteU8(dst, NAV_Status.state);
        sbufWriteU8(dst, NAV_Status.activeWpAction);
        sbufWriteU8(dst, NAV_Status.activeWpNumber);
        sbufWriteU8(dst, NAV_Status.error);
        //sbufWriteU16(dst,  (int16_t)(target_bearing/100));
        sbufWriteU16(dst, getHeadingHoldTarget());
        break;
#endif

    case MSP_GPSSVINFO:
        /* Compatibility stub - return zero SVs */
        sbufWriteU8(dst, 1);

        // HDOP
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, gpsSol.hdop / 100);
        sbufWriteU8(dst, gpsSol.hdop / 100);
        break;

    case MSP_GPSSTATISTICS:
        sbufWriteU16(dst, gpsStats.lastMessageDt);
        sbufWriteU32(dst, gpsStats.errors);
        sbufWriteU32(dst, gpsStats.timeouts);
        sbufWriteU32(dst, gpsStats.packetCount);
        sbufWriteU16(dst, gpsSol.hdop);
        sbufWriteU16(dst, gpsSol.eph);
        sbufWriteU16(dst, gpsSol.epv);
        break;
#endif
    case MSP_DEBUG:
        // output some useful QA statistics
        // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

        for (int i = 0; i < 4; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;

    case MSP2_INAV_DEBUG:
        for (int i = 0; i < DEBUG32_VALUE_COUNT; i++) {
            sbufWriteU32(dst, debug[i]);      // 8 variables are here for general monitoring purpose
        }
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

    case MSP_FEATURE:
        sbufWriteU32(dst, featureMask());
        break;

    case MSP_BOARD_ALIGNMENT:
        sbufWriteU16(dst, boardAlignment()->rollDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDeciDegrees);
        break;

    case MSP_VOLTAGE_METER_CONFIG:
#ifdef USE_ADC
        sbufWriteU8(dst, batteryMetersConfig()->voltage.scale / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellMin / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellMax / 10);
        sbufWriteU8(dst, currentBatteryProfile->voltage.cellWarning / 10);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_CURRENT_METER_CONFIG:
        sbufWriteU16(dst, batteryMetersConfig()->current.scale);
        sbufWriteU16(dst, batteryMetersConfig()->current.offset);
        sbufWriteU8(dst, batteryMetersConfig()->current.type);
        sbufWriteU16(dst, constrain(currentBatteryProfile->capacity.value, 0, 0xFFFF));
        break;

    case MSP_MIXER:
        sbufWriteU8(dst, 3); // mixerMode no longer supported, send 3 (QuadX) as fallback
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, rxConfig()->serialrx_provider);
        sbufWriteU16(dst, rxConfig()->maxcheck);
        sbufWriteU16(dst, PWM_RANGE_MIDDLE);
        sbufWriteU16(dst, rxConfig()->mincheck);
#ifdef USE_SPEKTRUM_BIND
        sbufWriteU8(dst, rxConfig()->spektrum_sat_bind);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU16(dst, rxConfig()->rx_min_usec);
        sbufWriteU16(dst, rxConfig()->rx_max_usec);
        sbufWriteU8(dst, 0); // for compatibility with betaflight (rcInterpolation)
        sbufWriteU8(dst, 0); // for compatibility with betaflight (rcInterpolationInterval)
        sbufWriteU16(dst, 0); // for compatibility with betaflight (airModeActivateThreshold)
#ifdef USE_RX_SPI
        sbufWriteU8(dst, rxConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxConfig()->rx_spi_id);
        sbufWriteU8(dst, rxConfig()->rx_spi_rf_channel_count);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, 0); // for compatibility with betaflight (fpvCamAngleDegrees)
        sbufWriteU8(dst, rxConfig()->receiverType);
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, 0);    // was failsafe_kill_switch
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        sbufWriteU8(dst, failsafeConfig()->failsafe_recovery_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_roll_angle);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_pitch_angle);
        sbufWriteU16(dst, failsafeConfig()->failsafe_fw_yaw_rate);
        sbufWriteU16(dst, failsafeConfig()->failsafe_stick_motion_threshold);
        sbufWriteU16(dst, failsafeConfig()->failsafe_min_distance);
        sbufWriteU8(dst, failsafeConfig()->failsafe_min_distance_procedure);
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, MAX_MAPPABLE_RX_INPUTS);
        break;

    case MSP_BF_CONFIG:
        sbufWriteU8(dst, 3); // mixerMode no longer supported, send 3 (QuadX) as fallback

        sbufWriteU32(dst, featureMask());

        sbufWriteU8(dst, rxConfig()->serialrx_provider);

        sbufWriteU16(dst, boardAlignment()->rollDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDeciDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDeciDegrees);

        sbufWriteU16(dst, batteryMetersConfig()->current.scale);
        sbufWriteU16(dst, batteryMetersConfig()->current.offset);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].peripheral_baudrateIndex);
        }
        break;

    case MSP2_COMMON_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU32(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].peripheral_baudrateIndex);
        }
        break;

#ifdef USE_LED_STRIP
    case MSP_LED_COLORS:
        for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
            const hsvColor_t *color = &ledStripConfig()->colors[i];
            sbufWriteU16(dst, color->h);
            sbufWriteU8(dst, color->s);
            sbufWriteU8(dst, color->v);
        }
        break;

    case MSP_LED_STRIP_CONFIG:
        for (int i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
            const ledConfig_t *ledConfig = &ledStripConfig()->ledConfigs[i];
            sbufWriteU32(dst, *ledConfig);
        }
        break;

    case MSP_LED_STRIP_MODECOLOR:
        for (int i = 0; i < LED_MODE_COUNT; i++) {
            for (int j = 0; j < LED_DIRECTION_COUNT; j++) {
                sbufWriteU8(dst, i);
                sbufWriteU8(dst, j);
                sbufWriteU8(dst, ledStripConfig()->modeColors[i].color[j]);
            }
        }

        for (int j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
            sbufWriteU8(dst, LED_MODE_COUNT);
            sbufWriteU8(dst, j);
            sbufWriteU8(dst, ledStripConfig()->specialColors.color[j]);
        }
        break;
#endif

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_BLACKBOX_CONFIG:
        sbufWriteU8(dst, 0); // API no longer supported
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        break;

    case MSP2_BLACKBOX_CONFIG:
#ifdef USE_BLACKBOX
        sbufWriteU8(dst, 1); //Blackbox supported
        sbufWriteU8(dst, blackboxConfig()->device);
        sbufWriteU16(dst, blackboxConfig()->rate_num);
        sbufWriteU16(dst, blackboxConfig()->rate_denom);
#else
        sbufWriteU8(dst, 0); // Blackbox not supported
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_OSD_CONFIG:
#ifdef USE_OSD
        sbufWriteU8(dst, OSD_DRIVER_MAX7456); // OSD supported
        // send video system (AUTO/PAL/NTSC)
        sbufWriteU8(dst, osdConfig()->video_system);
        sbufWriteU8(dst, osdConfig()->units);
        sbufWriteU8(dst, osdConfig()->rssi_alarm);
        sbufWriteU16(dst, currentBatteryProfile->capacity.warning);
        sbufWriteU16(dst, osdConfig()->time_alarm);
        sbufWriteU16(dst, osdConfig()->alt_alarm);
        sbufWriteU16(dst, osdConfig()->dist_alarm);
        sbufWriteU16(dst, osdConfig()->neg_alt_alarm);
        for (int i = 0; i < OSD_ITEM_COUNT; i++) {
            sbufWriteU16(dst, osdLayoutsConfig()->item_pos[0][i]);
        }
#else
        sbufWriteU8(dst, OSD_DRIVER_NONE); // OSD not supported
#endif
        break;

    case MSP_BF_BUILD_INFO:
        sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        sbufWriteU32(dst, 0); // future exp
        sbufWriteU32(dst, 0); // future exp
        break;

    case MSP_3D:
        sbufWriteU16(dst, reversibleMotorsConfig()->deadband_low);
        sbufWriteU16(dst, reversibleMotorsConfig()->deadband_high);
        sbufWriteU16(dst, reversibleMotorsConfig()->neutral);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, rcControlsConfig()->deadband);
        sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
        sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
        sbufWriteU16(dst, rcControlsConfig()->mid_throttle_deadband);
        break;

    case MSP_SENSOR_ALIGNMENT:
        sbufWriteU8(dst, gyroConfig()->gyro_align);
        sbufWriteU8(dst, accelerometerConfig()->acc_align);
#ifdef USE_MAG
        sbufWriteU8(dst, compassConfig()->mag_align);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_OPFLOW
        sbufWriteU8(dst, opticalFlowConfig()->opflow_align);
#else
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_ADVANCED_CONFIG:
        sbufWriteU8(dst, 1);    // gyroConfig()->gyroSyncDenominator
        sbufWriteU8(dst, 1);    // BF: masterConfig.pid_process_denom
        sbufWriteU8(dst, 1);    // BF: motorConfig()->useUnsyncedPwm
        sbufWriteU8(dst, motorConfig()->motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->motorPwmRate);
        sbufWriteU16(dst, servoConfig()->servoPwmRate);
        sbufWriteU8(dst, 0);
        break;

    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, gyroConfig()->gyro_main_lpf_hz);
        sbufWriteU16(dst, pidProfile()->dterm_lpf_hz);
        sbufWriteU16(dst, pidProfile()->yaw_lpf_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_notch_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_notch_cutoff);
        sbufWriteU16(dst, 0); //BF: pidProfile()->dterm_notch_hz
        sbufWriteU16(dst, 1); //pidProfile()->dterm_notch_cutoff

        sbufWriteU16(dst, 0); //BF: masterConfig.gyro_soft_notch_hz_2
        sbufWriteU16(dst, 1); //BF: masterConfig.gyro_soft_notch_cutoff_2

        sbufWriteU16(dst, accelerometerConfig()->acc_notch_hz);
        sbufWriteU16(dst, accelerometerConfig()->acc_notch_cutoff);

        sbufWriteU16(dst, 0);    //Was gyroConfig()->gyro_stage2_lowpass_hz
        break;

    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, 0); // pidProfile()->rollPitchItermIgnoreRate
        sbufWriteU16(dst, 0); // pidProfile()->yawItermIgnoreRate
        sbufWriteU16(dst, 0); //pidProfile()->yaw_p_limit
        sbufWriteU8(dst, 0); //BF: pidProfile()->deltaMethod
        sbufWriteU8(dst, 0); //BF: pidProfile()->vbatPidCompensation
        sbufWriteU8(dst, 0); //BF: pidProfile()->setpointRelaxRatio
        sbufWriteU8(dst, 0);
        sbufWriteU16(dst, pidProfile()->pidSumLimit);
        sbufWriteU8(dst, 0); //BF: pidProfile()->itermThrottleGain

        /*
         * To keep compatibility on MSP frame length level with Betaflight, axis axisAccelerationLimitYaw
         * limit will be sent and received in [dps / 10]
         */
        sbufWriteU16(dst, constrain(pidProfile()->axisAccelerationLimitRollPitch / 10, 0, 65535));
        sbufWriteU16(dst, constrain(pidProfile()->axisAccelerationLimitYaw / 10, 0, 65535));
        break;

    case MSP_INAV_PID:
        sbufWriteU8(dst, 0); //Legacy, no longer in use async processing value
        sbufWriteU16(dst, 0); //Legacy, no longer in use async processing value
        sbufWriteU16(dst, 0); //Legacy, no longer in use async processing value
        sbufWriteU8(dst, pidProfile()->heading_hold_rate_limit);
        sbufWriteU8(dst, HEADING_HOLD_ERROR_LPF_FREQ);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, gyroConfig()->gyro_lpf);
        sbufWriteU8(dst, accelerometerConfig()->acc_lpf_hz);
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        sbufWriteU8(dst, 0); //reserved
        break;

    case MSP_SENSOR_CONFIG:
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
#ifdef USE_BARO
        sbufWriteU8(dst, barometerConfig()->baro_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_MAG
        sbufWriteU8(dst, compassConfig()->mag_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_PITOT
        sbufWriteU8(dst, pitotmeterConfig()->pitot_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_RANGEFINDER
        sbufWriteU8(dst, rangefinderConfig()->rangefinder_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_OPFLOW
        sbufWriteU8(dst, opticalFlowConfig()->opflow_hardware);
#else
        sbufWriteU8(dst, 0);
#endif
        break;

#ifdef USE_NAV
    case MSP_NAV_POSHOLD:
        sbufWriteU8(dst, navConfig()->general.flags.user_control_mode);
        sbufWriteU16(dst, navConfig()->general.max_auto_speed);
        sbufWriteU16(dst, navConfig()->general.max_auto_climb_rate);
        sbufWriteU16(dst, navConfig()->general.max_manual_speed);
        sbufWriteU16(dst, navConfig()->general.max_manual_climb_rate);
        sbufWriteU8(dst, navConfig()->mc.max_bank_angle);
        sbufWriteU8(dst, navConfig()->general.flags.use_thr_mid_for_althold);
        sbufWriteU16(dst, navConfig()->mc.hover_throttle);
        break;

    case MSP_RTH_AND_LAND_CONFIG:
        sbufWriteU16(dst, navConfig()->general.min_rth_distance);
        sbufWriteU8(dst, navConfig()->general.flags.rth_climb_first);
        sbufWriteU8(dst, navConfig()->general.flags.rth_climb_ignore_emerg);
        sbufWriteU8(dst, navConfig()->general.flags.rth_tail_first);
        sbufWriteU8(dst, navConfig()->general.flags.rth_allow_landing);
        sbufWriteU8(dst, navConfig()->general.flags.rth_alt_control_mode);
        sbufWriteU16(dst, navConfig()->general.rth_abort_threshold);
        sbufWriteU16(dst, navConfig()->general.rth_altitude);
        sbufWriteU16(dst, navConfig()->general.land_minalt_vspd);
        sbufWriteU16(dst, navConfig()->general.land_maxalt_vspd);
        sbufWriteU16(dst, navConfig()->general.land_slowdown_minalt);
        sbufWriteU16(dst, navConfig()->general.land_slowdown_maxalt);
        sbufWriteU16(dst, navConfig()->general.emerg_descent_rate);
        break;

    case MSP_FW_CONFIG:
        sbufWriteU16(dst, navConfig()->fw.cruise_throttle);
        sbufWriteU16(dst, navConfig()->fw.min_throttle);
        sbufWriteU16(dst, navConfig()->fw.max_throttle);
        sbufWriteU8(dst, navConfig()->fw.max_bank_angle);
        sbufWriteU8(dst, navConfig()->fw.max_climb_angle);
        sbufWriteU8(dst, navConfig()->fw.max_dive_angle);
        sbufWriteU8(dst, navConfig()->fw.pitch_to_throttle);
        sbufWriteU16(dst, navConfig()->fw.loiter_radius);
        break;
#endif

    case MSP_CALIBRATION_DATA:
        sbufWriteU8(dst, accGetCalibrationAxisFlags());
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[X]);
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[Y]);
        sbufWriteU16(dst, accelerometerConfig()->accZero.raw[Z]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[X]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[Y]);
        sbufWriteU16(dst, accelerometerConfig()->accGain.raw[Z]);

    #ifdef USE_MAG
        sbufWriteU16(dst, compassConfig()->magZero.raw[X]);
        sbufWriteU16(dst, compassConfig()->magZero.raw[Y]);
        sbufWriteU16(dst, compassConfig()->magZero.raw[Z]);
    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    #endif

    #ifdef USE_OPFLOW
        sbufWriteU16(dst, opticalFlowConfig()->opflow_scale * 256);
    #else
        sbufWriteU16(dst, 0);
    #endif

    #ifdef USE_MAG
        sbufWriteU16(dst, compassConfig()->magGain[X]);
        sbufWriteU16(dst, compassConfig()->magGain[Y]);
        sbufWriteU16(dst, compassConfig()->magGain[Z]);
    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
    #endif

        break;

    case MSP_POSITION_ESTIMATION_CONFIG:
    #ifdef USE_NAV

        sbufWriteU16(dst, positionEstimationConfig()->w_z_baro_p * 100); //     inav_w_z_baro_p float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_z_gps_p * 100);  // 2   inav_w_z_gps_p  float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_z_gps_v * 100);  // 2   inav_w_z_gps_v  float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_xy_gps_p * 100); // 2   inav_w_xy_gps_p float as value * 100
        sbufWriteU16(dst, positionEstimationConfig()->w_xy_gps_v * 100); // 2   inav_w_xy_gps_v float as value * 100
        sbufWriteU8(dst, gpsConfigMutable()->gpsMinSats);                // 1
        sbufWriteU8(dst, positionEstimationConfig()->use_gps_velned);    // 1   inav_use_gps_velned ON/OFF

    #else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
    #endif
        break;

    case MSP_REBOOT:
        if (!ARMING_FLAG(ARMED)) {
            if (mspPostProcessFn) {
                *mspPostProcessFn = mspRebootFn;
            }
        }
        break;

    case MSP_WP_GETINFO:
#ifdef USE_NAV
        sbufWriteU8(dst, 0);                        // Reserved for waypoint capabilities
        sbufWriteU8(dst, NAV_MAX_WAYPOINTS);        // Maximum number of waypoints supported
        sbufWriteU8(dst, isWaypointListValid());    // Is current mission valid
        sbufWriteU8(dst, getWaypointCount());       // Number of waypoints in current mission
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif
        break;

    case MSP_TX_INFO:
        sbufWriteU8(dst, getRSSISource());
        uint8_t rtcDateTimeIsSet = 0;
        dateTime_t dt;
        if (rtcGetDateTime(&dt)) {
            rtcDateTimeIsSet = 1;
        }
        sbufWriteU8(dst, rtcDateTimeIsSet);
        break;

    case MSP_RTC:
        {
            int32_t seconds = 0;
            uint16_t millis = 0;
            rtcTime_t t;
            if (rtcGet(&t)) {
                seconds = rtcTimeGetSeconds(&t);
                millis = rtcTimeGetMillis(&t);
            }
            sbufWriteU32(dst, (uint32_t)seconds);
            sbufWriteU16(dst, millis);
        }
        break;

    case MSP_VTX_CONFIG:
        {
            vtxDevice_t *vtxDevice = vtxCommonDevice();
            if (vtxDevice) {

                uint8_t deviceType = vtxCommonGetDeviceType(vtxDevice);

                // Return band, channel and power from vtxSettingsConfig_t
                // since the VTX might be configured but temporarily offline.
                uint8_t pitmode = 0;
                vtxCommonGetPitMode(vtxDevice, &pitmode);

                sbufWriteU8(dst, deviceType);
                sbufWriteU8(dst, vtxSettingsConfig()->band);
                sbufWriteU8(dst, vtxSettingsConfig()->channel);
                sbufWriteU8(dst, vtxSettingsConfig()->power);
                sbufWriteU8(dst, pitmode);

                // Betaflight < 4 doesn't send these fields
                sbufWriteU8(dst, vtxCommonDeviceIsReady(vtxDevice) ? 1 : 0);
                sbufWriteU8(dst, vtxSettingsConfig()->lowPowerDisarm);
                // future extensions here...
            }
            else {
                sbufWriteU8(dst, VTXDEV_UNKNOWN); // no VTX configured
            }
        }
        break;

    case MSP_NAME:
        {
            const char *name = systemConfig()->name;
            while (*name) {
                sbufWriteU8(dst, *name++);
            }
        }
        break;

    case MSP2_COMMON_TZ:
        sbufWriteU16(dst, (uint16_t)timeConfig()->tz_offset);
        sbufWriteU8(dst, (uint8_t)timeConfig()->tz_automatic_dst);
        break;

    case MSP2_INAV_AIR_SPEED:
#ifdef USE_PITOT
        sbufWriteU32(dst, pitot.airSpeed);
#else
        sbufWriteU32(dst, 0);
#endif
        break;

    case MSP2_INAV_MIXER:
        sbufWriteU8(dst, mixerConfig()->motorDirectionInverted);
        sbufWriteU16(dst, 0);
        sbufWriteU8(dst, mixerConfig()->platformType);
        sbufWriteU8(dst, mixerConfig()->hasFlaps);
        sbufWriteU16(dst, mixerConfig()->appliedMixerPreset);
        sbufWriteU8(dst, MAX_SUPPORTED_MOTORS);
        sbufWriteU8(dst, MAX_SUPPORTED_SERVOS);
        break;

#if defined(USE_OSD)
    case MSP2_INAV_OSD_ALARMS:
        sbufWriteU8(dst, osdConfig()->rssi_alarm);
        sbufWriteU16(dst, osdConfig()->time_alarm);
        sbufWriteU16(dst, osdConfig()->alt_alarm);
        sbufWriteU16(dst, osdConfig()->dist_alarm);
        sbufWriteU16(dst, osdConfig()->neg_alt_alarm);
        sbufWriteU16(dst, osdConfig()->gforce_alarm * 1000);
        sbufWriteU16(dst, (int16_t)(osdConfig()->gforce_axis_alarm_min * 1000));
        sbufWriteU16(dst, (int16_t)(osdConfig()->gforce_axis_alarm_max * 1000));
        sbufWriteU8(dst, osdConfig()->current_alarm);
        sbufWriteU16(dst, osdConfig()->imu_temp_alarm_min);
        sbufWriteU16(dst, osdConfig()->imu_temp_alarm_max);
#ifdef USE_BARO
        sbufWriteU16(dst, osdConfig()->baro_temp_alarm_min);
        sbufWriteU16(dst, osdConfig()->baro_temp_alarm_max);
#else
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP2_INAV_OSD_PREFERENCES:
        sbufWriteU8(dst, osdConfig()->video_system);
        sbufWriteU8(dst, osdConfig()->main_voltage_decimals);
        sbufWriteU8(dst, osdConfig()->ahi_reverse_roll);
        sbufWriteU8(dst, osdConfig()->crosshairs_style);
        sbufWriteU8(dst, osdConfig()->left_sidebar_scroll);
        sbufWriteU8(dst, osdConfig()->right_sidebar_scroll);
        sbufWriteU8(dst, osdConfig()->sidebar_scroll_arrows);
        sbufWriteU8(dst, osdConfig()->units);
        sbufWriteU8(dst, osdConfig()->stats_energy_unit);
        break;

#endif

    case MSP2_INAV_OUTPUT_MAPPING:
        for (uint8_t i = 0; i < timerHardwareCount; ++i)
            if (!(timerHardware[i].usageFlags & (TIM_USE_PPM | TIM_USE_PWM)))
                sbufWriteU8(dst, timerHardware[i].usageFlags);
        break;

    case MSP2_INAV_MC_BRAKING:
#ifdef USE_MR_BRAKING_MODE
        sbufWriteU16(dst, navConfig()->mc.braking_speed_threshold);
        sbufWriteU16(dst, navConfig()->mc.braking_disengage_speed);
        sbufWriteU16(dst, navConfig()->mc.braking_timeout);
        sbufWriteU8(dst, navConfig()->mc.braking_boost_factor);
        sbufWriteU16(dst, navConfig()->mc.braking_boost_timeout);
        sbufWriteU16(dst, navConfig()->mc.braking_boost_speed_threshold);
        sbufWriteU16(dst, navConfig()->mc.braking_boost_disengage_speed);
        sbufWriteU8(dst, navConfig()->mc.braking_bank_angle);
#endif
        break;

#ifdef USE_TEMPERATURE_SENSOR
    case MSP2_INAV_TEMP_SENSOR_CONFIG:
        for (uint8_t index = 0; index < MAX_TEMP_SENSORS; ++index) {
            const tempSensorConfig_t *sensorConfig = tempSensorConfig(index);
            sbufWriteU8(dst, sensorConfig->type);
            for (uint8_t addrIndex = 0; addrIndex < 8; ++addrIndex)
                sbufWriteU8(dst, ((uint8_t *)&sensorConfig->address)[addrIndex]);
            sbufWriteU16(dst, sensorConfig->alarm_min);
            sbufWriteU16(dst, sensorConfig->alarm_max);
            sbufWriteU8(dst, sensorConfig->osdSymbol);
            for (uint8_t labelIndex = 0; labelIndex < TEMPERATURE_LABEL_LEN; ++labelIndex)
                sbufWriteU8(dst, sensorConfig->label[labelIndex]);
        }
        break;
#endif

#ifdef USE_TEMPERATURE_SENSOR
    case MSP2_INAV_TEMPERATURES:
        for (uint8_t index = 0; index < MAX_TEMP_SENSORS; ++index) {
            int16_t temperature;
            bool valid = getSensorTemperature(index, &temperature);
            sbufWriteU16(dst, valid ? temperature : -1000);
        }
        break;
#endif

    default:
        return false;
    }
    return true;
}

static mspResult_e mspFcSafeHomeOutCommand(sbuf_t *dst, sbuf_t *src)
{
    const uint8_t safe_home_no = sbufReadU8(src);    // get the home number
    if(safe_home_no < MAX_SAFE_HOMES) {
        sbufWriteU8(dst, safe_home_no);
        sbufWriteU8(dst, safeHomeConfig(safe_home_no)->enabled);
        sbufWriteU32(dst, safeHomeConfig(safe_home_no)->lat);
        sbufWriteU32(dst, safeHomeConfig(safe_home_no)->lon);
        return MSP_RESULT_ACK;
    } else {
         return MSP_RESULT_ERROR;
    }
}

#ifdef USE_NAV
static void mspFcWaypointOutCommand(sbuf_t *dst, sbuf_t *src)
{
    const uint8_t msp_wp_no = sbufReadU8(src);    // get the wp number
    navWaypoint_t msp_wp;
    getWaypoint(msp_wp_no, &msp_wp);
    sbufWriteU8(dst, msp_wp_no);   // wp_no
    sbufWriteU8(dst, msp_wp.action);  // action (WAYPOINT)
    sbufWriteU32(dst, msp_wp.lat);    // lat
    sbufWriteU32(dst, msp_wp.lon);    // lon
    sbufWriteU32(dst, msp_wp.alt);    // altitude (cm)
    sbufWriteU16(dst, msp_wp.p1);     // P1
    sbufWriteU16(dst, msp_wp.p2);     // P2
    sbufWriteU16(dst, msp_wp.p3);     // P3
    sbufWriteU8(dst, msp_wp.flag);    // flags
}
#endif

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    uint16_t readLength;

    const uint32_t readAddress = sbufReadU32(src);

    // Request payload:
    //  uint32_t    - address to read from
    //  uint16_t    - size of block to read (optional)
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
    }
    else {
        readLength = 128;
    }

    serializeDataflashReadReply(dst, readAddress, readLength);
}
#endif

static mspResult_e mspFcProcessInCommand(uint16_t cmdMSP, sbuf_t *src)
{
    uint8_t tmp_u8;
    uint16_t tmp_u16;
    batteryProfile_t *currentBatteryProfileMutable = (batteryProfile_t*)currentBatteryProfile;

    const unsigned int dataSize = sbufBytesRemaining(src);

    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        if (sbufReadU8Safe(&tmp_u8, src) && (!ARMING_FLAG(ARMED)))
            setConfigProfileAndWriteEEPROM(tmp_u8);
        else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_HEAD:
        if (sbufReadU16Safe(&tmp_u16, src))
            updateHeadingHoldTarget(tmp_u16);
        else
            return MSP_RESULT_ERROR;
        break;

#ifdef USE_RX_MSP
    case MSP_SET_RAW_RC:
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if ((channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) || (dataSize > channelCount * sizeof(uint16_t))) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
        break;
#endif

    case MSP_SET_ARMING_CONFIG:
        if (dataSize >= 2) {
            sbufReadU8(src); //Swallow the first byte, used to be auto_disarm_delay
            armingConfigMutable()->disarm_kill_switch = !!sbufReadU8(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_LOOP_TIME:
        if (sbufReadU16Safe(&tmp_u16, src))
            gyroConfigMutable()->looptime = tmp_u16;
        else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_PID_CONTROLLER:
        // FIXME: Do nothing
        break;

    case MSP_SET_PID:
        if (dataSize >= PID_ITEM_COUNT * 3) {
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                pidBankMutable()->pid[i].P = sbufReadU8(src);
                pidBankMutable()->pid[i].I = sbufReadU8(src);
                pidBankMutable()->pid[i].D = sbufReadU8(src);
            }
            schedulePidGainsUpdate();
#if defined(USE_NAV)
            navigationUsePIDs();
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_SET_PID:
        if (dataSize >= PID_ITEM_COUNT * 4) {
            for (int i = 0; i < PID_ITEM_COUNT; i++) {
                pidBankMutable()->pid[i].P = sbufReadU8(src);
                pidBankMutable()->pid[i].I = sbufReadU8(src);
                pidBankMutable()->pid[i].D = sbufReadU8(src);
                pidBankMutable()->pid[i].FF = sbufReadU8(src);
            }
            schedulePidGainsUpdate();
#if defined(USE_NAV)
            navigationUsePIDs();
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_MODE_RANGE:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize >= 5) && (tmp_u8 < MAX_MODE_ACTIVATION_CONDITION_COUNT)) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(tmp_u8);
            tmp_u8 = sbufReadU8(src);
            const box_t *box = findBoxByPermanentId(tmp_u8);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);

                updateUsedModeActivationConditionFlags();
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize >= 7) && (tmp_u8 < MAX_ADJUSTMENT_RANGE_COUNT)) {
            adjustmentRange_t *adjRange = adjustmentRangesMutable(tmp_u8);
            tmp_u8 = sbufReadU8(src);
            if (tmp_u8 < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                adjRange->adjustmentIndex = tmp_u8;
                adjRange->auxChannelIndex = sbufReadU8(src);
                adjRange->range.startStep = sbufReadU8(src);
                adjRange->range.endStep = sbufReadU8(src);
                adjRange->adjustmentFunction = sbufReadU8(src);
                adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        if ((dataSize >= 10) && (dataSize <= 11)) {
            sbufReadU8(src); //Read rcRate8, kept for protocol compatibility reasons
            // need to cast away const to set controlRateProfile
            ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rcExpo8 = sbufReadU8(src);
            for (int i = 0; i < 3; i++) {
                tmp_u8 = sbufReadU8(src);
                if (i == FD_YAW) {
                    ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rates[i] = constrain(tmp_u8, SETTING_YAW_RATE_MIN, SETTING_YAW_RATE_MAX);
                }
                else {
                    ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rates[i] = constrain(tmp_u8, SETTING_CONSTANT_ROLL_PITCH_RATE_MIN, SETTING_CONSTANT_ROLL_PITCH_RATE_MAX);
                }
            }
            tmp_u8 = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->throttle.dynPID = MIN(tmp_u8, SETTING_TPA_RATE_MAX);
            ((controlRateConfig_t*)currentControlRateProfile)->throttle.rcMid8 = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->throttle.rcExpo8 = sbufReadU8(src);
            ((controlRateConfig_t*)currentControlRateProfile)->throttle.pa_breakpoint = sbufReadU16(src);
            if (dataSize > 10) {
                ((controlRateConfig_t*)currentControlRateProfile)->stabilized.rcYawExpo8 = sbufReadU8(src);
            }

            schedulePidGainsUpdate();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP2_INAV_SET_RATE_PROFILE:
        if (dataSize == 15) {
            controlRateConfig_t *currentControlRateProfile_p = (controlRateConfig_t*)currentControlRateProfile; // need to cast away const to set controlRateProfile

            // throttle
            currentControlRateProfile_p->throttle.rcMid8 = sbufReadU8(src);
            currentControlRateProfile_p->throttle.rcExpo8 = sbufReadU8(src);
            currentControlRateProfile_p->throttle.dynPID = sbufReadU8(src);
            currentControlRateProfile_p->throttle.pa_breakpoint = sbufReadU16(src);

            // stabilized
            currentControlRateProfile_p->stabilized.rcExpo8 = sbufReadU8(src);
            currentControlRateProfile_p->stabilized.rcYawExpo8 = sbufReadU8(src);
            for (uint8_t i = 0; i < 3; ++i) {
                tmp_u8 = sbufReadU8(src);
                if (i == FD_YAW) {
                    currentControlRateProfile_p->stabilized.rates[i] = constrain(tmp_u8, SETTING_YAW_RATE_MIN, SETTING_YAW_RATE_MAX);
                } else {
                    currentControlRateProfile_p->stabilized.rates[i] = constrain(tmp_u8, SETTING_CONSTANT_ROLL_PITCH_RATE_MIN, SETTING_CONSTANT_ROLL_PITCH_RATE_MAX);
                }
            }

            // manual
            currentControlRateProfile_p->manual.rcExpo8 = sbufReadU8(src);
            currentControlRateProfile_p->manual.rcYawExpo8 = sbufReadU8(src);
            for (uint8_t i = 0; i < 3; ++i) {
                tmp_u8 = sbufReadU8(src);
                if (i == FD_YAW) {
                    currentControlRateProfile_p->manual.rates[i] = constrain(tmp_u8, SETTING_YAW_RATE_MIN, SETTING_YAW_RATE_MAX);
                } else {
                    currentControlRateProfile_p->manual.rates[i] = constrain(tmp_u8, SETTING_CONSTANT_ROLL_PITCH_RATE_MIN, SETTING_CONSTANT_ROLL_PITCH_RATE_MAX);
                }
            }

        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_MISC:
        if (dataSize >= 22) {
        sbufReadU16(src);   // midrc

        sbufReadU16(src); //Was min_throttle
        motorConfigMutable()->maxthrottle = constrain(sbufReadU16(src), PWM_RANGE_MIN, PWM_RANGE_MAX);
        motorConfigMutable()->mincommand = constrain(sbufReadU16(src), 0, PWM_RANGE_MAX);

        failsafeConfigMutable()->failsafe_throttle = constrain(sbufReadU16(src), PWM_RANGE_MIN, PWM_RANGE_MAX);

#ifdef USE_GPS
        gpsConfigMutable()->provider = sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        gpsConfigMutable()->sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
        sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        sbufReadU8(src); // gps_ubx_sbas
#endif
        sbufReadU8(src); // multiwiiCurrentMeterOutput
        tmp_u8 = sbufReadU8(src);
        if (tmp_u8 <= MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rxConfigMutable()->rssi_channel = tmp_u8;
            rxUpdateRSSISource(); // Changing rssi_channel might change the RSSI source
        }
        sbufReadU8(src);

#ifdef USE_MAG
        compassConfigMutable()->mag_declination = sbufReadU16(src) * 10;
#else
        sbufReadU16(src);
#endif

#ifdef USE_ADC
        batteryMetersConfigMutable()->voltage.scale = sbufReadU8(src) * 10;
        currentBatteryProfileMutable->voltage.cellMin = sbufReadU8(src) * 10;         // vbatlevel_warn1 in MWC2.3 GUI
        currentBatteryProfileMutable->voltage.cellMax = sbufReadU8(src) * 10;         // vbatlevel_warn2 in MWC2.3 GUI
        currentBatteryProfileMutable->voltage.cellWarning = sbufReadU8(src) * 10;     // vbatlevel when buzzer starts to alert
#else
        sbufReadU8(src);
        sbufReadU8(src);
        sbufReadU8(src);
        sbufReadU8(src);
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SET_MISC:
        if (dataSize == 41) {
            sbufReadU16(src);       // midrc

            sbufReadU16(src);   // was min_throttle
            motorConfigMutable()->maxthrottle = constrain(sbufReadU16(src), PWM_RANGE_MIN, PWM_RANGE_MAX);
            motorConfigMutable()->mincommand = constrain(sbufReadU16(src), 0, PWM_RANGE_MAX);

            failsafeConfigMutable()->failsafe_throttle = constrain(sbufReadU16(src), PWM_RANGE_MIN, PWM_RANGE_MAX);

#ifdef USE_GPS
            gpsConfigMutable()->provider = sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            gpsConfigMutable()->sbasMode = sbufReadU8(src); // gps_ubx_sbas
#else
            sbufReadU8(src); // gps_type
            sbufReadU8(src); // gps_baudrate
            sbufReadU8(src); // gps_ubx_sbas
#endif

            tmp_u8 = sbufReadU8(src);
            if (tmp_u8 <= MAX_SUPPORTED_RC_CHANNEL_COUNT)
                rxConfigMutable()->rssi_channel = tmp_u8;

#ifdef USE_MAG
            compassConfigMutable()->mag_declination = sbufReadU16(src) * 10;
#else
            sbufReadU16(src);
#endif

#ifdef USE_ADC
            batteryMetersConfigMutable()->voltage.scale = sbufReadU16(src);
            batteryMetersConfigMutable()->voltageSource = sbufReadU8(src);
            currentBatteryProfileMutable->cells = sbufReadU8(src);
            currentBatteryProfileMutable->voltage.cellDetect = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellMin = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellMax = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellWarning = sbufReadU16(src);
#else
            sbufReadU16(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif

            currentBatteryProfileMutable->capacity.value = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.warning = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.critical = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.unit = sbufReadU8(src);
            if ((batteryMetersConfig()->voltageSource != BAT_VOLTAGE_RAW) && (batteryMetersConfig()->voltageSource != BAT_VOLTAGE_SAG_COMP)) {
                batteryMetersConfigMutable()->voltageSource = BAT_VOLTAGE_RAW;
                return MSP_RESULT_ERROR;
            }
            if ((currentBatteryProfile->capacity.unit != BAT_CAPACITY_UNIT_MAH) && (currentBatteryProfile->capacity.unit != BAT_CAPACITY_UNIT_MWH)) {
                currentBatteryProfileMutable->capacity.unit = BAT_CAPACITY_UNIT_MAH;
                return MSP_RESULT_ERROR;
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SET_BATTERY_CONFIG:
        if (dataSize == 29) {
#ifdef USE_ADC
            batteryMetersConfigMutable()->voltage.scale = sbufReadU16(src);
            batteryMetersConfigMutable()->voltageSource = sbufReadU8(src);
            currentBatteryProfileMutable->cells = sbufReadU8(src);
            currentBatteryProfileMutable->voltage.cellDetect = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellMin = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellMax = sbufReadU16(src);
            currentBatteryProfileMutable->voltage.cellWarning = sbufReadU16(src);
#else
            sbufReadU16(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif

            batteryMetersConfigMutable()->current.offset = sbufReadU16(src);
            batteryMetersConfigMutable()->current.scale = sbufReadU16(src);

            currentBatteryProfileMutable->capacity.value = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.warning = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.critical = sbufReadU32(src);
            currentBatteryProfileMutable->capacity.unit = sbufReadU8(src);
            if ((batteryMetersConfig()->voltageSource != BAT_VOLTAGE_RAW) && (batteryMetersConfig()->voltageSource != BAT_VOLTAGE_SAG_COMP)) {
                batteryMetersConfigMutable()->voltageSource = BAT_VOLTAGE_RAW;
                return MSP_RESULT_ERROR;
            }
            if ((currentBatteryProfile->capacity.unit != BAT_CAPACITY_UNIT_MAH) && (currentBatteryProfile->capacity.unit != BAT_CAPACITY_UNIT_MWH)) {
                currentBatteryProfileMutable->capacity.unit = BAT_CAPACITY_UNIT_MAH;
                return MSP_RESULT_ERROR;
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_MOTOR:
        if (dataSize >= 8 * sizeof(uint16_t)) {
            for (int i = 0; i < 8; i++) {
                const int16_t disarmed = sbufReadU16(src);
                if (i < MAX_SUPPORTED_MOTORS) {
                    motor_disarmed[i] = disarmed;
                }
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_SERVO_CONFIGURATION:
        if (dataSize != (1 + 14)) {
            return MSP_RESULT_ERROR;
        }
        tmp_u8 = sbufReadU8(src);
        if (tmp_u8 >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        } else {
            servoParamsMutable(tmp_u8)->min = sbufReadU16(src);
            servoParamsMutable(tmp_u8)->max = sbufReadU16(src);
            servoParamsMutable(tmp_u8)->middle = sbufReadU16(src);
            servoParamsMutable(tmp_u8)->rate = sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src); // used to be forwardFromChannel, ignored
            sbufReadU32(src); // used to be reversedSources
            servoComputeScalingFactors(tmp_u8);
        }
        break;

    case MSP_SET_SERVO_MIX_RULE:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize >= 9) && (tmp_u8 < MAX_SERVO_RULES)) {
            customServoMixersMutable(tmp_u8)->targetChannel = sbufReadU8(src);
            customServoMixersMutable(tmp_u8)->inputSource = sbufReadU8(src);
            customServoMixersMutable(tmp_u8)->rate = sbufReadU16(src);
            customServoMixersMutable(tmp_u8)->speed = sbufReadU8(src);
            sbufReadU16(src); //Read 2bytes for min/max and ignore it
            sbufReadU8(src); //Read 1 byte for `box` and ignore it
            loadCustomServoMixer();
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SET_SERVO_MIXER:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize == 7) && (tmp_u8 < MAX_SERVO_RULES)) {
            customServoMixersMutable(tmp_u8)->targetChannel = sbufReadU8(src);
            customServoMixersMutable(tmp_u8)->inputSource = sbufReadU8(src);
            customServoMixersMutable(tmp_u8)->rate = sbufReadU16(src);
            customServoMixersMutable(tmp_u8)->speed = sbufReadU8(src);
        #ifdef USE_PROGRAMMING_FRAMEWORK
            customServoMixersMutable(tmp_u8)->conditionId = sbufReadU8(src);
        #else
            sbufReadU8(src);
        #endif
            loadCustomServoMixer();
        } else
            return MSP_RESULT_ERROR;
        break;
#ifdef USE_PROGRAMMING_FRAMEWORK
    case MSP2_INAV_SET_LOGIC_CONDITIONS:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize == 15) && (tmp_u8 < MAX_LOGIC_CONDITIONS)) {
            logicConditionsMutable(tmp_u8)->enabled = sbufReadU8(src);
            logicConditionsMutable(tmp_u8)->activatorId = sbufReadU8(src);
            logicConditionsMutable(tmp_u8)->operation = sbufReadU8(src);
            logicConditionsMutable(tmp_u8)->operandA.type = sbufReadU8(src);
            logicConditionsMutable(tmp_u8)->operandA.value = sbufReadU32(src);
            logicConditionsMutable(tmp_u8)->operandB.type = sbufReadU8(src);
            logicConditionsMutable(tmp_u8)->operandB.value = sbufReadU32(src);
            logicConditionsMutable(tmp_u8)->flags = sbufReadU8(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SET_PROGRAMMING_PID:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize == 20) && (tmp_u8 < MAX_PROGRAMMING_PID_COUNT)) {
            programmingPidsMutable(tmp_u8)->enabled = sbufReadU8(src);
            programmingPidsMutable(tmp_u8)->setpoint.type = sbufReadU8(src);
            programmingPidsMutable(tmp_u8)->setpoint.value = sbufReadU32(src);
            programmingPidsMutable(tmp_u8)->measurement.type = sbufReadU8(src);
            programmingPidsMutable(tmp_u8)->measurement.value = sbufReadU32(src);
            programmingPidsMutable(tmp_u8)->gains.P = sbufReadU16(src);
            programmingPidsMutable(tmp_u8)->gains.I = sbufReadU16(src);
            programmingPidsMutable(tmp_u8)->gains.D = sbufReadU16(src);
            programmingPidsMutable(tmp_u8)->gains.FF = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;
#endif
    case MSP2_COMMON_SET_MOTOR_MIXER:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize == 9) && (tmp_u8 < MAX_SUPPORTED_MOTORS)) {
            primaryMotorMixerMutable(tmp_u8)->throttle = constrainf(sbufReadU16(src) / 1000.0f, 0.0f, 1.0f);
            primaryMotorMixerMutable(tmp_u8)->roll = constrainf(sbufReadU16(src) / 1000.0f, 0.0f, 4.0f) - 2.0f;
            primaryMotorMixerMutable(tmp_u8)->pitch = constrainf(sbufReadU16(src) / 1000.0f, 0.0f, 4.0f) - 2.0f;
            primaryMotorMixerMutable(tmp_u8)->yaw = constrainf(sbufReadU16(src) / 1000.0f, 0.0f, 4.0f) - 2.0f;
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_3D:
        if (dataSize >= 6) {
            reversibleMotorsConfigMutable()->deadband_low = sbufReadU16(src);
            reversibleMotorsConfigMutable()->deadband_high = sbufReadU16(src);
            reversibleMotorsConfigMutable()->neutral = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_RC_DEADBAND:
        if (dataSize >= 5) {
            rcControlsConfigMutable()->deadband = sbufReadU8(src);
            rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
            rcControlsConfigMutable()->alt_hold_deadband = sbufReadU8(src);
            rcControlsConfigMutable()->mid_throttle_deadband = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_RESET_CURR_PID:
        PG_RESET_CURRENT(pidProfile);
        break;

    case MSP_SET_SENSOR_ALIGNMENT:
        if (dataSize >= 4) {
            gyroConfigMutable()->gyro_align = sbufReadU8(src);
            accelerometerConfigMutable()->acc_align = sbufReadU8(src);
#ifdef USE_MAG
            compassConfigMutable()->mag_align = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#ifdef USE_OPFLOW
            opticalFlowConfigMutable()->opflow_align = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_ADVANCED_CONFIG:
        if (dataSize >= 9) {
            sbufReadU8(src);    // gyroConfig()->gyroSyncDenominator
            sbufReadU8(src);    // BF: masterConfig.pid_process_denom
            sbufReadU8(src);    // BF: motorConfig()->useUnsyncedPwm
            motorConfigMutable()->motorPwmProtocol = sbufReadU8(src);
            motorConfigMutable()->motorPwmRate = sbufReadU16(src);
            servoConfigMutable()->servoPwmRate = sbufReadU16(src);
            sbufReadU8(src);    //Was gyroSync
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_FILTER_CONFIG :
        if (dataSize >= 5) {
            gyroConfigMutable()->gyro_main_lpf_hz = sbufReadU8(src);
            pidProfileMutable()->dterm_lpf_hz = constrain(sbufReadU16(src), 0, 500);
            pidProfileMutable()->yaw_lpf_hz = constrain(sbufReadU16(src), 0, 255);
            if (dataSize >= 9) {
                gyroConfigMutable()->gyro_notch_hz = constrain(sbufReadU16(src), 0, 500);
                gyroConfigMutable()->gyro_notch_cutoff = constrain(sbufReadU16(src), 1, 500);
            } else {
                return MSP_RESULT_ERROR;
            }
            if (dataSize >= 13) {
                sbufReadU16(src);
                sbufReadU16(src);
                pidInitFilters();
            } else {
                return MSP_RESULT_ERROR;
            }
            if (dataSize >= 17) {
                sbufReadU16(src); // Was gyroConfigMutable()->gyro_soft_notch_hz_2
                sbufReadU16(src); // Was gyroConfigMutable()->gyro_soft_notch_cutoff_2
            } else {
                return MSP_RESULT_ERROR;
            }

            if (dataSize >= 21) {
                accelerometerConfigMutable()->acc_notch_hz = constrain(sbufReadU16(src), 0, 255);
                accelerometerConfigMutable()->acc_notch_cutoff = constrain(sbufReadU16(src), 1, 255);
            } else {
                return MSP_RESULT_ERROR;
            }

            if (dataSize >= 22) {
                sbufReadU16(src); //Was gyro_stage2_lowpass_hz
            } else {
                return MSP_RESULT_ERROR;
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_PID_ADVANCED:
        if (dataSize >= 17) {
            sbufReadU16(src);   // pidProfileMutable()->rollPitchItermIgnoreRate
            sbufReadU16(src);   // pidProfileMutable()->yawItermIgnoreRate
            sbufReadU16(src); //pidProfile()->yaw_p_limit

            sbufReadU8(src); //BF: pidProfileMutable()->deltaMethod
            sbufReadU8(src); //BF: pidProfileMutable()->vbatPidCompensation
            sbufReadU8(src); //BF: pidProfileMutable()->setpointRelaxRatio
            sbufReadU8(src);
            pidProfileMutable()->pidSumLimit = sbufReadU16(src);
            sbufReadU8(src); //BF: pidProfileMutable()->itermThrottleGain

            /*
             * To keep compatibility on MSP frame length level with Betaflight, axis axisAccelerationLimitYaw
             * limit will be sent and received in [dps / 10]
             */
            pidProfileMutable()->axisAccelerationLimitRollPitch = sbufReadU16(src) * 10;
            pidProfileMutable()->axisAccelerationLimitYaw = sbufReadU16(src) * 10;
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_INAV_PID:
        if (dataSize >= 15) {
            sbufReadU8(src);  //Legacy, no longer in use async processing value
            sbufReadU16(src);  //Legacy, no longer in use async processing value
            sbufReadU16(src);  //Legacy, no longer in use async processing value
            pidProfileMutable()->heading_hold_rate_limit = sbufReadU8(src);
            sbufReadU8(src); //HEADING_HOLD_ERROR_LPF_FREQ
            sbufReadU16(src); //Legacy yaw_jump_prevention_limit
            gyroConfigMutable()->gyro_lpf = sbufReadU8(src);
            accelerometerConfigMutable()->acc_lpf_hz = sbufReadU8(src);
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
            sbufReadU8(src); //reserved
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_SENSOR_CONFIG:
        if (dataSize >= 6) {
            accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
#ifdef USE_BARO
            barometerConfigMutable()->baro_hardware = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#ifdef USE_MAG
            compassConfigMutable()->mag_hardware = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#ifdef USE_PITOT
            pitotmeterConfigMutable()->pitot_hardware = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
#ifdef USE_RANGEFINDER
            rangefinderConfigMutable()->rangefinder_hardware = sbufReadU8(src);
#else
            sbufReadU8(src);        // rangefinder hardware
#endif
#ifdef USE_OPFLOW
            opticalFlowConfigMutable()->opflow_hardware = sbufReadU8(src);
#else
            sbufReadU8(src);        // optical flow hardware
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

#ifdef USE_NAV
    case MSP_SET_NAV_POSHOLD:
        if (dataSize >= 13) {
            navConfigMutable()->general.flags.user_control_mode = sbufReadU8(src);
            navConfigMutable()->general.max_auto_speed = sbufReadU16(src);
            navConfigMutable()->general.max_auto_climb_rate = sbufReadU16(src);
            navConfigMutable()->general.max_manual_speed = sbufReadU16(src);
            navConfigMutable()->general.max_manual_climb_rate = sbufReadU16(src);
            navConfigMutable()->mc.max_bank_angle = sbufReadU8(src);
            navConfigMutable()->general.flags.use_thr_mid_for_althold = sbufReadU8(src);
            navConfigMutable()->mc.hover_throttle = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_RTH_AND_LAND_CONFIG:
        if (dataSize >= 19) {
            navConfigMutable()->general.min_rth_distance = sbufReadU16(src);
            navConfigMutable()->general.flags.rth_climb_first = sbufReadU8(src);
            navConfigMutable()->general.flags.rth_climb_ignore_emerg = sbufReadU8(src);
            navConfigMutable()->general.flags.rth_tail_first = sbufReadU8(src);
            navConfigMutable()->general.flags.rth_allow_landing = sbufReadU8(src);
            navConfigMutable()->general.flags.rth_alt_control_mode = sbufReadU8(src);
            navConfigMutable()->general.rth_abort_threshold = sbufReadU16(src);
            navConfigMutable()->general.rth_altitude = sbufReadU16(src);
            navConfigMutable()->general.land_minalt_vspd = sbufReadU16(src);
            navConfigMutable()->general.land_maxalt_vspd = sbufReadU16(src);
            navConfigMutable()->general.land_slowdown_minalt = sbufReadU16(src);
            navConfigMutable()->general.land_slowdown_maxalt = sbufReadU16(src);
            navConfigMutable()->general.emerg_descent_rate = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_FW_CONFIG:
        if (dataSize >= 12) {
            navConfigMutable()->fw.cruise_throttle = sbufReadU16(src);
            navConfigMutable()->fw.min_throttle = sbufReadU16(src);
            navConfigMutable()->fw.max_throttle = sbufReadU16(src);
            navConfigMutable()->fw.max_bank_angle = sbufReadU8(src);
            navConfigMutable()->fw.max_climb_angle = sbufReadU8(src);
            navConfigMutable()->fw.max_dive_angle = sbufReadU8(src);
            navConfigMutable()->fw.pitch_to_throttle = sbufReadU8(src);
            navConfigMutable()->fw.loiter_radius = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

    case MSP_SET_CALIBRATION_DATA:
        if (dataSize >= 18) {
            accelerometerConfigMutable()->accZero.raw[X] = sbufReadU16(src);
            accelerometerConfigMutable()->accZero.raw[Y] = sbufReadU16(src);
            accelerometerConfigMutable()->accZero.raw[Z] = sbufReadU16(src);
            accelerometerConfigMutable()->accGain.raw[X] = sbufReadU16(src);
            accelerometerConfigMutable()->accGain.raw[Y] = sbufReadU16(src);
            accelerometerConfigMutable()->accGain.raw[Z] = sbufReadU16(src);

#ifdef USE_MAG
            compassConfigMutable()->magZero.raw[X] = sbufReadU16(src);
            compassConfigMutable()->magZero.raw[Y] = sbufReadU16(src);
            compassConfigMutable()->magZero.raw[Z] = sbufReadU16(src);
#else
            sbufReadU16(src);
            sbufReadU16(src);
            sbufReadU16(src);
#endif
#ifdef USE_OPFLOW
            if (dataSize >= 20) {
                opticalFlowConfigMutable()->opflow_scale = sbufReadU16(src) / 256.0f;
            }
#endif
#ifdef USE_MAG
            if (dataSize >= 22) {
                compassConfigMutable()->magGain[X] = sbufReadU16(src);
                compassConfigMutable()->magGain[Y] = sbufReadU16(src);
                compassConfigMutable()->magGain[Z] = sbufReadU16(src);
            }
#else
            if (dataSize >= 22) {
                sbufReadU16(src);
                sbufReadU16(src);
                sbufReadU16(src);
            }
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

#ifdef USE_NAV
    case MSP_SET_POSITION_ESTIMATION_CONFIG:
        if (dataSize >= 12) {
            positionEstimationConfigMutable()->w_z_baro_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
            positionEstimationConfigMutable()->w_z_gps_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
            positionEstimationConfigMutable()->w_z_gps_v = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
            positionEstimationConfigMutable()->w_xy_gps_p = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
            positionEstimationConfigMutable()->w_xy_gps_v = constrainf(sbufReadU16(src) / 100.0f, 0.0f, 10.0f);
            gpsConfigMutable()->gpsMinSats = constrain(sbufReadU8(src), 5, 10);
            positionEstimationConfigMutable()->use_gps_velned = constrain(sbufReadU8(src), 0, 1);
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

    case MSP_RESET_CONF:
        if (!ARMING_FLAG(ARMED)) {
            resetEEPROM();
            readEEPROM();
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accStartCalibration();
        else
            return MSP_RESULT_ERROR;
        break;

    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            ENABLE_STATE(CALIBRATE_MAG);
        else
            return MSP_RESULT_ERROR;
        break;

#ifdef USE_OPFLOW
    case MSP2_INAV_OPFLOW_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            opflowStartCalibration();
        else
            return MSP_RESULT_ERROR;
        break;
#endif

    case MSP_EEPROM_WRITE:
        if (!ARMING_FLAG(ARMED)) {
            writeEEPROM();
            readEEPROM();
        } else
            return MSP_RESULT_ERROR;
        break;

#ifdef USE_BLACKBOX
    case MSP2_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if ((dataSize >= 5) && blackboxMayEditConfig()) {
            blackboxConfigMutable()->device = sbufReadU8(src);
            blackboxConfigMutable()->rate_num = sbufReadU16(src);
            blackboxConfigMutable()->rate_denom = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

#ifdef USE_OSD
    case MSP_SET_OSD_CONFIG:
        sbufReadU8Safe(&tmp_u8, src);
        // set all the other settings
        if ((int8_t)tmp_u8 == -1) {
            if (dataSize >= 10) {
                osdConfigMutable()->video_system = sbufReadU8(src);
                osdConfigMutable()->units = sbufReadU8(src);
                osdConfigMutable()->rssi_alarm = sbufReadU8(src);
                currentBatteryProfileMutable->capacity.warning = sbufReadU16(src);
                osdConfigMutable()->time_alarm = sbufReadU16(src);
                osdConfigMutable()->alt_alarm = sbufReadU16(src);
                // Won't be read if they weren't provided
                sbufReadU16Safe(&osdConfigMutable()->dist_alarm, src);
                sbufReadU16Safe(&osdConfigMutable()->neg_alt_alarm, src);
            } else
                return MSP_RESULT_ERROR;
        } else {
            // set a position setting
            if ((dataSize >= 3) && (tmp_u8 < OSD_ITEM_COUNT)) // tmp_u8 == addr
                osdLayoutsConfigMutable()->item_pos[0][tmp_u8] = sbufReadU16(src);
            else
                return MSP_RESULT_ERROR;
        }
        // Either a element position change or a units change needs
        // a full redraw, since an element can change size significantly
        // and the old position or the now unused space due to the
        // size change need to be erased.
        osdStartFullRedraw();
        break;

    case MSP_OSD_CHAR_WRITE:
        if (dataSize >= 55) {
            osdCharacter_t chr;
            size_t osdCharacterBytes;
            uint16_t addr;
            if (dataSize >= OSD_CHAR_VISIBLE_BYTES + 2) {
                if (dataSize >= OSD_CHAR_BYTES + 2) {
                    // 16 bit address, full char with metadata
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else if (dataSize >= OSD_CHAR_BYTES + 1) {
                    // 8 bit address, full char with metadata
                    addr = sbufReadU8(src);
                    osdCharacterBytes = OSD_CHAR_BYTES;
                } else {
                    // 16 bit character address, only visible char bytes
                    addr = sbufReadU16(src);
                    osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
                }
            } else {
                // 8 bit character address, only visible char bytes
                addr = sbufReadU8(src);
                osdCharacterBytes = OSD_CHAR_VISIBLE_BYTES;
            }
            for (unsigned ii = 0; ii < MIN(osdCharacterBytes, sizeof(chr.data)); ii++) {
                chr.data[ii] = sbufReadU8(src);
            }
            displayPort_t *osdDisplayPort = osdGetDisplayPort();
            if (osdDisplayPort) {
                displayWriteFontCharacter(osdDisplayPort, addr, &chr);
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;
#endif // USE_OSD

    case MSP_SET_VTX_CONFIG:
        if (dataSize >= 2) {
            vtxDevice_t *vtxDevice = vtxCommonDevice();
            if (vtxDevice) {
                if (vtxCommonGetDeviceType(vtxDevice) != VTXDEV_UNKNOWN) {
                    uint16_t newFrequency = sbufReadU16(src);
                    if (newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL) {  //value is band and channel
                        const uint8_t newBand = (newFrequency / 8) + 1;
                        const uint8_t newChannel = (newFrequency % 8) + 1;
                        vtxSettingsConfigMutable()->band = newBand;
                        vtxSettingsConfigMutable()->channel = newChannel;
                    }

                    if (sbufBytesRemaining(src) > 1) {
                        vtxSettingsConfigMutable()->power = sbufReadU8(src);
                        // Delegate pitmode to vtx directly
                        const uint8_t newPitmode = sbufReadU8(src);
                        uint8_t currentPitmode = 0;
                        vtxCommonGetPitMode(vtxDevice, &currentPitmode);
                        if (currentPitmode != newPitmode) {
                            vtxCommonSetPitMode(vtxDevice, newPitmode);
                        }

                        if (sbufBytesRemaining(src) > 0) {
                            vtxSettingsConfigMutable()->lowPowerDisarm = sbufReadU8(src);
                        }
                    }
                }
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_ERASE:
        flashfsEraseCompletely();
        break;
#endif

#ifdef USE_GPS
    case MSP_SET_RAW_GPS:
        if (dataSize >= 14) {
            if (sbufReadU8(src)) {
                ENABLE_STATE(GPS_FIX);
            } else {
                DISABLE_STATE(GPS_FIX);
            }
            gpsSol.flags.validVelNE = 0;
            gpsSol.flags.validVelD = 0;
            gpsSol.flags.validEPE = 0;
            gpsSol.numSat = sbufReadU8(src);
            gpsSol.llh.lat = sbufReadU32(src);
            gpsSol.llh.lon = sbufReadU32(src);
            gpsSol.llh.alt = sbufReadU16(src);
            gpsSol.groundSpeed = sbufReadU16(src);
            gpsSol.velNED[X] = 0;
            gpsSol.velNED[Y] = 0;
            gpsSol.velNED[Z] = 0;
            gpsSol.eph = 100;
            gpsSol.epv = 100;
            // Feed data to navigation
            sensorsSet(SENSOR_GPS);
            onNewGPSData();
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

#ifdef USE_NAV
    case MSP_SET_WP:
        if (dataSize >= 21) {
            const uint8_t msp_wp_no = sbufReadU8(src);     // get the waypoint number
            navWaypoint_t msp_wp;
            msp_wp.action = sbufReadU8(src);    // action
            msp_wp.lat = sbufReadU32(src);      // lat
            msp_wp.lon = sbufReadU32(src);      // lon
            msp_wp.alt = sbufReadU32(src);      // to set altitude (cm)
            msp_wp.p1 = sbufReadU16(src);       // P1
            msp_wp.p2 = sbufReadU16(src);       // P2
            msp_wp.p3 = sbufReadU16(src);       // P3
            msp_wp.flag = sbufReadU8(src);      // future: to set nav flag
            setWaypoint(msp_wp_no, &msp_wp);
        } else
            return MSP_RESULT_ERROR;
        break;
    case MSP2_COMMON_SET_RADAR_POS:
        if (dataSize >= 19) {
            const uint8_t msp_radar_no = MIN(sbufReadU8(src), RADAR_MAX_POIS - 1); // Radar poi number, 0 to 3
            radar_pois[msp_radar_no].state = sbufReadU8(src);                      // 0=undefined, 1=armed, 2=lost
            radar_pois[msp_radar_no].gps.lat = sbufReadU32(src);                   // lat 10E7
            radar_pois[msp_radar_no].gps.lon = sbufReadU32(src);                   // lon 10E7
            radar_pois[msp_radar_no].gps.alt = sbufReadU32(src);                   // altitude (cm)
            radar_pois[msp_radar_no].heading = sbufReadU16(src);                   // °
            radar_pois[msp_radar_no].speed = sbufReadU16(src);                     // cm/s
            radar_pois[msp_radar_no].lq = sbufReadU8(src);                         // Link quality, from 0 to 4
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

    case MSP_SET_FEATURE:
        if (dataSize >= 4) {
            featureClearAll();
            featureSet(sbufReadU32(src)); // features bitmap
            rxUpdateRSSISource(); // For FEATURE_RSSI_ADC
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_BOARD_ALIGNMENT:
        if (dataSize >= 6) {
            boardAlignmentMutable()->rollDeciDegrees = sbufReadU16(src);
            boardAlignmentMutable()->pitchDeciDegrees = sbufReadU16(src);
            boardAlignmentMutable()->yawDeciDegrees = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
        if (dataSize >= 4) {
#ifdef USE_ADC
            batteryMetersConfigMutable()->voltage.scale = sbufReadU8(src) * 10;
            currentBatteryProfileMutable->voltage.cellMin = sbufReadU8(src) * 10;
            currentBatteryProfileMutable->voltage.cellMax = sbufReadU8(src) * 10;
            currentBatteryProfileMutable->voltage.cellWarning = sbufReadU8(src) * 10;
#else
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
            sbufReadU8(src);
#endif
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_CURRENT_METER_CONFIG:
        if (dataSize >= 7) {
            batteryMetersConfigMutable()->current.scale = sbufReadU16(src);
            batteryMetersConfigMutable()->current.offset = sbufReadU16(src);
            batteryMetersConfigMutable()->current.type = sbufReadU8(src);
            currentBatteryProfileMutable->capacity.value = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_MIXER:
        if (dataSize >= 1) {
            sbufReadU8(src); //This is ignored, no longer supporting mixerMode
            mixerUpdateStateFlags();    // Required for correct preset functionality
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_RX_CONFIG:
        if (dataSize >= 24) {
            rxConfigMutable()->serialrx_provider = sbufReadU8(src);
            rxConfigMutable()->maxcheck = sbufReadU16(src);
            sbufReadU16(src); // midrc
            rxConfigMutable()->mincheck = sbufReadU16(src);
#ifdef USE_SPEKTRUM_BIND
            rxConfigMutable()->spektrum_sat_bind = sbufReadU8(src);
#else
            sbufReadU8(src);
#endif
            rxConfigMutable()->rx_min_usec = sbufReadU16(src);
            rxConfigMutable()->rx_max_usec = sbufReadU16(src);
            sbufReadU8(src); // for compatibility with betaflight (rcInterpolation)
            sbufReadU8(src); // for compatibility with betaflight (rcInterpolationInterval)
            sbufReadU16(src); // for compatibility with betaflight (airModeActivateThreshold)
#ifdef USE_RX_SPI
            rxConfigMutable()->rx_spi_protocol = sbufReadU8(src);
            rxConfigMutable()->rx_spi_id = sbufReadU32(src);
            rxConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
#else
            sbufReadU8(src);
            sbufReadU32(src);
            sbufReadU8(src);
#endif
            sbufReadU8(src); // for compatibility with betaflight (fpvCamAngleDegrees)
            rxConfigMutable()->receiverType = sbufReadU8(src);              // Won't be modified if buffer is not large enough
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        if (dataSize >= 20) {
            failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
            failsafeConfigMutable()->failsafe_off_delay = sbufReadU8(src);
            failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
            sbufReadU8(src); // was failsafe_kill_switch
            failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
            failsafeConfigMutable()->failsafe_recovery_delay = sbufReadU8(src);
            failsafeConfigMutable()->failsafe_fw_roll_angle = (int16_t)sbufReadU16(src);
            failsafeConfigMutable()->failsafe_fw_pitch_angle = (int16_t)sbufReadU16(src);
            failsafeConfigMutable()->failsafe_fw_yaw_rate = (int16_t)sbufReadU16(src);
            failsafeConfigMutable()->failsafe_stick_motion_threshold = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_min_distance = sbufReadU16(src);
            failsafeConfigMutable()->failsafe_min_distance_procedure = sbufReadU8(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_RSSI_CONFIG:
        sbufReadU8Safe(&tmp_u8, src);
        if ((dataSize >= 1) && (tmp_u8 <= MAX_SUPPORTED_RC_CHANNEL_COUNT)) {
            rxConfigMutable()->rssi_channel = tmp_u8;
            rxUpdateRSSISource();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RX_MAP:
        if (dataSize >= MAX_MAPPABLE_RX_INPUTS) {
            for (int i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
                rxConfigMutable()->rcmap[i] = sbufReadU8(src);
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_BF_CONFIG:
        if (dataSize >= 16) {
            sbufReadU8(src); // mixerMode no longer supported, just swallow
            mixerUpdateStateFlags();    // Required for correct preset functionality

            featureClearAll();
            featureSet(sbufReadU32(src)); // features bitmap

            rxConfigMutable()->serialrx_provider = sbufReadU8(src); // serialrx_type

            boardAlignmentMutable()->rollDeciDegrees = sbufReadU16(src); // board_align_roll
            boardAlignmentMutable()->pitchDeciDegrees = sbufReadU16(src); // board_align_pitch
            boardAlignmentMutable()->yawDeciDegrees = sbufReadU16(src); // board_align_yaw

            batteryMetersConfigMutable()->current.scale = sbufReadU16(src);
            batteryMetersConfigMutable()->current.offset = sbufReadU16(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->gps_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->telemetry_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->peripheral_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
            }
        }
        break;

    case MSP2_COMMON_SET_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint32_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU32(src);
                portConfig->msp_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->gps_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->telemetry_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
                portConfig->peripheral_baudrateIndex = constrain(sbufReadU8(src), BAUD_MIN, BAUD_MAX);
            }
        }
        break;

#ifdef USE_LED_STRIP
    case MSP_SET_LED_COLORS:
        if (dataSize >= LED_CONFIGURABLE_COLOR_COUNT * 4) {
            for (int i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
                hsvColor_t *color = &ledStripConfigMutable()->colors[i];
                color->h = sbufReadU16(src);
                color->s = sbufReadU8(src);
                color->v = sbufReadU8(src);
            }
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_LED_STRIP_CONFIG:
        if (dataSize >= 5) {
            tmp_u8 = sbufReadU8(src);
            if (tmp_u8 >= LED_MAX_STRIP_LENGTH || dataSize != (1 + 4)) {
                return MSP_RESULT_ERROR;
            }
            ledConfig_t *ledConfig = &ledStripConfigMutable()->ledConfigs[tmp_u8];
            *ledConfig = sbufReadU32(src);
            reevaluateLedConfig();
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_LED_STRIP_MODECOLOR:
        if (dataSize >= 3) {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color))
                return MSP_RESULT_ERROR;
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

#ifdef NAV_NON_VOLATILE_WAYPOINT_STORAGE
    case MSP_WP_MISSION_LOAD:
        sbufReadU8Safe(NULL, src);    // Mission ID (reserved)
        if ((dataSize != 1) || (!loadNonVolatileWaypointList()))
            return MSP_RESULT_ERROR;
        break;

    case MSP_WP_MISSION_SAVE:
        sbufReadU8Safe(NULL, src);    // Mission ID (reserved)
        if ((dataSize != 1) || (!saveNonVolatileWaypointList()))
            return MSP_RESULT_ERROR;
        break;
#endif

    case MSP_SET_RTC:
        if (dataSize >= 6) {
            // Use seconds and milliseconds to make senders
            // easier to implement. Generating a 64 bit value
            // might not be trivial in some platforms.
            int32_t secs = (int32_t)sbufReadU32(src);
            uint16_t millis = sbufReadU16(src);
            rtcTime_t t = rtcTimeMake(secs, millis);
            rtcSet(&t);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP_SET_TX_INFO:
        {
            // This message will be sent while the aircraft is
            // armed. Better to guard ourselves against potentially
            // malformed requests.
            uint8_t rssi;
            if (sbufReadU8Safe(&rssi, src)) {
                setRSSIFromMSP(rssi);
            }
        }
        break;

    case MSP_SET_NAME:
        if (dataSize <= MAX_NAME_LENGTH) {
            char *name = systemConfigMutable()->name;
            int len = MIN(MAX_NAME_LENGTH, (int)dataSize);
            sbufReadData(src, name, len);
            memset(&name[len], '\0', (MAX_NAME_LENGTH + 1) - len);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_COMMON_SET_TZ:
        if (dataSize == 2)
            timeConfigMutable()->tz_offset = (int16_t)sbufReadU16(src);
        else if (dataSize == 3) {
            timeConfigMutable()->tz_offset = (int16_t)sbufReadU16(src);
            timeConfigMutable()->tz_automatic_dst = (uint8_t)sbufReadU8(src);
        } else
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SET_MIXER:
        mixerConfigMutable()->motorDirectionInverted = sbufReadU8(src);
        sbufReadU16(src); // Was yaw_jump_prevention_limit
        mixerConfigMutable()->platformType = sbufReadU8(src);
        mixerConfigMutable()->hasFlaps = sbufReadU8(src);
        mixerConfigMutable()->appliedMixerPreset = sbufReadU16(src);
        sbufReadU8(src); //Read and ignore MAX_SUPPORTED_MOTORS
        sbufReadU8(src); //Read and ignore MAX_SUPPORTED_SERVOS
        mixerUpdateStateFlags();
        break;

#if defined(USE_OSD)
    case MSP2_INAV_OSD_SET_LAYOUT_ITEM:
        {
            uint8_t layout;
            if (!sbufReadU8Safe(&layout, src)) {
                return MSP_RESULT_ERROR;
            }
            uint8_t item;
            if (!sbufReadU8Safe(&item, src)) {
                return MSP_RESULT_ERROR;
            }
            if (!sbufReadU16Safe(&osdLayoutsConfigMutable()->item_pos[layout][item], src)) {
                return MSP_RESULT_ERROR;
            }
            // If the layout is not already overriden and it's different
            // than the layout for the item that was just configured,
            // override it for 10 seconds.
            bool overridden;
            int activeLayout = osdGetActiveLayout(&overridden);
            if (activeLayout != layout && !overridden) {
                osdOverrideLayout(layout, 10000);
            } else {
                osdStartFullRedraw();
            }
        }

        break;

    case MSP2_INAV_OSD_SET_ALARMS:
        {
            if (dataSize == 24) {
                osdConfigMutable()->rssi_alarm = sbufReadU8(src);
                osdConfigMutable()->time_alarm = sbufReadU16(src);
                osdConfigMutable()->alt_alarm = sbufReadU16(src);
                osdConfigMutable()->dist_alarm = sbufReadU16(src);
                osdConfigMutable()->neg_alt_alarm = sbufReadU16(src);
                tmp_u16 = sbufReadU16(src);
                osdConfigMutable()->gforce_alarm = tmp_u16 / 1000.0f;
                tmp_u16 = sbufReadU16(src);
                osdConfigMutable()->gforce_axis_alarm_min = (int16_t)tmp_u16 / 1000.0f;
                tmp_u16 = sbufReadU16(src);
                osdConfigMutable()->gforce_axis_alarm_max = (int16_t)tmp_u16 / 1000.0f;
                osdConfigMutable()->current_alarm = sbufReadU8(src);
                osdConfigMutable()->imu_temp_alarm_min = sbufReadU16(src);
                osdConfigMutable()->imu_temp_alarm_max = sbufReadU16(src);
#ifdef USE_BARO
                osdConfigMutable()->baro_temp_alarm_min = sbufReadU16(src);
                osdConfigMutable()->baro_temp_alarm_max = sbufReadU16(src);
#endif
            } else
                return MSP_RESULT_ERROR;
        }

        break;

    case MSP2_INAV_OSD_SET_PREFERENCES:
        {
            if (dataSize == 9) {
                osdConfigMutable()->video_system = sbufReadU8(src);
                osdConfigMutable()->main_voltage_decimals = sbufReadU8(src);
                osdConfigMutable()->ahi_reverse_roll = sbufReadU8(src);
                osdConfigMutable()->crosshairs_style = sbufReadU8(src);
                osdConfigMutable()->left_sidebar_scroll = sbufReadU8(src);
                osdConfigMutable()->right_sidebar_scroll = sbufReadU8(src);
                osdConfigMutable()->sidebar_scroll_arrows = sbufReadU8(src);
                osdConfigMutable()->units = sbufReadU8(src);
                osdConfigMutable()->stats_energy_unit = sbufReadU8(src);
                osdStartFullRedraw();
            } else
                return MSP_RESULT_ERROR;
        }

        break;
#endif

    case MSP2_INAV_SET_MC_BRAKING:
#ifdef USE_MR_BRAKING_MODE
        if (dataSize == 14) {
            navConfigMutable()->mc.braking_speed_threshold = sbufReadU16(src);
            navConfigMutable()->mc.braking_disengage_speed = sbufReadU16(src);
            navConfigMutable()->mc.braking_timeout = sbufReadU16(src);
            navConfigMutable()->mc.braking_boost_factor = sbufReadU8(src);
            navConfigMutable()->mc.braking_boost_timeout = sbufReadU16(src);
            navConfigMutable()->mc.braking_boost_speed_threshold = sbufReadU16(src);
            navConfigMutable()->mc.braking_boost_disengage_speed = sbufReadU16(src);
            navConfigMutable()->mc.braking_bank_angle = sbufReadU8(src);
        } else
#endif
            return MSP_RESULT_ERROR;
        break;

    case MSP2_INAV_SELECT_BATTERY_PROFILE:
        if (!ARMING_FLAG(ARMED) && sbufReadU8Safe(&tmp_u8, src)) {
                setConfigBatteryProfileAndWriteEEPROM(tmp_u8);
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

#ifdef USE_TEMPERATURE_SENSOR
    case MSP2_INAV_SET_TEMP_SENSOR_CONFIG:
        if (dataSize == sizeof(tempSensorConfig_t) * MAX_TEMP_SENSORS) {
            for (uint8_t index = 0; index < MAX_TEMP_SENSORS; ++index) {
                tempSensorConfig_t *sensorConfig = tempSensorConfigMutable(index);
                sensorConfig->type = sbufReadU8(src);
                for (uint8_t addrIndex = 0; addrIndex < 8; ++addrIndex)
                    ((uint8_t *)&sensorConfig->address)[addrIndex] = sbufReadU8(src);
                sensorConfig->alarm_min = sbufReadU16(src);
                sensorConfig->alarm_max = sbufReadU16(src);
                tmp_u8 = sbufReadU8(src);
                sensorConfig->osdSymbol = tmp_u8 > TEMP_SENSOR_SYM_COUNT ? 0 : tmp_u8;
                for (uint8_t labelIndex = 0; labelIndex < TEMPERATURE_LABEL_LEN; ++labelIndex)
                    sensorConfig->label[labelIndex] = toupper(sbufReadU8(src));
            }
        } else
            return MSP_RESULT_ERROR;
        break;
#endif

#ifdef MSP_FIRMWARE_UPDATE
    case MSP2_INAV_FWUPDT_PREPARE:
        if (!firmwareUpdatePrepare(sbufReadU32(src))) {
            return MSP_RESULT_ERROR;
        }
        break;
    case MSP2_INAV_FWUPDT_STORE:
        if (!firmwareUpdateStore(sbufPtr(src), sbufBytesRemaining(src))) {
            return MSP_RESULT_ERROR;
        }
        break;
    case MSP2_INAV_FWUPDT_EXEC:
        firmwareUpdateExec(sbufReadU8(src));
        return MSP_RESULT_ERROR; // will only be reached if the update is not ready
        break;
    case MSP2_INAV_FWUPDT_ROLLBACK_PREPARE:
        if (!firmwareUpdateRollbackPrepare()) {
            return MSP_RESULT_ERROR;
        }
        break;
    case MSP2_INAV_FWUPDT_ROLLBACK_EXEC:
        firmwareUpdateRollbackExec();
        return MSP_RESULT_ERROR; // will only be reached if the rollback is not ready
        break;
#endif
    case MSP2_INAV_SET_SAFEHOME:
        if (dataSize == 10) {
             uint8_t i;
             if (!sbufReadU8Safe(&i, src) || i >= MAX_SAFE_HOMES) {
                 return MSP_RESULT_ERROR;
             }
             safeHomeConfigMutable(i)->enabled = sbufReadU8(src);
             safeHomeConfigMutable(i)->lat = sbufReadU32(src);
             safeHomeConfigMutable(i)->lon = sbufReadU32(src);
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    default:
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

static const setting_t *mspReadSetting(sbuf_t *src)
{
    char name[SETTING_MAX_NAME_LENGTH];
    uint16_t id;
    uint8_t c;
    size_t s = 0;
    while (1) {
        if (!sbufReadU8Safe(&c, src)) {
            return NULL;
        }
        name[s++] = c;
        if (c == '\0') {
            if (s == 1) {
                // Payload starts with a zero, setting index
                // as uint16_t follows
                if (sbufReadU16Safe(&id, src)) {
                    return settingGet(id);
                }
                return NULL;
            }
            break;
        }
        if (s == SETTING_MAX_NAME_LENGTH) {
            // Name is too long
            return NULL;
        }
    }
    return settingFind(name);
}

static bool mspSettingCommand(sbuf_t *dst, sbuf_t *src)
{
    const setting_t *setting = mspReadSetting(src);
    if (!setting) {
        return false;
    }

    const void *ptr = settingGetValuePointer(setting);
    size_t size = settingGetValueSize(setting);
    sbufWriteDataSafe(dst, ptr, size);
    return true;
}

static bool mspSetSettingCommand(sbuf_t *dst, sbuf_t *src)
{
    UNUSED(dst);

    const setting_t *setting = mspReadSetting(src);
    if (!setting) {
        return false;
    }

    setting_min_t min = settingGetMin(setting);
    setting_max_t max = settingGetMax(setting);

    void *ptr = settingGetValuePointer(setting);
    switch (SETTING_TYPE(setting)) {
        case VAR_UINT8:
            {
                uint8_t val;
                if (!sbufReadU8Safe(&val, src)) {
                    return false;
                }
                if (val > max) {
                    return false;
                }
                *((uint8_t*)ptr) = val;
            }
            break;
        case VAR_INT8:
            {
                int8_t val;
                if (!sbufReadI8Safe(&val, src)) {
                    return false;
                }
                if (val < min || val > (int8_t)max) {
                    return false;
                }
                *((int8_t*)ptr) = val;
            }
            break;
        case VAR_UINT16:
            {
                uint16_t val;
                if (!sbufReadU16Safe(&val, src)) {
                    return false;
                }
                if (val > max) {
                    return false;
                }
                *((uint16_t*)ptr) = val;
            }
            break;
        case VAR_INT16:
            {
                int16_t val;
                if (!sbufReadI16Safe(&val, src)) {
                    return false;
                }
                if (val < min || val > (int16_t)max) {
                    return false;
                }
                *((int16_t*)ptr) = val;
            }
            break;
        case VAR_UINT32:
            {
                uint32_t val;
                if (!sbufReadU32Safe(&val, src)) {
                    return false;
                }
                if (val > max) {
                    return false;
                }
                *((uint32_t*)ptr) = val;
            }
            break;
        case VAR_FLOAT:
            {
                float val;
                if (!sbufReadDataSafe(src, &val, sizeof(float))) {
                    return false;
                }
                if (val < (float)min || val > (float)max) {
                    return false;
                }
                *((float*)ptr) = val;
            }
            break;
        case VAR_STRING:
            {
                settingSetString(setting, (const char*)sbufPtr(src), sbufBytesRemaining(src));
            }
            break;
    }

    return true;
}

static bool mspSettingInfoCommand(sbuf_t *dst, sbuf_t *src)
{
    const setting_t *setting = mspReadSetting(src);
    if (!setting) {
        return false;
    }

    char name_buf[SETTING_MAX_WORD_LENGTH+1];
    settingGetName(setting, name_buf);
    sbufWriteDataSafe(dst, name_buf, strlen(name_buf) + 1);

    // Parameter Group ID
    sbufWriteU16(dst, settingGetPgn(setting));

    // Type, section and mode
    sbufWriteU8(dst, SETTING_TYPE(setting));
    sbufWriteU8(dst, SETTING_SECTION(setting));
    sbufWriteU8(dst, SETTING_MODE(setting));

    // Min as int32_t
    int32_t min = settingGetMin(setting);
    sbufWriteU32(dst, (uint32_t)min);
    // Max as uint32_t
    uint32_t max = settingGetMax(setting);
    sbufWriteU32(dst, max);

    // Absolute setting index
    sbufWriteU16(dst, settingGetIndex(setting));

    // If the setting is profile based, send the current one
    // and the count, both as uint8_t. For MASTER_VALUE, we
    // send two zeroes, so the MSP client can assume there
    // will always be two bytes.
    switch (SETTING_SECTION(setting)) {
    case MASTER_VALUE:
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
        break;
    case PROFILE_VALUE:
        FALLTHROUGH;
    case CONTROL_RATE_VALUE:
        sbufWriteU8(dst, getConfigProfile());
        sbufWriteU8(dst, MAX_PROFILE_COUNT);
        break;
    case BATTERY_CONFIG_VALUE:
        sbufWriteU8(dst, getConfigBatteryProfile());
        sbufWriteU8(dst, MAX_BATTERY_PROFILE_COUNT);
        break;
    }

    // If the setting uses a table, send each possible string (null terminated)
    if (SETTING_MODE(setting) == MODE_LOOKUP) {
        for (int ii = (int)min; ii <= (int)max; ii++) {
            const char *name = settingLookupValueName(setting, ii);
            sbufWriteDataSafe(dst, name, strlen(name) + 1);
        }
    }

    // Finally, include the setting value. This way resource constrained callers
    // (e.g. a script in the radio) don't need to perform another call to retrieve
    // the value.
    const void *ptr = settingGetValuePointer(setting);
    size_t size = settingGetValueSize(setting);
    sbufWriteDataSafe(dst, ptr, size);

    return true;
}

static bool mspParameterGroupsCommand(sbuf_t *dst, sbuf_t *src)
{
    uint16_t first;
    uint16_t last;
    uint16_t start;
    uint16_t end;

    if (sbufReadU16Safe(&first, src)) {
        last = first;
    } else {
        first = PG_ID_FIRST;
        last = PG_ID_LAST;
    }

    for (int ii = first; ii <= last; ii++) {
        if (settingsGetParameterGroupIndexes(ii, &start, &end)) {
            sbufWriteU16(dst, ii);
            sbufWriteU16(dst, start);
            sbufWriteU16(dst, end);
        }
    }
    return true;
}

bool mspFCProcessInOutCommand(uint16_t cmdMSP, sbuf_t *dst, sbuf_t *src, mspResult_e *ret)
{
    switch (cmdMSP) {

#if defined(USE_NAV)
    case MSP_WP:
        mspFcWaypointOutCommand(dst, src);
        *ret = MSP_RESULT_ACK;
        break;
#endif

#if defined(USE_FLASHFS)
    case MSP_DATAFLASH_READ:
        mspFcDataFlashReadCommand(dst, src);
        *ret = MSP_RESULT_ACK;
        break;
#endif

    case MSP2_COMMON_SETTING:
        *ret = mspSettingCommand(dst, src) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
        break;

    case MSP2_COMMON_SET_SETTING:
        *ret = mspSetSettingCommand(dst, src) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
        break;

    case MSP2_COMMON_SETTING_INFO:
        *ret = mspSettingInfoCommand(dst, src) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
        break;

    case MSP2_COMMON_PG_LIST:
        *ret = mspParameterGroupsCommand(dst, src) ? MSP_RESULT_ACK : MSP_RESULT_ERROR;
        break;

#if defined(USE_OSD)
    case MSP2_INAV_OSD_LAYOUTS:
        if (sbufBytesRemaining(src) >= 1) {
            uint8_t layout = sbufReadU8(src);
            if (layout >= OSD_LAYOUT_COUNT) {
                *ret = MSP_RESULT_ERROR;
                break;
            }
            if (sbufBytesRemaining(src) >= 2) {
                // Asking for an specific item in a layout
                uint16_t item = sbufReadU16(src);
                if (item >= OSD_ITEM_COUNT) {
                    *ret = MSP_RESULT_ERROR;
                    break;
                }
                sbufWriteU16(dst, osdLayoutsConfig()->item_pos[layout][item]);
            } else {
                // Asking for an specific layout
                for (unsigned ii = 0; ii < OSD_ITEM_COUNT; ii++) {
                    sbufWriteU16(dst, osdLayoutsConfig()->item_pos[layout][ii]);
                }
            }
        } else {
            // Return the number of layouts and items
            sbufWriteU8(dst, OSD_LAYOUT_COUNT);
            sbufWriteU8(dst, OSD_ITEM_COUNT);
        }
        *ret = MSP_RESULT_ACK;
        break;
#endif

    case MSP2_INAV_SAFEHOME:
         *ret = mspFcSafeHomeOutCommand(dst, src);
         break;

    default:
        // Not handled
        return false;
    }
    return true;
}

static mspResult_e mspProcessSensorCommand(uint16_t cmdMSP, sbuf_t *src)
{
    UNUSED(src);

    switch (cmdMSP) {
#if defined(USE_RANGEFINDER_MSP)
        case MSP2_SENSOR_RANGEFINDER:
            mspRangefinderReceiveNewData(sbufPtr(src));
            break;
#endif

#if defined(USE_OPFLOW_MSP)
        case MSP2_SENSOR_OPTIC_FLOW:
            mspOpflowReceiveNewData(sbufPtr(src));
            break;
#endif

#if defined(USE_GPS_PROTO_MSP)
        case MSP2_SENSOR_GPS:
            mspGPSReceiveNewData(sbufPtr(src));
            break;
#endif

#if defined(USE_MAG_MSP)
        case MSP2_SENSOR_COMPASS:
            mspMagReceiveNewData(sbufPtr(src));
            break;
#endif

#if defined(USE_BARO_MSP)
        case MSP2_SENSOR_BAROMETER:
            mspBaroReceiveNewData(sbufPtr(src));
            break;
#endif

#if defined(USE_PITOT_MSP)
        case MSP2_SENSOR_AIRSPEED:
            mspPitotmeterReceiveNewData(sbufPtr(src));
            break;
#endif
    }

    return MSP_RESULT_NO_REPLY;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    mspResult_e ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (MSP2_IS_SENSOR_MESSAGE(cmdMSP)) {
        ret = mspProcessSensorCommand(cmdMSP, src);
    } else if (mspFcProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
    } else if (cmdMSP == MSP_SET_PASSTHROUGH) {
        mspFcSetPassthroughCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
    } else {
        if (!mspFCProcessInOutCommand(cmdMSP, dst, src, &ret)) {
            ret = mspFcProcessInCommand(cmdMSP, src);
        }
    }

    // Process DONT_REPLY flag
    if (cmd->flags & MSP_FLAG_DONT_REPLY) {
        ret = MSP_RESULT_NO_REPLY;
    }

    reply->result = ret;
    return ret;
}

/*
 * Return a pointer to the process command function
 */
void mspFcInit(void)
{
    initActiveBoxIds();
}
