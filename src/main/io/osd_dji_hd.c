/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Konstantin Sharlaimov (ksharlaimov@inavflight.com)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"
#include "build/version.h"

#include "common/streambuf.h"
#include "common/utils.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/crc.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"
#include "fc/fc_msp_box.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/mixer.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/osd_dji_hd.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"
#include "sensors/esc_sensor.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "navigation/navigation.h"

#include "scheduler/scheduler.h"

#if defined(USE_DJI_HD_OSD)

#define DJI_MSP_BAUDRATE                    115200

#define DJI_ARMING_DISABLE_FLAGS_COUNT      25
#define DJI_OSD_WARNING_COUNT               16
#define DJI_OSD_TIMER_COUNT                 2
#define DJI_OSD_FLAGS_OSD_FEATURE           (1 << 0)

/* 
 * DJI HD goggles use MSPv1 compatible with Betaflight 4.1.0
 * DJI uses a subset of messages and assume fixed bit positions for flight modes
 *
 * To avoid compatibility issues we maintain a separate MSP command processor
 * but reuse the packet decoder to minimize code duplication
 */

// External dependency on looptime
extern timeDelta_t cycleTime;

// MSP packet decoder state structure
static mspPort_t djiMspPort;

// Mapping table between DJI PID and INAV PID (order is different)
const uint8_t djiPidIndexMap[] = {
    PID_ROLL,   // DJI: PID_ROLL
    PID_PITCH,  // DJI: PID_PITCH
    PID_YAW,    // DJI: PID_YAW
    PID_LEVEL,  // DJI: PID_LEVEL
    PID_HEADING // DJI: PID_MAG
};

const int djiOSDItemIndexMap[] = {
    OSD_RSSI_VALUE,             // DJI: OSD_RSSI_VALUE
    OSD_MAIN_BATT_VOLTAGE,      // DJI: OSD_MAIN_BATT_VOLTAGE
    OSD_CROSSHAIRS,             // DJI: OSD_CROSSHAIRS
    OSD_ARTIFICIAL_HORIZON,     // DJI: OSD_ARTIFICIAL_HORIZON
    OSD_HORIZON_SIDEBARS,       // DJI: OSD_HORIZON_SIDEBARS
    OSD_ONTIME,                 // DJI: OSD_ITEM_TIMER_1
    OSD_FLYTIME,                // DJI: OSD_ITEM_TIMER_2
    OSD_FLYMODE,                // DJI: OSD_FLYMODE
    OSD_CRAFT_NAME,             // DJI: OSD_CRAFT_NAME
    OSD_THROTTLE_POS,           // DJI: OSD_THROTTLE_POS
    OSD_VTX_CHANNEL,            // DJI: OSD_VTX_CHANNEL
    OSD_CURRENT_DRAW,           // DJI: OSD_CURRENT_DRAW
    OSD_MAH_DRAWN,              // DJI: OSD_MAH_DRAWN
    OSD_GPS_SPEED,              // DJI: OSD_GPS_SPEED
    OSD_GPS_SATS,               // DJI: OSD_GPS_SATS
    OSD_ALTITUDE,               // DJI: OSD_ALTITUDE
    OSD_ROLL_PIDS,              // DJI: OSD_ROLL_PIDS
    OSD_PITCH_PIDS,             // DJI: OSD_PITCH_PIDS
    OSD_YAW_PIDS,               // DJI: OSD_YAW_PIDS
    OSD_POWER,                  // DJI: OSD_POWER
    -1,                         // DJI: OSD_PIDRATE_PROFILE
    -1,                         // DJI: OSD_WARNINGS
    OSD_MAIN_BATT_CELL_VOLTAGE, // DJI: OSD_AVG_CELL_VOLTAGE
    OSD_GPS_LON,                // DJI: OSD_GPS_LON
    OSD_GPS_LAT,                // DJI: OSD_GPS_LAT
    OSD_DEBUG,                  // DJI: OSD_DEBUG
    OSD_ATTITUDE_PITCH,         // DJI: OSD_PITCH_ANGLE
    OSD_ATTITUDE_ROLL,          // DJI: OSD_ROLL_ANGLE
    -1,                         // DJI: OSD_MAIN_BATT_USAGE
    -1,                         // DJI: OSD_DISARMED
    OSD_HOME_DIR,               // DJI: OSD_HOME_DIR
    OSD_HOME_DIST,              // DJI: OSD_HOME_DIST
    OSD_HEADING,                // DJI: OSD_NUMERICAL_HEADING
    OSD_VARIO_NUM,              // DJI: OSD_NUMERICAL_VARIO
    -1,                         // DJI: OSD_COMPASS_BAR
    -1,                         // DJI: OSD_ESC_TMP
    OSD_ESC_RPM,                // DJI: OSD_ESC_RPM
    OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH,   // DJI: OSD_REMAINING_TIME_ESTIMATE
    OSD_RTC_TIME,               // DJI: OSD_RTC_DATETIME
    -1,                         // DJI: OSD_ADJUSTMENT_RANGE
    -1,                         // DJI: OSD_CORE_TEMPERATURE
    -1,                         // DJI: OSD_ANTI_GRAVITY
    -1,                         // DJI: OSD_G_FORCE
    -1,                         // DJI: OSD_MOTOR_DIAG
    -1,                         // DJI: OSD_LOG_STATUS
    -1,                         // DJI: OSD_FLIP_ARROW
    -1,                         // DJI: OSD_LINK_QUALITY
    OSD_TRIP_DIST,              // DJI: OSD_FLIGHT_DIST
    -1,                         // DJI: OSD_STICK_OVERLAY_LEFT
    -1,                         // DJI: OSD_STICK_OVERLAY_RIGHT
    -1,                         // DJI: OSD_DISPLAY_NAME
    -1,                         // DJI: OSD_ESC_RPM_FREQ
    -1,                         // DJI: OSD_RATE_PROFILE_NAME
    -1,                         // DJI: OSD_PID_PROFILE_NAME
    -1,                         // DJI: OSD_PROFILE_NAME
    -1,                         // DJI: OSD_RSSI_DBM_VALUE
    -1,                         // DJI: OSD_RC_CHANNELS
};

const int djiOSDStatisticsMap[] = {
    -1,                         // DJI: OSD_STAT_RTC_DATE_TIME
    -1,                         // DJI: OSD_STAT_TIMER_1
    -1,                         // DJI: OSD_STAT_TIMER_2
    -1,                         // DJI: OSD_STAT_MAX_SPEED
    -1,                         // DJI: OSD_STAT_MAX_DISTANCE
    -1,                         // DJI: OSD_STAT_MIN_BATTERY
    -1,                         // DJI: OSD_STAT_END_BATTERY
    -1,                         // DJI: OSD_STAT_BATTERY
    -1,                         // DJI: OSD_STAT_MIN_RSSI
    -1,                         // DJI: OSD_STAT_MAX_CURRENT
    -1,                         // DJI: OSD_STAT_USED_MAH
    -1,                         // DJI: OSD_STAT_MAX_ALTITUDE
    -1,                         // DJI: OSD_STAT_BLACKBOX
    -1,                         // DJI: OSD_STAT_BLACKBOX_NUMBER
    -1,                         // DJI: OSD_STAT_MAX_G_FORCE
    -1,                         // DJI: OSD_STAT_MAX_ESC_TEMP
    -1,                         // DJI: OSD_STAT_MAX_ESC_RPM
    -1,                         // DJI: OSD_STAT_MIN_LINK_QUALITY
    -1,                         // DJI: OSD_STAT_FLIGHT_DISTANCE
    -1,                         // DJI: OSD_STAT_MAX_FFT
    -1,                         // DJI: OSD_STAT_TOTAL_FLIGHTS
    -1,                         // DJI: OSD_STAT_TOTAL_TIME
    -1,                         // DJI: OSD_STAT_TOTAL_DIST
    -1,                         // DJI: OSD_STAT_MIN_RSSI_DBM
};

void djiOsdSerialInit(void)
{
    memset(&djiMspPort, 0, sizeof(mspPort_t));

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_DJI_HD_OSD);

    if (!portConfig) {
        return;
    }

    serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_DJI_HD_OSD, NULL, NULL, DJI_MSP_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED);

    if (serialPort) {
        resetMspPort(&djiMspPort, serialPort);
    }
}

static void djiPackBoxModeBitmask(boxBitmask_t * flightModeBitmask)
{
    memset(flightModeBitmask, 0, sizeof(boxBitmask_t));

    // Map flight modes to DJI-supported bits
    switch(getFlightModeForTelemetry()) {
        case FLM_MANUAL:
        case FLM_ACRO:
        case FLM_ACRO_AIR:
            // DJI: No bits set = ACRO
            break;
        case FLM_ANGLE:
            bitArraySet(flightModeBitmask->bits, 1);    // DJI: 1 << 1 : ANGLE
            break;
        case FLM_HORIZON:
            bitArraySet(flightModeBitmask->bits, 2);    // DJI: 1 << 2
            break;
        case FLM_RTH:
            bitArraySet(flightModeBitmask->bits, 5);    // DJI: 1 << 5 : GPS_RESQUE
            break;
        case FLM_CRUISE:
            bitArraySet(flightModeBitmask->bits, 3);    // DJI: 1 << 3 : technically HEADFREE
            break;
        case FLM_FAILSAFE:
            bitArraySet(flightModeBitmask->bits, 4);    // DJI: 1 << 4
            break;
        case FLM_LAUNCH:
        case FLM_ALTITUDE_HOLD:
        case FLM_POSITION_HOLD:
        case FLM_MISSION:
        default:
            // Unsupported ATM, keep at ANGLE
            bitArraySet(flightModeBitmask->bits, 1);    // DJI: 1 << 1 : ANGLE
    }

    // Set ARMED mode
    if (ARMING_FLAG(ARMED)) {
        bitArraySet(flightModeBitmask->bits, 0);        // DJI: 1 << 0 : ARMED
    }
}

static uint32_t djiPackArmingDisabledFlags(void)
{
    // TODO: Map INAV arming disabled flags to DJI/BF ones
    // https://github.com/betaflight/betaflight/blob/c6e5882dd91fa20d246b8f8af10cf6c92876bc3d/src/main/fc/runtime_config.h#L42
    // For now hide everything in ARMING_DISABLED_ARM_SWITCH (bit 24)

    return isArmingDisabled() ? (1 << 24) : 0;
}

static uint32_t djiEncodeOSDEnabledWarnings(void)
{
    // TODO:
    return 0;
}

static void djiSerializeOSDConfigReply(sbuf_t *dst)
{
    // Only send supported flag - always
    sbufWriteU8(dst, DJI_OSD_FLAGS_OSD_FEATURE);

    // 7456 video system (AUTO/PAL/NTSC)
    sbufWriteU8(dst, osdConfig()->video_system);

    // Configuration
    sbufWriteU8(dst, osdConfig()->units);

    // Alarms
    sbufWriteU8(dst, osdConfig()->rssi_alarm);
    sbufWriteU16(dst, currentBatteryProfile->capacity.warning);

    // OSD_ITEM_COUNT (previously was timer alarm)
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, ARRAYLEN(djiOSDItemIndexMap));

    // Altitude alarm
    sbufWriteU16(dst, osdConfig()->alt_alarm);

    // OSD element position and visibility
    for (unsigned i = 0; i < ARRAYLEN(djiOSDItemIndexMap); i++) {
        int inavOSDIdx = djiOSDItemIndexMap[i];
        if (inavOSDIdx >= 0) {
            // Position & visibility encoded in 16 bits. Position encoding is the same between BF/DJI and INAV
            // However visibility is different. INAV has 3 layouts, while BF only has visibility profiles
            // Here we use only one OSD layout mapped to first OSD BF profile
            uint16_t itemPos = osdConfig()->item_pos[0][inavOSDIdx];

            // Workarounds for certain OSD element positions
            // INAV calculates these dynamically, while DJI expects the config to have defined coordinates
            switch(inavOSDIdx) {
                case OSD_CROSSHAIRS:
                    itemPos = (itemPos & (~OSD_POS_MAX)) | OSD_POS(13, 6);
                    break;

                case OSD_ARTIFICIAL_HORIZON:
                    itemPos = (itemPos & (~OSD_POS_MAX)) | OSD_POS(14, 2);
                    break;

                case OSD_HORIZON_SIDEBARS:
                    itemPos = (itemPos & (~OSD_POS_MAX)) | OSD_POS(14, 5);
                    break;
            }

            // Enforce visibility in 3 BF OSD profiles
            if (OSD_VISIBLE(itemPos)) {
                itemPos |= 0x3000;
            }

            sbufWriteU16(dst, itemPos);
        }
        else {
            // Hide OSD elements unsupported by INAV
            sbufWriteU16(dst, 0);
        }
    }

    // Post flight statistics
    sbufWriteU8(dst, ARRAYLEN(djiOSDStatisticsMap));
    for (unsigned i = 0; i < ARRAYLEN(djiOSDStatisticsMap); i++ ) {
        if (djiOSDStatisticsMap[i] >= 0) {
            // FIXME: Map post-flight statistics from INAV to BF/DJI
            sbufWriteU8(dst, 0);
        }
        else {
            sbufWriteU8(dst, 0);
        }
    }

    // Timers
    sbufWriteU8(dst, DJI_OSD_TIMER_COUNT);
    for (int i = 0; i < DJI_OSD_TIMER_COUNT; i++) {
        // STUB: We don't support BF's OSD timers
        sbufWriteU16(dst, 0);
    }

    // Enabled warnings
    // API < 1.41 stub, kept for compatibility
    sbufWriteU16(dst, djiEncodeOSDEnabledWarnings() & 0xFFFF);

    // API >= 1.41
    // Send the warnings count and 32bit enabled warnings flags.
    sbufWriteU8(dst, DJI_OSD_WARNING_COUNT);
    sbufWriteU32(dst, djiEncodeOSDEnabledWarnings());

    // DJI OSD expects 1 OSD profile
    sbufWriteU8(dst, 1);
    sbufWriteU8(dst, 1);

    // No OSD stick overlay
    sbufWriteU8(dst, 0);

    // API >= 1.43
    // Camera frame element width/height - magic numbers taken from Betaflight source
    //sbufWriteU8(dst, DJI_OSD_SCREEN_WIDTH); // osdConfig()->camera_frame_width
    //sbufWriteU8(dst, DJI_OSD_SCREEN_HEIGHT); // osdConfig()->camera_frame_height
}

static mspResult_e djiProcessMspCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);

    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;

    // Start initializing the reply message
    reply->cmd = cmd->cmd;
    reply->result = MSP_RESULT_ACK;

    switch (cmd->cmd) {
        case DJI_MSP_API_VERSION:
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU8(dst, DJI_API_VERSION_MAJOR);
            sbufWriteU8(dst, DJI_API_VERSION_MINOR);
            break;

        case DJI_MSP_FC_VARIANT:
            {
                const char * const flightControllerIdentifier = INAV_IDENTIFIER;
                sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
            }
            break;

        case DJI_MSP_FC_VERSION:
            sbufWriteU8(dst, 4);
            sbufWriteU8(dst, 1);
            sbufWriteU8(dst, 0);
            break;

        case DJI_MSP_NAME:
            {
                const char * name = systemConfig()->name;
                int len = strlen(name);
                if (len > 12) len = 12;
                   sbufWriteData(dst, name, len);
                }
            break;

        case DJI_MSP_STATUS:
        case DJI_MSP_STATUS_EX:
            {
                // DJI OSD relies on a statically defined bit order and doesn't use MSP_BOXIDS
                // to get actual BOX order. We need a special packBoxModeFlags()
                boxBitmask_t flightModeBitmask;
                djiPackBoxModeBitmask(&flightModeBitmask);

                sbufWriteU16(dst, (uint16_t)cycleTime);
                sbufWriteU16(dst, 0);
                sbufWriteU16(dst, packSensorStatus());
                sbufWriteData(dst, &flightModeBitmask, 4);        // unconditional part of flags, first 32 bits
                sbufWriteU8(dst, getConfigProfile());

                sbufWriteU16(dst, schedulerGetAverageSystemLoadPercent());
                if (cmd->cmd == MSP_STATUS_EX) {
                    sbufWriteU8(dst, 3);            // PID_PROFILE_COUNT
                    sbufWriteU8(dst, 1);            // getCurrentControlRateProfileIndex()
                } else {
                    sbufWriteU16(dst, cycleTime);   // gyro cycle time
                }

                // Cap BoxModeFlags to 32 bits
                // write flightModeFlags header. Lowest 4 bits contain number of bytes that follow
                sbufWriteU8(dst, 0);
                // sbufWriteData(dst, ((uint8_t*)&flightModeBitmask) + 4, byteCount);

                // Write arming disable flags
                sbufWriteU8(dst, DJI_ARMING_DISABLE_FLAGS_COUNT);
                sbufWriteU32(dst, djiPackArmingDisabledFlags());

                // Extra flags
                sbufWriteU8(dst, 0);
            }
            break;

        case DJI_MSP_RC:
            // Only send sticks (first 4 channels)
            for (int i = 0; i < STICK_CHANNEL_COUNT; i++) {
                sbufWriteU16(dst, rxGetRawChannelValue(i));
            }
            break;            

        case DJI_MSP_RAW_GPS:
            sbufWriteU8(dst, gpsSol.fixType);
            sbufWriteU8(dst, gpsSol.numSat);
            sbufWriteU32(dst, gpsSol.llh.lat);
            sbufWriteU32(dst, gpsSol.llh.lon);
            sbufWriteU16(dst, gpsSol.llh.alt / 100);
            sbufWriteU16(dst, gpsSol.groundSpeed);
            sbufWriteU16(dst, gpsSol.groundCourse);
            break;

        case DJI_MSP_COMP_GPS:
            sbufWriteU16(dst, GPS_distanceToHome);
            sbufWriteU16(dst, GPS_directionToHome);
            sbufWriteU8(dst, gpsSol.flags.gpsHeartbeat ? 1 : 0);
            break;

        case DJI_MSP_ATTITUDE:
            sbufWriteU16(dst, attitude.values.roll);
            sbufWriteU16(dst, attitude.values.pitch);
            sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
            break;

        case DJI_MSP_ALTITUDE:
            sbufWriteU32(dst, lrintf(getEstimatedActualPosition(Z)));
            sbufWriteU16(dst, lrintf(getEstimatedActualVelocity(Z)));
            break;

        case DJI_MSP_ANALOG:
            sbufWriteU8(dst,  constrain(getBatteryVoltage() / 10, 0, 255));
            sbufWriteU16(dst, constrain(getMAhDrawn(), 0, 0xFFFF)); // milliamp hours drawn from battery
            sbufWriteU16(dst, getRSSI());
            sbufWriteU16(dst, constrain(getAmperage(), -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
            sbufWriteU16(dst, getBatteryVoltage());
            break;

        case DJI_MSP_PID:
            for (unsigned i = 0; i < ARRAYLEN(djiPidIndexMap); i++) {
                sbufWriteU8(dst, pidBank()->pid[djiPidIndexMap[i]].P);
                sbufWriteU8(dst, pidBank()->pid[djiPidIndexMap[i]].I);
                sbufWriteU8(dst, pidBank()->pid[djiPidIndexMap[i]].D);
            }
            break;

        case DJI_MSP_BATTERY_STATE:
            // Battery characteristics
            sbufWriteU8(dst, constrain(getBatteryCellCount(), 0, 255));
            sbufWriteU16(dst, currentBatteryProfile->capacity.value);

            // Battery state
            sbufWriteU8(dst, constrain(getBatteryVoltage() / 10, 0, 255)); // in 0.1V steps
            sbufWriteU16(dst, constrain(getMAhDrawn(), 0, 0xFFFF));
            sbufWriteU16(dst, constrain(getAmperage(), -0x8000, 0x7FFF));

            // Battery alerts - used values match Betaflight's/DJI's
            sbufWriteU8(dst,  getBatteryState());

            // Additional battery voltage field (in 0.01V steps)
            sbufWriteU16(dst, getBatteryVoltage());
            break;

        case DJI_MSP_RTC:
            {
                dateTime_t datetime;

                // We don't care about validity here - dt will be always set to a sane value
                // rtcGetDateTime() will call rtcGetDefaultDateTime() internally
                rtcGetDateTime(&datetime);

                sbufWriteU16(dst, datetime.year);
                sbufWriteU8(dst, datetime.month);
                sbufWriteU8(dst, datetime.day);
                sbufWriteU8(dst, datetime.hours);
                sbufWriteU8(dst, datetime.minutes);
                sbufWriteU8(dst, datetime.seconds);
                sbufWriteU16(dst, datetime.millis);
            }
            break;

#if defined(USE_ESC_SENSOR)
        case DJI_MSP_ESC_SENSOR_DATA:
            if (STATE(ESC_SENSOR_ENABLED)) {
                sbufWriteU8(dst, getMotorCount());
                for (int i = 0; i < getMotorCount(); i++) {
                    const escSensorData_t * escSensor = getEscTelemetry(i);
                    sbufWriteU8(dst, escSensor->temperature);
                    sbufWriteU16(dst, escSensor->rpm);
                }
            }
            else {
                reply->result = MSP_RESULT_ERROR;
            }
            break;
#endif

        case DJI_MSP_OSD_CONFIG:
#if defined(USE_OSD)
            // This involved some serious magic, better contain in a separate function for readability
            djiSerializeOSDConfigReply(dst);
#else
            sbufWriteU8(dst, 0);
#endif
            break;

        case DJI_MSP_FILTER_CONFIG:
            sbufWriteU8(dst, gyroConfig()->gyro_soft_lpf_hz);           // BF: gyroConfig()->gyro_lowpass_hz
            sbufWriteU16(dst, pidProfile()->dterm_lpf_hz);              // BF: currentPidProfile->dterm_lowpass_hz
            sbufWriteU16(dst, pidProfile()->yaw_lpf_hz);                // BF: currentPidProfile->yaw_lowpass_hz
            sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1);      // BF: gyroConfig()->gyro_soft_notch_hz_1
            sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1);  // BF: gyroConfig()->gyro_soft_notch_cutoff_1
            sbufWriteU16(dst, pidProfile()->dterm_soft_notch_hz);       // BF: currentPidProfile->dterm_notch_hz
            sbufWriteU16(dst, pidProfile()->dterm_soft_notch_cutoff);   // BF: currentPidProfile->dterm_notch_cutoff
            sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2);      // BF: gyroConfig()->gyro_soft_notch_hz_2
            sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2);  // BF: gyroConfig()->gyro_soft_notch_cutoff_2
            sbufWriteU8(dst, 0);                                        // BF: currentPidProfile->dterm_filter_type
            sbufWriteU8(dst, gyroConfig()->gyro_lpf);                   // BF: gyroConfig()->gyro_hardware_lpf);
            sbufWriteU8(dst, 0);                                        // BF: DEPRECATED: gyro_32khz_hardware_lpf
            sbufWriteU16(dst, gyroConfig()->gyro_soft_lpf_hz);          // BF: gyroConfig()->gyro_lowpass_hz);
            sbufWriteU16(dst, gyroConfig()->gyro_stage2_lowpass_hz);    // BF: gyroConfig()->gyro_lowpass2_hz);
            sbufWriteU8(dst, 0);                                        // BF: gyroConfig()->gyro_lowpass_type);
            sbufWriteU8(dst, 0);                                        // BF: gyroConfig()->gyro_lowpass2_type);
            sbufWriteU16(dst, 0);                                       // BF: currentPidProfile->dterm_lowpass2_hz);
            sbufWriteU8(dst, 0);                                        // BF: currentPidProfile->dterm_filter2_type);
            break;

        case DJI_MSP_RC_TUNING:
            sbufWriteU8(dst, 100);                                      // INAV doesn't use rcRate
            sbufWriteU8(dst, currentControlRateProfile->stabilized.rcExpo8);
            for (int i = 0 ; i < 3; i++) {
                // R,P,Y rates see flight_dynamics_index_t
                sbufWriteU8(dst, currentControlRateProfile->stabilized.rates[i]);
            }
            sbufWriteU8(dst, currentControlRateProfile->throttle.dynPID);
            sbufWriteU8(dst, currentControlRateProfile->throttle.rcMid8);
            sbufWriteU8(dst, currentControlRateProfile->throttle.rcExpo8);
            sbufWriteU16(dst, currentControlRateProfile->throttle.pa_breakpoint);
            sbufWriteU8(dst, currentControlRateProfile->stabilized.rcYawExpo8);
            sbufWriteU8(dst, 100);                                      // INAV doesn't use rcRate
            sbufWriteU8(dst, 100);                                      // INAV doesn't use rcRate
            sbufWriteU8(dst, currentControlRateProfile->stabilized.rcExpo8);

            // added in 1.41
            sbufWriteU8(dst, 0);
            sbufWriteU8(dst, currentControlRateProfile->throttle.dynPID);
            break;

        case DJI_MSP_SET_PID:
            // Check if we have enough data for all PID coefficients
            if ((unsigned)sbufBytesRemaining(src) >= ARRAYLEN(djiPidIndexMap) * 3) {
                for (unsigned i = 0; i < ARRAYLEN(djiPidIndexMap); i++) {
                    pidBankMutable()->pid[djiPidIndexMap[i]].P = sbufReadU8(src);
                    pidBankMutable()->pid[djiPidIndexMap[i]].I = sbufReadU8(src);
                    pidBankMutable()->pid[djiPidIndexMap[i]].D = sbufReadU8(src);
                }
                schedulePidGainsUpdate();
#if defined(USE_NAV)
                // This is currently unnecessary, DJI HD doesn't set any NAV PIDs
                //navigationUsePIDs();
#endif
            }
            else {
                reply->result = MSP_RESULT_ERROR;
            }
            break;

        case DJI_MSP_PID_ADVANCED:
            // TODO
            reply->result = MSP_RESULT_ERROR;
            break;

        case DJI_MSP_SET_FILTER_CONFIG:
        case DJI_MSP_SET_PID_ADVANCED:
        case DJI_MSP_SET_RC_TUNING:
            // TODO
            reply->result = MSP_RESULT_ERROR;
            break;

        default:
            // debug[1]++;
            // debug[2] = cmd->cmd;
            reply->result = MSP_RESULT_ERROR;
            break;
    }

    // Process DONT_REPLY flag
    if (cmd->flags & MSP_FLAG_DONT_REPLY) {
        reply->result = MSP_RESULT_NO_REPLY;
    }

    return reply->result;
}

void djiOsdSerialProcess(void)
{
    // Check if DJI OSD is configured
    if (!djiMspPort.port) {
        return;
    }

    // Piggyback on existing MSP protocol stack, but pass our special command processing function
    mspSerialProcessOnePort(&djiMspPort, MSP_SKIP_NON_MSP_DATA, djiProcessMspCommand);
}

#endif
