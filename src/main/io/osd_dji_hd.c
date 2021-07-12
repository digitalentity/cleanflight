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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_msp.h"
#include "fc/fc_msp_box.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "fc/rc_adjustments.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/mixer.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/osd_dji_hd.h"
#include "io/osd_common.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"
#include "sensors/esc_sensor.h"
#include "sensors/temperature.h"
#include "sensors/pitotmeter.h"
#include "sensors/boardalignment.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "common/string_light.h"
#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "common/constants.h"
#include "scheduler/scheduler.h"
#include "common/printf.h"
#include <stdlib.h>
#include "rx/rx.h"
#include "fc/rc_controls.h"

#if defined(USE_DJI_HD_OSD)

#define DJI_MSP_BAUDRATE                    115200

#define DJI_ARMING_DISABLE_FLAGS_COUNT      25
#define DJI_OSD_WARNING_COUNT               16
#define DJI_OSD_TIMER_COUNT                 2
#define DJI_OSD_FLAGS_OSD_FEATURE           (1 << 0)
#define EFFICIENCY_UPDATE_INTERVAL          (5 * 1000)

#define RC_RX_LINK_LOST_MSG "!RC RX LINK LOST!"

// Adjust OSD_MESSAGE's default position when
// changing OSD_MESSAGE_LENGTH
#define OSD_MESSAGE_LENGTH 28

#define OSD_ALTERNATING_CHOICES(ms, num_choices) ((millis() / ms) % num_choices)
#define _CONST_STR_SIZE(s) ((sizeof(s)/sizeof(s[0]))-1) // -1 to avoid counting final '\0'

// Wrap all string constants intenteded for display as messages with
// this macro to ensure compile time length validation.
#define OSD_MESSAGE_STR(x) ({ \
    STATIC_ASSERT(_CONST_STR_SIZE(x) <= OSD_MESSAGE_LENGTH, message_string_ ## __COUNTER__ ## _too_long); \
    x; \
})


/*
 * DJI HD goggles use MSPv1 compatible with Betaflight 4.1.0
 * DJI uses a subset of messages and assume fixed bit positions for flight modes
 *
 * To avoid compatibility issues we maintain a separate MSP command processor
 * but reuse the packet decoder to minimize code duplication
 */

PG_REGISTER_WITH_RESET_TEMPLATE(djiOsdConfig_t, djiOsdConfig, PG_DJI_OSD_CONFIG, 2);
PG_RESET_TEMPLATE(djiOsdConfig_t, djiOsdConfig,
    .use_name_for_messages  = SETTING_DJI_USE_NAME_FOR_MESSAGES_DEFAULT,
    .esc_temperature_source = SETTING_DJI_ESC_TEMP_SOURCE_DEFAULT,
    .proto_workarounds = SETTING_DJI_WORKAROUNDS_DEFAULT,
    .messageSpeedSource = SETTING_DJI_MESSAGE_SPEED_SOURCE_DEFAULT,
    .rssi_source = SETTING_DJI_RSSI_SOURCE_DEFAULT,
    .useAdjustments = SETTING_DJI_USE_ADJUSTMENTS_DEFAULT,
    .craftNameAlternatingDuration = SETTING_DJI_CN_ALTERNATING_DURATION_DEFAULT
);

#define RSSI_BOUNDARY(PERCENT) (RSSI_MAX_VALUE / 100 * PERCENT)

typedef enum {
    DJI_OSD_CN_MESSAGES,
    DJI_OSD_CN_THROTTLE,
    DJI_OSD_CN_THROTTLE_AUTO_THR,
    DJI_OSD_CN_AIR_SPEED,
    DJI_OSD_CN_EFFICIENCY,
    DJI_OSD_CN_DISTANCE,
    DJI_OSD_CN_ADJUSTEMNTS,
    DJI_OSD_CN_MAX_ELEMENTS
} DjiCraftNameElements_t;

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

typedef struct {
    int         itemIndex;      // INAV OSD item
    features_e  depFeature;     // INAV feature that item is dependent on
} djiOsdMapping_t;

const djiOsdMapping_t djiOSDItemIndexMap[] = {
    { OSD_RSSI_VALUE,                         0 }, // DJI: OSD_RSSI_VALUE
    { OSD_MAIN_BATT_VOLTAGE,                  FEATURE_VBAT }, // DJI: OSD_MAIN_BATT_VOLTAGE
    { OSD_CROSSHAIRS,                         0 }, // DJI: OSD_CROSSHAIRS
    { OSD_ARTIFICIAL_HORIZON,                 0 }, // DJI: OSD_ARTIFICIAL_HORIZON
    { OSD_HORIZON_SIDEBARS,                   0 }, // DJI: OSD_HORIZON_SIDEBARS
    { OSD_ONTIME,                             0 }, // DJI: OSD_ITEM_TIMER_1
    { OSD_FLYTIME,                            0 }, // DJI: OSD_ITEM_TIMER_2
    { OSD_FLYMODE,                            0 }, // DJI: OSD_FLYMODE
    { OSD_CRAFT_NAME,                         0 }, // DJI: OSD_CRAFT_NAME
    { OSD_THROTTLE_POS,                       0 }, // DJI: OSD_THROTTLE_POS
    { OSD_VTX_CHANNEL,                        0 }, // DJI: OSD_VTX_CHANNEL
    { OSD_CURRENT_DRAW,                       FEATURE_CURRENT_METER }, // DJI: OSD_CURRENT_DRAW
    { OSD_MAH_DRAWN,                          FEATURE_CURRENT_METER }, // DJI: OSD_MAH_DRAWN
    { OSD_GPS_SPEED,                          FEATURE_GPS }, // DJI: OSD_GPS_SPEED
    { OSD_GPS_SATS,                           FEATURE_GPS }, // DJI: OSD_GPS_SATS
    { OSD_ALTITUDE,                           0 }, // DJI: OSD_ALTITUDE
    { OSD_ROLL_PIDS,                          0 }, // DJI: OSD_ROLL_PIDS
    { OSD_PITCH_PIDS,                         0 }, // DJI: OSD_PITCH_PIDS
    { OSD_YAW_PIDS,                           0 }, // DJI: OSD_YAW_PIDS
    { OSD_POWER,                              0 }, // DJI: OSD_POWER
    { -1,                                     0 }, // DJI: OSD_PIDRATE_PROFILE
    { -1,                                     0 }, // DJI: OSD_WARNINGS
    { OSD_MAIN_BATT_CELL_VOLTAGE,             0 }, // DJI: OSD_AVG_CELL_VOLTAGE
    { OSD_GPS_LON,                            FEATURE_GPS }, // DJI: OSD_GPS_LON
    { OSD_GPS_LAT,                            FEATURE_GPS }, // DJI: OSD_GPS_LAT
    { OSD_DEBUG,                              0 }, // DJI: OSD_DEBUG
    { OSD_ATTITUDE_PITCH,                     0 }, // DJI: OSD_PITCH_ANGLE
    { OSD_ATTITUDE_ROLL,                      0 }, // DJI: OSD_ROLL_ANGLE
    { -1,                                     0 }, // DJI: OSD_MAIN_BATT_USAGE
    { -1,                                     0 }, // DJI: OSD_DISARMED
    { OSD_HOME_DIR,                           FEATURE_GPS }, // DJI: OSD_HOME_DIR
    { OSD_HOME_DIST,                          FEATURE_GPS }, // DJI: OSD_HOME_DIST
    { OSD_HEADING,                            0 }, // DJI: OSD_NUMERICAL_HEADING
    { OSD_VARIO_NUM,                          0 }, // DJI: OSD_NUMERICAL_VARIO
    { -1,                                     0 }, // DJI: OSD_COMPASS_BAR
    { OSD_ESC_TEMPERATURE,                    0 }, // DJI: OSD_ESC_TEMPERATURE
    { OSD_ESC_RPM,                            0 }, // DJI: OSD_ESC_RPM
    { OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH,   FEATURE_CURRENT_METER }, // DJI: OSD_REMAINING_TIME_ESTIMATE
    { OSD_RTC_TIME,                           0 }, // DJI: OSD_RTC_DATETIME
    { -1,                                     0 }, // DJI: OSD_ADJUSTMENT_RANGE
    { -1,                                     0 }, // DJI: OSD_CORE_TEMPERATURE
    { -1,                                     0 }, // DJI: OSD_ANTI_GRAVITY
    { -1,                                     0 }, // DJI: OSD_G_FORCE
    { -1,                                     0 }, // DJI: OSD_MOTOR_DIAG
    { -1,                                     0 }, // DJI: OSD_LOG_STATUS
    { -1,                                     0 }, // DJI: OSD_FLIP_ARROW
    { -1,                                     0 }, // DJI: OSD_LINK_QUALITY
    { OSD_TRIP_DIST,                          FEATURE_GPS }, // DJI: OSD_FLIGHT_DIST
    { -1,                                     0 }, // DJI: OSD_STICK_OVERLAY_LEFT
    { -1,                                     0 }, // DJI: OSD_STICK_OVERLAY_RIGHT
    { -1,                                     0 }, // DJI: OSD_DISPLAY_NAME
    { -1,                                     0 }, // DJI: OSD_ESC_RPM_FREQ
    { -1,                                     0 }, // DJI: OSD_RATE_PROFILE_NAME
    { -1,                                     0 }, // DJI: OSD_PID_PROFILE_NAME
    { -1,                                     0 }, // DJI: OSD_PROFILE_NAME
    { -1,                                     0 }, // DJI: OSD_RSSI_DBM_VALUE
    { -1,                                     0 }, // DJI: OSD_RC_CHANNELS
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

#if defined(USE_OSD)
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
        const int inavOSDIdx = djiOSDItemIndexMap[i].itemIndex;

        // We call OSD item supported if it doesn't have dependencies or all feature-dependencies are satistied
        const bool itemIsSupported = ((djiOSDItemIndexMap[i].depFeature == 0) || feature(djiOSDItemIndexMap[i].depFeature));

        if (inavOSDIdx >= 0 && itemIsSupported) {
            // Position & visibility encoded in 16 bits. Position encoding is the same between BF/DJI and INAV
            // However visibility is different. INAV has 3 layouts, while BF only has visibility profiles
            // Here we use only one OSD layout mapped to first OSD BF profile
            uint16_t itemPos = osdLayoutsConfig()->item_pos[0][inavOSDIdx];

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

static char * osdArmingDisabledReasonMessage(void)
{
    switch (isArmingDisabledReason()) {
        case ARMING_DISABLED_FAILSAFE_SYSTEM:
            // See handling of FAILSAFE_RX_LOSS_MONITORING in failsafe.c
            if (failsafePhase() == FAILSAFE_RX_LOSS_MONITORING) {
                if (failsafeIsReceivingRxData()) {
                    // If we're not using sticks, it means the ARM switch
                    // hasn't been off since entering FAILSAFE_RX_LOSS_MONITORING
                    // yet
                    return OSD_MESSAGE_STR("DISARM!");
                }
                // Not receiving RX data
                return OSD_MESSAGE_STR(RC_RX_LINK_LOST_MSG);
            }
            return OSD_MESSAGE_STR("FAILSAFE");
        case ARMING_DISABLED_NOT_LEVEL:
            return OSD_MESSAGE_STR("!LEVEL");
        case ARMING_DISABLED_SENSORS_CALIBRATING:
            return OSD_MESSAGE_STR("CALIBRATING");
        case ARMING_DISABLED_SYSTEM_OVERLOADED:
            return OSD_MESSAGE_STR("OVERLOAD");
        case ARMING_DISABLED_NAVIGATION_UNSAFE:
            // Check the exact reason
            switch (navigationIsBlockingArming(NULL)) {
                case NAV_ARMING_BLOCKER_NONE:
                    break;
                case NAV_ARMING_BLOCKER_MISSING_GPS_FIX:
                    return OSD_MESSAGE_STR("NO GPS FIX");
                case NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE:
                    return OSD_MESSAGE_STR("DISABLE NAV");
                case NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR:
                    return OSD_MESSAGE_STR("1ST WYP TOO FAR");
                case NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR:
                    return OSD_MESSAGE_STR("WYP MISCONFIGURED");
            }
            break;
        case ARMING_DISABLED_COMPASS_NOT_CALIBRATED:
            return OSD_MESSAGE_STR("COMPS CALIB");
        case ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED:
            return OSD_MESSAGE_STR("ACC CALIB");
        case ARMING_DISABLED_ARM_SWITCH:
            return OSD_MESSAGE_STR("DISARM!");
        case ARMING_DISABLED_HARDWARE_FAILURE:
            return OSD_MESSAGE_STR("ERR HW!");
        //     {
        //         if (!HW_SENSOR_IS_HEALTHY(getHwGyroStatus())) {
        //             return OSD_MESSAGE_STR("GYRO FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwAccelerometerStatus())) {
        //             return OSD_MESSAGE_STR("ACCELEROMETER FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwCompassStatus())) {
        //             return OSD_MESSAGE_STR("COMPASS FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwBarometerStatus())) {
        //             return OSD_MESSAGE_STR("BAROMETER FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwGPSStatus())) {
        //             return OSD_MESSAGE_STR("GPS FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwRangefinderStatus())) {
        //             return OSD_MESSAGE_STR("RANGE FINDER FAILURE");
        //         }
        //         if (!HW_SENSOR_IS_HEALTHY(getHwPitotmeterStatus())) {
        //             return OSD_MESSAGE_STR("PITOT METER FAILURE");
        //         }
        //     }
        //     return OSD_MESSAGE_STR("HARDWARE FAILURE");
        case ARMING_DISABLED_BOXFAILSAFE:
            return OSD_MESSAGE_STR("FAILSAFE ENABLED");
        case ARMING_DISABLED_BOXKILLSWITCH:
            return OSD_MESSAGE_STR("KILLSWITCH ENABLED");
        case ARMING_DISABLED_RC_LINK:
            return OSD_MESSAGE_STR("NO RC LINK");
        case ARMING_DISABLED_THROTTLE:
            return OSD_MESSAGE_STR("THROTTLE!");
        case ARMING_DISABLED_ROLLPITCH_NOT_CENTERED:
            return OSD_MESSAGE_STR("ROLLPITCH!");
        case ARMING_DISABLED_SERVO_AUTOTRIM:
            return OSD_MESSAGE_STR("AUTOTRIM!");
        case ARMING_DISABLED_OOM:
            return OSD_MESSAGE_STR("MEM LOW");
        case ARMING_DISABLED_INVALID_SETTING:
            return OSD_MESSAGE_STR("ERR SETTING");
        case ARMING_DISABLED_CLI:
            return OSD_MESSAGE_STR("CLI");
        case ARMING_DISABLED_PWM_OUTPUT_ERROR:
            return OSD_MESSAGE_STR("PWM ERR");
        case ARMING_DISABLED_NO_PREARM:
            return OSD_MESSAGE_STR("NO PREARM");
        case ARMING_DISABLED_DSHOT_BEEPER:
            return OSD_MESSAGE_STR("MOTOR BEEPER ACTIVE");
            // Cases without message
        case ARMING_DISABLED_CMS_MENU:
            FALLTHROUGH;
        case ARMING_DISABLED_OSD_MENU:
            FALLTHROUGH;
        case ARMING_DISABLED_ALL_FLAGS:
            FALLTHROUGH;
        case ARMED:
            FALLTHROUGH;
        case WAS_EVER_ARMED:
            break;
    }

    return NULL;
}

static char * osdFailsafePhaseMessage(void)
{
    // See failsafe.h for each phase explanation
    switch (failsafePhase()) {
        case FAILSAFE_RETURN_TO_HOME:
            // XXX: Keep this in sync with OSD_FLYMODE.
            return OSD_MESSAGE_STR("(RTH)");
        case FAILSAFE_LANDING:
            // This should be considered an emergengy landing
            return OSD_MESSAGE_STR("(EMRGY LANDING)");
        case FAILSAFE_RX_LOSS_MONITORING:
            // Only reachable from FAILSAFE_LANDED, which performs
            // a disarm. Since aircraft has been disarmed, we no
            // longer show failsafe details.
            FALLTHROUGH;
        case FAILSAFE_LANDED:
            // Very brief, disarms and transitions into
            // FAILSAFE_RX_LOSS_MONITORING. Note that it prevents
            // further rearming via ARMING_DISABLED_FAILSAFE_SYSTEM,
            // so we'll show the user how to re-arm in when
            // that flag is the reason to prevent arming.
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_IDLE:
            // This only happens when user has chosen NONE as FS
            // procedure. The recovery messages should be enough.
            FALLTHROUGH;
        case FAILSAFE_IDLE:
            // Failsafe not active
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_DETECTED:
            // Very brief, changes to FAILSAFE_RX_LOSS_RECOVERED
            // or the FS procedure immediately.
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_RECOVERED:
            // Exiting failsafe
            break;
    }

    return NULL;
}

static char * osdFailsafeInfoMessage(void)
{
    if (failsafeIsReceivingRxData()) {
        // User must move sticks to exit FS mode
        return OSD_MESSAGE_STR("!MOVE STICKS TO EXIT FS!");
    }

    return OSD_MESSAGE_STR(RC_RX_LINK_LOST_MSG);
}

static char * navigationStateMessage(void)
{
    switch (NAV_Status.state) {
        case MW_NAV_STATE_NONE:
            break;
        case MW_NAV_STATE_RTH_START:
            return OSD_MESSAGE_STR("STARTING RTH");
        case MW_NAV_STATE_RTH_CLIMB:
            return OSD_MESSAGE_STR("ADJUSTING RTH ALTITUDE");
        case MW_NAV_STATE_RTH_ENROUTE:
            // TODO: Break this up between climb and head home
            return OSD_MESSAGE_STR("EN ROUTE TO HOME");
        case MW_NAV_STATE_HOLD_INFINIT:
            // Used by HOLD flight modes. No information to add.
            break;
        case MW_NAV_STATE_HOLD_TIMED:
            // TODO: Maybe we can display a count down
            return OSD_MESSAGE_STR("HOLDING WAYPOINT");
            break;
        case MW_NAV_STATE_WP_ENROUTE:
            // TODO: Show WP number
            return OSD_MESSAGE_STR("TO WP");
        case MW_NAV_STATE_PROCESS_NEXT:
            return OSD_MESSAGE_STR("PREPARING FOR NEXT WAYPOINT");
        case MW_NAV_STATE_DO_JUMP:
            // Not used
            break;
        case MW_NAV_STATE_LAND_START:
            // Not used
            break;
        case MW_NAV_STATE_EMERGENCY_LANDING:
            return OSD_MESSAGE_STR("EMRGY LANDING");
        case MW_NAV_STATE_LAND_IN_PROGRESS:
            return OSD_MESSAGE_STR("LANDING");
        case MW_NAV_STATE_HOVER_ABOVE_HOME:
            if (STATE(FIXED_WING_LEGACY)) {
                return OSD_MESSAGE_STR("LOITERING AROUND HOME");
            }
            return OSD_MESSAGE_STR("HOVERING");
        case MW_NAV_STATE_LANDED:
            return OSD_MESSAGE_STR("LANDED");
        case MW_NAV_STATE_LAND_SETTLE:
            return OSD_MESSAGE_STR("PREPARING TO LAND");
        case MW_NAV_STATE_LAND_START_DESCENT:
            // Not used
            break;
    }
    return NULL;
}


/**
 * Converts velocity based on the current unit system (kmh or mph).
 * @param alt Raw velocity (i.e. as taken from gpsSol.groundSpeed in centimeters/second)
 */
static int32_t osdConvertVelocityToUnit(int32_t vel)
{
    switch (osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            return (vel * 224) / 10000; // Convert to mph
        case OSD_UNIT_GA:
            return (vel * 0.019438444924406); // Convert to Knots
        case OSD_UNIT_METRIC:
            return (vel * 36) / 1000;   // Convert to kmh
    }

    // Unreachable
    return -1;
}

/**
 * Converts velocity into a string based on the current unit system.
 * @param alt Raw velocity (i.e. as taken from gpsSol.groundSpeed in centimeters/seconds)
 */
void osdDJIFormatVelocityStr(char* buff)
{
    char sourceBuf[4];
    int vel = 0;
    switch (djiOsdConfig()->messageSpeedSource) {
        case OSD_SPEED_SOURCE_GROUND:
            strcpy(sourceBuf, "GRD");
            vel = gpsSol.groundSpeed;
            break;
        case OSD_SPEED_SOURCE_3D:
            strcpy(sourceBuf, "3D");
            vel = osdGet3DSpeed();
            break;
        case OSD_SPEED_SOURCE_AIR:
            strcpy(sourceBuf, "AIR");
#ifdef USE_PITOT
            vel = pitot.airSpeed;
#endif
            break;
    }

    switch (osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            tfp_sprintf(buff, "%s %3d MPH", sourceBuf, (int)osdConvertVelocityToUnit(vel));
            break;
        case OSD_UNIT_GA:
            tfp_sprintf(buff, "%s %3d KT", sourceBuf, (int)osdConvertVelocityToUnit(vel));
            break;
        case OSD_UNIT_METRIC:
            tfp_sprintf(buff, "%s %3d KPH", sourceBuf, (int)osdConvertVelocityToUnit(vel));
            break;
    }
}
static void osdDJIFormatThrottlePosition(char *buff, bool autoThr )
{
    int16_t thr = rxGetChannelValue(THROTTLE);
    if (autoThr && navigationIsControllingThrottle()) {
        thr = rcCommand[THROTTLE];
    }

    tfp_sprintf(buff, "%3ld%s", (constrain(thr, PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN), "%THR");
}

/**
 * Converts distance into a string based on the current unit system.
 * @param dist Distance in centimeters
 */
static void osdDJIFormatDistanceStr(char *buff, int32_t dist)
{
    int32_t centifeet;

    switch (osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            centifeet = CENTIMETERS_TO_CENTIFEET(dist);
            if (abs(centifeet) < FEET_PER_MILE * 100 / 2) {
                // Show feet when dist < 0.5mi
                tfp_sprintf(buff, "%d%s", (int)(centifeet / 100), "FT");
            }
            else {
                // Show miles when dist >= 0.5mi
                tfp_sprintf(buff, "%d.%02d%s", (int)(centifeet / (100*FEET_PER_MILE)),
                (abs(centifeet) % (100 * FEET_PER_MILE)) / FEET_PER_MILE, "Mi");
            }
            break;
        case OSD_UNIT_GA:
            centifeet = CENTIMETERS_TO_CENTIFEET(dist);
            if (abs(centifeet) < FEET_PER_NAUTICALMILE * 100 / 2) {
                // Show feet when dist < 0.5mi
                tfp_sprintf(buff, "%d%s", (int)(centifeet / 100), "FT");
            }
            else {
                // Show miles when dist >= 0.5mi
                tfp_sprintf(buff, "%d.%02d%s", (int)(centifeet / (100 * FEET_PER_NAUTICALMILE)),
                (int)((abs(centifeet) % (int)(100 * FEET_PER_NAUTICALMILE)) / FEET_PER_NAUTICALMILE), "NM");
            }
            break;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            if (abs(dist) < METERS_PER_KILOMETER * 100) {
                // Show meters when dist < 1km
                tfp_sprintf(buff, "%d%s", (int)(dist / 100), "M");
            }
            else {
                // Show kilometers when dist >= 1km
                tfp_sprintf(buff, "%d.%02d%s", (int)(dist / (100*METERS_PER_KILOMETER)),
                    (abs(dist) % (100 * METERS_PER_KILOMETER)) / METERS_PER_KILOMETER, "KM");
            }
            break;
    }
}

static void osdDJIEfficiencyMahPerKM(char *buff)
{
    // amperage is in centi amps, speed is in cms/s. We want
    // mah/km. Values over 999 are considered useless and
    // displayed as "---""
    static pt1Filter_t eFilterState;
    static timeUs_t efficiencyUpdated = 0;
    int32_t value = 0;
    timeUs_t currentTimeUs = micros();
    timeDelta_t efficiencyTimeDelta = cmpTimeUs(currentTimeUs, efficiencyUpdated);

    if (STATE(GPS_FIX) && gpsSol.groundSpeed > 0) {
        if (efficiencyTimeDelta >= EFFICIENCY_UPDATE_INTERVAL) {
            value = pt1FilterApply4(&eFilterState, ((float)getAmperage() / gpsSol.groundSpeed) / 0.0036f,
                1, US2S(efficiencyTimeDelta));

            efficiencyUpdated = currentTimeUs;
        } else {
            value = eFilterState.state;
        }
    }

    if (value > 0 && value <= 999) {
        tfp_sprintf(buff, "%3d%s", (int)value, "mAhKM");
    } else {
        tfp_sprintf(buff, "%s", "---mAhKM");
    }
}

static void osdDJIAdjustmentMessage(char *buff, uint8_t adjustmentFunction)
{
    switch (adjustmentFunction) {
        case ADJUSTMENT_RC_EXPO:
            tfp_sprintf(buff, "RCE %d", currentControlRateProfile->stabilized.rcExpo8);
            break;
        case ADJUSTMENT_RC_YAW_EXPO:
            tfp_sprintf(buff, "RCYE %3d", currentControlRateProfile->stabilized.rcYawExpo8);
            break;
        case ADJUSTMENT_MANUAL_RC_EXPO:
            tfp_sprintf(buff, "MRCE %3d", currentControlRateProfile->manual.rcExpo8);
            break;
        case ADJUSTMENT_MANUAL_RC_YAW_EXPO:
            tfp_sprintf(buff, "MRCYE %3d", currentControlRateProfile->manual.rcYawExpo8);
            break;
        case ADJUSTMENT_THROTTLE_EXPO:
            tfp_sprintf(buff, "TE %3d", currentControlRateProfile->throttle.rcExpo8);
            break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
            tfp_sprintf(buff, "PRR %3d %3d", currentControlRateProfile->stabilized.rates[FD_PITCH], currentControlRateProfile->stabilized.rates[FD_ROLL]);
            break;
        case ADJUSTMENT_PITCH_RATE:
            tfp_sprintf(buff, "PR %3d", currentControlRateProfile->stabilized.rates[FD_PITCH]);
            break;
        case ADJUSTMENT_ROLL_RATE:
            tfp_sprintf(buff, "RR %3d", currentControlRateProfile->stabilized.rates[FD_ROLL]);
            break;
        case ADJUSTMENT_MANUAL_PITCH_ROLL_RATE:
            tfp_sprintf(buff, "MPRR %3d %3d", currentControlRateProfile->manual.rates[FD_PITCH], currentControlRateProfile->manual.rates[FD_ROLL]);
            break;
        case ADJUSTMENT_MANUAL_PITCH_RATE:
            tfp_sprintf(buff, "MPR %3d", currentControlRateProfile->manual.rates[FD_PITCH]);
            break;
        case ADJUSTMENT_MANUAL_ROLL_RATE:
            tfp_sprintf(buff, "MRR %3d", currentControlRateProfile->manual.rates[FD_ROLL]);
            break;
        case ADJUSTMENT_YAW_RATE:
            tfp_sprintf(buff, "YR %3d", currentControlRateProfile->stabilized.rates[FD_YAW]);
            break;
        case ADJUSTMENT_MANUAL_YAW_RATE:
            tfp_sprintf(buff, "MYR %3d", currentControlRateProfile->manual.rates[FD_YAW]);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
            tfp_sprintf(buff, "PRP %3d %3d", pidBankMutable()->pid[PID_PITCH].P, pidBankMutable()->pid[PID_ROLL].P);
            break;
        case ADJUSTMENT_PITCH_P:
            tfp_sprintf(buff, "PP %3d", pidBankMutable()->pid[PID_PITCH].P);
            break;
        case ADJUSTMENT_ROLL_P:
            tfp_sprintf(buff, "RP %3d", pidBankMutable()->pid[PID_ROLL].P);
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
            tfp_sprintf(buff, "PRI %3d %3d", pidBankMutable()->pid[PID_PITCH].I, pidBankMutable()->pid[PID_ROLL].I);
            break;
        case ADJUSTMENT_PITCH_I:
            tfp_sprintf(buff, "PI %3d", pidBankMutable()->pid[PID_PITCH].I);
            break;
        case ADJUSTMENT_ROLL_I:
            tfp_sprintf(buff, "RI %3d", pidBankMutable()->pid[PID_ROLL].I);
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
            tfp_sprintf(buff, "PRD %3d %3d", pidBankMutable()->pid[PID_PITCH].D, pidBankMutable()->pid[PID_ROLL].D);
            break;
        case ADJUSTMENT_PITCH_ROLL_FF:
            tfp_sprintf(buff, "PRFF %3d %3d", pidBankMutable()->pid[PID_PITCH].FF, pidBankMutable()->pid[PID_ROLL].FF);
            break;
        case ADJUSTMENT_PITCH_D:
            tfp_sprintf(buff, "PD %3d", pidBankMutable()->pid[PID_PITCH].D);
            break;
        case ADJUSTMENT_PITCH_FF:
            tfp_sprintf(buff, "PFF %3d", pidBankMutable()->pid[PID_PITCH].FF);
            break;
        case ADJUSTMENT_ROLL_D:
            tfp_sprintf(buff, "RD %3d", pidBankMutable()->pid[PID_ROLL].D);
            break;
        case ADJUSTMENT_ROLL_FF:
            tfp_sprintf(buff, "RFF %3d", pidBankMutable()->pid[PID_ROLL].FF);
            break;
        case ADJUSTMENT_YAW_P:
            tfp_sprintf(buff, "YP %3d", pidBankMutable()->pid[PID_YAW].P);
            break;
        case ADJUSTMENT_YAW_I:
            tfp_sprintf(buff, "YI  %3d", pidBankMutable()->pid[PID_YAW].I);
            break;
        case ADJUSTMENT_YAW_D:
            tfp_sprintf(buff, "YD %3d", pidBankMutable()->pid[PID_YAW].D);
            break;
        case ADJUSTMENT_YAW_FF:
            tfp_sprintf(buff, "YFF %3d", pidBankMutable()->pid[PID_YAW].FF);
            break;
        case ADJUSTMENT_NAV_FW_CRUISE_THR:
            tfp_sprintf(buff, "CR %4d", currentBatteryProfileMutable->nav.fw.cruise_throttle);
            break;
        case ADJUSTMENT_NAV_FW_PITCH2THR:
            tfp_sprintf(buff, "P2T %3d", currentBatteryProfileMutable->nav.fw.pitch_to_throttle);
            break;
        case ADJUSTMENT_ROLL_BOARD_ALIGNMENT:
            tfp_sprintf(buff, "RBA %3d", boardAlignment()->rollDeciDegrees);
            break;
        case ADJUSTMENT_PITCH_BOARD_ALIGNMENT:
            tfp_sprintf(buff, "PBA %3d", boardAlignment()->pitchDeciDegrees);
            break;
        case ADJUSTMENT_LEVEL_P:
            tfp_sprintf(buff, "LP %3d", pidBankMutable()->pid[PID_LEVEL].P);
            break;
        case ADJUSTMENT_LEVEL_I:
            tfp_sprintf(buff, "LI %3d", pidBankMutable()->pid[PID_LEVEL].I);
            break;
        case ADJUSTMENT_LEVEL_D:
            tfp_sprintf(buff, "LD %3d", pidBankMutable()->pid[PID_LEVEL].D);
            break;
        case ADJUSTMENT_POS_XY_P:
            tfp_sprintf(buff, "PXYP %3d", pidBankMutable()->pid[PID_POS_XY].P);
            break;
        case ADJUSTMENT_POS_XY_I:
            tfp_sprintf(buff, "PXYI %3d", pidBankMutable()->pid[PID_POS_XY].I);
            break;
        case ADJUSTMENT_POS_XY_D:
            tfp_sprintf(buff, "PXYD %3d", pidBankMutable()->pid[PID_POS_XY].D);
            break;
        case ADJUSTMENT_POS_Z_P:
            tfp_sprintf(buff, "PZP %3d", pidBankMutable()->pid[PID_POS_Z].P);
            break;
        case ADJUSTMENT_POS_Z_I:
            tfp_sprintf(buff, "PZI %3d", pidBankMutable()->pid[PID_POS_Z].I);
            break;
        case ADJUSTMENT_POS_Z_D:
            tfp_sprintf(buff, "PZD %3d", pidBankMutable()->pid[PID_POS_Z].D);
            break;
        case ADJUSTMENT_HEADING_P:
            tfp_sprintf(buff, "HP %3d", pidBankMutable()->pid[PID_HEADING].P);
            break;
        case ADJUSTMENT_VEL_XY_P:
            tfp_sprintf(buff, "VXYP %3d", pidBankMutable()->pid[PID_VEL_XY].P);
            break;
        case ADJUSTMENT_VEL_XY_I:
            tfp_sprintf(buff, "VXYI %3d", pidBankMutable()->pid[PID_VEL_XY].I);
            break;
        case ADJUSTMENT_VEL_XY_D:
            tfp_sprintf(buff, "VXYD %3d", pidBankMutable()->pid[PID_VEL_XY].D);
            break;
        case ADJUSTMENT_VEL_Z_P:
            tfp_sprintf(buff, "VZP %3d", pidBankMutable()->pid[PID_VEL_Z].P);
            break;
        case ADJUSTMENT_VEL_Z_I:
            tfp_sprintf(buff, "VZI %3d", pidBankMutable()->pid[PID_VEL_Z].I);
            break;
        case ADJUSTMENT_VEL_Z_D:
            tfp_sprintf(buff, "VZD %3d", pidBankMutable()->pid[PID_VEL_Z].D);
            break;
        case ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE:
            tfp_sprintf(buff, "MTDPA %4d", currentBatteryProfileMutable->fwMinThrottleDownPitchAngle);
            break;
        case ADJUSTMENT_TPA:
            tfp_sprintf(buff, "TPA %3d", currentControlRateProfile->throttle.dynPID);
            break;
        case ADJUSTMENT_TPA_BREAKPOINT:
            tfp_sprintf(buff, "TPABP %4d", currentControlRateProfile->throttle.pa_breakpoint);
            break;
        case ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS:
            tfp_sprintf(buff, "CSM %3d", navConfigMutable()->fw.control_smoothness);
            break;
        default:
            tfp_sprintf(buff, "UNSUPPORTED");
            break;
    }
}

static bool osdDJIFormatAdjustments(char *buff)
{
    uint8_t adjustmentFunctions[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];
    uint8_t adjustmentCount = getActiveAdjustmentFunctions(adjustmentFunctions);

    if (adjustmentCount > 0 && buff != NULL) {
        osdDJIAdjustmentMessage(buff, adjustmentFunctions[OSD_ALTERNATING_CHOICES(DJI_ALTERNATING_DURATION_LONG, adjustmentCount)]);
    }

    return adjustmentCount > 0;
}


static bool djiFormatMessages(char *buff)
{
    bool haveMessage = false;
    char messageBuf[MAX(SETTING_MAX_NAME_LENGTH, OSD_MESSAGE_LENGTH+1)];
    if (ARMING_FLAG(ARMED)) {
        // Aircraft is armed. We might have up to 6
        // messages to show.
        char *messages[6];
        unsigned messageCount = 0;

        if (FLIGHT_MODE(FAILSAFE_MODE)) {
            // In FS mode while being armed too
            char *failsafePhaseMessage = osdFailsafePhaseMessage();
            char *failsafeInfoMessage = osdFailsafeInfoMessage();
            char *navStateFSMessage = navigationStateMessage();

            if (failsafePhaseMessage) {
                messages[messageCount++] = failsafePhaseMessage;
            }

            if (failsafeInfoMessage) {
                messages[messageCount++] = failsafeInfoMessage;
            }

            if (navStateFSMessage) {
                messages[messageCount++] = navStateFSMessage;
            }
        } else {
#ifdef USE_SERIALRX_CRSF
            if (djiOsdConfig()->rssi_source == DJI_CRSF_LQ && rxLinkStatistics.rfMode == 0) {
                messages[messageCount++] = "CRSF LOW RF";
            }
#endif
            if (FLIGHT_MODE(NAV_RTH_MODE) || FLIGHT_MODE(NAV_WP_MODE) || navigationIsExecutingAnEmergencyLanding()) {
                char *navStateMessage = navigationStateMessage();
                if (navStateMessage) {
                    messages[messageCount++] = navStateMessage;
                }
            } else if (STATE(FIXED_WING_LEGACY) && (navGetCurrentStateFlags() & NAV_CTL_LAUNCH)) {
                messages[messageCount++] = "AUTOLAUNCH";
            } else {
                if (FLIGHT_MODE(NAV_ALTHOLD_MODE) && !navigationRequiresAngleMode()) {
                    // ALTHOLD might be enabled alongside ANGLE/HORIZON/ACRO
                    // when it doesn't require ANGLE mode (required only in FW
                    // right now). If if requires ANGLE, its display is handled
                    // by OSD_FLYMODE.
                    messages[messageCount++] = "(ALT HOLD)";
                }

                if (IS_RC_MODE_ACTIVE(BOXAUTOTRIM) && !feature(FEATURE_FW_AUTOTRIM)) {
                    messages[messageCount++] = "(AUTOTRIM)";
                }

                if (IS_RC_MODE_ACTIVE(BOXAUTOTUNE)) {
                    messages[messageCount++] = "(AUTOTUNE)";
                }

                if (IS_RC_MODE_ACTIVE(BOXAUTOLEVEL)) {
                    messages[messageCount++] = "(AUTOLEVEL)";
                }

                if (FLIGHT_MODE(HEADFREE_MODE)) {
                    messages[messageCount++] = "(HEADFREE)";
                }

                if (FLIGHT_MODE(MANUAL_MODE)) {
                    messages[messageCount++] = "(MANUAL)";
                }
            }
        }
        // Pick one of the available messages. Each message lasts
        // a second.
        if (messageCount > 0) {
           strcpy(buff, messages[OSD_ALTERNATING_CHOICES(DJI_ALTERNATING_DURATION_SHORT, messageCount)]);;
           haveMessage = true;
        }
    } else if (ARMING_FLAG(ARMING_DISABLED_ALL_FLAGS)) {
        unsigned invalidIndex;
        // Check if we're unable to arm for some reason
        if (ARMING_FLAG(ARMING_DISABLED_INVALID_SETTING) && !settingsValidate(&invalidIndex)) {
            if (OSD_ALTERNATING_CHOICES(DJI_ALTERNATING_DURATION_SHORT, 2) == 0) {
                const setting_t *setting = settingGet(invalidIndex);
                settingGetName(setting, messageBuf);
                for (int ii = 0; messageBuf[ii]; ii++) {
                    messageBuf[ii] = sl_toupper(messageBuf[ii]);
                }
                strcpy(buff, messageBuf);
            } else {
                strcpy(buff, "ERR SETTING");
                // TEXT_ATTRIBUTES_ADD_INVERTED(elemAttr);
            }
        } else {
            if (OSD_ALTERNATING_CHOICES(DJI_ALTERNATING_DURATION_SHORT, 2) == 0) {
                strcpy(buff, "CANT ARM");
                // TEXT_ATTRIBUTES_ADD_INVERTED(elemAttr);
            } else {
                // Show the reason for not arming
                strcpy(buff, osdArmingDisabledReasonMessage());
            }
        }
        haveMessage = true;
    }
    return haveMessage;
}

static void djiSerializeCraftNameOverride(sbuf_t *dst)
{
    char djibuf[DJI_CRAFT_NAME_LENGTH] = "\0";
    uint16_t *osdLayoutConfig = (uint16_t*)(osdLayoutsConfig()->item_pos[0]);

    if (!(OSD_VISIBLE(osdLayoutConfig[OSD_MESSAGES]) && djiFormatMessages(djibuf))
        && !(djiOsdConfig()->useAdjustments && osdDJIFormatAdjustments(djibuf))) {

        DjiCraftNameElements_t activeElements[DJI_OSD_CN_MAX_ELEMENTS];
        uint8_t activeElementsCount = 0;

        if (OSD_VISIBLE(osdLayoutConfig[OSD_THROTTLE_POS])) {
            activeElements[activeElementsCount++] = DJI_OSD_CN_THROTTLE;
        }

        if (OSD_VISIBLE(osdLayoutConfig[OSD_THROTTLE_POS_AUTO_THR])) {
            activeElements[activeElementsCount++] = DJI_OSD_CN_THROTTLE_AUTO_THR;
        }

        if (OSD_VISIBLE(osdLayoutConfig[OSD_3D_SPEED])) {
            activeElements[activeElementsCount++] = DJI_OSD_CN_AIR_SPEED;
        }

        if (OSD_VISIBLE(osdLayoutConfig[OSD_EFFICIENCY_MAH_PER_KM])) {
            activeElements[activeElementsCount++] = DJI_OSD_CN_EFFICIENCY;
        }

        if (OSD_VISIBLE(osdLayoutConfig[OSD_TRIP_DIST])) {
            activeElements[activeElementsCount++] = DJI_OSD_CN_DISTANCE;
        }

        switch (activeElements[OSD_ALTERNATING_CHOICES(DJI_ALTERNATING_DURATION_LONG, activeElementsCount)])
        {
            case DJI_OSD_CN_THROTTLE:
                osdDJIFormatThrottlePosition(djibuf, false);
                break;
            case DJI_OSD_CN_THROTTLE_AUTO_THR:
                osdDJIFormatThrottlePosition(djibuf, true);
                break;
            case DJI_OSD_CN_AIR_SPEED:
                osdDJIFormatVelocityStr(djibuf);
                break;
            case DJI_OSD_CN_EFFICIENCY:
                osdDJIEfficiencyMahPerKM(djibuf);
                break;
            case DJI_OSD_CN_DISTANCE:
                osdDJIFormatDistanceStr(djibuf, getTotalTravelDistance());
                break;
            default:
                break;
        }
    }

    if (djibuf[0] != '\0') {
        sbufWriteData(dst, djibuf, strlen(djibuf));
    }
}

#endif


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
#if defined(USE_OSD)
                if (djiOsdConfig()->use_name_for_messages)  {
                    djiSerializeCraftNameOverride(dst);
                } else {
#endif
                    sbufWriteData(dst, systemConfig()->name, MAX((int)strlen(systemConfig()->name), OSD_MESSAGE_LENGTH));
#if defined(USE_OSD)
                }
#endif

                break;
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

                sbufWriteU16(dst, constrain(averageSystemLoadPercent, 0, 100));
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
                sbufWriteU16(dst, rxGetChannelValue(i));
            }
            break;

        case DJI_MSP_RAW_GPS:
            sbufWriteU8(dst, gpsSol.fixType);
            sbufWriteU8(dst, gpsSol.numSat);
            sbufWriteU32(dst, gpsSol.llh.lat);
            sbufWriteU32(dst, gpsSol.llh.lon);
            sbufWriteU16(dst, gpsSol.llh.alt / 100);
            sbufWriteU16(dst, osdGetSpeedFromSelectedSource());
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
#ifdef USE_SERIALRX_CRSF
            // Range of RSSI field: 0-99: 99 = 150 hz , 0 - 98 50 hz / 4 hz
            if (djiOsdConfig()->rssi_source == DJI_CRSF_LQ) {
                uint16_t scaledLq = 0;
                if (rxLinkStatistics.rfMode >= 2) {
                    scaledLq = RSSI_MAX_VALUE;
                } else {
                    scaledLq = scaleRange(constrain(rxLinkStatistics.uplinkLQ, 0, 100), 0, 100, 0, RSSI_BOUNDARY(98));
                }
                sbufWriteU16(dst, scaledLq);
            } else {
#endif
                sbufWriteU16(dst, getRSSI());
#ifdef USE_SERIALRX_CRSF
            }
#endif
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

        case DJI_MSP_ESC_SENSOR_DATA:
            if (djiOsdConfig()->proto_workarounds & DJI_OSD_USE_NON_STANDARD_MSP_ESC_SENSOR_DATA) {
                // Version 1.00.06 of DJI firmware is not using the standard MSP_ESC_SENSOR_DATA
                uint16_t protoRpm = 0;
                int16_t protoTemp = 0;

#if defined(USE_ESC_SENSOR)
                if (STATE(ESC_SENSOR_ENABLED) && getMotorCount() > 0) {
                    uint32_t motorRpmAcc = 0;
                    int32_t motorTempAcc = 0;

                    for (int i = 0; i < getMotorCount(); i++) {
                        const escSensorData_t * escSensor = getEscTelemetry(i);
                        motorRpmAcc += escSensor->rpm;
                        motorTempAcc += escSensor->temperature;
                    }

                    protoRpm = motorRpmAcc / getMotorCount();
                    protoTemp = motorTempAcc / getMotorCount();
                }
#endif

                switch (djiOsdConfig()->esc_temperature_source) {
                    // This is ESC temperature (as intended)
                    case DJI_OSD_TEMP_ESC:
                        // No-op, temperature is already set to ESC
                        break;

                    // Re-purpose the field for core temperature
                    case DJI_OSD_TEMP_CORE:
                        getIMUTemperature(&protoTemp);
                        protoTemp = protoTemp / 10;
                        break;

                    // Re-purpose the field for baro temperature
                    case DJI_OSD_TEMP_BARO:
                        getBaroTemperature(&protoTemp);
                        protoTemp = protoTemp / 10;
                        break;
                }

                // No motor count, just raw temp and RPM data
                sbufWriteU8(dst, protoTemp);
                sbufWriteU16(dst, protoRpm);
            }
            else {
                // Use standard MSP_ESC_SENSOR_DATA message
                sbufWriteU8(dst, getMotorCount());
                for (int i = 0; i < getMotorCount(); i++) {
                    uint16_t motorRpm = 0;
                    int16_t motorTemp = 0;

                    // If ESC_SENSOR is enabled, pull the telemetry data and get motor RPM
#if defined(USE_ESC_SENSOR)
                    if (STATE(ESC_SENSOR_ENABLED)) {
                        const escSensorData_t * escSensor = getEscTelemetry(i);
                        motorRpm = escSensor->rpm;
                        motorTemp = escSensor->temperature;
                    }
#endif

                    // Now populate temperature field (which we may override for different purposes)
                    switch (djiOsdConfig()->esc_temperature_source) {
                        // This is ESC temperature (as intended)
                        case DJI_OSD_TEMP_ESC:
                            // No-op, temperature is already set to ESC
                            break;

                        // Re-purpose the field for core temperature
                        case DJI_OSD_TEMP_CORE:
                            getIMUTemperature(&motorTemp);
                            motorTemp = motorTemp / 10;
                            break;

                        // Re-purpose the field for baro temperature
                        case DJI_OSD_TEMP_BARO:
                            getBaroTemperature(&motorTemp);
                            motorTemp = motorTemp / 10;
                            break;
                    }

                    // Add data for this motor to the packet
                    sbufWriteU8(dst, motorTemp);
                    sbufWriteU16(dst, motorRpm);
                }
            }
            break;

        case DJI_MSP_OSD_CONFIG:
#if defined(USE_OSD)
            // This involved some serious magic, better contain in a separate function for readability
            djiSerializeOSDConfigReply(dst);
#else
            sbufWriteU8(dst, 0);
#endif
            break;

        case DJI_MSP_FILTER_CONFIG:
            sbufWriteU8(dst, gyroConfig()->gyro_main_lpf_hz);           // BF: gyroConfig()->gyro_lowpass_hz
            sbufWriteU16(dst, pidProfile()->dterm_lpf_hz);              // BF: currentPidProfile->dterm_lowpass_hz
            sbufWriteU16(dst, pidProfile()->yaw_lpf_hz);                // BF: currentPidProfile->yaw_lowpass_hz
            sbufWriteU16(dst, gyroConfig()->gyro_notch_hz);             // BF: gyroConfig()->gyro_soft_notch_hz_1
            sbufWriteU16(dst, gyroConfig()->gyro_notch_cutoff);         // BF: gyroConfig()->gyro_soft_notch_cutoff_1
            sbufWriteU16(dst, 0);                                       // BF: currentPidProfile->dterm_notch_hz
            sbufWriteU16(dst, 1);                                       // BF: currentPidProfile->dterm_notch_cutoff
            sbufWriteU16(dst, 0);                                       // BF: gyroConfig()->gyro_soft_notch_hz_2
            sbufWriteU16(dst, 1);                                       // BF: gyroConfig()->gyro_soft_notch_cutoff_2
            sbufWriteU8(dst, 0);                                        // BF: currentPidProfile->dterm_filter_type
            sbufWriteU8(dst, gyroConfig()->gyro_lpf);                   // BF: gyroConfig()->gyro_hardware_lpf);
            sbufWriteU8(dst, 0);                                        // BF: DEPRECATED: gyro_32khz_hardware_lpf
            sbufWriteU16(dst, gyroConfig()->gyro_main_lpf_hz);          // BF: gyroConfig()->gyro_lowpass_hz);
            sbufWriteU16(dst, 0);                                       // BF: gyroConfig()->gyro_lowpass2_hz);
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
