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

#pragma once

#include "common/time.h"
#include "config/parameter_group.h"

#include "drivers/vcd.h"

#ifndef OSD_ALTERNATE_LAYOUT_COUNT
#define OSD_ALTERNATE_LAYOUT_COUNT 3
#endif
#define OSD_LAYOUT_COUNT (OSD_ALTERNATE_LAYOUT_COUNT + 1)

#define OSD_VISIBLE_FLAG    0x0800
#define OSD_VISIBLE(x)      ((x) & OSD_VISIBLE_FLAG)
#define OSD_POS(x,y)        ((x) | ((y) << 5))
#define OSD_X(x)            ((x) & 0x001F)
#define OSD_Y(x)            (((x) >> 5) & 0x001F)
#define OSD_POS_MAX         0x3FF
#define OSD_POS_MAX_CLI     (OSD_POS_MAX | OSD_VISIBLE_FLAG)

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ONTIME,
    OSD_FLYTIME,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_HEADING,
    OSD_VARIO,
    OSD_VARIO_NUM,
    OSD_AIR_SPEED,
    OSD_ONTIME_FLYTIME,
    OSD_RTC_TIME,
    OSD_MESSAGES,
    OSD_GPS_HDOP,
    OSD_MAIN_BATT_CELL_VOLTAGE,
    OSD_THROTTLE_POS_AUTO_THR,
    OSD_HEADING_GRAPH,
    OSD_EFFICIENCY_MAH_PER_KM,
    OSD_WH_DRAWN,
    OSD_BATTERY_REMAINING_CAPACITY,
    OSD_BATTERY_REMAINING_PERCENT,
    OSD_EFFICIENCY_WH_PER_KM,
    OSD_TRIP_DIST,
    OSD_ATTITUDE_PITCH,
    OSD_ATTITUDE_ROLL,
    OSD_MAP_NORTH,
    OSD_MAP_TAKEOFF,
    OSD_RADAR,
    OSD_WIND_SPEED_HORIZONTAL,
    OSD_WIND_SPEED_VERTICAL,
    OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH,
    OSD_REMAINING_DISTANCE_BEFORE_RTH,
    OSD_HOME_HEADING_ERROR,
    OSD_CRUISE_HEADING_ERROR,
    OSD_CRUISE_HEADING_ADJUSTMENT,
    OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE,
    OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE,
    OSD_POWER_SUPPLY_IMPEDANCE,
    OSD_LEVEL_PIDS,
    OSD_POS_XY_PIDS,
    OSD_POS_Z_PIDS,
    OSD_VEL_XY_PIDS,
    OSD_VEL_Z_PIDS,
    OSD_HEADING_P,
    OSD_BOARD_ALIGN_ROLL,
    OSD_BOARD_ALIGN_PITCH,
    OSD_RC_EXPO,
    OSD_RC_YAW_EXPO,
    OSD_THROTTLE_EXPO,
    OSD_PITCH_RATE,
    OSD_ROLL_RATE,
    OSD_YAW_RATE,
    OSD_MANUAL_RC_EXPO,
    OSD_MANUAL_RC_YAW_EXPO,
    OSD_MANUAL_PITCH_RATE,
    OSD_MANUAL_ROLL_RATE,
    OSD_MANUAL_YAW_RATE,
    OSD_NAV_FW_CRUISE_THR,
    OSD_NAV_FW_PITCH2THR,
    OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE,
    OSD_DEBUG, // Intentionally absent from configurator and CMS. Set it from CLI.
    OSD_FW_ALT_PID_OUTPUTS,
    OSD_FW_POS_PID_OUTPUTS,
    OSD_MC_VEL_X_PID_OUTPUTS,
    OSD_MC_VEL_Y_PID_OUTPUTS,
    OSD_MC_VEL_Z_PID_OUTPUTS,
    OSD_MC_POS_XYZ_P_OUTPUTS,
    OSD_3D_SPEED,       // 85
    OSD_TEMPERATURE,    // 86
    OSD_ALTITUDE_MSL,   // 87
    OSD_PLUS_CODE,      // 88
    OSD_PILOT_NAME,     // 89
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC,
    OSD_UNIT_UK, // Show speed in mp/h, other values in metric
} osd_unit_e;

typedef enum {
    OSD_STATS_ENERGY_UNIT_MAH,
    OSD_STATS_ENERGY_UNIT_WH,
} osd_stats_energy_unit_e;

typedef enum {
    OSD_CROSSHAIRS_STYLE_DEFAULT,
    OSD_CROSSHAIRS_STYLE_AIRCRAFT,
} osd_crosshairs_style_e;

typedef enum {
    OSD_SIDEBAR_SCROLL_NONE,
    OSD_SIDEBAR_SCROLL_ALTITUDE,
    OSD_SIDEBAR_SCROLL_GROUND_SPEED,
    OSD_SIDEBAR_SCROLL_HOME_DISTANCE,
} osd_sidebar_scroll_e;

typedef struct osdConfig_s {
    // Layouts
    uint16_t item_pos[OSD_LAYOUT_COUNT][OSD_ITEM_COUNT];

    // Alarms
    uint8_t rssi_alarm; // rssi %
    uint16_t time_alarm; // fly minutes
    uint16_t alt_alarm; // positive altitude in m
    uint16_t dist_alarm; // home distance in m
    uint16_t neg_alt_alarm; // abs(negative altitude) in m

    videoSystem_e video_system;
    uint8_t row_shiftdown;

    // Preferences
    uint8_t main_voltage_decimals;
    uint8_t ahi_reverse_roll;
    uint8_t ahi_max_pitch;
    uint8_t crosshairs_style; // from osd_crosshairs_style_e
    uint8_t left_sidebar_scroll; // from osd_sidebar_scroll_e
    uint8_t right_sidebar_scroll; // from osd_sidebar_scroll_e
    uint8_t sidebar_scroll_arrows;

    uint8_t units; // from osd_unit_e
    uint8_t stats_energy_unit; // from osd_stats_energy_unit_e

    bool    estimations_wind_compensation; // use wind compensation for estimated remaining flight/distance
    uint8_t coordinate_digits;

    bool osd_failsafe_switch_layout;
    uint8_t plus_code_digits; // Number of digits to use in OSD_PLUS_CODE
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
void osdUpdate(timeUs_t currentTimeUs);
void osdStartFullRedraw(void);
// Sets a fixed OSD layout ignoring the RC input. Set it
// to -1 to disable the override. If layout is >= 0 and
// duration is > 0, the override is automatically cleared by
// the OSD after the given duration. Otherwise, the caller must
// explicitely remove it.
void osdOverrideLayout(int layout, timeMs_t duration);
// Returns the current current layout as well as wether its
// set by the user configuration (modes, etc..) or by overriding it.
int osdGetActiveLayout(bool *overridden);
bool osdItemIsFixed(osd_items_e item);
