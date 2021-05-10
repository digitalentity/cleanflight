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

#if defined(USE_OSD) && defined(USE_CMS) && defined(CMS_MENU_OSD)

#include "build/debug.h"

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "fc/settings.h"

#include "io/osd.h"

#define OSD_ITEM_ENTRY(label, item_id)      ((OSD_Entry){ label, {.itemId  = item_id}, &cmsx_menuOsdElementActions, OME_Submenu, 0 })

static int osdCurrentLayout = -1;
static int osdCurrentItem = -1;

static uint8_t osdCurrentElementRow = 0;
static uint8_t osdCurrentElementColumn = 0;
static uint8_t osdCurrentElementVisible = 0;

static long osdElementsOnEnter(const OSD_Entry *from);
static long osdElementsOnExit(const OSD_Entry *from);
static long osdElemActionsOnEnter(const OSD_Entry *from);

static const OSD_Entry cmsx_menuAlarmsEntries[] = {
    OSD_LABEL_ENTRY("--- ALARMS ---"),

    OSD_SETTING_ENTRY("RSSI", SETTING_OSD_RSSI_ALARM),
    OSD_SETTING_ENTRY("FLY TIME", SETTING_OSD_TIME_ALARM),
    OSD_SETTING_ENTRY("MAX ALT", SETTING_OSD_ALT_ALARM),
    OSD_SETTING_ENTRY("MAX DIST", SETTING_OSD_DIST_ALARM),
    OSD_SETTING_ENTRY("MAX NEG ALT", SETTING_OSD_NEG_ALT_ALARM),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuAlarms = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDA",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuAlarmsEntries,
};

static long cmsx_osdElementOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(ptr);

    uint16_t *pos = &osdLayoutsConfigMutable()->item_pos[osdCurrentLayout][osdCurrentItem];
    *pos = OSD_POS(osdCurrentElementColumn, osdCurrentElementRow);
    if (osdCurrentElementVisible) {
        *pos |= OSD_VISIBLE_FLAG;
    }
    cmsYieldDisplay(displayPort, 500);
    return 0;
}

static long osdElementPreview(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(ptr);

    cmsYieldDisplay(displayPort, 2000);

    return 0;
}

static const OSD_Entry menuOsdElemActionsEntries[] = {

    OSD_BOOL_CALLBACK_ENTRY("ENABLED", cmsx_osdElementOnChange, &osdCurrentElementVisible),
    OSD_UINT8_CALLBACK_ENTRY("ROW", cmsx_osdElementOnChange, (&(const OSD_UINT8_t){ &osdCurrentElementRow, 0, OSD_X(OSD_POS_MAX), 1 })),
    OSD_UINT8_CALLBACK_ENTRY("COLUMN", cmsx_osdElementOnChange, (&(const OSD_UINT8_t){ &osdCurrentElementColumn, 0, OSD_Y(OSD_POS_MAX), 1 })),
    OSD_FUNC_CALL_ENTRY("PREVIEW", osdElementPreview),

    OSD_BACK_AND_END_ENTRY,
};

static const OSD_Entry menuOsdFixedElemActionsEntries[] = {

    OSD_BOOL_CALLBACK_ENTRY("ENABLED", cmsx_osdElementOnChange, &osdCurrentElementVisible),
    OSD_FUNC_CALL_ENTRY("PREVIEW", osdElementPreview),

    OSD_BACK_AND_END_ENTRY,
};

static CMS_Menu cmsx_menuOsdElementActions = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDELEM",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = osdElemActionsOnEnter,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuOsdElemActionsEntries,
};

static long osdElemActionsOnEnter(const OSD_Entry *from)
{
    osdCurrentItem = from->itemId;
    uint16_t pos = osdLayoutsConfig()->item_pos[osdCurrentLayout][osdCurrentItem];
    osdCurrentElementColumn = OSD_X(pos);
    osdCurrentElementRow = OSD_Y(pos);
    osdCurrentElementVisible = OSD_VISIBLE(pos) ? 1 : 0;
    if (osdItemIsFixed(osdCurrentItem)) {
        cmsx_menuOsdElementActions.entries = menuOsdFixedElemActionsEntries;
    } else {
        cmsx_menuOsdElementActions.entries = menuOsdElemActionsEntries;
    }
    return 0;
}

#define OSD_ELEMENT_ENTRY(name, osd_item_id)    OSD_ITEM_ENTRY(name, osd_item_id)

static const OSD_Entry menuCrsfRxEntries[]=
{
    OSD_LABEL_ENTRY("-- CRSF RX --"),

    OSD_SETTING_ENTRY("LQ FORMAT", SETTING_OSD_CRSF_LQ_FORMAT),
    OSD_SETTING_ENTRY("LQ ALARM LEVEL", SETTING_OSD_LINK_QUALITY_ALARM),
    OSD_SETTING_ENTRY("SNR ALARM LEVEL", SETTING_OSD_SNR_ALARM),
    OSD_ELEMENT_ENTRY("RX RSSI DBM", OSD_CRSF_RSSI_DBM),
    OSD_ELEMENT_ENTRY("RX LQ", OSD_CRSF_LQ),
    OSD_ELEMENT_ENTRY("RX SNR ALARM", OSD_CRSF_SNR_DB),
    OSD_ELEMENT_ENTRY("TX POWER", OSD_CRSF_TX_POWER),

    OSD_BACK_AND_END_ENTRY,
};

const CMS_Menu cmsx_menuCrsf = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUCRF",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuCrsfRxEntries,
};

static const OSD_Entry menuOsdElemsEntries[] =
{
    OSD_LABEL_ENTRY("--- OSD ITEMS ---"),

    OSD_ELEMENT_ENTRY("RSSI", OSD_RSSI_VALUE),
#ifdef USE_SERIALRX_CRSF
    OSD_SUBMENU_ENTRY("CRSF RX", &cmsx_menuCrsf),
#endif // CRSF Menu
    OSD_ELEMENT_ENTRY("MAIN BATTERY", OSD_MAIN_BATT_VOLTAGE),
    OSD_ELEMENT_ENTRY("MAIN BATT SC", OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE),
    OSD_ELEMENT_ENTRY("CELL VOLTAGE", OSD_MAIN_BATT_CELL_VOLTAGE),
    OSD_ELEMENT_ENTRY("CELL VOLT. SC", OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE),
    OSD_ELEMENT_ENTRY("BAT IMPEDANCE", OSD_POWER_SUPPLY_IMPEDANCE),
    OSD_ELEMENT_ENTRY("CROSSHAIRS", OSD_CROSSHAIRS),
    OSD_ELEMENT_ENTRY("HORIZON", OSD_ARTIFICIAL_HORIZON),
    OSD_ELEMENT_ENTRY("HORIZON SIDEBARS", OSD_HORIZON_SIDEBARS),
    OSD_ELEMENT_ENTRY("ON TIME", OSD_ONTIME),
    OSD_ELEMENT_ENTRY("FLY TIME", OSD_FLYTIME),
    OSD_ELEMENT_ENTRY("ON/FLY TIME", OSD_ONTIME_FLYTIME),
    OSD_ELEMENT_ENTRY("REM. TIME", OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH),
    OSD_ELEMENT_ENTRY("REM. DIST", OSD_REMAINING_DISTANCE_BEFORE_RTH),
    OSD_ELEMENT_ENTRY("TIME (HOUR)", OSD_RTC_TIME),
    OSD_ELEMENT_ENTRY("FLY MODE", OSD_FLYMODE),
    OSD_ELEMENT_ENTRY("NAME", OSD_CRAFT_NAME),
    OSD_ELEMENT_ENTRY("THR. (MANU)", OSD_THROTTLE_POS),
    OSD_ELEMENT_ENTRY("THR. (MANU/AUTO)", OSD_THROTTLE_POS_AUTO_THR),
    OSD_ELEMENT_ENTRY("SYS MESSAGES", OSD_MESSAGES),
    OSD_ELEMENT_ENTRY("VTX CHAN", OSD_VTX_CHANNEL),
    OSD_ELEMENT_ENTRY("CURRENT (A)", OSD_CURRENT_DRAW),
    OSD_ELEMENT_ENTRY("POWER", OSD_POWER),
    OSD_ELEMENT_ENTRY("USED MAH", OSD_MAH_DRAWN),
    OSD_ELEMENT_ENTRY("USED WH", OSD_WH_DRAWN),
    OSD_ELEMENT_ENTRY("EFF/KM (AH)", OSD_EFFICIENCY_MAH_PER_KM),
    OSD_ELEMENT_ENTRY("EFF/KM (WH)", OSD_EFFICIENCY_WH_PER_KM),
    OSD_ELEMENT_ENTRY("BATT CAP REM", OSD_BATTERY_REMAINING_CAPACITY),
    OSD_ELEMENT_ENTRY("BATT % REM", OSD_BATTERY_REMAINING_PERCENT),
#ifdef USE_GPS
    OSD_ELEMENT_ENTRY("HOME DIR", OSD_HOME_DIR),
    OSD_ELEMENT_ENTRY("HOME HEAD. ERR", OSD_HOME_HEADING_ERROR),
    OSD_ELEMENT_ENTRY("HOME DIST", OSD_HOME_DIST),
    OSD_ELEMENT_ENTRY("TRIP DIST", OSD_TRIP_DIST),
    OSD_ELEMENT_ENTRY("GPS SPEED", OSD_GPS_SPEED),
    OSD_ELEMENT_ENTRY("GPS SATS", OSD_GPS_SATS),
    OSD_ELEMENT_ENTRY("GPS LAT", OSD_GPS_LAT),
    OSD_ELEMENT_ENTRY("GPS LON", OSD_GPS_LON),
    OSD_ELEMENT_ENTRY("GPS HDOP", OSD_GPS_HDOP),
    OSD_ELEMENT_ENTRY("3D SPEED", OSD_3D_SPEED),
    OSD_ELEMENT_ENTRY("PLUS CODE", OSD_PLUS_CODE),
    OSD_ELEMENT_ENTRY("AZIMUTH", OSD_AZIMUTH),
#endif // GPS
    OSD_ELEMENT_ENTRY("HEADING", OSD_HEADING),
    OSD_ELEMENT_ENTRY("HEADING GR.", OSD_HEADING_GRAPH),
    OSD_ELEMENT_ENTRY("CRS HLD ERR", OSD_COURSE_HOLD_ERROR),
    OSD_ELEMENT_ENTRY("CRS HLD ADJ", OSD_COURSE_HOLD_ADJUSTMENT),
#if defined(USE_BARO) || defined(USE_GPS)
    OSD_ELEMENT_ENTRY("VARIO", OSD_VARIO),
    OSD_ELEMENT_ENTRY("VARIO NUM", OSD_VARIO_NUM),
#endif // defined
    OSD_ELEMENT_ENTRY("ALTITUDE", OSD_ALTITUDE),
    OSD_ELEMENT_ENTRY("ALTITUDE MSL", OSD_ALTITUDE_MSL),
#if defined(USE_PITOT)
    OSD_ELEMENT_ENTRY("AIR SPEED", OSD_AIR_SPEED),
#endif
#if defined(USE_GPS)
    OSD_ELEMENT_ENTRY("MAP NORTH", OSD_MAP_NORTH),
    OSD_ELEMENT_ENTRY("MAP TAKE OFF", OSD_MAP_TAKEOFF),
    OSD_ELEMENT_ENTRY("RADAR", OSD_RADAR),
    OSD_ELEMENT_ENTRY("MAP SCALE", OSD_MAP_SCALE),
    OSD_ELEMENT_ENTRY("MAP REFERENCE", OSD_MAP_REFERENCE),
#endif
    OSD_ELEMENT_ENTRY("EXPO", OSD_RC_EXPO),
    OSD_ELEMENT_ENTRY("YAW EXPO", OSD_RC_YAW_EXPO),
    OSD_ELEMENT_ENTRY("THR EXPO", OSD_THROTTLE_EXPO),
    OSD_ELEMENT_ENTRY("ROLL RATE", OSD_ROLL_RATE),
    OSD_ELEMENT_ENTRY("PITCH RATE", OSD_PITCH_RATE),
    OSD_ELEMENT_ENTRY("YAW RATE", OSD_YAW_RATE),
    OSD_ELEMENT_ENTRY("M EXPO", OSD_MANUAL_RC_EXPO),
    OSD_ELEMENT_ENTRY("M YAW EXPO", OSD_MANUAL_RC_YAW_EXPO),
    OSD_ELEMENT_ENTRY("M ROLL RATE", OSD_MANUAL_ROLL_RATE),
    OSD_ELEMENT_ENTRY("M PITCH RATE", OSD_MANUAL_PITCH_RATE),
    OSD_ELEMENT_ENTRY("M YAW RATE", OSD_MANUAL_YAW_RATE),
    OSD_ELEMENT_ENTRY("CRUISE THR", OSD_NAV_FW_CRUISE_THR),
    OSD_ELEMENT_ENTRY("PITCH TO THR", OSD_NAV_FW_PITCH2THR),
    OSD_ELEMENT_ENTRY("ROLL PIDS", OSD_ROLL_PIDS),
    OSD_ELEMENT_ENTRY("PITCH PIDS", OSD_PITCH_PIDS),
    OSD_ELEMENT_ENTRY("YAW PIDS", OSD_YAW_PIDS),
    OSD_ELEMENT_ENTRY("LEVEL PIDS", OSD_LEVEL_PIDS),
    OSD_ELEMENT_ENTRY("POSXY PIDS", OSD_POS_XY_PIDS),
    OSD_ELEMENT_ENTRY("POSZ PIDS", OSD_POS_Z_PIDS),
    OSD_ELEMENT_ENTRY("VELXY PIDS", OSD_VEL_XY_PIDS),
    OSD_ELEMENT_ENTRY("VELZ PIDS", OSD_VEL_Z_PIDS),
    OSD_ELEMENT_ENTRY("HEAD P", OSD_HEADING_P),
    OSD_ELEMENT_ENTRY("ALIGN ROLL", OSD_BOARD_ALIGN_ROLL),
    OSD_ELEMENT_ENTRY("ALIGN PITCH", OSD_BOARD_ALIGN_PITCH),
    OSD_ELEMENT_ENTRY("0THR PITCH", OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE),

    OSD_ELEMENT_ENTRY("FW ALT PID OUT", OSD_FW_ALT_PID_OUTPUTS),
    OSD_ELEMENT_ENTRY("FW POS PID OUT", OSD_FW_POS_PID_OUTPUTS),
    OSD_ELEMENT_ENTRY("MC VELX PID OUT", OSD_MC_VEL_X_PID_OUTPUTS),
    OSD_ELEMENT_ENTRY("MC VELY PID OUT", OSD_MC_VEL_Y_PID_OUTPUTS),
    OSD_ELEMENT_ENTRY("MC VELZ PID OUT", OSD_MC_VEL_Z_PID_OUTPUTS),
    OSD_ELEMENT_ENTRY("MC POS PID OUT", OSD_MC_POS_XYZ_P_OUTPUTS),

    OSD_ELEMENT_ENTRY("ATTI PITCH", OSD_ATTITUDE_PITCH),
    OSD_ELEMENT_ENTRY("ATTI ROLL", OSD_ATTITUDE_ROLL),

    OSD_ELEMENT_ENTRY("WIND HOR", OSD_WIND_SPEED_HORIZONTAL),
    OSD_ELEMENT_ENTRY("WIND VERT", OSD_WIND_SPEED_VERTICAL),

    OSD_ELEMENT_ENTRY("G-FORCE", OSD_GFORCE),
    OSD_ELEMENT_ENTRY("G-FORCE X", OSD_GFORCE_X),
    OSD_ELEMENT_ENTRY("G-FORCE Y", OSD_GFORCE_Y),
    OSD_ELEMENT_ENTRY("G-FORCE Z", OSD_GFORCE_Z),

    OSD_ELEMENT_ENTRY("VTX POWER", OSD_VTX_POWER),

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    OSD_ELEMENT_ENTRY("RC SOURCE", OSD_RC_SOURCE),
#endif

    OSD_ELEMENT_ENTRY("IMU TEMP", OSD_IMU_TEMPERATURE),
#ifdef USE_BARO
    OSD_ELEMENT_ENTRY("BARO TEMP", OSD_BARO_TEMPERATURE),
#endif

#ifdef USE_TEMPERATURE_SENSOR
    OSD_ELEMENT_ENTRY("SENSOR 0 TEMP", OSD_TEMP_SENSOR_0_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 1 TEMP", OSD_TEMP_SENSOR_1_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 2 TEMP", OSD_TEMP_SENSOR_2_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 3 TEMP", OSD_TEMP_SENSOR_3_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 4 TEMP", OSD_TEMP_SENSOR_4_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 5 TEMP", OSD_TEMP_SENSOR_5_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 6 TEMP", OSD_TEMP_SENSOR_6_TEMPERATURE),
    OSD_ELEMENT_ENTRY("SENSOR 7 TEMP", OSD_TEMP_SENSOR_7_TEMPERATURE),
#endif

    OSD_ELEMENT_ENTRY("GVAR 0", OSD_GVAR_0),
    OSD_ELEMENT_ENTRY("GVAR 1", OSD_GVAR_1),
    OSD_ELEMENT_ENTRY("GVAR 2", OSD_GVAR_2),
    OSD_ELEMENT_ENTRY("GVAR 3", OSD_GVAR_3),

    OSD_ELEMENT_ENTRY("TPA", OSD_TPA),
    OSD_ELEMENT_ENTRY("FW CTRL SMOOTH", OSD_NAV_FW_CONTROL_SMOOTHNESS),
    OSD_ELEMENT_ENTRY("VERSION", OSD_VERSION),
    OSD_ELEMENT_ENTRY("RANGEFINDER", OSD_RANGEFINDER),

#ifdef USE_ESC_SENSOR
    OSD_ELEMENT_ENTRY("ESC RPM", OSD_ESC_RPM),
    OSD_ELEMENT_ENTRY("ESC TEMPERATURE", OSD_ESC_TEMPERATURE),
#endif

#ifdef USE_POWER_LIMITS
    OSD_ELEMENT_ENTRY("PLIM BURST TIME", OSD_PLIMIT_REMAINING_BURST_TIME),
    OSD_ELEMENT_ENTRY("PLIM CURR LIMIT", OSD_PLIMIT_ACTIVE_CURRENT_LIMIT),
#ifdef USE_ADC
    OSD_ELEMENT_ENTRY("PLIM POWER LIMIT", OSD_PLIMIT_ACTIVE_POWER_LIMIT),
#endif // USE_ADC
#endif // USE_POWER_LIMITS

    OSD_BACK_AND_END_ENTRY,
};

#if defined(USE_GPS) && defined(USE_BARO) && defined(USE_PITOT) && defined(USE_TEMPERATURE_SENSOR) && defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
// All CMS OSD elements should be enabled in this case. The menu has 2 extra
// elements (label, back+end), but there's an OSD element that we intentionally
// don't show here (OSD_DEBUG).
_Static_assert(ARRAYLEN(menuOsdElemsEntries) - 2 + 1 == OSD_ITEM_COUNT, "missing OSD elements in CMS");
#endif

const CMS_Menu menuOsdElements = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = osdElementsOnEnter,
    .onExit = osdElementsOnExit,
    .onGlobalExit = NULL,
    .entries = menuOsdElemsEntries,
};


#define OSD_LAYOUT_SUBMENU_ENTRY(label)   OSD_SUBMENU_ENTRY(label, &menuOsdElements)

static const OSD_Entry cmsx_menuOsdLayoutEntries[] =
{
    OSD_LABEL_ENTRY("---SCREEN LAYOUT---"),

    OSD_LAYOUT_SUBMENU_ENTRY("DEFAULT"),
#if OSD_ALTERNATE_LAYOUT_COUNT > 0
    OSD_LAYOUT_SUBMENU_ENTRY("ALTERNATE 1"),
#if OSD_ALTERNATE_LAYOUT_COUNT > 1
    OSD_LAYOUT_SUBMENU_ENTRY("ALTERNATE 2"),
#if OSD_ALTERNATE_LAYOUT_COUNT > 2
    OSD_LAYOUT_SUBMENU_ENTRY("ALTERNATE 3"),
#endif
#endif
#endif

    OSD_BACK_AND_END_ENTRY,
};

const CMS_Menu cmsx_menuOsdLayout = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDL",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuOsdLayoutEntries,
};

static long osdElementsOnEnter(const OSD_Entry *from)
{
    // First entry is the label. Store the current layout
    // and override it on the OSD so previews so this layout.
    osdCurrentLayout = from - cmsx_menuOsdLayoutEntries - 1;
    osdOverrideLayout(osdCurrentLayout, 0);
    return 0;
}

static long osdElementsOnExit(const OSD_Entry *from)
{
    UNUSED(from);

    // Stop overriding OSD layout
    osdOverrideLayout(-1, 0);
    return 0;
}

static const OSD_Entry menuOsdSettingsEntries[] = {
    OSD_LABEL_ENTRY("--- OSD SETTINGS ---"),

    OSD_SETTING_ENTRY("VOLT. DECIMALS", SETTING_OSD_MAIN_VOLTAGE_DECIMALS),
    OSD_SETTING_ENTRY("COORD. DIGITS", SETTING_OSD_COORDINATE_DIGITS),
    OSD_SETTING_ENTRY("LEFT SCROLL", SETTING_OSD_LEFT_SIDEBAR_SCROLL),
    OSD_SETTING_ENTRY("RIGHT SCROLL", SETTING_OSD_RIGHT_SIDEBAR_SCROLL),
    OSD_SETTING_ENTRY("SCROLL ARROWS", SETTING_OSD_SIDEBAR_SCROLL_ARROWS),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuOsdSettings = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuOsdSettingsEntries,
};

static const OSD_Entry menuOsdHud2Entries[] = {
    OSD_LABEL_ENTRY("--- HUD ITEMS ---"),

    OSD_SETTING_ENTRY("HOMING ARROWS", SETTING_OSD_HUD_HOMING),
    OSD_SETTING_ENTRY("HOME POINT", SETTING_OSD_HUD_HOMEPOINT),
    OSD_SETTING_ENTRY("RADAR MAX AIRCRAFT", SETTING_OSD_HUD_RADAR_DISP),
    OSD_SETTING_ENTRY("RADAR MIN RANGE", SETTING_OSD_HUD_RADAR_RANGE_MIN),
    OSD_SETTING_ENTRY("RADAR MAX RANGE", SETTING_OSD_HUD_RADAR_RANGE_MAX),
    OSD_SETTING_ENTRY("RADAR DET. NEAREST", SETTING_OSD_HUD_RADAR_NEAREST),
    OSD_SETTING_ENTRY("NEXT WAYPOINTS", SETTING_OSD_HUD_WP_DISP),
    OSD_BACK_ENTRY,
    OSD_END_ENTRY,
};

static const CMS_Menu cmsx_menuOsdHud2 = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDH2",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuOsdHud2Entries,
};

static const OSD_Entry menuOsdHudEntries[] = {
    OSD_LABEL_ENTRY("--- HUD ---"),

    OSD_SETTING_ENTRY("CROSSHAIRS STYLE", SETTING_OSD_CROSSHAIRS_STYLE),
    OSD_SETTING_ENTRY("HORIZON OFFSET", SETTING_OSD_HORIZON_OFFSET),
    OSD_SETTING_ENTRY("CAMERA UPTILT", SETTING_OSD_CAMERA_UPTILT),
    OSD_SETTING_ENTRY("CAMERA FOV HORI", SETTING_OSD_CAMERA_FOV_H),
    OSD_SETTING_ENTRY("CAMERA FOV VERT", SETTING_OSD_CAMERA_FOV_V),
    OSD_SETTING_ENTRY("HUD MARGIN HORI", SETTING_OSD_HUD_MARGIN_H),
    OSD_SETTING_ENTRY("HUD MARGIN VERT", SETTING_OSD_HUD_MARGIN_V),
    OSD_SUBMENU_ENTRY("DISPLAYED ITEMS", &cmsx_menuOsdHud2),

    OSD_BACK_ENTRY,
    OSD_END_ENTRY,
};

static const CMS_Menu cmsx_menuOsdHud = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuOsdHudEntries,
};

static const OSD_Entry menuOsdEntries[] = {
    OSD_LABEL_ENTRY("--- OSD ---"),

    OSD_SUBMENU_ENTRY("LAYOUTS", &cmsx_menuOsdLayout),
    OSD_SUBMENU_ENTRY("SETTINGS", &cmsx_menuOsdSettings),
    OSD_SUBMENU_ENTRY("ALARMS", &cmsx_menuAlarms),
    OSD_SUBMENU_ENTRY("HUD", &cmsx_menuOsdHud),

    OSD_BACK_AND_END_ENTRY,
};


const CMS_Menu cmsx_menuOsd = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSD",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = menuOsdEntries,
};

#endif // CMS
