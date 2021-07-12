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
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include "platform.h"

#if defined(USE_NAV)

#include <stdlib.h>
#include <string.h>

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "navigation/navigation.h"

static const OSD_Entry cmsx_menuNavSettingsEntries[] =
{
    OSD_LABEL_ENTRY("-- BASIC SETTINGS --"),

    OSD_SETTING_ENTRY("CONTROL MODE", SETTING_NAV_USER_CONTROL_MODE),
    OSD_SETTING_ENTRY("MAX NAV SPEED", SETTING_NAV_AUTO_SPEED),
    OSD_SETTING_ENTRY("MAX CRUISE SPEED", SETTING_NAV_MANUAL_SPEED),
    OSD_SETTING_ENTRY("MAX NAV CLIMB RATE", SETTING_NAV_AUTO_CLIMB_RATE),
    OSD_SETTING_ENTRY("MAX AH CLIMB RATE", SETTING_NAV_MANUAL_CLIMB_RATE),
    OSD_SETTING_ENTRY("MC MAX BANK ANGLE", SETTING_NAV_MC_BANK_ANGLE),
    OSD_SETTING_ENTRY("MID THR FOR AH", SETTING_NAV_USE_MIDTHR_FOR_ALTHOLD),
    OSD_SETTING_ENTRY("MC HOVER THR", SETTING_NAV_MC_HOVER_THR),
    OSD_SETTING_ENTRY("MISSION RESTART", SETTING_NAV_WP_MISSION_RESTART),

    OSD_BACK_AND_END_ENTRY,
 };

static const CMS_Menu cmsx_menuNavSettings = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAVSETTINGS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuNavSettingsEntries
};

 static const OSD_Entry cmsx_menuRTHEntries[] =
 {
    OSD_LABEL_ENTRY("-- RTH --"),

    OSD_SETTING_ENTRY("RTH ALT MODE", SETTING_NAV_RTH_ALT_MODE),
    OSD_SETTING_ENTRY("RTH ALT", SETTING_NAV_RTH_ALTITUDE),
    OSD_SETTING_ENTRY("CLIMB FIRST", SETTING_NAV_RTH_CLIMB_FIRST),
    OSD_SETTING_ENTRY("TAIL FIRST", SETTING_NAV_RTH_TAIL_FIRST),
    OSD_SETTING_ENTRY("LAND AFTER RTH", SETTING_NAV_RTH_ALLOW_LANDING),
    OSD_SETTING_ENTRY("LAND MINALT VSPD", SETTING_NAV_LAND_MINALT_VSPD),
    OSD_SETTING_ENTRY("LAND MAXALT VSPD", SETTING_NAV_LAND_MAXALT_VSPD),
    OSD_SETTING_ENTRY("LAND SPEED MIN AT", SETTING_NAV_LAND_SLOWDOWN_MINALT),
    OSD_SETTING_ENTRY("LAND SPEED SLOW AT", SETTING_NAV_LAND_SLOWDOWN_MAXALT),
    OSD_SETTING_ENTRY("MIN RTH DISTANCE", SETTING_NAV_MIN_RTH_DISTANCE),
    OSD_SETTING_ENTRY("RTH ABORT THRES", SETTING_NAV_RTH_ABORT_THRESHOLD),
    OSD_SETTING_ENTRY("EMERG LANDING SPEED", SETTING_NAV_EMERG_LANDING_SPEED),
    OSD_SETTING_ENTRY("SAFEHOME USAGE MODE", SETTING_SAFEHOME_USAGE_MODE),

    OSD_BACK_AND_END_ENTRY,
 };

static const CMS_Menu cmsx_menuRTH = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAVRTH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuRTHEntries
};

static const OSD_Entry cmsx_menuFWCruiseEntries[] =
{
    OSD_LABEL_ENTRY("-- CRUISE --"),

    OSD_SETTING_ENTRY("CRUISE THROTTLE", SETTING_NAV_FW_CRUISE_THR),
    OSD_SETTING_ENTRY("MIN THROTTLE", SETTING_NAV_FW_MIN_THR),
    OSD_SETTING_ENTRY("MAX THROTTLE", SETTING_NAV_FW_MAX_THR),
    OSD_SETTING_ENTRY("MAX BANK ANGLE", SETTING_NAV_FW_BANK_ANGLE),
    OSD_SETTING_ENTRY("MAX CLIMB ANGLE", SETTING_NAV_FW_CLIMB_ANGLE),
    OSD_SETTING_ENTRY("MAX DIVE ANGLE", SETTING_NAV_FW_DIVE_ANGLE),
    OSD_SETTING_ENTRY("PITCH TO THR RATIO", SETTING_NAV_FW_PITCH2THR),
    OSD_SETTING_ENTRY("LOITER RADIUS", SETTING_NAV_FW_LOITER_RADIUS),
    OSD_SETTING_ENTRY("CONTROL SMOOTHNESS", SETTING_NAV_FW_CONTROL_SMOOTHNESS),
    OSD_SETTING_ENTRY("PITCH TO THR SMOOTHING", SETTING_NAV_FW_PITCH2THR_SMOOTHING),
    OSD_SETTING_ENTRY("PITCH TO THR THRESHOLD", SETTING_NAV_FW_PITCH2THR_THRESHOLD),
    OSD_SETTING_ENTRY("MANUAL THR INCREASE", SETTING_NAV_FW_ALLOW_MANUAL_THR_INCREASE),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuFWCruise = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAVFWCRUISE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFWCruiseEntries
};

static const OSD_Entry cmsx_menuFWLaunchEntries[] =
{
    OSD_LABEL_ENTRY("-- AUTOLAUNCH --"),

    OSD_SETTING_ENTRY("LAUNCH THR", SETTING_NAV_FW_LAUNCH_THR),
    OSD_SETTING_ENTRY("IDLE THROTTLE", SETTING_NAV_FW_LAUNCH_IDLE_THR),
    OSD_SETTING_ENTRY("MOTOR SPINUP TIME", SETTING_NAV_FW_LAUNCH_SPINUP_TIME),
    OSD_SETTING_ENTRY("TIMEOUT", SETTING_NAV_FW_LAUNCH_TIMEOUT),
    OSD_SETTING_ENTRY("END TRANSITION TIME", SETTING_NAV_FW_LAUNCH_END_TIME),
    OSD_SETTING_ENTRY("MAX ALTITUDE", SETTING_NAV_FW_LAUNCH_MAX_ALTITUDE),
    OSD_SETTING_ENTRY("CLIMB ANGLE", SETTING_NAV_FW_LAUNCH_CLIMB_ANGLE),
    OSD_SETTING_ENTRY("MAX BANK ANGLE", SETTING_NAV_FW_LAUNCH_MAX_ANGLE),
    OSD_SETTING_ENTRY("MOTOR DELAY", SETTING_NAV_FW_LAUNCH_MOTOR_DELAY),
    OSD_SETTING_ENTRY("VELOCITY", SETTING_NAV_FW_LAUNCH_VELOCITY),
    OSD_SETTING_ENTRY("ACCELERATION", SETTING_NAV_FW_LAUNCH_ACCEL),
    OSD_SETTING_ENTRY("DETECT TIME", SETTING_NAV_FW_LAUNCH_DETECT_TIME),

    OSD_BACK_AND_END_ENTRY,
 };

static const CMS_Menu cmsx_menuFWLaunch = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAVFWLAUNCH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFWLaunchEntries
};

static const OSD_Entry cmsx_menuFWSettingsEntries[] =
{
    OSD_LABEL_ENTRY("-- FIXED WING --"),

    OSD_SUBMENU_ENTRY("AUTOLAUNCH", &cmsx_menuFWLaunch),
    OSD_SUBMENU_ENTRY("CRUISE", &cmsx_menuFWCruise),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuFWSettings = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAVFW",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFWSettingsEntries
};

static const OSD_Entry cmsx_menuMissionSettingsEntries[] =
{
    OSD_LABEL_ENTRY("-- MISSIONS --"),

    OSD_SETTING_ENTRY("MC WP SLOWDOWN", SETTING_NAV_MC_WP_SLOWDOWN),
    OSD_SETTING_ENTRY("MISSION FAILSAFE", SETTING_FAILSAFE_MISSION),
    OSD_SETTING_ENTRY("WP LOAD ON BOOT", SETTING_NAV_WP_LOAD_ON_BOOT),
    OSD_SETTING_ENTRY("WP REACHED RADIUS", SETTING_NAV_WP_RADIUS),
    OSD_SETTING_ENTRY("WP SAFE DISTANCE", SETTING_NAV_WP_SAFE_DISTANCE),

    OSD_BACK_AND_END_ENTRY,
 };

static const CMS_Menu cmsx_menuMissionSettings = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUMISSIONSETTINGS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuMissionSettingsEntries
};

static const OSD_Entry cmsx_menuNavigationEntries[] =
{
    OSD_LABEL_ENTRY("-- NAVIGATION --"),

    OSD_SUBMENU_ENTRY("BASIC SETTINGS", &cmsx_menuNavSettings),
    OSD_SUBMENU_ENTRY("RTH", &cmsx_menuRTH),
    OSD_SUBMENU_ENTRY("FIXED WING", &cmsx_menuFWSettings),
    OSD_SUBMENU_ENTRY("MISSIONS", &cmsx_menuMissionSettings),

    OSD_BACK_AND_END_ENTRY,
};

const CMS_Menu cmsx_menuNavigation = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUNAV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuNavigationEntries
};

 #endif
