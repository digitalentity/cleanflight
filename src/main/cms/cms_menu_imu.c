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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CMS

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/axis.h"

#include "flight/pid.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/settings.h"

#include "navigation/navigation.h"

#include "sensors/gyro.h"


//
// PIDFF
//
#define PIDFF_MIN 0
#define PIDFF_STEP 1

#define RPY_PIDFF_MAX 200
#define OTHER_PIDDF_MAX 255

#define PIDFF_ENTRY(label, ptr, max) OSD_UINT16_ENTRY(label, (&(const OSD_UINT16_t){ ptr, PIDFF_MIN, max, PIDFF_STEP }))
#define RPY_PIDFF_ENTRY(label, ptr) PIDFF_ENTRY(label, ptr, RPY_PIDFF_MAX)
#define OTHER_PIDFF_ENTRY(label, ptr) PIDFF_ENTRY(label, ptr, OTHER_PIDDF_MAX)

static pid8_t cmsx_pidRoll;
static pid8_t cmsx_pidPitch;
static pid8_t cmsx_pidYaw;
static pid8_t cmsx_pidPosZ;
static pid8_t cmsx_pidVelZ;
static pid8_t cmsx_pidHead;
static pid8_t cmsx_pidPosXY;
static pid8_t cmsx_pidVelXY;

static uint8_t tmpProfileIndex;
static uint8_t profileIndex;
static char profileIndexString[] = " p";

static void cmsx_ReadPidToArray(pid8_t *dst, int pidIndex)
{
    memcpy(dst, &pidBank()->pid[pidIndex], sizeof(*dst));
}

static void cmsx_WritebackPidFromArray(const pid8_t *src, int pidIndex)
{
    memcpy(&pidBankMutable()->pid[pidIndex], src, sizeof(*src));
}

static long cmsx_menuImu_onEnter(const OSD_Entry *from)
{
    UNUSED(from);

    profileIndex = getConfigProfile();
    tmpProfileIndex = profileIndex + 1;
    profileIndexString[1] = '0' + tmpProfileIndex;

    return 0;
}

static long cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    profileIndex = tmpProfileIndex - 1;
    profileIndexString[1] = '0' + tmpProfileIndex;
    setConfigProfile(profileIndex);

    return 0;
}

static long cmsx_PidRead(void)
{
    cmsx_ReadPidToArray(&cmsx_pidRoll, PID_ROLL);
    cmsx_ReadPidToArray(&cmsx_pidPitch, PID_PITCH);
    cmsx_ReadPidToArray(&cmsx_pidYaw, PID_YAW);

    return 0;
}

static long cmsx_PidOnEnter(const OSD_Entry *from)
{
    UNUSED(from);

    profileIndexString[1] = '0' + tmpProfileIndex;
    cmsx_PidRead();

    return 0;
}

static long cmsx_PidWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_WritebackPidFromArray(&cmsx_pidRoll, PID_ROLL);
    cmsx_WritebackPidFromArray(&cmsx_pidPitch, PID_PITCH);
    cmsx_WritebackPidFromArray(&cmsx_pidYaw, PID_YAW);

    schedulePidGainsUpdate();

    return 0;
}

static const OSD_Entry cmsx_menuPidEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- PID --", profileIndexString),

    RPY_PIDFF_ENTRY("ROLL  P", &cmsx_pidRoll.P),
    RPY_PIDFF_ENTRY("ROLL  I", &cmsx_pidRoll.I),
    RPY_PIDFF_ENTRY("ROLL  D", &cmsx_pidRoll.D),
    RPY_PIDFF_ENTRY("ROLL  FF", &cmsx_pidRoll.FF),

    RPY_PIDFF_ENTRY("PITCH P", &cmsx_pidPitch.P),
    RPY_PIDFF_ENTRY("PITCH I", &cmsx_pidPitch.I),
    RPY_PIDFF_ENTRY("PITCH D", &cmsx_pidPitch.D),
    RPY_PIDFF_ENTRY("PITCH FF", &cmsx_pidPitch.FF),

    RPY_PIDFF_ENTRY("YAW   P", &cmsx_pidYaw.P),
    RPY_PIDFF_ENTRY("YAW   I", &cmsx_pidYaw.I),
    RPY_PIDFF_ENTRY("YAW   D", &cmsx_pidYaw.D),
    RPY_PIDFF_ENTRY("YAW   FF", &cmsx_pidYaw.FF),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidEntries
};

static long cmsx_menuPidAltMag_onEnter(const OSD_Entry *from)
{
    UNUSED(from);

    cmsx_ReadPidToArray(&cmsx_pidPosZ, PID_POS_Z);
    cmsx_ReadPidToArray(&cmsx_pidVelZ, PID_VEL_Z);
    cmsx_ReadPidToArray(&cmsx_pidHead, PID_HEADING);

    return 0;
}

static long cmsx_menuPidAltMag_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_WritebackPidFromArray(&cmsx_pidPosZ, PID_POS_Z);
    cmsx_WritebackPidFromArray(&cmsx_pidVelZ, PID_VEL_Z);
    cmsx_WritebackPidFromArray(&cmsx_pidHead, PID_HEADING);

    navigationUsePIDs();

    return 0;
}

static const OSD_Entry cmsx_menuPidAltMagEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- ALT&MAG --", profileIndexString),

    OTHER_PIDFF_ENTRY("ALT P", &cmsx_pidPosZ.P),
    OTHER_PIDFF_ENTRY("ALT I", &cmsx_pidPosZ.I),
    OTHER_PIDFF_ENTRY("ALT D", &cmsx_pidPosZ.D),

    OTHER_PIDFF_ENTRY("VEL P", &cmsx_pidVelZ.P),
    OTHER_PIDFF_ENTRY("VEL I", &cmsx_pidVelZ.I),
    OTHER_PIDFF_ENTRY("VEL D", &cmsx_pidVelZ.D),

    OTHER_PIDFF_ENTRY("MAG P", &cmsx_pidHead.P),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuPidAltMag = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XALTMAG",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuPidAltMag_onEnter,
    .onExit = cmsx_menuPidAltMag_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidAltMagEntries,
};

static long cmsx_menuPidGpsnav_onEnter(const OSD_Entry *from)
{
    UNUSED(from);

    cmsx_ReadPidToArray(&cmsx_pidPosXY, PID_POS_XY);
    cmsx_ReadPidToArray(&cmsx_pidVelXY, PID_VEL_XY);

    return 0;
}

static long cmsx_menuPidGpsnav_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_WritebackPidFromArray(&cmsx_pidPosXY, PID_POS_XY);
    cmsx_WritebackPidFromArray(&cmsx_pidVelXY, PID_VEL_XY);

    navigationUsePIDs();

    return 0;
}

static const OSD_Entry cmsx_menuPidGpsnavEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- GPSNAV --", profileIndexString),

    OTHER_PIDFF_ENTRY("POS  P", &cmsx_pidPosXY.P),
    OTHER_PIDFF_ENTRY("POS  I", &cmsx_pidPosXY.I),

    OTHER_PIDFF_ENTRY("POSR P", &cmsx_pidVelXY.P),
    OTHER_PIDFF_ENTRY("POSR I", &cmsx_pidVelXY.I),
    OTHER_PIDFF_ENTRY("POSR D", &cmsx_pidVelXY.D),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuPidGpsnav = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XGPSNAV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuPidGpsnav_onEnter,
    .onExit = cmsx_menuPidGpsnav_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidGpsnavEntries,
};

//
// MANUAL Rate & Expo
//
static const OSD_Entry cmsx_menuManualRateProfileEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- MANUAL RATE --", profileIndexString),

    OSD_SETTING_ENTRY("MANU ROLL RATE", SETTING_MANUAL_ROLL_RATE),
    OSD_SETTING_ENTRY("MANU PITCH RATE", SETTING_MANUAL_PITCH_RATE),
    OSD_SETTING_ENTRY("MANU YAW RATE", SETTING_MANUAL_YAW_RATE),

    OSD_SETTING_ENTRY("MANU RC EXPO", SETTING_MANUAL_RC_EXPO),
    OSD_SETTING_ENTRY("MANU RC YAW EXP", SETTING_MANUAL_RC_YAW_EXPO),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuManualRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUMANURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuManualRateProfileEntries
};

//
// Rate & Expo
//
static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- RATE --", profileIndexString),

#if 0
    { "RC RATE",     OME_FLOAT,  NULL, &(OSD_FLOAT_t){ &rateProfile.rcRate8,    0, 255, 1, 10 }, 0 },
    { "RC YAW RATE", OME_FLOAT,  NULL, &(OSD_FLOAT_t){ &rateProfile.rcYawRate8, 0, 255, 1, 10 }, 0 },
#endif

    OSD_SETTING_ENTRY_TYPE("ROLL RATE", SETTING_ROLL_RATE, CMS_DATA_TYPE_ANGULAR_RATE),
    OSD_SETTING_ENTRY_TYPE("PITCH RATE", SETTING_PITCH_RATE, CMS_DATA_TYPE_ANGULAR_RATE),
    OSD_SETTING_ENTRY_TYPE("YAW RATE", SETTING_YAW_RATE, CMS_DATA_TYPE_ANGULAR_RATE),

    OSD_SETTING_ENTRY("RC EXPO", SETTING_RC_EXPO),
    OSD_SETTING_ENTRY("RC YAW EXP", SETTING_RC_YAW_EXPO),

    OSD_SETTING_ENTRY("THR MID", SETTING_THR_MID),
    OSD_SETTING_ENTRY("THR EXPO", SETTING_THR_EXPO),

    OSD_SETTING_ENTRY("THRPID ATT", SETTING_TPA_RATE),
    OSD_SETTING_ENTRY_STEP("TPA BRKPT", SETTING_TPA_BREAKPOINT, 10),

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuRateProfileEntries
};

#ifdef NOT_YET
static uint8_t cmsx_dtermSetpointWeight;
static uint8_t cmsx_setpointRelaxRatio;
static uint8_t cmsx_angleStrength;
static uint8_t cmsx_horizonStrength;
static uint8_t cmsx_horizonTransition;

static long cmsx_profileOtherOnEnter(void)
{
    profileIndexString[1] = '0' + tmpProfileIndex;

    cmsx_dtermSetpointWeight = pidProfile()->dtermSetpointWeight;
    cmsx_setpointRelaxRatio  = pidProfile()->setpointRelaxRatio;

    cmsx_angleStrength       = pidProfile()[PIDLEVEL].P;
    cmsx_horizonStrength     = pidProfile()[PIDLEVEL].I;
    cmsx_horizonTransition   = pidProfile()[PIDLEVEL].D;

    return 0;
}

static long cmsx_profileOtherOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfileMutable()->dtermSetpointWeight = cmsx_dtermSetpointWeight;
    pidProfileMutable()->setpointRelaxRatio  = cmsx_setpointRelaxRatio;

    pidProfileMutable()[PIDLEVEL].P        = cmsx_angleStrength;
    pidProfileMutable()[PIDLEVEL].I        = cmsx_horizonStrength;
    pidProfileMutable()[PIDLEVEL].D        = cmsx_horizonTransition;

    return 0;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, profileIndexString, 0 },

    { "D SETPT WT",  OME_FLOAT, NULL, &(OSD_FLOAT_t){ &cmsx_dtermSetpointWeight, 0, 255, 1, 10 }, 0 },
    { "SETPT TRS",   OME_FLOAT, NULL, &(OSD_FLOAT_t){ &cmsx_setpointRelaxRatio,  0, 100, 1, 10 }, 0 },
    { "ANGLE STR",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_angleStrength,       0, 200, 1 }    , 0 },
    { "HORZN STR",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_horizonStrength,     0, 200, 1 }    , 0 },
    { "HORZN TRS",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_horizonTransition,   0, 200, 1 }    , 0 },

    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuProfileOther = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};
#endif // NOT_YET

//
// Per profile filters
//
static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- FILTERING  --", profileIndexString),
    OSD_SETTING_ENTRY("HARDWARE LPF", SETTING_GYRO_HARDWARE_LPF),
    OSD_SETTING_ENTRY("GYRO MAIN", SETTING_GYRO_MAIN_LPF_HZ),
    OSD_SETTING_ENTRY("DTERM LPF", SETTING_DTERM_LPF_HZ),
    OSD_SETTING_ENTRY("DTERM LPF2", SETTING_DTERM_LPF2_HZ),
#ifdef USE_DYNAMIC_FILTERS
    OSD_SETTING_ENTRY("MATRIX FILTER", SETTING_DYNAMIC_GYRO_NOTCH_ENABLED),
    OSD_SETTING_ENTRY("MATRIX MIN HZ", SETTING_DYNAMIC_GYRO_NOTCH_MIN_HZ),  //dynamic_gyro_notch_min_hz
    OSD_SETTING_ENTRY("MATRIX Q", SETTING_DYNAMIC_GYRO_NOTCH_Q),            //dynamic_gyro_notch_q
#endif
#ifdef USE_GYRO_KALMAN
    OSD_SETTING_ENTRY("UNICORN FILTER", SETTING_SETPOINT_KALMAN_ENABLED),   //setpoint_kalman_enabled
    OSD_SETTING_ENTRY("UNICORN Q", SETTING_SETPOINT_KALMAN_Q),              //setpoint_kalman_q
#endif
    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFilterPerProfileEntries,
};

static const OSD_Entry cmsx_menuMechanicsEntries[] =
{
    OSD_LABEL_DATA_ENTRY("-- MECHANICS --", profileIndexString),
#ifdef USE_D_BOOST
    OSD_SETTING_ENTRY("DBOOST_FACTOR", SETTING_D_BOOST_FACTOR),
#endif
#ifdef USE_ANTIGRAVITY
    OSD_SETTING_ENTRY("ANTIGRAV. GAIN", SETTING_ANTIGRAVITY_GAIN),
#endif
    OSD_SETTING_ENTRY("ITERM RELAX", SETTING_MC_ITERM_RELAX),
    OSD_SETTING_ENTRY("ITERM CUTOFF", SETTING_MC_ITERM_RELAX_CUTOFF),
    OSD_SETTING_ENTRY("CD LPF", SETTING_MC_CD_LPF_HZ),
 
    OSD_BACK_AND_END_ENTRY,
};

static const CMS_Menu cmsx_menuMechanics = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XGYROGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuMechanicsEntries,
};

static const OSD_Entry cmsx_menuImuEntries[] =
{
    OSD_LABEL_ENTRY("-- PID TUNING --"),

    // Profile dependent
    OSD_UINT8_CALLBACK_ENTRY("PID PROF", cmsx_profileIndexOnChange, (&(const OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_PROFILE_COUNT, 1})),
    OSD_SUBMENU_ENTRY("PID", &cmsx_menuPid),
    OSD_SUBMENU_ENTRY("PID ALTMAG", &cmsx_menuPidAltMag),
    OSD_SUBMENU_ENTRY("PID GPSNAV", &cmsx_menuPidGpsnav),
    OSD_SUBMENU_ENTRY("FILTERING", &cmsx_menuFilterPerProfile),
    OSD_SUBMENU_ENTRY("MECHANICS",  &cmsx_menuMechanics),

    // Rate profile dependent
    OSD_UINT8_CALLBACK_ENTRY("RATE PROF", cmsx_profileIndexOnChange, (&(const OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_CONTROL_RATE_PROFILE_COUNT, 1})),
    OSD_SUBMENU_ENTRY("RATE", &cmsx_menuRateProfile),
    OSD_SUBMENU_ENTRY("MANU RATE", &cmsx_menuManualRateProfile),

    // Global

#ifdef NOT_YET
    {"OTHER PP",  OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                      0},
    // Profile independent
    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                      0},
#endif

    OSD_BACK_AND_END_ENTRY,
};

const CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuImuEntries,
};
#endif // CMS
