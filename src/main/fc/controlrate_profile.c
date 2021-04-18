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

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_curves.h"
#include "fc/settings.h"

const controlRateConfig_t *currentControlRateProfile;


PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 3);

void pgResetFn_controlRateProfiles(controlRateConfig_t *instance)
{
    for (int i = 0; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &instance[i],
            .throttle = {
                .rcMid8 = SETTING_THR_MID_DEFAULT,
                .rcExpo8 = SETTING_THR_EXPO_DEFAULT,
                .dynPID = SETTING_TPA_RATE_DEFAULT,
                .pa_breakpoint = SETTING_TPA_BREAKPOINT_DEFAULT,
                .fixedWingTauMs = SETTING_FW_TPA_TIME_CONSTANT_DEFAULT
            },

            .stabilized = {
                .rcExpo8 = SETTING_RC_EXPO_DEFAULT,
                .rcYawExpo8 = SETTING_RC_YAW_EXPO_DEFAULT,
                .rates[FD_ROLL] = SETTING_ROLL_RATE_DEFAULT,
                .rates[FD_PITCH] = SETTING_PITCH_RATE_DEFAULT,
                .rates[FD_YAW] = SETTING_YAW_RATE_DEFAULT,
            },

            .manual = {
                .rcExpo8 = SETTING_MANUAL_RC_EXPO_DEFAULT,
                .rcYawExpo8 = SETTING_MANUAL_RC_YAW_EXPO_DEFAULT,
                .rates[FD_ROLL] = SETTING_MANUAL_ROLL_RATE_DEFAULT,
                .rates[FD_PITCH] = SETTING_MANUAL_PITCH_RATE_DEFAULT,
                .rates[FD_YAW] = SETTING_MANUAL_YAW_RATE_DEFAULT
            },

            .misc = {
                .fpvCamAngleDegrees = SETTING_FPV_MIX_DEGREES_DEFAULT
            }
        );
    }
}

void setControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = 0;
    }
    currentControlRateProfile = controlRateProfiles(profileIndex);
}

void activateControlRateConfig(void)
{
    generateRCThrottleCurve(currentControlRateProfile);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}
