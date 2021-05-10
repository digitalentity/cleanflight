/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/vtx_common.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/failsafe.h"

#include "io/vtx.h"
#include "io/vtx_string.h"
#include "io/vtx_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 2);

PG_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig,
    .band = SETTING_VTX_BAND_DEFAULT,
    .channel = SETTING_VTX_CHANNEL_DEFAULT,
    .power = SETTING_VTX_POWER_DEFAULT,
    .pitModeChan = SETTING_VTX_PIT_MODE_CHAN_DEFAULT,
    .lowPowerDisarm = SETTING_VTX_LOW_POWER_DISARM_DEFAULT,
    .maxPowerOverride = SETTING_VTX_MAX_POWER_OVERRIDE_DEFAULT,
);

typedef enum {
    VTX_PARAM_POWER = 0,
    VTX_PARAM_BANDCHAN,
    VTX_PARAM_PITMODE,
    VTX_PARAM_COUNT
} vtxScheduleParams_e;

void vtxInit(void)
{
}

static vtxSettingsConfig_t * vtxGetRuntimeSettings(void)
{
    static vtxSettingsConfig_t settings;

    settings.band = vtxSettingsConfig()->band;
    settings.channel = vtxSettingsConfig()->channel;
    settings.power = vtxSettingsConfig()->power;
    settings.pitModeChan = vtxSettingsConfig()->pitModeChan;
    settings.lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm;

    if (!ARMING_FLAG(ARMED) && !failsafeIsActive() &&
        ((settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_ALWAYS) ||
        (settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM && !ARMING_FLAG(WAS_EVER_ARMED)))) {

        settings.power = VTX_SETTINGS_DEFAULT_POWER;
    }

    return &settings;
}

static bool vtxProcessBandAndChannel(vtxDevice_t *vtxDevice, const vtxSettingsConfig_t * runtimeSettings)
{
    uint8_t vtxBand;
    uint8_t vtxChan;

    // Shortcut for undefined band
    if (!runtimeSettings->band) {
        return false;
    }

    if (!vtxCommonGetBandAndChannel(vtxDevice, &vtxBand, &vtxChan)) {
        return false;
    }

    if (vtxBand != runtimeSettings->band || vtxChan != runtimeSettings->channel) {
        vtxCommonSetBandAndChannel(vtxDevice, runtimeSettings->band, runtimeSettings->channel);
        return true;
    }

    return false;
}

static bool vtxProcessPower(vtxDevice_t *vtxDevice, const vtxSettingsConfig_t * runtimeSettings)
{
    uint8_t vtxPower;

    if (!vtxCommonGetPowerIndex(vtxDevice, &vtxPower)) {
        return false;
    }

    if (vtxPower != runtimeSettings->power) {
        vtxCommonSetPowerByIndex(vtxDevice, runtimeSettings->power);
        return true;
    }

    return false;
}

static bool vtxProcessPitMode(vtxDevice_t *vtxDevice, const vtxSettingsConfig_t * runtimeSettings)
{
    UNUSED(runtimeSettings);

    uint8_t pitOnOff;

    bool        currPmSwitchState = false;
    static bool prevPmSwitchState = false;

    if (!vtxCommonGetPitMode(vtxDevice, &pitOnOff)) {
        return false;
    }

    if (currPmSwitchState != prevPmSwitchState) {
        prevPmSwitchState = currPmSwitchState;

        if (currPmSwitchState) {
            if (0) {
                if (!pitOnOff) {
                    vtxCommonSetPitMode(vtxDevice, true);
                    return true;
                }
            }
        } else {
            if (pitOnOff) {
                vtxCommonSetPitMode(vtxDevice, false);
                return true;
            }
        }
    }

    return false;
}

void vtxUpdate(timeUs_t currentTimeUs)
{
    static uint8_t currentSchedule = 0;

    if (cliMode) {
        return;
    }

    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        // Check input sources for config updates
        vtxControlInputPoll();

        // Build runtime settings
        const vtxSettingsConfig_t * runtimeSettings = vtxGetRuntimeSettings();

        switch (currentSchedule) {
            case VTX_PARAM_POWER:
                vtxProcessPower(vtxDevice, runtimeSettings);
                break;
            case VTX_PARAM_BANDCHAN:
                vtxProcessBandAndChannel(vtxDevice, runtimeSettings);
                break;
            case VTX_PARAM_PITMODE:
                vtxProcessPitMode(vtxDevice, runtimeSettings);
                break;
            default:
                break;
        }

        vtxCommonProcess(vtxDevice, currentTimeUs);

        currentSchedule = (currentSchedule + 1) % VTX_PARAM_COUNT;
    }
}
