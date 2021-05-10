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


// Get target build configuration
#include "platform.h"

#include "common/maths.h"

#include "config/config_eeprom.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/vtx_common.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "io/beeper.h"
#include "io/osd.h"
#include "io/vtx_control.h"


#if defined(USE_VTX_CONTROL)

PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 3);

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
      .halfDuplex = SETTING_VTX_HALFDUPLEX_DEFAULT,
      .smartAudioEarlyAkkWorkaroundEnable = SETTING_VTX_SMARTAUDIO_EARLY_AKK_WORKAROUND_DEFAULT,
);

static uint8_t locked = 0;

void vtxControlInit(void)
{
    // NOTHING TO DO
}

void vtxControlInputPoll(void)
{
  // Check variuos input sources for VTX config updates

  // XXX: None supported in INAV
}

static void vtxUpdateBandAndChannel(uint8_t bandStep, uint8_t channelStep)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (!locked) {
        vtxDevice_t *vtxDevice = vtxCommonDevice();
        if (vtxDevice) {
            uint8_t band = 0, channel = 0;
            vtxCommonGetBandAndChannel(vtxDevice, &band, &channel);
            vtxCommonSetBandAndChannel(vtxDevice, band + bandStep, channel + channelStep);
        }
    }
}

void vtxIncrementBand(void)
{
    vtxUpdateBandAndChannel(+1, 0);
}

void vtxDecrementBand(void)
{
    vtxUpdateBandAndChannel(-1, 0);
}

void vtxIncrementChannel(void)
{
    vtxUpdateBandAndChannel(0, +1);
}

void vtxDecrementChannel(void)
{
    vtxUpdateBandAndChannel(0, -1);
}

void vtxUpdateActivatedChannel(void)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (!locked) {
        vtxDevice_t *vtxDevice = vtxCommonDevice();
        if (vtxDevice) {
            static uint8_t lastIndex = -1;

            for (uint8_t index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
                const vtxChannelActivationCondition_t *vtxChannelActivationCondition = &vtxConfig()->vtxChannelActivationConditions[index];

                if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                    && index != lastIndex) {
                    lastIndex = index;

                    vtxCommonSetBandAndChannel(vtxDevice, vtxChannelActivationCondition->band, vtxChannelActivationCondition->channel);
                    break;
                }
            }
        }
    }
}

void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep)
{
    vtxDevice_t *vtxDevice = vtxCommonDevice();

    if (!vtxDevice) {
        return;
    }

    uint8_t band = 0, channel = 0;
    vtxDeviceCapability_t capability;

    bool haveAllNeededInfo = vtxCommonGetBandAndChannel(vtxDevice, &band, &channel) && vtxCommonGetDeviceCapability(vtxDevice, &capability);
    if (!haveAllNeededInfo) {
        return;
    }

    int newChannel = channel + channelStep;
    if (newChannel > capability.channelCount) {
        newChannel = 1;
    } else if (newChannel < 1) {
        newChannel = capability.channelCount;
    }

    int newBand = band + bandStep;
    if (newBand > capability.bandCount) {
        newBand = 1;
    } else if (newBand < 1) {
        newBand = capability.bandCount;
    }

    vtxCommonSetBandAndChannel(vtxDevice, newBand, newChannel);
}

void vtxCyclePower(const uint8_t powerStep)
{
    vtxDevice_t *vtxDevice = vtxCommonDevice();

    if (!vtxDevice) {
        return;
    }

    uint8_t power = 0;
    vtxDeviceCapability_t capability;

    bool haveAllNeededInfo = vtxCommonGetPowerIndex(vtxDevice, &power) && vtxCommonGetDeviceCapability(vtxDevice, &capability);
    if (!haveAllNeededInfo) {
        return;
    }

    int newPower = power + powerStep;
    if (newPower >= capability.powerCount) {
        newPower = 0;
    } else if (newPower < 0) {
        newPower = capability.powerCount;
    }

    vtxCommonSetPowerByIndex(vtxDevice, newPower);
}

#endif
