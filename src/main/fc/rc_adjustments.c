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

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_curves.h"
#include "fc/settings.h"

#include "navigation/navigation.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/vtx.h"

#include "sensors/boardalignment.h"

#include "rx/rx.h"

PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);

static uint8_t adjustmentStateMask = 0;

#define MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex) adjustmentStateMask |= (1 << adjustmentIndex)
#define MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex) adjustmentStateMask &= ~(1 << adjustmentIndex)

#define IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex) (adjustmentStateMask & (1 << adjustmentIndex))

// sync with adjustmentFunction_e
static const adjustmentConfig_t defaultAdjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT - 1] = {
    {
        .adjustmentFunction = ADJUSTMENT_RC_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_THROTTLE_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_FF,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_FF,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_FF,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_FF,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_YAW_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_RC_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_RC_YAW_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_PITCH_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_PITCH_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_MANUAL_YAW_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_NAV_FW_CRUISE_THR,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 10 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_NAV_FW_PITCH2THR,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_BOARD_ALIGNMENT,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 5 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_BOARD_ALIGNMENT,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 5 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_LEVEL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_LEVEL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_LEVEL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_XY_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_XY_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_XY_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_Z_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_Z_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_POS_Z_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_HEADING_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_XY_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_XY_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_XY_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_Z_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_Z_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VEL_Z_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 5 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_VTX_POWER_LEVEL,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_TPA,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_TPA_BREAKPOINT,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 5 }}
    }, {
        .adjustmentFunction = ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }
};

#define ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET 1

static adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

static void configureAdjustment(uint8_t index, uint8_t auxSwitchChannelIndex, const adjustmentConfig_t *adjustmentConfig)
{
    adjustmentState_t * const adjustmentState = &adjustmentStates[index];

    if (adjustmentState->config == adjustmentConfig) {
        // already configured
        return;
    }
    adjustmentState->auxChannelIndex = auxSwitchChannelIndex;
    adjustmentState->config = adjustmentConfig;
    adjustmentState->timeoutAt = 0;

    MARK_ADJUSTMENT_FUNCTION_AS_READY(index);
}

static void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

#if 0
static void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newFloatValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newFloatValue = newFloatValue;
        eventData.floatFlag = true;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}
#endif

static void applyAdjustmentU8(adjustmentFunction_e adjustmentFunction, uint8_t *val, int delta, int low, int high)
{
    int newValue = constrain((int)(*val) + delta, low, high);
    *val = newValue;
    blackboxLogInflightAdjustmentEvent(adjustmentFunction, newValue);
}

static void applyAdjustmentU16(adjustmentFunction_e adjustmentFunction, uint16_t *val, int delta, int low, int high)
{
    int newValue = constrain((int)(*val) + delta, low, high);
    *val = newValue;
    blackboxLogInflightAdjustmentEvent(adjustmentFunction, newValue);
}

static void applyAdjustmentExpo(adjustmentFunction_e adjustmentFunction, uint8_t *val, int delta)
{
    applyAdjustmentU8(adjustmentFunction, val, delta, SETTING_RC_EXPO_MIN, SETTING_RC_EXPO_MAX);
}

static void applyAdjustmentManualRate(adjustmentFunction_e adjustmentFunction, uint8_t *val, int delta)
{
    return applyAdjustmentU8(adjustmentFunction, val, delta, SETTING_CONSTANT_MANUAL_RATE_MIN, SETTING_CONSTANT_MANUAL_RATE_MAX);
}

static void applyAdjustmentPID(adjustmentFunction_e adjustmentFunction, uint16_t *val, int delta)
{
    applyAdjustmentU16(adjustmentFunction, val, delta, SETTING_CONSTANT_RPYL_PID_MIN, SETTING_CONSTANT_RPYL_PID_MAX);
}

static void applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta)
{
    if (delta > 0) {
        beeperConfirmationBeeps(2);
    } else {
        beeperConfirmationBeeps(1);
    }
    switch (adjustmentFunction) {
        case ADJUSTMENT_RC_EXPO:
            applyAdjustmentExpo(ADJUSTMENT_RC_EXPO, &controlRateConfig->stabilized.rcExpo8, delta);
            break;
        case ADJUSTMENT_RC_YAW_EXPO:
            applyAdjustmentExpo(ADJUSTMENT_RC_YAW_EXPO, &controlRateConfig->stabilized.rcYawExpo8, delta);
            break;
        case ADJUSTMENT_MANUAL_RC_EXPO:
            applyAdjustmentExpo(ADJUSTMENT_MANUAL_RC_EXPO, &controlRateConfig->manual.rcExpo8, delta);
            break;
        case ADJUSTMENT_MANUAL_RC_YAW_EXPO:
            applyAdjustmentExpo(ADJUSTMENT_MANUAL_RC_YAW_EXPO, &controlRateConfig->manual.rcYawExpo8, delta);
            break;
        case ADJUSTMENT_THROTTLE_EXPO:
            applyAdjustmentExpo(ADJUSTMENT_THROTTLE_EXPO, &controlRateConfig->throttle.rcExpo8, delta);
            break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
        case ADJUSTMENT_PITCH_RATE:
            applyAdjustmentU8(ADJUSTMENT_PITCH_RATE, &controlRateConfig->stabilized.rates[FD_PITCH], delta, SETTING_PITCH_RATE_MIN, SETTING_PITCH_RATE_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
                schedulePidGainsUpdate();
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_RATE
            FALLTHROUGH;

        case ADJUSTMENT_ROLL_RATE:
            applyAdjustmentU8(ADJUSTMENT_ROLL_RATE, &controlRateConfig->stabilized.rates[FD_ROLL], delta, SETTING_CONSTANT_ROLL_PITCH_RATE_MIN, SETTING_CONSTANT_ROLL_PITCH_RATE_MAX);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_MANUAL_PITCH_ROLL_RATE:
        case ADJUSTMENT_MANUAL_ROLL_RATE:
            applyAdjustmentManualRate(ADJUSTMENT_MANUAL_ROLL_RATE, &controlRateConfig->manual.rates[FD_ROLL], delta);
            if (adjustmentFunction == ADJUSTMENT_MANUAL_ROLL_RATE)
                break;
            // follow though for combined ADJUSTMENT_MANUAL_PITCH_ROLL_RATE
            FALLTHROUGH;
        case ADJUSTMENT_MANUAL_PITCH_RATE:
            applyAdjustmentManualRate(ADJUSTMENT_MANUAL_PITCH_RATE, &controlRateConfig->manual.rates[FD_PITCH], delta);
            break;
        case ADJUSTMENT_YAW_RATE:
            applyAdjustmentU8(ADJUSTMENT_YAW_RATE, &controlRateConfig->stabilized.rates[FD_YAW], delta, SETTING_YAW_RATE_MIN, SETTING_YAW_RATE_MAX);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_MANUAL_YAW_RATE:
            applyAdjustmentManualRate(ADJUSTMENT_MANUAL_YAW_RATE, &controlRateConfig->manual.rates[FD_YAW], delta);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
        case ADJUSTMENT_PITCH_P:
            applyAdjustmentPID(ADJUSTMENT_PITCH_P, &pidBankMutable()->pid[PID_PITCH].P, delta);
            if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
                schedulePidGainsUpdate();
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_P
            FALLTHROUGH;

        case ADJUSTMENT_ROLL_P:
            applyAdjustmentPID(ADJUSTMENT_ROLL_P, &pidBankMutable()->pid[PID_ROLL].P, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
        case ADJUSTMENT_PITCH_I:
            applyAdjustmentPID(ADJUSTMENT_PITCH_I, &pidBankMutable()->pid[PID_PITCH].I, delta);
            if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
                schedulePidGainsUpdate();
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_I
            FALLTHROUGH;

        case ADJUSTMENT_ROLL_I:
            applyAdjustmentPID(ADJUSTMENT_ROLL_I, &pidBankMutable()->pid[PID_ROLL].I, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
        case ADJUSTMENT_PITCH_D:
            applyAdjustmentPID(ADJUSTMENT_PITCH_D, &pidBankMutable()->pid[PID_PITCH].D, delta);
            if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
                schedulePidGainsUpdate();
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_D
            FALLTHROUGH;

        case ADJUSTMENT_ROLL_D:
            applyAdjustmentPID(ADJUSTMENT_ROLL_D, &pidBankMutable()->pid[PID_ROLL].D, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_PITCH_ROLL_FF:
        case ADJUSTMENT_PITCH_FF:
            applyAdjustmentPID(ADJUSTMENT_PITCH_FF, &pidBankMutable()->pid[PID_PITCH].FF, delta);
            if (adjustmentFunction == ADJUSTMENT_PITCH_FF) {
                schedulePidGainsUpdate();
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_FF
            FALLTHROUGH;

        case ADJUSTMENT_ROLL_FF:
            applyAdjustmentPID(ADJUSTMENT_ROLL_FF, &pidBankMutable()->pid[PID_ROLL].FF, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_YAW_P:
            applyAdjustmentPID(ADJUSTMENT_YAW_P, &pidBankMutable()->pid[PID_YAW].P, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_YAW_I:
            applyAdjustmentPID(ADJUSTMENT_YAW_I, &pidBankMutable()->pid[PID_YAW].I, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_YAW_D:
            applyAdjustmentPID(ADJUSTMENT_YAW_D, &pidBankMutable()->pid[PID_YAW].D, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_YAW_FF:
            applyAdjustmentPID(ADJUSTMENT_YAW_FF, &pidBankMutable()->pid[PID_YAW].FF, delta);
            schedulePidGainsUpdate();
            break;
        case ADJUSTMENT_NAV_FW_CRUISE_THR:
            applyAdjustmentU16(ADJUSTMENT_NAV_FW_CRUISE_THR, &navConfigMutable()->fw.cruise_throttle, delta, SETTING_NAV_FW_CRUISE_THR_MIN, SETTING_NAV_FW_CRUISE_THR_MAX);
            break;
        case ADJUSTMENT_NAV_FW_PITCH2THR:
            applyAdjustmentU8(ADJUSTMENT_NAV_FW_PITCH2THR, &navConfigMutable()->fw.pitch_to_throttle, delta, SETTING_NAV_FW_PITCH2THR_MIN, SETTING_NAV_FW_PITCH2THR_MAX);
            break;
        case ADJUSTMENT_ROLL_BOARD_ALIGNMENT:
            updateBoardAlignment(delta, 0);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_BOARD_ALIGNMENT, boardAlignment()->rollDeciDegrees);
            break;
        case ADJUSTMENT_PITCH_BOARD_ALIGNMENT:
            updateBoardAlignment(0, delta);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_BOARD_ALIGNMENT, boardAlignment()->pitchDeciDegrees);
            break;
        case ADJUSTMENT_LEVEL_P:
            applyAdjustmentPID(ADJUSTMENT_LEVEL_P, &pidBankMutable()->pid[PID_LEVEL].P, delta);
            // TODO: Need to call something to take it into account?
            break;
        case ADJUSTMENT_LEVEL_I:
            applyAdjustmentPID(ADJUSTMENT_LEVEL_I, &pidBankMutable()->pid[PID_LEVEL].I, delta);
            // TODO: Need to call something to take it into account?
            break;
        case ADJUSTMENT_LEVEL_D:
            applyAdjustmentPID(ADJUSTMENT_LEVEL_D, &pidBankMutable()->pid[PID_LEVEL].D, delta);
            // TODO: Need to call something to take it into account?
            break;
        case ADJUSTMENT_POS_XY_P:
            applyAdjustmentPID(ADJUSTMENT_POS_XY_P, &pidBankMutable()->pid[PID_POS_XY].P, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_POS_XY_I:
            applyAdjustmentPID(ADJUSTMENT_POS_XY_I, &pidBankMutable()->pid[PID_POS_XY].I, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_POS_XY_D:
            applyAdjustmentPID(ADJUSTMENT_POS_XY_D, &pidBankMutable()->pid[PID_POS_XY].D, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_POS_Z_P:
            applyAdjustmentPID(ADJUSTMENT_POS_Z_P, &pidBankMutable()->pid[PID_POS_Z].P, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_POS_Z_I:
            applyAdjustmentPID(ADJUSTMENT_POS_Z_I, &pidBankMutable()->pid[PID_POS_Z].I, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_POS_Z_D:
            applyAdjustmentPID(ADJUSTMENT_POS_Z_D, &pidBankMutable()->pid[PID_POS_Z].D, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_HEADING_P:
            applyAdjustmentPID(ADJUSTMENT_HEADING_P, &pidBankMutable()->pid[PID_HEADING].P, delta);
            // TODO: navigationUsePIDs()?
            break;
        case ADJUSTMENT_VEL_XY_P:
            applyAdjustmentPID(ADJUSTMENT_VEL_XY_P, &pidBankMutable()->pid[PID_VEL_XY].P, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_VEL_XY_I:
            applyAdjustmentPID(ADJUSTMENT_VEL_XY_I, &pidBankMutable()->pid[PID_VEL_XY].I, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_VEL_XY_D:
            applyAdjustmentPID(ADJUSTMENT_VEL_XY_D, &pidBankMutable()->pid[PID_VEL_XY].D, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_VEL_Z_P:
            applyAdjustmentPID(ADJUSTMENT_VEL_Z_P, &pidBankMutable()->pid[PID_VEL_Z].P, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_VEL_Z_I:
            applyAdjustmentPID(ADJUSTMENT_VEL_Z_I, &pidBankMutable()->pid[PID_VEL_Z].I, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_VEL_Z_D:
            applyAdjustmentPID(ADJUSTMENT_VEL_Z_D, &pidBankMutable()->pid[PID_VEL_Z].D, delta);
            navigationUsePIDs();
            break;
        case ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE:
            applyAdjustmentU16(ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE, &mixerConfigMutable()->fwMinThrottleDownPitchAngle, delta, SETTING_FW_MIN_THROTTLE_DOWN_PITCH_MIN, SETTING_FW_MIN_THROTTLE_DOWN_PITCH_MAX);
            break;
#if defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
        case ADJUSTMENT_VTX_POWER_LEVEL:
            {
                vtxDeviceCapability_t vtxDeviceCapability;
                if (vtxCommonGetDeviceCapability(vtxCommonDevice(), &vtxDeviceCapability)) {
                    applyAdjustmentU8(ADJUSTMENT_VTX_POWER_LEVEL, &vtxSettingsConfigMutable()->power, delta, VTX_SETTINGS_MIN_POWER, vtxDeviceCapability.powerCount);
                }
            }
            break;
#endif
        case ADJUSTMENT_TPA:
            applyAdjustmentU8(ADJUSTMENT_TPA, &controlRateConfig->throttle.dynPID, delta, 0, SETTING_TPA_RATE_MAX);
            break;
        case ADJUSTMENT_TPA_BREAKPOINT:
            applyAdjustmentU16(ADJUSTMENT_TPA_BREAKPOINT, &controlRateConfig->throttle.pa_breakpoint, delta, PWM_RANGE_MIN, PWM_RANGE_MAX);
            break;
        case ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS:
            applyAdjustmentU8(ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS, &navConfigMutable()->fw.control_smoothness, delta, SETTING_NAV_FW_CONTROL_SMOOTHNESS_MIN, SETTING_NAV_FW_CONTROL_SMOOTHNESS_MAX);
            break;
        default:
            break;
    };
}

#ifdef USE_INFLIGHT_PROFILE_ADJUSTMENT
static void applySelectAdjustment(uint8_t adjustmentFunction, uint8_t position)
{
    bool applied = false;

    switch (adjustmentFunction) {
        case ADJUSTMENT_RATE_PROFILE:
            if (getCurrentControlRateProfile() != position) {
                changeControlRateProfile(position);
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);
                applied = true;
            }
            break;
    }

    if (applied) {
        beeperConfirmationBeeps(position + 1);
    }
}
#endif

#define RESET_FREQUENCY_2HZ (1000 / 2)

void processRcAdjustments(controlRateConfig_t *controlRateConfig, bool canUseRxData)
{
    const uint32_t now = millis();

    for (int adjustmentIndex = 0; adjustmentIndex < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; adjustmentIndex++) {
        adjustmentState_t * const adjustmentState = &adjustmentStates[adjustmentIndex];

        if (!adjustmentState->config) {
            continue;
        }
        const uint8_t adjustmentFunction = adjustmentState->config->adjustmentFunction;
        if (adjustmentFunction == ADJUSTMENT_NONE) {
            continue;
        }

        const int32_t signedDiff = now - adjustmentState->timeoutAt;
        const bool canResetReadyStates = signedDiff >= 0L;

        if (canResetReadyStates) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
        }

        if (!canUseRxData) {
            continue;
        }

        const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentState->auxChannelIndex;

        if (adjustmentState->config->mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rxGetChannelValue(channelIndex) > PWM_RANGE_MIDDLE + 200) {
                delta = adjustmentState->config->data.stepConfig.step;
            } else if (rxGetChannelValue(channelIndex) < PWM_RANGE_MIDDLE - 200) {
                delta = 0 - adjustmentState->config->data.stepConfig.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
                continue;
            }

            // it is legitimate to adjust an otherwise const item here
            applyStepAdjustment(controlRateConfig, adjustmentFunction, delta);
#ifdef USE_INFLIGHT_PROFILE_ADJUSTMENT
        } else if (adjustmentState->config->mode == ADJUSTMENT_MODE_SELECT) {
            const uint16_t rangeWidth = ((2100 - 900) / adjustmentState->config->data.selectConfig.switchPositions);
            const uint8_t position = (constrain(rxGetChannelValue(channelIndex), 900, 2100 - 1) - 900) / rangeWidth;

            applySelectAdjustment(adjustmentFunction, position);
#endif
        }
        MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex);
    }
}

void resetAdjustmentStates(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}

void updateAdjustmentStates(bool canUseRxData)
{
    for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        const adjustmentRange_t * const adjustmentRange = adjustmentRanges(index);
        if (adjustmentRange->adjustmentFunction == ADJUSTMENT_NONE) {
            // Range not set up
            continue;
        }
        const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentFunction - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        adjustmentState_t * const adjustmentState = &adjustmentStates[adjustmentRange->adjustmentIndex];

        if (canUseRxData && isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range)) {
            if (!adjustmentState->config) {
                configureAdjustment(adjustmentRange->adjustmentIndex, adjustmentRange->auxSwitchChannelIndex, adjustmentConfig);
            }
        } else {
            if (adjustmentState->config == adjustmentConfig) {
                adjustmentState->config = NULL;
            }
        }
    }
}

bool isAdjustmentFunctionSelected(uint8_t adjustmentFunction) {
    for (uint8_t index = 0; index < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; ++index) {
        if (adjustmentStates[index].config && adjustmentStates[index].config->adjustmentFunction == adjustmentFunction) {
            return true;
        }
    }
    return false;
}
