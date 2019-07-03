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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/axis.h"
#include "common/maths.h"

#include "fc/controlrate_profile.h"
#include "fc/rc_control.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

static rcControl_t control;
static rcControlBidirThrottleMode_e bidirThrottleMode;

static float getAxisRcCommand(float input, uint8_t expo, float deadband)
{
    // XXX: As long as we make sure expo is in [0, 100], there's no need to
    // constrain here, since the only caller is rcControlUpdateFromRX()
    // which already constrains the input.
    // TODO: Precompute expo/100.0f
    float stickDeflection = fapplyDeadbandf(input, deadband) / (RC_COMMAND_MAX - deadband);
    return rcCurveApplyExpo(stickDeflection, expo / 100.0f);
}

static bool throttleIsBidirectional(void)
{
    switch (bidirThrottleMode) {
        case RC_CONTROL_BIDIR_THR_USER:
            return IS_RC_MODE_ACTIVE(BOXBIDIRTHR);
        case RC_CONTROL_BIDIR_THR_ON:
            return true;
        case RC_CONTROL_BIDIR_THR_OFF:
            return false;
    }
    return false;
}

void rcControlUpdateFromRX(void)
{
    const controlRateAxes_t *axes = FLIGHT_MODE(MANUAL_MODE) ? &currentControlRateProfile->manual : &currentControlRateProfile->stabilized;

    int16_t roll = rxGetChannelValue(ROLL);
    int16_t pitch = rxGetChannelValue(PITCH);
    int16_t yaw = rxGetChannelValue(YAW);
    int16_t throttle = rxGetChannelValue(THROTTLE);

    // Update input
    control.input.roll = rcCommandMapPWMValue(roll);
    control.input.pitch = rcCommandMapPWMValue(pitch);
    control.input.yaw = rcCommandMapPWMValue(yaw);
    if (throttleIsBidirectional()) {
        float deadband3d = rcCommandConvertPWMDeadband(rcControlsConfig()->deadband3d_throttle);
        control.input.throttle = fapplyDeadbandf(rcCommandMapPWMValue(throttle), deadband3d);
        // Normalize to [-1, 1] taking the deadband into account
        control.input.throttle /= RC_COMMAND_MAX - deadband3d;
    } else {
        if (throttle < rxConfig()->mincheck) {
            control.input.throttle = 0;
        } else {
            // Normalize to from [MINCHECK;2000] to [1000;2000]
            int16_t normalizeThrottle = scaleRange(throttle, rxConfig()->mincheck, PWM_RANGE_MAX, PWM_RANGE_MIN, PWM_RANGE_MAX);
            control.input.throttle = rcCommandMapUnidirectionalPWMValue(normalizeThrottle);
        }
    }

    // Compute output ROLL PITCH and YAW command
    control.output.roll = getAxisRcCommand(control.input.roll, axes->rcExpo8, rcCommandConvertPWMDeadband(rcControlsConfig()->deadband));
    control.output.pitch = getAxisRcCommand(control.input.pitch, axes->rcExpo8, rcCommandConvertPWMDeadband(rcControlsConfig()->deadband));
    // Invert YAW, since GYRO/ACC measures CCW but we want input yaw
    // to produce CW rotation (i.e. stick to the right rotates to the right).
    control.output.yaw = -getAxisRcCommand(control.input.yaw, axes->rcYawExpo8, rcCommandConvertPWMDeadband(rcControlsConfig()->yaw_deadband));

    // Apply manual control rates
    if (FLIGHT_MODE(MANUAL_MODE)) {
        control.output.roll *= currentControlRateProfile->manual.rates[FD_ROLL] / 100L;
        control.output.pitch *= currentControlRateProfile->manual.rates[FD_PITCH] / 100L;
        control.output.yaw *= currentControlRateProfile->manual.rates[FD_YAW] / 100L;
    }

    // Compute output THROTTLE command
    control.output.throttle = rcCurveApplyThrottleExpo(control.input.throttle);

    // Set the initial source as the RX
    control.source = RC_CONTROL_SOURCE_RX;
}

const rcCommand_t *rcControlGetInput(void)
{
    return &control.input;
}

float rcControlGetInputAxis(rc_alias_e axis)
{
    if (axis <= THROTTLE) {
        return control.input.axes[axis];
    }
    return 0;
}

float rcControlGetInputAxisApplyingPosholdDeadband(rc_alias_e axis)
{
    float deadband = rcCommandConvertPWMDeadband(rcControlsConfig()->pos_hold_deadband);
    return fapplyDeadbandf(rcControlGetInputAxis(axis), deadband) / (RC_COMMAND_MAX - deadband);
}

rcControlBidirThrottleMode_e rcControlGetBidirThrottleMode(void)
{
    return bidirThrottleMode;
}

void rcControlSetBidirThrottleMode(rcControlBidirThrottleMode_e mode)
{
    if (mode != bidirThrottleMode) {
        bidirThrottleMode = mode;
        // Changing the throttle mode might have changed wether
        // THR is bidir or not, recalculate.
        rcControlUpdateFromRX();
    }
}

const rcCommand_t *rcControlGetOutput(void)
{
    return &control.output;
}

float rcControlGetOutputAxis(rc_alias_e axis)
{
    if (axis <= THROTTLE) {
        return control.output.axes[axis];
    }
    return 0;
}

void rcControlUpdateOutput(const rcCommand_t *cmd, rcControlSource_e source)
{
    memcpy(&control.output, cmd, sizeof(*cmd));
    if (source > RC_CONTROL_SOURCE_KEEP) {
        control.source = source;
    }
}
