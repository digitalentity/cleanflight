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

#include "common/maths.h"

#include "fc/rc_command.h"

#include "flight/mixer.h"

#include "rx/rx.h"

void rcCommandReset(rcCommand_t *cmd)
{
    cmd->roll = 0;
    cmd->pitch = 0;
    cmd->yaw = 0;
    cmd->throttle = 0;
}

void rcCommandRotate(rcCommand_t *dst, const rcCommand_t *src, float radians)
{
    const float cosDiff = cos_approx(radians);
    const float sinDiff = sin_approx(radians);

    float pitch = src->pitch * cosDiff + src->roll * sinDiff;
    dst->roll = src->roll * cosDiff -  src->pitch * sinDiff;
    dst->pitch = pitch;
}

void rcCommandCopy(rcCommand_t *dst, const rcCommand_t *src)
{
    dst->roll = src->roll;
    dst->pitch = src->pitch;
    dst->yaw = src->yaw;
    dst->throttle = src->throttle;
}

float rcCommandMapPWMValue(int16_t value)
{
    return constrainf(scaleRangef(value, PWM_RANGE_MIN, PWM_RANGE_MAX, RC_COMMAND_MIN, RC_COMMAND_MAX), RC_COMMAND_MIN, RC_COMMAND_MAX);
}

float rcCommandMapUnidirectionalPWMThrottle(int16_t thr)
{
    return constrainf(scaleRangef(thr, motorConfig()->minthrottle, motorConfig()->maxthrottle, RC_COMMAND_CENTER, RC_COMMAND_MAX), RC_COMMAND_CENTER, RC_COMMAND_MAX);
}

float rcCommandMapUnidirectionalPWMValue(int16_t value)
{
    return constrainf(scaleRangef(value, PWM_RANGE_MIN, PWM_RANGE_MAX, RC_COMMAND_CENTER, RC_COMMAND_MAX), RC_COMMAND_CENTER, RC_COMMAND_MAX);
}

int16_t rcCommandToPWMValue(float cmd)
{
    int16_t value = cmd * ((PWM_RANGE_MAX - PWM_RANGE_MIN) / RC_COMMAND_RANGE) + PWM_RANGE_MIDDLE;
    return constrain(value, PWM_RANGE_MIN, PWM_RANGE_MAX);
}

int16_t rcCommandThrottleMagnitudeToPWM(float thr)
{
    return motorConfig()->minthrottle + (motorConfig()->maxthrottle - motorConfig()->minthrottle) * fabsf(thr);
}

float rcCommandConvertPWMDeadband(uint8_t deadband)
{
    // Deadband are specified as PWM units, which
    // have a range of 1000. To map them to [-1, 1]
    // we have a range of 2 so we divide by 500
    return deadband / 500.0f;
}
