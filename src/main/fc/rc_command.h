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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RC_COMMAND_AXES_COUNT 4

#define RC_COMMAND_MIN -1.0f
#define RC_COMMAND_CENTER 0.0f
#define RC_COMMAND_MAX 1.0f
#define RC_COMMAND_RANGE (RC_COMMAND_MAX - RC_COMMAND_MIN)

typedef struct rcCommand_s {
    union {
        float axes[RC_COMMAND_AXES_COUNT];
        struct {
            float roll;
            float pitch;
            float yaw;
            float throttle;
        };
    };
} rcCommand_t;

// Sets all values to neutral
void rcCommandReset(rcCommand_t *cmd);

// Rotates the given command pitch and roll by the given angle in
// radians. Used mainly for HEADFREE mode.
void rcCommandRotate(rcCommand_t *dst, const rcCommand_t *src, float radians);
void rcCommandCopy(rcCommand_t *dst, const rcCommand_t *src);

// Maps a PWM value in [PWM_RANGE_MIN, PWM_RANGE_MAX] to [RC_COMMAND_MIN, RC_COMMAND_MAX]
float rcCommandMapPWMValue(int16_t value);
// Maps a throttle PWM value in [motorConfig()->minthrottle, motorConfig->maxthrottle]
// to [RC_COMMAND_CENTER, RC_COMMAND_MAX]
float rcCommandMapUnidirectionalPWMThrottle(int16_t thr);
// Maps a PWM value in [PWM_RANGE_MIN, PWM_RANGE_MAX] to [RC_COMMAND_CENTER, RC_COMMAND_MAX]
float rcCommandMapUnidirectionalPWMValue(int16_t value);

int16_t rcCommandToPWMValue(float cmd);

// Returns the absolute value of throttle mapped to [motorConfig()->minthrottle, motorConfig->maxthrottle]
int16_t rcCommandThrottleMagnitudeToPWM(float thr);

float rcCommandConvertPWMDeadband(uint8_t deadband);
