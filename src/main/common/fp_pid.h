/*
 * This file is part of INAV Project.
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
 */

#pragma once

#include <stdbool.h>

#include "common/filter.h"
#include "common/maths.h"

typedef struct {
    float kP;
    float kI;
    float kD;
    float kT;   // Tracking gain (anti-windup)
    float kFF;  // FeedForward Component
} pidControllerParam_t;

typedef enum {
    PID_DTERM_FROM_ERROR            = 1 << 0,
    PID_ZERO_INTEGRATOR             = 1 << 1,
    PID_SHRINK_INTEGRATOR           = 1 << 2,
    PID_LIMIT_INTEGRATOR            = 1 << 3,
    PID_FREEZE_INTEGRATOR           = 1 << 4,
} pidControllerFlags_e;

typedef struct {
    bool reset;
    pidControllerParam_t param;
    pt1Filter_t dterm_filter_state;     // last derivative for low-pass filter
    float dTermLpfHz;                   // dTerm low pass filter cutoff frequency
    float integrator;                   // integrator value
    float last_input;                   // last input for derivative

    float integral;                     // used integral value in output
    float proportional;                 // used proportional value in output
    float derivative;                   // used derivative value in output
    float feedForward;                  // used FeedForward value in output
    float output_constrained;           // controller output constrained
} pidController_t;

float navPidApply2(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags);
float navPidApply3(
    pidController_t *pid,
    const float setpoint,
    const float measurement,
    const float dt,
    const float outMin,
    const float outMax,
    const pidControllerFlags_e pidFlags,
    const float gainScaler,
    const float dTermScaler
);
void navPidReset(pidController_t *pid);
void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD, float _kFF, float _dTermLpfHz);