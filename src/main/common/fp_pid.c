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

#include "platform.h"

FILE_COMPILE_FOR_SPEED

#include <math.h>
#include "common/fp_pid.h"

/*-----------------------------------------------------------
 * Float point PID-controller implementation
 *-----------------------------------------------------------*/
// Implementation of PID with back-calculation I-term anti-windup
// Control System Design, Lecture Notes for ME 155A by Karl Johan Åström (p.228)
// http://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
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
) {
    float newProportional, newDerivative, newFeedForward;
    float error = setpoint - measurement;

    /* P-term */
    newProportional = error * pid->param.kP * gainScaler;

    /* D-term */
    if (pid->reset) {
        pid->last_input = (pidFlags & PID_DTERM_FROM_ERROR) ? error : measurement;
        pid->reset = false;
    }

    if (pidFlags & PID_DTERM_FROM_ERROR) {
        /* Error-tracking D-term */
        newDerivative = (error - pid->last_input) / dt;
        pid->last_input = error;
    } else {
        /* Measurement tracking D-term */
        newDerivative = -(measurement - pid->last_input) / dt;
        pid->last_input = measurement;
    }

    if (pid->dTermLpfHz > 0) {
        newDerivative = pid->param.kD * pt1FilterApply4(&pid->dterm_filter_state, newDerivative, pid->dTermLpfHz, dt);
    } else {
        newDerivative = pid->param.kD * newDerivative;
    }

    newDerivative = newDerivative * gainScaler * dTermScaler;

    if (pidFlags & PID_ZERO_INTEGRATOR) {
        pid->integrator = 0.0f;
    }

    /*
     * Compute FeedForward parameter
     */
    newFeedForward = setpoint * pid->param.kFF * gainScaler;

    /* Pre-calculate output and limit it if actuator is saturating */
    const float outVal = newProportional + (pid->integrator * gainScaler) + newDerivative + newFeedForward;
    const float outValConstrained = constrainf(outVal, outMin, outMax);

    pid->proportional = newProportional;
    pid->integral = pid->integrator;
    pid->derivative = newDerivative;
    pid->feedForward = newFeedForward;
    pid->output_constrained = outValConstrained;

    /* Update I-term */
    if (
        !(pidFlags & PID_ZERO_INTEGRATOR) &&
        !(pidFlags & PID_FREEZE_INTEGRATOR) 
    ) {
        const float newIntegrator = pid->integrator + (error * pid->param.kI * gainScaler * dt) + ((outValConstrained - outVal) * pid->param.kT * dt);

        if (pidFlags & PID_SHRINK_INTEGRATOR) {
            // Only allow integrator to shrink
            if (fabsf(newIntegrator) < fabsf(pid->integrator)) {
                pid->integrator = newIntegrator;
            }
        }
        else {
            pid->integrator = newIntegrator;
        }
    }
    
    if (pidFlags & PID_LIMIT_INTEGRATOR) {
        pid->integrator = constrainf(pid->integrator, outMin, outMax);
    } 

    return outValConstrained;
}

float navPidApply2(pidController_t *pid, const float setpoint, const float measurement, const float dt, const float outMin, const float outMax, const pidControllerFlags_e pidFlags)
{
    return navPidApply3(pid, setpoint, measurement, dt, outMin, outMax, pidFlags, 1.0f, 1.0f);
}

void navPidReset(pidController_t *pid)
{
    pid->reset = true;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->last_input = 0.0f;
    pid->feedForward = 0.0f;
    pt1FilterReset(&pid->dterm_filter_state, 0.0f);
    pid->output_constrained = 0.0f;
}

void navPidInit(pidController_t *pid, float _kP, float _kI, float _kD, float _kFF, float _dTermLpfHz)
{
    pid->param.kP = _kP;
    pid->param.kI = _kI;
    pid->param.kD = _kD;
    pid->param.kFF = _kFF;

    if (_kI > 1e-6f && _kP > 1e-6f) {
        float Ti = _kP / _kI;
        float Td = _kD / _kP;
        pid->param.kT = 2.0f / (Ti + Td);
    }
    else if (_kI > 1e-6f) {
        pid->param.kI = _kI;
        pid->param.kT = 0.0f;
    }
    else {
        pid->param.kI = 0.0;
        pid->param.kT = 0.0;
    }
    pid->dTermLpfHz = _dTermLpfHz;
    navPidReset(pid);
}