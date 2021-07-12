/* 
 * This file is part of the INAV distribution https://github.com/iNavFlight/inav.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * The code in this file is a derivative work of EmuFlight distribution https://github.com/emuflight/EmuFlight/
 * 
 */
#include "platform.h"

#ifdef USE_RATE_DYNAMICS

FILE_COMPILE_FOR_SPEED

#include <stdlib.h>
#include "rate_dynamics.h"
#include "fc/controlrate_profile.h"
#include <math.h>

static FASTRAM float lastRcCommandData[3];
static FASTRAM float iterm[3];

FAST_CODE static float calculateK(const float k, const float dT) {
    if (k == 0.0f) {
        return 0;
    }
    // scale so it feels like running at 62.5hz (16ms) regardless of the current rx rate
    return k / (62.5f * dT * (1 - k) + k);
}

FAST_CODE int applyRateDynamics(int rcCommand, const int axis, const float dT) {
    if (
        currentControlRateProfile->rateDynamics.sensitivityCenter != 100 || 
        currentControlRateProfile->rateDynamics.sensitivityEnd != 100 || 
        currentControlRateProfile->rateDynamics.weightCenter > 0 || 
        currentControlRateProfile->rateDynamics.weightEnd > 0
    ) {

        float pterm_centerStick, pterm_endStick, pterm, iterm_centerStick, iterm_endStick, dterm_centerStick, dterm_endStick, dterm;
        float rcCommandPercent;
        float rcCommandError;
        float inverseRcCommandPercent;

        rcCommandPercent = abs(rcCommand) / 500.0f; // make rcCommandPercent go from 0 to 1
        inverseRcCommandPercent = 1.0f - rcCommandPercent;

        pterm_centerStick = inverseRcCommandPercent * rcCommand * (currentControlRateProfile->rateDynamics.sensitivityCenter / 100.0f); // valid pterm values are between 50-150
        pterm_endStick = rcCommandPercent * rcCommand * (currentControlRateProfile->rateDynamics.sensitivityEnd / 100.0f);
        pterm = pterm_centerStick + pterm_endStick;
        rcCommandError = rcCommand - (pterm + iterm[axis]);
        rcCommand = pterm; // add this fake pterm to the rcCommand

        iterm_centerStick = inverseRcCommandPercent * rcCommandError * calculateK(currentControlRateProfile->rateDynamics.correctionCenter / 100.0f, dT); // valid iterm values are between 0-95
        iterm_endStick = rcCommandPercent * rcCommandError * calculateK(currentControlRateProfile->rateDynamics.correctionEnd / 100.0f, dT);
        iterm[axis] += iterm_centerStick + iterm_endStick;
        rcCommand = rcCommand + iterm[axis]; // add the iterm to the rcCommand

        dterm_centerStick = inverseRcCommandPercent * (lastRcCommandData[axis] - rcCommand) * calculateK(currentControlRateProfile->rateDynamics.weightCenter / 100.0f, dT); // valid dterm values are between 0-95
        dterm_endStick = rcCommandPercent * (lastRcCommandData[axis] - rcCommand) * calculateK(currentControlRateProfile->rateDynamics.weightEnd / 100.0f, dT);
        dterm = dterm_centerStick + dterm_endStick;
        rcCommand = rcCommand + dterm; // add dterm to the rcCommand (this is real dterm)

        lastRcCommandData[axis] = rcCommand;
    }
    return rcCommand;
}

#endif