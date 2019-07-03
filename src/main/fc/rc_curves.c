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
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "fc/controlrate_profile.h"
#include "fc/rc_command.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"

#include "flight/mixer.h"

#include "rx/rx.h"


static float throttleRcMid;
static float throttleRcExpo;

void rcCurvePrepareThrottle(const controlRateConfig_t *controlRateConfig)
{
    throttleRcMid = constrainf(controlRateConfig->throttle.rcMid8 / 100.0f, 0, 1);
    throttleRcExpo = constrainf(controlRateConfig->throttle.rcExpo8 / 100.0f, 0, 1);
}

float rcCurveApplyExpo(float deflection, float expo)
{
    // This is just
    // ((1 - expo) * deflection) + (expo * deflection^3))
    // rearranged, so instead of doing
    // 1 add, 1 sub, 4 mult
    // it does
    // 1 add, 1 sub, 3 mult
    return (1 + expo * (deflection * deflection - 1)) * deflection;
}

float rcCurveApplyThrottleExpo(float deflection)
{
    if (deflection < 0) {
        return -rcCurveApplyThrottleExpo(-deflection);
    }

    // deflection is always >= 0 past this point
    float tmp = deflection - throttleRcMid;
    float y = 1;
    if (tmp > 0) {
        y = 1 - throttleRcMid;
    } else if (tmp < 0) {
        y = throttleRcMid;
    }

    // See docs/Stick Input.md
    return throttleRcMid + tmp * (1 - throttleRcExpo + throttleRcExpo * sq(tmp) / sq(y));
}
