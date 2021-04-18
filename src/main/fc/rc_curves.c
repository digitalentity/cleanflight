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
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"

#include "flight/mixer.h"

#include "rx/rx.h"


#define PITCH_LOOKUP_LENGTH 7
#define YAW_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 11

static EXTENDED_FASTRAM uint16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];       // lookup table for expo & mid THROTTLE
static EXTENDED_FASTRAM uint16_t lookupThrottleLin[THROTTLE_LOOKUP_LENGTH];      // lookup table for expo & mid THROTTLE
int16_t lookupThrottleRCMid;                                                    // THROTTLE curve mid point

static void generateThrottleCurve(uint16_t *lookupTable, uint8_t mid, uint8_t expo)
{
    const int minThrottle = getThrottleIdleValue();
    lookupThrottleRCMid = minThrottle + (int32_t)(motorConfig()->maxthrottle - minThrottle) * mid / 100; // [MINTHROTTLE;MAXTHROTTLE]

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - mid;
        uint8_t y = 1;
        if (tmp > 0) y = 100 - mid;
        if (tmp < 0) y = mid;
        tmp = 10 * mid + tmp * (100 - expo + (int32_t)expo * (tmp * tmp) / (y * y)) / 10;
        lookupTable[i] = minThrottle + (int32_t)(motorConfig()->maxthrottle - minThrottle) * tmp / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

void generateRCThrottleCurve(const controlRateConfig_t *controlRateConfig)
{
    generateThrottleCurve(lookupThrottleRC, controlRateConfig->throttle.rcMid8, controlRateConfig->throttle.rcExpo8);
}

void generateLinThrottleCurve(void)
{
    generateThrottleCurve(lookupThrottleLin, (motorConfig()->thrExpo < 0 ? 100 : 0), ABS(motorConfig()->thrExpo));
}

int16_t rcLookup(int16_t stickDeflection, uint8_t expo)
{
    float tmpf = stickDeflection / 100.0f;
    return lrintf((2500.0f + (float)expo * (tmpf * tmpf - 25.0f)) * tmpf / 25.0f);
}

static uint16_t lookupThrottle(uint16_t *lookupTable, uint16_t throttle)
{
    const int minThrottle = getThrottleIdleValue();

    if (throttle < minThrottle)
        return minThrottle;

    throttle = (int32_t)(throttle - minThrottle) * PWM_RANGE_MIN / (PWM_RANGE_MAX - minThrottle);       // [MINTHROTTLE;2000] -> [0;999]

    if (throttle > 999)
        return motorConfig()->maxthrottle;

    const uint8_t lookupStep = throttle / 100;
    return lookupTable[lookupStep] + (throttle - lookupStep * 100) * (lookupTable[lookupStep + 1] - lookupTable[lookupStep]) / 100;
}

uint16_t rcLookupThrottle(uint16_t throttle)
{
    return lookupThrottle(lookupThrottleRC, throttle);
}

uint16_t linLookupThrottle(uint16_t throttle)
{
    return lookupThrottle(lookupThrottleLin, throttle);
}

uint16_t rcLookupThrottleMid(void)
{
    return lookupThrottleRCMid;
}
