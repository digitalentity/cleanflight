/*
 * frsky.c
 * Common functions for FrSky D-series and SmartPort telemetry
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY) && (defined(USE_TELEMETRY_FRSKY) || defined(USE_TELEMETRY_SMARTPORT))

#include "common/maths.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"
#include "io/gps.h"
#include "telemetry/frsky.h"

uint16_t frskyGetFlightMode(void)
{
    uint16_t tmpi = 0;

    // ones column
    if (!isArmingDisabled())
        tmpi += 1;
    else
        tmpi += 2;
    if (ARMING_FLAG(ARMED))
        tmpi += 4;

    // tens column
    if (FLIGHT_MODE(ANGLE_MODE))
        tmpi += 10;
    if (FLIGHT_MODE(HORIZON_MODE))
        tmpi += 20;
    if (FLIGHT_MODE(MANUAL_MODE))
        tmpi += 40;

    // hundreds column
    if (FLIGHT_MODE(HEADING_MODE))
        tmpi += 100;
    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        tmpi += 200;
    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        tmpi += 400;

    // thousands column
    if (FLIGHT_MODE(NAV_RTH_MODE))
        tmpi += 1000;
    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) // intentionally out of order and 'else-ifs' to prevent column overflow
        tmpi += 8000;
    else if (FLIGHT_MODE(NAV_WP_MODE))
        tmpi += 2000;
    else if (FLIGHT_MODE(HEADFREE_MODE))
        tmpi += 4000;

    // ten thousands column
    if (FLIGHT_MODE(FLAPERON))
        tmpi += 10000;
    if (FLIGHT_MODE(FAILSAFE_MODE))
        tmpi += 40000;
    else if (FLIGHT_MODE(AUTO_TUNE)) // intentionally reverse order and 'else-if' to prevent 16-bit overflow
        tmpi += 20000;

    return tmpi;
}

uint16_t frskyGetGPSState(void)
{
    uint16_t tmpi = 0;

    // ones and tens columns (# of satellites 0 - 99)
    tmpi += constrain(gpsSol.numSat, 0, 99);

    // hundreds column (satellite accuracy HDOP: 0 = worst [HDOP > 5.5], 9 = best [HDOP <= 1.0])
    tmpi += (9 - constrain((gpsSol.hdop - 51) / 50, 0, 9)) * 100;

    // thousands column (GPS fix status)
    if (STATE(GPS_FIX))
        tmpi += 1000;
    if (STATE(GPS_FIX_HOME))
        tmpi += 2000;
    if (ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXHOMERESET) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE))
        tmpi += 4000;

    return tmpi;
}

#endif
