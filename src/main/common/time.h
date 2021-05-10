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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "config/parameter_group.h"

// time difference, signed 32 bits of microseconds overflows at ~35 minutes
// this is worth leaving as int32_t for performance reasons, use timeDeltaLarge_t for deltas that can be big
typedef int32_t timeDelta_t;
#define TIMEDELTA_MAX INT32_MAX

// time difference large, signed 64 bits of microseconds overflows at ~300000 years
typedef int64_t timeDeltaLarge_t;
#define TIMEDELTALARGE_MAX INT64_MAX

// millisecond time
typedef uint32_t timeMs_t;
#define TIMEMS_MAX UINT32_MAX

// microsecond time
#ifdef USE_64BIT_TIME
typedef uint64_t timeUs_t;
#define TIMEUS_MAX UINT64_MAX
#else
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX
#endif

// Constants for better readability
#define MILLISECS_PER_SEC 1000
#define USECS_PER_SEC (1000 * 1000)

#define HZ2US(hz)   (1000000 / (hz))
#define US2S(us)    ((us) * 1e-6f)
#define US2MS(us)   ((us) * 1e-3f)
#define MS2US(ms)   ((ms) * 1000)
#define MS2S(ms)    ((ms) * 1e-3f)
#define HZ2S(hz)    US2S(HZ2US(hz))

// Use this function only to get small deltas (difference overflows at ~35 minutes)
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }

typedef enum {
    TZ_AUTO_DST_OFF,
    TZ_AUTO_DST_EU,
    TZ_AUTO_DST_USA,
} tz_automatic_dst_e;

typedef struct timeConfig_s {
    int16_t tz_offset; // Offset from UTC in minutes, might be positive or negative
    uint8_t tz_automatic_dst; // Automatically handle DST or ignore it, values come from tz_automatic_dst_e
} timeConfig_t;

PG_DECLARE(timeConfig_t, timeConfig);

// Milliseconds since Jan 1 1970
typedef int64_t rtcTime_t;

rtcTime_t rtcTimeMake(int32_t secs, uint16_t millis);
int32_t rtcTimeGetSeconds(rtcTime_t *t);
uint16_t rtcTimeGetMillis(rtcTime_t *t);

typedef struct _dateTime_s {
    // full year
    uint16_t year;
    // 1-12
    uint8_t month;
    // 1-31
    uint8_t day;
    // 0-23
    uint8_t hours;
    // 0-59
    uint8_t minutes;
    // 0-59
    uint8_t seconds;
    // 0-999
    uint16_t millis;
} dateTime_t;

#define FORMATTED_DATE_TIME_BUFSIZE 30

// buf must be at least FORMATTED_DATE_TIME_BUFSIZE
bool dateTimeFormatUTC(char *buf, dateTime_t *dt);
bool dateTimeFormatLocal(char *buf, dateTime_t *dt);

void dateTimeUTCToLocal(dateTime_t *localDateTime, const dateTime_t *utcDateTime);
// dateTimeSplitFormatted splits a formatted date into its date
// and time parts. Note that the string pointed by formatted will
// be modifed and will become invalid after calling this function.
bool dateTimeSplitFormatted(char *formatted, char **date, char **time);

bool rtcHasTime(void);

bool rtcGet(rtcTime_t *t);
bool rtcSet(rtcTime_t *t);

bool rtcGetDateTime(dateTime_t *dt);
bool rtcGetDateTimeLocal(dateTime_t *dt);
bool rtcSetDateTime(dateTime_t *dt);
