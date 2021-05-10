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

#include "common/maths.h"
#include "common/printf.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/settings.h"

// For the "modulo 4" arithmetic to work, we need a leap base year
#define REFERENCE_YEAR 2000
// Offset (seconds) from the UNIX epoch (1970-01-01) to 2000-01-01
#define EPOCH_2000_OFFSET 946684800

#define MILLIS_PER_SECOND 1000

// rtcTime_t when the system was started.
// Calculated in rtcSet().
static rtcTime_t started = 0;

static const uint16_t days[4][12] =
{
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430},
};

PG_REGISTER_WITH_RESET_TEMPLATE(timeConfig_t, timeConfig, PG_TIME_CONFIG, 1);

PG_RESET_TEMPLATE(timeConfig_t, timeConfig,
    .tz_offset = SETTING_TZ_OFFSET_DEFAULT,
    .tz_automatic_dst = SETTING_TZ_AUTOMATIC_DST_DEFAULT,
);

static rtcTime_t dateTimeToRtcTime(const dateTime_t *dt)
{
    unsigned int second = dt->seconds;  // 0-59
    unsigned int minute = dt->minutes;  // 0-59
    unsigned int hour = dt->hours;      // 0-23
    unsigned int day = dt->day - 1;     // 0-30
    unsigned int month = dt->month - 1; // 0-11
    unsigned int year = dt->year - REFERENCE_YEAR; // 0-99
    int32_t unixTime = (((year / 4 * (365 * 4 + 1) + days[year % 4][month] + day) * 24 + hour) * 60 + minute) * 60 + second + EPOCH_2000_OFFSET;
    return rtcTimeMake(unixTime, dt->millis);
}

static void rtcTimeToDateTime(dateTime_t *dt, rtcTime_t t)
{
    int32_t unixTime = t / MILLIS_PER_SECOND - EPOCH_2000_OFFSET;
    dt->seconds = unixTime % 60;
    unixTime /= 60;
    dt->minutes = unixTime % 60;
    unixTime /= 60;
    dt->hours = unixTime % 24;
    unixTime /= 24;

    unsigned int years = unixTime / (365 * 4 + 1) * 4;
    unixTime %= 365 * 4 + 1;

    unsigned int year;
    for (year = 3; year > 0; year--) {
        if (unixTime >= days[year][0]) {
            break;
        }
    }

    unsigned int month;
    for (month = 11; month > 0; month--) {
        if (unixTime >= days[year][month]) {
            break;
        }
    }

    dt->year = years + year + REFERENCE_YEAR;
    dt->month = month + 1;
    dt->day = unixTime - days[year][month] + 1;
    dt->millis = t % MILLIS_PER_SECOND;
}

static void rtcGetDefaultDateTime(dateTime_t *dateTime)
{
    dateTime->year = 0;
    dateTime->month = 1;
    dateTime->day = 1;
    dateTime->hours = 0;
    dateTime->minutes = 0;
    dateTime->seconds = 0;
    dateTime->millis = 0;
}

static bool rtcIsDateTimeValid(dateTime_t *dateTime)
{
    return (dateTime->year >= REFERENCE_YEAR) &&
           (dateTime->month >= 1 && dateTime->month <= 12) &&
           (dateTime->day >= 1 && dateTime->day <= 31) &&
           (dateTime->hours <= 23) &&
           (dateTime->minutes <= 59) &&
           (dateTime->seconds <= 59) &&
           (dateTime->millis <= 999);
}

#if defined(RTC_AUTOMATIC_DST)
static int lastSundayOfMonth(int currentYear, int wantedMonth)
{
    int days[] = { 31 , 29 , 31 , 30 , 31 , 30 , 31 , 31 , 30 , 31 , 30 , 31 };
    days[1] -= (currentYear % 4) || (!(currentYear % 100) && (currentYear % 400));
    int w = currentYear * 365 + (currentYear - 1) / 4 - (currentYear - 1) / 100 + (currentYear - 1) / 400 + 6;

	for (int m = 0; m < 12; m++) {
		w = (w + days[m]) % 7;
        if (m == wantedMonth - 1) {
            return days[m] - w;
        }
	}
    return 0;
}

static int nthSundayOfMonth(int lastSunday, int nth)
{
    while (lastSunday > 7 * nth) {
        lastSunday -= 7;
    }
    return lastSunday;
}

static bool isDST(rtcTime_t t)
{
    dateTime_t dateTime;
    rtcTimeToDateTime(&dateTime, t);
    int lastSunday;
    switch ((tz_automatic_dst_e) timeConfig()->tz_automatic_dst) {
        case TZ_AUTO_DST_OFF:
            break;
        case TZ_AUTO_DST_EU: // begins at 1:00 a.m. on the last Sunday of March and ends at 1:00 a.m. on the last Sunday of October
            if (dateTime.month < 3 || dateTime.month > 10) {
                return false;
            }
            if (dateTime.month > 3 && dateTime.month < 10) {
                return true;
            }
            lastSunday = lastSundayOfMonth(dateTime.year, dateTime.month);
            if ((dateTime.day < lastSunday) || (dateTime.day > lastSunday)) {
                return !(dateTime.month == 3);
            }
            if (dateTime.day == lastSunday) {
                if (dateTime.month == 3) {
                    return dateTime.hours >= 1;
                }
                if (dateTime.month == 10) {
                    return dateTime.hours < 1;
                }
            }
            break;
        case TZ_AUTO_DST_USA: // begins at 2:00 a.m. on the second Sunday of March and ends at 2:00 a.m. on the first Sunday of November
            if (dateTime.month < 3 || dateTime.month > 11) {
                return false;
            }
            if (dateTime.month > 3 && dateTime.month < 11) {
                return true;
            }
            lastSunday = lastSundayOfMonth(dateTime.year, dateTime.month);
            if (dateTime.month == 3) {
                int secondSunday = nthSundayOfMonth(lastSunday, 2);
                if (dateTime.day == secondSunday) {
                    return dateTime.hours >= 2;
                }
                return dateTime.day > secondSunday;
            }
            if (dateTime.month == 11) {
                int firstSunday = nthSundayOfMonth(lastSunday, 1);
                if (dateTime.day == firstSunday) {
                    return dateTime.hours < 2;
                }
                return dateTime.day < firstSunday;
            }
            break;
    }
    return false;
}
#endif

static void dateTimeWithOffset(dateTime_t *dateTimeOffset, const dateTime_t *dateTimeInitial, int16_t *minutes, bool automatic_dst)
{
    rtcTime_t initialTime = dateTimeToRtcTime(dateTimeInitial);
    rtcTime_t offsetTime = rtcTimeMake(rtcTimeGetSeconds(&initialTime) + *minutes * 60, rtcTimeGetMillis(&initialTime));
#if defined(RTC_AUTOMATIC_DST)
    if (automatic_dst && isDST(offsetTime)) {
        // Add one hour. Tell the caller that the
        // offset has changed.
        *minutes += 60;
        offsetTime += 60 * 60 * MILLIS_PER_SECOND;
    }
#else
    UNUSED(automatic_dst);
#endif
    rtcTimeToDateTime(dateTimeOffset, offsetTime);
}

static bool dateTimeFormat(char *buf, dateTime_t *dateTime, int16_t offset, bool automatic_dst)
{
    dateTime_t local;

    int tz_hours = 0;
    int tz_minutes = 0;
    bool retVal = true;

    // Apply offset if necessary
    if (offset != 0 || automatic_dst) {
        dateTimeWithOffset(&local, dateTime, &offset, automatic_dst);
        tz_hours = offset / 60;
        tz_minutes = ABS(offset % 60);
        dateTime = &local;
    }

    if (!rtcIsDateTimeValid(dateTime)) {
        rtcGetDefaultDateTime(&local);
        dateTime = &local;
        retVal = false;
    }

    // XXX: Changes to this format might require updates in
    // dateTimeSplitFormatted()
    tfp_sprintf(buf, "%04u-%02u-%02uT%02u:%02u:%02u.%03u%c%02d:%02d",
        dateTime->year, dateTime->month, dateTime->day,
        dateTime->hours, dateTime->minutes, dateTime->seconds, dateTime->millis,
        tz_hours >= 0 ? '+' : '-', ABS(tz_hours), tz_minutes);

    return retVal;
}

rtcTime_t rtcTimeMake(int32_t secs, uint16_t millis)
{
    return ((rtcTime_t)secs) * MILLIS_PER_SECOND + millis;
}

int32_t rtcTimeGetSeconds(rtcTime_t *t)
{
    return *t / MILLIS_PER_SECOND;
}

uint16_t rtcTimeGetMillis(rtcTime_t *t)
{
    return *t % MILLIS_PER_SECOND;
}

bool dateTimeFormatUTC(char *buf, dateTime_t *dt)
{
    return dateTimeFormat(buf, dt, 0, false);
}

bool dateTimeFormatLocal(char *buf, dateTime_t *dt)
{
    return dateTimeFormat(buf, dt, timeConfig()->tz_offset, true);
}

void dateTimeUTCToLocal(dateTime_t *localDateTime, const dateTime_t *utcDateTime)
{
    int16_t offset = timeConfig()->tz_offset;
    dateTimeWithOffset(localDateTime, utcDateTime, &offset, true);
}

bool dateTimeSplitFormatted(char *formatted, char **date, char **time)
{
    // Just look for the T and replace it with a zero
    // XXX: Keep in sync with dateTimeFormat()
    for (char *p = formatted; *p; p++) {
        if (*p == 'T') {
            *date = formatted;
            *time = (p+1);
            *p = '\0';
            return true;
        }
    }
    return false;
}

bool rtcHasTime(void)
{
    return started != 0;
}

bool rtcGet(rtcTime_t *t)
{
    if (!rtcHasTime()) {
        return false;
    }
    *t = started + millis();
    return true;
}

bool rtcSet(rtcTime_t *t)
{
    started = *t - millis();
    return true;
}

bool rtcGetDateTime(dateTime_t *dt)
{
    rtcTime_t t;
    if (rtcGet(&t)) {
        rtcTimeToDateTime(dt, t);
        return true;
    }
    // No time stored, fill dt with 0000-01-01T00:00:00.000
    rtcGetDefaultDateTime(dt);
    return false;
}

bool rtcGetDateTimeLocal(dateTime_t *dt)
{
    if (rtcGetDateTime(dt)) {
        dateTimeUTCToLocal(dt, dt);
        return true;
    }
    return false;
}

bool rtcSetDateTime(dateTime_t *dt)
{
    rtcTime_t t = dateTimeToRtcTime(dt);
    return rtcSet(&t);
}
