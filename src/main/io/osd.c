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

/*
 Created by Marcin Baliniak
 some functions based on MinimOSD

 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

FILE_COMPILE_FOR_SPEED

#ifdef USE_OSD

#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/axis.h"
#include "common/constants.h"
#include "common/filter.h"
#include "common/log.h"
#include "common/olc.h"
#include "common/printf.h"
#include "common/string_light.h"
#include "common/time.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"
#include "drivers/display_font_metadata.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "io/flashfs.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/osd_common.h"
#include "io/osd_hud.h"
#include "io/vtx.h"
#include "io/vtx_string.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_tasks.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/power_limits.h"
#include "flight/rth_estimator.h"
#include "flight/secondary_imu.h"
#include "flight/servos.h"
#include "flight/wind_estimator.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"
#include "rx/msp_override.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/diagnostics.h"
#include "sensors/sensors.h"
#include "sensors/pitotmeter.h"
#include "sensors/temperature.h"
#include "sensors/esc_sensor.h"
#include "sensors/rangefinder.h"

#include "programming/logic_condition.h"
#include "programming/global_variables.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#define VIDEO_BUFFER_CHARS_PAL    480

#define GFORCE_FILTER_TC 0.2

#define DELAYED_REFRESH_RESUME_COMMAND (checkStickPosition(THR_HI) || checkStickPosition(PIT_HI))
#define STATS_PAGE2 (checkStickPosition(ROL_HI))
#define STATS_PAGE1 (checkStickPosition(ROL_LO))

#define SPLASH_SCREEN_DISPLAY_TIME 4000 // ms
#define ARMED_SCREEN_DISPLAY_TIME 1500 // ms
#define STATS_SCREEN_DISPLAY_TIME 60000 // ms

#define EFFICIENCY_UPDATE_INTERVAL (5 * 1000)

// Adjust OSD_MESSAGE's default position when
// changing OSD_MESSAGE_LENGTH
#define OSD_MESSAGE_LENGTH 28
#define OSD_ALTERNATING_CHOICES(ms, num_choices) ((millis() / ms) % num_choices)
#define _CONST_STR_SIZE(s) ((sizeof(s)/sizeof(s[0]))-1) // -1 to avoid counting final '\0'
// Wrap all string constants intenteded for display as messages with
// this macro to ensure compile time length validation.
#define OSD_MESSAGE_STR(x) ({ \
    STATIC_ASSERT(_CONST_STR_SIZE(x) <= OSD_MESSAGE_LENGTH, message_string_ ## __COUNTER__ ## _too_long); \
    x; \
})

#define OSD_CHR_IS_NUM(c) (c >= '0' && c <= '9')

#define OSD_CENTER_LEN(x) ((osdDisplayPort->cols - x) / 2)
#define OSD_CENTER_S(s) OSD_CENTER_LEN(strlen(s))

#define OSD_MIN_FONT_VERSION 2

static unsigned currentLayout = 0;
static int layoutOverride = -1;
static bool hasExtendedFont = false; // Wether the font supports characters > 256
static timeMs_t layoutOverrideUntil = 0;
static pt1Filter_t GForceFilter, GForceFilterAxis[XYZ_AXIS_COUNT];
static float GForce, GForceAxis[XYZ_AXIS_COUNT];

typedef struct statistic_s {
    uint16_t max_speed;
    uint16_t min_voltage; // /100
    int16_t max_current;
    int32_t max_power;
    int16_t min_rssi;
    int16_t min_lq; // for CRSF
    int16_t min_rssi_dbm; // for CRSF
    int32_t max_altitude;
    uint32_t max_distance;
} statistic_t;

static statistic_t stats;

static timeUs_t resumeRefreshAt = 0;
static bool refreshWaitForResumeCmdRelease;

static bool fullRedraw = false;

static uint8_t armState;
static uint8_t statsPagesCheck = 0;

typedef struct osdMapData_s {
    uint32_t scale;
    char referenceSymbol;
} osdMapData_t;

static osdMapData_t osdMapData;

static displayPort_t *osdDisplayPort;
static bool osdDisplayIsReady = false;
#if defined(USE_CANVAS)
static displayCanvas_t osdCanvas;
static bool osdDisplayHasCanvas;
#else
#define osdDisplayHasCanvas false
#endif

#define AH_MAX_PITCH_DEFAULT 20 // Specify default maximum AHI pitch value displayed (degrees)

PG_REGISTER_WITH_RESET_TEMPLATE(osdConfig_t, osdConfig, PG_OSD_CONFIG, 4);
PG_REGISTER_WITH_RESET_FN(osdLayoutsConfig_t, osdLayoutsConfig, PG_OSD_LAYOUTS_CONFIG, 0);

static int digitCount(int32_t value)
{
    int digits = 1;
    while(1) {
        value = value / 10;
        if (value == 0) {
            break;
        }
        digits++;
    }
    return digits;
}

bool osdDisplayIsPAL(void)
{
    return displayScreenSize(osdDisplayPort) == VIDEO_BUFFER_CHARS_PAL;
}

/**
 * Formats a number given in cents, to support non integer values
 * without using floating point math. Value is always right aligned
 * and spaces are inserted before the number to always yield a string
 * of the same length. If the value doesn't fit into the provided length
 * it will be divided by scale and true will be returned.
 */
bool osdFormatCentiNumber(char *buff, int32_t centivalue, uint32_t scale, int maxDecimals, int maxScaledDecimals, int length)
{
    char *ptr = buff;
    char *dec;
    int decimals = maxDecimals;
    bool negative = false;
    bool scaled = false;

    buff[length] = '\0';

    if (centivalue < 0) {
        negative = true;
        centivalue = -centivalue;
        length--;
    }

    int32_t integerPart = centivalue / 100;
    // 3 decimal digits
    int32_t millis = (centivalue % 100) * 10;

    int digits = digitCount(integerPart);
    int remaining = length - digits;

    if (remaining < 0 && scale > 0) {
        // Reduce by scale
        scaled = true;
        decimals = maxScaledDecimals;
        integerPart = integerPart / scale;
        // Multiply by 10 to get 3 decimal digits
        millis = ((centivalue % (100 * scale)) * 10) / scale;
        digits = digitCount(integerPart);
        remaining = length - digits;
    }

    // 3 decimals at most
    decimals = MIN(remaining, MIN(decimals, 3));
    remaining -= decimals;

    // Done counting. Time to write the characters.

    // Write spaces at the start
    while (remaining > 0) {
        *ptr = SYM_BLANK;
        ptr++;
        remaining--;
    }

    // Write the minus sign if required
    if (negative) {
        *ptr = '-';
        ptr++;
    }
    // Now write the digits.
    ui2a(integerPart, 10, 0, ptr);
    ptr += digits;
    if (decimals > 0) {
        *(ptr-1) += SYM_ZERO_HALF_TRAILING_DOT - '0';
        dec = ptr;
        int factor = 3; // we're getting the decimal part in millis first
        while (decimals < factor) {
            factor--;
            millis /= 10;
        }
        int decimalDigits = digitCount(millis);
        while (decimalDigits < decimals) {
            decimalDigits++;
            *ptr = '0';
            ptr++;
        }
        ui2a(millis, 10, 0, ptr);
        *dec += SYM_ZERO_HALF_LEADING_DOT - '0';
    }
    return scaled;
}

/*
 * Aligns text to the left side. Adds spaces at the end to keep string length unchanged.
 */
static void osdLeftAlignString(char *buff)
{
    uint8_t sp = 0, ch = 0;
    uint8_t len = strlen(buff);
    while (buff[sp] == ' ') sp++;
    for (ch = 0; ch < (len - sp); ch++) buff[ch] = buff[ch + sp];
    for (sp = ch; sp < len; sp++) buff[sp] = ' ';
}

/**
 * Converts distance into a string based on the current unit system
 * prefixed by a a symbol to indicate the unit used.
 * @param dist Distance in centimeters
 */
static void osdFormatDistanceSymbol(char *buff, int32_t dist, uint8_t decimals)
{
    switch ((osd_unit_e)osdConfig()->units) {
    case OSD_UNIT_UK:
        FALLTHROUGH;
    case OSD_UNIT_IMPERIAL:
        if (osdFormatCentiNumber(buff, CENTIMETERS_TO_CENTIFEET(dist), FEET_PER_MILE, decimals, 3, 3)) {
            buff[3] = SYM_DIST_MI;
        } else {
            buff[3] = SYM_DIST_FT;
        }
        buff[4] = '\0';
        break;
    case OSD_UNIT_METRIC_MPH:
        FALLTHROUGH;
    case OSD_UNIT_METRIC:
        if (osdFormatCentiNumber(buff, dist, METERS_PER_KILOMETER, decimals, 3, 3)) {
            buff[3] = SYM_DIST_KM;
        } else {
            buff[3] = SYM_DIST_M;
        }
        buff[4] = '\0';
        break;
    }
}

/**
 * Converts distance into a string based on the current unit system.
 * @param dist Distance in centimeters
 */
static void osdFormatDistanceStr(char *buff, int32_t dist)
{
    int32_t centifeet;
    switch ((osd_unit_e)osdConfig()->units) {
    case OSD_UNIT_UK:
        FALLTHROUGH;
    case OSD_UNIT_IMPERIAL:
        centifeet = CENTIMETERS_TO_CENTIFEET(dist);
        if (abs(centifeet) < FEET_PER_MILE * 100 / 2) {
            // Show feet when dist < 0.5mi
            tfp_sprintf(buff, "%d%c", (int)(centifeet / 100), SYM_FT);
        } else {
            // Show miles when dist >= 0.5mi
            tfp_sprintf(buff, "%d.%02d%c", (int)(centifeet / (100*FEET_PER_MILE)),
                (abs(centifeet) % (100 * FEET_PER_MILE)) / FEET_PER_MILE, SYM_MI);
        }
        break;
    case OSD_UNIT_METRIC_MPH:
        FALLTHROUGH;
    case OSD_UNIT_METRIC:
        if (abs(dist) < METERS_PER_KILOMETER * 100) {
            // Show meters when dist < 1km
            tfp_sprintf(buff, "%d%c", (int)(dist / 100), SYM_M);
        } else {
            // Show kilometers when dist >= 1km
            tfp_sprintf(buff, "%d.%02d%c", (int)(dist / (100*METERS_PER_KILOMETER)),
                (abs(dist) % (100 * METERS_PER_KILOMETER)) / METERS_PER_KILOMETER, SYM_KM);
        }
        break;
    }
}

/**
 * Converts velocity based on the current unit system (kmh or mph).
 * @param alt Raw velocity (i.e. as taken from gpsSol.groundSpeed in centimeters/second)
 */
static int32_t osdConvertVelocityToUnit(int32_t vel)
{
    switch ((osd_unit_e)osdConfig()->units) {
    case OSD_UNIT_UK:
        FALLTHROUGH;
    case OSD_UNIT_METRIC_MPH:
        FALLTHROUGH;
    case OSD_UNIT_IMPERIAL:
        return (vel * 224) / 10000; // Convert to mph
    case OSD_UNIT_METRIC:
        return (vel * 36) / 1000;   // Convert to kmh
    }
    // Unreachable
    return -1;
}

/**
 * Converts velocity into a string based on the current unit system.
 * @param alt Raw velocity (i.e. as taken from gpsSol.groundSpeed in centimeters/seconds)
 */
void osdFormatVelocityStr(char* buff, int32_t vel, bool _3D)
{
    switch ((osd_unit_e)osdConfig()->units) {
    case OSD_UNIT_UK:
        FALLTHROUGH;
    case OSD_UNIT_METRIC_MPH:
        FALLTHROUGH;
    case OSD_UNIT_IMPERIAL:
        tfp_sprintf(buff, "%3d%c", (int)osdConvertVelocityToUnit(vel), (_3D ? SYM_3D_MPH : SYM_MPH));
        break;
    case OSD_UNIT_METRIC:
        tfp_sprintf(buff, "%3d%c", (int)osdConvertVelocityToUnit(vel), (_3D ? SYM_3D_KMH : SYM_KMH));
        break;
    }
}

/**
 * Converts wind speed into a string based on the current unit system, using
 * always 3 digits and an additional character for the unit at the right. buff
 * is null terminated.
 * @param ws Raw wind speed in cm/s
 */
#ifdef USE_WIND_ESTIMATOR
static void osdFormatWindSpeedStr(char *buff, int32_t ws, bool isValid)
{
    int32_t centivalue;
    char suffix;
    switch (osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            centivalue = (ws * 224) / 100;
            suffix = SYM_MPH;
            break;
        default:
        case OSD_UNIT_METRIC:
            centivalue = (ws * 36) / 10;
            suffix = SYM_KMH;
            break;
    }
    if (isValid) {
        osdFormatCentiNumber(buff, centivalue, 0, 2, 0, 3);
    } else {
        buff[0] = buff[1] = buff[2] = '-';
    }
    buff[3] = suffix;
    buff[4] = '\0';
}
#endif

/**
* Converts altitude into a string based on the current unit system
* prefixed by a a symbol to indicate the unit used.
* @param alt Raw altitude/distance (i.e. as taken from baro.BaroAlt in centimeters)
*/
void osdFormatAltitudeSymbol(char *buff, int32_t alt)
{
    int digits;
    if (alt < 0) {
        digits = 4;
    } else {
        digits = 3;
        buff[0] = ' ';
    }
    switch ((osd_unit_e)osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            if (osdFormatCentiNumber(buff + 4 - digits, CENTIMETERS_TO_CENTIFEET(alt), 1000, 0, 2, digits)) {
                // Scaled to kft
                buff[4] = SYM_ALT_KFT;
            } else {
                // Formatted in feet
                buff[4] = SYM_ALT_FT;
            }
            buff[5] = '\0';
            break;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            // alt is alredy in cm
            if (osdFormatCentiNumber(buff + 4 - digits, alt, 1000, 0, 2, digits)) {
                // Scaled to km
                buff[4] = SYM_ALT_KM;
            } else {
                // Formatted in m
                buff[4] = SYM_ALT_M;
            }
            buff[5] = '\0';
            break;
    }
}

/**
* Converts altitude into a string based on the current unit system.
* @param alt Raw altitude/distance (i.e. as taken from baro.BaroAlt in centimeters)
*/
static void osdFormatAltitudeStr(char *buff, int32_t alt)
{
    int32_t value;
    switch ((osd_unit_e)osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            value = CENTIMETERS_TO_FEET(alt);
            tfp_sprintf(buff, "%d%c", (int)value, SYM_FT);
            break;
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            value = CENTIMETERS_TO_METERS(alt);
            tfp_sprintf(buff, "%d%c", (int)value, SYM_M);
            break;
    }
}

static void osdFormatTime(char *buff, uint32_t seconds, char sym_m, char sym_h)
{
    uint32_t value = seconds;
    char sym = sym_m;
    // Maximum value we can show in minutes is 99 minutes and 59 seconds
    if (seconds > (99 * 60) + 59) {
        sym = sym_h;
        value = seconds / 60;
    }
    buff[0] = sym;
    tfp_sprintf(buff + 1, "%02d:%02d", (int)(value / 60), (int)(value % 60));
}

static inline void osdFormatOnTime(char *buff)
{
    osdFormatTime(buff, micros() / 1000000, SYM_ON_M, SYM_ON_H);
}

static inline void osdFormatFlyTime(char *buff, textAttributes_t *attr)
{
    uint32_t seconds = getFlightTime();
    osdFormatTime(buff, seconds, SYM_FLY_M, SYM_FLY_H);
    if (attr && osdConfig()->time_alarm > 0) {
       if (seconds / 60 >= osdConfig()->time_alarm && ARMING_FLAG(ARMED)) {
            TEXT_ATTRIBUTES_ADD_BLINK(*attr);
        }
    }
}

/**
 * Converts RSSI into a % value used by the OSD.
 */
static uint16_t osdConvertRSSI(void)
{
    // change range to [0, 99]
    return constrain(getRSSI() * 100 / RSSI_MAX_VALUE, 0, 99);
}

static uint16_t osdGetCrsfLQ(void)
{
    int16_t statsLQ = rxLinkStatistics.uplinkLQ;
    int16_t scaledLQ = scaleRange(constrain(statsLQ, 0, 100), 0, 100, 170, 300);
    int16_t displayedLQ;
    switch (osdConfig()->crsf_lq_format) {
        case OSD_CRSF_LQ_TYPE1:
        case OSD_CRSF_LQ_TYPE2:
            displayedLQ = statsLQ;
            break;
        case OSD_CRSF_LQ_TYPE3:
            displayedLQ = rxLinkStatistics.rfMode >= 2 ? scaledLQ : statsLQ;
            break;
    }
    return displayedLQ;
}

static int16_t osdGetCrsfdBm(void)
{
    return rxLinkStatistics.uplinkRSSI;
}
/**
* Displays a temperature postfixed with a symbol depending on the current unit system
* @param label to display
* @param valid true if measurement is valid
* @param temperature in deciDegrees Celcius
*/
static void osdDisplayTemperature(uint8_t elemPosX, uint8_t elemPosY, uint16_t symbol, const char *label, bool valid, int16_t temperature, int16_t alarm_min, int16_t alarm_max)
{
    char buff[TEMPERATURE_LABEL_LEN + 2 < 6 ? 6 : TEMPERATURE_LABEL_LEN + 2];
    textAttributes_t elemAttr = valid ? TEXT_ATTRIBUTES_NONE : _TEXT_ATTRIBUTES_BLINK_BIT;
    uint8_t valueXOffset = 0;

    if (symbol) {
        buff[0] = symbol;
        buff[1] = '\0';
        displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, buff, elemAttr);
        valueXOffset = 1;
    }
#ifdef USE_TEMPERATURE_SENSOR
    else if (label[0] != '\0') {
        uint8_t label_len = strnlen(label, TEMPERATURE_LABEL_LEN);
        memcpy(buff, label, label_len);
        memset(buff + label_len, ' ', TEMPERATURE_LABEL_LEN + 1 - label_len);
        buff[5] = '\0';
        displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, buff, elemAttr);
        valueXOffset = osdConfig()->temp_label_align == OSD_ALIGN_LEFT ? 5 : label_len + 1;
    }
#else
    UNUSED(label);
#endif

    if (valid) {

        if ((temperature <= alarm_min) || (temperature >= alarm_max)) TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        if (osdConfig()->units == OSD_UNIT_IMPERIAL) temperature = temperature * 9 / 5.0f + 320;
        tfp_sprintf(buff, "%3d", temperature / 10);

    } else
        strcpy(buff, "---");

    buff[3] = osdConfig()->units == OSD_UNIT_IMPERIAL ? SYM_TEMP_F : SYM_TEMP_C;
    buff[4] = '\0';

    displayWriteWithAttr(osdDisplayPort, elemPosX + valueXOffset, elemPosY, buff, elemAttr);
}

#ifdef USE_TEMPERATURE_SENSOR
static void osdDisplayTemperatureSensor(uint8_t elemPosX, uint8_t elemPosY, uint8_t sensorIndex)
{
    int16_t temperature;
    const bool valid = getSensorTemperature(sensorIndex, &temperature);
    const tempSensorConfig_t *sensorConfig = tempSensorConfig(sensorIndex);
    uint16_t symbol = sensorConfig->osdSymbol ? SYM_TEMP_SENSOR_FIRST + sensorConfig->osdSymbol - 1 : 0;
    osdDisplayTemperature(elemPosX, elemPosY, symbol, sensorConfig->label, valid, temperature, sensorConfig->alarm_min, sensorConfig->alarm_max);
}
#endif

static void osdFormatCoordinate(char *buff, char sym, int32_t val)
{
    // up to 4 for number + 1 for the symbol + null terminator + fill the rest with decimals
    const int coordinateLength = osdConfig()->coordinate_digits + 1;

    buff[0] = sym;
    int32_t integerPart = val / GPS_DEGREES_DIVIDER;
    // Latitude maximum integer width is 3 (-90) while
    // longitude maximum integer width is 4 (-180).
    int integerDigits = tfp_sprintf(buff + 1, (integerPart == 0 && val < 0) ? "-%d" : "%d", (int)integerPart);
    // We can show up to 7 digits in decimalPart.
    int32_t decimalPart = abs(val % GPS_DEGREES_DIVIDER);
    STATIC_ASSERT(GPS_DEGREES_DIVIDER == 1e7, adjust_max_decimal_digits);
    int decimalDigits = tfp_sprintf(buff + 1 + integerDigits, "%07d", (int)decimalPart);
    // Embbed the decimal separator
    buff[1 + integerDigits - 1] += SYM_ZERO_HALF_TRAILING_DOT - '0';
    buff[1 + integerDigits] += SYM_ZERO_HALF_LEADING_DOT - '0';
    // Fill up to coordinateLength with zeros
    int total = 1 + integerDigits + decimalDigits;
    while(total < coordinateLength) {
        buff[total] = '0';
        total++;
    }
    buff[coordinateLength] = '\0';
}

static void osdFormatCraftName(char *buff)
{
    if (strlen(systemConfig()->name) == 0)
            strcpy(buff, "CRAFT_NAME");
    else {
        for (int i = 0; i < MAX_NAME_LENGTH; i++) {
            buff[i] = sl_toupper((unsigned char)systemConfig()->name[i]);
            if (systemConfig()->name[i] == 0)
                break;
        }
    }
}

static const char * osdArmingDisabledReasonMessage(void)
{
    const char *message = NULL;
    char messageBuf[MAX(SETTING_MAX_NAME_LENGTH, OSD_MESSAGE_LENGTH+1)];

    switch (isArmingDisabledReason()) {
        case ARMING_DISABLED_FAILSAFE_SYSTEM:
            // See handling of FAILSAFE_RX_LOSS_MONITORING in failsafe.c
            if (failsafePhase() == FAILSAFE_RX_LOSS_MONITORING) {
                if (failsafeIsReceivingRxData()) {
                    // If we're not using sticks, it means the ARM switch
                    // hasn't been off since entering FAILSAFE_RX_LOSS_MONITORING
                    // yet
                    return OSD_MESSAGE_STR(OSD_MSG_TURN_ARM_SW_OFF);
                }
                // Not receiving RX data
                return OSD_MESSAGE_STR(OSD_MSG_RC_RX_LINK_LOST);
            }
            return OSD_MESSAGE_STR(OSD_MSG_DISABLED_BY_FS);
        case ARMING_DISABLED_NOT_LEVEL:
            return OSD_MESSAGE_STR(OSD_MSG_AIRCRAFT_UNLEVEL);
        case ARMING_DISABLED_SENSORS_CALIBRATING:
            return OSD_MESSAGE_STR(OSD_MSG_SENSORS_CAL);
        case ARMING_DISABLED_SYSTEM_OVERLOADED:
            return OSD_MESSAGE_STR(OSD_MSG_SYS_OVERLOADED);
        case ARMING_DISABLED_NAVIGATION_UNSAFE:
#if defined(USE_NAV)
            // Check the exact reason
            switch (navigationIsBlockingArming(NULL)) {
                char buf[6];
                case NAV_ARMING_BLOCKER_NONE:
                    break;
                case NAV_ARMING_BLOCKER_MISSING_GPS_FIX:
                    return OSD_MESSAGE_STR(OSD_MSG_WAITING_GPS_FIX);
                case NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE:
                    return OSD_MESSAGE_STR(OSD_MSG_DISABLE_NAV_FIRST);
                case NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR:
                    osdFormatDistanceSymbol(buf, distanceToFirstWP(), 0);
                    tfp_sprintf(messageBuf, "FIRST WP TOO FAR (%s)", buf);
                    return message = messageBuf;
                case NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR:
                    return OSD_MESSAGE_STR(OSD_MSG_JUMP_WP_MISCONFIG);
            }
#endif
            break;
        case ARMING_DISABLED_COMPASS_NOT_CALIBRATED:
            return OSD_MESSAGE_STR(OSD_MSG_MAG_NOT_CAL);
        case ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED:
            return OSD_MESSAGE_STR(OSD_MSG_ACC_NOT_CAL);
        case ARMING_DISABLED_ARM_SWITCH:
            return OSD_MESSAGE_STR(OSD_MSG_DISARM_1ST);
        case ARMING_DISABLED_HARDWARE_FAILURE:
            {
                if (!HW_SENSOR_IS_HEALTHY(getHwGyroStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_GYRO_FAILURE);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwAccelerometerStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_ACC_FAIL);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwCompassStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_MAG_FAIL);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwBarometerStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_BARO_FAIL);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwGPSStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_GPS_FAIL);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwRangefinderStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_RANGEFINDER_FAIL);
                }
                if (!HW_SENSOR_IS_HEALTHY(getHwPitotmeterStatus())) {
                    return OSD_MESSAGE_STR(OSD_MSG_PITOT_FAIL);
                }
            }
            return OSD_MESSAGE_STR(OSD_MSG_HW_FAIL);
        case ARMING_DISABLED_BOXFAILSAFE:
            return OSD_MESSAGE_STR(OSD_MSG_FS_EN);
        case ARMING_DISABLED_BOXKILLSWITCH:
            return OSD_MESSAGE_STR(OSD_MSG_KILL_SW_EN);
        case ARMING_DISABLED_RC_LINK:
            return OSD_MESSAGE_STR(OSD_MSG_NO_RC_LINK);
        case ARMING_DISABLED_THROTTLE:
            return OSD_MESSAGE_STR(OSD_MSG_THROTTLE_NOT_LOW);
        case ARMING_DISABLED_ROLLPITCH_NOT_CENTERED:
            return OSD_MESSAGE_STR(OSD_MSG_ROLLPITCH_OFFCENTER);
        case ARMING_DISABLED_SERVO_AUTOTRIM:
            return OSD_MESSAGE_STR(OSD_MSG_AUTOTRIM_ACTIVE);
        case ARMING_DISABLED_OOM:
            return OSD_MESSAGE_STR(OSD_MSG_NOT_ENOUGH_MEMORY);
        case ARMING_DISABLED_INVALID_SETTING:
            return OSD_MESSAGE_STR(OSD_MSG_INVALID_SETTING);
        case ARMING_DISABLED_CLI:
            return OSD_MESSAGE_STR(OSD_MSG_CLI_ACTIVE);
        case ARMING_DISABLED_PWM_OUTPUT_ERROR:
            return OSD_MESSAGE_STR(OSD_MSG_PWM_INIT_ERROR);
        case ARMING_DISABLED_NO_PREARM:
            return OSD_MESSAGE_STR(OSD_MSG_NO_PREARM);
        case ARMING_DISABLED_DSHOT_BEEPER:
            return OSD_MESSAGE_STR(OSD_MSG_DSHOT_BEEPER);
            // Cases without message
        case ARMING_DISABLED_CMS_MENU:
            FALLTHROUGH;
        case ARMING_DISABLED_OSD_MENU:
            FALLTHROUGH;
        case ARMING_DISABLED_ALL_FLAGS:
            FALLTHROUGH;
        case ARMED:
            FALLTHROUGH;
        case WAS_EVER_ARMED:
            break;
    }
    return NULL;
}

static const char * osdFailsafePhaseMessage(void)
{
    // See failsafe.h for each phase explanation
    switch (failsafePhase()) {
#ifdef USE_NAV
        case FAILSAFE_RETURN_TO_HOME:
            // XXX: Keep this in sync with OSD_FLYMODE.
            return OSD_MESSAGE_STR(OSD_MSG_RTH_FS);
#endif
        case FAILSAFE_LANDING:
            // This should be considered an emergengy landing
            return OSD_MESSAGE_STR(OSD_MSG_EMERG_LANDING_FS);
        case FAILSAFE_RX_LOSS_MONITORING:
            // Only reachable from FAILSAFE_LANDED, which performs
            // a disarm. Since aircraft has been disarmed, we no
            // longer show failsafe details.
            FALLTHROUGH;
        case FAILSAFE_LANDED:
            // Very brief, disarms and transitions into
            // FAILSAFE_RX_LOSS_MONITORING. Note that it prevents
            // further rearming via ARMING_DISABLED_FAILSAFE_SYSTEM,
            // so we'll show the user how to re-arm in when
            // that flag is the reason to prevent arming.
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_IDLE:
            // This only happens when user has chosen NONE as FS
            // procedure. The recovery messages should be enough.
            FALLTHROUGH;
        case FAILSAFE_IDLE:
            // Failsafe not active
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_DETECTED:
            // Very brief, changes to FAILSAFE_RX_LOSS_RECOVERED
            // or the FS procedure immediately.
            FALLTHROUGH;
        case FAILSAFE_RX_LOSS_RECOVERED:
            // Exiting failsafe
            break;
    }
    return NULL;
}

static const char * osdFailsafeInfoMessage(void)
{
    if (failsafeIsReceivingRxData()) {
        // User must move sticks to exit FS mode
        return OSD_MESSAGE_STR(OSD_MSG_MOVE_EXIT_FS);
    }
    return OSD_MESSAGE_STR(OSD_MSG_RC_RX_LINK_LOST);
}
#if defined(USE_SAFE_HOME)
static const char * divertingToSafehomeMessage(void)
{
	if (NAV_Status.state != MW_NAV_STATE_HOVER_ABOVE_HOME && safehome_applied) {
	    return OSD_MESSAGE_STR(OSD_MSG_DIVERT_SAFEHOME);
	}
	return NULL;
}
#endif

static const char * navigationStateMessage(void)
{
    switch (NAV_Status.state) {
        case MW_NAV_STATE_NONE:
            break;
        case MW_NAV_STATE_RTH_START:
            return OSD_MESSAGE_STR(OSD_MSG_STARTING_RTH);
        case MW_NAV_STATE_RTH_CLIMB:
            return OSD_MESSAGE_STR(OSD_MSG_RTH_CLIMB);
        case MW_NAV_STATE_RTH_ENROUTE:
            return OSD_MESSAGE_STR(OSD_MSG_HEADING_HOME);
        case MW_NAV_STATE_HOLD_INFINIT:
            // Used by HOLD flight modes. No information to add.
            break;
        case MW_NAV_STATE_HOLD_TIMED:
            // "HOLDING WP FOR xx S" Countdown added in osdGetSystemMessage
            break;
        case MW_NAV_STATE_WP_ENROUTE:
            // "TO WP" + WP countdown added in osdGetSystemMessage
            break;
        case MW_NAV_STATE_PROCESS_NEXT:
            return OSD_MESSAGE_STR(OSD_MSG_PREPARE_NEXT_WP);
        case MW_NAV_STATE_DO_JUMP:
            // Not used
            break;
        case MW_NAV_STATE_LAND_START:
            // Not used
            break;
        case MW_NAV_STATE_EMERGENCY_LANDING:
            return OSD_MESSAGE_STR(OSD_MSG_EMERG_LANDING);
        case MW_NAV_STATE_LAND_IN_PROGRESS:
            return OSD_MESSAGE_STR(OSD_MSG_LANDING);
        case MW_NAV_STATE_HOVER_ABOVE_HOME:
            if (STATE(FIXED_WING_LEGACY)) {
#if defined(USE_SAFE_HOME)
                if (safehome_applied) {
                    return OSD_MESSAGE_STR(OSD_MSG_LOITERING_SAFEHOME);
                }
#endif
                return OSD_MESSAGE_STR(OSD_MSG_LOITERING_HOME);
            }
            return OSD_MESSAGE_STR(OSD_MSG_HOVERING);
        case MW_NAV_STATE_LANDED:
            return OSD_MESSAGE_STR(OSD_MSG_LANDED);
        case MW_NAV_STATE_LAND_SETTLE:
            return OSD_MESSAGE_STR(OSD_MSG_PREPARING_LAND);
        case MW_NAV_STATE_LAND_START_DESCENT:
            // Not used
            break;
    }
    return NULL;
}

static void osdFormatMessage(char *buff, size_t size, const char *message, bool isCenteredText)
{
    // String is always filled with Blanks
    memset(buff, SYM_BLANK, size);
    if (message) {
        size_t messageLength = strlen(message);
        int rem = isCenteredText ? MAX(0, (int)size - (int)messageLength) : 0;
        strncpy(buff + rem / 2, message, MIN((int)size - rem / 2, (int)messageLength));
    }
    // Ensure buff is zero terminated
    buff[size - 1] = '\0';
}

/**
 * Draws the battery symbol filled in accordingly to the
 * battery voltage to buff[0].
 **/
static void osdFormatBatteryChargeSymbol(char *buff)
{
    uint8_t p = calculateBatteryPercentage();
    p = (100 - p) / 16.6;
    buff[0] = SYM_BATT_FULL + p;
}

static void osdUpdateBatteryCapacityOrVoltageTextAttributes(textAttributes_t *attr)
{
    if ((getBatteryState() != BATTERY_NOT_PRESENT) && ((batteryUsesCapacityThresholds() && (getBatteryRemainingCapacity() <= currentBatteryProfile->capacity.warning - currentBatteryProfile->capacity.critical)) || ((!batteryUsesCapacityThresholds()) && (getBatteryVoltage() <= getBatteryWarningVoltage()))))
        TEXT_ATTRIBUTES_ADD_BLINK(*attr);
}

void osdCrosshairPosition(uint8_t *x, uint8_t *y)
{
    *x = osdDisplayPort->cols / 2;
    *y = osdDisplayPort->rows / 2;
    *y += osdConfig()->horizon_offset;
}

/**
 * Formats throttle position prefixed by its symbol.
 * Shows output to motor, not stick position
 **/
static void osdFormatThrottlePosition(char *buff, bool autoThr, textAttributes_t *elemAttr)
{
    buff[0] = SYM_BLANK;
    buff[1] = SYM_THR;
    if (autoThr && navigationIsControllingThrottle()) {
        buff[0] = SYM_AUTO_THR0;
        buff[1] = SYM_AUTO_THR1;
        if (isFixedWingAutoThrottleManuallyIncreased()) {
            TEXT_ATTRIBUTES_ADD_BLINK(*elemAttr);
        }
    }
#ifdef USE_POWER_LIMITS
    if (powerLimiterIsLimiting()) {
        TEXT_ATTRIBUTES_ADD_BLINK(*elemAttr);
    }
#endif
    tfp_sprintf(buff + 2, "%3d", getThrottlePercent());
}

/**
 * Formats gvars prefixed by its number (0-indexed). If autoThr
 **/
static void osdFormatGVar(char *buff, uint8_t index)
{
    buff[0] = 'G';
    buff[1] = '0'+index;
    buff[2] = ':';
    #ifdef USE_PROGRAMMING_FRAMEWORK
    osdFormatCentiNumber(buff + 3, (int32_t)gvGet(index)*(int32_t)100, 1, 0, 0, 5);
    #endif
}

#if defined(USE_ESC_SENSOR)
static void osdFormatRpm(char *buff, uint32_t rpm)
{
    buff[0] = SYM_RPM;
    if (rpm) {
        if (rpm >= 1000) {
            osdFormatCentiNumber(buff + 1, rpm / 10, 0, 1, 1, 2);
            buff[3] = 'K';
            buff[4] = '\0';
        }
        else {
            tfp_sprintf(buff + 1, "%3lu", rpm);
        }
    }
    else {
        strcpy(buff + 1, "---");
    }
}
#endif

int32_t osdGetAltitude(void)
{
#if defined(USE_NAV)
    return getEstimatedActualPosition(Z);
#elif defined(USE_BARO)
    return baro.alt;
#else
    return 0;
#endif
}

static inline int32_t osdGetAltitudeMsl(void)
{
#if defined(USE_NAV)
    return getEstimatedActualPosition(Z)+GPS_home.alt;
#elif defined(USE_BARO)
    return baro.alt+GPS_home.alt;
#else
    return 0;
#endif
}

static bool osdIsHeadingValid(void)
{
#ifdef USE_SECONDARY_IMU
    if (secondaryImuState.active && secondaryImuConfig()->useForOsdHeading) {
        return true;
    } else {
        return isImuHeadingValid();
    }
#else
    return isImuHeadingValid();
#endif
}

int16_t osdGetHeading(void)
{
#ifdef USE_SECONDARY_IMU
    if (secondaryImuState.active && secondaryImuConfig()->useForOsdHeading) {
        return secondaryImuState.eulerAngles.values.yaw;
    } else {
        return attitude.values.yaw;
    }
#else
    return attitude.values.yaw;
#endif
}

int16_t osdPanServoHomeDirectionOffset(void)
{
    int8_t servoIndex = osdConfig()->pan_servo_index;
    int16_t servoPosition = servo[servoIndex];
    int16_t servoMiddle = servoParams(servoIndex)->middle;
    return (int16_t)CENTIDEGREES_TO_DEGREES((servoPosition - servoMiddle) * osdConfig()->pan_servo_pwm2centideg);
}

// Returns a heading angle in degrees normalized to [0, 360).
int osdGetHeadingAngle(int angle)
{
    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }
    return angle;
}

#if defined(USE_GPS)

/* Draws a map with the given symbol in the center and given point of interest
 * defined by its distance in meters and direction in degrees.
 * referenceHeading indicates the up direction in the map, in degrees, while
 * referenceSym (if non-zero) is drawn at the upper right corner below a small
 * arrow to indicate the map reference to the user. The drawn argument is an
 * in-out used to store the last position where the craft was drawn to avoid
 * erasing all screen on each redraw.
 */
static void osdDrawMap(int referenceHeading, uint8_t referenceSym, uint8_t centerSym,
                       uint32_t poiDistance, int16_t poiDirection, uint8_t poiSymbol,
                       uint16_t *drawn, uint32_t *usedScale)
{
    // TODO: These need to be tested with several setups. We might
    // need to make them configurable.
    const int hMargin = 5;
    const int vMargin = 3;

    // TODO: Get this from the display driver?
    const int charWidth = 12;
    const int charHeight = 18;

    uint8_t minX = hMargin;
    uint8_t maxX = osdDisplayPort->cols - 1 - hMargin;
    uint8_t minY = vMargin;
    uint8_t maxY = osdDisplayPort->rows - 1 - vMargin;
    uint8_t midX = osdDisplayPort->cols / 2;
    uint8_t midY = osdDisplayPort->rows / 2;

    // Fixed marks
    displayWriteChar(osdDisplayPort, midX, midY, centerSym);

    // First, erase the previous drawing.
    if (OSD_VISIBLE(*drawn)) {
        displayWriteChar(osdDisplayPort, OSD_X(*drawn), OSD_Y(*drawn), SYM_BLANK);
        *drawn = 0;
    }

    uint32_t initialScale;
    const unsigned scaleMultiplier = 2;
    // We try to reduce the scale when the POI will be around half the distance
    // between the center and the closers map edge, to avoid too much jumping
    const int scaleReductionMultiplier = MIN(midX - hMargin, midY - vMargin) / 2;

    switch (osdConfig()->units) {
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_IMPERIAL:
            initialScale = 16; // 16m ~= 0.01miles
            break;
        default:
        case OSD_UNIT_METRIC_MPH:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            initialScale = 10; // 10m as initial scale
            break;
    }

    // Try to keep the same scale when getting closer until we draw over the center point
    uint32_t scale = initialScale;
    if (*usedScale) {
        scale = *usedScale;
        if (scale > initialScale && poiDistance < *usedScale * scaleReductionMultiplier) {
            scale /= scaleMultiplier;
        }
    }

    if (STATE(GPS_FIX)) {

        int directionToPoi = osdGetHeadingAngle(poiDirection - referenceHeading);
        float poiAngle = DEGREES_TO_RADIANS(directionToPoi);
        float poiSin = sin_approx(poiAngle);
        float poiCos = cos_approx(poiAngle);

        // Now start looking for a valid scale that lets us draw everything
        int ii;
        for (ii = 0; ii < 50; ii++) {
            // Calculate location of the aircraft in map
            int points = poiDistance / ((float)scale / charHeight);

            float pointsX = points * poiSin;
            int poiX = midX - roundf(pointsX / charWidth);
            if (poiX < minX || poiX > maxX) {
                scale *= scaleMultiplier;
                continue;
            }

            float pointsY = points * poiCos;
            int poiY = midY + roundf(pointsY / charHeight);
            if (poiY < minY || poiY > maxY) {
                scale *= scaleMultiplier;
                continue;
            }

            if (poiX == midX && poiY == midY) {
                // We're over the map center symbol, so we would be drawing
                // over it even if we increased the scale. Alternate between
                // drawing the center symbol or drawing the POI.
                if (centerSym != SYM_BLANK && OSD_ALTERNATING_CHOICES(1000, 2) == 0) {
                    break;
                }
            } else {

                uint16_t c;
                if (displayReadCharWithAttr(osdDisplayPort, poiX, poiY, &c, NULL) && c != SYM_BLANK) {
                    // Something else written here, increase scale. If the display doesn't support reading
                    // back characters, we assume there's nothing.
                    //
                    // If we're close to the center, decrease scale. Otherwise increase it.
                    uint8_t centerDeltaX = (maxX - minX) / (scaleMultiplier * 2);
                    uint8_t centerDeltaY = (maxY - minY) / (scaleMultiplier * 2);
                    if (poiX >= midX - centerDeltaX && poiX <= midX + centerDeltaX &&
                        poiY >= midY - centerDeltaY && poiY <= midY + centerDeltaY &&
                        scale > scaleMultiplier) {

                        scale /= scaleMultiplier;
                    } else {
                        scale *= scaleMultiplier;
                    }
                    continue;
                }
            }

            // Draw the point on the map
            if (poiSymbol == SYM_ARROW_UP) {
                // Drawing aircraft, rotate
                int mapHeading = osdGetHeadingAngle(DECIDEGREES_TO_DEGREES(osdGetHeading()) - referenceHeading);
                poiSymbol += mapHeading * 2 / 45;
            }
            displayWriteChar(osdDisplayPort, poiX, poiY, poiSymbol);

            // Update saved location
            *drawn = OSD_POS(poiX, poiY) | OSD_VISIBLE_FLAG;
            break;
        }
    }

    *usedScale = scale;

    // Update global map data for scale and reference
    osdMapData.scale = scale;
    osdMapData.referenceSymbol = referenceSym;
}

/* Draws a map with the home in the center and the craft moving around.
 * See osdDrawMap() for reference.
 */
static void osdDrawHomeMap(int referenceHeading, uint8_t referenceSym, uint16_t *drawn, uint32_t *usedScale)
{
    osdDrawMap(referenceHeading, referenceSym, SYM_HOME, GPS_distanceToHome, GPS_directionToHome, SYM_ARROW_UP, drawn, usedScale);
}

/* Draws a map with the aircraft in the center and the home moving around.
 * See osdDrawMap() for reference.
 */
static void osdDrawRadar(uint16_t *drawn, uint32_t *usedScale)
{
    int16_t reference = DECIDEGREES_TO_DEGREES(osdGetHeading());
    int16_t poiDirection = osdGetHeadingAngle(GPS_directionToHome + 180);
    osdDrawMap(reference, 0, SYM_ARROW_UP, GPS_distanceToHome, poiDirection, SYM_HOME, drawn, usedScale);
}

static uint16_t crc_accumulate(uint8_t data, uint16_t crcAccum)
{
    uint8_t tmp;
    tmp = data ^ (uint8_t)(crcAccum & 0xff);
    tmp ^= (tmp << 4);
    crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    return crcAccum;
}


static void osdDisplayTelemetry(void)
{
    uint32_t          trk_data;
    uint16_t          trk_crc = 0;
    char              trk_buffer[31];
    static int16_t    trk_elevation = 127;
    static uint16_t   trk_bearing   = 0;

    if (ARMING_FLAG(ARMED)) {
      if (STATE(GPS_FIX)){
        if (GPS_distanceToHome > 5) {
          trk_bearing = GPS_directionToHome;
          trk_bearing += 360 + 180;
          trk_bearing %= 360;
          int32_t alt = CENTIMETERS_TO_METERS(osdGetAltitude());
          float at = atan2(alt, GPS_distanceToHome);
          trk_elevation = (float)at * 57.2957795; // 57.2957795 = 1 rad
          trk_elevation += 37; // because elevation in telemetry should be from -37 to 90
          if (trk_elevation < 0) {
            trk_elevation = 0;
          }
        }
      }
    }
    else{
      trk_elevation = 127;
      trk_bearing   = 0;
    }

    trk_data = 0;                                                // bit  0    - packet type 0 = bearing/elevation, 1 = 2 byte data packet
    trk_data = trk_data | (uint32_t)(0x7F & trk_elevation) << 1; // bits 1-7  - elevation angle to target. NOTE number is abused. constrained value of -37 to 90 sent as 0 to 127.
    trk_data = trk_data | (uint32_t)trk_bearing << 8;            // bits 8-17 - bearing angle to target. 0 = true north. 0 to 360
    trk_crc = crc_accumulate(0xFF & trk_data, trk_crc);          // CRC First Byte  bits 0-7
    trk_crc = crc_accumulate(0xFF & trk_bearing, trk_crc);       // CRC Second Byte bits 8-15
    trk_crc = crc_accumulate(trk_bearing >> 8, trk_crc);         // CRC Third Byte  bits  16-17
    trk_data = trk_data | (uint32_t)trk_crc << 17;               // bits 18-29 CRC & 0x3FFFF

    for (uint8_t t_ctr = 0; t_ctr < 30; t_ctr++) {               // Prepare screen buffer and write data line.
      if (trk_data & (uint32_t)1 << t_ctr){
        trk_buffer[29 - t_ctr] = SYM_TELEMETRY_0;
      }
      else{
        trk_buffer[29 - t_ctr] = SYM_TELEMETRY_1;
      }
    }
    trk_buffer[30] = 0;
    displayWrite(osdDisplayPort, 0, 0, trk_buffer);
    if (osdConfig()->telemetry>1){
      displayWrite(osdDisplayPort, 0, 3, trk_buffer);               // Test display because normal telemetry line is not visible
    }
}
#endif

static void osdFormatPidControllerOutput(char *buff, const char *label, const pidController_t *pidController, uint8_t scale, bool showDecimal) {
    strcpy(buff, label);
    for (uint8_t i = strlen(label); i < 5; ++i) buff[i] = ' ';
    uint8_t decimals = showDecimal ? 1 : 0;
    osdFormatCentiNumber(buff + 5, pidController->proportional * scale, 0, decimals, 0, 4);
    buff[9] = ' ';
    osdFormatCentiNumber(buff + 10, pidController->integrator * scale, 0, decimals, 0, 4);
    buff[14] = ' ';
    osdFormatCentiNumber(buff + 15, pidController->derivative * scale, 0, decimals, 0, 4);
    buff[19] = ' ';
    osdFormatCentiNumber(buff + 20, pidController->output_constrained * scale, 0, decimals, 0, 4);
    buff[24] = '\0';
}

static void osdDisplayBatteryVoltage(uint8_t elemPosX, uint8_t elemPosY, uint16_t voltage, uint8_t digits, uint8_t decimals)
{
    char buff[6];
    textAttributes_t elemAttr = TEXT_ATTRIBUTES_NONE;

    osdFormatBatteryChargeSymbol(buff);
    buff[1] = '\0';
    osdUpdateBatteryCapacityOrVoltageTextAttributes(&elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    digits = MIN(digits, 4);
    osdFormatCentiNumber(buff, voltage, 0, decimals, 0, digits);
    buff[digits] = SYM_VOLT;
    buff[digits+1] = '\0';
    if ((getBatteryState() != BATTERY_NOT_PRESENT) && (getBatteryVoltage() <= getBatteryWarningVoltage()))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 1, elemPosY, buff, elemAttr);
}

static void osdDisplayFlightPIDValues(uint8_t elemPosX, uint8_t elemPosY, const char *str, pidIndex_e pidIndex, adjustmentFunction_e adjFuncP, adjustmentFunction_e adjFuncI, adjustmentFunction_e adjFuncD, adjustmentFunction_e adjFuncFF)
{
    textAttributes_t elemAttr;
    char buff[4];

    const pid8_t *pid = &pidBank()->pid[pidIndex];
    pidType_e pidType = pidIndexGetType(pidIndex);

    displayWrite(osdDisplayPort, elemPosX, elemPosY, str);

    if (pidType == PID_TYPE_NONE) {
        // PID is not used in this configuration. Draw dashes.
        // XXX: Keep this in sync with the %3d format and spacing used below
        displayWrite(osdDisplayPort, elemPosX + 6, elemPosY, "-   -   -   -");
        return;
    }

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->P);
    if ((isAdjustmentFunctionSelected(adjFuncP)) || (((adjFuncP == ADJUSTMENT_ROLL_P) || (adjFuncP == ADJUSTMENT_PITCH_P)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_P))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->I);
    if ((isAdjustmentFunctionSelected(adjFuncI)) || (((adjFuncI == ADJUSTMENT_ROLL_I) || (adjFuncI == ADJUSTMENT_PITCH_I)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_I))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 8, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->D);
    if ((isAdjustmentFunctionSelected(adjFuncD)) || (((adjFuncD == ADJUSTMENT_ROLL_D) || (adjFuncD == ADJUSTMENT_PITCH_D)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_D))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 12, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->FF);
    if ((isAdjustmentFunctionSelected(adjFuncFF)) || (((adjFuncFF == ADJUSTMENT_ROLL_FF) || (adjFuncFF == ADJUSTMENT_PITCH_FF)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_FF))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 16, elemPosY, buff, elemAttr);
}

static void osdDisplayNavPIDValues(uint8_t elemPosX, uint8_t elemPosY, const char *str, pidIndex_e pidIndex, adjustmentFunction_e adjFuncP, adjustmentFunction_e adjFuncI, adjustmentFunction_e adjFuncD)
{
    textAttributes_t elemAttr;
    char buff[4];

    const pid8_t *pid = &pidBank()->pid[pidIndex];
    pidType_e pidType = pidIndexGetType(pidIndex);

    displayWrite(osdDisplayPort, elemPosX, elemPosY, str);

    if (pidType == PID_TYPE_NONE) {
        // PID is not used in this configuration. Draw dashes.
        // XXX: Keep this in sync with the %3d format and spacing used below
        displayWrite(osdDisplayPort, elemPosX + 6, elemPosY, "-   -   -");
        return;
    }

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->P);
    if ((isAdjustmentFunctionSelected(adjFuncP)) || (((adjFuncP == ADJUSTMENT_ROLL_P) || (adjFuncP == ADJUSTMENT_PITCH_P)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_P))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pid->I);
    if ((isAdjustmentFunctionSelected(adjFuncI)) || (((adjFuncI == ADJUSTMENT_ROLL_I) || (adjFuncI == ADJUSTMENT_PITCH_I)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_I))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 8, elemPosY, buff, elemAttr);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    tfp_sprintf(buff, "%3d", pidType == PID_TYPE_PIFF ? pid->FF : pid->D);
    if ((isAdjustmentFunctionSelected(adjFuncD)) || (((adjFuncD == ADJUSTMENT_ROLL_D) || (adjFuncD == ADJUSTMENT_PITCH_D)) && (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_D))))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + 12, elemPosY, buff, elemAttr);
}

static void osdDisplayAdjustableDecimalValue(uint8_t elemPosX, uint8_t elemPosY, const char *str, const uint8_t valueOffset, const float value, const uint8_t valueLength, const uint8_t maxDecimals, adjustmentFunction_e adjFunc) {
    char buff[8];
    textAttributes_t elemAttr;
    displayWrite(osdDisplayPort, elemPosX, elemPosY, str);

    elemAttr = TEXT_ATTRIBUTES_NONE;
    osdFormatCentiNumber(buff, value * 100, 0, maxDecimals, 0, MIN(valueLength, 8));
    if (isAdjustmentFunctionSelected(adjFunc))
        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
    displayWriteWithAttr(osdDisplayPort, elemPosX + strlen(str) + 1 + valueOffset, elemPosY, buff, elemAttr);
}

int8_t getGeoWaypointNumber(int8_t waypointIndex)
{
    static int8_t lastWaypointIndex = 1;
    static int8_t geoWaypointIndex;

    if (waypointIndex != lastWaypointIndex) {
        lastWaypointIndex = geoWaypointIndex = waypointIndex;
        for (uint8_t i = 0; i <= waypointIndex; i++) {
            if (posControl.waypointList[i].action == NAV_WP_ACTION_SET_POI ||
                posControl.waypointList[i].action == NAV_WP_ACTION_SET_HEAD ||
                posControl.waypointList[i].action == NAV_WP_ACTION_JUMP) {
                    geoWaypointIndex -= 1;
            }
        }
    }

    return geoWaypointIndex + 1;
}

static bool osdDrawSingleElement(uint8_t item)
{
    uint16_t pos = osdLayoutsConfig()->item_pos[currentLayout][item];
    if (!OSD_VISIBLE(pos)) {
        return false;
    }
    uint8_t elemPosX = OSD_X(pos);
    uint8_t elemPosY = OSD_Y(pos);
    textAttributes_t elemAttr = TEXT_ATTRIBUTES_NONE;
    char buff[32] = {0};

    switch (item) {
    case OSD_RSSI_VALUE:
        {
            uint16_t osdRssi = osdConvertRSSI();
            buff[0] = SYM_RSSI;
            tfp_sprintf(buff + 1, "%2d", osdRssi);
            if (osdRssi < osdConfig()->rssi_alarm) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }

    case OSD_MAIN_BATT_VOLTAGE:
        osdDisplayBatteryVoltage(elemPosX, elemPosY, getBatteryRawVoltage(), 2 + osdConfig()->main_voltage_decimals, osdConfig()->main_voltage_decimals);
        return true;

    case OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE:
        osdDisplayBatteryVoltage(elemPosX, elemPosY, getBatterySagCompensatedVoltage(), 2 + osdConfig()->main_voltage_decimals, osdConfig()->main_voltage_decimals);
        return true;

    case OSD_CURRENT_DRAW:
        osdFormatCentiNumber(buff, getAmperage(), 0, 2, 0, 3);
        buff[3] = SYM_AMP;
        buff[4] = '\0';

        uint8_t current_alarm = osdConfig()->current_alarm;
        if ((current_alarm > 0) && ((getAmperage() / 100.0f) > current_alarm)) {
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        }
        break;

    case OSD_MAH_DRAWN:
        tfp_sprintf(buff, "%4d", (int)getMAhDrawn());
        buff[4] = SYM_MAH;
        buff[5] = '\0';
        osdUpdateBatteryCapacityOrVoltageTextAttributes(&elemAttr);
        break;

    case OSD_WH_DRAWN:
        osdFormatCentiNumber(buff, getMWhDrawn() / 10, 0, 2, 0, 3);
        osdUpdateBatteryCapacityOrVoltageTextAttributes(&elemAttr);
        buff[3] = SYM_WH;
        buff[4] = '\0';
        break;

    case OSD_BATTERY_REMAINING_CAPACITY:

        if (currentBatteryProfile->capacity.value == 0)
            tfp_sprintf(buff, "  NA");
        else if (!batteryWasFullWhenPluggedIn())
            tfp_sprintf(buff, "  NF");
        else if (currentBatteryProfile->capacity.unit == BAT_CAPACITY_UNIT_MAH)
            tfp_sprintf(buff, "%4lu", getBatteryRemainingCapacity());
        else // currentBatteryProfile->capacity.unit == BAT_CAPACITY_UNIT_MWH
            osdFormatCentiNumber(buff + 1, getBatteryRemainingCapacity() / 10, 0, 2, 0, 3);

        buff[4] = currentBatteryProfile->capacity.unit == BAT_CAPACITY_UNIT_MAH ? SYM_MAH : SYM_WH;
        buff[5] = '\0';

        if ((getBatteryState() != BATTERY_NOT_PRESENT) && batteryUsesCapacityThresholds() && (getBatteryRemainingCapacity() <= currentBatteryProfile->capacity.warning - currentBatteryProfile->capacity.critical))
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);

        break;

    case OSD_BATTERY_REMAINING_PERCENT:
        tfp_sprintf(buff, "%3d%%", calculateBatteryPercentage());
        osdUpdateBatteryCapacityOrVoltageTextAttributes(&elemAttr);
        break;

    case OSD_POWER_SUPPLY_IMPEDANCE:
        if (isPowerSupplyImpedanceValid())
            tfp_sprintf(buff, "%3d", getPowerSupplyImpedance());
        else
            strcpy(buff, "---");
        buff[3] = SYM_MILLIOHM;
        buff[4] = '\0';
        break;

#ifdef USE_GPS
    case OSD_GPS_SATS:
        buff[0] = SYM_SAT_L;
        buff[1] = SYM_SAT_R;
        tfp_sprintf(buff + 2, "%2d", gpsSol.numSat);
        if (!STATE(GPS_FIX)) {
            if (getHwGPSStatus() == HW_SENSOR_UNAVAILABLE || getHwGPSStatus() == HW_SENSOR_UNHEALTHY) {
                strcpy(buff + 2, "X!");
            }
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        }
        break;

    case OSD_GPS_SPEED:
        osdFormatVelocityStr(buff, gpsSol.groundSpeed, false);
        break;

    case OSD_3D_SPEED:
        {
            osdFormatVelocityStr(buff, osdGet3DSpeed(), true);
            break;
        }

    case OSD_GLIDESLOPE:
        {
            float horizontalSpeed = gpsSol.groundSpeed;
            float sinkRate = -getEstimatedActualVelocity(Z);
            static pt1Filter_t gsFilterState;
            const timeMs_t currentTimeMs = millis();
            static timeMs_t gsUpdatedTimeMs;
            float glideSlope = horizontalSpeed / sinkRate;
            glideSlope = pt1FilterApply4(&gsFilterState, isnormal(glideSlope) ? glideSlope : 200, 0.5, MS2S(currentTimeMs - gsUpdatedTimeMs));
            gsUpdatedTimeMs = currentTimeMs;

            buff[0] = SYM_GLIDESLOPE;
            if (glideSlope > 0.0f && glideSlope < 100.0f) {
                osdFormatCentiNumber(buff + 1, glideSlope * 100.0f, 0, 2, 0, 3);
            } else {
                buff[1] = buff[2] = buff[3] = '-';
            }
            buff[4] = '\0';
            break;
        }

    case OSD_GPS_LAT:
        osdFormatCoordinate(buff, SYM_LAT, gpsSol.llh.lat);
        break;

    case OSD_GPS_LON:
        osdFormatCoordinate(buff, SYM_LON, gpsSol.llh.lon);
        break;

    case OSD_HOME_DIR:
        {
            if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME) && isImuHeadingValid()) {
                if (GPS_distanceToHome < (navConfig()->general.min_rth_distance / 100) ) {
                    displayWriteChar(osdDisplayPort, elemPosX, elemPosY, SYM_HOME_NEAR);
                }
                else
                {
                    int16_t panHomeDirOffset = 0;
                    if (!(osdConfig()->pan_servo_pwm2centideg == 0)){
                        panHomeDirOffset = osdPanServoHomeDirectionOffset();
                    }
                    int homeDirection = GPS_directionToHome - DECIDEGREES_TO_DEGREES(osdGetHeading()) + panHomeDirOffset;
                    osdDrawDirArrow(osdDisplayPort, osdGetDisplayPortCanvas(), OSD_DRAW_POINT_GRID(elemPosX, elemPosY), homeDirection);
                }
            } else {
                // No home or no fix or unknown heading, blink.
                // If we're unarmed, show the arrow pointing up so users can see the arrow
                // while configuring the OSD. If we're armed, show a '-' indicating that
                // we don't know the direction to home.
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
                displayWriteCharWithAttr(osdDisplayPort, elemPosX, elemPosY, ARMING_FLAG(ARMED) ? '-' : SYM_ARROW_UP, elemAttr);
            }
            return true;
        }

    case OSD_HOME_HEADING_ERROR:
        {
            buff[0] = SYM_HOME;
            buff[1] = SYM_HEADING;

            if (isImuHeadingValid() && navigationPositionEstimateIsHealthy()) {
                int16_t h = lrintf(CENTIDEGREES_TO_DEGREES((float)wrap_18000(DEGREES_TO_CENTIDEGREES((int32_t)GPS_directionToHome) - DECIDEGREES_TO_CENTIDEGREES((int32_t)osdGetHeading()))));
                tfp_sprintf(buff + 2, "%4d", h);
            } else {
                strcpy(buff + 2, "----");
            }

            buff[6] = SYM_DEGREES;
            buff[7] = '\0';
            break;
        }

    case OSD_HOME_DIST:
        {
            buff[0] = SYM_HOME;
            osdFormatDistanceSymbol(&buff[1], GPS_distanceToHome * 100, 0);
            uint16_t dist_alarm = osdConfig()->dist_alarm;
            if (dist_alarm > 0 && GPS_distanceToHome > dist_alarm) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
        }
        break;

    case OSD_TRIP_DIST:
        buff[0] = SYM_TOTAL;
        osdFormatDistanceSymbol(buff + 1, getTotalTravelDistance(), 0);
        break;

    case OSD_HEADING:
        {
            buff[0] = SYM_HEADING;
            if (osdIsHeadingValid()) {
                int16_t h = DECIDEGREES_TO_DEGREES(osdGetHeading());
                if (h < 0) {
                    h += 360;
                }
                tfp_sprintf(&buff[1], "%3d", h);
            } else {
                buff[1] = buff[2] = buff[3] = '-';
            }
            buff[4] = '\0';
            break;
        }

    case OSD_COURSE_HOLD_ERROR:
        {
            if (ARMING_FLAG(ARMED) && !FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
                displayWrite(osdDisplayPort, elemPosX, elemPosY, "     ");
                return true;
            }

            buff[0] = SYM_HEADING;

            if ((!ARMING_FLAG(ARMED)) || (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && isAdjustingPosition())) {
                buff[1] = buff[2] = buff[3] = '-';
            } else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
                int16_t herr = lrintf(CENTIDEGREES_TO_DEGREES((float)navigationGetHeadingError()));
                if (ABS(herr) > 99)
                    strcpy(buff + 1, ">99");
                else
                    tfp_sprintf(buff + 1, "%3d", herr);
            }

            buff[4] = SYM_DEGREES;
            buff[5] = '\0';
            break;
        }

    case OSD_COURSE_HOLD_ADJUSTMENT:
        {
            int16_t heading_adjust = lrintf(CENTIDEGREES_TO_DEGREES((float)getCruiseHeadingAdjustment()));

            if (ARMING_FLAG(ARMED) && ((!FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) || !(isAdjustingPosition() || isAdjustingHeading() || (heading_adjust != 0)))) {
                displayWrite(osdDisplayPort, elemPosX, elemPosY, "      ");
                return true;
            }

            buff[0] = SYM_HEADING;

            if (!ARMING_FLAG(ARMED)) {
                buff[1] = buff[2] = buff[3] = buff[4] = '-';
            } else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) {
                tfp_sprintf(buff + 1, "%4d", heading_adjust);
            }

            buff[5] = SYM_DEGREES;
            buff[6] = '\0';
            break;
        }

    case OSD_GPS_HDOP:
        {
            buff[0] = SYM_HDP_L;
            buff[1] = SYM_HDP_R;
            int32_t centiHDOP = 100 * gpsSol.hdop / HDOP_SCALE;
            osdFormatCentiNumber(&buff[2], centiHDOP, 0, 1, 0, 2);
            break;
        }

    case OSD_MAP_NORTH:
        {
            static uint16_t drawn = 0;
            static uint32_t scale = 0;
            osdDrawHomeMap(0, 'N', &drawn, &scale);
            return true;
        }
    case OSD_MAP_TAKEOFF:
        {
            static uint16_t drawn = 0;
            static uint32_t scale = 0;
            osdDrawHomeMap(CENTIDEGREES_TO_DEGREES(navigationGetHomeHeading()), 'T', &drawn, &scale);
            return true;
        }
    case OSD_RADAR:
        {
            static uint16_t drawn = 0;
            static uint32_t scale = 0;
            osdDrawRadar(&drawn, &scale);
            return true;
        }
#endif // GPS

    case OSD_ALTITUDE:
        {
            int32_t alt = osdGetAltitude();
            osdFormatAltitudeSymbol(buff, alt);
            uint16_t alt_alarm = osdConfig()->alt_alarm;
            uint16_t neg_alt_alarm = osdConfig()->neg_alt_alarm;
            if ((alt_alarm > 0 && CENTIMETERS_TO_METERS(alt) > alt_alarm) ||
                (neg_alt_alarm > 0 && alt < 0 && -CENTIMETERS_TO_METERS(alt) > neg_alt_alarm)) {

                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }

    case OSD_ALTITUDE_MSL:
        {
            int32_t alt = osdGetAltitudeMsl();
            osdFormatAltitudeSymbol(buff, alt);
            break;
        }

#ifdef USE_RANGEFINDER
    case OSD_RANGEFINDER:
        {
            int32_t range = rangefinderGetLatestRawAltitude();
            if (range < 0) {
                buff[0] = '-';
            } else {
                osdFormatDistanceSymbol(buff, range, 1);
            }
        }
        break;
#endif

    case OSD_ONTIME:
        {
            osdFormatOnTime(buff);
            break;
        }

    case OSD_FLYTIME:
        {
            osdFormatFlyTime(buff, &elemAttr);
            break;
        }

    case OSD_ONTIME_FLYTIME:
        {
            if (ARMING_FLAG(ARMED)) {
                osdFormatFlyTime(buff, &elemAttr);
            } else {
                osdFormatOnTime(buff);
            }
            break;
        }

    case OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH:
        {
            /*static int32_t updatedTimeSeconds = 0;*/
            static int32_t timeSeconds = -1;
#if defined(USE_ADC) && defined(USE_GPS)
            static timeUs_t updatedTimestamp = 0;
            timeUs_t currentTimeUs = micros();
            if (cmpTimeUs(currentTimeUs, updatedTimestamp) >= MS2US(1000)) {
#ifdef USE_WIND_ESTIMATOR
                timeSeconds = calculateRemainingFlightTimeBeforeRTH(osdConfig()->estimations_wind_compensation);
#else
                timeSeconds = calculateRemainingFlightTimeBeforeRTH(false);
#endif
                updatedTimestamp = currentTimeUs;
            }
#endif
            if ((!ARMING_FLAG(ARMED)) || (timeSeconds == -1)) {
                buff[0] = SYM_FLY_M;
                strcpy(buff + 1, "--:--");
#if defined(USE_ADC) && defined(USE_GPS)
                updatedTimestamp = 0;
#endif
            } else if (timeSeconds == -2) {
                // Wind is too strong to come back with cruise throttle
                buff[0] = SYM_FLY_M;
                buff[1] = buff[2] = buff[4] = buff[5] = SYM_WIND_HORIZONTAL;
                buff[3] = ':';
                buff[6] = '\0';
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            } else {
                osdFormatTime(buff, timeSeconds, SYM_FLY_M, SYM_FLY_H);
                if (timeSeconds == 0)
                    TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
        }
        break;

    case OSD_REMAINING_DISTANCE_BEFORE_RTH:;
        static int32_t distanceMeters = -1;
#if defined(USE_ADC) && defined(USE_GPS)
        static timeUs_t updatedTimestamp = 0;
        timeUs_t currentTimeUs = micros();
        if (cmpTimeUs(currentTimeUs, updatedTimestamp) >= MS2US(1000)) {
#ifdef USE_WIND_ESTIMATOR
            distanceMeters = calculateRemainingDistanceBeforeRTH(osdConfig()->estimations_wind_compensation);
#else
            distanceMeters = calculateRemainingDistanceBeforeRTH(false);
#endif
            updatedTimestamp = currentTimeUs;
        }
#endif
        buff[0] = SYM_TRIP_DIST;
        if ((!ARMING_FLAG(ARMED)) || (distanceMeters == -1)) {
            buff[4] = SYM_DIST_M;
            buff[5] = '\0';
            strcpy(buff + 1, "---");
        } else if (distanceMeters == -2) {
            // Wind is too strong to come back with cruise throttle
            buff[1] = buff[2] = buff[3] = SYM_WIND_HORIZONTAL;
            buff[4] = SYM_DIST_M;
            buff[5] = '\0';
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        } else {
            osdFormatDistanceSymbol(buff + 1, distanceMeters * 100, 0);
            if (distanceMeters == 0)
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        }
        break;

    case OSD_FLYMODE:
        {
            char *p = "ACRO";

            if (FLIGHT_MODE(FAILSAFE_MODE))
                p = "!FS!";
            else if (FLIGHT_MODE(MANUAL_MODE))
                p = "MANU";
            else if (FLIGHT_MODE(TURTLE_MODE))
                p = "TURT";
            else if (FLIGHT_MODE(NAV_RTH_MODE))
                p = isWaypointMissionRTHActive() ? "WRTH" : "RTH ";
            else if (FLIGHT_MODE(NAV_POSHOLD_MODE))
                p = "HOLD";
            else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE))
                p = "CRUZ";
            else if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE))
                p = "CRSH";
            else if (FLIGHT_MODE(NAV_WP_MODE))
                p = " WP ";
            else if (FLIGHT_MODE(NAV_ALTHOLD_MODE) && navigationRequiresAngleMode()) {
                // If navigationRequiresAngleMode() returns false when ALTHOLD is active,
                // it means it can be combined with ANGLE, HORIZON, ACRO, etc...
                // and its display is handled by OSD_MESSAGES rather than OSD_FLYMODE.
                p = " AH ";
            }
            else if (FLIGHT_MODE(ANGLE_MODE))
                p = "ANGL";
            else if (FLIGHT_MODE(HORIZON_MODE))
                p = "HOR ";

            displayWrite(osdDisplayPort, elemPosX, elemPosY, p);
            return true;
        }

    case OSD_CRAFT_NAME:
        osdFormatCraftName(buff);
        break;

    case OSD_THROTTLE_POS:
    {
        osdFormatThrottlePosition(buff, false, NULL);
        break;
    }

    case OSD_VTX_CHANNEL:
        {
            vtxDeviceOsdInfo_t osdInfo;
            vtxCommonGetOsdInfo(vtxCommonDevice(), &osdInfo);

            tfp_sprintf(buff, "CH:%c%s:", osdInfo.bandLetter, osdInfo.channelName);
            displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);

            tfp_sprintf(buff, "%c", osdInfo.powerIndexLetter);
            if (isAdjustmentFunctionSelected(ADJUSTMENT_VTX_POWER_LEVEL)) TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            displayWriteWithAttr(osdDisplayPort, elemPosX + 6, elemPosY, buff, elemAttr);
            return true;
        }
        break;

    case OSD_VTX_POWER:
        {
            vtxDeviceOsdInfo_t osdInfo;
            vtxCommonGetOsdInfo(vtxCommonDevice(), &osdInfo);

            tfp_sprintf(buff, "%c", osdInfo.powerIndexLetter);
            if (isAdjustmentFunctionSelected(ADJUSTMENT_VTX_POWER_LEVEL)) TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, buff, elemAttr);
            return true;
        }

#if defined(USE_SERIALRX_CRSF)
    case OSD_CRSF_RSSI_DBM:
        {
            int16_t rssi = rxLinkStatistics.uplinkRSSI;
            buff[0] = (rxLinkStatistics.activeAnt == 0) ? SYM_RSSI : SYM_2RSS; // Separate symbols for each antenna
            if (rssi <= -100) {
                tfp_sprintf(buff + 1, "%4d%c", rssi, SYM_DBM);
            } else {
                tfp_sprintf(buff + 1, "%3d%c%c", rssi, SYM_DBM, ' ');
            }
            if (!failsafeIsReceivingRxData()){
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            } else if (osdConfig()->rssi_dbm_alarm && rssi < osdConfig()->rssi_dbm_alarm) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }
    case OSD_CRSF_LQ:
        {
            buff[0] = SYM_LQ;
            int16_t statsLQ = rxLinkStatistics.uplinkLQ;
            int16_t scaledLQ = scaleRange(constrain(statsLQ, 0, 100), 0, 100, 170, 300);
            switch (osdConfig()->crsf_lq_format) {
                case OSD_CRSF_LQ_TYPE1:
                    tfp_sprintf(buff+1, "%3d", rxLinkStatistics.uplinkLQ);
                    break;
                case OSD_CRSF_LQ_TYPE2:
                    tfp_sprintf(buff+1, "%d:%3d", rxLinkStatistics.rfMode, rxLinkStatistics.uplinkLQ);
                    break;
                case OSD_CRSF_LQ_TYPE3:
                    tfp_sprintf(buff+1, "%3d", rxLinkStatistics.rfMode >= 2 ? scaledLQ : rxLinkStatistics.uplinkLQ);
                    break;
            }
            if (!failsafeIsReceivingRxData()){
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            } else if (rxLinkStatistics.uplinkLQ < osdConfig()->link_quality_alarm) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }

    case OSD_CRSF_SNR_DB:
        {
            static pt1Filter_t snrFilterState;
            static timeMs_t snrUpdated = 0;
            int8_t snrFiltered = pt1FilterApply4(&snrFilterState, rxLinkStatistics.uplinkSNR, 0.5f, MS2S(millis() - snrUpdated));
            snrUpdated = millis();

            const char* showsnr = "-20";
            const char* hidesnr = "   ";
            if (snrFiltered > osdConfig()->snr_alarm) {
                if (cmsInMenu) {
                    buff[0] = SYM_SNR;
                    tfp_sprintf(buff + 1, "%s%c", showsnr, SYM_DB);
                } else {
                    buff[0] = SYM_BLANK;
                    tfp_sprintf(buff + 1, "%s%c", hidesnr, SYM_BLANK);
                }
            } else if (snrFiltered <= osdConfig()->snr_alarm) {
                buff[0] = SYM_SNR;
                if (snrFiltered <= -10) {
                    tfp_sprintf(buff + 1, "%3d%c", snrFiltered, SYM_DB);
                } else {
                    tfp_sprintf(buff + 1, "%2d%c%c", snrFiltered, SYM_DB, ' ');
                }
            }
            break;
        }

    case OSD_CRSF_TX_POWER:
        {
            tfp_sprintf(buff, "%4d%c", rxLinkStatistics.uplinkTXPower, SYM_MW);
            break;
        }
#endif

    case OSD_CROSSHAIRS: // Hud is a sub-element of the crosshair

        osdCrosshairPosition(&elemPosX, &elemPosY);
        osdHudDrawCrosshair(osdGetDisplayPortCanvas(), elemPosX, elemPosY);

        if (osdConfig()->hud_homing && STATE(GPS_FIX) && STATE(GPS_FIX_HOME) && isImuHeadingValid()) {
            osdHudDrawHoming(elemPosX, elemPosY);
        }

        if (STATE(GPS_FIX) && isImuHeadingValid()) {

            if (osdConfig()->hud_homepoint || osdConfig()->hud_radar_disp > 0 || osdConfig()->hud_wp_disp > 0) {
                    osdHudClear();
            }

            // -------- POI : Home point

            if (osdConfig()->hud_homepoint) { // Display the home point (H)
                osdHudDrawPoi(GPS_distanceToHome, GPS_directionToHome, -osdGetAltitude() / 100, 0, SYM_HOME, 0 , 0);
            }

            // -------- POI : Nearby aircrafts from ESP32 radar

            if (osdConfig()->hud_radar_disp > 0) { // Display the POI from the radar
                for (uint8_t i = 0; i < osdConfig()->hud_radar_disp; i++) {
                    if (radar_pois[i].gps.lat != 0 && radar_pois[i].gps.lon != 0 && radar_pois[i].state < 2) { // state 2 means POI has been lost and must be skipped
                        fpVector3_t poi;
                        geoConvertGeodeticToLocal(&poi, &posControl.gpsOrigin, &radar_pois[i].gps, GEO_ALT_RELATIVE);
                        radar_pois[i].distance = calculateDistanceToDestination(&poi) / 100; // In meters

                        if (radar_pois[i].distance >= osdConfig()->hud_radar_range_min && radar_pois[i].distance <= osdConfig()->hud_radar_range_max) {
                            radar_pois[i].direction = calculateBearingToDestination(&poi) / 100; // In °
                            radar_pois[i].altitude = (radar_pois[i].gps.alt - osdGetAltitudeMsl()) / 100;
                            osdHudDrawPoi(radar_pois[i].distance, osdGetHeadingAngle(radar_pois[i].direction), radar_pois[i].altitude, 1, 65 + i, radar_pois[i].heading, radar_pois[i].lq);
                        }
                    }
                }

                if (osdConfig()->hud_radar_nearest > 0) { // Display extra datas for 1 POI closer than a set distance
                    int poi_id = radarGetNearestPOI();
                    if (poi_id >= 0 && radar_pois[poi_id].distance <= osdConfig()->hud_radar_nearest) {
                        osdHudDrawExtras(poi_id);
                    }
                }
            }

            // -------- POI : Next waypoints from navigation

            if (osdConfig()->hud_wp_disp > 0 && posControl.waypointListValid && posControl.waypointCount > 0) { // Display the next waypoints
                gpsLocation_t wp2;
                int j;

                for (int i = osdConfig()->hud_wp_disp - 1; i >= 0 ; i--) { // Display in reverse order so the next WP is always written on top
                    j = posControl.activeWaypointIndex + i;
                    if (j > posControl.waypointCount - 1) { // limit to max WP index for mission
                        break;
                    }
                    if (posControl.waypointList[j].lat != 0 && posControl.waypointList[j].lon != 0) {
                        wp2.lat = posControl.waypointList[j].lat;
                        wp2.lon = posControl.waypointList[j].lon;
                        wp2.alt = posControl.waypointList[j].alt;
                        fpVector3_t poi;
                        geoConvertGeodeticToLocal(&poi, &posControl.gpsOrigin, &wp2, waypointMissionAltConvMode(posControl.waypointList[j].p3));
                        int32_t altConvModeAltitude = waypointMissionAltConvMode(posControl.waypointList[j].p3) == GEO_ALT_ABSOLUTE ? osdGetAltitudeMsl() : osdGetAltitude();
                        j = getGeoWaypointNumber(j);
                        while (j > 9) j -= 10; // Only the last digit displayed if WP>=10, no room for more (48 = ascii 0)
                        osdHudDrawPoi(calculateDistanceToDestination(&poi) / 100, osdGetHeadingAngle(calculateBearingToDestination(&poi) / 100), (posControl.waypointList[j].alt - altConvModeAltitude)/ 100, 2, SYM_WAYPOINT, 48 + j, i);
                    }
                }
            }
        }

        return true;
        break;

    case OSD_ATTITUDE_ROLL:
        buff[0] = SYM_ROLL_LEVEL;
        if (ABS(attitude.values.roll) >= 1)
            buff[0] += (attitude.values.roll < 0 ? -1 : 1);
        osdFormatCentiNumber(buff + 1, DECIDEGREES_TO_CENTIDEGREES(ABS(attitude.values.roll)), 0, 1, 0, 3);
        break;

    case OSD_ATTITUDE_PITCH:
        if (ABS(attitude.values.pitch) < 1)
            buff[0] = 'P';
        else if (attitude.values.pitch > 0)
            buff[0] = SYM_PITCH_DOWN;
        else if (attitude.values.pitch < 0)
            buff[0] = SYM_PITCH_UP;
        osdFormatCentiNumber(buff + 1, DECIDEGREES_TO_CENTIDEGREES(ABS(attitude.values.pitch)), 0, 1, 0, 3);
        break;

    case OSD_ARTIFICIAL_HORIZON:
        {
            float rollAngle;
            float pitchAngle;

#ifdef USE_SECONDARY_IMU
            if (secondaryImuState.active && secondaryImuConfig()->useForOsdAHI) {
                rollAngle = DECIDEGREES_TO_RADIANS(secondaryImuState.eulerAngles.values.roll);
                pitchAngle = DECIDEGREES_TO_RADIANS(secondaryImuState.eulerAngles.values.pitch);
            } else {
                rollAngle = DECIDEGREES_TO_RADIANS(attitude.values.roll);
                pitchAngle = DECIDEGREES_TO_RADIANS(attitude.values.pitch);
            }
#else
            rollAngle = DECIDEGREES_TO_RADIANS(attitude.values.roll);
            pitchAngle = DECIDEGREES_TO_RADIANS(attitude.values.pitch);
#endif
            pitchAngle -= osdConfig()->ahi_camera_uptilt_comp ? DEGREES_TO_RADIANS(osdConfig()->camera_uptilt) : 0;
            pitchAngle += DEGREES_TO_RADIANS(getFixedWingLevelTrim());
            if (osdConfig()->ahi_reverse_roll) {
                rollAngle = -rollAngle;
            }
            osdDrawArtificialHorizon(osdDisplayPort, osdGetDisplayPortCanvas(),
                 OSD_DRAW_POINT_GRID(elemPosX, elemPosY), rollAngle, pitchAngle);
            osdDrawSingleElement(OSD_HORIZON_SIDEBARS);
            osdDrawSingleElement(OSD_CROSSHAIRS);

            return true;
        }

    case OSD_HORIZON_SIDEBARS:
        {
            osdDrawSidebars(osdDisplayPort, osdGetDisplayPortCanvas());
            return true;
        }

#if defined(USE_BARO) || defined(USE_GPS)
    case OSD_VARIO:
        {
            float zvel = getEstimatedActualVelocity(Z);
            osdDrawVario(osdDisplayPort, osdGetDisplayPortCanvas(), OSD_DRAW_POINT_GRID(elemPosX, elemPosY), zvel);
            return true;
        }

    case OSD_VARIO_NUM:
        {
            int16_t value = getEstimatedActualVelocity(Z);
            char sym;
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    // Convert to centifeet/s
                    value = CENTIMETERS_TO_CENTIFEET(value);
                    sym = SYM_FTS;
                    break;
                default:
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    // Already in cm/s
                    sym = SYM_MS;
                    break;
            }

            osdFormatCentiNumber(buff, value, 0, 1, 0, 3);
            buff[3] = sym;
            buff[4] = '\0';
            break;
        }
#endif

    case OSD_ROLL_PIDS:
        osdDisplayFlightPIDValues(elemPosX, elemPosY, "ROL", PID_ROLL, ADJUSTMENT_ROLL_P, ADJUSTMENT_ROLL_I, ADJUSTMENT_ROLL_D, ADJUSTMENT_ROLL_FF);
        return true;

    case OSD_PITCH_PIDS:
        osdDisplayFlightPIDValues(elemPosX, elemPosY, "PIT", PID_PITCH, ADJUSTMENT_PITCH_P, ADJUSTMENT_PITCH_I, ADJUSTMENT_PITCH_D, ADJUSTMENT_PITCH_FF);
        return true;

    case OSD_YAW_PIDS:
        osdDisplayFlightPIDValues(elemPosX, elemPosY, "YAW", PID_YAW, ADJUSTMENT_YAW_P, ADJUSTMENT_YAW_I, ADJUSTMENT_YAW_D, ADJUSTMENT_YAW_FF);
        return true;

    case OSD_LEVEL_PIDS:
        osdDisplayNavPIDValues(elemPosX, elemPosY, "LEV", PID_LEVEL, ADJUSTMENT_LEVEL_P, ADJUSTMENT_LEVEL_I, ADJUSTMENT_LEVEL_D);
        return true;

    case OSD_POS_XY_PIDS:
        osdDisplayNavPIDValues(elemPosX, elemPosY, "PXY", PID_POS_XY, ADJUSTMENT_POS_XY_P, ADJUSTMENT_POS_XY_I, ADJUSTMENT_POS_XY_D);
        return true;

    case OSD_POS_Z_PIDS:
        osdDisplayNavPIDValues(elemPosX, elemPosY, "PZ", PID_POS_Z, ADJUSTMENT_POS_Z_P, ADJUSTMENT_POS_Z_I, ADJUSTMENT_POS_Z_D);
        return true;

    case OSD_VEL_XY_PIDS:
        osdDisplayNavPIDValues(elemPosX, elemPosY, "VXY", PID_VEL_XY, ADJUSTMENT_VEL_XY_P, ADJUSTMENT_VEL_XY_I, ADJUSTMENT_VEL_XY_D);
        return true;

    case OSD_VEL_Z_PIDS:
        osdDisplayNavPIDValues(elemPosX, elemPosY, "VZ", PID_VEL_Z, ADJUSTMENT_VEL_Z_P, ADJUSTMENT_VEL_Z_I, ADJUSTMENT_VEL_Z_D);
        return true;

    case OSD_HEADING_P:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "HP", 0, pidBank()->pid[PID_HEADING].P, 3, 0, ADJUSTMENT_HEADING_P);
        return true;

    case OSD_BOARD_ALIGN_ROLL:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "AR", 0, DECIDEGREES_TO_DEGREES((float)boardAlignment()->rollDeciDegrees), 4, 1, ADJUSTMENT_ROLL_BOARD_ALIGNMENT);
        return true;

    case OSD_BOARD_ALIGN_PITCH:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "AP", 0, DECIDEGREES_TO_DEGREES((float)boardAlignment()->pitchDeciDegrees), 4, 1, ADJUSTMENT_PITCH_BOARD_ALIGNMENT);
        return true;

    case OSD_RC_EXPO:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "EXP", 0, currentControlRateProfile->stabilized.rcExpo8, 3, 0, ADJUSTMENT_RC_EXPO);
        return true;

    case OSD_RC_YAW_EXPO:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "YEX", 0, currentControlRateProfile->stabilized.rcYawExpo8, 3, 0, ADJUSTMENT_RC_YAW_EXPO);
        return true;

    case OSD_THROTTLE_EXPO:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "TEX", 0, currentControlRateProfile->throttle.rcExpo8, 3, 0, ADJUSTMENT_THROTTLE_EXPO);
        return true;

    case OSD_PITCH_RATE:
        displayWrite(osdDisplayPort, elemPosX, elemPosY, "SPR");

        elemAttr = TEXT_ATTRIBUTES_NONE;
        tfp_sprintf(buff, "%3d", currentControlRateProfile->stabilized.rates[FD_PITCH]);
        if (isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_RATE) || isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_RATE))
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);
        return true;

    case OSD_ROLL_RATE:
        displayWrite(osdDisplayPort, elemPosX, elemPosY, "SRR");

        elemAttr = TEXT_ATTRIBUTES_NONE;
        tfp_sprintf(buff, "%3d", currentControlRateProfile->stabilized.rates[FD_ROLL]);
        if (isAdjustmentFunctionSelected(ADJUSTMENT_ROLL_RATE) || isAdjustmentFunctionSelected(ADJUSTMENT_PITCH_ROLL_RATE))
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);
        return true;

    case OSD_YAW_RATE:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "SYR", 0, currentControlRateProfile->stabilized.rates[FD_YAW], 3, 0, ADJUSTMENT_YAW_RATE);
        return true;

    case OSD_MANUAL_RC_EXPO:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "MEX", 0, currentControlRateProfile->manual.rcExpo8, 3, 0, ADJUSTMENT_MANUAL_RC_EXPO);
        return true;

    case OSD_MANUAL_RC_YAW_EXPO:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "MYX", 0, currentControlRateProfile->manual.rcYawExpo8, 3, 0, ADJUSTMENT_MANUAL_RC_YAW_EXPO);
        return true;

    case OSD_MANUAL_PITCH_RATE:
        displayWrite(osdDisplayPort, elemPosX, elemPosY, "MPR");

        elemAttr = TEXT_ATTRIBUTES_NONE;
        tfp_sprintf(buff, "%3d", currentControlRateProfile->manual.rates[FD_PITCH]);
        if (isAdjustmentFunctionSelected(ADJUSTMENT_MANUAL_PITCH_RATE) || isAdjustmentFunctionSelected(ADJUSTMENT_MANUAL_PITCH_ROLL_RATE))
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);
        return true;

    case OSD_MANUAL_ROLL_RATE:
        displayWrite(osdDisplayPort, elemPosX, elemPosY, "MRR");

        elemAttr = TEXT_ATTRIBUTES_NONE;
        tfp_sprintf(buff, "%3d", currentControlRateProfile->manual.rates[FD_ROLL]);
        if (isAdjustmentFunctionSelected(ADJUSTMENT_MANUAL_ROLL_RATE) || isAdjustmentFunctionSelected(ADJUSTMENT_MANUAL_PITCH_ROLL_RATE))
            TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
        displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY, buff, elemAttr);
        return true;

    case OSD_MANUAL_YAW_RATE:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "MYR", 0, currentControlRateProfile->stabilized.rates[FD_YAW], 3, 0, ADJUSTMENT_YAW_RATE);
        return true;

    case OSD_NAV_FW_CRUISE_THR:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "CRZ", 0, currentBatteryProfile->nav.fw.cruise_throttle, 4, 0, ADJUSTMENT_NAV_FW_CRUISE_THR);
        return true;

    case OSD_NAV_FW_PITCH2THR:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "P2T", 0, currentBatteryProfile->nav.fw.pitch_to_throttle, 3, 0, ADJUSTMENT_NAV_FW_PITCH2THR);
        return true;

    case OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "0TP", 0, (float)currentBatteryProfile->fwMinThrottleDownPitchAngle / 10, 3, 1, ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE);
        return true;

    case OSD_FW_ALT_PID_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers();
            osdFormatPidControllerOutput(buff, "PZO", &nav_pids->fw_alt, 10, true); // display requested pitch degrees
            break;
        }

    case OSD_FW_POS_PID_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers(); // display requested roll degrees
            osdFormatPidControllerOutput(buff, "PXYO", &nav_pids->fw_nav, 1, true);
            break;
        }

    case OSD_MC_VEL_Z_PID_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers();
            osdFormatPidControllerOutput(buff, "VZO", &nav_pids->vel[Z], 100, false); // display throttle adjustment µs
            break;
        }

    case OSD_MC_VEL_X_PID_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers();
            osdFormatPidControllerOutput(buff, "VXO", &nav_pids->vel[X], 100, false); // display requested acceleration cm/s^2
            break;
        }

    case OSD_MC_VEL_Y_PID_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers();
            osdFormatPidControllerOutput(buff, "VYO", &nav_pids->vel[Y], 100, false); // display requested acceleration cm/s^2
            break;
        }

    case OSD_MC_POS_XYZ_P_OUTPUTS:
        {
            const navigationPIDControllers_t *nav_pids = getNavigationPIDControllers();
            strcpy(buff, "POSO ");
            // display requested velocity cm/s
            tfp_sprintf(buff + 5, "%4d", (int)lrintf(nav_pids->pos[X].output_constrained * 100));
            buff[9] = ' ';
            tfp_sprintf(buff + 10, "%4d", (int)lrintf(nav_pids->pos[Y].output_constrained * 100));
            buff[14] = ' ';
            tfp_sprintf(buff + 15, "%4d", (int)lrintf(nav_pids->pos[Z].output_constrained * 100));
            buff[19] = '\0';
            break;
        }

    case OSD_POWER:
        {
            bool kiloWatt = osdFormatCentiNumber(buff, getPower(), 1000, 2, 2, 3);
            buff[3] = kiloWatt ? SYM_KILOWATT : SYM_WATT;
            buff[4] = '\0';

            uint8_t current_alarm = osdConfig()->current_alarm;
            if ((current_alarm > 0) && ((getAmperage() / 100.0f) > current_alarm)) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }

    case OSD_AIR_SPEED:
        {
        #ifdef USE_PITOT
            buff[0] = SYM_AIR;
            osdFormatVelocityStr(buff + 1, pitot.airSpeed, false);
        #else
            return false;
        #endif
            break;
        }

    case OSD_RTC_TIME:
        {
            // RTC not configured will show 00:00
            dateTime_t dateTime;
            rtcGetDateTimeLocal(&dateTime);
            buff[0] = SYM_CLOCK;
            tfp_sprintf(buff + 1, "%02u:%02u:%02u", dateTime.hours, dateTime.minutes, dateTime.seconds);
            break;
        }

    case OSD_MESSAGES:
        {
            elemAttr = osdGetSystemMessage(buff, OSD_MESSAGE_LENGTH, true);
            break;
        }

    case OSD_VERSION:
        {
            tfp_sprintf(buff, "INAV %s", FC_VERSION_STRING);
            displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);
            break;
        }

    case OSD_MAIN_BATT_CELL_VOLTAGE:
        {
            osdDisplayBatteryVoltage(elemPosX, elemPosY, getBatteryRawAverageCellVoltage(), 3, 2);
            return true;
        }

    case OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE:
        {
            osdDisplayBatteryVoltage(elemPosX, elemPosY, getBatterySagCompensatedAverageCellVoltage(), 3, 2);
            return true;
        }

    case OSD_THROTTLE_POS_AUTO_THR:
        {
            osdFormatThrottlePosition(buff, true, &elemAttr);
            break;
        }

    case OSD_HEADING_GRAPH:
        {
            if (osdIsHeadingValid()) {
                osdDrawHeadingGraph(osdDisplayPort, osdGetDisplayPortCanvas(), OSD_DRAW_POINT_GRID(elemPosX, elemPosY), osdGetHeading());
                return true;
            } else {
                buff[0] = buff[2] = buff[4] = buff[6] = buff[8] = SYM_HEADING_LINE;
                buff[1] = buff[3] = buff[5] = buff[7] = SYM_HEADING_DIVIDED_LINE;
                buff[OSD_HEADING_GRAPH_WIDTH] = '\0';
            }
            break;
        }

    case OSD_EFFICIENCY_MAH_PER_KM:
        {
            // amperage is in centi amps, speed is in cms/s. We want
            // mah/km. Only show when ground speed > 1m/s.
            static pt1Filter_t eFilterState;
            static timeUs_t efficiencyUpdated = 0;
            int32_t value = 0;
            bool moreThanAh = false;
            timeUs_t currentTimeUs = micros();
            timeDelta_t efficiencyTimeDelta = cmpTimeUs(currentTimeUs, efficiencyUpdated);
            if (STATE(GPS_FIX) && gpsSol.groundSpeed > 0) {
                if (efficiencyTimeDelta >= EFFICIENCY_UPDATE_INTERVAL) {
                    value = pt1FilterApply4(&eFilterState, ((float)getAmperage() / gpsSol.groundSpeed) / 0.0036f,
                        1, US2S(efficiencyTimeDelta));

                    efficiencyUpdated = currentTimeUs;
                } else {
                    value = eFilterState.state;
                }
            }
            bool efficiencyValid = (value > 0) && (gpsSol.groundSpeed > 100);
            switch (osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    moreThanAh = osdFormatCentiNumber(buff, value * METERS_PER_MILE / 10, 1000, 0, 2, 3);
                    if (!moreThanAh) {
                        tfp_sprintf(buff, "%s%c%c", buff, SYM_MAH_MI_0, SYM_MAH_MI_1);
                    } else {
                        tfp_sprintf(buff, "%s%c", buff, SYM_AH_MI);
                    }
                    if (!efficiencyValid) {
                        buff[0] = buff[1] = buff[2] = '-';
                        buff[3] = SYM_MAH_MI_0;
                        buff[4] = SYM_MAH_MI_1;
                        buff[5] = '\0';
                    }
                    break;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    moreThanAh = osdFormatCentiNumber(buff, value * 100, 1000, 0, 2, 3);
                    if (!moreThanAh) {
                        tfp_sprintf(buff, "%s%c%c", buff, SYM_MAH_KM_0, SYM_MAH_KM_1);
                    } else {
                        tfp_sprintf(buff, "%s%c", buff, SYM_AH_KM);
                    }
                    if (!efficiencyValid) {
                        buff[0] = buff[1] = buff[2] = '-';
                        buff[3] = SYM_MAH_KM_0;
                        buff[4] = SYM_MAH_KM_1;
                        buff[5] = '\0';
                    }
                    break;
            }
            break;
        }

    case OSD_EFFICIENCY_WH_PER_KM:
        {
            // amperage is in centi amps, speed is in cms/s. We want
            // mWh/km. Only show when ground speed > 1m/s.
            static pt1Filter_t eFilterState;
            static timeUs_t efficiencyUpdated = 0;
            int32_t value = 0;
            timeUs_t currentTimeUs = micros();
            timeDelta_t efficiencyTimeDelta = cmpTimeUs(currentTimeUs, efficiencyUpdated);
            if (STATE(GPS_FIX) && gpsSol.groundSpeed > 0) {
                if (efficiencyTimeDelta >= EFFICIENCY_UPDATE_INTERVAL) {
                    value = pt1FilterApply4(&eFilterState, ((float)getPower() / gpsSol.groundSpeed) / 0.0036f,
                        1, US2S(efficiencyTimeDelta));

                    efficiencyUpdated = currentTimeUs;
                } else {
                    value = eFilterState.state;
                }
            }
            bool efficiencyValid = (value > 0) && (gpsSol.groundSpeed > 100);
            switch (osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    osdFormatCentiNumber(buff, value * METERS_PER_MILE / 10000, 0, 2, 0, 3);
                    buff[3] = SYM_WH_MI;
                    break;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    osdFormatCentiNumber(buff, value / 10, 0, 2, 0, 3);
                    buff[3] = SYM_WH_KM;
                    break;
            }
            buff[4] = '\0';
            if (!efficiencyValid) {
                buff[0] = buff[1] = buff[2] = '-';
            }
            break;
        }

    case OSD_GFORCE:
        {
            buff[0] = SYM_GFORCE;
            osdFormatCentiNumber(buff + 1, GForce, 0, 2, 0, 3);
            if (GForce > osdConfig()->gforce_alarm * 100) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }

    case OSD_GFORCE_X:
    case OSD_GFORCE_Y:
    case OSD_GFORCE_Z:
        {
            float GForceValue = GForceAxis[item - OSD_GFORCE_X];
            buff[0] = SYM_GFORCE_X + item - OSD_GFORCE_X;
            osdFormatCentiNumber(buff + 1, GForceValue, 0, 2, 0, 4);
            if ((GForceValue < osdConfig()->gforce_axis_alarm_min * 100) || (GForceValue > osdConfig()->gforce_axis_alarm_max * 100)) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
            break;
        }
    case OSD_DEBUG:
        {
            /*
             * Longest representable string is -2147483648 does not fit in the screen.
             * Only 7 digits for negative and 8 digits for positive values allowed
             */
            for (uint8_t bufferIndex = 0; bufferIndex < DEBUG32_VALUE_COUNT; ++elemPosY, bufferIndex += 2) {
                tfp_sprintf(
                    buff,
                    "[%u]=%8ld [%u]=%8ld",
                    bufferIndex,
                    constrain(debug[bufferIndex], -9999999, 99999999),
                    bufferIndex+1,
                    constrain(debug[bufferIndex+1], -9999999, 99999999)
                );
                displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);
            }
            break;
        }

    case OSD_IMU_TEMPERATURE:
        {
            int16_t temperature;
            const bool valid = getIMUTemperature(&temperature);
            osdDisplayTemperature(elemPosX, elemPosY, SYM_IMU_TEMP, NULL, valid, temperature, osdConfig()->imu_temp_alarm_min, osdConfig()->imu_temp_alarm_max);
            return true;
        }

    case OSD_BARO_TEMPERATURE:
        {
            int16_t temperature;
            const bool valid = getBaroTemperature(&temperature);
            osdDisplayTemperature(elemPosX, elemPosY, SYM_BARO_TEMP, NULL, valid, temperature, osdConfig()->imu_temp_alarm_min, osdConfig()->imu_temp_alarm_max);
            return true;
        }

#ifdef USE_TEMPERATURE_SENSOR
    case OSD_TEMP_SENSOR_0_TEMPERATURE:
    case OSD_TEMP_SENSOR_1_TEMPERATURE:
    case OSD_TEMP_SENSOR_2_TEMPERATURE:
    case OSD_TEMP_SENSOR_3_TEMPERATURE:
    case OSD_TEMP_SENSOR_4_TEMPERATURE:
    case OSD_TEMP_SENSOR_5_TEMPERATURE:
    case OSD_TEMP_SENSOR_6_TEMPERATURE:
    case OSD_TEMP_SENSOR_7_TEMPERATURE:
        {
            osdDisplayTemperatureSensor(elemPosX, elemPosY, item - OSD_TEMP_SENSOR_0_TEMPERATURE);
            return true;
        }
#endif /* ifdef USE_TEMPERATURE_SENSOR */

    case OSD_WIND_SPEED_HORIZONTAL:
#ifdef USE_WIND_ESTIMATOR
        {
            bool valid = isEstimatedWindSpeedValid();
            float horizontalWindSpeed;
            if (valid) {
                uint16_t angle;
                horizontalWindSpeed = getEstimatedHorizontalWindSpeed(&angle);
                int16_t windDirection = osdGetHeadingAngle((int)angle - DECIDEGREES_TO_DEGREES(attitude.values.yaw));
                buff[1] = SYM_DIRECTION + (windDirection * 2 / 90);
            } else {
                horizontalWindSpeed = 0;
                buff[1] = SYM_BLANK;
            }
            buff[0] = SYM_WIND_HORIZONTAL;
            osdFormatWindSpeedStr(buff + 2, horizontalWindSpeed, valid);
            break;
        }
#else
        return false;
#endif

    case OSD_WIND_SPEED_VERTICAL:
#ifdef USE_WIND_ESTIMATOR
        {
            buff[0] = SYM_WIND_VERTICAL;
            buff[1] = SYM_BLANK;
            bool valid = isEstimatedWindSpeedValid();
            float verticalWindSpeed;
            if (valid) {
                verticalWindSpeed = getEstimatedWindSpeed(Z);
                if (verticalWindSpeed < 0) {
                    buff[1] = SYM_AH_DECORATION_DOWN;
                    verticalWindSpeed = -verticalWindSpeed;
                } else if (verticalWindSpeed > 0) {
                    buff[1] = SYM_AH_DECORATION_UP;
                }
            } else {
                verticalWindSpeed = 0;
            }
            osdFormatWindSpeedStr(buff + 2, verticalWindSpeed, valid);
            break;
        }
#else
        return false;
#endif

    case OSD_PLUS_CODE:
        {
            STATIC_ASSERT(GPS_DEGREES_DIVIDER == OLC_DEG_MULTIPLIER, invalid_olc_deg_multiplier);
            int digits = osdConfig()->plus_code_digits;
            int digitsRemoved = osdConfig()->plus_code_short * 2;
            if (STATE(GPS_FIX)) {
                olc_encode(gpsSol.llh.lat, gpsSol.llh.lon, digits, buff, sizeof(buff));
            } else {
                // +codes with > 8 digits have a + at the 9th digit
                // and we only support 10 and up.
                memset(buff, '-', digits + 1);
                buff[8] = '+';
                buff[digits + 1] = '\0';
            }
            // Optionally trim digits from the left
            memmove(buff, buff+digitsRemoved, strlen(buff) + digitsRemoved);
            buff[digits + 1 - digitsRemoved] = '\0';
            break;
        }

    case OSD_AZIMUTH:
        {

            buff[0] = SYM_AZIMUTH;
            if (osdIsHeadingValid()) {
                int16_t h = GPS_directionToHome;
                if (h < 0) {
                    h += 360;
                }
                if(h >= 180)
                    h = h - 180;
                else
                    h = h + 180;

                tfp_sprintf(&buff[1], "%3d", h);
            } else {
                buff[1] = buff[2] = buff[3] = '-';
            }
            buff[4] = '\0';
            break;
        }

    case OSD_MAP_SCALE:
        {
            float scaleToUnit;
            int scaleUnitDivisor;
            char symUnscaled;
            char symScaled;
            int maxDecimals;

            switch (osdConfig()->units) {
            case OSD_UNIT_UK:
                FALLTHROUGH;
            case OSD_UNIT_IMPERIAL:
                scaleToUnit = 100 / 1609.3440f; // scale to 0.01mi for osdFormatCentiNumber()
                scaleUnitDivisor = 0;
                symUnscaled = SYM_MI;
                symScaled = SYM_MI;
                maxDecimals = 2;
                break;
            default:
            case OSD_UNIT_METRIC_MPH:
                FALLTHROUGH;
            case OSD_UNIT_METRIC:
                scaleToUnit = 100; // scale to cm for osdFormatCentiNumber()
                scaleUnitDivisor = 1000; // Convert to km when scale gets bigger than 999m
                symUnscaled = SYM_M;
                symScaled = SYM_KM;
                maxDecimals = 0;
                break;
            }
            buff[0] = SYM_SCALE;
            if (osdMapData.scale > 0) {
                bool scaled = osdFormatCentiNumber(&buff[1], osdMapData.scale * scaleToUnit, scaleUnitDivisor, maxDecimals, 2, 3);
                buff[4] = scaled ? symScaled : symUnscaled;
                // Make sure this is cleared if the map stops being drawn
                osdMapData.scale = 0;
            } else {
                memset(&buff[1], '-', 4);
            }
            buff[5] = '\0';
            break;
        }
    case OSD_MAP_REFERENCE:
        {
            char referenceSymbol;
            if (osdMapData.referenceSymbol) {
                referenceSymbol = osdMapData.referenceSymbol;
                // Make sure this is cleared if the map stops being drawn
                osdMapData.referenceSymbol = 0;
            } else {
                referenceSymbol = '-';
            }
            displayWriteChar(osdDisplayPort, elemPosX, elemPosY, SYM_DIRECTION);
            displayWriteChar(osdDisplayPort, elemPosX, elemPosY + 1, referenceSymbol);
            return true;
        }

    case OSD_GVAR_0:
    {
        osdFormatGVar(buff, 0);
        break;
    }
    case OSD_GVAR_1:
    {
        osdFormatGVar(buff, 1);
        break;
    }
    case OSD_GVAR_2:
    {
        osdFormatGVar(buff, 2);
        break;
    }
    case OSD_GVAR_3:
    {
        osdFormatGVar(buff, 3);
        break;
    }

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    case OSD_RC_SOURCE:
        {
            const char *source_text = IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe() ? "MSP" : "STD";
            if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && mspOverrideIsInFailsafe()) TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, source_text, elemAttr);
            return true;
        }
#endif

#if defined(USE_ESC_SENSOR)
    case OSD_ESC_RPM:
        {
            escSensorData_t * escSensor = escSensorGetData();
            if (escSensor && escSensor->dataAge <= ESC_DATA_MAX_AGE) {
                osdFormatRpm(buff, escSensor->rpm);
            }
            else {
                osdFormatRpm(buff, 0);
            }
            break;
        }
    case OSD_ESC_TEMPERATURE:
        {
            escSensorData_t * escSensor = escSensorGetData();
            bool escTemperatureValid = escSensor && escSensor->dataAge <= ESC_DATA_MAX_AGE;
            osdDisplayTemperature(elemPosX, elemPosY, SYM_ESC_TEMP, NULL, escTemperatureValid, (escSensor->temperature)*10, osdConfig()->esc_temp_alarm_min, osdConfig()->esc_temp_alarm_max);
            return true;
        }
#endif
    case OSD_TPA:
        {
            char buff[4];
            textAttributes_t attr;

            displayWrite(osdDisplayPort, elemPosX, elemPosY, "TPA");
            attr = TEXT_ATTRIBUTES_NONE;
            tfp_sprintf(buff, "%3d", currentControlRateProfile->throttle.dynPID);
            if (isAdjustmentFunctionSelected(ADJUSTMENT_TPA)) {
                TEXT_ATTRIBUTES_ADD_BLINK(attr);
            }
            displayWriteWithAttr(osdDisplayPort, elemPosX + 5, elemPosY, buff, attr);

            displayWrite(osdDisplayPort, elemPosX, elemPosY + 1, "BP");
            attr = TEXT_ATTRIBUTES_NONE;
            tfp_sprintf(buff, "%4d", currentControlRateProfile->throttle.pa_breakpoint);
            if (isAdjustmentFunctionSelected(ADJUSTMENT_TPA_BREAKPOINT)) {
                TEXT_ATTRIBUTES_ADD_BLINK(attr);
            }
            displayWriteWithAttr(osdDisplayPort, elemPosX + 4, elemPosY + 1, buff, attr);

            return true;
        }

    case OSD_NAV_FW_CONTROL_SMOOTHNESS:
        osdDisplayAdjustableDecimalValue(elemPosX, elemPosY, "CTL S", 0, navConfig()->fw.control_smoothness, 1, 0, ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS);
        return true;

#ifdef USE_POWER_LIMITS
    case OSD_PLIMIT_REMAINING_BURST_TIME:
        osdFormatCentiNumber(buff, powerLimiterGetRemainingBurstTime() * 100, 0, 1, 0, 3);
        buff[3] = 'S';
        buff[4] = '\0';
        break;

    case OSD_PLIMIT_ACTIVE_CURRENT_LIMIT:
        if (currentBatteryProfile->powerLimits.continuousCurrent) {
            osdFormatCentiNumber(buff, powerLimiterGetActiveCurrentLimit(), 0, 2, 0, 3);
            buff[3] = SYM_AMP;
            buff[4] = '\0';

            if (powerLimiterIsLimitingCurrent()) {
                TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
            }
        }
        break;

#ifdef USE_ADC
    case OSD_PLIMIT_ACTIVE_POWER_LIMIT:
        {
            if (currentBatteryProfile->powerLimits.continuousPower) {
                bool kiloWatt = osdFormatCentiNumber(buff, powerLimiterGetActivePowerLimit(), 1000, 2, 2, 3);
                buff[3] = kiloWatt ? SYM_KILOWATT : SYM_WATT;
                buff[4] = '\0';

                if (powerLimiterIsLimitingPower()) {
                    TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
                }
            }
            break;
        }
#endif // USE_ADC
#endif // USE_POWER_LIMITS

    default:
        return false;
    }

    displayWriteWithAttr(osdDisplayPort, elemPosX, elemPosY, buff, elemAttr);
    return true;
}

static uint8_t osdIncElementIndex(uint8_t elementIndex)
{
    ++elementIndex;

    if (elementIndex == OSD_ARTIFICIAL_HORIZON)
        ++elementIndex;

#ifndef USE_TEMPERATURE_SENSOR
    if (elementIndex == OSD_TEMP_SENSOR_0_TEMPERATURE)
        elementIndex = OSD_ALTITUDE_MSL;
#endif

    if (!sensors(SENSOR_ACC)) {
        if (elementIndex == OSD_CROSSHAIRS) {
            elementIndex = OSD_ONTIME;
        }
    }

    if (!feature(FEATURE_VBAT)) {
        if (elementIndex == OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE) {
            elementIndex = OSD_LEVEL_PIDS;
        }
    }

    if (!feature(FEATURE_CURRENT_METER)) {
        if (elementIndex == OSD_CURRENT_DRAW) {
            elementIndex = OSD_GPS_SPEED;
        }
        if (elementIndex == OSD_EFFICIENCY_MAH_PER_KM) {
            elementIndex = OSD_TRIP_DIST;
        }
        if (elementIndex == OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH) {
            elementIndex = OSD_HOME_HEADING_ERROR;
        }
        if (elementIndex == OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE) {
            elementIndex = OSD_LEVEL_PIDS;
        }
    }

    if (!feature(FEATURE_GPS)) {
        if (elementIndex == OSD_GPS_SPEED) {
            elementIndex = OSD_ALTITUDE;
        }
        if (elementIndex == OSD_GPS_LON) {
            elementIndex = OSD_VARIO;
        }
        if (elementIndex == OSD_GPS_HDOP) {
            elementIndex = OSD_MAIN_BATT_CELL_VOLTAGE;
        }
        if (elementIndex == OSD_TRIP_DIST) {
            elementIndex = OSD_ATTITUDE_PITCH;
        }
        if (elementIndex == OSD_WIND_SPEED_HORIZONTAL) {
            elementIndex = OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE;
        }
        if (elementIndex == OSD_3D_SPEED) {
            elementIndex++;
        }
    }

    if (!STATE(ESC_SENSOR_ENABLED)) {
        if (elementIndex == OSD_ESC_RPM) {
            elementIndex++;
        }
    }

#ifndef USE_POWER_LIMITS
    if (elementIndex == OSD_NAV_FW_CONTROL_SMOOTHNESS) {
        elementIndex = OSD_ITEM_COUNT;
    }
#endif

    if (elementIndex == OSD_ITEM_COUNT) {
        elementIndex = 0;
    }
    return elementIndex;
}

void osdDrawNextElement(void)
{
    static uint8_t elementIndex = 0;
    // Prevent infinite loop when no elements are enabled
    uint8_t index = elementIndex;
    do {
        elementIndex = osdIncElementIndex(elementIndex);
    } while(!osdDrawSingleElement(elementIndex) && index != elementIndex);

    // Draw artificial horizon + tracking telemtry last
    osdDrawSingleElement(OSD_ARTIFICIAL_HORIZON);
    if (osdConfig()->telemetry>0){
      osdDisplayTelemetry();
    }
}

PG_RESET_TEMPLATE(osdConfig_t, osdConfig,
    .rssi_alarm = SETTING_OSD_RSSI_ALARM_DEFAULT,
    .time_alarm = SETTING_OSD_TIME_ALARM_DEFAULT,
    .alt_alarm = SETTING_OSD_ALT_ALARM_DEFAULT,
    .dist_alarm = SETTING_OSD_DIST_ALARM_DEFAULT,
    .neg_alt_alarm = SETTING_OSD_NEG_ALT_ALARM_DEFAULT,
    .current_alarm = SETTING_OSD_CURRENT_ALARM_DEFAULT,
    .imu_temp_alarm_min = SETTING_OSD_IMU_TEMP_ALARM_MIN_DEFAULT,
    .imu_temp_alarm_max = SETTING_OSD_IMU_TEMP_ALARM_MAX_DEFAULT,
    .esc_temp_alarm_min = SETTING_OSD_ESC_TEMP_ALARM_MIN_DEFAULT,
    .esc_temp_alarm_max = SETTING_OSD_ESC_TEMP_ALARM_MAX_DEFAULT,
    .gforce_alarm = SETTING_OSD_GFORCE_ALARM_DEFAULT,
    .gforce_axis_alarm_min = SETTING_OSD_GFORCE_AXIS_ALARM_MIN_DEFAULT,
    .gforce_axis_alarm_max = SETTING_OSD_GFORCE_AXIS_ALARM_MAX_DEFAULT,
#ifdef USE_BARO
    .baro_temp_alarm_min = SETTING_OSD_BARO_TEMP_ALARM_MIN_DEFAULT,
    .baro_temp_alarm_max = SETTING_OSD_BARO_TEMP_ALARM_MAX_DEFAULT,
#endif
#ifdef USE_SERIALRX_CRSF
    .snr_alarm = SETTING_OSD_SNR_ALARM_DEFAULT,
    .crsf_lq_format = SETTING_OSD_CRSF_LQ_FORMAT_DEFAULT,
    .link_quality_alarm = SETTING_OSD_LINK_QUALITY_ALARM_DEFAULT,
    .rssi_dbm_alarm = SETTING_OSD_RSSI_DBM_ALARM_DEFAULT,
#endif
#ifdef USE_TEMPERATURE_SENSOR
    .temp_label_align = SETTING_OSD_TEMP_LABEL_ALIGN_DEFAULT,
#endif

    .video_system = SETTING_OSD_VIDEO_SYSTEM_DEFAULT,
    .row_shiftdown = SETTING_OSD_ROW_SHIFTDOWN_DEFAULT,

    .ahi_reverse_roll = SETTING_OSD_AHI_REVERSE_ROLL_DEFAULT,
    .ahi_max_pitch = SETTING_OSD_AHI_MAX_PITCH_DEFAULT,
    .crosshairs_style = SETTING_OSD_CROSSHAIRS_STYLE_DEFAULT,
    .horizon_offset = SETTING_OSD_HORIZON_OFFSET_DEFAULT,
    .camera_uptilt = SETTING_OSD_CAMERA_UPTILT_DEFAULT,
    .ahi_camera_uptilt_comp = SETTING_OSD_AHI_CAMERA_UPTILT_COMP_DEFAULT,
    .camera_fov_h = SETTING_OSD_CAMERA_FOV_H_DEFAULT,
    .camera_fov_v = SETTING_OSD_CAMERA_FOV_V_DEFAULT,
    .hud_margin_h = SETTING_OSD_HUD_MARGIN_H_DEFAULT,
    .hud_margin_v = SETTING_OSD_HUD_MARGIN_V_DEFAULT,
    .hud_homing = SETTING_OSD_HUD_HOMING_DEFAULT,
    .hud_homepoint = SETTING_OSD_HUD_HOMEPOINT_DEFAULT,
    .hud_radar_disp = SETTING_OSD_HUD_RADAR_DISP_DEFAULT,
    .hud_radar_range_min = SETTING_OSD_HUD_RADAR_RANGE_MIN_DEFAULT,
    .hud_radar_range_max = SETTING_OSD_HUD_RADAR_RANGE_MAX_DEFAULT,
    .hud_radar_nearest = SETTING_OSD_HUD_RADAR_NEAREST_DEFAULT,
    .hud_wp_disp = SETTING_OSD_HUD_WP_DISP_DEFAULT,
    .left_sidebar_scroll = SETTING_OSD_LEFT_SIDEBAR_SCROLL_DEFAULT,
    .right_sidebar_scroll = SETTING_OSD_RIGHT_SIDEBAR_SCROLL_DEFAULT,
    .sidebar_scroll_arrows = SETTING_OSD_SIDEBAR_SCROLL_ARROWS_DEFAULT,
    .sidebar_horizontal_offset = SETTING_OSD_SIDEBAR_HORIZONTAL_OFFSET_DEFAULT,
    .left_sidebar_scroll_step = SETTING_OSD_LEFT_SIDEBAR_SCROLL_STEP_DEFAULT,
    .right_sidebar_scroll_step = SETTING_OSD_RIGHT_SIDEBAR_SCROLL_STEP_DEFAULT,
    .sidebar_height = SETTING_OSD_SIDEBAR_HEIGHT_DEFAULT,
    .osd_home_position_arm_screen = SETTING_OSD_HOME_POSITION_ARM_SCREEN_DEFAULT,
    .pan_servo_index = SETTING_OSD_PAN_SERVO_INDEX_DEFAULT,
    .pan_servo_pwm2centideg = SETTING_OSD_PAN_SERVO_PWM2CENTIDEG_DEFAULT,

    .units = SETTING_OSD_UNITS_DEFAULT,
    .main_voltage_decimals = SETTING_OSD_MAIN_VOLTAGE_DECIMALS_DEFAULT,

#ifdef USE_WIND_ESTIMATOR
    .estimations_wind_compensation = SETTING_OSD_ESTIMATIONS_WIND_COMPENSATION_DEFAULT,
#endif

    .coordinate_digits = SETTING_OSD_COORDINATE_DIGITS_DEFAULT,

    .osd_failsafe_switch_layout = SETTING_OSD_FAILSAFE_SWITCH_LAYOUT_DEFAULT,

    .plus_code_digits = SETTING_OSD_PLUS_CODE_DIGITS_DEFAULT,
    .plus_code_short = SETTING_OSD_PLUS_CODE_SHORT_DEFAULT,

    .ahi_width = SETTING_OSD_AHI_WIDTH_DEFAULT,
    .ahi_height = SETTING_OSD_AHI_HEIGHT_DEFAULT,
    .ahi_vertical_offset = SETTING_OSD_AHI_VERTICAL_OFFSET_DEFAULT,
    .ahi_bordered = SETTING_OSD_AHI_BORDERED_DEFAULT,
    .ahi_style = SETTING_OSD_AHI_STYLE_DEFAULT,

    .force_grid = SETTING_OSD_FORCE_GRID_DEFAULT,

    .stats_energy_unit = SETTING_OSD_STATS_ENERGY_UNIT_DEFAULT,
    .stats_min_voltage_unit = SETTING_OSD_STATS_MIN_VOLTAGE_UNIT_DEFAULT
);

void pgResetFn_osdLayoutsConfig(osdLayoutsConfig_t *osdLayoutsConfig)
{
    osdLayoutsConfig->item_pos[0][OSD_ALTITUDE] = OSD_POS(1, 0) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12, 0) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE] = OSD_POS(12, 1);

    osdLayoutsConfig->item_pos[0][OSD_RSSI_VALUE] = OSD_POS(23, 0) | OSD_VISIBLE_FLAG;
    //line 2
    osdLayoutsConfig->item_pos[0][OSD_HOME_DIST] = OSD_POS(1, 1);
    osdLayoutsConfig->item_pos[0][OSD_TRIP_DIST] = OSD_POS(1, 2);
    osdLayoutsConfig->item_pos[0][OSD_MAIN_BATT_CELL_VOLTAGE] = OSD_POS(12, 1);
    osdLayoutsConfig->item_pos[0][OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE] = OSD_POS(12, 1);
    osdLayoutsConfig->item_pos[0][OSD_GPS_SPEED] = OSD_POS(23, 1);
    osdLayoutsConfig->item_pos[0][OSD_3D_SPEED] = OSD_POS(23, 1);
    osdLayoutsConfig->item_pos[0][OSD_GLIDESLOPE] = OSD_POS(23, 2);

    osdLayoutsConfig->item_pos[0][OSD_THROTTLE_POS] = OSD_POS(1, 2) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_THROTTLE_POS_AUTO_THR] = OSD_POS(6, 2);
    osdLayoutsConfig->item_pos[0][OSD_HEADING] = OSD_POS(12, 2);
    osdLayoutsConfig->item_pos[0][OSD_COURSE_HOLD_ERROR] = OSD_POS(12, 2);
    osdLayoutsConfig->item_pos[0][OSD_COURSE_HOLD_ADJUSTMENT] = OSD_POS(12, 2);
    osdLayoutsConfig->item_pos[0][OSD_HEADING_GRAPH] = OSD_POS(18, 2);
    osdLayoutsConfig->item_pos[0][OSD_CURRENT_DRAW] = OSD_POS(2, 3) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_MAH_DRAWN] = OSD_POS(1, 4) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_WH_DRAWN] = OSD_POS(1, 5);
    osdLayoutsConfig->item_pos[0][OSD_BATTERY_REMAINING_CAPACITY] = OSD_POS(1, 6);
    osdLayoutsConfig->item_pos[0][OSD_BATTERY_REMAINING_PERCENT] = OSD_POS(1, 7);
    osdLayoutsConfig->item_pos[0][OSD_POWER_SUPPLY_IMPEDANCE] = OSD_POS(1, 8);

    osdLayoutsConfig->item_pos[0][OSD_EFFICIENCY_MAH_PER_KM] = OSD_POS(1, 5);
    osdLayoutsConfig->item_pos[0][OSD_EFFICIENCY_WH_PER_KM] = OSD_POS(1, 5);

    osdLayoutsConfig->item_pos[0][OSD_ATTITUDE_ROLL] = OSD_POS(1, 7);
    osdLayoutsConfig->item_pos[0][OSD_ATTITUDE_PITCH] = OSD_POS(1, 8);

    // avoid OSD_VARIO under OSD_CROSSHAIRS
    osdLayoutsConfig->item_pos[0][OSD_VARIO] = OSD_POS(23, 5);
    // OSD_VARIO_NUM at the right of OSD_VARIO
    osdLayoutsConfig->item_pos[0][OSD_VARIO_NUM] = OSD_POS(24, 7);
    osdLayoutsConfig->item_pos[0][OSD_HOME_DIR] = OSD_POS(14, 11);
    osdLayoutsConfig->item_pos[0][OSD_ARTIFICIAL_HORIZON] = OSD_POS(8, 6);
    osdLayoutsConfig->item_pos[0][OSD_HORIZON_SIDEBARS] = OSD_POS(8, 6);

    osdLayoutsConfig->item_pos[0][OSD_CRAFT_NAME] = OSD_POS(20, 2);
    osdLayoutsConfig->item_pos[0][OSD_VTX_CHANNEL] = OSD_POS(8, 6);

#ifdef USE_SERIALRX_CRSF
    osdLayoutsConfig->item_pos[0][OSD_CRSF_RSSI_DBM] = OSD_POS(23, 12);
    osdLayoutsConfig->item_pos[0][OSD_CRSF_LQ] = OSD_POS(23, 11);
    osdLayoutsConfig->item_pos[0][OSD_CRSF_SNR_DB] = OSD_POS(24, 9);
    osdLayoutsConfig->item_pos[0][OSD_CRSF_TX_POWER] = OSD_POS(24, 10);
#endif

    osdLayoutsConfig->item_pos[0][OSD_ONTIME] = OSD_POS(23, 8);
    osdLayoutsConfig->item_pos[0][OSD_FLYTIME] = OSD_POS(23, 9);
    osdLayoutsConfig->item_pos[0][OSD_ONTIME_FLYTIME] = OSD_POS(23, 11) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_RTC_TIME] = OSD_POS(23, 12);
    osdLayoutsConfig->item_pos[0][OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH] = OSD_POS(23, 7);
    osdLayoutsConfig->item_pos[0][OSD_REMAINING_DISTANCE_BEFORE_RTH] = OSD_POS(23, 6);

    osdLayoutsConfig->item_pos[0][OSD_GPS_SATS] = OSD_POS(0, 11) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_GPS_HDOP] = OSD_POS(0, 10);

    osdLayoutsConfig->item_pos[0][OSD_GPS_LAT] = OSD_POS(0, 12);
    // Put this on top of the latitude, since it's very unlikely
    // that users will want to use both at the same time.
    osdLayoutsConfig->item_pos[0][OSD_PLUS_CODE] = OSD_POS(0, 12);
    osdLayoutsConfig->item_pos[0][OSD_FLYMODE] = OSD_POS(13, 12) | OSD_VISIBLE_FLAG;
    osdLayoutsConfig->item_pos[0][OSD_GPS_LON] = OSD_POS(18, 12);

    osdLayoutsConfig->item_pos[0][OSD_AZIMUTH] = OSD_POS(2, 12);

    osdLayoutsConfig->item_pos[0][OSD_ROLL_PIDS] = OSD_POS(2, 10);
    osdLayoutsConfig->item_pos[0][OSD_PITCH_PIDS] = OSD_POS(2, 11);
    osdLayoutsConfig->item_pos[0][OSD_YAW_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_LEVEL_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_POS_XY_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_POS_Z_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_VEL_XY_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_VEL_Z_PIDS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_HEADING_P] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_BOARD_ALIGN_ROLL] = OSD_POS(2, 10);
    osdLayoutsConfig->item_pos[0][OSD_BOARD_ALIGN_PITCH] = OSD_POS(2, 11);
    osdLayoutsConfig->item_pos[0][OSD_RC_EXPO] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_RC_YAW_EXPO] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_THROTTLE_EXPO] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_PITCH_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_ROLL_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_YAW_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MANUAL_RC_EXPO] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MANUAL_RC_YAW_EXPO] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MANUAL_PITCH_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MANUAL_ROLL_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MANUAL_YAW_RATE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_NAV_FW_CRUISE_THR] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_NAV_FW_PITCH2THR] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_FW_ALT_PID_OUTPUTS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_FW_POS_PID_OUTPUTS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MC_VEL_X_PID_OUTPUTS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MC_VEL_Y_PID_OUTPUTS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MC_VEL_Z_PID_OUTPUTS] = OSD_POS(2, 12);
    osdLayoutsConfig->item_pos[0][OSD_MC_POS_XYZ_P_OUTPUTS] = OSD_POS(2, 12);

    osdLayoutsConfig->item_pos[0][OSD_POWER] = OSD_POS(15, 1);

    osdLayoutsConfig->item_pos[0][OSD_IMU_TEMPERATURE] = OSD_POS(19, 2);
    osdLayoutsConfig->item_pos[0][OSD_BARO_TEMPERATURE] = OSD_POS(19, 3);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_0_TEMPERATURE] = OSD_POS(19, 4);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_1_TEMPERATURE] = OSD_POS(19, 5);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_2_TEMPERATURE] = OSD_POS(19, 6);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_3_TEMPERATURE] = OSD_POS(19, 7);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_4_TEMPERATURE] = OSD_POS(19, 8);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_5_TEMPERATURE] = OSD_POS(19, 9);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_6_TEMPERATURE] = OSD_POS(19, 10);
    osdLayoutsConfig->item_pos[0][OSD_TEMP_SENSOR_7_TEMPERATURE] = OSD_POS(19, 11);

    osdLayoutsConfig->item_pos[0][OSD_AIR_SPEED] = OSD_POS(3, 5);
    osdLayoutsConfig->item_pos[0][OSD_WIND_SPEED_HORIZONTAL] = OSD_POS(3, 6);
    osdLayoutsConfig->item_pos[0][OSD_WIND_SPEED_VERTICAL] = OSD_POS(3, 7);

    osdLayoutsConfig->item_pos[0][OSD_GFORCE] = OSD_POS(12, 4);
    osdLayoutsConfig->item_pos[0][OSD_GFORCE_X] = OSD_POS(12, 5);
    osdLayoutsConfig->item_pos[0][OSD_GFORCE_Y] = OSD_POS(12, 6);
    osdLayoutsConfig->item_pos[0][OSD_GFORCE_Z] = OSD_POS(12, 7);

    osdLayoutsConfig->item_pos[0][OSD_VTX_POWER] = OSD_POS(3, 5);

    osdLayoutsConfig->item_pos[0][OSD_GVAR_0] = OSD_POS(1, 1);
    osdLayoutsConfig->item_pos[0][OSD_GVAR_1] = OSD_POS(1, 2);
    osdLayoutsConfig->item_pos[0][OSD_GVAR_2] = OSD_POS(1, 3);
    osdLayoutsConfig->item_pos[0][OSD_GVAR_3] = OSD_POS(1, 4);

#if defined(USE_ESC_SENSOR)
    osdLayoutsConfig->item_pos[0][OSD_ESC_RPM] = OSD_POS(1, 2);
    osdLayoutsConfig->item_pos[0][OSD_ESC_TEMPERATURE] = OSD_POS(1, 3);
#endif

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    osdLayoutsConfig->item_pos[0][OSD_RC_SOURCE] = OSD_POS(3, 4);
#endif

#ifdef USE_POWER_LIMITS
    osdLayoutsConfig->item_pos[0][OSD_PLIMIT_REMAINING_BURST_TIME] = OSD_POS(3, 4);
    osdLayoutsConfig->item_pos[0][OSD_PLIMIT_ACTIVE_CURRENT_LIMIT] = OSD_POS(3, 5);
    osdLayoutsConfig->item_pos[0][OSD_PLIMIT_ACTIVE_POWER_LIMIT] = OSD_POS(3, 6);
#endif

    // Under OSD_FLYMODE. TODO: Might not be visible on NTSC?
    osdLayoutsConfig->item_pos[0][OSD_MESSAGES] = OSD_POS(1, 13) | OSD_VISIBLE_FLAG;

    for (unsigned ii = 1; ii < OSD_LAYOUT_COUNT; ii++) {
        for (unsigned jj = 0; jj < ARRAYLEN(osdLayoutsConfig->item_pos[0]); jj++) {
            osdLayoutsConfig->item_pos[ii][jj] = osdLayoutsConfig->item_pos[0][jj] & ~OSD_VISIBLE_FLAG;
        }
    }
}

static void osdSetNextRefreshIn(uint32_t timeMs) {
    resumeRefreshAt = micros() + timeMs * 1000;
    refreshWaitForResumeCmdRelease = true;
}

static void osdCompleteAsyncInitialization(void)
{
    if (!displayIsReady(osdDisplayPort)) {
        // Update the display.
        // XXX: Rename displayDrawScreen() and associated functions
        // to displayUpdate()
        displayDrawScreen(osdDisplayPort);
        return;
    }

    osdDisplayIsReady = true;

#if defined(USE_CANVAS)
    if (osdConfig()->force_grid) {
        osdDisplayHasCanvas = false;
    } else {
        osdDisplayHasCanvas = displayGetCanvas(&osdCanvas, osdDisplayPort);
    }
#endif

    displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
    displayClearScreen(osdDisplayPort);

    uint8_t y = 1;
    displayFontMetadata_t metadata;
    bool fontHasMetadata = displayGetFontMetadata(&metadata, osdDisplayPort);
    LOG_D(OSD, "Font metadata version %s: %u (%u chars)",
        fontHasMetadata ? "Y" : "N", metadata.version, metadata.charCount);

    if (fontHasMetadata && metadata.charCount > 256) {
        hasExtendedFont = true;
        unsigned logo_c = SYM_LOGO_START;
        unsigned logo_x = OSD_CENTER_LEN(SYM_LOGO_WIDTH);
        for (unsigned ii = 0; ii < SYM_LOGO_HEIGHT; ii++) {
            for (unsigned jj = 0; jj < SYM_LOGO_WIDTH; jj++) {
                displayWriteChar(osdDisplayPort, logo_x + jj, y, logo_c++);
            }
            y++;
        }
        y++;
    } else {
        if (!fontHasMetadata || metadata.version < OSD_MIN_FONT_VERSION) {
            const char *m = "INVALID FONT";
            displayWrite(osdDisplayPort, OSD_CENTER_S(m), 3, m);
        }
        y = 4;
    }

    char string_buffer[30];
    tfp_sprintf(string_buffer, "INAV VERSION: %s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 5, y++, string_buffer);
#ifdef USE_CMS
    displayWrite(osdDisplayPort, 7, y++,  CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, y++, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, y++, CMS_STARTUP_HELP_TEXT3);
#endif

#ifdef USE_STATS
#define STATS_LABEL_X_POS 4
#define STATS_VALUE_X_POS 24
    if (statsConfig()->stats_enabled) {
        displayWrite(osdDisplayPort, STATS_LABEL_X_POS, ++y, "ODOMETER:");
        switch (osdConfig()->units) {
            case OSD_UNIT_UK:
                FALLTHROUGH;
            case OSD_UNIT_IMPERIAL:
                tfp_sprintf(string_buffer, "%5d", (int)(statsConfig()->stats_total_dist / METERS_PER_MILE));
                string_buffer[5] = SYM_MI;
                break;
            default:
            case OSD_UNIT_METRIC_MPH:
                FALLTHROUGH;
            case OSD_UNIT_METRIC:
                tfp_sprintf(string_buffer, "%5d", (int)(statsConfig()->stats_total_dist / METERS_PER_KILOMETER));
                string_buffer[5] = SYM_KM;
                break;
        }
        string_buffer[6] = '\0';
        displayWrite(osdDisplayPort, STATS_VALUE_X_POS-5, y,  string_buffer);

        displayWrite(osdDisplayPort, STATS_LABEL_X_POS, ++y, "TOTAL TIME:");
        uint32_t tot_mins = statsConfig()->stats_total_time / 60;
        tfp_sprintf(string_buffer, "%2d:%02dHM", (int)(tot_mins / 60), (int)(tot_mins % 60));
        displayWrite(osdDisplayPort, STATS_VALUE_X_POS-5, y,  string_buffer);

#ifdef USE_ADC
        if (feature(FEATURE_VBAT) && feature(FEATURE_CURRENT_METER)) {
            displayWrite(osdDisplayPort, STATS_LABEL_X_POS, ++y, "TOTAL ENERGY:");
            osdFormatCentiNumber(string_buffer, statsConfig()->stats_total_energy / 10, 0, 2, 0, 4);
            strcat(string_buffer, "\xAB"); // SYM_WH
            displayWrite(osdDisplayPort, STATS_VALUE_X_POS-4, y,  string_buffer);

            displayWrite(osdDisplayPort, STATS_LABEL_X_POS, ++y, "AVG EFFICIENCY:");
            if (statsConfig()->stats_total_dist) {
                uint32_t avg_efficiency = statsConfig()->stats_total_energy / (statsConfig()->stats_total_dist / METERS_PER_KILOMETER); // mWh/km
                switch (osdConfig()->units) {
                    case OSD_UNIT_UK:
                        FALLTHROUGH;
                    case OSD_UNIT_IMPERIAL:
                        osdFormatCentiNumber(string_buffer, avg_efficiency / 10, 0, 2, 0, 3);
                        string_buffer[3] = SYM_WH_MI;
                        break;
                    default:
                    case OSD_UNIT_METRIC_MPH:
                        FALLTHROUGH;
                    case OSD_UNIT_METRIC:
                        osdFormatCentiNumber(string_buffer, avg_efficiency / 10000 * METERS_PER_MILE, 0, 2, 0, 3);
                        string_buffer[3] = SYM_WH_KM;
                        break;
                }
            } else {
                string_buffer[0] = string_buffer[1] = string_buffer[2] = '-';
            }
            string_buffer[4] = '\0';
            displayWrite(osdDisplayPort, STATS_VALUE_X_POS-3, y,  string_buffer);
        }
#endif // USE_ADC
    }
#endif

    displayCommitTransaction(osdDisplayPort);
    displayResync(osdDisplayPort);
    osdSetNextRefreshIn(SPLASH_SCREEN_DISPLAY_TIME);
}

void osdInit(displayPort_t *osdDisplayPortToUse)
{
    if (!osdDisplayPortToUse)
        return;

    BUILD_BUG_ON(OSD_POS_MAX != OSD_POS(31,31));

    osdDisplayPort = osdDisplayPortToUse;

#ifdef USE_CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif

    armState = ARMING_FLAG(ARMED);
    osdCompleteAsyncInitialization();
}

static void osdResetStats(void)
{
    stats.max_current = 0;
    stats.max_power = 0;
    stats.max_speed = 0;
    stats.min_voltage = 5000;
    stats.min_rssi = 99;
    stats.min_lq = 300;
    stats.min_rssi_dbm = 0;
    stats.max_altitude = 0;
}

static void osdUpdateStats(void)
{
    int32_t value;

    if (feature(FEATURE_GPS)) {
        value = osdGet3DSpeed();
        if (stats.max_speed < value)
            stats.max_speed = value;

        if (stats.max_distance < GPS_distanceToHome)
            stats.max_distance = GPS_distanceToHome;
    }

    value = getBatteryVoltage();
    if (stats.min_voltage > value)
        stats.min_voltage = value;

    value = abs(getAmperage());
    if (stats.max_current < value)
        stats.max_current = value;

    value = labs(getPower());
    if (stats.max_power < value)
        stats.max_power = value;

    value = osdConvertRSSI();
    if (stats.min_rssi > value)
        stats.min_rssi = value;

    value = osdGetCrsfLQ();
    if (stats.min_lq > value)
        stats.min_lq = value;

    value = osdGetCrsfdBm();
    if (stats.min_rssi_dbm > value)
        stats.min_rssi_dbm = value;

    stats.max_altitude = MAX(stats.max_altitude, osdGetAltitude());
}

static void osdShowStatsPage1(void)
{
    const char * disarmReasonStr[DISARM_REASON_COUNT] = { "UNKNOWN", "TIMEOUT", "STICKS", "SWITCH", "SWITCH", "KILLSW", "FAILSAFE", "NAV SYS" };
    uint8_t top = 1;    /* first fully visible line */
    const uint8_t statNameX = 1;
    const uint8_t statValuesX = 20;
    char buff[10];
    statsPagesCheck = 1;

    displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
    displayClearScreen(osdDisplayPort);

    displayWrite(osdDisplayPort, statNameX, top++, "--- STATS ---      1/2 ->");

    if (feature(FEATURE_GPS)) {
        displayWrite(osdDisplayPort, statNameX, top, "MAX SPEED        :");
        osdFormatVelocityStr(buff, stats.max_speed, true);
        osdLeftAlignString(buff);
        displayWrite(osdDisplayPort, statValuesX, top++, buff);

        displayWrite(osdDisplayPort, statNameX, top, "MAX DISTANCE     :");
        osdFormatDistanceStr(buff, stats.max_distance*100);
        displayWrite(osdDisplayPort, statValuesX, top++, buff);

        displayWrite(osdDisplayPort, statNameX, top, "TRAVELED DISTANCE:");
        osdFormatDistanceStr(buff, getTotalTravelDistance());
        displayWrite(osdDisplayPort, statValuesX, top++, buff);
    }

    displayWrite(osdDisplayPort, statNameX, top, "MAX ALTITUDE     :");
    osdFormatAltitudeStr(buff, stats.max_altitude);
    displayWrite(osdDisplayPort, statValuesX, top++, buff);

    switch (rxConfig()->serialrx_provider) {
        case SERIALRX_CRSF:
            displayWrite(osdDisplayPort, statNameX, top, "MIN LQ           :");
            itoa(stats.min_lq, buff, 10);
            strcat(buff, "%");
            displayWrite(osdDisplayPort, statValuesX, top++, buff);

            displayWrite(osdDisplayPort, statNameX, top, "MIN RSSI         :");
            itoa(stats.min_rssi_dbm, buff, 10);
            tfp_sprintf(buff, "%s%c", buff, SYM_DBM);
            displayWrite(osdDisplayPort, statValuesX, top++, buff);
            break;
        default:
            displayWrite(osdDisplayPort, statNameX, top, "MIN RSSI         :");
            itoa(stats.min_rssi, buff, 10);
            strcat(buff, "%");
            displayWrite(osdDisplayPort, statValuesX, top++, buff);
        }

    displayWrite(osdDisplayPort, statNameX, top, "FLY TIME         :");
    uint16_t flySeconds = getFlightTime();
    uint16_t flyMinutes = flySeconds / 60;
    flySeconds %= 60;
    uint16_t flyHours = flyMinutes / 60;
    flyMinutes %= 60;
    tfp_sprintf(buff, "%02u:%02u:%02u", flyHours, flyMinutes, flySeconds);
    displayWrite(osdDisplayPort, statValuesX, top++, buff);

    displayWrite(osdDisplayPort, statNameX, top, "DISARMED BY      :");
    displayWrite(osdDisplayPort, statValuesX, top++, disarmReasonStr[getDisarmReason()]);
    displayCommitTransaction(osdDisplayPort);
}

static void osdShowStatsPage2(void)
{
    uint8_t top = 1;    /* first fully visible line */
    const uint8_t statNameX = 1;
    const uint8_t statValuesX = 20;
    char buff[10];
    statsPagesCheck = 1;

    displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
    displayClearScreen(osdDisplayPort);

    displayWrite(osdDisplayPort, statNameX, top++, "--- STATS ---   <- 2/2");

    if (osdConfig()->stats_min_voltage_unit == OSD_STATS_MIN_VOLTAGE_UNIT_BATTERY) {
        displayWrite(osdDisplayPort, statNameX, top, "MIN BATTERY VOLT :");
        osdFormatCentiNumber(buff, stats.min_voltage, 0, osdConfig()->main_voltage_decimals, 0, osdConfig()->main_voltage_decimals + 2);
    } else {
        displayWrite(osdDisplayPort, statNameX, top, "MIN CELL VOLTAGE :");
        osdFormatCentiNumber(buff, stats.min_voltage/getBatteryCellCount(), 0, 2, 0, 3);
    }
    tfp_sprintf(buff, "%s%c", buff, SYM_VOLT);
    displayWrite(osdDisplayPort, statValuesX, top++, buff);

    if (feature(FEATURE_CURRENT_METER)) {
        displayWrite(osdDisplayPort, statNameX, top, "MAX CURRENT      :");
        osdFormatCentiNumber(buff, stats.max_current, 0, 2, 0, 3);
        tfp_sprintf(buff, "%s%c", buff, SYM_AMP);
        displayWrite(osdDisplayPort, statValuesX, top++, buff);

        displayWrite(osdDisplayPort, statNameX, top, "MAX POWER        :");
        bool kiloWatt = osdFormatCentiNumber(buff, stats.max_power, 1000, 2, 2, 3);
        buff[3] = kiloWatt ? SYM_KILOWATT : SYM_WATT;
        buff[4] = '\0';
        displayWrite(osdDisplayPort, statValuesX, top++, buff);

        displayWrite(osdDisplayPort, statNameX, top, "USED CAPACITY    :");
        if (osdConfig()->stats_energy_unit == OSD_STATS_ENERGY_UNIT_MAH) {
            tfp_sprintf(buff, "%d%c", (int)getMAhDrawn(), SYM_MAH);
        } else {
            osdFormatCentiNumber(buff, getMWhDrawn() / 10, 0, 2, 0, 3);
            tfp_sprintf(buff, "%s%c", buff, SYM_WH);
        }
        displayWrite(osdDisplayPort, statValuesX, top++, buff);

        int32_t totalDistance = getTotalTravelDistance();
        bool moreThanAh = false;
        bool efficiencyValid = totalDistance >= 10000;
        if (feature(FEATURE_GPS)) {
            displayWrite(osdDisplayPort, statNameX, top, "AVG EFFICIENCY   :");
            switch (osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    if (osdConfig()->stats_energy_unit == OSD_STATS_ENERGY_UNIT_MAH) {
                        moreThanAh = osdFormatCentiNumber(buff, (int32_t)(getMAhDrawn() * 10000.0f * METERS_PER_MILE / totalDistance), 1000, 0, 2, 3);
                        if (!moreThanAh) {
                            tfp_sprintf(buff, "%s%c%c", buff, SYM_MAH_MI_0, SYM_MAH_MI_1);
                        } else {
                            tfp_sprintf(buff, "%s%c", buff, SYM_AH_MI);
                        }
                        if (!efficiencyValid) {
                            buff[0] = buff[1] = buff[2] = '-';
                            buff[3] = SYM_MAH_MI_0;
                            buff[4] = SYM_MAH_MI_1;
                            buff[5] = '\0';
                        }
                    } else {
                        osdFormatCentiNumber(buff, (int32_t)(getMWhDrawn() * 10.0f * METERS_PER_MILE / totalDistance), 0, 2, 0, 3);
                        tfp_sprintf(buff, "%s%c", buff, SYM_WH_MI);
                        if (!efficiencyValid) {
                            buff[0] = buff[1] = buff[2] = '-';
                        }
                    }
                    break;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    if (osdConfig()->stats_energy_unit == OSD_STATS_ENERGY_UNIT_MAH) {
                        moreThanAh = osdFormatCentiNumber(buff, (int32_t)(getMAhDrawn() * 10000000.0f / totalDistance), 1000, 0, 2, 3);
                        if (!moreThanAh) {
                            tfp_sprintf(buff, "%s%c%c", buff, SYM_MAH_KM_0, SYM_MAH_KM_1);
                        } else {
                            tfp_sprintf(buff, "%s%c", buff, SYM_AH_KM);
                        }
                        if (!efficiencyValid) {
                            buff[0] = buff[1] = buff[2] = '-';
                            buff[3] = SYM_MAH_KM_0;
                            buff[4] = SYM_MAH_KM_1;
                            buff[5] = '\0';
                        }
                    } else {
                        osdFormatCentiNumber(buff, (int32_t)(getMWhDrawn() * 10000.0f / totalDistance), 0, 2, 0, 3);
                        tfp_sprintf(buff, "%s%c", buff, SYM_WH_KM);
                        if (!efficiencyValid) {
                            buff[0] = buff[1] = buff[2] = '-';
                        }
                    }
                    break;
            }
            osdLeftAlignString(buff);
            displayWrite(osdDisplayPort, statValuesX, top++, buff);
        }
    }

    const float max_gforce = accGetMeasuredMaxG();
    displayWrite(osdDisplayPort, statNameX, top, "MAX G-FORCE      :");
    osdFormatCentiNumber(buff, max_gforce * 100, 0, 2, 0, 3);
    displayWrite(osdDisplayPort, statValuesX, top++, buff);

    const acc_extremes_t *acc_extremes = accGetMeasuredExtremes();
    displayWrite(osdDisplayPort, statNameX, top, "MIN/MAX Z G-FORCE:");
    osdFormatCentiNumber(buff, acc_extremes[Z].min * 100, 0, 2, 0, 4);
    strcat(buff,"/");
    displayWrite(osdDisplayPort, statValuesX - 1, top, buff);
    osdFormatCentiNumber(buff, acc_extremes[Z].max * 100, 0, 2, 0, 3);
    displayWrite(osdDisplayPort, statValuesX + 4, top++, buff);
    displayCommitTransaction(osdDisplayPort);
}

// called when motors armed
static void osdShowArmed(void)
{
    dateTime_t dt;
    char buf[MAX(32, FORMATTED_DATE_TIME_BUFSIZE)];
    char craftNameBuf[MAX_NAME_LENGTH];
    char versionBuf[30];
    char *date;
    char *time;
    // We need 12 visible rows, start row never < first fully visible row 1
    uint8_t y = osdDisplayPort->rows > 13 ? (osdDisplayPort->rows - 12) / 2 : 1;

    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12, y, "ARMED");
    y += 2;

    if (strlen(systemConfig()->name) > 0) {
        osdFormatCraftName(craftNameBuf);
        displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(systemConfig() -> name)) / 2, y, craftNameBuf );
        y += 1;
    }

    if (posControl.waypointListValid && posControl.waypointCount > 0) {
        displayWrite(osdDisplayPort, 7, y, "*MISSION LOADED*");
    }
    y += 1;

#if defined(USE_GPS)
    if (feature(FEATURE_GPS)) {
        if (STATE(GPS_FIX_HOME)) {
            if (osdConfig()->osd_home_position_arm_screen){
                osdFormatCoordinate(buf, SYM_LAT, GPS_home.lat);
                displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(buf)) / 2, y, buf);
                osdFormatCoordinate(buf, SYM_LON, GPS_home.lon);
                displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(buf)) / 2, y + 1, buf);
                int digits = osdConfig()->plus_code_digits;
                olc_encode(GPS_home.lat, GPS_home.lon, digits, buf, sizeof(buf));
                displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(buf)) / 2, y + 2, buf);
            }
            y += 4;
#if defined (USE_SAFE_HOME)
            if (safehome_distance) { // safehome found during arming
                if (navConfig()->general.flags.safehome_usage_mode == SAFEHOME_USAGE_OFF) {
                    strcpy(buf, "SAFEHOME FOUND; MODE OFF");
				} else {
					char buf2[12]; // format the distance first
					osdFormatDistanceStr(buf2, safehome_distance);
					tfp_sprintf(buf, "%c - %s -> SAFEHOME %u", SYM_HOME, buf2, safehome_index);
				}
				textAttributes_t elemAttr = _TEXT_ATTRIBUTES_BLINK_BIT;
				// write this message above the ARMED message to make it obvious
				displayWriteWithAttr(osdDisplayPort, (osdDisplayPort->cols - strlen(buf)) / 2, y - 8, buf, elemAttr);
            }
#endif
        } else {
            strcpy(buf, "!NO HOME POSITION!");
            displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(buf)) / 2, y, buf);
            y += 1;
        }
    }
#endif

    if (rtcGetDateTime(&dt)) {
        dateTimeFormatLocal(buf, &dt);
        dateTimeSplitFormatted(buf, &date, &time);

        displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(date)) / 2, y, date);
        displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(time)) / 2, y + 1, time);
        y += 3;
    }

    tfp_sprintf(versionBuf, "INAV VERSION: %s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, (osdDisplayPort->cols - strlen(versionBuf)) / 2, y, versionBuf);
}

static void osdFilterData(timeUs_t currentTimeUs) {
    static timeUs_t lastRefresh = 0;
    float refresh_dT = US2S(cmpTimeUs(currentTimeUs, lastRefresh));

    GForce = fast_fsqrtf(vectorNormSquared(&imuMeasuredAccelBF)) / GRAVITY_MSS;
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) GForceAxis[axis] = imuMeasuredAccelBF.v[axis] / GRAVITY_MSS;

    if (lastRefresh) {
        GForce = pt1FilterApply3(&GForceFilter, GForce, refresh_dT);
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) pt1FilterApply3(GForceFilterAxis + axis, GForceAxis[axis], refresh_dT);
    } else {
        pt1FilterInitRC(&GForceFilter, GFORCE_FILTER_TC, 0);
        pt1FilterReset(&GForceFilter, GForce);

        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
            pt1FilterInitRC(GForceFilterAxis + axis, GFORCE_FILTER_TC, 0);
            pt1FilterReset(GForceFilterAxis + axis, GForceAxis[axis]);
        }
    }

    lastRefresh = currentTimeUs;
}

static void osdRefresh(timeUs_t currentTimeUs)
{
    osdFilterData(currentTimeUs);

#ifdef USE_CMS
    if (IS_RC_MODE_ACTIVE(BOXOSD) && (!cmsInMenu) && !(osdConfig()->osd_failsafe_switch_layout && FLIGHT_MODE(FAILSAFE_MODE))) {
#else
    if (IS_RC_MODE_ACTIVE(BOXOSD) && !(osdConfig()->osd_failsafe_switch_layout && FLIGHT_MODE(FAILSAFE_MODE))) {
#endif
      displayClearScreen(osdDisplayPort);
      armState = ARMING_FLAG(ARMED);
      return;
    }

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED)) {
            osdResetStats();
            osdShowArmed(); // reset statistic etc
            uint32_t delay = ARMED_SCREEN_DISPLAY_TIME;
            statsPagesCheck = 0;
#if defined(USE_SAFE_HOME)
            if (safehome_distance)
                delay *= 3;
#endif
            osdSetNextRefreshIn(delay);
        } else {
            osdShowStatsPage1(); // show first page of statistic
            osdSetNextRefreshIn(STATS_SCREEN_DISPLAY_TIME);
        }

        armState = ARMING_FLAG(ARMED);
    }

    if (resumeRefreshAt) {
        // If we already reached he time for the next refresh,
        // or THR is high or PITCH is high, resume refreshing.
        // Clear the screen first to erase other elements which
        // might have been drawn while the OSD wasn't refreshing.

        if (!DELAYED_REFRESH_RESUME_COMMAND)
            refreshWaitForResumeCmdRelease = false;

        if ((currentTimeUs > resumeRefreshAt) || ((!refreshWaitForResumeCmdRelease) && DELAYED_REFRESH_RESUME_COMMAND)) {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
        } else if ((currentTimeUs > resumeRefreshAt) || ((!refreshWaitForResumeCmdRelease) && STATS_PAGE1)) {
            if (statsPagesCheck == 1) {
                osdShowStatsPage1();
            }
        } else if ((currentTimeUs > resumeRefreshAt) || ((!refreshWaitForResumeCmdRelease) && STATS_PAGE2)) {
            if (statsPagesCheck == 1) {
                osdShowStatsPage2();
            }
        } else {
            displayHeartbeat(osdDisplayPort);
        }
        return;
    }

#ifdef USE_CMS
    if (!displayIsGrabbed(osdDisplayPort)) {
        displayBeginTransaction(osdDisplayPort, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
        if (fullRedraw) {
            displayClearScreen(osdDisplayPort);
            fullRedraw = false;
        }
        osdDrawNextElement();
        displayHeartbeat(osdDisplayPort);
        displayCommitTransaction(osdDisplayPort);
#ifdef OSD_CALLS_CMS
    } else {
        cmsUpdate(currentTimeUs);
#endif
    }
#endif
}

/*
 * Called periodically by the scheduler
 */
void osdUpdate(timeUs_t currentTimeUs)
{
    static uint32_t counter = 0;

    // don't touch buffers if DMA transaction is in progress
    if (displayIsTransferInProgress(osdDisplayPort)) {
        return;
    }

    if (!osdDisplayIsReady) {
        osdCompleteAsyncInitialization();
        return;
    }

#if defined(OSD_ALTERNATE_LAYOUT_COUNT) && OSD_ALTERNATE_LAYOUT_COUNT > 0
    // Check if the layout has changed. Higher numbered
    // boxes take priority.
    unsigned activeLayout;
    if (layoutOverride >= 0) {
        activeLayout = layoutOverride;
        // Check for timed override, it will go into effect on
        // the next OSD iteration
        if (layoutOverrideUntil > 0 && millis() > layoutOverrideUntil) {
            layoutOverrideUntil = 0;
            layoutOverride = -1;
        }
    } else if (osdConfig()->osd_failsafe_switch_layout && FLIGHT_MODE(FAILSAFE_MODE)) {
        activeLayout = 0;
    } else {
#if OSD_ALTERNATE_LAYOUT_COUNT > 2
        if (IS_RC_MODE_ACTIVE(BOXOSDALT3))
            activeLayout = 3;
        else
#endif
#if OSD_ALTERNATE_LAYOUT_COUNT > 1
        if (IS_RC_MODE_ACTIVE(BOXOSDALT2))
            activeLayout = 2;
        else
#endif
        if (IS_RC_MODE_ACTIVE(BOXOSDALT1))
            activeLayout = 1;
        else
#ifdef USE_PROGRAMMING_FRAMEWORK
        if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT))
            activeLayout = constrain(logicConditionValuesByType[LOGIC_CONDITION_SET_OSD_LAYOUT], 0, OSD_ALTERNATE_LAYOUT_COUNT);
        else
#endif
            activeLayout = 0;
    }
    if (currentLayout != activeLayout) {
        currentLayout = activeLayout;
        osdStartFullRedraw();
    }
#endif

#define DRAW_FREQ_DENOM     4
#define STATS_FREQ_DENOM    50
    counter++;

    if ((counter % STATS_FREQ_DENOM) == 0) {
        osdUpdateStats();
    }

    if ((counter & DRAW_FREQ_DENOM) == 0) {
        // redraw values in buffer
        osdRefresh(currentTimeUs);
    } else {
        // rest of time redraw screen
        displayDrawScreen(osdDisplayPort);
    }

#ifdef USE_CMS
    // do not allow ARM if we are in menu
    if (displayIsGrabbed(osdDisplayPort)) {
        ENABLE_ARMING_FLAG(ARMING_DISABLED_OSD_MENU);
    } else {
        DISABLE_ARMING_FLAG(ARMING_DISABLED_OSD_MENU);
    }
#endif
}

void osdStartFullRedraw(void)
{
    fullRedraw = true;
}

void osdOverrideLayout(int layout, timeMs_t duration)
{
    layoutOverride = constrain(layout, -1, ARRAYLEN(osdLayoutsConfig()->item_pos) - 1);
    if (layoutOverride >= 0 && duration > 0) {
        layoutOverrideUntil = millis() + duration;
    } else {
        layoutOverrideUntil = 0;
    }
}

int osdGetActiveLayout(bool *overridden)
{
    if (overridden) {
        *overridden = layoutOverride >= 0;
    }
    return currentLayout;
}

bool osdItemIsFixed(osd_items_e item)
{
    return item == OSD_CROSSHAIRS ||
        item == OSD_ARTIFICIAL_HORIZON ||
        item == OSD_HORIZON_SIDEBARS;
}

displayPort_t *osdGetDisplayPort(void)
{
    return osdDisplayPort;
}

displayCanvas_t *osdGetDisplayPortCanvas(void)
{
#if defined(USE_CANVAS)
    if (osdDisplayHasCanvas) {
        return &osdCanvas;
    }
#endif
    return NULL;
}

textAttributes_t osdGetSystemMessage(char *buff, size_t buff_size, bool isCenteredText)
{
    textAttributes_t elemAttr = TEXT_ATTRIBUTES_NONE;

    if (buff != NULL) {
        const char *message = NULL;
        char messageBuf[MAX(SETTING_MAX_NAME_LENGTH, OSD_MESSAGE_LENGTH+1)];
        if (ARMING_FLAG(ARMED)) {
            // Aircraft is armed. We might have up to 5
            // messages to show.
            const char *messages[5];
            unsigned messageCount = 0;
            if (FLIGHT_MODE(FAILSAFE_MODE)) {
                // In FS mode while being armed too
                const char *failsafePhaseMessage = osdFailsafePhaseMessage();
                const char *failsafeInfoMessage = osdFailsafeInfoMessage();
                const char *navStateFSMessage = navigationStateMessage();

                if (failsafePhaseMessage) {
                    messages[messageCount++] = failsafePhaseMessage;
                }
                if (failsafeInfoMessage) {
                    messages[messageCount++] = failsafeInfoMessage;
                }
                if (navStateFSMessage) {
                    messages[messageCount++] = navStateFSMessage;
                }
#if defined(USE_SAFE_HOME)
                const char *safehomeMessage = divertingToSafehomeMessage();
				if (safehomeMessage) {
					messages[messageCount++] = safehomeMessage;
				}
#endif
                if (messageCount > 0) {
                    message = messages[OSD_ALTERNATING_CHOICES(1000, messageCount)];
                    if (message == failsafeInfoMessage) {
                        // failsafeInfoMessage is not useful for recovering
                        // a lost model, but might help avoiding a crash.
                        // Blink to grab user attention.
                        TEXT_ATTRIBUTES_ADD_BLINK(elemAttr);
                    }
                    // We're shoing either failsafePhaseMessage or
                    // navStateFSMessage. Don't BLINK here since
                    // having this text available might be crucial
                    // during a lost aircraft recovery and blinking
                    // will cause it to be missing from some frames.
                }
            } else {
                if (FLIGHT_MODE(NAV_RTH_MODE) || FLIGHT_MODE(NAV_WP_MODE) || navigationIsExecutingAnEmergencyLanding()) {
                    if (isWaypointMissionRTHActive()) {
                        // if RTH activated whilst WP mode selected, remind pilot to cancel WP mode to exit RTH
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_WP_RTH_CANCEL);
                    }

                    if (navGetCurrentStateFlags() & NAV_AUTO_WP_DONE) {
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_WP_FINISHED);
                    } else if (NAV_Status.state == MW_NAV_STATE_WP_ENROUTE) {
                        // Countdown display for remaining Waypoints
                        char buf[6];
                        osdFormatDistanceSymbol(buf, posControl.wpDistance, 0);
                        tfp_sprintf(messageBuf, "TO WP %u/%u (%s)", getGeoWaypointNumber(posControl.activeWaypointIndex), posControl.geoWaypointCount, buf);
                        messages[messageCount++] = messageBuf;
                    } else if (NAV_Status.state == MW_NAV_STATE_HOLD_TIMED) {
                        // WP hold time countdown in seconds
                        timeMs_t currentTime = millis();
                        int holdTimeRemaining = posControl.waypointList[posControl.activeWaypointIndex].p1 - (int)((currentTime - posControl.wpReachedTime)/1000);
                        if (holdTimeRemaining >=0) {
                            tfp_sprintf(messageBuf, "HOLDING WP FOR %2u S", holdTimeRemaining);
                            messages[messageCount++] = messageBuf;
                        }
                    } else {
                        const char *navStateMessage = navigationStateMessage();
                        if (navStateMessage) {
                            messages[messageCount++] = navStateMessage;
                        }
                    }
#if defined(USE_SAFE_HOME)
					const char *safehomeMessage = divertingToSafehomeMessage();
					if (safehomeMessage) {
						messages[messageCount++] = safehomeMessage;
					}
#endif
                } else if (STATE(FIXED_WING_LEGACY) && (navGetCurrentStateFlags() & NAV_CTL_LAUNCH)) {
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_AUTOLAUNCH);
                        const char *launchStateMessage = fixedWingLaunchStateMessage();
                        if (launchStateMessage) {
                            messages[messageCount++] = launchStateMessage;
                        }
                } else {
                    if (FLIGHT_MODE(NAV_ALTHOLD_MODE) && !navigationRequiresAngleMode()) {
                        // ALTHOLD might be enabled alongside ANGLE/HORIZON/ACRO
                        // when it doesn't require ANGLE mode (required only in FW
                        // right now). If if requires ANGLE, its display is handled
                        // by OSD_FLYMODE.
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_ALTITUDE_HOLD);
                    }
                    if (IS_RC_MODE_ACTIVE(BOXAUTOTRIM) && !feature(FEATURE_FW_AUTOTRIM)) {
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_AUTOTRIM);
                    }
                    if (IS_RC_MODE_ACTIVE(BOXAUTOTUNE)) {
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_AUTOTUNE);
                        if (FLIGHT_MODE(MANUAL_MODE)) {
                            messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_AUTOTUNE_ACRO);
                        }
                    }
                    if (FLIGHT_MODE(HEADFREE_MODE)) {
                        messages[messageCount++] = OSD_MESSAGE_STR(OSD_MSG_HEADFREE);
                    }
                }
                // Pick one of the available messages. Each message lasts
                // a second.
                if (messageCount > 0) {
                    message = messages[OSD_ALTERNATING_CHOICES(1000, messageCount)];
                }
            }
        } else if (ARMING_FLAG(ARMING_DISABLED_ALL_FLAGS)) {
            unsigned invalidIndex;
            // Check if we're unable to arm for some reason
            if (ARMING_FLAG(ARMING_DISABLED_INVALID_SETTING) && !settingsValidate(&invalidIndex)) {
                if (OSD_ALTERNATING_CHOICES(1000, 2) == 0) {
                    const setting_t *setting = settingGet(invalidIndex);
                    settingGetName(setting, messageBuf);
                    for (int ii = 0; messageBuf[ii]; ii++) {
                        messageBuf[ii] = sl_toupper(messageBuf[ii]);
                    }
                    message = messageBuf;
                } else {
                    message = OSD_MESSAGE_STR(OSD_MSG_INVALID_SETTING);
                    TEXT_ATTRIBUTES_ADD_INVERTED(elemAttr);
                }
            } else {
                if (OSD_ALTERNATING_CHOICES(1000, 2) == 0) {
                    message = OSD_MESSAGE_STR(OSD_MSG_UNABLE_ARM);
                    TEXT_ATTRIBUTES_ADD_INVERTED(elemAttr);
                } else {
                    // Show the reason for not arming
                    message = osdArmingDisabledReasonMessage();
                }
            }
        }
        osdFormatMessage(buff, buff_size, message, isCenteredText);
    }
    return elemAttr;
}

#endif // OSD
