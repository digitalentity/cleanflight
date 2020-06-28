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
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "build/build_config.h"
#include "printf.h"
#include "common/string_light.h"

#include "maths.h"
#include "platform.h"
FILE_COMPILE_FOR_SPEED

#ifdef REQUIRE_PRINTF_LONG_SUPPORT

void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
    int n = 0;
    unsigned int d = 1;
    while (num / d >= base)
        d *= base;
    while (d != 0) {
        int dgt = num / d;
        num %= d;
        d /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

void li2a(long num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    uli2a(num, 10, 0, bf);
}

#endif

void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
    int n = 0;
    unsigned int d = 1;
    while (num / d >= base)
        d *= base;
    while (d != 0) {
        int dgt = num / d;
        num %= d;
        d /= base;
        if (n || dgt > 0 || d == 0) {
            *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
            ++n;
        }
    }
    *bf = 0;
}

void i2a(int num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    ui2a(num, 10, 0, bf);
}

int a2d(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    else
        return -1;
}

char a2i(char ch, const char **src, int base, int *nump)
{
    const char *p = *src;
    int num = 0;
    int digit;
    while ((digit = a2d(ch)) >= 0) {
        if (digit > base)
            break;
        num = num * base + digit;
        ch = *p++;
    }
    *src = p;
    *nump = num;
    return ch;
}

#ifndef HAVE_ITOA_FUNCTION

/*
 ** The following two functions together make up an itoa()
 ** implementation. Function i2a() is a 'private' function
 ** called by the public itoa() function.
 **
 ** itoa() takes three arguments:
 **        1) the integer to be converted,
 **        2) a pointer to a character conversion buffer,
 **        3) the radix for the conversion
 **           which can range between 2 and 36 inclusive
 **           range errors on the radix default it to base10
 ** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
 */

static char *_i2a(unsigned i, char *a, unsigned base)
{
    if (i / base > 0)
        a = _i2a(i / base, a, base);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % base];
    return a + 1;
}

char *itoa(int i, char *a, int base)
{
    if ((base < 2) || (base > 36))
        base = 10;
    if (i < 0) {
        *a = '-';
        *_i2a(-(unsigned) i, a + 1, base) = 0;
    } else
        *_i2a(i, a, base) = 0;
    return a;
}

#endif

char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t)(x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(ABS(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    memcpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
float fastA2F(const char *p)
{
    int frac = 0;
    float sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p)) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0f;
    if (*p == '-') {
        sign = -1.0f;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0f;
    while (valid_digit(*p)) {
        value = value * 10.0f + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        float pow10 = 10.0f;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0f;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308)
            expon = 308;

        // Calculate scaling factor.
        // while (expon >= 50) { scale *= 1E50f; expon -= 50; }
        while (expon >= 8) {
            scale *= 1E8f;
            expon -= 8;
        }
        while (expon > 0) {
            scale *= 10.0f;
            expon -= 1;
        }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

unsigned long int fastA2UL(const char *p)
{
    unsigned long int result = 0;
    unsigned char digit;

    while (white_space(*p)) {
        p += 1;
    }

    for ( ; ; p++) {
        digit = *p - '0';
        if (digit > 9) {
            break;
        }
        result *= 10;
        result += digit;
    }
    return result;
}

int fastA2I(const char *s)
{
    int sign = 1;
    int num = 0;
    int digit;

    while (white_space(*s)) {
        s++;
    }

    if (*s == '-') {
        sign = -1;
        s++;
    }

    while ((digit = a2d(*s)) >= 0) {
        if (digit > 10)
            break;
        num = num * 10 + digit;
        s++;
    }

    return sign * num;
}

// Converts floating point value to string with a specified number of decimal places.
// Decimal places to be between 0 and 4
// lenstring is the size of field
// ndecs is number of decimal places required
// Convert float to character with fixed ndecs decimal places
// Make sure outBuf has space for the output!
void ftoa_decs(float x, char *outBuf,int ndecs)
{
	//limit decimal places to 4
	int8_t ldecs = ndecs;
	if (ldecs > 4)ldecs = 4;
	if (ldecs < 0)ldecs = 0;

	// Calculate exponent for rounding
	int tdecs = ldecs;
	int32_t exp = 1;
	while (tdecs) { exp *= 10; tdecs--; }

	// Remove sign
	char csign[2];
	if (x < 0) { x *= -1; strcpy(csign, "-"); }
	else {csign[0] = '\0';}

	// Rounding
	long rr = round((double)x*(double)exp);

	// Convert to string
	char buf[30];
	itoa(rr, buf,10);
	int len = strlen(buf);

	// Make sure we have enough characters in outbuf - need at least cndecs+1 being ndecs plus integer part.
	// If the integer part is zero then we need to add 0s at the beginning

	sl_rightShift(buf, ldecs - len + 1, '0');

	len = strlen(buf);

	// Find the address where the decimal point should go
	char * dpoint = buf +len-ldecs;

	// Copy data to fraction buffer and terminate it
	char frac[10];
	strncpy(frac, dpoint, ldecs);
	frac[ldecs] = '\0';

	// Now terminate the outBuf at the decimal point.
	dpoint[0] = '\0';

	if (ldecs == 0)tfp_sprintf(outBuf, "%s%s", csign,buf);
	else tfp_sprintf(outBuf, "%s%s%s%s", csign,buf, ".", frac);

}



int isInteger(const char *s)
{
	int ind = 0;
	while (*s) {
		if (ind == 0 && *s == '-')
		{
			s++;
			continue;
		}
		if (sl_isdigit(*s++) == 0) return 0;
		ind++;
	}

	return 1;
}
