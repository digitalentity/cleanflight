/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>

#if defined(SEMIHOSTING)
#include <stdio.h>
#endif

#include "build/version.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "common/log.h"
#include "common/printf.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group_ids.h"

#include "io/serial.h"

#include "fc/config.h"
#include "fc/settings.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"
#include "msp/msp_protocol.h"

#if defined(USE_LOG)

#define LOG_PREFIX                  "[%6d.%03d] "
#define LOG_PREFIX_FORMATTED_SIZE   13

static serialPort_t * logPort = NULL;
static mspPort_t * mspLogPort = NULL;

PG_REGISTER(logConfig_t, logConfig, PG_LOG_CONFIG, 0);

PG_RESET_TEMPLATE(logConfig_t, logConfig,
    .level = SETTING_LOG_LEVEL_DEFAULT,
    .topics = SETTING_LOG_TOPICS_DEFAULT
);

void logInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LOG);
    if (!portConfig) {
        return;
    }

    bool portIsSharedWithMSP = false;

    if (determinePortSharing(portConfig, FUNCTION_LOG) == PORTSHARING_SHARED) {
        // We support sharing a LOG port only with MSP
        if (portConfig->functionMask != (FUNCTION_LOG | FUNCTION_MSP)) {
            return;
        }
        portIsSharedWithMSP = true;
    }

    // If the port is shared with MSP, reuse the port
    if (portIsSharedWithMSP) {
        const serialPort_t *logAndMspPort = findSharedSerialPort(FUNCTION_LOG, FUNCTION_MSP);
        if (!logAndMspPort) {
            return;
        }

        mspLogPort = mspSerialPortFind(logAndMspPort);
        if (!mspLogPort) {
            return;
        }

    } else {
        logPort = openSerialPort(portConfig->identifier, FUNCTION_LOG, NULL, NULL, baudRates[BAUD_921600], MODE_TX, SERIAL_NOT_INVERTED);
        if (!logPort) {
            return;
        }
    }
    // Initialization done
    LOG_I(SYSTEM, "%s/%s %s %s / %s (%s)",
        FC_FIRMWARE_NAME,
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
}

static void logPutcp(void *p, char ch)
{
    *(*((char **) p))++ = ch;
}

static void logPrint(const char *buf, size_t size)
{
#if defined(SEMIHOSTING)
    static bool semihostingInitialized = false;
    extern void initialise_monitor_handles(void);

    if (!semihostingInitialized) {
        initialise_monitor_handles();
        semihostingInitialized = true;
    }
    for (size_t ii = 0; ii < size; ii++) {
        fputc(buf[ii], stdout);
    }
#endif
    if (logPort) {
        // Send data via UART (if configured & connected - a safeguard against zombie VCP)
        if (serialIsConnected(logPort)) {
            serialPrint(logPort, buf);
        }
    } else if (mspLogPort) {
        mspSerialPushPort(MSP_DEBUGMSG, (uint8_t*)buf, size, mspLogPort, MSP_V2_NATIVE);
    }
}

static size_t logFormatPrefix(char *buf, const timeMs_t timeMs)
{
    // Write timestamp
    return tfp_sprintf(buf, LOG_PREFIX, (int)(timeMs / 1000), (int)(timeMs % 1000));
}

static bool logHasOutput(void)
{
#if defined(SEMIHOSTING)
    return true;
#else
    return logPort || mspLogPort;
#endif
}

static bool logIsEnabled(logTopic_e topic, unsigned level)
{
    return logHasOutput() && (level <= logConfig()->level || (logConfig()->topics & (1 << topic)));
}

void _logf(logTopic_e topic, unsigned level, const char *fmt, ...)
{
    char buf[128];
    char *bufPtr;
    int charCount;

    STATIC_ASSERT(MSP_PORT_OUTBUF_SIZE >= sizeof(buf), MSP_PORT_OUTBUF_SIZE_not_big_enough_for_log);

    if (!logIsEnabled(topic, level)) {
        return;
    }

    charCount = logFormatPrefix(buf, millis());
    bufPtr = &buf[charCount];

    // Write message
    va_list va;
    va_start(va, fmt);
    charCount += tfp_format(&bufPtr, logPutcp, fmt, va);
    logPutcp(&bufPtr, '\n');
    logPutcp(&bufPtr, 0);
    charCount += 2;
    va_end(va);

    logPrint(buf, charCount);
}

void _logBufferHex(logTopic_e topic, unsigned level, const void *buffer, size_t size)
{
    // Print lines of up to maxBytes bytes. We need 5 characters per byte
    // 0xAB[space|\n]
    const size_t charsPerByte = 5;
    const size_t maxBytes = 8;
    char buf[LOG_PREFIX_FORMATTED_SIZE + charsPerByte * maxBytes + 1]; // +1 for the null terminator
    size_t bufPos = LOG_PREFIX_FORMATTED_SIZE;
    const uint8_t *inputPtr = buffer;

    if (!logIsEnabled(topic, level)) {
        return;
    }

    logFormatPrefix(buf, millis());

    for (size_t ii = 0; ii < size; ii++) {
        tfp_sprintf(buf + bufPos, "0x%02x ", inputPtr[ii]);
        bufPos += charsPerByte;
        if (bufPos == sizeof(buf)-1) {
            buf[bufPos-1] = '\n';
            buf[bufPos] = '\0';
            logPrint(buf, bufPos + 1);
            bufPos = LOG_PREFIX_FORMATTED_SIZE;
        }
    }

    if (bufPos > LOG_PREFIX_FORMATTED_SIZE) {
        buf[bufPos-1] = '\n';
        buf[bufPos] = '\0';
        logPrint(buf, bufPos + 1);
    }
}

#endif
