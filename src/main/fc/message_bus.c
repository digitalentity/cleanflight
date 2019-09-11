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

#include <stdarg.h>

#include "common/log.h"
#include "common/printf.h"
#include "common/utils.h"

#include "fc/message_bus.h"

#define MESSAGE_BUS_MESSAGE_COUNT_MAX 6
#define BUSMESSAGE_ARG_COUNT_MAX 5

typedef struct busMessage_s {
    const char *format;
    timeMs_t expiresAt;
    void *args[BUSMESSAGE_ARG_COUNT_MAX];
    uint8_t topic;
    uint8_t id;
} busMessage_t;

typedef struct messageBus_s {
    int count;
    busMessage_t messages[MESSAGE_BUS_MESSAGE_COUNT_MAX];
} messageBus_t;

static messageBus_t bus;

static bool messageBusMessageIsValid(busMessage_t *msg)
{
    return msg->format;
}

static void messageBusFreeMessage(busMessage_t *msg)
{
    msg->format = NULL;
}

static void vmessageBusInsert(busMessage_t *msg, busMessageTopic_e topic, busMessageId_t id, timeMs_t expiresIn, const char *fmt, va_list ap)
{
    msg->format = fmt;
    msg->topic = topic;
    msg->id = id;
    msg->expiresAt = expiresIn > 0 ? millis() + expiresIn : UINT32_MAX;
    int argIndex = 0;
    for (const char *p = fmt; *p; p++) {
        if (*p == '%') {
            while (true) {
                char ch = (*(++p));
                if (ch == '\0') {
                    // end
                    break;
                }
                if (ch == '%') {
                    // Escape %
                    break;
                }
                if ((ch >= '0' && ch <= '9') || ch == '.') {
                    // Padding/formatting, skip
                    continue;
                }
                if (ch == 'l') {
                    // Long specifier, unsupported
                    LOG_D(SYSTEM, "Invalid 'l' format modifier in message");
                    continue;
                }
                if (ch == 'd' || ch == 'c') {
                    if (argIndex >= BUSMESSAGE_ARG_COUNT_MAX) {
                        break;
                    }
                    int d = va_arg(ap, int);
                    msg->args[argIndex++] = (void *)d;
                    break;
                }
                if (ch == 'u' || ch == 'x' || ch == 'X') {
                    if (argIndex >= BUSMESSAGE_ARG_COUNT_MAX) {
                        break;
                    }
                    unsigned u = va_arg(ap, unsigned);
                    msg->args[argIndex++] = (void *)u;
                    break;
                }
                if (ch == 'f') {
                    if (argIndex >= BUSMESSAGE_ARG_COUNT_MAX - 1) {
                        break;
                    }
                    double f = va_arg(ap, double);
                    uint64_t *ptr = (uint64_t *)&f;
                    msg->args[argIndex++] = (void *)((uint32_t)(*ptr & UINT32_MAX));
                    msg->args[argIndex++] = (void *)((uint32_t)(*ptr >> 32));
                    break;
                }
                if (ch == 's') {
                    if (argIndex >= BUSMESSAGE_ARG_COUNT_MAX) {
                        break;
                    }
                    const char *s = va_arg(ap, const char *);
                    msg->args[argIndex++] = (void *)s;
                    break;
                }
                // Unsupported format specifier
                LOG_E(SYSTEM, "Unsupported message bus format specifier '%c'", ch);
                break;
            }
        }
    }
}

static bool vmessageBusPut(busMessageTopic_e topic, busMessageId_t id, timeMs_t expiresIn, const char *fmt, va_list ap)
{
    // First, check if we are overriding an existing message
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (messageBusMessageIsValid(&bus.messages[ii]) && bus.messages[ii].topic == topic && bus.messages[ii].id == id) {
            vmessageBusInsert(&bus.messages[ii], topic, id, expiresIn, fmt, ap);
            return true;
        }
    }
    // Check if we have empty space
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (!bus.messages[ii].format) {
            vmessageBusInsert(&bus.messages[ii], topic, id, expiresIn, fmt, ap);
            bus.count++;
            return true;
        }
    }
    // Delete the earliest expiring message and try again
    timeMs_t minExpiresAt = UINT32_MAX;
    int minExpiresAtIndex = -1;
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (messageBusMessageIsValid(&bus.messages[ii]) && bus.messages[ii].expiresAt < minExpiresAt) {
            minExpiresAt = bus.messages[ii].expiresAt;
            minExpiresAtIndex = ii;
        }
    }
    if (minExpiresAtIndex >= 0) {
        vmessageBusInsert(&bus.messages[minExpiresAtIndex], topic, id, expiresIn, fmt, ap);
        return true;
    }
    return false;
}

void messageBusUpdate(timeUs_t currentTimeUs)
{
    timeMs_t nowMs = currentTimeUs / 1000;

    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (messageBusMessageIsValid(&bus.messages[ii]) &&
            bus.messages[ii].expiresAt < nowMs) {

            messageBusFreeMessage(&bus.messages[ii]);
            bus.count--;
        }
    }
}

void messageBusPutf(busMessageTopic_e topic, busMessageId_t id, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    messageBusPutTimedf(topic, id, 0, fmt, ap);
    va_end(ap);
}

void messageBusPutTimedf(busMessageTopic_e topic, busMessageId_t id, timeMs_t expiresIn, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vmessageBusPut(topic, id, expiresIn, fmt, ap);
    va_end(ap);
}

void messageBusClear(uint8_t topic, uint8_t id)
{
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (messageBusMessageIsValid(&bus.messages[ii]) && bus.messages[ii].topic == topic && bus.messages[ii].id == id) {
            messageBusFreeMessage(&bus.messages[ii]);
            bus.count--;
            break;
        }
    }
}

void messageBusClearTopic(uint8_t topic)
{
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (messageBusMessageIsValid(&bus.messages[ii]) && bus.messages[ii].topic == topic) {
            messageBusFreeMessage(&bus.messages[ii]);
            bus.count--;
        }
    }
}

int messageBusGetCount(void)
{
    return bus.count;
}

static const busMessage_t * messageBusGetMessageAt(int index)
{
    int p = 0;
    for (unsigned ii = 0; ii < ARRAYLEN(bus.messages); ii++) {
        if (bus.messages[ii].format) {
            if (p == index) {
                return &bus.messages[ii];
            }
            p++;
        }
    }
    return NULL;
}

int messageBusFormatMessageAt(char *buf, size_t bufsize, int messageIndex)
{
    const busMessage_t *msg = messageBusGetMessageAt(messageIndex);
    if (msg) {
        // XXX: This constructs a va_list manually
        // for arm-eabi-none for a function with 4
        // or more arguments and the va_list at the end.
        // Other setups/architectures might fail with undefined
        // behavior here.
        const void *args[BUSMESSAGE_ARG_COUNT_MAX];
        for (unsigned ii = 0; ii < ARRAYLEN(args); ii++) {
            args[ii] = &msg->args[ii];
        }
        va_list *ap = (va_list *)args;
        return tfp_vsnprintf(buf, bufsize, msg->format, *ap);
    }
    return -1;
}
