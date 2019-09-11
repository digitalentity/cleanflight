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

#pragma once

#include <stdint.h>

#include "drivers/time.h"

typedef enum {
    BUS_MESSAGE_TOPIC_SYSTEM,
} busMessageTopic_e;

typedef uint8_t busMessageId_t;

void messageBusUpdate(timeUs_t currentTimeUs);

// Same as messageBusPutTimedf, but the message never expires and must be
// manually cleared.
void messageBusPutf(busMessageTopic_e topic, busMessageId_t id, const char *fmt, ...) __attribute__ ((format (printf, 3, 4)));
// Puts a message into the bus, which expires in the given milliseconds.
// The format string doesn't support long modifiers although padding/length
// modifiers are supported. The maximum number of arguments that can be using
// during formatting is defined by BUSMESSAGE_ARG_COUNT_MAX in message_bus.c.
// Note that a %f argument takes twice the amount of memory, hence it counts
// as 2 arguments.
void messageBusPutTimedf(busMessageTopic_e topic, busMessageId_t id, timeMs_t expiresIn, const char *fmt, ...) __attribute__ ((format (printf, 4, 5)));

void messageBusClear(uint8_t topic, uint8_t id);
void messageBusClearTopic(uint8_t topic);

int messageBusGetCount(void);
int messageBusFormatMessageAt(char *buf, size_t bufsize, int messageIndex);
