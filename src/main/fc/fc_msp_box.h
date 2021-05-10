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

#include "fc/rc_modes.h"

#include "io/piniobox.h"

typedef struct box_s {
    const uint8_t boxId;            // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      // permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
} box_t;

const box_t *findBoxByActiveBoxId(uint8_t activeBoxId);
const box_t *findBoxByPermanentId(uint8_t permanentId);

struct boxBitmask_s;
void packBoxModeFlags(struct boxBitmask_s * mspBoxModeFlags);
uint16_t packSensorStatus(void);
struct sbuf_s;
bool serializeBoxNamesReply(struct sbuf_s *dst);
void serializeBoxReply(struct sbuf_s *dst);
void initActiveBoxIds(void);
