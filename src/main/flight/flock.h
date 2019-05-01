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
 * along with INAV. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/time.h"

#if !defined(FLOCK_MAX_NAME_LENGTH)
#define FLOCK_MAX_NAME_LENGTH 17
#endif

#if !defined(FLOCK_MAX_BIRDS)
#define FLOCK_MAX_BIRDS 8
#endif

#define FLOCK_MAX_PAYLOAD_SIZE 64

#define FLOCK_ADDR_SIZE 6
#define FLOCK_VERTICAL_DISTANCE_BITS 13

#define FLOCK_EINVALIDINPUT -1
#define FLOCK_ENOMEM -2

typedef enum {
    FLOCK_BIRD_TYPE_UNKNOWN = 0,
    FLOCK_BIRD_TYPE_AIRPLANE = 1,
    FLOCK_BIRD_TYPE_BOAT = 2,
    FLOCK_BIRD_TYPE_FLYING_WING = 3,
    FLOCK_BIRD_TYPE_HELICOPTER = 4,
    FLOCK_BIRD_TYPE_MULTIROTOR = 5,
    FLOCK_BIRD_TYPE_ROVER = 6,
} flockBirdType_e;

typedef enum {
    FLOCK_BIRD_CAN_NOTIFY_PILOT = 1 << 0,
    FLOCK_BIRD_HAS_AUTOPILOT = 1 << 2,
    FLOCK_BIRD_HAS_COLLISION_AVOIDANCE = 1 << 3,
    FLOCK_BIRD_IS_MANNED = 1 << 4,
} flockBirdFlags_t;

typedef struct flockAdress_s {
    uint8_t addr[FLOCK_ADDR_SIZE];
} flockAddress_t;

typedef enum {
    FLOCK_BIRD_FLAG_HAS_INFO = 1 << 0,
    FLOCK_BIRD_FLAG_HAS_POSVEL = 1 << 1,
    FLOCK_BIRD_FLAG_HAS_REL_POS = 1 << 2,
} flockBirdStateFlags_t;

typedef struct flockBirdInfo_s
{
    flockAddress_t addr;
    char name[FLOCK_MAX_NAME_LENGTH];
    uint8_t birdType; // from flockBirdType_e
    uint8_t flags; // from flockBirdFlags_t
} __attribute__((packed)) flockBirdInfo_t;

typedef struct flockBirdPosvel_s
{
    int32_t lat;                // deg * 10`000`000
    int32_t lon;                // deg * 10`000`000
    uint16_t alt;               // absolute altitude in m with +1000m offset (e.g. 1000 is 0 MSL)
    uint16_t groundSpeed;       // m/s * 10
    int8_t verticalSpeed;       // m/s * 10
    int16_t heading;            // deg * 10
} __attribute__((packed)) flockBirdPosvel_t;

typedef struct flockBird_s
{
    flockBirdInfo_t info;
    flockBirdPosvel_t posvel;
    timeMs_t lastUpdate;
    uint8_t stateFlags; // from flockBirdStateFlags_t
    struct {
        uint16_t groundDistance; //meters [0, 65535]
        int16_t verticalDistance : FLOCK_VERTICAL_DISTANCE_BITS; // meters [-4096, 4096]
        uint16_t bearing : 11; // direction to aircraft in 0.2deg steps [0, 360]
    } rel;
} __attribute__((packed)) flockBird_t;

typedef struct flockIterator_s {
    int pos;
} flockIterator_t;

void flockInit(void);
void flockUpdate(timeUs_t currentTimeUs);
int flockGetCount(void);

int flockReceivedMSP(const void *data, size_t dataSize, void *buf, size_t bufSize);

void flockIteratorInit(flockIterator_t *it);
bool flockIteratorNext(flockIterator_t *it);
const flockBird_t *flockIteratorGet(flockIterator_t *it);
