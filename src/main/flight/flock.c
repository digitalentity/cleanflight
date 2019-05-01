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

#include <string.h>

#include "platform.h"

#include "common/bitarray.h"
#include "common/log.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/flock.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/flock_serial.h"
#include "io/gps.h"

#include "navigation/navigation.h"

#define FLOCK_TIMEOUT_MS 15000
#define FLOCK_REQUEST_INFO_INTERVAL_MS 1000

#define FLOCK_MAX_UPDATES_PER_ITERATION 5

#define FLOCK_ALTITUDE_OFFSET 1000
// Reverse flock +1000 meters offset, convert to cm
#define FLOCK_ALT_TO_ALT(alt) ((((int32_t)alt) - FLOCK_ALTITUDE_OFFSET) * 100)
// Convert to m, apply +1000 offset
#define ALT_TO_FLOCK_ALT(alt) ((alt / 100) + FLOCK_ALTITUDE_OFFSET)

#define FLOCK_REL_ALT_MIN (-(1 << (FLOCK_VERTICAL_DISTANCE_BITS - 1)))
#define FLOCK_REL_ALT_MAX ((1 << (FLOCK_VERTICAL_DISTANCE_BITS - 1)) - 1)

#define FLOCK_BIRD_IS_VALID(b) ((b)->lastUpdate > 0)

typedef enum {
    FLOCK_TRANSPORT_NONE,
    FLOCK_TRANSPORT_SERIAL,
    FLOCK_TRANSPORT_MSP
} flockTransport_e;

typedef enum {
    FLOCK_CMD_DEVICE_INFO = 0x01,
    FLOCK_CMD_GET_HOST_INFO = 0x04,
    FLOCK_CMD_SET_HOST_INFO = 0x05,
    FLOCK_CMD_SET_HOST_POSVEL = 0x06,

    FLOCK_CMD_RECEIVED_BIRD_INFO = 0x80,
    FLOCK_CMD_RECEIVED_BIRD_POSVEL = 0x81,
} flockCommand_e;

// Flock over-the-wire data types

typedef struct flockMessageDeviceInfo_s {
    uint8_t version;
    char deviceName[12];
    uint8_t deviceVersion[3];
    flockAddress_t addr;
    uint16_t posvelIntervalMs;
    uint8_t radioType;
    uint64_t radioMinFreq;
    uint64_t radioMaxFreq;
    uint64_t radioDefaultFreq;
} __attribute__((packed)) flockMessageDeviceInfo_t;

typedef struct flockMessageBirdInfo_s {
    uint8_t type;
    uint16_t flags;
    char name[17];
} __attribute__((packed)) flockMessageBirdInfo_t;

// See https://github.com/RavenLRS/Flock/blob/master/docs/FLOCK.md#0x06---set-host-posvel
typedef struct flockMessagePosvel_s {
    int32_t latitude;
    int32_t longitude;
    uint16_t altitude;
    uint16_t groundSpeed;
    int16_t verticalSpeed;
    uint16_t heading;
} __attribute__((packed)) flockMessagePosvel_t;

typedef struct flockMessageRemoteBirdInfo_s {
    flockAddress_t from;
    flockMessageBirdInfo_t info;
} __attribute__((packed)) flockMessageRemoteBirdInfo_t;


typedef struct flockMessageRemotePosvel_s {
    flockAddress_t from;
    flockMessagePosvel_t posvel;
} __attribute__((packed)) flockMessageRemotePosvel_t;

typedef struct flockDevice_s {
    uint8_t flockVersion;
    uint8_t transport; // flockTransport_e
    timeMs_t updateInterval; // Update interval for coordinates, used only for serial
    timeMs_t nextUpdate; // Next scheduled request to device, used only for serial
} flockDevice_t;

typedef struct flock_s {
    flockDevice_t dev;
    flockBird_t birds[FLOCK_MAX_BIRDS];
    uint16_t maxDistance; // meters
    int count;
    BITARRAY_DECLARE(dirty, FLOCK_MAX_BIRDS);
} flock_t;


#ifdef USE_FLOCK

#ifndef USE_NAV
#error "FLOCK requires NAV"
#endif

static flock_t flock;

static int flockReceivedCommand(uint8_t cmd, const void *data, size_t dataSize);

void flockInit(void)
{
    memset(flock.birds, 0, sizeof(flock.birds));
    flock.count = 0;

    if (flockSerialInit()) {
        flock.dev.transport = FLOCK_TRANSPORT_SERIAL;
        flock.dev.nextUpdate = 0;
        LOG_D(FLOCK, "Serial transport enabled");
    }
}

static void flockUpdateRemoveStaleEntries(timeMs_t currentTimeMs)
{
    if (currentTimeMs <= FLOCK_TIMEOUT_MS) {
        return;
    }

    timeMs_t threshold = currentTimeMs - FLOCK_TIMEOUT_MS;
    for (unsigned ii = 0; ii < ARRAYLEN(flock.birds); ii++) {
        flockBird_t *bird = &flock.birds[ii];
        if (FLOCK_BIRD_IS_VALID(bird) &&
            bird->lastUpdate < threshold) {

            bird->lastUpdate = 0;
            flock.count--;
        }
    }
}

static bool flockUpdateBirdRelativePosition(flockBird_t *bird)
{
    if (bird->stateFlags & FLOCK_BIRD_FLAG_HAS_POSVEL) {
        fpVector3_t pos;
        gpsLocation_t llh = {
            .lat = bird->posvel.lat,
            .lon = bird->posvel.lon,
            .alt = FLOCK_ALT_TO_ALT(bird->posvel.alt),
        };
        if (geoConvertGeodeticToLocalOrigin(&pos, &llh, GEO_ALT_ABSOLUTE)) {
            navDestinationPath_t path;
            if (navCalculatePathToDestination(&path, &pos)) {
                bird->rel.groundDistance = constrain(path.distance / 100, 0, UINT16_MAX);
                bird->rel.verticalDistance = constrain((pos.z - getEstimatedActualPosition(Z)) / 100, FLOCK_REL_ALT_MIN, FLOCK_REL_ALT_MAX);
                bird->rel.bearing = path.bearing / (100 / 5);
                return true;
            }
        }
    }
    return false;
}

static void flockUpdateDirty(void)
{
    int p = BITARRAY_FIND_FIRST_SET(flock.dirty, 0);
    if (p < 0) {
        // We've passed over all entries. Update max values and start again.
        flock.maxDistance = 0;
        for (unsigned ii = 0; ii < ARRAYLEN(flock.birds); ii++) {
            flockBird_t *b = &flock.birds[p];
            if (FLOCK_BIRD_IS_VALID(b) && (b->stateFlags & FLOCK_BIRD_FLAG_HAS_REL_POS)) {
                flock.maxDistance = MAX(flock.maxDistance, b->rel.groundDistance);
            }
        }
        BITARRAY_SET_ALL(flock.dirty);
        p = 0;
    }
    flockBird_t *bird = &flock.birds[p];
    if (FLOCK_BIRD_IS_VALID(bird)) {
        if (flockUpdateBirdRelativePosition(bird)) {
            bird->stateFlags |= FLOCK_BIRD_FLAG_HAS_REL_POS;
        } else {
            bird->stateFlags &= ~FLOCK_BIRD_FLAG_HAS_REL_POS;
        }
    }
    bitArrayClr(flock.dirty, p);
}

// quantizes (-90, 90) to [0, 2^32 - 1]
static uint32_t flockQuantizeLatitude(int32_t latitude)
{
    float mult = INT32_MAX / (float)(90 * GPS_DEGREES_DIVIDER);
    uint32_t shifted = latitude + 90 * GPS_DEGREES_DIVIDER;
    return (uint32_t)roundf(shifted * mult);
}

// quantizes (-180, 180) to [0, 2^32 - 1]
static uint32_t flockQuantizeLongitude(int32_t longitude)
{
    float mult = INT32_MAX / (float)(180 * GPS_DEGREES_DIVIDER);
    // Be careful to not overflow, since +360 * GPS_DEGREES_DIVIDER
    // doesn't fit in an int32_t
    uint32_t shifted = longitude;
    shifted += 180 * GPS_DEGREES_DIVIDER;
    return (uint32_t)roundf(shifted * mult);
}

// normalizes and quantizes an angle in deg * 10 to [0, 2^16 - 1]
static uint16_t flockQuantizeAngle(int16_t angle)
{
    // Normalize to [0, 3600)
    while (angle < 0) {
        angle += 3600;
    }
    while (angle >= 3600) {
        angle -= 3600;
    }
    float mult = UINT16_MAX / (float)(360 * 10);
    return roundf(angle * mult);
}

static int32_t flockDequantizeLatitude(uint32_t latitude)
{
    float mult = (float)(90 * GPS_DEGREES_DIVIDER) / INT32_MAX;
    return (uint32_t)roundf(latitude * mult) - 90 * GPS_DEGREES_DIVIDER;
}

static int32_t flockDequantizeLongitude(uint32_t longitude)
{
    float mult = (float)(180 * GPS_DEGREES_DIVIDER) / INT32_MAX;
    return (uint32_t)roundf(longitude * mult) - 180 * GPS_DEGREES_DIVIDER;
}

static int16_t flockDequantizeAngle(uint16_t angle)
{
    float mult = (float)(360 * 10) / UINT16_MAX;
    return roundf(angle * mult);
}

static void flockUpdateSerialPosvel(void)
{
    if (!STATE(GPS_FIX_3D)) {
        return;
    }

    flockMessagePosvel_t posvel = {
        .latitude = flockQuantizeLatitude(gpsSol.llh.lat),
        .longitude = flockQuantizeLongitude(gpsSol.llh.lon),
        .altitude = ALT_TO_FLOCK_ALT(gpsSol.llh.alt),
        .groundSpeed = gpsSol.groundSpeed / 10,
        .verticalSpeed = getEstimatedActualVelocity(Z) / 10,
        .heading = flockQuantizeAngle(attitude.values.yaw),
    };
    LOG_D(FLOCK, "Send posvel");
    flockSerialWrite(FLOCK_CMD_SET_HOST_POSVEL, &posvel, sizeof(posvel));
}

static void flockUpdateSerial(timeUs_t currentTimeUs)
{
    uint8_t cmd;
    uint8_t buf[FLOCK_MAX_PAYLOAD_SIZE];
    int serialPayloadSize;
    if ((serialPayloadSize = flockSerialRead(&cmd, buf, sizeof(buf))) >= 0) {
        flockReceivedCommand(cmd, buf, serialPayloadSize);
    }
    timeMs_t now = currentTimeUs / 1000;
    if (now >= flock.dev.nextUpdate) {
        if (flock.dev.flockVersion == 0) {
            // No device info yet
            LOG_D(FLOCK, "Request device info");
            flockSerialWrite(FLOCK_CMD_DEVICE_INFO, NULL, 0);
            flock.dev.nextUpdate = now + FLOCK_REQUEST_INFO_INTERVAL_MS;
        } else {
            // Send a posvel update
            flockUpdateSerialPosvel();
            flock.dev.nextUpdate = now + flock.dev.updateInterval;
        }
    }
}

void flockUpdate(timeUs_t currentTimeUs)
{
    if (flock.dev.transport == FLOCK_TRANSPORT_SERIAL) {
        flockUpdateSerial(currentTimeUs);
    }
    // Shortcircuit to avoid doing useless work
    if (flock.count > 0) {
        flockUpdateRemoveStaleEntries(currentTimeUs / 1000);
        for (int ii = 0; ii < FLOCK_MAX_UPDATES_PER_ITERATION; ii++) {
            flockUpdateDirty();
        }
    }
}

static flockBird_t *flockGetBird(const flockAddress_t *addr)
{
    for (unsigned ii = 0; ii < ARRAYLEN(flock.birds); ii++) {
        flockBird_t *bird = &flock.birds[ii];
        if (FLOCK_BIRD_IS_VALID(bird) &&
            memcmp(&bird->info.addr, addr, sizeof(*addr)) == 0) {

            return bird;
        }
    }
    return NULL;
}

static flockBird_t *flockGetFreeSlot(void)
{
    for (unsigned ii = 0; ii < ARRAYLEN(flock.birds); ii++) {
        if (!FLOCK_BIRD_IS_VALID(&flock.birds[ii])) {
            return &flock.birds[ii];
        }
    }
    return NULL;
}

static flockBird_t *flockGetOrCreateBird(const flockAddress_t *addr)
{
    flockBird_t *bird = flockGetBird(addr);
    if (!bird) {
        bird = flockGetFreeSlot();
        if (!bird) {
            return NULL;
        }
        bird->stateFlags = 0;
        flock.count++;
    }
    return bird;
}

static flockBirdType_e flockBirdType(void)
{
    switch (mixerConfig()->platformType) {
        case PLATFORM_MULTIROTOR:
            return FLOCK_BIRD_TYPE_MULTIROTOR;
        case PLATFORM_AIRPLANE:
            return FLOCK_BIRD_TYPE_AIRPLANE;
        case PLATFORM_HELICOPTER:
            return FLOCK_BIRD_TYPE_HELICOPTER;
        case PLATFORM_TRICOPTER:
            return FLOCK_BIRD_TYPE_MULTIROTOR;
        case PLATFORM_ROVER:
            return FLOCK_BIRD_TYPE_ROVER;
        case PLATFORM_BOAT:
            return FLOCK_BIRD_TYPE_BOAT;
        case PLATFORM_OTHER:
            break;
    }
    return FLOCK_BIRD_TYPE_UNKNOWN;
}

static flockBirdFlags_t flockBirdFlags(void)
{
    // TODO: Check if we actually have autonomous modes enabled
    flockBirdFlags_t flags = FLOCK_BIRD_HAS_AUTOPILOT;
#if defined(USE_OSD)
    // For now we only notify the pilot via OSD
    flags |= FLOCK_BIRD_CAN_NOTIFY_PILOT;
#endif
    return flags;
}

static void flockSendHostInfoSerial(void)
{
    flockMessageBirdInfo_t info = {
        .type = flockBirdType(),
        .flags = flockBirdFlags(),
    };
    strncpy(info.name, systemConfig()->name, sizeof(info.name));
    info.name[sizeof(info.name) - 1] = '\0';
    flockSerialWrite(FLOCK_CMD_SET_HOST_INFO, &info, sizeof(info));
}

#define FLOCK_ENSURE_DATASIZE_TYPE(t) do { \
    if (dataSize < sizeof(t)) { \
        LOG_D(FLOCK, "Invalid size for cmd %02x %d < %d %s", cmd, dataSize, sizeof(t), #t); \
        return FLOCK_EINVALIDINPUT; \
    } \
} while(0)

static int flockReceivedCommand(uint8_t cmd, const void *data, size_t dataSize)
{
    switch (cmd) {
        case FLOCK_CMD_DEVICE_INFO:
        {
            FLOCK_ENSURE_DATASIZE_TYPE(flockMessageDeviceInfo_t);

            const flockMessageDeviceInfo_t *dev = data;
            LOG_D(FLOCK, "Got Flock device info, version %u", dev->version);
            flock.dev.flockVersion = dev->version;
            flock.dev.updateInterval = dev->posvelIntervalMs;
            flock.dev.nextUpdate = 0;
            if (flock.dev.transport == FLOCK_TRANSPORT_SERIAL) {
                flockSendHostInfoSerial();
            }
            break;
        }
        case FLOCK_CMD_RECEIVED_BIRD_INFO:
        {
            FLOCK_ENSURE_DATASIZE_TYPE(flockMessageRemoteBirdInfo_t);

            const flockMessageRemoteBirdInfo_t *msg = data;
            flockBird_t *bird = flockGetOrCreateBird(&msg->from);
            if (!bird) {
                return FLOCK_ENOMEM;
            }
            bird->info.birdType = msg->info.type;
            bird->info.flags = msg->info.flags;
            strncpy(bird->info.name, msg->info.name, sizeof(bird->info.name));
            bird->info.name[sizeof(bird->info.name) - 1] = '\0';
            bird->stateFlags |= FLOCK_BIRD_FLAG_HAS_INFO;
            bird->lastUpdate = millis();
            break;
        }
        case FLOCK_CMD_RECEIVED_BIRD_POSVEL:
        {
            FLOCK_ENSURE_DATASIZE_TYPE(flockMessageRemotePosvel_t);

            const flockMessageRemotePosvel_t *msg = data;
            flockBird_t *bird = flockGetOrCreateBird(&msg->from);
            if (!bird) {
                return FLOCK_ENOMEM;
            }
            bird->posvel.lat = flockDequantizeLatitude(msg->posvel.latitude);
            bird->posvel.lon = flockDequantizeLongitude(msg->posvel.longitude);
            bird->posvel.alt = msg->posvel.altitude;
            bird->posvel.groundSpeed = msg->posvel.groundSpeed;
            bird->posvel.verticalSpeed = msg->posvel.verticalSpeed;
            bird->posvel.heading = flockDequantizeAngle(msg->posvel.heading);
            bird->stateFlags |= FLOCK_BIRD_FLAG_HAS_POSVEL;
            bird->lastUpdate = millis();
            break;
        }
        default:
            LOG_W(FLOCK, "Unhandled message %u", cmd);
            return FLOCK_EINVALIDINPUT;

    }
    return 0;
}

static int flockReceived(const void *data, size_t dataSize)
{
    if (dataSize > 0) {
        const uint8_t *ptr = data;
        uint8_t cmd = *ptr;
        return flockReceivedCommand(cmd, ptr + 1, dataSize - 1);
    }
    return FLOCK_EINVALIDINPUT;
}

int flockReceivedMSP(const void *data, size_t dataSize, void *buf, size_t bufSize)
{
    UNUSED(buf);
    UNUSED(bufSize);

    if (flock.dev.transport == FLOCK_TRANSPORT_NONE) {
        flock.dev.transport = FLOCK_TRANSPORT_MSP;
        LOG_D(FLOCK, "MSP transport enabled");
    }
    return flockReceived(data, dataSize);
}

int flockGetCount(void)
{
    return flock.count;
}

void flockIteratorInit(flockIterator_t *it)
{
    it->pos = -1;
}

bool flockIteratorNext(flockIterator_t *it)
{
    while (it->pos < (int)ARRAYLEN(flock.birds)) {
        it->pos++;
        if (FLOCK_BIRD_IS_VALID(&flock.birds[it->pos])) {
            return true;
        }
    }
    return false;
}

const flockBird_t *flockIteratorGet(flockIterator_t *it)
{
    if (it->pos >= 0 && it->pos < (int)ARRAYLEN(flock.birds) &&
        FLOCK_BIRD_IS_VALID(&flock.birds[it->pos])) {

        return &flock.birds[it->pos];
    }
    return NULL;
}

#endif
