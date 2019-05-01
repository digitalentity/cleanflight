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

#include "common/crc.h"
#include "common/log.h"

#include "io/flock_serial.h"
#include "io/serial.h"

#include "flight/flock.h"

// FLOCK-SERIAL - See https://github.com/ravenLRS/Flock

#define FLOCK_SERIAL_FUNCTION FUNCTION_FLOCK
#define FLOCK_SERIAL_SYNC_BYTE_1 0xff
#define FLOCK_SERIAL_SYNC_BYTE_2 'F'
#define FLOCK_SERIAL_MAX_PACKET_SIZE (FLOCK_MAX_PAYLOAD_SIZE + 1) // 1 byte for cmd

typedef enum {
    FLOCK_PROTOCOL_STATUS_NONE,
    FLOCK_PROTOCOL_STATUS_SYNC,
    FLOCK_PROTOCOL_STATUS_SIZE,
    FLOCK_PROTOCOL_STATUS_DATA,
    FLOCK_PROTOCOL_STATUS_CRC,
} flockProtocolStatus_e;

typedef struct flockSerialDevice_s {
    serialPort_t *port;
    uint8_t buf[FLOCK_SERIAL_MAX_PACKET_SIZE];
    uint8_t bufPos;
    uint8_t crc;
    uint8_t packetSize;
    uint8_t status;
} flockSerialDevice_t;

static flockSerialDevice_t device;

static void flockSerialResetProtocolState(void)
{
    device.bufPos = 0;
    device.crc = 0;
    device.status = FLOCK_PROTOCOL_STATUS_NONE;
}

bool flockSerialInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FLOCK_SERIAL_FUNCTION);
    if (portConfig != NULL) {
        device.port = openSerialPort(portConfig->identifier, FLOCK_SERIAL_FUNCTION, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
        flockSerialResetProtocolState();
        return true;
    }
    return false;
}

int flockSerialRead(uint8_t *cmd, void *buf, size_t bufsize)
{
    if (!device.port) {
        return -1;
    }

    unsigned payloadSize;
    while (serialRxBytesWaiting(device.port)) {
        uint8_t c = serialRead(device.port);
        switch ((flockProtocolStatus_e)device.status) {
        case FLOCK_PROTOCOL_STATUS_NONE:
            if (c == FLOCK_SERIAL_SYNC_BYTE_1) {
                device.status = FLOCK_PROTOCOL_STATUS_SYNC;
            }
            break;
        case FLOCK_PROTOCOL_STATUS_SYNC:
            if (c == FLOCK_SERIAL_SYNC_BYTE_2) {
                device.status = FLOCK_PROTOCOL_STATUS_SIZE;
                break;
            }
            flockSerialResetProtocolState();
            break;
        case FLOCK_PROTOCOL_STATUS_SIZE:
            if (c <= FLOCK_SERIAL_MAX_PACKET_SIZE) {
                device.crc = crc8_dvb_s2(device.crc, c);
                device.packetSize = c;
                device.status = FLOCK_PROTOCOL_STATUS_DATA;
                break;
            }
            flockSerialResetProtocolState();
            break;
        case FLOCK_PROTOCOL_STATUS_DATA:
            device.crc = crc8_dvb_s2(device.crc, c);
            device.buf[device.bufPos++] = c;
            if (device.bufPos == device.packetSize) {
                device.status = FLOCK_PROTOCOL_STATUS_CRC;
            }
            break;
        case FLOCK_PROTOCOL_STATUS_CRC:
            if (c != device.crc) {
                LOG_W(FLOCK, "Invalid CRC, got %02x != %02x", c, device.crc);
                flockSerialResetProtocolState();
                break;
            }
            payloadSize = device.bufPos - 1;
            if (bufsize < payloadSize) {
                flockSerialResetProtocolState();
                return -1;
            }
            *cmd = device.buf[0];
            memcpy(buf, &device.buf[1], payloadSize);
            flockSerialResetProtocolState();
            return payloadSize;
        }
    }
    return -1;
}

int flockSerialWrite(uint8_t cmd, const void *buf, size_t size)
{
    if (!device.port) {
        return -1;
    }

    if (size > FLOCK_MAX_PAYLOAD_SIZE) {
        return -2;
    }

    serialWrite(device.port, FLOCK_SERIAL_SYNC_BYTE_1);
    serialWrite(device.port, FLOCK_SERIAL_SYNC_BYTE_2);
    int packetSize = 1;
    uint8_t crc;
    if (buf) {
        packetSize += size;
    } else {
        size = 0;
    }

    crc = crc8_dvb_s2(0, packetSize);
    crc = crc8_dvb_s2(crc, cmd);
    crc = crc8_dvb_s2_update(crc, buf, size);

    serialWrite(device.port, packetSize);
    serialWrite(device.port, cmd);
    serialWriteBuf(device.port, buf, size);
    serialWrite(device.port, crc);
    return size;
}
