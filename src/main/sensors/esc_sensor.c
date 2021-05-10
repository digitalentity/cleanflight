/*
 * This file is part of INAV Project.
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
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/crc.h"

#include "config/feature.h"
#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "flight/mixer.h"
#include "drivers/pwm_output.h"
#include "sensors/esc_sensor.h"
#include "io/serial.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"


#if defined(USE_ESC_SENSOR)

#define ESC_BOOTTIME_MS         5000
#define ESC_REQUEST_TIMEOUT_MS  50
#define ESC_SENSOR_BAUDRATE     115200
#define TELEMETRY_FRAME_SIZE    10

typedef enum {
    ESC_SENSOR_WAIT_STARTUP = 0,
    ESC_SENSOR_READY = 1,
    ESC_SENSOR_WAITING = 2
} escSensorState_t;

typedef enum {
    ESC_SENSOR_FRAME_PENDING,
    ESC_SENSOR_FRAME_COMPLETE,
    ESC_SENSOR_FRAME_FAILED
} escSensorFrameStatus_t;

static serialPort_t *   escSensorPort = NULL;
static escSensorState_t escSensorState = ESC_SENSOR_WAIT_STARTUP;
static timeMs_t         escTriggerTimeMs;
static int              escSensorMotor;
static uint8_t          telemetryBuffer[TELEMETRY_FRAME_SIZE];
static int              bufferPosition = 0;
static escSensorData_t  escSensorData[MAX_SUPPORTED_MOTORS];
static escSensorData_t  escSensorDataCombined;
static bool             escSensorDataNeedsUpdate;

PG_REGISTER_WITH_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 1);
PG_RESET_TEMPLATE(escSensorConfig_t, escSensorConfig,
    .currentOffset = 0, // UNUSED
    .listenOnly = SETTING_ESC_SENSOR_LISTEN_ONLY_DEFAULT,
);

static int getTelemetryMotorCount(void)
{
    if (escSensorConfig()->listenOnly) {
        return 1;
    }
    else {
        return getMotorCount();
    }
}

static void escSensorSelectNextMotor(void)
{
    if (escSensorConfig()->listenOnly) {
        escSensorMotor = 0;
    }
    else {
        escSensorMotor = (escSensorMotor + 1) % getTelemetryMotorCount();
    }
}

static void escSensorIncreaseDataAge(void)
{
    if (escSensorData[escSensorMotor].dataAge < ESC_DATA_INVALID) {
        escSensorData[escSensorMotor].dataAge++;
        escSensorDataNeedsUpdate = true;
    }
}

static bool escSensorDecodeFrame(void)
{
    // Receive bytes
    while (serialRxBytesWaiting(escSensorPort) > 0) {
        uint8_t c = serialRead(escSensorPort);

        if (bufferPosition < TELEMETRY_FRAME_SIZE) {
            telemetryBuffer[bufferPosition++] = c;
        }
    }

    // Decode frame
    if (bufferPosition >= TELEMETRY_FRAME_SIZE) {
        uint8_t checksum = crc8_update(0, telemetryBuffer, TELEMETRY_FRAME_SIZE - 1);
        if (checksum == telemetryBuffer[TELEMETRY_FRAME_SIZE - 1]) {
            escSensorData[escSensorMotor].dataAge       = 0;
            escSensorData[escSensorMotor].temperature   = telemetryBuffer[0];
            escSensorData[escSensorMotor].voltage       = ((uint16_t)telemetryBuffer[1]) << 8 | telemetryBuffer[2];
            escSensorData[escSensorMotor].current       = ((uint16_t)telemetryBuffer[3]) << 8 | telemetryBuffer[4];
            escSensorData[escSensorMotor].rpm           = computeRpm(((uint16_t)telemetryBuffer[7]) << 8 | telemetryBuffer[8]);
            escSensorDataNeedsUpdate = true;

            if (escSensorMotor < 4) {
                DEBUG_SET(DEBUG_ERPM, escSensorMotor, escSensorData[escSensorMotor].rpm);
            }

            return ESC_SENSOR_FRAME_COMPLETE;
        }
        else {
            // CRC error
            return ESC_SENSOR_FRAME_FAILED;
        }
    }

    return ESC_SENSOR_FRAME_PENDING;
}

uint32_t computeRpm(int16_t erpm) {
    return lrintf((float)erpm * ERPM_PER_LSB / (motorConfig()->motorPoleCount / 2));
}

escSensorData_t NOINLINE * getEscTelemetry(uint8_t esc)
{
    return &escSensorData[esc];
}

escSensorData_t * escSensorGetData(void)
{
    if (!escSensorPort) {
        return NULL;
    }

    if (escSensorDataNeedsUpdate) {
        escSensorDataCombined.dataAge = 0;
        escSensorDataCombined.temperature = 0;
        escSensorDataCombined.voltage = 0;
        escSensorDataCombined.current = 0;
        escSensorDataCombined.rpm = 0;

        // Combine data only from active sensors, ignore stale sensors
        int usedEscSensorCount = 0;
        for (int i = 0; i < getTelemetryMotorCount(); i++) {
            if (escSensorData[i].dataAge < ESC_DATA_INVALID) {
                usedEscSensorCount++;
                escSensorDataCombined.dataAge = MAX(escSensorDataCombined.dataAge, escSensorData[i].dataAge);
                escSensorDataCombined.temperature = MAX(escSensorDataCombined.temperature, escSensorData[i].temperature);
                escSensorDataCombined.voltage += escSensorData[i].voltage;
                escSensorDataCombined.current += escSensorData[i].current;
                escSensorDataCombined.rpm += escSensorData[i].rpm;
            }
        }

        // Make sure we calculate our sensor values only from non-stale values
        if (usedEscSensorCount) {
            escSensorDataCombined.current = (uint32_t)escSensorDataCombined.current * getTelemetryMotorCount() / usedEscSensorCount + escSensorConfig()->currentOffset;
            escSensorDataCombined.voltage = (uint32_t)escSensorDataCombined.voltage / usedEscSensorCount;
            escSensorDataCombined.rpm = (float)escSensorDataCombined.rpm / usedEscSensorCount;
        }
        else {
            escSensorDataCombined.dataAge = ESC_DATA_INVALID;
        }

        escSensorDataNeedsUpdate = false;
    }

    // Return NULL if sensors are old
    if (escSensorDataCombined.dataAge >= ESC_DATA_INVALID) {
        return NULL;
    }
    else {
        return &escSensorDataCombined;
    }
}

bool escSensorInitialize(void)
{
    escSensorDataNeedsUpdate = true;
    escSensorPort = NULL;

    // Fail immediately if motor output are disabled or motor outputs are not configured
    if (!feature(FEATURE_PWM_OUTPUT_ENABLE) || getMotorCount() == 0) {
        return false;
    }

    // FUNCTION_ESCSERIAL is shared between SERIALSHOT and ESC_SENSOR telemetry
    // They are mutually exclusive
    serialPortConfig_t * portConfig = findSerialPortConfig(FUNCTION_ESCSERIAL);
    if (!portConfig) {
        return false;
    }

    escSensorPort = openSerialPort(portConfig->identifier, FUNCTION_ESCSERIAL, NULL, NULL, ESC_SENSOR_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED | SERIAL_UNIDIR);
    if (!escSensorPort) {
        return false;
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        escSensorData[i].dataAge = ESC_DATA_INVALID;
    }

    ENABLE_STATE(ESC_SENSOR_ENABLED);

    return true;
}

void escSensorUpdate(timeUs_t currentTimeUs)
{
    if (!escSensorPort) {
        return;
    }

    const timeMs_t currentTimeMs = currentTimeUs / 1000;

    switch (escSensorState) {
        case ESC_SENSOR_WAIT_STARTUP:
            if (currentTimeMs > ESC_BOOTTIME_MS) {
                escSensorMotor = 0;
                escSensorState = ESC_SENSOR_READY;
            }
            break;

        case ESC_SENSOR_READY:
            if (!escSensorConfig()->listenOnly) {
                pwmRequestMotorTelemetry(escSensorMotor);
            }
            bufferPosition = 0;
            escTriggerTimeMs = currentTimeMs;
            escSensorState = ESC_SENSOR_WAITING;
            break;

        case ESC_SENSOR_WAITING:
            if ((currentTimeMs - escTriggerTimeMs) >= ESC_REQUEST_TIMEOUT_MS) {
                // Timed out. Select next motor and move on
                escSensorIncreaseDataAge();
                escSensorSelectNextMotor();
                escSensorState = ESC_SENSOR_READY;
            }
            else {
                // Receive serial data and decode frame
                escSensorFrameStatus_t status = escSensorDecodeFrame();

                switch (status) {
                    case ESC_SENSOR_FRAME_COMPLETE:
                        escSensorSelectNextMotor();
                        escSensorState = ESC_SENSOR_READY;
                        break;

                    case ESC_SENSOR_FRAME_FAILED:
                        escSensorIncreaseDataAge();
                        escSensorSelectNextMotor();
                        escSensorState = ESC_SENSOR_READY;
                        break;

                    case ESC_SENSOR_FRAME_PENDING:
                    default:
                        break;
                }
            }
            break;

    }

}

#endif
