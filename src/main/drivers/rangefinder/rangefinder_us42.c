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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_US42)

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_us42.h"

#include "build/debug.h"

// GY-US42(v2) Ultrasonic Range Sensor
#define US42_MAX_RANGE_CM 400 // vcc=3.3v -> 500cm; vcc=5v -> 700cm; Ardupilot recommends a maximum of 400cm
#define US42_DETECTION_CONE_DECIDEGREES 250
#define US42_DETECTION_CONE_EXTENDED_DECIDEGREES 300
#define US42_MIN_PROBE_INTERVAL 50

#define US42_I2C_ADDRESS 0x70
#define US42_I2C_REGISTRY_BASE 0x00
#define US42_I2C_REGISTRY_PROBE 0x51
#define US42_I2C_REGISTRY_STATUS_OK 0x00
#define US42_I2C_REGISTRY_DISTANCE_HIGH 0x00
#define US42_I2C_REGISTRY_DISTANCE_LOW 0x01

volatile int32_t us42MeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static int16_t minimumReadingIntervalMs = US42_MIN_PROBE_INTERVAL;
static uint32_t timeOfLastMeasurementMs;
uint8_t nullProbeCommandValue[0];
static bool isUs42Responding = false;

static void us42Init(rangefinderDev_t *rangefinder)
{
    busWriteBuf(rangefinder->busDev, US42_I2C_REGISTRY_PROBE, nullProbeCommandValue, 0);
}

void us42Update(rangefinderDev_t *rangefinder)
{
    uint8_t data[2];
    isUs42Responding = busReadBuf(rangefinder->busDev, US42_I2C_REGISTRY_PROBE, data, 2);

    if (isUs42Responding) {
        us42MeasurementCm = (int32_t)data[0] << 8 | (int32_t)data[1];

        if (us42MeasurementCm > US42_MAX_RANGE_CM) {
             us42MeasurementCm = RANGEFINDER_OUT_OF_RANGE;
        }    

    } else {
        us42MeasurementCm = RANGEFINDER_HARDWARE_FAILURE;
    }    

    const timeMs_t timeNowMs = millis();
    if (timeNowMs > timeOfLastMeasurementMs + minimumReadingIntervalMs) {
        // measurement repeat interval should be greater than minimumReadingIntervalMs
        // to avoid interference between connective measurements.
        timeOfLastMeasurementMs = timeNowMs;
        busWriteBuf(rangefinder->busDev, US42_I2C_REGISTRY_PROBE, nullProbeCommandValue, 0);
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t us42GetDistance(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    return us42MeasurementCm;
}

static bool deviceDetect(busDevice_t * busDev)
{
    for (int retry = 0; retry < 5; retry++) {
        uint8_t inquiryResult;

        delay(150);

        bool ack = busRead(busDev, US42_I2C_REGISTRY_BASE, &inquiryResult);
        if (ack && inquiryResult == US42_I2C_REGISTRY_STATUS_OK) {
            return true;
        }
    };

    return false;
}

bool us42Detect(rangefinderDev_t *rangefinder)
{
    rangefinder->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_US42, 0, OWNER_RANGEFINDER);
    if (rangefinder->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(rangefinder->busDev)) {
        busDeviceDeInit(rangefinder->busDev);
        return false;
    }

    rangefinder->delayMs = RANGEFINDER_US42_TASK_PERIOD_MS;
    rangefinder->maxRangeCm = US42_MAX_RANGE_CM;
    rangefinder->detectionConeDeciDegrees = US42_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = US42_DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinder->init = &us42Init;
    rangefinder->update = &us42Update;
    rangefinder->read = &us42GetDistance;

    return true;
}
#endif