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

#include <platform.h>

#include "drivers/bus_i2c.h"
#include "pitotmeter.h"
#include "drivers/time.h"

#include "common/utils.h"
#include "common/maths.h"

// eagle_tree, Standard address 0x4D
#define EAGLE_TREE_ADDR                 0x4D    //the define itself not currently used, but should be used in common_hardware.c and maybe target.c where 0x4D is used

#define EAGLE_TREE_RAW_VALUE_MAX        4048.0f //Max value we can expect from sensor
#define EAGLE_TREE_RAW_VALUE_MIN        896.0f  //Min value we can expect from the sensor, anything less than this is negative pressure and useless for our needs
#define EAGLE_TREE_PA_VALUE_MAX         3920.0f //Max value the sensor can read in Pascals
#define EAGLE_TREE_PA_VALUE_MIN         0.0f    //Min value the sensor can read in Pascals, anything less than this is negative pressure and useless for our needs

#define STANDARD_DAY_TEMP_KELVIN        288.15f //SEE: https://en.wikipedia.org/wiki/Standard_day

static uint16_t eagleTreeUp;  // static result of pressure measurement
static uint8_t rxbuf[4];

static inline float pitotEagleTreeRawToPa(uint16_t rawValue) {
  return (((constrainf((float)rawValue,EAGLE_TREE_RAW_VALUE_MIN,EAGLE_TREE_RAW_VALUE_MAX) - EAGLE_TREE_RAW_VALUE_MIN) * (EAGLE_TREE_PA_VALUE_MAX - EAGLE_TREE_PA_VALUE_MIN)) / (EAGLE_TREE_RAW_VALUE_MAX - EAGLE_TREE_RAW_VALUE_MIN) ) + EAGLE_TREE_PA_VALUE_MIN;
}

static void eagleTreeStart(pitotDev_t *pitot) {
    busReadBuf(pitot->busDev, 0x00, rxbuf, 2);
}

static void eagleTreeRead(pitotDev_t *pitot) {
    if (busReadBuf(pitot->busDev, 0x00, rxbuf, 2)) {
        eagleTreeUp = (rxbuf[0] << 8) | (rxbuf[1] << 0);
    }
}

static void eagleTreeCalculate(pitotDev_t *pitot, float *pressure, float *temperature) {
    UNUSED(pitot);
    if (pressure)
        *pressure = (float)eagleTreeUp;          // Pa
    if (temperature)
        *temperature = STANDARD_DAY_TEMP_KELVIN; // Temperature at standard sea level (288.15 K) SEE: https://en.wikipedia.org/wiki/Standard_day
}

bool pitotEagleTreeDetect(pitotDev_t *pitot) {
    pitot->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_PITOT_EAGLE_TREE, 0, OWNER_AIRSPEED);

    if (pitot->busDev == NULL) {
        return false;
    }

    bool ack = false;

    // Read twice to fix:
    // Sending a start-stop condition without any transitions on the SCL line (no clock pulses in between) creates a
    // communication error for the next communication, even if the next start condition is correct and the clock pulse is applied.
    // An additional start condition must be sent, which results in restoration of proper communication.
    ack = busReadBuf(pitot->busDev, 0x00, rxbuf, 2);
    ack = busReadBuf(pitot->busDev, 0x00, rxbuf, 2);
    if (!ack) {
        return false;
    }

    pitot->delay = 10000;
    pitot->start = eagleTreeStart;
    pitot->get = eagleTreeRead;
    pitot->calculate = eagleTreeCalculate;
    eagleTreeRead(pitot);
    return true;
}
