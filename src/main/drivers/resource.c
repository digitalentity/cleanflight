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

#include "drivers/resource.h"

const char * const ownerNames[OWNER_TOTAL_COUNT] = {
    "FREE", "PWM/IO", "PWM", "PPM", "MOTOR", "SERVO", "SOFTSERIAL", "ADC", "SERIAL", "DEBUG", "TIMER",
    "RANGEFINDER", "SYSTEM", "SPI", "I2C", "SDCARD", "FLASH", "USB", "BEEPER", "OSD",
    "BARO", "MPU", "INVERTER", "LED STRIP", "LED", "RECEIVER", "TRANSMITTER",
    "NRF24", "VTX", "SPI_PREINIT", "COMPASS", "TEMPERATURE", "1-WIRE", "AIRSPEED", "OLED DISPLAY",
    "PINIO", "CAMERA_CONTROL"
};

const char * const resourceNames[RESOURCE_TOTAL_COUNT] = {
    "", // NONE
    "IN", "OUT", "IN / OUT",
    "TIMER",
    "UART TX", "UART RX", "UART TX/RX",
    "EXTI",
    "SCL", "SDA",
    "SCK", "MOSI", "MISO", "CS",
    "CH1", "CH2", "CH3", "CH4",
    "CE"
};
