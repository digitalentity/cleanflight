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

#define RESOURCE_INDEX(x) (x + 1)

typedef enum {
    OWNER_FREE = 0,
    OWNER_PWMIO,
    OWNER_PWMINPUT,
    OWNER_PPMINPUT,
    OWNER_MOTOR,
    OWNER_SERVO,
    OWNER_SOFTSERIAL,
    OWNER_ADC,
    OWNER_SERIAL,
    OWNER_PINDEBUG,
    OWNER_TIMER,
    OWNER_RANGEFINDER,
    OWNER_SYSTEM,
    OWNER_SPI,
    OWNER_I2C,
    OWNER_SDCARD,
    OWNER_FLASH,
    OWNER_USB,
    OWNER_BEEPER,
    OWNER_OSD,
    OWNER_BARO,
    OWNER_MPU,
    OWNER_INVERTER,
    OWNER_LED_STRIP,
    OWNER_LED,
    OWNER_RX,
    OWNER_TX,
    OWNER_RX_SPI,
    OWNER_VTX,
    OWNER_SPI_PREINIT,
    OWNER_COMPASS,
    OWNER_TEMPERATURE,
    OWNER_1WIRE,
    OWNER_AIRSPEED,
    OWNER_OLED_DISPLAY,
    OWNER_PINIO,
    OWNER_CAMERA_CONTROL,
    OWNER_TOTAL_COUNT
} resourceOwner_e;

extern const char * const ownerNames[OWNER_TOTAL_COUNT];

// Currently TIMER should be shared resource (softserial dualtimer and timerqueue needs to allocate timer channel, but pin can be used for other function)
// with mode switching (shared serial ports, ...) this will need some improvement
typedef enum {
    RESOURCE_NONE       = 0,
    RESOURCE_INPUT, RESOURCE_OUTPUT, RESOURCE_IO,
    RESOURCE_TIMER,
    RESOURCE_UART_TX, RESOURCE_UART_RX, RESOURCE_UART_TXRX,
    RESOURCE_EXTI,
    RESOURCE_I2C_SCL, RESOURCE_I2C_SDA,
    RESOURCE_SPI_SCK, RESOURCE_SPI_MOSI, RESOURCE_SPI_MISO, RESOURCE_SPI_CS,
    RESOURCE_ADC_CH1, RESOURCE_ADC_CH2, RESOURCE_ADC_CH3, RESOURCE_ADC_CH4,
    RESOURCE_RX_CE,
    RESOURCE_TOTAL_COUNT
} resourceType_e;

extern const char * const resourceNames[RESOURCE_TOTAL_COUNT];
