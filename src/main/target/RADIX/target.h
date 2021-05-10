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

#define TARGET_BOARD_IDENTIFIER "RADIX"

#define USBD_PRODUCT_STRING     "BrainFPV RADIX"

#define LED0                    PA4
#define LED0_INVERTED

#define BEEPER                  NONE

#define USE_EXTI
#define BMI160_SPI_BUS          BUS_SPI3
#define BMI160_CS_PIN           PB4
#define GYRO_EXTI_PIN           PC13

#define USE_IMU_BMI160
#define IMU_BMI160_ALIGN        CW0_DEG

// #define USE_MAG
// #define MAG_I2C_BUS             BUS_I2C1
// #define USE_MAG_HMC5883
// #define USE_MAG_QMC5883
// #define USE_MAG_IST8310
// #define USE_MAG_MAG3110
// #define USE_MAG_LIS3MDL

#define USE_BARO
#define USE_BARO_BMP280
#define BMP280_SPI_BUS          BUS_SPI3
#define BMP280_CS_PIN           PB8

//#define USE_PITOT_MS4525
//#define PITOT_I2C_BUS           BUS_I2C2

#define USE_VCP
#define VBUS_SENSING_PIN        PA9
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART3
#define UART3_RX_PIN            PC5
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3, USART4, USART6

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

// #define USE_I2C
// #define USE_I2C_DEVICE_1
// #define I2C1_SCL                PB8
// #define I2C1_SDA                PB9

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC3
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
// #define USE_SDCARD
// #define USE_SDCARD_SPI

// #define SDCARD_DETECT_INVERTED
// #define SDCARD_DETECT_PIN               PB13
// #define SDCARD_SPI_BUS                  BUS_SPI2
// #define SDCARD_CS_PIN                   PB15

#define SENSORS_SET (SENSOR_ACC|SENSOR_BARO)

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PB14

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
