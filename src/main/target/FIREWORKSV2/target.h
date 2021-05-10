/*
 * This file is part of INAV.
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

#pragma once

#if defined(OMNIBUSF4V6)
#define TARGET_BOARD_IDENTIFIER "OBV6"
#else 
#define TARGET_BOARD_IDENTIFIER "FWX2"
#endif

#if defined(OMNIBUSF4V6)
#define USBD_PRODUCT_STRING "OmnibusF4 V6"
#else
#define USBD_PRODUCT_STRING "OMNIBUS F4 FWX V2"
#endif

#define USE_DSHOT
#define USE_ESC_SENSOR

// Status LED
#define LED0                    PA8

// Beeper
#define BEEPER                  PB4
#define BEEPER_INVERTED

// I2C
#define USE_I2C
#if defined(OMNIBUSF4V6)
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8 // SCL PIN,alt MST8
#define I2C1_SDA                PB9 // SDA PIN,alt MST7
#define DEFAULT_I2C_BUS         BUS_I2C1
#else
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2_SHARES_UART3
#endif

#define USE_EXTI
#define GYRO_INT_EXTI            PC8
// #define USE_MPU_DATA_READY_SIGNAL        // Not connected on FireworksV2

#define USE_DUAL_GYRO
#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS     // Don't use common busdev descriptors for IMU
#define USE_IMU_MPU6500
#define USE_IMU_MPU6000

#if defined(OMNIBUSF4V6)
#   define IMU_1_CS_PIN            PA4
#   define IMU_1_SPI_BUS           BUS_SPI1
#   define IMU_1_ALIGN             CW180_DEG
#   define IMU_2_CS_PIN            PC14
#   define IMU_2_SPI_BUS           BUS_SPI1
#   define IMU_2_ALIGN             CW0_DEG
#else
    // IMU_1 is verified to work on OBF4V6 and Omnibus Fireworks board
#   define IMU_1_CS_PIN            PA4
#   define IMU_1_SPI_BUS           BUS_SPI1
#   define IMU_1_ALIGN             CW0_DEG_FLIP
    // IMU_2 is sketchy and was not verified on actual hardware
#   define IMU_2_CS_PIN            PD2
#   define IMU_2_SPI_BUS           BUS_SPI3
#   define IMU_2_ALIGN             CW180_DEG
#endif

#define USE_MAG
#if defined(OMNIBUSF4V6)
#define MAG_I2C_BUS             BUS_I2C1
#else
#define MAG_I2C_BUS             BUS_I2C2
#endif
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#if defined(OMNIBUSF4V6)
#define TEMPERATURE_I2C_BUS     BUS_I2C1
#else
#define TEMPERATURE_I2C_BUS     BUS_I2C2
#endif

#define USE_BARO
#define USE_BARO_BMP280
#define BMP280_SPI_BUS          BUS_SPI3
#define BMP280_CS_PIN           PB3
#if defined(OMNIBUSF4V6)
#define BARO_I2C_BUS            BUS_I2C1
#endif

#if defined(OMNIBUSF4V6)
#define PITOT_I2C_BUS           BUS_I2C1
#else
#define PITOT_I2C_BUS           BUS_I2C2
#endif

#define USE_RANGEFINDER
#if defined(OMNIBUSF4V6)
#define RANGEFINDER_I2C_BUS     BUS_I2C1
#else
#define RANGEFINDER_I2C_BUS     BUS_I2C2
#endif
#define USE_RANGEFINDER_HCSR04_I2C

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            NONE

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     NONE
#define SOFTSERIAL_1_TX_PIN     PA9     // Clash with UART1_TX, needed for S.Port

#define SERIAL_PORT_COUNT       7       // VCP, USART1, USART2, USART3, USART4, USART6, SOFTSERIAL1

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PA15

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_BUS          BUS_SPI2
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PA0

#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO)

#define USE_LED_STRIP
#define WS2811_PIN                      PB6
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST0_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream0
#define WS2811_DMA_CHANNEL              DMA_Channel_2

#define DEFAULT_FEATURES                (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_OSD | FEATURE_CURRENT_METER | FEATURE_SOFTSERIAL | FEATURE_TELEMETRY)
#define DEFAULT_RX_TYPE                 RX_TYPE_SERIAL
#define SERIALRX_PROVIDER               SERIALRX_SBUS
#define DISABLE_RX_PWM_FEATURE

#define TELEMETRY_UART                  SERIAL_PORT_SOFTSERIAL1
#define SERIALRX_UART                   SERIAL_PORT_USART1
#define SMARTAUDIO_UART                 SERIAL_PORT_USART4

//Default values for OmnibusF4V6,calib values for FireworksV2
#if !defined(OMNIBUSF4V6)
#define CURRENT_METER_SCALE             175
#define CURRENT_METER_OFFSET            326
#endif

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    8
#define TARGET_MOTOR_COUNT      6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#if defined(OMNIBUSF4V6)
#define PCA9685_I2C_BUS         BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1
#else
#define PCA9685_I2C_BUS         BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2
#endif
