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
#if defined(KAKUTEF4V2)
#   define TARGET_BOARD_IDENTIFIER "KTV2"
#   define USBD_PRODUCT_STRING "KakuteF4-V2"
#else
#   define TARGET_BOARD_IDENTIFIER "KTV1"
#   define USBD_PRODUCT_STRING "KakuteF4-V1"
#endif

#define LED0                    PB5
#define LED1                    PB4
#define LED2                    PB6

#define BEEPER                  PC9
#define BEEPER_INVERTED

#define USE_EXTI
#define GYRO_INT_EXTI            PC5
#define USE_MPU_DATA_READY_SIGNAL

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW270_DEG
#define MPU6500_CS_PIN          PC4
#define MPU6500_SPI_BUS         BUS_SPI1


#ifdef KAKUTEF4V2
#   define USE_I2C
#   define USE_I2C_DEVICE_1
#   define I2C1_SCL                PB8        // SCL pad
#   define I2C1_SDA                PB9        // SDA pad

#   define USE_MAG
#   define MAG_I2C_BUS             BUS_I2C1
#   define USE_MAG_HMC5883
#   define USE_MAG_QMC5883
#   define USE_MAG_MAG3110
#   define USE_MAG_IST8310
#   define USE_MAG_IST8308
#   define USE_MAG_LIS3MDL

#   define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS             BUS_I2C1

#   define USE_BARO
#   define BARO_I2C_BUS            BUS_I2C1
#   define USE_BARO_MS5611
#   define USE_BARO_BMP280
#else // V1 does not have I2C exposed, common_post.h will pull in USE_*_MSP
#   define USE_BARO
#   define USE_MAG
#endif

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PB14

#define M25P16_CS_PIN           PB3
#define M25P16_SPI_BUS          BUS_SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN        PA8
#define VBUS_SENSING_ENABLED

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define INVERTER_PIN_UART3_RX   PB15

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#ifdef KAKUTEF4V2
#   define USE_UART4
#   define UART4_RX_PIN            PA1
#   define UART4_TX_PIN            PA0

#   define USE_UART5
#   define UART5_RX_PIN            PD2
#   define UART5_TX_PIN            NONE

// #   define USE_SOFTSERIAL1
#   define SERIAL_PORT_COUNT 6
#else
// #   define USE_SOFTSERIAL1
// #   define USE_SOFTSERIAL2
#   define SERIAL_PORT_COUNT 4
#endif

#define USE_SPI

// SPI1: Connected to ICM gyro
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

// SPI3: Connected to flash memory
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_ADC
#define ADC_CHANNEL_1_PIN           PC3
#define ADC_CHANNEL_2_PIN           PC2
#define ADC_CHANNEL_3_PIN           PC1

#define USE_LED_STRIP
#define WS2811_PIN                      PC8
#define WS2811_DMA_HANDLER_IDENTIFER    DMA2_ST4_HANDLER
#define WS2811_DMA_STREAM               DMA2_Stream4
#define WS2811_DMA_CHANNEL              DMA_Channel_7

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_TELEMETRY | FEATURE_OSD)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3
#define TELEMETRY_UART          SERIAL_PORT_USART1

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD        (BIT(2))

#define USE_DSHOT
#define USE_SERIALSHOT
#define USE_ESC_SENSOR

#ifdef KAKUTEF4V2
#   define MAX_PWM_OUTPUT_PORTS       4
#else
#   define MAX_PWM_OUTPUT_PORTS       6
#endif
