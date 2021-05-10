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

#define TARGET_BOARD_IDENTIFIER "SP4E"

#ifndef SPRACINGF4EVO_REV
#define SPRACINGF4EVO_REV 2
#endif

#define USBD_PRODUCT_STRING     "SP Racing F4 EVO"

#define LED0                    PA0

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define GYRO_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW0_DEG

#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW0_DEG

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_MPU9250
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1

#define USE_VCP

#define USE_UART_INVERTER

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define SERIAL_PORT_COUNT       6

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3
#define INVERTER_PIN_UART2_RX   PB2

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define UART4_TX_PIN            PC10
#define UART4_RX_PIN            PC11

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define USE_I2C
#define USE_I2C_DEVICE_1

#if (SPRACINGF4EVO_REV >= 2)
    #define I2C1_SCL            PB8
    #define I2C1_SDA            PB9
#else
    #define I2C1_SCL            PB6
    #define I2C1_SDA            PB7
#endif

#define USE_SPI
#define USE_SPI_DEVICE_1 // MPU
#define USE_SPI_DEVICE_2 // SDCard
#define USE_SPI_DEVICE_3 // External

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15 // NC
#define SPI3_SCK_PIN            PB3  // NC
#define SPI3_MISO_PIN           PB4  // NC
#define SPI3_MOSI_PIN           PB5  // NC

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PC14
#define SDCARD_SPI_BUS          BUS_SPI2
#define SDCARD_CS_PIN           SPI2_NSS_PIN

#define MPU6500_CS_PIN                      SPI1_NSS_PIN
#define MPU6500_SPI_BUS                     BUS_SPI1

#define MPU9250_CS_PIN                      SPI1_NSS_PIN
#define MPU9250_SPI_BUS                     BUS_SPI1

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC0
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// PC4 - NC - Free for ADC12_IN14 / VTX CS
// PC5 - NC - Free for ADC12_IN15 / VTX Enable / OSD VSYNC

//#define USE_OSD
//#define USE_MAX7456
//#define USE_OSD_OVER_MSP_DISPLAYPORT

#define USE_LED_STRIP
#define WS2811_PIN                      PA1

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_LED_STRIP)
#define SERIALRX_UART           SERIAL_PORT_USART2
#define TELEMETRY_UART          SERIAL_PORT_USART5
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS        8
