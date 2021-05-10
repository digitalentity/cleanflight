/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SP7D"
#define USE_TARGET_CONFIG

#ifndef SPRACINGF7DUAL_REV
#define SPRACINGF7DUAL_REV 2
#endif

#define USBD_PRODUCT_STRING     "SP Racing F7 DUAL"

#define USE_DUAL_GYRO

#define LED0_PIN                PC4

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define GYRO_2_EXTI_PIN         PC14

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS

#define USE_IMU_MPU6500
#define IMU_1_ALIGN                 IMU_MPU6500_1_ALIGN
#define IMU_2_ALIGN                 IMU_MPU6500_2_ALIGN

#if (SPRACINGF7DUAL_REV >= 2)
#define IMU_MPU6500_1_ALIGN         CW0_DEG
#define IMU_MPU6500_2_ALIGN         CW270_DEG
#else
#define IMU_MPU6500_1_ALIGN         CW180_DEG
#define IMU_MPU6500_2_ALIGN         CW270_DEG
#endif

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define BNO055_I2C_BUS          BUS_I2C1

#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1

#define USE_VCP
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

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define UART4_TX_PIN            PC10
#define UART4_RX_PIN            PC11

#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#if (SPRACINGF7DUAL_REV <= 1)
    #define TARGET_USART_CONFIG
#endif

// TODO
// #define USE_ESCSERIAL
// #define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_SPI

#define USE_SPI_DEVICE_1 // 2xMPU
#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2 // MAX7456
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          SPI2_NSS_PIN

#define USE_SDCARD
#define USE_SDCARD_SPI

#define USE_SPI_DEVICE_3 // SDCARD
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define SDCARD_SPI_BUS          BUS_SPI3
#define SDCARD_CS_PIN           PC3

#define GYRO_1_CS_PIN                       SPI1_NSS_PIN
#define GYRO_1_SPI_INSTANCE                 BUS_SPI1
#define GYRO_2_CS_PIN                       PB2
#define GYRO_2_SPI_INSTANCE                 BUS_SPI1
#define MPU6500_1_CS_PIN                    GYRO_1_CS_PIN
#define MPU6500_1_SPI_BUS                   GYRO_1_SPI_INSTANCE
#define MPU6500_2_CS_PIN                    GYRO_2_CS_PIN
#define MPU6500_2_SPI_BUS                   GYRO_2_SPI_INSTANCE

#define USE_ADC
// It's possible to use ADC1 or ADC3 on this target, same pins.
//#define ADC_INSTANCE                        ADC1
//#define ADC1_DMA_STREAM                     DMA2_Stream0

// Using ADC3 frees up DMA2_Stream0 for SPI1_RX (not necessarily, SPI1_RX has DMA2_Stream2 as well)
#define ADC_CHANNEL_1_PIN                   PC1
#define ADC_CHANNEL_2_PIN                   PC2
#define ADC_CHANNEL_3_PIN                   PC0


//#define ADC_INSTANCE                        ADC3
//#define ADC3_DMA_STREAM                     DMA2_Stream0
#define VBAT_ADC_CHANNEL                    ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL           ADC_CHN_2
#define RSSI_ADC_CHANNEL                    ADC_CHN_3

#define CURRENT_METER_SCALE         300

#define USE_OSD

#define USE_LED_STRIP
#define WS2811_PIN                      PA1

//#define RX_CHANNELS_TAER
#define DEFAULT_FEATURES                    (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP| FEATURE_VBAT | FEATURE_CURRENT_METER| FEATURE_BLACKBOX)

#define GPS_UART                            SERIAL_PORT_USART3

#define SERIALRX_UART                       SERIAL_PORT_USART2
#define SERIALRX_PROVIDER                   SERIALRX_SBUS

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS    8

#define USE_DSHOT
#define USE_ESC_SENSOR
