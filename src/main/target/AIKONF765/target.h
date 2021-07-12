/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS
#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "A765"
#define USBD_PRODUCT_STRING     "AIKONF765"

#define LED0                    PE3
#define LED1                    PE2

#define BEEPER                  PE13
#define BEEPER_INVERTED

// *************** SPI1 Gyro & ACC *******************
#define USE_SPI

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_DUAL_GYRO

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW90_DEG_FLIP
#define MPU6000_SPI_BUS         BUS_SPI3
#define MPU6000_CS_PIN          PA15
#define MPU6000_EXTI_PIN        PD7

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW270_DEG_FLIP
#define MPU6500_SPI_BUS         BUS_SPI2
#define MPU6500_CS_PIN          PB12
#define MPU6500_EXTI_PIN        PB2

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

// *************** I2C /Baro/Mag *********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1
#define PITOT_I2C_BUS           BUS_I2C1

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C1

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

// *************** UART *****************************
#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6

#define USE_UART3
#define UART3_TX_PIN            PD8
#define UART3_RX_PIN            PD9

#define USE_UART4
#define UART4_TX_PIN            PD1
#define UART4_RX_PIN            PD0

#define USE_UART5
#define UART5_TX_PIN            PB9
#define UART5_RX_PIN            PB8
#define UART5_AF                7

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define USE_UART7
#define UART7_TX_PIN            PE8
#define UART7_RX_PIN            PE7

#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            PE0


#define SERIAL_PORT_COUNT       9

// *************** SPI1 OSD ***********************
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI1
#define MAX7456_CS_PIN          PA4

// *************** SPI3 SD BLACKBOX*******************
#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_SDIO_DMA         DMA_TAG(2,3,4)
#define SDCARD_SDIO_4BIT

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1

#define ADC_CHANNEL_1_PIN           PC3
#define ADC_CHANNEL_2_PIN           PC2
#define ADC_CHANNEL_3_PIN           PC5
#define ADC_CHANNEL_4_PIN           PC4
#define ADC_CHANNEL_5_PIN           PC1 // VBAT2
#define ADC_CHANNEL_6_PIN           PC0 // CU2

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3
#define AIRSPEED_ADC_CHANNEL        ADC_CHN_4

// *************** PINIO ***************************
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                  PE9  // 9V switching

// *************** LEDSTRIP ************************
#define USE_LED_STRIP
#define WS2811_PIN                  PE14

// **************** MISC ***************************

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PD11

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_FEATURES            (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define MAX_PWM_OUTPUT_PORTS        10
#define USE_DSHOT
#define USE_ESC_SENSOR
#define USE_SERIALSHOT
