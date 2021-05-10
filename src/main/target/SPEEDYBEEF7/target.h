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

#define TARGET_BOARD_IDENTIFIER "SBF7"
#define USBD_PRODUCT_STRING  "SpeedyBeeF7"

#define LED0                    PC14

#define BEEPER                  PC13
#define BEEPER_INVERTED

// UARTs
#define USB_IO
#define USE_VCP
#define VBUS_SENSING_PIN        PC15
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       6

// GYRO -- SPI1
#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_MPU_DATA_READY_SIGNAL
#define USE_EXTI

#define USE_IMU_ICM20689
#define IMU_ICM20689_ALIGN          CW0_DEG

#define GYRO_INT_EXTI               PC4
#define ICM20689_CS_PIN             PA4
#define ICM20689_SPI_BUS            BUS_SPI1

#define SPI1_SCK_PIN                PA5
#define SPI1_MISO_PIN               PA6
#define SPI1_MOSI_PIN               PA7

// OSD -- SPI2
#define USE_SPI_DEVICE_2
#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          PB12

#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// Onboard 32MB Flash
#define USE_SPI_DEVICE_3

#define M25P16_CS_PIN           PC0
#define M25P16_SPI_BUS          BUS_SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

// Onboard BMP280 Baro --- I2C1
#define USE_I2C
#define USE_BARO
#define USE_BARO_BMP280
#define BARO_I2C_BUS            BUS_I2C1

#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

// External I2C Pads -- I2C2
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2
#define PITOT_I2C_BUS           BUS_I2C2

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C2

// ADC
#define USE_ADC
#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC1
#define ADC_CHANNEL_3_PIN           PC3

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

// PINIO to disable BT
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                  PA15
#define PINIO1_FLAGS                PINIO_FLAGS_INVERTED

// FC Default Config
#define DEFAULT_FEATURES            (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_VBAT | FEATURE_OSD | FEATURE_BLACKBOX)
#define USE_DSHOT
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define SERIALRX_UART           SERIAL_PORT_USART2
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define MAX_PWM_OUTPUT_PORTS        6

#define CURRENT_METER_SCALE   102

// Used Pins
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
