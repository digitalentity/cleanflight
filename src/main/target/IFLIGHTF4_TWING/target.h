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

#define TARGET_BOARD_IDENTIFIER "IF4T"
#define USBD_PRODUCT_STRING     "IFLIGHTF4_TWING"

#define LED0                    PB5

#define BEEPER                  PB4
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS     // Don't use common busdev descriptors for IMU

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_DUAL_GYRO

#define USE_IMU_MPU6500
#define IMU_0_ALIGN             CW0_DEG
#define IMU_1_ALIGN             CW0_DEG

#define MPU6500_0_CS_PIN        PA4
#define MPU6500_0_SPI_BUS       BUS_SPI1
#define MPU6500_0_EXTI_PIN      PC4

#define MPU6500_1_CS_PIN        PC3
#define MPU6500_1_SPI_BUS       BUS_SPI1
#define MPU6500_1_EXTI_PIN      PA8

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

// *************** I2C/Baro/Mag *********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_DPS310

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1

#define PITOT_I2C_BUS           BUS_I2C1

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C1

#define PCA9685_I2C_BUS         BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS		    BUS_SPI2
#define MAX7456_CS_PIN          PB12 

// *************** FLASH **************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define M25P16_CS_PIN           PA15
#define M25P16_SPI_BUS          BUS_SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// *************** UART *****************************
#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2
// PC13 used as inverter select GPIO for UART2
#define INVERTER_PIN_UART2_RX   PC13

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       5 // VCP, USART1, USART2, USART3, USART6

// *************** ADC *****************************
#define USE_ADC
#define ADC_CHANNEL_1_PIN		  PC1
#define ADC_CHANNEL_2_PIN		  PC2
#define CURRENT_METER_ADC_CHANNEL ADC_CHN_1
#define VBAT_ADC_CHANNEL		  ADC_CHN_2

// *************** OTHERS *****************************
#define USE_LED_STRIP
#define WS2811_PIN              PB6

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_BLACKBOX)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB         (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC         (0xffff & ~(BIT(15)|BIT(14)))
#define TARGET_IO_PORTD         BIT(2)

#define MAX_PWM_OUTPUT_PORTS 4
