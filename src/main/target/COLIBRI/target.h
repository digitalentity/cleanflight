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

#define TARGET_BOARD_IDENTIFIER "COLI"

#ifdef QUANTON
#define USBD_PRODUCT_STRING "Quanton"
#else
#define USBD_PRODUCT_STRING "Colibri"
#endif
#ifdef OPBL
#define USBD_SERIALNUMBER_STRING "0x8020000"
#endif

#define PLL_M                   16
#define PLL_N                   336

#define LED0                    PC14
#define LED1                    PC13

#define BEEPER                  PC5

#define MPU6000_CS_PIN          PC4
#define MPU6000_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6000

// MPU6000 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PC0
#define USE_MPU_DATA_READY_SIGNAL

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C3
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C3
#define BNO055_I2C_BUS          BUS_I2C3

#ifdef QUANTON
#define IMU_MPU6000_ALIGN       CW90_DEG
#else
#define IMU_MPU6000_ALIGN       CW270_DEG_FLIP
#endif

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C3
#define USE_BARO_MS5611

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_BUS          BUS_SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
#define VBUS_SENSING_PIN        PA9

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2
#define INVERTER_PIN_UART2_RX   PB2 // PB2 used as inverter select GPIO

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN PC11
#define UART4_TX_PIN PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       6 //VCP, UART1, UART2, UART3, UART4, UART5

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define USE_I2C
#define USE_I2C_DEVICE_3
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C3
#define USE_RANGEFINDER_HCSR04
#define RANGEFINDER_HCSR04_TRIGGER_PIN      PB8
#define RANGEFINDER_HCSR04_ECHO_PIN         PB9

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define MAX_PWM_OUTPUT_PORTS    15
