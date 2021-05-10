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
#define TARGET_BOARD_IDENTIFIER "BJF4"

#define USBD_PRODUCT_STRING     "BlueJayF4"

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PB2

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_EXTI

#define LED0                    PB6
#define LED1                    PB5
#define LED2                    PB4

#define BEEPER                  PC1
#define BEEPER_OPT              PB7
#define BEEPER_INVERTED

// MPU6500 interrupt
#define USE_MPU_DATA_READY_SIGNAL
#define GYRO_INT_EXTI            PC5
#define MPU6500_CS_PIN          PC4
#define MPU6500_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW0_DEG

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

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PD2
#define SDCARD_SPI_BUS          BUS_SPI3
#define SDCARD_CS_PIN           PA15

#define M25P16_CS_PIN           PB7
#define M25P16_SPI_BUS          BUS_SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USE_VCP
//#define VBUS_SENSING_PIN PA8
//#define VBUS_SENSING_ENABLED

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
//#define INVERTER_PIN_UART1_TX     PC9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define INVERTER_PIN_UART6_RX   PB15

#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT       5

#define SOFTSERIAL_1_RX_PIN     PB1
#define SOFTSERIAL_1_TX_PIN     PB0


#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_PULLUP

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC3
#define VBAT_ADC_CHANNEL                ADC_CHN_1

#define USE_LED_STRIP
#define WS2811_PIN                      PB1

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PB11

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
