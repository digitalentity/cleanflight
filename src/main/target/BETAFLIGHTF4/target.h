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

#define TARGET_BOARD_IDENTIFIER "BFF4"
#define USBD_PRODUCT_STRING     "BetaFlightF4"

//#define USE_ESC_SENSOR

#define LED0                    PB5

#define BEEPER                  PB4
#define BEEPER_INVERTED

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS		BUS_SPI1

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG


// MPU6000 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_BARO
#define USE_BARO_BMP280

#define BMP280_SPI_BUS		BUS_SPI2
#define BMP280_CS_PIN           PB3

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS		BUS_SPI2
#define MAX7456_CS_PIN      PB12
//#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz // XXX
//#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST) // XXX

#define M25P16_CS_PIN           PA15
#define M25P16_SPI_BUS          BUS_SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16

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

//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART2, USART3, USART6, SOFTSERIAL1, SOFTSERIAL2

//#define USE_ESCSERIAL // XXX
//#define ESCSERIAL_TIMER_TX_PIN  PB8 // (Hardware=0, PPM)

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            NONE
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

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10 // PB10, UART3_TX
#define I2C2_SDA                PB11 // PB11, UART3_RX
//#define I2C_DEVICE              (I2CDEV_2)

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2

#define USE_BARO
#define BARO_I2C_BUS             BUS_I2C2
#define USE_BARO_BMP085
#define USE_BARO_BMP280
#define USE_BARO_MS5611

#define USE_PITOT_ADC
#define PITOT_I2C_BUS           BUS_I2C2

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C2
#define USE_RANGEFINDER_HCSR04_I2C

#define USE_ADC
#define ADC_CHANNEL_1_PIN		    PC1
#define ADC_CHANNEL_2_PIN		    PC2
#define CURRENT_METER_ADC_CHANNEL	ADC_CHN_1
#define VBAT_ADC_CHANNEL		    ADC_CHN_2

#define USE_LED_STRIP
#define WS2811_PIN                      PB6

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART6

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_OSD )

#define USE_SPEKTRUM_BIND
// USART3,
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)))
#define TARGET_IO_PORTD BIT(2)

#define MAX_PWM_OUTPUT_PORTS 4

#define PCA9685_I2C_BUS         BUS_I2C2

#define USE_DSHOT
#define USE_ESC_SENSOR
