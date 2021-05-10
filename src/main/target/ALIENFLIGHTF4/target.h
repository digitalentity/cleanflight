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
#define TARGET_BOARD_IDENTIFIER "AFF4"

#define USE_HARDWARE_REVISION_DETECTION
#define HW_PIN                  PC13
#define USE_BRUSHED_ESC_AUTODETECT

#define USBD_PRODUCT_STRING "AlienFlight F4"

#define LED0                    PC12
#define LED1                    PD2

#define BEEPER                  PC13
#define BEEPER_INVERTED

#define INVERTER_PIN_USART2     PC15

// MPU interrupt
#define USE_EXTI
#define GYRO_INT_EXTI            PC14
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_BUS         BUS_SPI1

#define MPU9250_CS_PIN          SPI1_NSS_PIN
#define MPU9250_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW270_DEG
#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW270_DEG


#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_MPU9250
#define USE_MAG_HMC5883
#define USE_MAG_MAG3110
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_MS5611
#define USE_BARO_BMP280

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PB11
#define SDCARD_SPI_BUS          BUS_SPI2
#define SDCARD_CS_PIN           PB10

//#define M25P16_CS_PIN        SPI2_NSS_PIN
//#define M25P16_SPI_BUS       BUS_SPI2

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2 //inverter

//#define USE_UART3
//#define UART3_RX_PIN            PB11
//#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define SERIAL_PORT_COUNT       4

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_PULLUP
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_ADC
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC_CHANNEL_1_PIN           PC0
#define ADC_CHANNEL_2_PIN           PC1
#define ADC_CHANNEL_3_PIN           PC4
#define ADC_CHANNEL_4_PIN           PC5

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

//#define BOARD_HAS_VOLTAGE_DIVIDER
//#define BOARD_HAS_CURRENT_SENSOR

// LED strip configuration using RC1 pin.
#define USE_LED_STRIP
// LED Strip can run off Pin 41 (PA8) of the ESC outputs.
#define WS2811_PIN                      PA8

#define USE_SPEKTRUM_BIND
// USART2, PA3
#define BIND_PIN                UART2_RX_PIN

#define HARDWARE_BIND_PLUG
// Hardware bind plug at PB2 (Pin 28)
#define BINDPLUG_PIN            PB2

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_MOTOR_STOP | FEATURE_BLACKBOX)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART2
#define RX_CHANNELS_TAER

#define TELEMETRY_UART          SERIAL_PORT_USART1

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    12

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
