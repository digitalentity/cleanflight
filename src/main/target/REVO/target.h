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

#define TARGET_BOARD_IDENTIFIER "REVO"

#define USBD_PRODUCT_STRING     "Revolution"

// Use target-specific MAG hardware descriptors (don't use common_hardware.h)
#define USE_TARGET_MAG_HARDWARE_DESCRIPTORS

#define LED0                    PB5
#define LED1                    PB4

#define BEEPER                  PB4

#define USE_DSHOT
#define USE_ESC_SENSOR
#define USE_SERIALSHOT

// MPU6000 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI           PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW270_DEG
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1

#define USE_MAG
#define USE_DUAL_MAG
#define MAG_I2C_BUS_EXT         BUS_I2C2
#define MAG_I2C_BUS_INT         BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_MS5611

#define PITOT_I2C_BUS           BUS_I2C2

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C2

#define M25P16_CS_PIN           PB3
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
// PC0 used as inverter select GPIO
#define INVERTER_PIN_UART1_RX   PC0

#define USE_UART3
#define I2C_DEVICE_2_SHARES_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

// #define USE_SOFTSERIAL1
// #define SOFTSERIAL_1_RX_PIN     PC8
// #define SOFTSERIAL_1_TX_PIN     PC9

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PA0
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3


#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define USE_LED_STRIP
#define WS2811_PIN                      PA1

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PB11 //UART3_RX_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define PCA9685_I2C_BUS         BUS_I2C2
