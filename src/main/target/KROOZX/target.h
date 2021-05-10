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

#define TARGET_BOARD_IDENTIFIER "KROOZX"
#define USBD_PRODUCT_STRING     "KroozX"

#define USE_HARDWARE_PREBOOT_SETUP

#define LED0                    PA14 // Red LED
#define LED1                    PA13 // Green LED

#define BEEPER                  PC1
#define BEEPER_INVERTED

#define MPU6000_CS_PIN          PB2
#define MPU6000_SPI_BUS         BUS_SPI1

// MPU6000 interrupts
#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL
#define GYRO_INT_EXTI            PA4

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW90_DEG_FLIP

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C3
#define USE_BARO_MS5611

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI3
#define SDCARD_CS_PIN           PA15
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PC13

#define USE_OSD
#ifdef USE_MSP_DISPLAYPORT
#undef USE_MSP_DISPLAYPORT
#endif

#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI1
#define MAX7456_CS_PIN          PC4

#define OSD_CH_SWITCH           PC5

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC3
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC0
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define USE_VCP

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define INVERTER_PIN_UART1_RX   PB13

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define INVERTER_PIN_UART6_RX   PB12

#define SERIAL_PORT_COUNT       6

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_I2C_DEVICE_3
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define RX_CHANNELS_TAER
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_OSD)

#define USE_LED_STRIP
#define WS2811_PIN                      PC6

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS    10
