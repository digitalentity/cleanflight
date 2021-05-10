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

#define TARGET_BOARD_IDENTIFIER "FDV1"

#define USBD_PRODUCT_STRING "FishDroneF4NAV"

// *************** LED *****************************
#define LED0                    PC13    // Red
#define LED1                    PC14    // Yellow

// *************** BEEPER *****************************
#define BEEPER                  PC15

// *************** SPI *****************************
#define USE_SPI

// *************** ICM20608 *****************************
#define USE_SPI_DEVICE_1
#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_BUS         BUS_SPI1
#define MPU9250_CS_PIN          PA4
#define MPU9250_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW180_DEG

#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW180_DEG

// MPU6500 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// *************** Compass *****************************
#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_MPU9250
#define USE_MAG_MAG3110
#define USE_MAG_HMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

// *************** Temperature sensor *****************
#define TEMPERATURE_I2C_BUS     BUS_I2C1

#define BNO055_I2C_BUS          BUS_I2C1

// *************** BARO *****************************
#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_MS5611

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PC2
#define SPI2_MOSI_PIN   PC3

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          SPI2_NSS_PIN

// *************** TF Support *****************************
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#define SPI3_NSS_PIN    PB6

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PB7
#define SDCARD_SPI_BUS          BUS_SPI3
#define SDCARD_CS_PIN           PB6

// *************** Flash *****************************
#define M25P16_CS_PIN           PA15
#define M25P16_SPI_BUS          BUS_SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16

// *************** UART *****************************
#define USE_VCP

#define USE_UART_INVERTER

// provide for Telemetry module
#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

// provide for xBUS Receiver
#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2
#define INVERTER_PIN_UART2_RX   PB2

// provide for GPS module
#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define SERIAL_PORT_COUNT       4 // VCP, USART1, USART2, USART5

// *************** WS2811 *****************************
#define USE_LED_STRIP
#define WS2811_PIN                      PB1

// *************** IIC *****************************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                        PB8
#define I2C1_SDA                        PB9

// *************** ADC *****************************
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC5
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3
#define VBAT_SCALE_DEFAULT         103

// *************** RANGEFINDER *****************************
// #define USE_RANGEFINDER
// #define USE_RANGEFINDER_HCSR04
// #define RANGEFINDER_HCSR04_TRIGGER_PIN       PB10
// #define RANGEFINDER_HCSR04_ECHO_PIN          PB11
// #define USE_RANGEFINDER_SRF10

// *************** NAV *****************************
#define USE_NAV
#define NAV_GPS_GLITCH_DETECTION
#define NAV_MAX_WAYPOINTS       60

// *************** Others *****************************
#define DISPLAY

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX | FEATURE_RSSI_ADC | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
