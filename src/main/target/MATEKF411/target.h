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

#define TARGET_BOARD_IDENTIFIER "MK41"

#define USBD_PRODUCT_STRING  "MatekF411"

#define LED0                    PC13
#define LED1                    PC14

#define BEEPER                  PB2
#define BEEPER_INVERTED

// *************** SPI1 Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW180_DEG
#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_BUS         BUS_SPI1

#define USE_EXTI
#define GYRO_INT_EXTI            PA1
#define USE_MPU_DATA_READY_SIGNAL

// *************** SPI2 OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          PB12

// *************** UART *****************************
#define USE_VCP
#define VBUS_SENSING_PIN        PC15
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#if defined(MATEKF411_SFTSRL2)
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_TX_PIN     PA0 // ST1 pad
#define SOFTSERIAL_1_RX_PIN     PA0
#define USE_SOFTSERIAL2
#define SOFTSERIAL_2_TX_PIN     PA8 // LED pad
#define SOFTSERIAL_2_RX_PIN     PA8
#define SERIAL_PORT_COUNT       5

#elif defined(MATEKF411_FD_SFTSRL)
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_TX_PIN     PA0 // ST1 pad
#define SOFTSERIAL_1_RX_PIN     PA8 // LED pad
#define SERIAL_PORT_COUNT       4

#elif defined(MATEKF411_RSSI)
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_TX_PIN     PA8 // LED pad
#define SOFTSERIAL_1_RX_PIN     PA8
#define SERIAL_PORT_COUNT       4

#else
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_TX_PIN     PA0 // ST1 pad
#define SOFTSERIAL_1_RX_PIN     PA0
#define SERIAL_PORT_COUNT       4
#endif

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

// *************** I2C /Baro/Mag/Pitot ********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define DEFAULT_I2C_BUS         BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_DPS310

#define TEMPERATURE_I2C_BUS     BUS_I2C1
#define BNO055_I2C_BUS          BUS_I2C1

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define PITOT_I2C_BUS           BUS_I2C1

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC_CHANNEL_1_PIN           PB0
#define ADC_CHANNEL_2_PIN           PB1
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2

#if defined(MATEKF411_RSSI)
#define ADC_CHANNEL_3_PIN           PA0
#define RSSI_ADC_CHANNEL            ADC_CHN_3
#endif

// *************** LED2812 ************************
#if !defined(MATEKF411_SFTSRL2) && !defined(MATEKF411_RSSI) && !defined(MATEKF411_FD_SFTSRL)
#define USE_LED_STRIP
#define WS2811_PIN                      PA8
#endif
// ***************  OTHERS *************************
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_OSD | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY | FEATURE_SOFTSERIAL )

#define USE_SPEKTRUM_BIND
#define BIND_PIN                PA10 //  RX1

#define USE_DSHOT
#define USE_ESC_SENSOR

#define USE_SERIALSHOT
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS       7
