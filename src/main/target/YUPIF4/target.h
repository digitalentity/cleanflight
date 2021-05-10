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

#if defined (YUPIF4R2)
#define TARGET_BOARD_IDENTIFIER "YPF4R2"
#define USBD_PRODUCT_STRING "YUPIF4R2"
#elif (YUPIF4MINI)
#define TARGET_BOARD_IDENTIFIER "YPF4M"
#define USBD_PRODUCT_STRING     "YUPIF4MINI"
#else
#define TARGET_BOARD_IDENTIFIER "YPF4"
#define USBD_PRODUCT_STRING     "YUPIF4"
#endif

#define LED0                    PB6
#define LED1                    PB4

#if defined(YUPIF4R2)
#define BEEPER                  PB14
#else
#define BEEPER                  PC9
#endif

#if defined(YUPIF4MINI)
// #define BEEPER_INVERTED
#else
#define BEEPER_PWM
#define BEEPER_INVERTED
#define BEEPER_PWM_FREQUENCY    3150
#endif

#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW90_DEG

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C1_SCL                PB10
#define I2C1_SDA                PB11

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C2
#define USE_BARO_MS5611
#define USE_BARO_BMP280

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI1
#define MAX7456_CS_PIN          PA14

#if defined(YUPIF4MINI)
#else
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PD2
#define SDCARD_SPI_BUS          BUS_SPI3
#define SDCARD_CS_PIN           SPI3_NSS_PIN
#endif

#define USB_IO
#define USE_VCP

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define INVERTER_PIN_UART6_RX   PB15

#define SERIAL_PORT_COUNT       5

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define USE_SPI
#define USE_SPI_DEVICE_1 	// Gyro
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3 	// SD Card
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

// ADC inputs
#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN       PC1
#define ADC_CHANNEL_2_PIN       PC0
#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define RSSI_ADC_CHANNEL        ADC_CHN_2

// LED Strip can run off Pin 5 (PB1) of the motor outputs
#define USE_LED_STRIP
#define WS2811_PIN                      PB1

// Features
// #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6
#define DEFAULT_FEATURES        (FEATURE_VBAT | FEATURE_OSD)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define PCA9685_I2C_BUS         BUS_I2C2
