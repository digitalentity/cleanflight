/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef OMNIBUSF7V2
#define TARGET_BOARD_IDENTIFIER "OB72"
#define USBD_PRODUCT_STRING "OmnibusF7V2"
#else
#define TARGET_BOARD_IDENTIFIER "OBF7"
#define USBD_PRODUCT_STRING "OmnibusF7"
#endif

// Use target-specific hardware descriptors (don't use common_hardware.h)
#define USE_TARGET_HARDWARE_DESCRIPTORS

#define LED0   PE0

#define BEEPER   PD15
#define BEEPER_INVERTED

#define USE_DUAL_GYRO

// ICM-20608-G

// MPU6000

#ifdef OMNIBUSF7V2
#   define USE_IMU_MPU6000
#   define IMU_MPU6000_ALIGN        CW0_DEG
#   define MPU6000_CS_PIN           SPI1_NSS_PIN
#   define MPU6000_SPI_BUS          BUS_SPI1
#   define MPU6000_EXTI_PIN         PE8

#   define USE_IMU_MPU6500
#   define IMU_MPU6500_ALIGN        CW90_DEG
#   define MPU6500_CS_PIN           SPI3_NSS_PIN
#   define MPU6500_SPI_BUS          BUS_SPI3
#   define MPU6500_EXTI_PIN         PD0
#else
#   define USE_IMU_MPU6000
#   define IMU_MPU6000_ALIGN        CW0_DEG
#   define MPU6000_CS_PIN           SPI3_NSS_PIN
#   define MPU6000_SPI_BUS          BUS_SPI3
#   define MPU6000_EXTI_PIN         PD0

#   define USE_IMU_MPU6500
#   define IMU_MPU6500_ALIGN        CW0_DEG
#   define MPU6500_CS_PIN           SPI1_NSS_PIN
#   define MPU6500_SPI_BUS          BUS_SPI1
#   define MPU6500_EXTI_PIN         PE8
#endif

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

#define USE_VCP
#define VBUS_SENSING_PIN PC4

#define USE_UART1
#define UART1_RX_PIN PA10
#define UART1_TX_PIN PA9

//#define AVOID_UART2_FOR_PWM_PPM
#define USE_UART2
#define UART2_TX_PIN PA2 //not wired
#define UART2_RX_PIN PA3

// Assigned to shared output I2C2
#define USE_UART3
#define UART3_RX_PIN PB11
#define UART3_TX_PIN PB10

#define USE_UART6
#define UART6_RX_PIN PC7
#define UART6_TX_PIN PC6

#ifdef OMNIBUSF7V2
#define USE_UART7
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            NONE
#endif

#ifdef OMNIBUSF7V2
#define SERIAL_PORT_COUNT 6 //VCP, USART1, USART2, USART3, USART6, USART7
// #define SERIAL_PORT_COUNT 5 //VCP, USART1, USART2, USART3, USART6
#else
#define SERIAL_PORT_COUNT 5 //VCP, USART1, USART2, USART3, USART6
#endif

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          SPI2_NSS_PIN

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PE3
#define SDCARD_SPI_BUS          BUS_SPI4
#define SDCARD_CS_PIN           SPI4_NSS_PIN

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2_SHARES_UART3

#define USE_BARO
#define USE_BARO_BMP280
#define BMP280_SPI_BUS          BUS_SPI1
#define BMP280_CS_PIN           PA1

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2

#define USE_RANGEFINDER
#define USE_RANGEFINDER_HCSR04_I2C
#define RANGEFINDER_I2C_BUS     BUS_I2C2

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO)

#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC2
#define ADC_CHANNEL_2_PIN               PC3
#define ADC_CHANNEL_3_PIN               PC5
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// #define USE_LED_STRIP

//Following configuration needs to be reviewed, when LED is enabled, VCP stops to work
//Until someone with deeper knowledge od DMA fixes it, LED are disabled in target
#define WS2811_PIN                      PD12

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_OSD)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    4
#define TARGET_MOTOR_COUNT      4
#define USE_DSHOT
#define USE_ESC_SENSOR

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define PCA9685_I2C_BUS         BUS_I2C2
