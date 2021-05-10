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

#ifdef MAMBAF405
#define TARGET_BOARD_IDENTIFIER "MBF4"
#define USBD_PRODUCT_STRING     "MAMBAF405"
#else
#define TARGET_BOARD_IDENTIFIER "FYF4"
#define USBD_PRODUCT_STRING     "FuryF4"
#endif

#define LED0                    PB5
#define LED1                    PB4

#define BEEPER                  PA8
#define BEEPER_INVERTED

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW180_DEG
#define MPU6500_CS_PIN          PA4
#define MPU6500_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1

#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN   	      PC11
#define SPI3_MOSI_PIN   	      PC12

// *************** M25P256 flash ********************
#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_BUS          BUS_SPI3
#define M25P16_CS_PIN           PB3

// *************** OSD *****************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN   	      PB14
#define SPI2_MOSI_PIN   	      PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI2
#define MAX7456_CS_PIN          PB12

// *************** UART *****************************
#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN      PA1
#define SOFTSERIAL_1_TX_PIN      PC9  // PPM

//#define USE_SOFTSERIAL2
//#define SOFTSERIAL_2_RX_PIN      PA2
//#define SOFTSERIAL_2_TX_PIN      PA2

#define SERIAL_PORT_COUNT       5

#ifdef MAMBAF405
#   define USE_UART_INVERTER
#   define INVERTER_PIN_UART1_RX    PC0
#endif


#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7
#define DEFAULT_I2C_BUS         BUS_I2C1

#define USE_BARO
#define BARO_I2C_BUS                DEFAULT_I2C_BUS
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_DPS310

#define USE_MAG
#define MAG_I2C_BUS                 DEFAULT_I2C_BUS
#define USE_MAG_AK8963
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS         DEFAULT_I2C_BUS
#define BNO055_I2C_BUS              DEFAULT_I2C_BUS

#define USE_RANGEFINDER
#define USE_RANGEFINDER_MSP
#define USE_RANGEFINDER_HCSR04_I2C
#define RANGEFINDER_I2C_BUS     DEFAULT_I2C_BUS

#define PITOT_I2C_BUS               DEFAULT_I2C_BUS

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0
#define ADC_CHANNEL_1_PIN           PC1

#ifdef MAMBAF405
#   define ADC_CHANNEL_2_PIN       PC3
#   define ADC_CHANNEL_3_PIN       PC2
#else
#   define ADC_CHANNEL_2_PIN       PC2
#   define ADC_CHANNEL_3_PIN       PC3
#endif

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY )
#define CURRENT_METER_SCALE   179

#define USE_LED_STRIP
#define WS2811_PIN                  PA0

//#define USE_SPEKTRUM_BIND
//#define BIND_PIN                PA3 //  RX2

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USE_DSHOT
#define USE_ESC_SENSOR

#define MAX_PWM_OUTPUT_PORTS       6

#define PCA9685_I2C_BUS             DEFAULT_I2C_BUS
