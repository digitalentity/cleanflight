/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS
#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "FRP"
#define USBD_PRODUCT_STRING     "FrSkyPilot"

#define LED0                    PA3
#define LED1                    PC5

#define BEEPER                  PA0
#define BEEPER_INVERTED
#define BEEPER_PWM
#define BEEPER_PWM_FREQUENCY    4000

#define USE_SPI

// *************** SPI1 BARO **********************

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_BARO
#define USE_BARO_SPL06
#define SPL06_SPI_BUS          BUS_SPI1
#define SPL06_CS_PIN           SPI1_NSS_PIN

// *************** SPI2/3 IMUs *******************

// IMU box
#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

// IMU on-board
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS

#define USE_IMU_MPU6000
#define USE_IMU_MPU6500

#define USE_EXTI
#define USE_MPU_DATA_READY_SIGNAL

//#define USE_ONBOARD_IMU
//#define USE_BOX_IMU
#define USE_DUAL_GYRO

//#if defined(USE_ONBOARD_IMU)
#define MPU6500_EXTI_PIN        PE8
#define MPU6500_SPI_BUS         BUS_SPI3
#define MPU6500_CS_PIN          SPI3_NSS_PIN
#define IMU_MPU6500_ALIGN       CW0_DEG_FLIP

//#elif defined(USE_BOX_IMU)
#define MPU6000_EXTI_PIN        NONE
#define MPU6000_SPI_BUS         BUS_SPI2
#define MPU6000_CS_PIN          SPI2_NSS_PIN
#define IMU_MPU6000_ALIGN       CW270_DEG_FLIP // XXX check

//#endif // IMU SELECTION

// *************** SPI4: SDCARD *******************

#define USE_SPI_DEVICE_4
#define SPI4_NSS_PIN            PE4
#define SPI4_SCK_PIN            PE2
#define SPI4_MISO_PIN           PE5
#define SPI4_MOSI_PIN           PE6

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI4
#define SDCARD_CS_PIN           SPI4_NSS_PIN
#define SDCARD_DETECT_PIN       PE3
#define SDCARD_DETECT_INVERTED

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// *************** I2C *********************
#define USE_I2C

#define USE_I2C_DEVICE_3
#define I2C3_SCL                PA8
#define I2C3_SDA                PC9

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C3
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C3
#define BNO055_I2C_BUS          BUS_I2C3
#define PITOT_I2C_BUS           BUS_I2C3

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS     BUS_I2C3

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PC4
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6

#define USE_UART3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_UART5
#define UART5_TX_PIN            PB6
#define UART5_RX_PIN            PB5
#define UART5_AF                1

// OSD
#define USE_OSD
#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

// RX connector
#define USE_UART7
#define UART7_TX_PIN            PB4
#define UART7_RX_PIN            PB3
#define UART7_AF                12

#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            PE0

#define SERIAL_PORT_COUNT       8

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_FPORT2
#define SERIALRX_UART           SERIAL_PORT_USART8

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1

#define ADC_CHANNEL_1_PIN           PC2
#define ADC_CHANNEL_2_PIN           PC3
#define ADC_CHANNEL_3_PIN           PC1
#define ADC_CHANNEL_4_PIN           PC0

#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_1
#define VBAT_ADC_CHANNEL            ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3
#define AIRSPEED_ADC_CHANNEL        ADC_CHN_4

#define DEFAULT_FEATURES            (FEATURE_OSD | FEATURE_TELEMETRY | FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)
#define VBAT_SCALE_DEFAULT          1545
#define CURRENT_METER_SCALE         171
#define CURRENT_METER_OFFSET        3288

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#ifdef FRSKYPILOT_LED
    #define USE_LED_STRIP
    #define WS2811_PIN                      PA1 // S10 pad for iNav

    #define MAX_PWM_OUTPUT_PORTS 9
#else
    #define MAX_PWM_OUTPUT_PORTS 12
#endif

#define USE_DSHOT
