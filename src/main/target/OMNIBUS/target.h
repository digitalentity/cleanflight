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

#define TARGET_BOARD_IDENTIFIER "OMNI" // https://en.wikipedia.org/wiki/Omnibus

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#undef USE_VTX_SMARTAUDIO       // Disabled due to flash size

#define USE_EXTI

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW90_DEG
#define MPU6000_SPI_BUS         BUS_SPI1
#define MPU6000_CS_PIN          PA4
#define GYRO_INT_EXTI            PC13
#define USE_MPU_DATA_READY_SIGNAL


#define USE_BARO
#define USE_BARO_BMP280
#define BMP280_SPI_BUS          BUS_SPI1
#define BMP280_CS_PIN           PA13

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
// #define USE_MAG_MAG3110
// #define USE_MAG_LIS3MDL
// #define USE_MAG_AK8975

// Disable certain features to save flash space
#undef USE_GPS_PROTO_MTK

#define USB_CABLE_DETECTION
#define USB_DETECT_PIN          PB5

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

// Enable I2C instead of PWM7&8 for iNav
#define USE_I2C
#define USE_I2C_DEVICE_1 // PB6/SCL(PWM8), PB7/SDA(PWM7)
#define USE_I2C_PULLUP

#define PITOT_I2C_BUS           BUS_I2C1

#define USE_SPI
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define RX_SPI_INSTANCE SPI2
#define RX_NSS_PIN PB3

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN       PC14
#define SDCARD_SPI_BUS          BUS_SPI2
#define SDCARD_CS_PIN           SPI2_NSS_PIN

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS             BUS_SPI1
#define MAX7456_CS_PIN              PB1

#define USE_ADC
#define ADC_INSTANCE                ADC1
//#define BOARD_HAS_VOLTAGE_DIVIDER
#define ADC_CHANNEL_1_PIN           PA0
#define ADC_CHANNEL_2_PIN           PA1
#define ADC_CHANNEL_3_PIN           PB2
#define ADC_CHANNEL_3_INSTANCE      ADC2
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

//#define USE_LED_STRIP
//#define WS2811_PIN                      PA8

//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_BLACKBOX | FEATURE_OSD)

#define BUTTONS
#define BUTTON_A_PORT           GPIOB // Non-existent (PB1 used for RSSI/MAXCS)
#define BUTTON_A_PIN            Pin_1
#define BUTTON_B_PORT           GPIOB // TRIG button, used for BINDPLUG_PIN
#define BUTTON_B_PIN            Pin_0

#define USE_SPEKTRUM_BIND
// USART3
#define BIND_PIN                PB11

#define HARDWARE_BIND_PLUG
#define BINDPLUG_PIN            PB0

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))
