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
#define TARGET_BOARD_IDENTIFIER "PXR4"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#define USBD_PRODUCT_STRING     "PixRacer"

// Use target-specific hardware descriptors (don't use common_hardware.h)
#define USE_TARGET_HARDWARE_DESCRIPTORS

// PixRacer target requires some hardware to be set up before booting and detecting sensors
#define USE_HARDWARE_PREBOOT_SETUP

#define USE_EXTI

#define LED0                    PB11    //red
#define LED1                    PB3     //blue
#define LED2                    PB1     //green

#define BEEPER                  PA15
#define BEEPER_INVERTED

#define INVERTER_PIN_UART       PC13

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW180_DEG_FLIP

#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW180_DEG_FLIP

#define USE_DUAL_GYRO

// ICM20608
#define ICM20608_CS_PIN         PC15
#define ICM20608_EXTI_PIN       PC14
#define ICM20608_SPI_BUS        BUS_SPI1

// MPU9250 gyro/acc/mag
#define MPU9250_CS_PIN          PC2
#define MPU9250_EXTI_PIN        PD15
#define MPU9250_SPI_BUS         BUS_SPI1

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_MPU9250
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define TEMPERATURE_I2C_BUS     BUS_I2C1

#define USE_BARO
#define USE_BARO_MS5611
#define MS5611_CS_PIN           PD7
#define MS5611_SPI_BUS          BUS_SPI2

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_SDIO_DMA         DMA_TAG(2,3,4)
#define SDCARD_SDIO_4BIT
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// MPU9250 interrupt
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define USE_VCP
#define VBUS_SENSING_PIN PA9
#define VBUS_SENSING_ENABLED

#define USE_UART1                       // ESP8266
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART2                       // TELEM1
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3                       // TELEM2
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4                       // GPS
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6                       // SerialRX (RX line is used only)
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_UART7                       // DEBUG connector
#define UART7_RX_PIN            PE7
#define UART7_TX_PIN            PE8

#define USE_UART8                       // FRS
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define SERIAL_PORT_COUNT       8

#define USE_SPI

#define USE_SPI_DEVICE_1        // Acc/Gyro/Compass
#define SPI1_NSS_PIN            PC2
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2        // FRAM/BARO
#define SPI2_SCK_PIN            PB10
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_PULLUP
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9


#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PA2
#define ADC_CHANNEL_2_PIN               PA3
#define ADC_CHANNEL_3_PIN               PC1
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// SDCARD not yet supported
//#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART6

#define DEFAULT_RX_TYPE         RX_TYPE_PPM
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    6

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff
