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
#define TARGET_BOARD_IDENTIFIER "F4BY"

#define USBD_PRODUCT_STRING     "Swift-Flyer F4BY"

#define LED0                    PE3 // Blue LED
#define LED1                    PE2 // Red LED
#define LED2                    PE1 // Green LED

#define BEEPER                  PE5
#define BEEPER_INVERTED


// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define GYRO_INT_EXTI            PB0
#define USE_EXTI

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_BUS         BUS_SPI1
#define ICM20689_CS_PIN         PA4
#define ICM20689_SPI_BUS        BUS_SPI1


#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW90_DEG

#define USE_IMU_ICM20689
#define IMU_ICM20689_ALIGN       CW90_DEG

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

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C2
#define USE_BARO_MS5611


#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI2
#define SDCARD_CS_PIN           PE15

#define USE_VCP
#define VBUS_SENSING_PIN        PA9

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2

#define USE_UART2
#define UART2_RX_PIN            PD6
#define UART2_TX_PIN            PD5

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PC11
#define UART4_TX_PIN            PC10


#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#define INVERTER_PIN_UART6_RX   PD3

#define SERIAL_PORT_COUNT       6 //VCP, UART1, UART2, UART3, UART4,  UART6

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define SPI1_NSS_PIN            NONE

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            NONE
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5


#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC3
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC1
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_BLACKBOX)
#define DEFAULT_RX_TYPE         RX_TYPE_PPM

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    15

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTE         0xffff

#define PCA9685_I2C_BUS         BUS_I2C2
