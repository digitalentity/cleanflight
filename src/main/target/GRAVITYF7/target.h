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

#define TARGET_BOARD_IDENTIFIER "FLMO"
#define USBD_PRODUCT_STRING "GRAVITYF7"

//#define USE_HARDWARE_PREBOOT_SETUP

// User pin
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                          PC2 // VTX power switcher

#define LED0                                PC14
#define BEEPER                              PC13
#define BEEPER_INVERTED

// MPU6000
#define MPU6000_CS_PIN                      PB0
#define MPU6000_SPI_BUS                     BUS_SPI1
#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN                   CW0_DEG_FLIP

#define USE_EXTI
#define IMU_INT_EXTI                        PB10
#define USE_IMU_DATA_READY_SIGNAL

// USB
#define USE_VCP
#define VBUS_SENSING_PIN                    PA10
#define VBUS_SENSING_ENABLED

// UARTs
#define USE_UART1
#define UART1_TX_PIN                        PB6
#define UART1_RX_PIN                        PB7

#define USE_UART2
#define UART2_TX_PIN                        PA2
#define UART2_RX_PIN                        PA3

#define USE_UART3
#define UART3_TX_PIN                        PC10
#define UART3_RX_PIN                        PC11

#define USE_UART4
#define UART4_TX_PIN                        PA0
#define UART4_RX_PIN                        PA1

#define USE_UART5
#define UART5_TX_PIN                        PC12
#define UART5_RX_PIN                        PD2

#define USE_UART6
#define UART6_TX_PIN                        PC6
#define UART6_RX_PIN                        PC7

#define SERIAL_PORT_COUNT                   7

// i2c
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                            PB8
#define I2C1_SDA                            PB9

#define USE_BARO
#define BARO_I2C_BUS                        BUS_I2C1
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define BMP280_I2C_ADDR                     0x77


#define TEMPERATURE_I2C_BUS                 BUS_I2C1

#define PITOT_I2C_BUS                       BUS_I2C1

#define USE_RANGEFINDER
#define RANGEFINDER_I2C_BUS                 BUS_I2C1


// mag
#define USE_MAG
#define MAG_I2C_BUS                         BUS_I2C1
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

// temp
#define TEMPERATURE_I2C_BUS                 BUS_I2C1

#define PITOT_I2C_BUS                       BUS_I2C1


// SPIs
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                        PA5
#define SPI1_MISO_PIN                       PA6
#define SPI1_MOSI_PIN                       PA7


#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN                        PB12
#define SPI2_SCK_PIN                        PB13
#define SPI2_MISO_PIN                       PB14
#define SPI2_MOSI_PIN                       PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN                        PB2
#define SPI3_SCK_PIN                        PB3
#define SPI3_MISO_PIN                       PB4
#define SPI3_MOSI_PIN                       PB5


// OSD
#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS                     BUS_SPI2
#define MAX7456_CS_PIN                      SPI2_NSS_PIN


// BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN                       SPI3_NSS_PIN
#define M25P16_SPI_BUS                      BUS_SPI3
#define USE_FLASHFS
#define USE_FLASH_M25P16



// ADC
#define USE_ADC
#define ADC_INSTANCE                         ADC1
#define ADC1_DMA_STREAM                      DMA2_Stream0
#define ADC_CHANNEL_1_PIN                    PC0
#define ADC_CHANNEL_2_PIN                    PC1

#define CURRENT_METER_ADC_CHANNEL            ADC_CHN_1
#define VBAT_ADC_CHANNEL                     ADC_CHN_2
#define VBAT_SCALE_DEFAULT                   1120
#define CURRENT_METER_SCALE                  182


// LED Strip
#define USE_LED_STRIP
#define WS2811_PIN                          PA15   //TIM2_CH1



// FEATURES
#define DEFAULT_FEATURES                     (FEATURE_VBAT | FEATURE_OSD | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_BLACKBOX )
#define DEFAULT_RX_TYPE                      RX_TYPE_SERIAL
#define SERIALRX_PROVIDER                    SERIALRX_SBUS
#define SERIALRX_UART                        SERIAL_PORT_USART1


#define MAX_PWM_OUTPUT_PORTS                 6

#define TARGET_IO_PORTA                      0xffff
#define TARGET_IO_PORTB                      0xffff
#define TARGET_IO_PORTC                      0xffff
#define TARGET_IO_PORTD                      (BIT(2))

#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_DSHOT
#define USE_ESC_SENSOR
#define USE_SERIALSHOT

