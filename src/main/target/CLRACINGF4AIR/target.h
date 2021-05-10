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

#if defined(CLRACINGF4AIRV2)
#define TARGET_BOARD_IDENTIFIER "CLA2"
#define USBD_PRODUCT_STRING "CLRACINGF4AIRV2"
#elif defined(CLRACINGF4AIRV3)
#define TARGET_BOARD_IDENTIFIER "CLA3"
#define USBD_PRODUCT_STRING "CLRACINGF4AIRV3"
#else
#define TARGET_BOARD_IDENTIFIER "CLRA"
#define USBD_PRODUCT_STRING "CLRACINGF4AIR"
#endif
#if defined(CLRACINGF4AIRV3)
#define LED0                    PC14
#else
#define LED0                    PB5
#endif
#define BEEPER                  PB4
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#if defined(CLRACINGF4AIRV3)
#define SPI3_MOSI_PIN           PB5
#else
#define SPI3_MOSI_PIN           PC12
#endif

//MPU-9250
#define MPU9250_CS_PIN          PA4
#define MPU9250_SPI_BUS         BUS_SPI1
#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW0_DEG

#define USE_MAG
#define USE_MAG_MPU9250

// MPU6 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PC4
#define USE_MPU_DATA_READY_SIGNAL

#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_BUS          BUS_SPI3
#define BMP280_CS_PIN           PB3

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI3
#define MAX7456_CS_PIN          PA15

#define USE_LED_STRIP
#define WS2811_PIN                      PB8

#define USE_VCP

#define USE_UART_INVERTER

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9
#define UART1_AHB1_PERIPHERALS  RCC_AHB1Periph_DMA2
#define INVERTER_PIN_UART1_RX   PC0 // PC0 used as inverter select GPIO

#if defined( CLRACINGF4AIRV2) || defined(CLRACINGF4AIRV3)
#define USE_UART2
#define UART2_RX_PIN            PA2
#define UART2_TX_PIN            PA3
#endif
#define USE_UART3
#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10
#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0
#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6
#if defined( CLRACINGF4AIRV2 )
#define SERIAL_PORT_COUNT        6 //VCP, USART1, UART2, USART3,USART4, USART6,
#elif defined(CLRACINGF4AIRV3)
#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12
#define SERIAL_PORT_COUNT        7 //VCP, USART1, UART2, USART3,USART4, USART5,USART6,
#else
#define SERIAL_PORT_COUNT       5 //VCP, USART1, USART3,USART4, USART6,
#endif
#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1
#if defined( CLRACINGF4AIRV2 ) || defined(CLRACINGF4AIRV3)
#define USE_I2C
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                 PB10
#define I2C2_SDA                PB11

#define TEMPERATURE_I2C_BUS     BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2
#endif

#define USE_ADC
#define ADC_INSTANCE                         ADC1
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC3

#define CURRENT_METER_ADC_CHANNEL        ADC_CHN_1
#define VBAT_ADC_CHANNEL                 ADC_CHN_2
#define RSSI_ADC_CHANNEL                   ADC_CHN_3

// V2 has airspeed input
#if defined( CLRACINGF4AIRV2 ) || defined(CLRACINGF4AIRV3)
#define ADC_CHANNEL_4_PIN               PC5
#define ADC_AIRSPEED_CHANNEL          ADC_CHN_4
#define CURRENT_METER_SCALE 250
#endif

#define USE_ESC_SENSOR
#define DEFAULT_FEATURES         (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_TELEMETRY| FEATURE_VBAT | FEATURE_OSD )

#define SPEKTRUM_BIND_PIN       UART1_RX_PIN
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    8
#define TARGET_MOTOR_COUNT      8

#define TARGET_IO_PORTA (0xffff)
#define TARGET_IO_PORTB (0xffff)
#define TARGET_IO_PORTC (0xffff)
#define TARGET_IO_PORTD BIT(2)
