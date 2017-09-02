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

#undef TELEMETRY_JETIEXBUS // no space left

#define TARGET_BOARD_IDENTIFIER "FRF3"
#define TARGET_CONFIG
#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_NONE

#define LED0_PIN                PB3
#define BEEPER                  PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define MPU_INT_EXTI PC13
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU_INT, SDCardDetect
#define USE_MPU_DATA_READY_SIGNAL
#define MPU_ADDRESS             0x69

#ifdef MYMPU6000
#define MPU6000_SPI_INSTANCE    SPI2
#define MPU6000_CS_PIN          PB12
#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG
#else
#define GYRO
#define USE_GYRO_MPU6050
#define GYRO_MPU6050_ALIGN      CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define ACC_MPU6050_ALIGN       CW270_DEG
#endif

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       5

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

// Hardware wired to the SBUS output of the onboard XSR
#define UART2_TX_PIN            PA14 
#define UART2_RX_PIN            PA15

// Hardware wired to the Smart port output of the onboard XSR
#define UART3_TX_PIN            PB10 
#define UART3_RX_PIN            PB11 

#define SOFTSERIAL_1_RX_PIN     PB1
#define SOFTSERIAL_1_TX_PIN     PA1

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7


#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1
//#define ESCSERIAL_TIMER_TX_PIN  PB9  // (HARDARE=0)

#define USE_SPI
#define USE_SPI_DEVICE_1 
#define USE_SPI_DEVICE_2

#define OSD

// include the max7456 driver
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB4
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD*2)
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

#define SPI1_NSS_PIN            PC14
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15


#define USE_SDCARD

#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN                   PB5
#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_PIN                   SPI1_NSS_PIN

#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4


#define USE_ADC
#define ADC_INSTANCE                ADC2
#define ADC_CHANNEL_1_PIN               PA4
#define ADC_CHANNEL_2_PIN               PB2
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2

//#define VBAT_ADC_PIN                PA4
//#define CURRENT_METER_ADC_PIN       PB2
#define ADC24_DMA_REMAP            // Moves ADC2 DMA from DMA2ch1 to DMA2ch3.

#define LED_STRIP
#define WS2811_PIN                      PA8
#define WS2811_DMA_STREAM               DMA1_Channel2
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH2_HANDLER

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES        ( FEATURE_TELEMETRY | FEATURE_LED_STRIP | FEATURE_OSD)
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define TELEMETRY_UART          SERIAL_PORT_USART3
#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 10
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(17))
