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
 //Target code By BorisB and Hector "Hectech FPV" Hind

#pragma once

#define TARGET_BOARD_IDENTIFIER "BFF3"

#define BEEPER                  PC15
#define BEEPER_INVERTED

#define MPU6000_CS_PIN          PA15
#define MPU6000_SPI_BUS    BUS_SPI1

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI_CALLBACK_HANDLER_COUNT 1
#define GYRO_INT_EXTI                PC13
#define USE_EXTI

//#define USE_ESC_SENSOR // XXX
//#define REMAP_TIM16_DMA // XXX
//#define REMAP_TIM17_DMA // XXX

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5

//#define USE_ESCSERIAL // XXX
//#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1 XXX

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define SOFTSERIAL_1_RX_PIN      PB0 // PWM 5
#define SOFTSERIAL_1_TX_PIN      PB1 // PWM 6

#undef USE_I2C

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define SPI1_NSS_PIN            PA15
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI1
#define MAX7456_CS_PIN          PA1

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_DETECT_INVERTED
#define SDCARD_DETECT_PIN           PC14
#define SDCARD_SPI_BUS              BUS_SPI2
#define SDCARD_CS_PIN               SPI2_NSS_PIN

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE                ADC2
#define ADC_CHANNEL_1_PIN	        PA4
#define ADC_CHANNEL_2_PIN	        PA5
#define ADC_CHANNEL_3_PIN	        PB2
#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

//#define USE_LED_STRIP

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2
//#define SBUS_TELEMETRY_UART     SERIAL_PORT_USART1 // XXX
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX | FEATURE_CURRENT_METER | FEATURE_TELEMETRY ) // XXX

//#define USE_SPEKTRUM_BIND
//#define BIND_PIN                UART2_RX_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define MAX_PWM_OUTPUT_PORTS 8
