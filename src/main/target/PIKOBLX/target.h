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

#define TARGET_BOARD_IDENTIFIER "PIKO" // Furious FPV Piko BLX

#define CONFIG_FASTLOOP_PREFERRED_ACC IMU_DEFAULT

#define LED0                    PB9
#define LED1                    PB5

#define BEEPER                  PA0
#define BEEPER_INVERTED

// MPU6000 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PA15
#define USE_MPU_DATA_READY_SIGNAL

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG
#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_BUS         BUS_SPI2

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define USE_SPI
#define USE_SPI_DEVICE_2

#define SENSORS_SET             (SENSOR_ACC)

#define USE_TELEMETRY
#define USE_BLACKBOX
#define USE_SERIAL_RX

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE            ADC2
//#define ADC_CHANNEL_1_PIN               PA2   <-- conflict with pwm output 6, and the board does not have current sensor
#define ADC_CHANNEL_2_PIN               PA5
//#define ADC_CHANNEL_3_PIN               PB2   <-- the board does not have a rssi pad

#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

//#define USE_LED_STRIP
//#define WS2811_PIN                      PB8 // TIM16_CH1

// #define USE_TRANSPONDER
// #define TRANSPONDER_PIN                 PA8

#define USE_SPEKTRUM_BIND
// USART3, PB11
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

// sn dec06.16 added MAX_PWM_OUTPUT_PORTS: number of available PWM outputs
// porting inav to PIKO BLX by using betaflight target files from before inav changes to timer.h/timer_def.h
// review of target.c shows definitions for PWB1-PWM9. PWM9 is for PPM input
#define MAX_PWM_OUTPUT_PORTS    6
