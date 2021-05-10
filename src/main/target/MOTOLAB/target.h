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

#define TARGET_BOARD_IDENTIFIER "MOTO" // MotoLab

#define LED0                    PB5 // Blue LEDs - PB5
#define LED1                    PB9 // Green LEDs - PB9

#define BEEPER                  PA0
#define BEEPER_INVERTED

// MPU6050 interrupts
#define USE_EXTI
#define GYRO_INT_EXTI            PA15
#define USE_MPU_DATA_READY_SIGNAL

#define USE_IMU_MPU6050
#define IMU_MPU6050_ALIGN       CW180_DEG

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW180_DEG

#define MPU6050_I2C_BUS         BUS_I2C2
#define MPU6000_CS_PIN          PB12
#define MPU6000_SPI_BUS         BUS_SPI2

//#define USE_BARO
//#define USE_BARO_MS5611

//#define USE_MAG
//#define USE_MAG_HMC5883
//#define USE_MAG_QMC5883
//#define USE_MAG_IST8310
//#define USE_MAG_IST8308
//#define USE_MAG_MAG3110
//#define USE_MAG_LIS3MDL

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

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_SPI
#define USE_SPI_DEVICE_2

#define M25P16_CS_PIN           PB12
#define M25P16_SPI_BUS          BUS_SPI2

//#define SENSORS_SET             (SENSOR_ACC | SENSOR_BARO | SENSOR_GPS | SENSOR_MAG)
#define SENSORS_SET             (SENSOR_ACC)

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define ADC_CHANNEL_1_PIN       PA5
#define ADC_CHANNEL_2_PIN       PB2
#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define RSSI_ADC_CHANNEL        ADC_CHN_2

#define USE_LED_STRIP

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_PIN                      PB8 // TIM16_CH1


#define USE_SPEKTRUM_BIND
// USART2, PB4
#define BIND_PIN                PB4

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

//#undef USE_GPS
#undef USE_GPS_PROTO_NMEA
//#undef USE_GPS_PROTO_UBLOX
#undef USE_GPS_PROTO_I2C_NAV
#undef USE_GPS_PROTO_NAZA

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    8

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
// #define TARGET_IO_PORTF         (BIT(0)|BIT(1))
// !!TODO - check the following line is correct
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(3)|BIT(4))

#define PCA9685_I2C_BUS         BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2