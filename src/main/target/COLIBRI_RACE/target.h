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

#define TARGET_BOARD_IDENTIFIER "CLBR"

#define LED0                    PC15
#define LED1                    PC14
#define LED2                    PC13

#define BEEPER                  PB13
#define BEEPER_INVERTED

#define USE_DSHOT
#define USE_ESC_SENSOR

// MPU6500 interrupt
#define USE_EXTI
#define GYRO_INT_EXTI            PA5
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5
#define SPI1_NSS_PIN            PA4

#define MPU6500_CS_PIN          SPI1_NSS_PIN
#define MPU6500_SPI_BUS         BUS_SPI1

#define MPU6000_CS_PIN          SPI1_NSS_PIN
#define MPU6000_SPI_BUS         BUS_SPI1

#define MPU9250_CS_PIN          SPI1_NSS_PIN
#define MPU9250_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW270_DEG
#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW270_DEG
#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW270_DEG

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C2
#define USE_BARO_MS5611

#define USE_MAG
#define MAG_I2C_BUS            BUS_I2C2
#define USE_MAG_MAG9250
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define SERIAL_PORT_COUNT       4

#define UART1_TX_PIN            PC4
#define UART1_RX_PIN            PC5

#define UART2_TX_PIN            PA14
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_I2C
#define USE_I2C_DEVICE_2

#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC2
#define ADC_CHANNEL_4_PIN               PC3
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

#define USE_LED_STRIP
#define WS2811_PIN                      PA6

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART3

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define PCA9685_I2C_BUS         BUS_I2C2
#define BNO055_I2C_BUS          BUS_I2C2