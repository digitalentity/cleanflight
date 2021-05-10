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

#define TARGET_BOARD_IDENTIFIER "CHF3" // Chebuzz F3

#define LED0                    PE8 // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1                    PE10  // Orange LEDs - PE10/PE14
#define LED1_INVERTED

#define BEEPER                  PE9 // Red LEDs - PE9/PE13
#define BEEPER_INVERTED

#define USE_SPI
#define USE_SPI_DEVICE_1

#define USE_IMU_L3GD20
#define USE_IMU_MPU6050
#define USE_IMU_LSM303DLHC

#define MPU6050_I2C_BUS                 BUS_I2C1
#define LSM303DLHC_I2C_BUS              BUS_I2C1
#define L3GD20_SPI_BUS                  BUS_SPI1
#define L3GD20_CS_PIN                   PE3

#define IMU_L3GD20_ALIGN       CW270_DEG
#define IMU_LSM303DLHC_ALIGN   CW0_DEG
#define IMU_MPU6050_ALIGN      CW0_DEG

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_MS5611

#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_AK8975
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_IST8310
#define USE_MAG_IST8308
#define USE_MAG_MAG3110
#define USE_MAG_LIS3MDL

#define USE_VCP
#define USE_UART1
#define USE_UART2
#define SERIAL_PORT_COUNT 3

#define USE_I2C
#define USE_I2C_DEVICE_1

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC_CHANNEL_1_PIN               PC0
#define ADC_CHANNEL_2_PIN               PC1
#define ADC_CHANNEL_3_PIN               PC2
#define ADC_CHANNEL_4_PIN               PC3
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    10

// IO - assuming 303 in 64pin package, TODO
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2)|BIT(5)|BIT(6)|BIT(10)|BIT(12)|BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTE         0xffff
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4)|BIT(9)|BIT(10))
