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

#define TARGET_BOARD_IDENTIFIER "SBF4"		
#define USBD_PRODUCT_STRING  "SpeedyBeeF4"	

/*** Indicators ***/
#define LED0                    PB9			
#define BEEPER                  PC13		
#define BEEPER_INVERTED						

/*** IMU sensors ***/
#define USE_EXTI							

#define GYRO_INT_EXTI            PC4		
#define USE_MPU_DATA_READY_SIGNAL			

#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG|SENSOR_BARO)

/*** MPU6000 ***/
#define USE_IMU_MPU6000
#define MPU6000_CS_PIN          PB11
#define MPU6000_SPI_BUS         BUS_SPI1
#define IMU_MPU6000_ALIGN       CW0_DEG

/*** SPI/I2C bus ***/
#define USE_SPI								
#define USE_SPI_DEVICE_1					
#define SPI1_SCK_PIN            PA5			
#define SPI1_MISO_PIN           PA6			
#define SPI1_MOSI_PIN           PA7			

#define USE_SPI_DEVICE_2					
#define SPI2_NSS_PIN            PB12		
#define SPI2_SCK_PIN            PB13		
#define SPI2_MISO_PIN           PB14		
#define SPI2_MOSI_PIN           PB15		

#define USE_SPI_DEVICE_3					
#define SPI3_SCK_PIN            PB3			
#define SPI3_MISO_PIN           PB4			
#define SPI3_MOSI_PIN           PB5			

#define I2C_DEVICE              (I2CDEV_1)	
#define UG2864_I2C_BUS BUS_I2C1				

#define USE_I2C								
#define USE_I2C_DEVICE_1					
#define I2C1_SCL                PB6        	
#define I2C1_SDA                PB7        	

/*** Onboard flash ***/
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#define M25P16_CS_PIN           PC0				
#define M25P16_SPI_BUS          BUS_SPI3		
#define USE_FLASHFS								
#define USE_FLASH_M25P16	

/*** OSD ***/
#define USE_OSD								
#define USE_MAX7456							
#define MAX7456_SPI_BUS         BUS_SPI2	
#define MAX7456_CS_PIN          PB10		

/*** Serial ports ***/
#define USE_VCP
#define VBUS_SENSING_PIN        PB12
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART3
#define UART3_RX_PIN            PC11
#define UART3_TX_PIN            PC10

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#if defined(SPEEDYBEEF4_SFTSRL1)
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA15 // S5
#define SOFTSERIAL_1_TX_PIN     PB8  // S7

#define SERIAL_PORT_COUNT       7

#elif defined(SPEEDYBEEF4_SFTSRL2)
#define USE_SOFTSERIAL1
#define SOFTSERIAL_1_RX_PIN     PA15 // S5
#define SOFTSERIAL_1_TX_PIN     PB8  // S7
#define USE_SOFTSERIAL2
#define SOFTSERIAL_2_RX_PIN     PA8 // S6   
#define SOFTSERIAL_2_TX_PIN     PA4 // DAC

#define SERIAL_PORT_COUNT       8

#else
#define SERIAL_PORT_COUNT       6
#endif

#define BNO055_I2C_BUS          BUS_I2C1

/*** BARO & MAG ***/
#define USE_BARO							
#define BARO_I2C_BUS          	BUS_I2C1	
#define USE_BARO_BMP085						
#define USE_BARO_BMP280						
#define USE_BARO_MS5611						

#define USE_MAG								
#define MAG_I2C_BUS             BUS_I2C1	
#define USE_MAG_HMC5883						
#define USE_MAG_QMC5883						
#define USE_MAG_IST8310						
#define USE_MAG_MAG3110						
#define USE_MAG_LIS3MDL						

/*** ADC ***/
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC1
#define ADC_CHANNEL_2_PIN               PC2
#define ADC_CHANNEL_3_PIN               PC3

#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_1
#define VBAT_ADC_CHANNEL                ADC_CHN_2
#define RSSI_ADC_CHANNEL                ADC_CHN_3

/*** LED STRIP ***/

/*** Default settings ***/
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_BLACKBOX | FEATURE_VBAT | FEATURE_OSD)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define CURRENT_METER_SCALE     166

/*** Timer/PWM output ***/
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define MAX_PWM_OUTPUT_PORTS    7
#define USE_DSHOT
#define USE_ESC_SENSOR

/*** Used pins ***/
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define RANGEFINDER_I2C_BUS BUS_I2C1