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

#if defined(STM32F3)
#define DYNAMIC_HEAP_SIZE   1536
#else
#define DYNAMIC_HEAP_SIZE   2048
#endif

#define I2C1_OVERCLOCK false
#define I2C2_OVERCLOCK false
#define USE_I2C_PULLUP          // Enable built-in pullups on all boards in case external ones are too week

#define USE_RX_PWM
#define USE_RX_PPM
#define USE_SERIAL_RX
#define USE_SERIALRX_SPEKTRUM   // Cheap and fairly common protocol
#define USE_SERIALRX_SBUS       // Very common protocol
#define USE_SERIALRX_IBUS       // Cheap FlySky & Turnigy receivers
#define USE_SERIALRX_FPORT

#define COMMON_DEFAULT_FEATURES (FEATURE_TX_PROF_SEL)

#if defined(STM32F3)
#define USE_UNDERCLOCK
#endif

#if defined(STM32F3) || defined(STM32F4)
#define USE_ADC_AVERAGING
#define ADC_AVERAGE_N_SAMPLES 20
#endif

#define USE_64BIT_TIME
#define USE_BLACKBOX
#define USE_GPS
#define USE_GPS_PROTO_UBLOX
#define USE_NAV
#define USE_TELEMETRY
#define USE_TELEMETRY_LTM
#define USE_TELEMETRY_FRSKY

#define USE_GYRO_BIQUAD_RC_FIR2
#define USE_MR_BRAKING_MODE

#if defined(STM_FAST_TARGET)
#define SCHEDULER_DELAY_LIMIT           10
#else
#define SCHEDULER_DELAY_LIMIT           100
#endif

#if (FLASH_SIZE > 256)
#define USE_EXTENDED_CMS_MENUS
#define USE_UAV_INTERCONNECT
#define USE_RX_UIB
#define USE_WATCHDOG
#endif

#if (FLASH_SIZE > 128)
#define NAV_FIXED_WING_LANDING
#define USE_AUTOTUNE_FIXED_WING
#define USE_ASYNC_GYRO_PROCESSING
#define USE_DEBUG_TRACE
#define USE_BOOTLOG
#define BOOTLOG_DESCRIPTIONS
#define USE_STATS
#define USE_GYRO_NOTCH_1
#define USE_GYRO_NOTCH_2
#define USE_DTERM_NOTCH
#define USE_ACC_NOTCH
#define USE_CMS
#define CMS_MENU_OSD
#define USE_DASHBOARD
#define USE_OLED_UG2864
#define USE_MSP_DISPLAYPORT
#define DASHBOARD_ARMED_BITMAP
#define USE_GPS_PROTO_NMEA
#define USE_GPS_PROTO_I2C_NAV
#define USE_GPS_PROTO_NAZA
#define USE_GPS_PROTO_UBLOX_NEO7PLUS
#define USE_GPS_PROTO_MTK
#define NAV_AUTO_MAG_DECLINATION
#define NAV_GPS_GLITCH_DETECTION
#define NAV_NON_VOLATILE_WAYPOINT_STORAGE
#define USE_TELEMETRY_HOTT
#define USE_TELEMETRY_IBUS
#define USE_TELEMETRY_MAVLINK
#define USE_TELEMETRY_SMARTPORT
#define USE_TELEMETRY_CRSF
#define USE_MSP_OVER_TELEMETRY
// These are rather exotic serial protocols
#define USE_RX_MSP
#define USE_SERIALRX_SUMD
#define USE_SERIALRX_SUMH
#define USE_SERIALRX_XBUS
#define USE_SERIALRX_JETIEXBUS
#define USE_SERIALRX_CRSF
#define USE_PWM_SERVO_DRIVER
#define USE_SERIAL_PASSTHROUGH
#define USE_PWM_DRIVER_PCA9685
#define NAV_MAX_WAYPOINTS       60
#define MAX_BOOTLOG_ENTRIES     64
#define USE_RCDEVICE
#define USE_PITOT
#define USE_PITOT_ADC

//Enable VTX controll
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_VTX_TRAMP

//Enable DST calculations
#define RTC_AUTOMATIC_DST
// Wind estimator
#define USE_WIND_ESTIMATOR

#else // FLASH_SIZE < 128
#define CLI_MINIMAL_VERBOSITY
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#define SKIP_CLI_RESOURCES
#define NAV_MAX_WAYPOINTS       30
#define MAX_BOOTLOG_ENTRIES     32
#endif
