/*
 * This file is part of INAV
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

// iNav specific IDs start from 0x2000
// See https://github.com/iNavFlight/inav/wiki/MSP-V2#msp-v2-message-catalogue

#define MSP2_INAV_STATUS                        0x2000
#define MSP2_INAV_OPTICAL_FLOW                  0x2001
#define MSP2_INAV_ANALOG                        0x2002
#define MSP2_INAV_MISC                          0x2003
#define MSP2_INAV_SET_MISC                      0x2004
#define MSP2_INAV_BATTERY_CONFIG                0x2005
#define MSP2_INAV_SET_BATTERY_CONFIG            0x2006
#define MSP2_INAV_RATE_PROFILE                  0x2007
#define MSP2_INAV_SET_RATE_PROFILE              0x2008
#define MSP2_INAV_AIR_SPEED                     0x2009
#define MSP2_INAV_OUTPUT_MAPPING                0x200A
#define MSP2_INAV_MC_BRAKING                    0x200B
#define MSP2_INAV_SET_MC_BRAKING                0x200C

#define MSP2_INAV_MIXER                         0x2010
#define MSP2_INAV_SET_MIXER                     0x2011

#define MSP2_INAV_OSD_LAYOUTS                   0x2012
#define MSP2_INAV_OSD_SET_LAYOUT_ITEM           0x2013
#define MSP2_INAV_OSD_ALARMS                    0x2014
#define MSP2_INAV_OSD_SET_ALARMS                0x2015
#define MSP2_INAV_OSD_PREFERENCES               0x2016
#define MSP2_INAV_OSD_SET_PREFERENCES           0x2017

#define MSP2_INAV_SELECT_BATTERY_PROFILE        0x2018

#define MSP2_INAV_DEBUG                         0x2019

#define MSP2_INAV_NAMES                         0x2020
#define MSP2_INAV_SET_NAMES                     0x2021
