/* @file osd_symbols.h
 * @brief Based on max7456 symbols for the mwosd font set
 *
 * @author Nathan Tsoi nathan@vertile.com
 *
 * Copyright (C) 2016 Nathan Tsoi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#pragma once

#ifdef USE_OSD

#define SYM_RSSI                    0x01  // 001 Icon RSSI
#define SYM_LQ                      0x02  // 002 LQ
#define SYM_LAT                     0x03  // 003 GPS LAT
#define SYM_LON                     0x04  // 004 GPS LON
#define SYM_AZIMUTH                 0x05  // 005 Azimuth
#define SYM_TELEMETRY_0             0x06  // 006 Antenna tracking telemetry
#define SYM_TELEMETRY_1             0x07  // 007 Antenna tracking telemetry
#define SYM_SAT_L                   0x08  // 008 Sats left
#define SYM_SAT_R                   0x09  // 009 Sats right
#define SYM_HOME_NEAR               0x0A  // 010 Home, near
#define SYM_DEGREES                 0x0B  // 011 ° heading angle
#define SYM_HEADING                 0x0C  // 012 Compass Heading symbol
#define SYM_SCALE                   0x0D  // 013 Map scale
#define SYM_HDP_L                   0x0E  // 014 HDOP left
#define SYM_HDP_R                   0x0F  // 015 HDOP right
#define SYM_HOME                    0x10  // 016 Home icon
#define SYM_2RSS                    0x11  // 017 RSSI 2
#define SYM_DB                      0x12  // 018 dB
#define SYM_DBM                     0x13  // 019 dBm
#define SYM_SNR                     0x14  // 020 SNR

#define SYM_AH_DECORATION_UP        0x15  // 021 Arrow up AHI
#define SYM_AH_DECORATION_DOWN      0x16  // 022 Arrow down AHI
#define SYM_DIRECTION               0x17  // 023 to 030, directional little arrows

#define SYM_VOLT                    0x1F  // 031 VOLTS
#define SYM_MAH                     0x99  // 153 mAh
//                                  0x21  // 033 ASCII !
#define SYM_AH_KM                   0x22  // 034 Ah/km
//                                  0x23  // 035 ASCII #
#define SYM_AH_MI                   0x24  // 036 Ah/mi
//                                  0x25  // 037 ASCII %
#define SYM_AH_NM       		    0x26  // 038 Ah/NM
#define SYM_MAH_MI_0                0x27  // 039 mAh/mi left
//                                  0x28  // 040 to 062 ASCII
#define SYM_MAH_MI_1                0x3F  // 063 mAh/mi right
//                                  0x40  // 064 to 095 ASCII
#define SYM_MAH_NM_0                0x60  // 096 mAh/NM left
#define SYM_MAH_NM_1                0x61  // 097 mAh/NM right
#define SYM_MAH_KM_0                0x6B  // 107 MAH/KM left
#define SYM_MAH_KM_1                0x6C  // 108 MAH/KM right
#define SYM_MILLIOHM                0x62  // 098 battery impedance Mohm

#define SYM_BATT_FULL               0x63  // 099 Battery full
#define SYM_BATT_5                  0x64  // 100 Battery
#define SYM_BATT_4                  0x65  // 101 Battery
#define SYM_BATT_3                  0x66  // 102 Battery
#define SYM_BATT_2                  0x67  // 103 Battery
#define SYM_BATT_1                  0x68  // 104 Battery
#define SYM_BATT_EMPTY              0x69  // 105 Battery empty

#define SYM_AMP                     0x6A  // 106 AMPS
#define SYM_WH                      0x6D  // 109 WH
#define SYM_WH_KM                   0x6E  // 110 WH/KM
#define SYM_WH_MI                   0x6F  // 111 WH/MI
#define SYM_WH_NM       		    0x70  // 112 Wh/NM
#define SYM_WATT                    0x71  // 113 WATTS
#define SYM_MW                      0x72  // 114 mW
#define SYM_KILOWATT                0x73  // 115 kW

#define SYM_FT                      0x74  // 116 FEET
#define SYM_TRIP_DIST               0x75  // 117 Trip distance
#define SYM_TOTAL                   0x75  // 117 Total
#define SYM_ALT_M                   0x76  // 118 ALT M
#define SYM_ALT_KM                  0x77  // 119 ALT KM
#define SYM_ALT_FT                  0x78  // 120 ALT FT
#define SYM_ALT_KFT                 0x79  // 121 Alt KFT
#define SYM_DIST_M                  0x7A  // 122 DIS M
//                                  0x7B  // 123 to 125 ASCII
#define SYM_DIST_KM                 0x7E  // 126 DIST KM
#define SYM_DIST_FT                 0x7F  // 127 DIST FT
#define SYM_DIST_MI                 0x80  // 128 DIST MI
#define SYM_DIST_NM                 0x81  // 129 DIST NM
#define SYM_M                       0x82  // 130 M
#define SYM_KM                      0x83  // 131 KM
#define SYM_MI                      0x84  // 132 MI
#define SYM_NM                      0x85  // 133 NM

#define SYM_WIND_HORIZONTAL         0x86  // 134 Air speed horizontal
#define SYM_WIND_VERTICAL           0x87  // 135 Air speed vertical
#define SYM_3D_KMH                  0x88  // 136 KM/H 3D
#define SYM_3D_MPH                  0x89  // 137 MPH 3D
#define SYM_3D_KT                   0x8A  // 138 Knots 3D
#define SYM_RPM                     0x8B  // 139 RPM
#define SYM_AIR                     0x8C  // 140 Air speed
#define SYM_FTS                     0x8D  // 141 FT/S
#define SYM_100FTM                  0x8E  // 142 100 Feet per Min
#define SYM_MS                      0x8F  // 143 M/S
#define SYM_KMH                     0x90  // 144 KM/H
#define SYM_MPH                     0x91  // 145 MPH
#define SYM_KT                      0x92  // 146 Knots

#define SYM_HEADING_DIVIDED_LINE    0x93  // 147 Heading Graphic
#define SYM_HEADING_LINE            0x94  // 148 Heading Graphic

#define SYM_THR                     0x95  // 149 Throttle
#define SYM_TEMP_F                  0x96  // 150 °F
#define SYM_TEMP_C                  0x97  // 151 °C
//                                  0x98  // 152 Home point map
#define SYM_BLANK                   0x20  // 32 blank (space)
#define SYM_ON_H                    0x9A  // 154 ON HR
#define SYM_FLY_H                   0x9B  // 155 FLY HR
#define SYM_ON_M                    0x9E  // 158 On MIN
#define SYM_FLY_M                   0x9F  // 159 FL MIN
#define SYM_GLIDESLOPE              0x9C  // 156 Glideslope
#define SYM_WAYPOINT                0x9D  // 157 Waypoint
#define SYM_CLOCK                   0xA0  // 160 Clock

#define SYM_ZERO_HALF_TRAILING_DOT  0xA1  // 161 to 170 Numbers with trailing dot
#define SYM_ZERO_HALF_LEADING_DOT   0xB1  // 177 to 186 Numbers with leading dot

#define SYM_AUTO_THR0               0xAB  // 171 Auto-throttle left
#define SYM_AUTO_THR1               0xAC  // 172 Auto-throttle right
#define SYM_ROLL_LEFT               0xAD  // 173 Sym roll left
#define SYM_ROLL_LEVEL              0xAE  // 174 Sym roll horizontal
#define SYM_ROLL_RIGHT              0xAF  // 175 Sym roll right
#define SYM_PITCH_UP                0xB0  // 176 Pitch up
#define SYM_PITCH_DOWN              0xBB  // 187 Pitch down
#define SYM_GFORCE                  0xBC  // 188 Gforce (all axis)
#define SYM_GFORCE_X                0xBD  // 189 Gforce X
#define SYM_GFORCE_Y                0xBE  // 190 Gforce Y
#define SYM_GFORCE_Z                0xBF  // 191 Gforce Z

#define SYM_BARO_TEMP               0xC0  // 192
#define SYM_IMU_TEMP                0xC1  // 193
#define SYM_TEMP                    0xC2  // 194 Thermometer icon
#define SYM_TEMP_SENSOR_FIRST       0xC2  // 194
#define SYM_ESC_TEMP                0xC3  // 195
#define SYM_TEMP_SENSOR_LAST        0xC7  // 199
#define TEMP_SENSOR_SYM_COUNT       (SYM_TEMP_SENSOR_LAST - SYM_TEMP_SENSOR_FIRST + 1)

#define SYM_HEADING_N               0xC8  // 200 Heading Graphic north
#define SYM_HEADING_S               0xC9  // 201 Heading Graphic south
#define SYM_HEADING_E               0xCA  // 202 Heading Graphic east
#define SYM_HEADING_W               0xCB  // 203 Heading Graphic west


#define SYM_LOGO_START              0x101 // 257 to 280, INAV logo
#define SYM_LOGO_WIDTH              6
#define SYM_LOGO_HEIGHT             4

#define SYM_AH_LEFT                 0x12C // 300 Arrow left
#define SYM_AH_RIGHT                0x12D // 301 Arrow right
#define SYM_AH_DECORATION_MIN       0x12E // 302 to 307 Scrolling
#define SYM_AH_DECORATION           0x131 // 305 Scrolling
#define SYM_AH_DECORATION_MAX       0x133 // 307 Scrolling
#define SYM_AH_DECORATION_COUNT (SYM_AH_DECORATION_MAX - SYM_AH_DECORATION_MIN + 1) // Scrolling

#define SYM_AH_CH_LEFT              0x13A // 314 Crossair left
#define SYM_AH_CH_RIGHT             0x13B // 315 Crossair right

#define SYM_ARROW_UP                0x13C // 316 Direction arrow 0°
#define SYM_ARROW_2                 0x13D // 317 Direction arrow 22.5°
#define SYM_ARROW_3                 0x13E // 318 Direction arrow 45°
#define SYM_ARROW_4                 0x13F // 319 Direction arrow 67.5°
#define SYM_ARROW_RIGHT             0x140 // 320 Direction arrow 90°
#define SYM_ARROW_6                 0x141 // 321 Direction arrow 112.5°
#define SYM_ARROW_7                 0x142 // 322 Direction arrow 135°
#define SYM_ARROW_8                 0x143 // 323 Direction arrow 157.5°
#define SYM_ARROW_DOWN              0x144 // 324 Direction arrow 180°
#define SYM_ARROW_10                0x145 // 325 Direction arrow 202.5°
#define SYM_ARROW_11                0x146 // 326 Direction arrow 225°
#define SYM_ARROW_12                0x147 // 327 Direction arrow 247.5°
#define SYM_ARROW_LEFT              0x148 // 328 Direction arrow 270°
#define SYM_ARROW_14                0x149 // 329 Direction arrow 292.5°
#define SYM_ARROW_15                0x14A // 330 Direction arrow 315°
#define SYM_ARROW_16                0x14B // 331 Direction arrow 337.5°

#define SYM_AH_H_START              0x14C // 332 to 340 Horizontal AHI
#define SYM_AH_V_START              0x15A // 346 to 351 Vertical AHI

#define SYM_VARIO_UP_2A             0x155 // 341 Vario up up
#define SYM_VARIO_UP_1A             0x156 // 342 Vario up
#define SYM_VARIO_DOWN_1A           0x157 // 343 Vario down
#define SYM_VARIO_DOWN_2A           0x158 // 344 Vario down down
#define SYM_ALT                     0x159 // 345 ALT

#define SYM_HUD_SIGNAL_0            0x160 // 352 Hud signal icon Lost
#define SYM_HUD_SIGNAL_1            0x161 // 353 Hud signal icon 25%
#define SYM_HUD_SIGNAL_2            0x162 // 354 Hud signal icon 50%
#define SYM_HUD_SIGNAL_3            0x163 // 355 Hud signal icon 75%
#define SYM_HUD_SIGNAL_4            0x164 // 356 Hud signal icon 100%

#define SYM_HOME_DIST 	            0x165 // 357 DIST
#define SYM_AH_CH_CENTER            0x166 // 358 Crossair center

#define SYM_AH_CH_TYPE3             0x190 // 400 to 402, crosshair 3
#define SYM_AH_CH_TYPE4             0x193 // 403 to 405, crosshair 4
#define SYM_AH_CH_TYPE5             0x196 // 406 to 408, crosshair 5
#define SYM_AH_CH_TYPE6             0x199 // 409 to 411, crosshair 6
#define SYM_AH_CH_TYPE7             0x19C // 412 to 414, crosshair 7
#define SYM_AH_CH_TYPE8             0x19F // 415 to 417, crosshair 8

#define SYM_AH_CH_AIRCRAFT0         0x1A2 // 418 Crossair aircraft left
#define SYM_AH_CH_AIRCRAFT1         0x1A3 // 419 Crossair aircraft
#define SYM_AH_CH_AIRCRAFT2         0x1A4 // 420 Crossair aircraft center
#define SYM_AH_CH_AIRCRAFT3         0x1A5 // 421 Crossair aircraft
#define SYM_AH_CH_AIRCRAFT4         0x1A6 // 422 Crossair aircraft RIGHT

#define SYM_HUD_ARROWS_L1           0x1AE // 430 1 arrow left
#define SYM_HUD_ARROWS_L2           0x1AF // 431 2 arrows left
#define SYM_HUD_ARROWS_L3           0x1B0 // 432 3 arrows left
#define SYM_HUD_ARROWS_R1           0x1B1 // 433 1 arrow right
#define SYM_HUD_ARROWS_R2           0x1B2 // 434 2 arrows right
#define SYM_HUD_ARROWS_R3           0x1B3 // 435 3 arrows right
#define SYM_HUD_ARROWS_U1           0x1B4 // 436 1 arrow up
#define SYM_HUD_ARROWS_U2           0x1B5 // 437 2 arrows up
#define SYM_HUD_ARROWS_U3           0x1B6 // 438 3 arrows up
#define SYM_HUD_ARROWS_D1           0x1B7 // 439 1 arrow down
#define SYM_HUD_ARROWS_D2           0x1B8 // 440 2 arrows down
#define SYM_HUD_ARROWS_D3           0x1B9 // 441 3 arrows down

#else

#define TEMP_SENSOR_SYM_COUNT 0

#endif // USE_OSD
