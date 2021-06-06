/*
 * This file is part of INAV.
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

#include <stdint.h>

#include "platform.h"

#include "common/constants.h"
#include "common/printf.h"

#include "flight/imu.h"

#include "io/osd.h"
#include "io/osd_hud.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "navigation/navigation.h"

#ifdef USE_OSD

#define HUD_DRAWN_MAXCHARS 54 // 8 POI (1 home, 4 radar, 3 WP) x 7 chars max for each, minus 2 for H

static int8_t hud_drawn[HUD_DRAWN_MAXCHARS][2];
static int8_t hud_drawn_pt;

/*
 * Overwrite all previously written positions on the OSD with a blank
 */
void osdHudClear(void)
{
    for (int i = 0; i < HUD_DRAWN_MAXCHARS; i++) {
        if (hud_drawn[i][0] > -1) {
            displayWriteChar(osdGetDisplayPort(), hud_drawn[i][0], hud_drawn[i][1], SYM_BLANK);
            hud_drawn[i][0] = -1;
        }
    }
    hud_drawn_pt = 0;
}

/*
 * Write a single char on the OSD, and record the position for the next loop
 */
static int osdHudWrite(uint8_t px, uint8_t py, uint16_t symb, bool crush)
{
    if (!crush) {
        uint16_t c;
        if (displayReadCharWithAttr(osdGetDisplayPort(), px, py, &c, NULL) && c != SYM_BLANK) {
            return false;
        }
    }

    displayWriteChar(osdGetDisplayPort(), px, py, symb);
    hud_drawn[hud_drawn_pt][0] = px;
    hud_drawn[hud_drawn_pt][1] = py;
    hud_drawn_pt++;
    if (hud_drawn_pt >= HUD_DRAWN_MAXCHARS) hud_drawn_pt = 0;
    return true;
}

/*
 * Constrain an angle between -180 and +180°
 */
static int16_t hudWrap180(int16_t angle)
{
    while (angle < -179) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
}

/*
 * Constrain an angle between 0 and +360°
 */
static int16_t hudWrap360(int16_t angle)
{
    while (angle < 0) angle += 360;
    while (angle > 360) angle -= 360;
    return angle;
}

/*
 * Radar, get the nearest POI
 */
int8_t radarGetNearestPOI(void)
{
    int8_t poi = -1;
    uint16_t min = 10000; // 10kms

    for (int i = 0; i < RADAR_MAX_POIS; i++) {
        if ((radar_pois[i].distance > 0) && (radar_pois[i].distance < min) && (radar_pois[i].state != 2)) {
            min = radar_pois[i].distance;
            poi = i;
        }
    }
    return poi;
}

/*
 * Display a POI as a 3D-marker on the hud
 * Distance (m), Direction (°), Altitude (relative, m, negative means below), Heading (°),
 * Type = 0 : Home point
 * Type = 1 : Radar POI, P1: Heading, P2: Signal
 * Type = 2 : Waypoint, P1: WP number, P2: 1=WP+1, 2=WP+2, 3=WP+3
 */
void osdHudDrawPoi(uint32_t poiDistance, int16_t poiDirection, int32_t poiAltitude, uint8_t poiType, uint16_t poiSymbol, int16_t poiP1, int16_t poiP2)
{
    int poi_x;
    int poi_y;
    uint8_t center_x;
    uint8_t center_y;
    bool poi_is_oos = 0;
    char buff[3];
    int altc = 0;

    uint8_t minX = osdConfig()->hud_margin_h + 2;
    uint8_t maxX = osdGetDisplayPort()->cols - osdConfig()->hud_margin_h - 3;
    uint8_t minY = osdConfig()->hud_margin_v;
    uint8_t maxY = osdGetDisplayPort()->rows - osdConfig()->hud_margin_v - 2;

    osdCrosshairPosition(&center_x, &center_y);

    int16_t error_x = hudWrap180(poiDirection - DECIDEGREES_TO_DEGREES(osdGetHeading()));

    if ((error_x > -(osdConfig()->camera_fov_h / 2)) && (error_x < osdConfig()->camera_fov_h / 2)) { // POI might be in sight, extra geometry needed
        float scaled_x = sin_approx(DEGREES_TO_RADIANS(error_x)) / sin_approx(DEGREES_TO_RADIANS(osdConfig()->camera_fov_h / 2));
        poi_x = center_x + 15 * scaled_x;

        if (poi_x < minX || poi_x > maxX ) { // In camera view, but out of the hud area
            poi_is_oos = 1;
        }
        else { // POI is on sight, compute the vertical
            float poi_angle = atan2_approx(-poiAltitude, poiDistance);
            poi_angle = RADIANS_TO_DEGREES(poi_angle);
            int16_t plane_angle = attitude.values.pitch / 10;
            int camera_angle = osdConfig()->camera_uptilt;
            int16_t error_y = poi_angle - plane_angle + camera_angle;
            float scaled_y = sin_approx(DEGREES_TO_RADIANS(error_y)) / sin_approx(DEGREES_TO_RADIANS(osdConfig()->camera_fov_v / 2));
            poi_y = constrain(center_y + (osdGetDisplayPort()->rows / 2) * scaled_y, minY, maxY - 1);
        }
    }
    else {
        poi_is_oos = 1; // POI is out of camera view for sure
    }

    // Out-of-sight arrows and stacking

    if (poi_is_oos) {
        uint16_t d;
        uint16_t c;

        poi_x = (error_x > 0 ) ? maxX : minX;
        poi_y = center_y - 1;

        if (displayReadCharWithAttr(osdGetDisplayPort(), poi_x, poi_y, &c, NULL) && c != SYM_BLANK) {
            poi_y = center_y - 3;
            while (displayReadCharWithAttr(osdGetDisplayPort(), poi_x, poi_y, &c, NULL) && c != SYM_BLANK && poi_y < maxY - 3) { // Stacks the out-of-sight POI from top to bottom
                poi_y += 2;
            }
        }

        if (error_x > 0 ) {
            d = SYM_HUD_ARROWS_R3 - constrain((180 - error_x) / 45, 0, 2);
            osdHudWrite(poi_x + 2, poi_y, d, 1);
        }
        else {
            d = SYM_HUD_ARROWS_L3 - constrain((180 + error_x) / 45, 0, 2);
            osdHudWrite(poi_x - 2, poi_y, d, 1);
        }
    }

    // Markers

    osdHudWrite(poi_x, poi_y, poiSymbol, 1);

    if (poiType == 1) { // POI from the ESP radar
        error_x = hudWrap360(poiP1 - DECIDEGREES_TO_DEGREES(osdGetHeading()));
        osdHudWrite(poi_x - 1, poi_y, SYM_DIRECTION + ((error_x + 22) / 45) % 8, 1);
        osdHudWrite(poi_x + 1, poi_y, SYM_HUD_SIGNAL_0 + poiP2, 1);
    }
    else if (poiType == 2) { // Waypoint,
        osdHudWrite(poi_x - 1, poi_y, SYM_HUD_ARROWS_U1 + poiP2, 1);
        osdHudWrite(poi_x + 1, poi_y, poiP1, 1);
        }

    // Distance or relative altitude

    if (((millis() / 1700) % 4 == 0) && poiType > 0) { // For Radar and WPs, display the difference in altitude
        altc = ((osd_unit_e)osdConfig()->units == OSD_UNIT_IMPERIAL) ? constrain(CENTIMETERS_TO_FEET(poiAltitude * 100), -99, 99) : constrain(poiAltitude, -99 , 99);
        tfp_sprintf(buff, "%3d", altc);
        buff[0] = (poiAltitude >= 0) ? SYM_VARIO_UP_2A : SYM_VARIO_DOWN_2A;
    }
    else { // Display the distance by default 
        if ((osd_unit_e)osdConfig()->units == OSD_UNIT_IMPERIAL) {
            osdFormatCentiNumber(buff, CENTIMETERS_TO_CENTIFEET(poiDistance * 100), FEET_PER_MILE, 0, 3, 3);
        }
        else {
            osdFormatCentiNumber(buff, poiDistance * 100, METERS_PER_KILOMETER, 0, 3, 3);
        }
    }
    osdHudWrite(poi_x - 1, poi_y + 1, buff[0], 1);
    osdHudWrite(poi_x , poi_y + 1, buff[1], 1);
    osdHudWrite(poi_x + 1, poi_y + 1, buff[2], 1);
}

/*
 * Draw the crosshair
 */
void osdHudDrawCrosshair(displayCanvas_t *canvas, uint8_t px, uint8_t py)
{
    static const uint16_t crh_style_all[] = {
        SYM_AH_CH_LEFT, SYM_AH_CH_CENTER, SYM_AH_CH_RIGHT,
        SYM_AH_CH_AIRCRAFT1, SYM_AH_CH_AIRCRAFT2, SYM_AH_CH_AIRCRAFT3,
        SYM_AH_CH_TYPE3, SYM_AH_CH_TYPE3 + 1, SYM_AH_CH_TYPE3 + 2,
        SYM_AH_CH_TYPE4, SYM_AH_CH_TYPE4 + 1, SYM_AH_CH_TYPE4 + 2,
        SYM_AH_CH_TYPE5, SYM_AH_CH_TYPE5 + 1, SYM_AH_CH_TYPE5 + 2,
        SYM_AH_CH_TYPE6, SYM_AH_CH_TYPE6 + 1, SYM_AH_CH_TYPE6 + 2,
        SYM_AH_CH_TYPE7, SYM_AH_CH_TYPE7 + 1, SYM_AH_CH_TYPE7 + 2,
    };

    // Center on the screen
    if (canvas) {
        displayCanvasContextPush(canvas);
        displayCanvasCtmTranslate(canvas, -canvas->gridElementWidth / 2, -canvas->gridElementHeight / 2);
    }

    uint8_t crh_crosshair = (osd_crosshairs_style_e)osdConfig()->crosshairs_style;

    displayWriteChar(osdGetDisplayPort(), px - 1, py,crh_style_all[crh_crosshair * 3]);
    displayWriteChar(osdGetDisplayPort(), px, py, crh_style_all[crh_crosshair * 3 + 1]);
    displayWriteChar(osdGetDisplayPort(), px + 1, py, crh_style_all[crh_crosshair * 3 + 2]);

    if (canvas) {
        displayCanvasContextPop(canvas);
    }
}


/*
 * Draw the homing arrows around the crosshair
 */
void osdHudDrawHoming(uint8_t px, uint8_t py)
{
    int crh_l = SYM_BLANK;
    int crh_r = SYM_BLANK;
    int crh_u = SYM_BLANK;
    int crh_d = SYM_BLANK;

    int16_t crh_diff_head = hudWrap180(GPS_directionToHome - DECIDEGREES_TO_DEGREES(osdGetHeading()));

    if (crh_diff_head <= -162 || crh_diff_head >= 162) {
        crh_l = SYM_HUD_ARROWS_L3;
        crh_r = SYM_HUD_ARROWS_R3;
    } else if (crh_diff_head > -162 && crh_diff_head <= -126) {
        crh_l = SYM_HUD_ARROWS_L3;
        crh_r = SYM_HUD_ARROWS_R2;
    } else if (crh_diff_head > -126 && crh_diff_head <= -90) {
        crh_l = SYM_HUD_ARROWS_L3;
        crh_r = SYM_HUD_ARROWS_R1;
    } else if (crh_diff_head > -90 && crh_diff_head <= -OSD_HOMING_LIM_H3) {
        crh_l = SYM_HUD_ARROWS_L3;
    } else if (crh_diff_head > -OSD_HOMING_LIM_H3 && crh_diff_head <= -OSD_HOMING_LIM_H2) {
        crh_l = SYM_HUD_ARROWS_L2;
    } else if (crh_diff_head > -OSD_HOMING_LIM_H2 && crh_diff_head <= -OSD_HOMING_LIM_H1) {
        crh_l = SYM_HUD_ARROWS_L1;
    } else if (crh_diff_head >= OSD_HOMING_LIM_H1 && crh_diff_head < OSD_HOMING_LIM_H2) {
        crh_r = SYM_HUD_ARROWS_R1;
    } else if (crh_diff_head >= OSD_HOMING_LIM_H2 && crh_diff_head < OSD_HOMING_LIM_H3) {
        crh_r = SYM_HUD_ARROWS_R2;
    } else if (crh_diff_head >= OSD_HOMING_LIM_H3 && crh_diff_head < 90) {
        crh_r = SYM_HUD_ARROWS_R3;
    } else if (crh_diff_head >= 90 && crh_diff_head < 126) {
        crh_l = SYM_HUD_ARROWS_L1;
        crh_r = SYM_HUD_ARROWS_R3;
    } else if (crh_diff_head >= 126 && crh_diff_head < 162) {
        crh_l = SYM_HUD_ARROWS_L2;
        crh_r = SYM_HUD_ARROWS_R3;
    }

    if (ABS(crh_diff_head) < 90) {

        int32_t crh_altitude = osdGetAltitude() / 100;
        int32_t crh_distance = GPS_distanceToHome;

        float crh_home_angle = atan2_approx(crh_altitude, crh_distance);
        crh_home_angle = RADIANS_TO_DEGREES(crh_home_angle);
        int crh_plane_angle = attitude.values.pitch / 10;
        int crh_camera_angle = osdConfig()->camera_uptilt;
        int crh_diff_vert = crh_home_angle - crh_plane_angle + crh_camera_angle;

        if (crh_diff_vert > -OSD_HOMING_LIM_V2 && crh_diff_vert <= -OSD_HOMING_LIM_V1 ) {
            crh_u = SYM_HUD_ARROWS_U1;
        } else if (crh_diff_vert > -OSD_HOMING_LIM_V3 && crh_diff_vert <= -OSD_HOMING_LIM_V2) {
            crh_u = SYM_HUD_ARROWS_U2;
        } else if (crh_diff_vert <= -OSD_HOMING_LIM_V3) {
            crh_u = SYM_HUD_ARROWS_U3;
        } else if (crh_diff_vert >= OSD_HOMING_LIM_V1  && crh_diff_vert < OSD_HOMING_LIM_V2) {
            crh_d = SYM_HUD_ARROWS_D1;
        } else if (crh_diff_vert >= OSD_HOMING_LIM_V2 && crh_diff_vert < OSD_HOMING_LIM_V3) {
            crh_d = SYM_HUD_ARROWS_D2;
        } else if (crh_diff_vert >= OSD_HOMING_LIM_V3) {
            crh_d = SYM_HUD_ARROWS_D3;
        }
    }

    displayWriteChar(osdGetDisplayPort(), px - 2, py, crh_l);
    displayWriteChar(osdGetDisplayPort(), px + 2, py, crh_r);
    displayWriteChar(osdGetDisplayPort(), px, py - 1, crh_u);
    displayWriteChar(osdGetDisplayPort(), px, py + 1, crh_d);
}

#endif // USE_OSD
