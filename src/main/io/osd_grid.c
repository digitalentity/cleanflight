/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include <math.h>

#include "platform.h"

#if defined(USE_OSD)

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "io/osd.h"
#include "io/osd_common.h"

#include "navigation/navigation.h"

typedef enum {
    OSD_SIDEBAR_ARROW_NONE,
    OSD_SIDEBAR_ARROW_UP,
    OSD_SIDEBAR_ARROW_DOWN,
} osd_sidebar_arrow_e;

typedef struct osd_sidebar_s {
    int32_t offset;
    timeMs_t updated;
    osd_sidebar_arrow_e arrow;
    uint8_t idle;
} osd_sidebar_t;

void osdGridDrawVario(displayPort_t *display, unsigned gx, unsigned gy, float zvel)
{
    int v = zvel / OSD_VARIO_CM_S_PER_ARROW;
    uint8_t vchars[] = {SYM_BLANK, SYM_BLANK, SYM_BLANK, SYM_BLANK, SYM_BLANK};

    if (v >= 6)
        vchars[0] = SYM_VARIO_UP_2A;
    else if (v == 5)
        vchars[0] = SYM_VARIO_UP_1A;
    if (v >=4)
        vchars[1] = SYM_VARIO_UP_2A;
    else if (v == 3)
        vchars[1] = SYM_VARIO_UP_1A;
    if (v >=2)
        vchars[2] = SYM_VARIO_UP_2A;
    else if (v == 1)
        vchars[2] = SYM_VARIO_UP_1A;
    if (v <= -2)
        vchars[2] = SYM_VARIO_DOWN_2A;
    else if (v == -1)
        vchars[2] = SYM_VARIO_DOWN_1A;
    if (v <= -4)
        vchars[3] = SYM_VARIO_DOWN_2A;
    else if (v == -3)
        vchars[3] = SYM_VARIO_DOWN_1A;
    if (v <= -6)
        vchars[4] = SYM_VARIO_DOWN_2A;
    else if (v == -5)
        vchars[4] = SYM_VARIO_DOWN_1A;

    displayWriteChar(display, gx, gy, vchars[0]);
    displayWriteChar(display, gx, gy + 1, vchars[1]);
    displayWriteChar(display, gx, gy + 2, vchars[2]);
    displayWriteChar(display, gx, gy + 3, vchars[3]);
    displayWriteChar(display, gx, gy + 4, vchars[4]);
}

void osdGridDrawDirArrow(displayPort_t *display, unsigned gx, unsigned gy, float degrees)
{
    // There are 16 orientations for the direction arrow.
    // so we use 22.5deg per image, where the 1st image is used
    // for [349, 11], the 2nd for [12, 33], etc...
    // Add 11 to the angle, so first character maps to [349, 11]
    int dir = osdGetHeadingAngle(degrees + 11);
    unsigned arrowOffset = dir * 2 / 45;
    displayWriteChar(display, gx, gy, SYM_ARROW_UP + arrowOffset);
}

static float osdGetAspectRatioCorrection(void)
{
    return osdDisplayIsPAL() ? 12.0f/15.0f : 12.0f/18.46f;
}

void osdGridDrawArtificialHorizon(displayPort_t *display, unsigned gx, unsigned gy, float pitchAngle, float rollAngle)
{
    UNUSED(gx);
    UNUSED(gy);

    uint8_t elemPosX;
    uint8_t elemPosY;

    osdCrosshairPosition(&elemPosX, &elemPosY);

    // Store the positions we draw over to erase only these at the next iteration
    static int8_t previous_written[OSD_AHI_PREV_SIZE];
    static int8_t previous_orient = -1;

    const float pitch_rad_to_char = (float)(OSD_AHI_HEIGHT / 2 + 0.5) / DEGREES_TO_RADIANS(osdConfig()->ahi_max_pitch);

    const float ky = sin_approx(rollAngle);
    const float kx = cos_approx(rollAngle);
    const float ratio = osdGetAspectRatioCorrection();

    if (previous_orient != -1) {
        for (int i = 0; i < OSD_AHI_PREV_SIZE; ++i) {
            if (previous_written[i] > -1) {
                int8_t dx = (previous_orient ? previous_written[i] : i) - OSD_AHI_PREV_SIZE / 2;
                int8_t dy = (previous_orient ? i : previous_written[i]) - OSD_AHI_PREV_SIZE / 2;
                displayWriteChar(display, elemPosX + dx, elemPosY - dy, SYM_BLANK);
                previous_written[i] = -1;
            }
        }
    }

    if (fabsf(ky) < fabsf(kx)) {

        previous_orient = 0;

        for (int8_t dx = -OSD_AHI_WIDTH / 2; dx <= OSD_AHI_WIDTH / 2; dx++) {
            float fy = (ratio * dx) * (ky / kx) + pitchAngle * pitch_rad_to_char + 0.49f;
            int8_t dy = floorf(fy);
            const uint8_t chX = elemPosX + dx, chY = elemPosY - dy;
            uint16_t c;

            if ((dy >= -OSD_AHI_HEIGHT / 2) && (dy <= OSD_AHI_HEIGHT / 2) && displayReadCharWithAttr(display, chX, chY, &c, NULL) && (c == SYM_BLANK)) {
                c = SYM_AH_H_START + ((OSD_AHI_H_SYM_COUNT - 1) - (uint8_t)((fy - dy) * OSD_AHI_H_SYM_COUNT));
                displayWriteChar(display, elemPosX + dx, elemPosY - dy, c);
                previous_written[dx + OSD_AHI_PREV_SIZE / 2] = dy + OSD_AHI_PREV_SIZE / 2;
            }
        }

    } else {

        previous_orient = 1;

        for (int8_t dy = -OSD_AHI_HEIGHT / 2; dy <= OSD_AHI_HEIGHT / 2; dy++) {
            const float fx = ((dy / ratio) - pitchAngle * pitch_rad_to_char) * (kx / ky) + 0.5f;
            const int8_t dx = floorf(fx);
            const uint8_t chX = elemPosX + dx, chY = elemPosY - dy;
            uint16_t c;

            if ((dx >= -OSD_AHI_WIDTH / 2) && (dx <= OSD_AHI_WIDTH / 2) && displayReadCharWithAttr(display, chX, chY, &c, NULL) && (c == SYM_BLANK)) {
                c = SYM_AH_V_START + (fx - dx) * OSD_AHI_V_SYM_COUNT;
                displayWriteChar(display, chX, chY, c);
                previous_written[dy + OSD_AHI_PREV_SIZE / 2] = dx + OSD_AHI_PREV_SIZE / 2;
            }
        }
    }
}

void osdGridDrawHeadingGraph(displayPort_t *display, unsigned gx, unsigned gy, int heading)
{
    static const uint8_t graph[] = {
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_S,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_W,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_N,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_S,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_W,
        SYM_HEADING_LINE,
    };
    char buf[OSD_HEADING_GRAPH_WIDTH + 1];
    int16_t h = DECIDEGREES_TO_DEGREES(heading);
    if (h >= 180) {
        h -= 360;
    }
    int hh = h * 4;
    hh = hh + 720 + 45;
    hh = hh / 90;
    memcpy_fn(buf, graph + hh + 1, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    displayWrite(display, gx, gy, buf);
}

static uint8_t osdUpdateSidebar(osd_sidebar_scroll_e scroll, osd_sidebar_t *sidebar, timeMs_t currentTimeMs)
{
    // Scroll between SYM_AH_DECORATION_MIN and SYM_AH_DECORATION_MAX.
    // Zero scrolling should draw SYM_AH_DECORATION.
    uint8_t decoration = SYM_AH_DECORATION;
    int offset = 0;
    int steps;
    switch (scroll) {
        case OSD_SIDEBAR_SCROLL_NONE:
            sidebar->arrow = OSD_SIDEBAR_ARROW_NONE;
            sidebar->offset = 0;
            return decoration;
        case OSD_SIDEBAR_SCROLL_ALTITUDE:
            // Move 1 char for every 20cm
            offset = osdGetAltitude();
            steps = offset / 20;
            break;
        case OSD_SIDEBAR_SCROLL_SPEED:
#if defined(USE_GPS)
            offset = gpsSol.groundSpeed;
#endif
            // Move 1 char for every 20 cm/s
            steps = offset / 20;
            break;
        case OSD_SIDEBAR_SCROLL_HOME_DISTANCE:
#if defined(USE_GPS)
            offset = GPS_distanceToHome;
#endif
            // Move 1 char for every 5m
            steps = offset / 5;
            break;
    }
    if (offset) {
        decoration -= steps % SYM_AH_DECORATION_COUNT;
        if (decoration > SYM_AH_DECORATION_MAX) {
            decoration -= SYM_AH_DECORATION_COUNT;
        } else if (decoration < SYM_AH_DECORATION_MIN) {
            decoration += SYM_AH_DECORATION_COUNT;
        }
    }
    if (currentTimeMs - sidebar->updated > 100) {
        if (offset > sidebar->offset) {
            sidebar->arrow = OSD_SIDEBAR_ARROW_UP;
            sidebar->idle = 0;
        } else if (offset < sidebar->offset) {
            sidebar->arrow = OSD_SIDEBAR_ARROW_DOWN;
            sidebar->idle = 0;
        } else {
            if (sidebar->idle > 3) {
                sidebar->arrow = OSD_SIDEBAR_ARROW_NONE;
            } else {
                sidebar->idle++;
            }
        }
        sidebar->offset = offset;
        sidebar->updated = currentTimeMs;
    }
    return decoration;
}

void osdGridDrawSidebars(displayPort_t *display)
{
    uint8_t elemPosX;
    uint8_t elemPosY;

    osdCrosshairPosition(&elemPosX, &elemPosY);

    static osd_sidebar_t left;
    static osd_sidebar_t right;

    timeMs_t currentTimeMs = millis();
    uint8_t leftDecoration = osdUpdateSidebar(osdConfig()->left_sidebar_scroll, &left, currentTimeMs);
    uint8_t rightDecoration = osdUpdateSidebar(osdConfig()->right_sidebar_scroll, &right, currentTimeMs);

    const int hudwidth = OSD_AH_SIDEBAR_WIDTH_POS;
    const int hudheight = osdConfig()->sidebar_height;

    // Arrows
    if (osdConfig()->sidebar_scroll_arrows) {
        displayWriteChar(display, elemPosX - hudwidth, elemPosY - hudheight - 1,
            left.arrow == OSD_SIDEBAR_ARROW_UP ? SYM_AH_DECORATION_UP : SYM_BLANK);

        displayWriteChar(display, elemPosX + hudwidth, elemPosY - hudheight - 1,
            right.arrow == OSD_SIDEBAR_ARROW_UP ? SYM_AH_DECORATION_UP : SYM_BLANK);

        displayWriteChar(display, elemPosX - hudwidth, elemPosY + hudheight + 1,
            left.arrow == OSD_SIDEBAR_ARROW_DOWN ? SYM_AH_DECORATION_DOWN : SYM_BLANK);

        displayWriteChar(display, elemPosX + hudwidth, elemPosY + hudheight + 1,
            right.arrow == OSD_SIDEBAR_ARROW_DOWN ? SYM_AH_DECORATION_DOWN : SYM_BLANK);
    }

    // Draw AH sides
    int leftX = MAX(elemPosX - hudwidth - osdConfig()->sidebar_horizontal_offset, 0);
    int rightX = MIN(elemPosX + hudwidth + osdConfig()->sidebar_horizontal_offset, display->cols - 1);
    if (osdConfig()->sidebar_height) {
        for (int y = -hudheight; y <= hudheight; y++) {
            displayWriteChar(display, leftX, elemPosY + y, leftDecoration);
            displayWriteChar(display, rightX, elemPosY + y, rightDecoration);
        }
    }
    // AH level indicators
    displayWriteChar(display, leftX + 1, elemPosY, SYM_AH_RIGHT);
    displayWriteChar(display, rightX - 1, elemPosY, SYM_AH_LEFT);
}

#endif
