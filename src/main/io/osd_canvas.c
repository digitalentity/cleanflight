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

#if defined(USE_CANVAS)

#define AHI_MIN_DRAW_INTERVAL_MS 50
#define AHI_MAX_DRAW_INTERVAL_MS 1000
#define AHI_CROSSHAIR_MARGIN 6

#define SIDEBAR_REDRAW_INTERVAL_MS 100
#define WIDGET_SIDEBAR_LEFT_INSTANCE 0
#define WIDGET_SIDEBAR_RIGHT_INSTANCE 1

#include "common/constants.h"
#include "common/log.h"
#include "common/maths.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/display_canvas.h"
#include "drivers/display_widgets.h"
#include "drivers/osd.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "io/osd_common.h"
#include "io/osd.h"

#include "navigation/navigation.h"

#define OSD_CANVAS_VARIO_ARROWS_PER_SLOT 2.0f

static void osdCanvasVarioRect(int *y, int *h, displayCanvas_t *canvas, int midY, float zvel)
{
    int maxHeight = ceilf(OSD_VARIO_HEIGHT_ROWS /OSD_CANVAS_VARIO_ARROWS_PER_SLOT) * canvas->gridElementHeight;
    // We use font characters with 2 arrows per slot
    int height = MIN(fabsf(zvel) / (OSD_VARIO_CM_S_PER_ARROW * OSD_CANVAS_VARIO_ARROWS_PER_SLOT) * canvas->gridElementHeight, maxHeight);
    if (zvel >= 0) {
        *y = midY - height;
    } else {
        *y = midY;
    }
    *h = height;
}

void osdCanvasDrawVario(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float zvel)
{
    UNUSED(display);

    static float prev = 0;

    if (fabsf(prev - zvel) < (OSD_VARIO_CM_S_PER_ARROW / (OSD_CANVAS_VARIO_ARROWS_PER_SLOT * 10))) {
        return;
    }

    // Make sure we clear the grid buffer
    uint8_t gx;
    uint8_t gy;
    osdDrawPointGetGrid(&gx, &gy, display, canvas, p);
    osdGridBufferClearGridRect(gx, gy, 1, OSD_VARIO_HEIGHT_ROWS);

    int midY = gy * canvas->gridElementHeight + (OSD_VARIO_HEIGHT_ROWS * canvas->gridElementHeight) / 2;
    // Make sure we're aligned with the center-ish of the grid based variant
    midY -= canvas->gridElementHeight;

    int x = gx * canvas->gridElementWidth;
    int w = canvas->gridElementWidth;
    int y;
    int h;

    osdCanvasVarioRect(&y, &h, canvas, midY, zvel);

    if (signbit(prev) != signbit(zvel) || fabsf(prev) > fabsf(zvel)) {
        // New rectangle doesn't overwrite the old one, we need to erase
        int py;
        int ph;
        osdCanvasVarioRect(&py, &ph, canvas, midY, prev);
        if (ph != h) {
            displayCanvasClearRect(canvas, x, py, w, ph);
        }
    }
    displayCanvasClipToRect(canvas, x, y, w, h);
    if (zvel > 0) {
        for (int yy = midY - canvas->gridElementHeight; yy > midY - h - canvas->gridElementHeight; yy -= canvas->gridElementHeight) {
            displayCanvasDrawCharacter(canvas, x, yy, SYM_VARIO_UP_2A, DISPLAY_CANVAS_BITMAP_OPT_ERASE_TRANSPARENT);
        }
    } else {
        for (int yy = midY; yy < midY + h; yy += canvas->gridElementHeight) {
            displayCanvasDrawCharacter(canvas, x, yy, SYM_VARIO_DOWN_2A, DISPLAY_CANVAS_BITMAP_OPT_ERASE_TRANSPARENT);
        }
    }
    prev = zvel;
}

void osdCanvasDrawDirArrow(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float degrees)
{
    UNUSED(display);

    const int top = 6;
    const int topInset = -2;
    const int bottom = -6;
    const int width = 5;
    // Since grid slots are not square, when we rotate the arrow
    // it overflows horizontally a bit. Making a square-ish arrow
    // doesn't look good, so it's better to overstep the grid slot
    // boundaries a bit and then clean after ourselves.
    const int overflow = 3;

    int px;
    int py;
    osdDrawPointGetPixels(&px, &py, display, canvas, p);

    displayCanvasClearRect(canvas, px - overflow, py, canvas->gridElementWidth + overflow * 2, canvas->gridElementHeight);

    displayCanvasSetFillColor(canvas, DISPLAY_CANVAS_COLOR_WHITE);
    displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_BLACK);

    displayCanvasCtmRotate(canvas, -DEGREES_TO_RADIANS(180 + degrees));
    displayCanvasCtmTranslate(canvas, px + canvas->gridElementWidth / 2, py + canvas->gridElementHeight / 2);

    // Main triangle
    displayCanvasFillStrokeTriangle(canvas, 0, top, width, bottom, -width, bottom);
    // Inset triangle
    displayCanvasSetFillColor(canvas, DISPLAY_CANVAS_COLOR_TRANSPARENT);
    displayCanvasFillTriangle(canvas, 0, topInset, width + 1, bottom - 1, -width, bottom - 1);
    // Draw bottom strokes of the triangle
    displayCanvasMoveToPoint(canvas, -width, bottom - 1);
    displayCanvasStrokeLineToPoint(canvas, 0, topInset);
    displayCanvasStrokeLineToPoint(canvas, width, bottom - 1);
}

static void osdDrawArtificialHorizonLevelLine(displayCanvas_t *canvas, int width, int pos, int margin)
{
    displayCanvasSetLineOutlineType(canvas, DISPLAY_CANVAS_OUTLINE_TYPE_BOTTOM);

    int yoff = pos >= 0 ? 10 : -10;
    int yc = -pos - 1;
    int sz = width / 2;

    // Horizontal strokes
    displayCanvasMoveToPoint(canvas, -sz, yc);
    displayCanvasStrokeLineToPoint(canvas, -margin, yc);
    displayCanvasMoveToPoint(canvas, sz, yc);
    displayCanvasStrokeLineToPoint(canvas, margin, yc);

    // Vertical strokes
    displayCanvasSetLineOutlineType(canvas, DISPLAY_CANVAS_OUTLINE_TYPE_LEFT);
    displayCanvasMoveToPoint(canvas, -sz, yc);
    displayCanvasStrokeLineToPoint(canvas, -sz, yc + yoff);
    displayCanvasSetLineOutlineType(canvas, DISPLAY_CANVAS_OUTLINE_TYPE_RIGHT);
    displayCanvasMoveToPoint(canvas, sz, yc);
    displayCanvasStrokeLineToPoint(canvas, sz, yc + yoff);
}

static void osdArtificialHorizonRect(displayCanvas_t *canvas, int *lx, int *ty, int *w, int *h)
{
    *w = (OSD_AHI_WIDTH + 1) * canvas->gridElementWidth;
    *h = OSD_AHI_HEIGHT * canvas->gridElementHeight;

    *lx = (canvas->width - *w) / 2;
    *ty = (canvas->height - *h) / 2;
}

static void osdDrawArtificialHorizonShapes(displayCanvas_t *canvas, float pitchAngle, float rollAngle)
{
    int barWidth = (OSD_AHI_WIDTH - 1) * canvas->gridElementWidth;
    int levelBarWidth = barWidth * (3.0/4);
    float pixelsPerDegreeLevel = 3.5f;
    int borderSize = 3;
    char buf[12];

    int lx;
    int ty;
    int maxWidth;
    int maxHeight;

    osdArtificialHorizonRect(canvas, &lx, &ty, &maxWidth, &maxHeight);

    displayCanvasContextPush(canvas);

    int rx = lx + maxWidth;
    int by = ty + maxHeight;

    displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_BLACK);

    displayCanvasMoveToPoint(canvas, lx, ty + borderSize);
    displayCanvasStrokeLineToPoint(canvas, lx, ty);
    displayCanvasStrokeLineToPoint(canvas, lx + borderSize, ty);

    displayCanvasMoveToPoint(canvas, rx, ty + borderSize);
    displayCanvasStrokeLineToPoint(canvas, rx, ty);
    displayCanvasStrokeLineToPoint(canvas, rx - borderSize, ty);

    displayCanvasMoveToPoint(canvas,lx, by - borderSize);
    displayCanvasStrokeLineToPoint(canvas, lx, by);
    displayCanvasStrokeLineToPoint(canvas, lx + borderSize, by);

    displayCanvasMoveToPoint(canvas, rx, by - borderSize);
    displayCanvasStrokeLineToPoint(canvas, rx, by);
    displayCanvasStrokeLineToPoint(canvas, rx - borderSize, by);

    displayCanvasClipToRect(canvas, lx + 1, ty + 1, maxWidth - 2, maxHeight - 2);
    osdGridBufferClearPixelRect(canvas, lx, ty, maxWidth, maxHeight);

    displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_WHITE);
    displayCanvasSetLineOutlineColor(canvas, DISPLAY_CANVAS_COLOR_BLACK);

    // The draw just the 5 bars closest to the current pitch level
    float pitchDegrees = RADIANS_TO_DEGREES(pitchAngle);
    float pitchCenter = roundf(pitchDegrees / 10.0f);
    float pitchOffset = -pitchDegrees * pixelsPerDegreeLevel;
    float translateX = canvas->width / 2;
    float translateY = canvas->height / 2;

    displayCanvasCtmTranslate(canvas, 0, pitchOffset);
    displayCanvasContextPush(canvas);
    displayCanvasCtmRotate(canvas, rollAngle);

    displayCanvasCtmTranslate(canvas, translateX, translateY);

    for (int ii = pitchCenter - 2; ii <= pitchCenter + 2; ii++) {
        if (ii == 0) {
            displayCanvasSetLineOutlineType(canvas, DISPLAY_CANVAS_OUTLINE_TYPE_BOTTOM);
            displayCanvasMoveToPoint(canvas, -barWidth / 2, 0);
            displayCanvasStrokeLineToPoint(canvas, -AHI_CROSSHAIR_MARGIN, 0);
            displayCanvasMoveToPoint(canvas, barWidth / 2, 0);
            displayCanvasStrokeLineToPoint(canvas, AHI_CROSSHAIR_MARGIN, 0);
            continue;
        }

        int pos = ii * 10 * pixelsPerDegreeLevel;
        int margin = (ii > 9 || ii < -9) ? 9 : 6;
        osdDrawArtificialHorizonLevelLine(canvas, levelBarWidth, -pos, margin);
    }

    displayCanvasContextPop(canvas);

    displayCanvasCtmTranslate(canvas, translateX, translateY);
    displayCanvasCtmScale(canvas, 0.5f, 0.5f);

    // Draw line labels
    float sx = sin_approx(rollAngle);
    float sy = cos_approx(-rollAngle);
    for (int ii = pitchCenter - 2; ii <= pitchCenter + 2; ii++) {
        if (ii == 0) {
            continue;
        }

        int level = ii * 10;
        int absLevel = ABS(level);
        itoa(absLevel, buf, 10);
        int pos = level * pixelsPerDegreeLevel;
        int charY = 9 - pos * 2;
        int cx = (absLevel >= 100 ? -1.5f : -1.0) * canvas->gridElementWidth;
        int px = cx + (pitchOffset + pos) * sx * 2;
        int py = -charY - (pitchOffset + pos) * (1 - sy) * 2;
        displayCanvasDrawString(canvas, px, py, buf, 0);
    }
    displayCanvasContextPop(canvas);
}

void osdDrawArtificialHorizonLine(displayCanvas_t *canvas, float pitchAngle, float rollAngle, bool erase)
{
    int barWidth = (OSD_AHI_WIDTH - 1) * canvas->gridElementWidth;
    int maxHeight = canvas->height;
    float pixelsPerDegreeLevel = 1.5f;
    int maxWidth = (OSD_AHI_WIDTH + 1) * canvas->gridElementWidth;

    int lx = (canvas->width - maxWidth) / 2;

    displayCanvasContextPush(canvas);

    displayCanvasClipToRect(canvas, lx, 0, maxWidth, maxHeight);
    osdGridBufferClearPixelRect(canvas, lx, 0, maxWidth, maxHeight);

    if (erase) {
        displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_TRANSPARENT);
        displayCanvasSetLineOutlineColor(canvas, DISPLAY_CANVAS_COLOR_TRANSPARENT);
    } else {
        displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_WHITE);
        displayCanvasSetLineOutlineColor(canvas, DISPLAY_CANVAS_COLOR_BLACK);
    }

    float pitchDegrees = RADIANS_TO_DEGREES(pitchAngle);
    float pitchOffset = -pitchDegrees * pixelsPerDegreeLevel;
    float translateX = canvas->width / 2;
    float translateY = canvas->height / 2;

    displayCanvasCtmTranslate(canvas, 0, pitchOffset);
    displayCanvasCtmRotate(canvas, rollAngle);
    displayCanvasCtmTranslate(canvas, translateX, translateY);


    displayCanvasSetStrokeWidth(canvas, 2);
    displayCanvasSetLineOutlineType(canvas, DISPLAY_CANVAS_OUTLINE_TYPE_BOTTOM);
    displayCanvasMoveToPoint(canvas, -barWidth / 2, 0);
    displayCanvasStrokeLineToPoint(canvas, -AHI_CROSSHAIR_MARGIN, 0);
    displayCanvasMoveToPoint(canvas, barWidth / 2, 0);
    displayCanvasStrokeLineToPoint(canvas, AHI_CROSSHAIR_MARGIN, 0);

    displayCanvasContextPop(canvas);
}

static bool osdCanvasDrawArtificialHorizonWidget(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float pitchAngle, float rollAngle)
{
    UNUSED(display);
    UNUSED(p);

    const int instance = 0;
    static int iterations = 0;
    static bool configured = false;

    displayWidgets_t widgets;
    if (displayCanvasGetWidgets(&widgets, canvas) &&
        displayWidgetsSupportedInstances(&widgets, DISPLAY_WIDGET_TYPE_AHI) >= instance) {

        int ahiWidth = osdConfig()->ahi_width;
        int ahiX = (canvas->width - ahiWidth) / 2;
        int ahiHeight = osdConfig()->ahi_height;
        int ahiY = ((canvas->height - ahiHeight) / 2) + osdConfig()->ahi_vertical_offset;
        if (ahiY < 0) {
            ahiY = 0;
        }
        if (ahiY + ahiHeight >= canvas->height) {
            ahiY = canvas->height - ahiHeight - 1;
        }
        if (!configured) {
            widgetAHIStyle_e ahiStyle = 0;
            switch ((osd_ahi_style_e)osdConfig()->ahi_style) {
                case OSD_AHI_STYLE_DEFAULT:
                    ahiStyle = DISPLAY_WIDGET_AHI_STYLE_STAIRCASE;
                    break;
                case OSD_AHI_STYLE_LINE:
                    ahiStyle = DISPLAY_WIDGET_AHI_STYLE_LINE;
                    break;
            }
            widgetAHIConfiguration_t config = {
                .rect.x = ahiX,
                .rect.y = ahiY,
                .rect.w = ahiWidth,
                .rect.h = ahiHeight,
                .style = ahiStyle,
                .options = osdConfig()->ahi_bordered ? DISPLAY_WIDGET_AHI_OPTION_SHOW_CORNERS : 0,
                .crosshairMargin = AHI_CROSSHAIR_MARGIN,
                .strokeWidth = 1,
            };
            if (!displayWidgetsConfigureAHI(&widgets, instance, &config)) {
                return false;
            }
            configured = true;
        }
        // The widget displays 270degs before fixing the bar at the top/bottom
        // so that's 135degs each direction. Map that to the configured limit.
        const float halfRange = 135.0f;
        const float limit = halfRange / 180.0f * M_PIf;
        float multiplier = osdConfig()->ahi_style == OSD_AHI_STYLE_DEFAULT ? 1.0f : halfRange / osdConfig()->ahi_max_pitch;
        widgetAHIData_t data = {
            .pitch = constrainf(pitchAngle * multiplier, -limit, limit),
            .roll = rollAngle,
        };
        if (displayWidgetsDrawAHI(&widgets, instance, &data)) {
            if (++iterations == 10) {
                iterations = 0;
                osdGridBufferClearPixelRect(canvas, ahiX, ahiY, ahiWidth, ahiHeight);
            }
            return true;
        }
    }
    return false;
}

void osdCanvasDrawArtificialHorizon(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float pitchAngle, float rollAngle)
{
    UNUSED(display);
    UNUSED(p);

    static float prevPitchAngle = 9999;
    static float prevRollAngle = 9999;
    static timeMs_t nextDrawMaxMs = 0;
    static timeMs_t nextDrawMinMs = 0;

    timeMs_t now = millis();

    float totalError = fabsf(prevPitchAngle - pitchAngle) + fabsf(prevRollAngle - rollAngle);
    if ((now > nextDrawMinMs && totalError > 0.05f)|| now > nextDrawMaxMs) {

        if (!osdCanvasDrawArtificialHorizonWidget(display, canvas, p, pitchAngle, rollAngle)) {
            switch ((osd_ahi_style_e)osdConfig()->ahi_style) {
                case OSD_AHI_STYLE_DEFAULT:
                {
                    int x, y, w, h;
                    osdArtificialHorizonRect(canvas, &x, &y, &w, &h);
                    displayCanvasClearRect(canvas, x, y, w, h);
                    osdDrawArtificialHorizonShapes(canvas, pitchAngle, rollAngle);
                    break;
                }
                case OSD_AHI_STYLE_LINE:
                    osdDrawArtificialHorizonLine(canvas, prevPitchAngle, prevRollAngle, true);
                    osdDrawArtificialHorizonLine(canvas, pitchAngle, rollAngle, false);
                    break;
            }
        }

        prevPitchAngle = pitchAngle;
        prevRollAngle = rollAngle;
        nextDrawMinMs = now + AHI_MIN_DRAW_INTERVAL_MS;
        nextDrawMaxMs = now + AHI_MAX_DRAW_INTERVAL_MS;
    }
}

void osdCanvasDrawHeadingGraph(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, int heading)
{
    static const uint16_t graph[] = {
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
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_N,
        SYM_HEADING_LINE,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_HEADING_E,
        SYM_HEADING_LINE,
    };

    STATIC_ASSERT(sizeof(graph) > (3599 / OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR) + OSD_HEADING_GRAPH_WIDTH + 1, graph_is_too_short);

    char buf[OSD_HEADING_GRAPH_WIDTH + 1];
    int px;
    int py;

    osdDrawPointGetPixels(&px, &py, display, canvas, p);
    int rw = OSD_HEADING_GRAPH_WIDTH * canvas->gridElementWidth;
    int rh = canvas->gridElementHeight;

    displayCanvasClipToRect(canvas, px, py, rw, rh);

    int idx = heading / OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR;
    int offset = ((heading % OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR) * canvas->gridElementWidth) / OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR;
    memcpy_fn(buf, graph + idx, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    // We need a +1 because characters are 12px wide, so
    // they can't have a 1px arrow centered. All existing fonts
    // place the arrow at 5px, hence there's a 1px offset.
    // TODO: Put this in font metadata and read it back.
    displayCanvasDrawString(canvas, px - offset + 1, py, buf, DISPLAY_CANVAS_BITMAP_OPT_ERASE_TRANSPARENT);

    displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_BLACK);
    displayCanvasSetFillColor(canvas, DISPLAY_CANVAS_COLOR_WHITE);
    int rmx = px + rw / 2;
    displayCanvasFillStrokeTriangle(canvas, rmx - 2, py - 1, rmx + 2, py - 1, rmx, py + 1);
}

static int32_t osdCanvasSidebarGetValue(osd_sidebar_scroll_e scroll)
{
    switch (scroll) {
        case OSD_SIDEBAR_SCROLL_NONE:
            break;
        case OSD_SIDEBAR_SCROLL_ALTITUDE:
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_GA:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    return CENTIMETERS_TO_CENTIFEET(osdGetAltitude());
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    return osdGetAltitude();
            }
            break;
        case OSD_SIDEBAR_SCROLL_SPEED:
            {
#if defined(USE_GPS)
                int speed = osdGetSpeedFromSelectedSource();
                switch ((osd_unit_e)osdConfig()->units) {
                    case OSD_UNIT_UK:
                        FALLTHROUGH;
                    case OSD_UNIT_METRIC_MPH:
                        FALLTHROUGH;
                    case OSD_UNIT_IMPERIAL:
                        // cms/s to (mi/h) * 100
                        return speed * 224 / 100;
                    case OSD_UNIT_GA:
                        // cm/s to Knots * 100
                        return (int)(speed * 0.019438444924406) * 100;
                    case OSD_UNIT_METRIC:
                        // cm/s to (km/h) * 100
                        return speed * 36 / 10;
                }
#endif
                break;
            }
        case OSD_SIDEBAR_SCROLL_HOME_DISTANCE:
#if defined(USE_GPS)
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_GA:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    return CENTIMETERS_TO_CENTIFEET(GPS_distanceToHome * 100);
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    return GPS_distanceToHome * 100;
#endif
            }
            break;
    }
    return 0;
}

static uint8_t osdCanvasSidebarGetOptions(int *width, osd_sidebar_scroll_e scroll)
{
    switch (scroll) {
        case OSD_SIDEBAR_SCROLL_NONE:
            break;
        case OSD_SIDEBAR_SCROLL_ALTITUDE:
            FALLTHROUGH;
        case OSD_SIDEBAR_SCROLL_SPEED:
            FALLTHROUGH;
        case OSD_SIDEBAR_SCROLL_HOME_DISTANCE:
            *width = OSD_CHAR_WIDTH * 5; // 4 numbers + unit
            return 0;
    }
    *width = OSD_CHAR_WIDTH;
    return DISPLAY_WIDGET_SIDEBAR_OPTION_STATIC | DISPLAY_WIDGET_SIDEBAR_OPTION_UNLABELED;
}

static void osdCanvasSidebarGetUnit(osdUnit_t *unit, uint16_t *countsPerStep, osd_sidebar_scroll_e scroll)
{
    // We always count in 1/100 units, converting to
    // "centifeet" when displaying imperial units
    unit->scale = 100;

    switch (scroll) {
        case OSD_SIDEBAR_SCROLL_NONE:
            unit->symbol = 0;
            unit->divisor = 0;
            unit->divided_symbol = 0;
            *countsPerStep = 1;
            break;
        case OSD_SIDEBAR_SCROLL_ALTITUDE:
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_GA:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    unit->symbol = SYM_ALT_FT;
                    unit->divisor = FEET_PER_KILOFEET;
                    unit->divided_symbol = SYM_ALT_KFT;
                    *countsPerStep = 50;
                    break;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    unit->symbol = SYM_ALT_M;
                    unit->divisor = METERS_PER_KILOMETER;
                    unit->divided_symbol = SYM_ALT_KM;
                    *countsPerStep = 20;
                    break;
            }
            break;
        case OSD_SIDEBAR_SCROLL_SPEED:
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    unit->symbol = SYM_MPH;
                    unit->divisor = 0;
                    unit->divided_symbol = 0;
                    *countsPerStep = 5;
                    break;
                case OSD_UNIT_GA:
                    unit->symbol = SYM_KT;
                    unit->divisor = 0;
                    unit->divided_symbol = 0;
                    *countsPerStep = 5;
                    break;
                case OSD_UNIT_METRIC:
                    unit->symbol = SYM_KMH;
                    unit->divisor = 0;
                    unit->divided_symbol = 0;
                    *countsPerStep = 10;
                    break;
            }
            break;
        case OSD_SIDEBAR_SCROLL_HOME_DISTANCE:
            switch ((osd_unit_e)osdConfig()->units) {
                case OSD_UNIT_UK:
                    FALLTHROUGH;
                case OSD_UNIT_IMPERIAL:
                    unit->symbol = SYM_FT;
                    unit->divisor = FEET_PER_MILE;
                    unit->divided_symbol = SYM_MI;
                    *countsPerStep = 300;
                    break;
                case OSD_UNIT_GA:
                    unit->symbol = SYM_FT;
                    unit->divisor = (int)FEET_PER_NAUTICALMILE;
                    unit->divided_symbol = SYM_NM;
                    *countsPerStep = 300;
                    break;
                case OSD_UNIT_METRIC_MPH:
                    FALLTHROUGH;
                case OSD_UNIT_METRIC:
                    unit->symbol = SYM_M;
                    unit->divisor = METERS_PER_KILOMETER;
                    unit->divided_symbol = SYM_KM;
                    *countsPerStep = 100;
                    break;
            }
            break;
    }
}

static bool osdCanvasDrawSidebar(uint32_t *configured, displayWidgets_t *widgets,
                                displayCanvas_t *canvas,
                                int instance,
                                osd_sidebar_scroll_e scroll, unsigned scrollStep)
{
    STATIC_ASSERT(OSD_SIDEBAR_SCROLL_MAX <= 3, adjust_scroll_shift);
    STATIC_ASSERT(OSD_UNIT_MAX <= 5, adjust_units_shift);
    // Configuration
    uint32_t configuration = scrollStep << 16 | (unsigned)osdConfig()->sidebar_horizontal_offset << 8 | scroll << 6 | osdConfig()->units << 4;
    if (configuration != *configured) {
        int width;
        uint8_t options = osdCanvasSidebarGetOptions(&width, scroll);
        uint8_t ex;
        uint8_t ey;
        osdCrosshairPosition(&ex, &ey);
        const int height = 2 * OSD_AH_SIDEBAR_HEIGHT_POS * OSD_CHAR_HEIGHT;
        const int y = (ey - OSD_AH_SIDEBAR_HEIGHT_POS) * OSD_CHAR_HEIGHT;

        widgetSidebarConfiguration_t config = {
            .rect.y = y,
            .rect.w = width,
            .rect.h = height,
            .options = options,
            .divisions = OSD_AH_SIDEBAR_HEIGHT_POS * 2,
        };
        uint16_t countsPerStep = 0;
        osdCanvasSidebarGetUnit(&config.unit, &countsPerStep, scroll);

        int center = ex * OSD_CHAR_WIDTH;
        int horizontalOffset = OSD_AH_SIDEBAR_WIDTH_POS * OSD_CHAR_WIDTH + osdConfig()->sidebar_horizontal_offset;
        if (instance == WIDGET_SIDEBAR_LEFT_INSTANCE) {
            config.rect.x = MAX(center - horizontalOffset - width, 0);
            config.options |= DISPLAY_WIDGET_SIDEBAR_OPTION_LEFT;
        } else {
            config.rect.x = MIN(center + horizontalOffset, canvas->width - width - 1);
        }

        if (scrollStep > 0) {
            countsPerStep = scrollStep;
        }
        config.counts_per_step = config.unit.scale * countsPerStep;

        if (!displayWidgetsConfigureSidebar(widgets, instance, &config)) {
            return false;
        }

        *configured = configuration;
    }
    // Actual drawing
    int32_t data = osdCanvasSidebarGetValue(scroll);
    return displayWidgetsDrawSidebar(widgets, instance, data);
}

bool osdCanvasDrawSidebars(displayPort_t *display, displayCanvas_t *canvas)
{
    UNUSED(display);

    static uint32_t leftConfigured = UINT32_MAX;
    static uint32_t rightConfigured = UINT32_MAX;
    static timeMs_t nextRedraw = 0;

    timeMs_t now = millis();

    if (now < nextRedraw) {
        return true;
    }

    displayWidgets_t widgets;
    if (displayCanvasGetWidgets(&widgets, canvas)) {
        if (!osdCanvasDrawSidebar(&leftConfigured, &widgets, canvas,
            WIDGET_SIDEBAR_LEFT_INSTANCE,
            osdConfig()->left_sidebar_scroll,
            osdConfig()->left_sidebar_scroll_step)) {
            return false;
        }
        if (!osdCanvasDrawSidebar(&rightConfigured, &widgets, canvas,
            WIDGET_SIDEBAR_RIGHT_INSTANCE,
            osdConfig()->right_sidebar_scroll,
            osdConfig()->right_sidebar_scroll_step)) {
            return false;
        }
        nextRedraw = now + SIDEBAR_REDRAW_INTERVAL_MS;
        return true;
    }
    return false;
}

#endif
