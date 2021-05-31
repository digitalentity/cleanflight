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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

#if defined(USE_OSD) || defined(USE_DJI_HD_OSD)

typedef enum {
    OSD_SPEED_SOURCE_GROUND    = 0,
    OSD_SPEED_SOURCE_3D        = 1,
    OSD_SPEED_SOURCE_AIR       = 2
} osdSpeedSource_e;

typedef struct {
    osdSpeedSource_e speedSource;
} osdCommonConfig_t;

PG_DECLARE(osdCommonConfig_t, osdCommonConfig);

int osdGetSpeedFromSelectedSource(void);

#endif // defined(USE_OSD) || defined(USE_DJI_HD_OSD)

#define OSD_VARIO_CM_S_PER_ARROW 50 // 1 arrow = 50cm/s
#define OSD_VARIO_HEIGHT_ROWS 5

#define OSD_AHI_HEIGHT 9
#define OSD_AHI_WIDTH 11
#define OSD_AHI_PREV_SIZE (OSD_AHI_WIDTH > OSD_AHI_HEIGHT ? OSD_AHI_WIDTH : OSD_AHI_HEIGHT)
#define OSD_AHI_H_SYM_COUNT 9
#define OSD_AHI_V_SYM_COUNT 6

#define OSD_HEADING_GRAPH_WIDTH 9
#define OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR 225

#define OSD_AH_SIDEBAR_WIDTH_POS 7
#define OSD_AH_SIDEBAR_HEIGHT_POS 3

typedef struct displayPort_s displayPort_t;
typedef struct displayCanvas_s displayCanvas_t;

typedef enum {
    OSD_DRAW_POINT_TYPE_GRID,
    OSD_DRAW_POINT_TYPE_PIXEL,
} osdDrawPointType_e;

typedef struct osdDrawPoint_s {
    osdDrawPointType_e type;
    union {
        struct {
            uint8_t gx;
            uint8_t gy;
        } grid;
        struct {
            int16_t px;
            int16_t py;
        } pixel;
    };
} osdDrawPoint_t;

#define OSD_DRAW_POINT_GRID(_x, _y) (&(osdDrawPoint_t){ .type = OSD_DRAW_POINT_TYPE_GRID, .grid = {.gx = (_x), .gy = (_y)}})
#define OSD_DRAW_POINT_PIXEL(_x, _y) (&(osdDrawPoint_t){ .type = OSD_DRAW_POINT_TYPE_PIXEL, .pixel = {.px = (_x), .py = (_y)}})

void osdDrawPointGetGrid(uint8_t *gx, uint8_t *gy, const displayPort_t *display, const displayCanvas_t *canvas, const osdDrawPoint_t *p);
void osdDrawPointGetPixels(int *px, int *py, const displayPort_t *display, const displayCanvas_t *canvas, const osdDrawPoint_t *p);

void osdDrawVario(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float zvel);
// Draws an arrow at the given point, where 0 degrees points to the top of the viewport and
// positive angles result in clockwise rotation. If eraseBefore is true, the rect surrouing
// the arrow will be erased first (need for e.g. the home arrow, since it uses multiple symbols)
void osdDrawDirArrow(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float degrees);
void osdDrawArtificialHorizon(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, float rollAngle, float pitchAngle);
// Draws a heading graph with heading given as 0.1 degree steps i.e. [0, 3600). It uses 9 horizontal
// grid slots.
void osdDrawHeadingGraph(displayPort_t *display, displayCanvas_t *canvas, const osdDrawPoint_t *p, int heading);
void osdDrawSidebars(displayPort_t *display, displayCanvas_t *canvas);

#ifdef USE_GPS
int16_t osdGet3DSpeed(void);
#endif
