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

#pragma once

#include <stdint.h>
#include <math.h>

#include "common/maths.h"

typedef union {
    float v[3];
    struct {
       float x,y,z;
    };
} fpVector3_t;

typedef struct {
    float m[3][3];
} fpMat3_t;

typedef struct {
    fpVector3_t axis;
    float angle;
} fpAxisAngle_t;

void rotationMatrixFromAngles(fpMat3_t * rmat, const fp_angles_t * angles);
void rotationMatrixFromAxisAngle(fpMat3_t * rmat, const fpAxisAngle_t * a);

static inline void vectorZero(fpVector3_t * v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

static inline fpVector3_t * rotationMatrixRotateVector(fpVector3_t * result, const fpVector3_t * a, const fpMat3_t * rmat)
{
    fpVector3_t r;

    r.x = rmat->m[0][0] * a->x + rmat->m[1][0] * a->y + rmat->m[2][0] * a->z;
    r.y = rmat->m[0][1] * a->x + rmat->m[1][1] * a->y + rmat->m[2][1] * a->z;
    r.z = rmat->m[0][2] * a->x + rmat->m[1][2] * a->y + rmat->m[2][2] * a->z;

    *result = r;
    return result;
}

static inline float vectorNormSquared(const fpVector3_t * v)
{
    return sq(v->x) + sq(v->y) + sq(v->z);
}

static inline fpVector3_t * vectorNormalize(fpVector3_t * result, const fpVector3_t * v)
{
    float length = sqrtf(vectorNormSquared(v));
    if (length != 0) {
        result->x = v->x / length;
        result->y = v->y / length;
        result->z = v->z / length;
    }
    else {
        result->x = 0;
        result->y = 0;
        result->z = 0;
    }
    return result;
}

static inline fpVector3_t * vectorCrossProduct(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->y * b->z - a->z * b->y;
    ab.y = a->z * b->x - a->x * b->z;
    ab.z = a->x * b->y - a->y * b->x;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorAdd(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->x + b->x;
    ab.y = a->y + b->y;
    ab.z = a->z + b->z;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorSub(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->x - b->x;
    ab.y = a->y - b->y;
    ab.z = a->z - b->z;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorScale(fpVector3_t * result, const fpVector3_t * a, const float b)
{
    fpVector3_t ab;

    ab.x = a->x * b;
    ab.y = a->y * b;
    ab.z = a->z * b;

    *result = ab;
    return result;
}
