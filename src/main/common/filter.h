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

typedef struct rateLimitFilter_s {
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
    float alpha;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

typedef union { 
    biquadFilter_t biquad; 
    pt1Filter_t pt1; 
} filter_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;

typedef struct alphaBetaGammaFilter_s {
    float a, b, g, e;
    float ak; // derivative of system velociy (ie: acceleration)
    float vk; // derivative of system state (ie: velocity)
    float xk; // current system state (ie: position)
    float jk; // derivative of system acceleration (ie: jerk)
    float rk; // residual error
    float dT, dT2, dT3;
    float halfLife, boost;
    pt1Filter_t boostFilter;
} alphaBetaGammaFilter_t;

typedef float (*filterApplyFnPtr)(void *filter, float input);
typedef float (*filterApply4FnPtr)(void *filter, float input, float f_cut, float dt);

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

float nullFilterApply(void *filter, float input);
float nullFilterApply4(void *filter, float input, float f_cut, float dt);

void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT);
void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT);
void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut);
float pt1FilterGetLastOutput(pt1Filter_t *filter);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply3(pt1Filter_t *filter, float input, float dT);
float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);

void rateLimitFilterInit(rateLimitFilter_t *filter);
float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT);

void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs);
void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float biquadFilterReset(biquadFilter_t *filter, float value);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float filterGetNotchQ(float centerFrequencyHz, float cutoffFrequencyHz);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);

void alphaBetaGammaFilterInit(alphaBetaGammaFilter_t *filter, float alpha, float boostGain, float halfLife, float dT);
float alphaBetaGammaFilterApply(alphaBetaGammaFilter_t *filter, float input);