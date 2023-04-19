/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#define BUTTER_Q    0.707106781f     /* 2nd order Butterworth: 1/sqrt(2) */
#define BESSEL_Q    0.577350269f     /* 2nd order Bessel: 1/sqrt(3) */
#define DAMPED_Q    0.5f             /* 2nd order Critically damped: 1/sqrt(4) */

#define BUTTER_C    1.0f
#define BESSEL_C    1.272019649f
#define DAMPED_C    1.553773974f

enum {
    LPF_NULL = 0,
    LPF_ORDER_1,
    LPF_ORDER_2,
    LPF_PT1,
    LPF_PT2,
    LPF_PT3,
    LPF_PT4,
    LPF_BUTTER,
    LPF_BESSEL,
    LPF_DAMPED,
};

enum {
    BIQUAD_NULL = 0,
    BIQUAD_LPF,
    BIQUAD_HPF,
    BIQUAD_BPF,
    BIQUAD_NOTCH,
};

typedef struct {
    float y1;
} nullFilter_t;

typedef struct {
    float y1;
    float gain;
} pt1Filter_t;

typedef struct {
    float y1;
    float y2;
    float gain;
} pt2Filter_t;

typedef struct {
    float y1;
    float y2;
    float y3;
    float gain;
} pt3Filter_t;

typedef struct {
    float y1;
    float y2;
    float y3;
    float y4;
    float gain;
} pt4Filter_t;

typedef struct {
    float y1;
    float x1;
    float a;
    float b;
} difFilter_t;

typedef struct {
    float y1;
    float x1;
    float min;
    float max;
    float gain;
} intFilter_t;

typedef struct {
    float y1;
    float y2;
    float x1;
    float x2;
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} biquadFilter_t;

typedef union {
    nullFilter_t    nul;
    pt1Filter_t     pt1;
    pt2Filter_t     pt2;
    pt3Filter_t     pt3;
    pt4Filter_t     pt4;
    biquadFilter_t  sos;
} filterData_t;

typedef struct filter_s filter_t;

typedef void  (*filterInitFn)(filterData_t *filter, float cutoff, float sampleRate);
typedef void  (*filterUpdateFn)(filterData_t *filter, float cutoff, float sampleRate);
typedef float (*filterApplyFn)(filterData_t *filter, float input);

typedef struct filter_s {
    filterInitFn    init;
    filterApplyFn   apply;
    filterUpdateFn  update;
    filterData_t    data;
} filter_t;

enum {
    LPF_UPDATE  = BIT(0),
};


static inline void filterInit(filter_t *filter, float cutoff, float sampleRate)
{
    if (filter->init)
        filter->init(&filter->data, cutoff, sampleRate);
}

static inline void filterUpdate(filter_t *filter, float cutoff, float sampleRate)
{
    if (filter->update)
        filter->update(&filter->data, cutoff, sampleRate);
}

static inline float filterApply(filter_t *filter, float input)
{
    if (filter->apply)
        return filter->apply(&filter->data, input);
    else
        return input;
}

static inline float filterOutput(filter_t *filter)
{
    return filter->data.nul.y1;
}

void nullFilterInit(nullFilter_t *filter, float cutoff, float sampleRate);
void nullFilterUpdate(nullFilter_t *filter, float cutoff, float sampleRate);
float nullFilterApply(nullFilter_t *filter, float input);

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float sampleRate);
void pt1FilterUpdate(pt1Filter_t *filter, float cutoff, float sampleRate);
void pt1FilterInitGain(pt1Filter_t *filter, float gain);
void pt1FilterUpdateGain(pt1Filter_t *filter, float gain);
float pt1FilterGain(float cutoff, float sampleRate);
float pt1FilterApply(pt1Filter_t *filter, float input);

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float sampleRate);
void pt2FilterUpdate(pt2Filter_t *filter, float cutoff, float sampleRate);
void pt2FilterInitGain(pt2Filter_t *filter, float gain);
void pt2FilterUpdateGain(pt2Filter_t *filter, float gain);
float pt2FilterGain(float cutoff, float sampleRate);
float pt2FilterApply(pt2Filter_t *filter, float input);

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float sampleRate);
void pt3FilterUpdate(pt3Filter_t *filter, float cutoff, float sampleRate);
void pt3FilterInitGain(pt3Filter_t *filter, float gain);
void pt3FilterUpdateGain(pt3Filter_t *filter, float gain);
float pt3FilterGain(float cutoff, float sampleRate);
float pt3FilterApply(pt3Filter_t *filter, float input);

void pt4FilterInit(pt4Filter_t *filter, float cutoff, float sampleRate);
void pt4FilterUpdate(pt4Filter_t *filter, float cutoff, float sampleRate);
void pt4FilterInitGain(pt4Filter_t *filter, float gain);
void pt4FilterUpdateGain(pt4Filter_t *filter, float gain);
float pt4FilterGain(float cutoff, float sampleRate);
float pt4FilterApply(pt4Filter_t *filter, float input);

void difFilterInit(difFilter_t *filter, float cutoff, float sampleRate);
void difFilterUpdate(difFilter_t *filter, float cutoff, float sampleRate);
float difFilterApply(difFilter_t *filter, float input);

void intFilterInit(intFilter_t *filter, float sampleRate, float min, float max);
void intFilterReset(intFilter_t *filter);
void intFilterUpdate(intFilter_t *filter, float sampleRate, float min, float max);
float intFilterApply(intFilter_t *filter, float input);

void biquadBesselInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadBesselUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadButterInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadButterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadDampedInit(biquadFilter_t *filter, float cutoff, float sampleRate);
void biquadDampedUpdate(biquadFilter_t *filter, float cutoff, float sampleRate);

void biquadFilterInit(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType);
void biquadFilterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType);

float biquadFilterApply(biquadFilter_t *filter, float input);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApplyDF2(biquadFilter_t *filter, float input);

void lowpassFilterInit(filter_t *filter, uint8_t type, float cutoff, float sampleRate, uint32_t flags);

void notchFilterInit(filter_t *filter, float cutoff, float Q, float sampleRate, uint32_t flags);
void notchFilterUpdate(filter_t *filter, float cutoff, float Q, float sampleRate);
float notchFilterGetQ(float centerFreq, float cutoffFreq);


typedef struct simpleLowpassFilter_s {
    int32_t fp;
    int32_t beta;
    int32_t fpShift;
} simpleLowpassFilter_t;

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal);
void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift);
