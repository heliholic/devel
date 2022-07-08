/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define BUTTERWORTH_Q       0.70710678118654746171f         // 1 / sqrt(2)
#define BESSEL_Q            0.57735026918962584208f         // 1 / sqrt(3)
#define CRITICAL_Q          0.5f

typedef struct pt1Filter_s {
    float y1;
    float k;
} pt1Filter_t;

typedef struct pt2Filter_s {
    float y1, y2;
    float k;
} pt2Filter_t;

typedef struct pt3Filter_s {
    float y1, y2, y3;
    float k;
} pt3Filter_t;

typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float wf;
} biquadFilter_t;

typedef enum {
    FILTER_LPF,
    FILTER_HPF,
    FILTER_NOTCH,
} filterMode_e;

typedef enum {
    FILTER_NONE = 0,
    FILTER_PT1,
    FILTER_PT2,
    FILTER_PT3,
    FILTER_SOS,
} filterType_e;

typedef struct filter_s filter_t;

typedef float (*filterFunc_f)(filter_t *filter, float arg);

struct filter_s {

    union {
        pt1Filter_t pt1;
        pt2Filter_t pt2;
        pt3Filter_t pt3;
        biquadFilter_t sos;
    };

    uint8_t type;
    uint8_t mode;

    float Fs;
    float Q;

    filterFunc_f apply;
};


float nullFilterApply(filter_t *filter, float input);

float ptFilterGain(float Fo, float Fs);

void pt1FilterInit(pt1Filter_t *filter);
void pt1FilterCutoff(pt1Filter_t *filter, float Fo, float Fs);
float pt1FilterApply(pt1Filter_t *filter, float input);

void pt2FilterInit(pt2Filter_t *filter);
void pt2FilterCutoff(pt2Filter_t *filter, float Fo, float Fs);
float pt2FilterApply(pt2Filter_t *filter, float input);

void pt3FilterInit(pt3Filter_t *filter);
void pt3FilterCutoff(pt3Filter_t *filter, float Fo, float Fs);
float pt3FilterApply(pt3Filter_t *filter, float input);

float notchFilterComputeQ(float Fc, float Fo);

void biquadFilterInit(biquadFilter_t *filter);
void biquadFilterCutoff(biquadFilter_t *filter, float Fo, float Fs, float Q, biquadFilterType_e filterType);
void biquadFilterWeight(biquadFilter_t *filter, float Wc);

float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApplyDF2(biquadFilter_t *filter, float input);
float biquadFilterApplyWeighted(biquadFilter_t *filter, float input);


void filterInit(filter_t *filter, filterType_e type, filterMode_e mode, float Fs);


