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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"



// NULL filter

FAST_CODE float nullFilterApply(filter_t *filter, float x)
{
    UNUSED(filter);
    return x;
}


// First order all-pole Low Pass Filter (PT1)

float ptFilterGain(float Fo, float Fs)
{
    float Wc = tan_approx(M_PIf * Fo / Fs);
    return 2 * Wc / (Wc + 1);
}

void pt1FilterInit(pt1Filter_t *filter)
{
    filter->y1 = 0;
}

void pt1FilterCutoff(pt1Filter_t *filter, float Fo, Float Fs)
{
    filter->k = ptFilterGain(Fo,Fs);
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float x)
{
    filter->y1 += filter->k * (x - filter->y1);
    return filter->x0;
}


// Second order all-pole Low Pass Filter (PT2)

void pt2FilterInit(pt2Filter_t *filter)
{
    filter->y1 = 0;
    filter->y2 = 0;
}

void pt2FilterCutoff(pt2Filter_t *filter, float Fo, float Fs)
{
    filter->k = ptFilterGain(Fo,Fs);
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float x)
{
    filter->y1 += filter->k * (x          - filter->y1);
    filter->y2 += filter->k * (filter->y1 - filter->y2);
    return filter->y2;
}


// Third order all-pole Low Pass Filter (PT3)

void pt3FilterInit(pt3Filter_t *filter)
{
    filter->y1 = 0;
    filter->y2 = 0;
}

void pt3FilterCutoff(pt3Filter_t *filter, float Fo, float Fs)
{
    filter->k = ptFilterGain(Fo,Fs);
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float x)
{
    filter->y1 += filter->k * (x          - filter->y1);
    filter->y2 += filter->k * (filter->y1 - filter->y2);
    filter->y3 += filter->k * (filter->y2 - filter->y3);
    return filter->y3;
}


// Second order generic filter (Biquad)

float notchFilterComputeQ(float Fc, float Fo)
{
    return fabsf(Fc * Fo / (Fc * Fc - Fo * Fo));
}

void biquadFilterInit(biquadFilter_t *filter)
{
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
    filter->weight = 0;
}

FAST_CODE void biquadFilterCutoff(biquadFilter_t *filter, float Fo, float Fs, float Q, biquadType_e filterType)
{
    const float omega = 2 * M_PIf * Fo / Fs;
    const float sin = sin_approx(omega);
    const float cos = cos_approx(omega);
    const float alpha = sin / (2 * Q);

    float a0, a1, a2, b0, b1, b2;
 
    switch (filterType) {
        case FILTER_LPF:
            b1 = 1 - cos;
            b0 = b1 / 2;
            b2 = b0;
            a0 = 1 + alpha;
            a1 = -2 * cos;
            a2 = 1 - alpha;
            break;
        case FILTER_HPF:
            b1 = 1 + cos;
            b0 = b1 / 2;
            b2 = b0;
            a0 = 1 + alpha;
            a1 = -2 * cos;
            a2 = 1 - alpha;
            break;
        case FILTER_NOTCH:
            b0 = 1;
            b1 = -2 * cos;
            b2 = 1;
            a0 = 1 + alpha;
            a1 = b1;
            a2 = 1 - alpha;
            break;
    }

    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
}

void biquadFilterWeight(biquadFilter_t *filter, float Wf)
{
    filter->wf = Wf;
}

FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    /* compute result */
    const float y0 = 
        filter->b0 * input + 
        filter->b1 * filter->x1 + 
        filter->b2 * filter->x2 - 
        filter->a1 * filter->y1 - 
        filter->a2 * filter->y2;

    /* shift numerator */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift denominator */
    filter->y2 = filter->y1;
    filter->y1 = y0;

    return y0;
}

FAST_CODE float biquadFilterApplyDF2(biquadFilter_t *filter, float input)
{
    const float y0 = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * y0 + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * y0;

    return y0;
}

FAST_CODE float biquadFilterApplyWeighted(biquadFilter_t* filter, float input)
{
    const float output = biquadFilterApplyDF1(filter, input);

    return filter->wf * output + (1 - filter->wf) * input;
}
