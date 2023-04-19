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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"


// NULL filter

FAST_CODE float nullFilterApply(nullFilter_t *filter, float input)
{
    return filter->y1 = input;
}

void nullFilterUpdate(nullFilter_t *filter, float cutoff, float sampleRate)
{
    UNUSED(filter);
    UNUSED(cutoff);
    UNUSED(sampleRate);
}

void nullFilterInit(nullFilter_t *filter, float cutoff, float sampleRate)
{
    UNUSED(filter);
    UNUSED(cutoff);
    UNUSED(sampleRate);
}


// PT1 Low Pass filter

float pt1FilterGain(float cutoff, float sampleRate)
{
    float omega = tan_approx(M_PIf * cutoff / sampleRate);
    float alpha = 2.0f / (1.0f / omega + 1.0f);

    return alpha;
}

void pt1FilterInit(pt1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->gain = pt1FilterGain(cutoff, sampleRate);
}

void pt1FilterInitGain(pt1Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->gain = gain;
}

void pt1FilterUpdate(pt1Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt1FilterGain(cutoff, sampleRate);
}

void pt1FilterUpdateGain(pt1Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->y1 += (input - filter->y1) * filter->gain;
    return filter->y1;
}


// PT2 Low Pass filter

float pt2FilterGain(float cutoff, float sampleRate)
{
    // order=2: 1 / sqrt( (2^(1 / order) - 1)) = 1.553773974
    return pt1FilterGain(cutoff * 1.553773974f, sampleRate);
}

void pt2FilterInit(pt2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->gain = pt2FilterGain(cutoff, sampleRate);
}

void pt2FilterInitGain(pt2Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->gain = gain;
}

void pt2FilterUpdate(pt2Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt2FilterGain(cutoff, sampleRate);
}

void pt2FilterUpdateGain(pt2Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->y2 += (input      - filter->y2) * filter->gain;
    filter->y1 += (filter->y2 - filter->y1) * filter->gain;
    return filter->y1;
}


// PT3 Low Pass filter

float pt3FilterGain(float cutoff, float sampleRate)
{
    // order=3: 1 / sqrt( (2^(1 / order) - 1)) = 1.961459177
    return pt1FilterGain(cutoff * 1.961459177f, sampleRate);
}

void pt3FilterInit(pt3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->gain = pt3FilterGain(cutoff, sampleRate);
}

void pt3FilterInitGain(pt3Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->gain = gain;
}

void pt3FilterUpdate(pt3Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt3FilterGain(cutoff, sampleRate);
}

void pt3FilterUpdateGain(pt3Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt3FilterApply(pt3Filter_t *filter, float input)
{
    filter->y3 += (input      - filter->y3) * filter->gain;
    filter->y2 += (filter->y3 - filter->y2) * filter->gain;
    filter->y1 += (filter->y2 - filter->y1) * filter->gain;
    return filter->y1;
}


// PT4 Low Pass filter

float pt4FilterGain(float cutoff, float sampleRate)
{
    // order=4: 1 / sqrt( (2^(1 / order) - 1)) = 2.298959223
    return pt1FilterGain(cutoff * 2.298959223f, sampleRate);
}

void pt4FilterInit(pt4Filter_t *filter, float cutoff, float sampleRate)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->y4 = 0;
    filter->gain = pt4FilterGain(cutoff, sampleRate);
}

void pt4FilterInitGain(pt4Filter_t *filter, float gain)
{
    filter->y1 = 0;
    filter->y2 = 0;
    filter->y3 = 0;
    filter->y4 = 0;
    filter->gain = gain;
}

void pt4FilterUpdate(pt4Filter_t *filter, float cutoff, float sampleRate)
{
    filter->gain = pt4FilterGain(cutoff, sampleRate);
}

void pt4FilterUpdateGain(pt4Filter_t *filter, float gain)
{
    filter->gain = gain;
}

FAST_CODE float pt4FilterApply(pt4Filter_t *filter, float input)
{
    filter->y4 += (input      - filter->y4) * filter->gain;
    filter->y3 += (filter->y4 - filter->y3) * filter->gain;
    filter->y2 += (filter->y3 - filter->y2) * filter->gain;
    filter->y1 += (filter->y2 - filter->y1) * filter->gain;
    return filter->y1;
}


// BiQuad filter a.k.a. Second-Order-Section

void biquadFilterInit(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType)
{
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;

    biquadFilterUpdate(filter, cutoff, sampleRate, Q, filterType);
}

FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float cutoff, float sampleRate, float Q, uint8_t filterType)
{
    const float omega = M_2PIf * cutoff / sampleRate;
    const float sinom = sin_approx(omega);
    const float cosom = cos_approx(omega);
    const float alpha = sinom / (2 * Q);

    switch (filterType) {
        case BIQUAD_LPF:
            filter->b1 = 1 - cosom;
            filter->b0 = filter->b1 * 0.5f;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_HPF:
            filter->b1 = 1 + cosom;
            filter->b0 = filter->b1 * 0.5f;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_BPF:
            filter->b0 = alpha;
            filter->b1 = 0;
            filter->b2 = -alpha;
            filter->a1 = -2 * cosom;
            filter->a2 = 1 - alpha;
            break;

        case BIQUAD_NOTCH:
            filter->b0 = 1;
            filter->b1 = -2 * cosom;
            filter->b2 = 1;
            filter->a1 = filter->b1;
            filter->a2 = 1 - alpha;
            break;
    }

    const float a0 = 1 + alpha;

    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;
}


FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    const float output =
        filter->b0 * input +
        filter->b1 * filter->x1 +
        filter->b2 * filter->x2 -
        filter->a1 * filter->y1 -
        filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

FAST_CODE float biquadFilterApplyDF2(biquadFilter_t *filter, float input)
{
    const float output = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * output + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * output;

    filter->y1 = output;

    return output;
}


// Generic Low-Pass Filter (LPF)

static void biquadButterLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, BUTTER_C * cutoff, sampleRate, BUTTER_Q, BIQUAD_LPF);
}

static void biquadBesselLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, BESSEL_C * cutoff, sampleRate, BESSEL_Q, BIQUAD_LPF);
}

static void biquadDampedLPFInit(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterInit(filter, DAMPED_C * cutoff, sampleRate, DAMPED_Q, BIQUAD_LPF);
}

static void biquadButterLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, BUTTER_C * cutoff, sampleRate, BUTTER_Q, BIQUAD_LPF);
}

static void biquadBesselLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, BESSEL_C * cutoff, sampleRate, BESSEL_Q, BIQUAD_LPF);
}

static void biquadDampedLPFUpdate(biquadFilter_t *filter, float cutoff, float sampleRate)
{
    biquadFilterUpdate(filter, DAMPED_C * cutoff, sampleRate, DAMPED_Q, BIQUAD_LPF);
}

void lowpassFilterInit(filter_t *filter, uint8_t type, float cutoff, float sampleRate, uint32_t flags)
{
    if (cutoff == 0 || sampleRate == 0)
        type = LPF_NULL;

    switch (type) {
        case LPF_ORDER_1:
        case LPF_PT1:
            filter->init   = (filterInitFn)pt1FilterInit;
            filter->apply  = (filterApplyFn)pt1FilterApply;
            filter->update = (filterUpdateFn)pt1FilterUpdate;
            break;

        case LPF_PT2:
            filter->init   = (filterInitFn)pt2FilterInit;
            filter->apply  = (filterApplyFn)pt2FilterApply;
            filter->update = (filterUpdateFn)pt2FilterUpdate;
            break;

        case LPF_PT3:
            filter->init   = (filterInitFn)pt3FilterInit;
            filter->apply  = (filterApplyFn)pt3FilterApply;
            filter->update = (filterUpdateFn)pt3FilterUpdate;
            break;

        case LPF_PT4:
            filter->init   = (filterInitFn)pt4FilterInit;
            filter->apply  = (filterApplyFn)pt4FilterApply;
            filter->update = (filterUpdateFn)pt4FilterUpdate;
            break;

        case LPF_BUTTER:
            filter->init   = (filterInitFn)biquadButterLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF2;
                filter->update = (filterUpdateFn)biquadButterLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)nullFilterUpdate;
            }
            break;

        case LPF_ORDER_2:
        case LPF_BESSEL:
            filter->init   = (filterInitFn)biquadBesselLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF2;
                filter->update = (filterUpdateFn)biquadBesselLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)nullFilterUpdate;
            }
            break;

        case LPF_DAMPED:
            filter->init   = (filterInitFn)biquadDampedLPFInit;
            if (flags & LPF_UPDATE) {
                filter->apply = (filterApplyFn)biquadFilterApplyDF2;
                filter->update = (filterUpdateFn)biquadDampedLPFUpdate;
            }
            else {
                filter->apply = (filterApplyFn)biquadFilterApplyDF1;
                filter->update = (filterUpdateFn)nullFilterUpdate;
            }
            break;

        default:
            filter->init   = (filterInitFn)nullFilterInit;
            filter->apply  = (filterApplyFn)nullFilterApply;
            filter->update = (filterUpdateFn)nullFilterUpdate;
            break;
    }

    filter->init(&filter->data, cutoff, sampleRate);
}

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float notchFilterGetQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

void notchFilterInit(filter_t *filter, float cutoff, float Q, float sampleRate, uint32_t flags)
{
    filter->init   = (filterInitFn)nullFilterInit;
    filter->update = (filterUpdateFn)nullFilterUpdate;
    filter->apply  = (filterApplyFn)nullFilterApply;

    if (cutoff > 0 && Q > 0) {
        if (flags & LPF_UPDATE)
            filter->apply = (filterApplyFn)biquadFilterApplyDF2;
        else
            filter->apply = (filterApplyFn)biquadFilterApplyDF1;

        biquadFilterInit(&filter->data.sos, cutoff, sampleRate, Q, BIQUAD_NOTCH);
    }
}

void notchFilterUpdate(filter_t *filter, float cutoff, float Q, float sampleRate)
{
    if (cutoff > 0 && Q > 0)
        biquadFilterUpdate(&filter->data.sos, cutoff, sampleRate, Q, BIQUAD_NOTCH);
}


// Simple fixed-point lowpass filter based on integer math

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal)
{
    filter->fp = (filter->fp << filter->beta) - filter->fp;
    filter->fp += newVal << filter->fpShift;
    filter->fp >>= filter->beta;
    return filter->fp >> filter->fpShift;
}

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift)
{
    filter->fp = 0;
    filter->beta = beta;
    filter->fpShift = fpShift;
}
