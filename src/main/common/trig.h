/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

// 2/π
#define M_2_PIf   0.63661977236758134308f

typedef struct {
    float sin;
    float cos;
} sincosf_t;

#ifndef USE_STANDARD_MATH

float sin_approx2(float x);

float sin_approx3(float x);
float cos_approx3(float x);
float tan_approx3(float x);
sincosf_t sincos_approx3(float x);

float sin_approx4(float x);
float cos_approx4(float x);
float tan_approx4(float x);
sincosf_t sincos_approx4(float x);

float sin_precise(float x);
float cos_precise(float x);
sincosf_t sincos_precise(float x);

#else /* USE_STANDARD_MATH */

#include <math.h>

#define sin_approx2(x)      sinf(x)

#define sin_approx3(x)      sinf(x)
#define cos_approx3(x)      cosf(x)
#define tan_approx3(x)      tanf(x)

#define sin_approx4(x)      sinf(x)
#define cos_approx4(x)      cosf(x)
#define tan_approx4(x)      tanf(x)

#define sin_precise(x)      sinf(x)
#define cos_precise(x)      cosf(x)

static inline sincosf_t sincos_approx3(float x)
{
    return (sincosf_t){ sinf(x), cosf(x) };
}

static inline sincosf_t sincos_approx4(float x)
{
    return (sincosf_t){ sinf(x), cosf(x) };
}

static inline sincosf_t sincos_precise(float x)
{
    return (sincosf_t){ sinf(x), cosf(x) };
}

#endif /* USE_STANDARD_MATH */
