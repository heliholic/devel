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

#include <math.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "trig.h"

#ifndef USE_STANDARD_MATH


// Quadrant-folding single-polynomial approximation for sin.
// Reduces rad to [0,1) per quadrant, folds, then evaluates a degree-7 minimax polynomial.

float sin_approx2(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = (int32_t)floorf(x);
    float f = x - (float)q;
    f = (q & 1) ? (1.0f - f) : f;
    f = (q & 2) ? -f : f;
    const float g = f * f;
    return f * (1.5707910110756176f + g * (-0.64589284954843862f + g * (0.079434344616858263f + g * (-0.0043330952924842871f))));
}

float cos_approx2(float rad)
{
    return sin_approx2(rad + M_PI_2f);
}

sincosf_t sincos_approx2(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = (int32_t)floorf(x);
    float f = x - (float)q;

    // unsigned sin/cos magnitudes; cos complement of sin in [0, 0.5]
    float fs = (q & 1) ? (1.0f - f) : f;
    float fc = 1.0f - fs;

    float gs = fs * fs;
    float gc = fc * fc;

    float s = fs * (1.5707910110756176f + gs * (-0.64589284954843862f + gs * (0.079434344616858263f + gs * (-0.0043330952924842871f))));
    float c = fc * (1.5707910110756176f + gc * (-0.64589284954843862f + gc * (0.079434344616858263f + gc * (-0.0043330952924842871f))));

    s = (q & 2) ? -s : s;
    c = ((q + 1) & 2) ? -c : c;

    return (sincosf_t){ s, c };
}

float tan_approx2(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = (int32_t)floorf(x);
    float f = x - (float)q;

    float fs = (q & 1) ? (1.0f - f) : f;
    float fc = 1.0f - fs;

    float gs = fs * fs;
    float gc = fc * fc;

    float s = fs * (1.5707910110756176f + gs * (-0.64589284954843862f + gs * (0.079434344616858263f + gs * (-0.0043330952924842871f))));
    float c = fc * (1.5707910110756176f + gc * (-0.64589284954843862f + gc * (0.079434344616858263f + gc * (-0.0043330952924842871f))));

    // (q & 2) sign cancels in the ratio; only quadrant parity matters
    return (q & 1) ? -s / c : s / c;
}


// Degree-5 sin / degree-6 cos paired polynomials over r ∈ [-0.5, 0.5],
// approximating sin(r·π/2) and cos(r·π/2).

static inline float sin_poly5(float r)
{
    const float c1 =  1.570788468983057f;
    const float c3 = -0.645711990181946f;
    const float c5 =  0.077667393626301f;
    const float r2 = r * r;
    return r * (c1 + r2 * (c3 + r2 * c5));
}

static inline float cos_poly6(float r)
{
    const float c2 = -1.233697953970536f;
    const float c4 =  0.253606361920527f;
    const float c6 = -0.020426250304794f;
    const float r2 = r * r;
    return 1.0f + r2 * (c2 + r2 * (c4 + r2 * c6));
}

float sin_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? cos_poly6(r) : sin_poly5(r);
    return (q & 2) ? -y : y;
}

float cos_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? -sin_poly5(r) : cos_poly6(r);
    return (q & 2) ? -y : y;
}

float tan_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly5(r);
    float cb = cos_poly6(r);

    // (q & 2) sign flip cancels in the ratio; only quadrant parity matters
    return (q & 1) ? -cb / sb : sb / cb;
}

sincosf_t sincos_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly5(r);
    float cb = cos_poly6(r);

    float s = (q & 1) ? cb : sb;
    float c = (q & 1) ? -sb : cb;

    s = (q & 2) ? -s : s;
    c = (q & 2) ? -c : c;

    return (sincosf_t){ s, c };
}


// Degree-7 sin / degree-8 cos paired polynomials over r ∈ [-0.5, 0.5],
// approximating sin(r·π/2) and cos(r·π/2).  Minimax-optimised coefficients.

static inline float sin_poly7(float r)
{
    const float c1 =  1.5707963050854579f;
    const float c3 = -0.64596293816733275f;
    const float c5 =  0.079675902970013021f;
    const float c7 = -0.0045922890757518504f;
    const float r2 = r * r;
    return r * (c1 + r2 * (c3 + r2 * (c5 + r2 * c7)));
}

static inline float cos_poly8(float r)
{
    const float c2 = -1.2337005406473684f;
    const float c4 =  0.25366920393921583f;
    const float c6 = -0.020860071318401938f;
    const float c8 =  0.00090363170248550532f;
    const float r2 = r * r;
    return 1.0f + r2 * (c2 + r2 * (c4 + r2 * (c6 + r2 * c8)));
}

float sin_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? cos_poly8(r) : sin_poly7(r);
    return (q & 2) ? -y : y;
}

float cos_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? -sin_poly7(r) : cos_poly8(r);
    return (q & 2) ? -y : y;
}

float tan_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly7(r);
    float cb = cos_poly8(r);

    return (q & 1) ? -cb / sb : sb / cb;
}

sincosf_t sincos_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly7(r);
    float cb = cos_poly8(r);

    float s = (q & 1) ? cb : sb;
    float c = (q & 1) ? -sb : cb;

    s = (q & 2) ? -s : s;
    c = (q & 2) ? -c : c;

    return (sincosf_t){ s, c };
}


// Degree-9 sin / degree-10 cos paired polynomials — Taylor coefficients.
// Truncation error is well below a float ULP; actual error is float-arithmetic limited.

static inline float sin_poly9(float r)
{
    const float c1 =  1.5707963267948966f;
    const float c3 = -0.6459640975062462f;
    const float c5 =  0.07969262624616703f;
    const float c7 = -0.004681754135318685f;
    const float c9 =  1.6044118478735963e-4f;
    const float r2 = r * r;
    return r * (c1 + r2 * (c3 + r2 * (c5 + r2 * (c7 + r2 * c9))));
}

static inline float cos_poly10(float r)
{
    const float c2 = -1.2337005501361698f;
    const float c4 =  0.25366920193218095f;
    const float c6 = -0.020862209263265985f;
    const float c8 =  9.192661394714042e-4f;
    const float c10 = -2.2969903187012496e-5f;
    const float r2 = r * r;
    return 1.0f + r2 * (c2 + r2 * (c4 + r2 * (c6 + r2 * (c8 + r2 * c10))));
}

float sin_approx5(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? cos_poly10(r) : sin_poly9(r);
    return (q & 2) ? -y : y;
}

float cos_approx5(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;
    float y = (q & 1) ? -sin_poly9(r) : cos_poly10(r);
    return (q & 2) ? -y : y;
}

float tan_approx5(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly9(r);
    float cb = cos_poly10(r);

    return (q & 1) ? -cb / sb : sb / cb;
}

sincosf_t sincos_approx5(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = lrintf(x);
    float r = x - (float)q;

    float sb = sin_poly9(r);
    float cb = cos_poly10(r);

    float s = (q & 1) ? cb : sb;
    float c = (q & 1) ? -sb : cb;

    s = (q & 2) ? -s : s;
    c = (q & 2) ? -c : c;

    return (sincosf_t){ s, c };
}


#endif /* USE_STANDARD_MATH */
