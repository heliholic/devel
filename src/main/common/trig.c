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


// Fast sin approximation for rad ∈ [-π/4, π/4].  Minimax-optimised degree-7 odd polynomial.
float sin_fast(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * (-0.16666650669294222f + x2 * (0.00833197866315977f + x2 * (-0.00019495636237996f))));
}

// Fast cos approximation for rad ∈ [-π/4, π/4].  Minimax-optimised degree-6 even polynomial.
float cos_fast(float x)
{
    float x2 = x * x;
    return (1.0f + x2 * (-0.49999894781370191f + x2 * (0.04165629457842692f + x2 * (-0.00135978231111122f))));
}


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




#define INV_PIO2    M_2_PIf

static inline float sin_poly5_qf(float r)
{
    // Pre-scaled for u = r*(π/2)
    const float c0 =  0x1.921f1cp0f; // 1.5707871913909912109375
    const float c1 = -0x1.4a974p-1f; // -0.6456851959228515625
    const float c2 =  0x1.3db294p-4f; // 7.756288349628448486328125e-2
    float s = r * r;
    return r * ((c2 * s + c1) * s + c0);
}

static inline float cos_poly6_qf(float r)
{
    const float d1 = -0x1.3bd39cp0f; // -1.2336976528167724609375
    const float d2 =  0x1.03bp-2f; // 0.25360107421875
    const float d3 = -0x1.4e5eecp-6f; // -2.04083733260631561279296875e-2
    float s = r * r;
    return ((d3 * s + d2) * s + d1) * s + 1.0f;
}

// ---- Quadrant mapping helpers ----
// r ∈ [-0.5, 0.5], q is quadrant index (…,-1,0,1,2,3,4,…).
static inline float sinf_quadrant_qf(float r, int q)
{
    q &= 3;
    if (q & 1) { // odd: use cos, sign handled below
        float v = cos_poly6_qf(r);
        return (q & 2) ? -v : v;
    } else {     // even: use sin
        float v = sin_poly5_qf(r);
        return (q & 2) ? -v : v;
    }
}

static inline float cosf_quadrant_qf(float r, int q)
{
    q &= 3;
    if (q & 1) { // odd: -sin, sign handled below
        float v = -sin_poly5_qf(r);
        return (q & 2) ? -v : v;   // q=1 -> -sin, q=3 -> +sin
    } else {     // even: cos
        float v = cos_poly6_qf(r);
        return (q & 2) ? -v : v;   // q=2 -> -cos
    }
}


float sin_quickflash(float x)
{
    float t = x * INV_PIO2;     // in quadrant units
    float qf = roundf(t);       // nearest quadrant as float
    int   q  = (int)qf;
    float r  = t - qf;          // remainder in [-0.5, 0.5]
    return sinf_quadrant_qf(r, q);
}

float cos_quickflash(float x)
{
    float t = x * INV_PIO2;
    float qf = roundf(t);
    int   q  = (int)qf;
    float r  = t - qf;          // [-0.5, 0.5]
    return cosf_quadrant_qf(r, q);
}


