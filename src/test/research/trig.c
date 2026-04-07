/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Rotorflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define M_PIf     3.14159265358979323846f
#define M_PI2f    6.28318530717958647693f
#define M_2_PIf   0.63661977236758134308f

typedef struct {
    float sin;
    float cos;
} sincosf_t;



/* Degree-9 sin and degree-10 cos polynomials over r ∈ [-0.5, 0.5],
 * where the argument is r = x*(2/π) and the polynomials approximate
 * sin(r*π/2) and cos(r*π/2).  Taylor coefficients; truncation error
 * is well below a float ULP at this degree. */

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

 /* sin_precise / cos_precise use the same degree-9/10 polynomials but
  * perform argument reduction in double precision so that the reduced
  * residual r is accurate to ~1 ULP even for large inputs.  The final
  * result is still float; only the reduction step uses double. */

 static inline int32_t precise_reduce(float rad, float *r_out)
 {
     const double TWO_OVER_PI = 0.6366197723675813430755350534900566;
     double x = (double)rad * TWO_OVER_PI;
     int32_t q = (int32_t)round(x);
     *r_out = (float)(x - (double)q);
     return q;
 }

 float sin_precise(float rad)
 {
     float r;
     int32_t q = precise_reduce(rad, &r);
     float y = (q & 1) ? cos_poly10(r) : sin_poly9(r);
     float s = (q & 2) ? -y : y;
     return s;
 }

 float cos_precise(float rad)
 {
     float r;
     int32_t q = precise_reduce(rad, &r);
     float y = (q & 1) ? -sin_poly9(r) : cos_poly10(r);
     float c = (q & 2) ? -y : y;
     return c;
 }

 sincosf_t sincos_precise(float rad)
 {
     float r;
     int32_t q = precise_reduce(rad, &r);

     float sb = sin_poly9(r);
     float cb = cos_poly10(r);

     float s = (q & 1) ? cb : sb;
     float c = (q & 1) ? -sb : cb;

     s = (q & 2) ? -s : s;
     c = (q & 2) ? -c : c;

     return (sincosf_t){ s, c };
 }



 // Rotorflight math approximations

float sin_approx(float x)
{
    int32_t xint = x;

    if (xint < -32 || xint > 32)
        return 0;

    while (x >  M_PIf)
        x -= M_PI2f;
    while (x < -M_PIf)
        x += M_PI2f;

    if (x >  M_PIf / 2)
        x =  M_PIf - x;
    else if (x < -M_PIf / 2)
        x = -M_PIf - x;

    const float sinPolyCoef3 = -1.666568107e-1f;
    const float sinPolyCoef5 =  8.312366210e-3f;
    const float sinPolyCoef7 = -1.849218155e-4f;
    const float x2 = x * x;

    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * sinPolyCoef7));
}

float cos_approx(float x)
{
    return sin_approx(x + M_PIf / 2);
}


/* Fold a scaled angle s = x*(2/π) into u ∈ [0,1], then apply quadrant sign. */
float sin_approx1(float rad)
{
    float x,y,f,g;
    int32_t q;

    x = rad * M_2_PIf;
    q = (int32_t)floorf(x);
    f = x - (float)q;
    f = (q & 1u) ? (1.0f - f) : f;
    f = (q & 2u) ? -f : f;
    g = f * f;
    y = f * (1.5703200191555204f + g * (-0.6421131669862640f + g * (0.07186085423315933f)));

    return y;
}

float sin_approx2(float rad)
{
    float x,y,f,g;
    int32_t q;

    x = rad * M_2_PIf;
    q = (int32_t)floorf(x);
    f = x - (float)q;
    f = (q & 1u) ? (1.0f - f) : f;
    f = (q & 2u) ? -f : f;
    g = f * f;
    y = f * (1.5707910110756176f + g * (-0.64589284954843862f + g * (0.079434344616858263f + g * (-0.0043330952924842871f))));

    return y;
}



// New Betaflight approximations

inline float sin_poly5(float r)
{
    const float c1 =  1.570788468983057f;
    const float c3 = -0.645711990181946f;
    const float c5 =  0.077667393626301f;
    const float r2 = r * r;
    return r * (c1 + r2 * (c3 + r2 * c5));
}

inline float cos_poly6(float r)
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
    int32_t q = roundf(x);
    float r  = x - q;
    float y = (q & 1) ? cos_poly6(r) : sin_poly5(r);
    float s = (q & 2) ? -y : y;
    return s;
}

float cos_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;
    float y = (q & 1) ? -sin_poly5(r) : cos_poly6(r);
    float c = (q & 2) ? -y : y;
    return c;
}

sincosf_t sincos_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;

    float sb = sin_poly5(r);
    float cb = cos_poly6(r);

    float s = (q & 1) ? cb : sb;
    float c = (q & 1) ? -sb : cb;

    s = (q & 2) ? -s : s;
    c = (q & 2) ? -c : c;

    return (sincosf_t){ s, c };
}

float tan_approx3(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;

    float sb = sin_poly5(r);
    float cb = cos_poly6(r);

    float t = (q & 1) ? -cb / sb : sb / cb;

    return t;
}


// Like Betaflight, but with degree-7 and degree-8 polynomials.

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
    int32_t q = roundf(x);
    float r  = x - q;
    float y = (q & 1) ? cos_poly8(r) : sin_poly7(r);
    float s = (q & 2) ? -y : y;
    return s;
}

float cos_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;
    float y = (q & 1) ? -sin_poly7(r) : cos_poly8(r);
    float c = (q & 2) ? -y : y;
    return c;
}

sincosf_t sincos_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;

    float sb = sin_poly7(r);
    float cb = cos_poly8(r);

    float s = (q & 1) ? cb : sb;
    float c = (q & 1) ? -sb : cb;

    s = (q & 2) ? -s : s;
    c = (q & 2) ? -c : c;

    return (sincosf_t){ s, c };
}

float tan_approx4(float rad)
{
    float x = rad * M_2_PIf;
    int32_t q = roundf(x);
    float r  = x - q;

    float sb = sin_poly7(r);
    float cb = cos_poly8(r);

    float t = (q & 1) ? -cb / sb : sb / cb;

    return t;
}


typedef struct {
    const char *name;
    float (*fn)(float);
} sin_func_t;

const sin_func_t sin_funcs[] = {
    { "sinf",        sinf         },
    { "sin_approx",  sin_approx   },
    { "sin_approx1", sin_approx1  },
    { "sin_approx2", sin_approx2  },
    { "sin_approx3", sin_approx3  },
    { "sin_approx4", sin_approx4  },
    { "sin_precise", sin_precise  },
};


//============================================================================================================

bool runMathTests(void)
{
    bool pass = true;
    const int sample_count = 1000;
    const int func_count = sizeof(sin_funcs) / sizeof(sin_funcs[0]);

    const float min_x = -4.0f * M_PIf;
    const float max_x =  4.0f * M_PIf;
    const float step = (max_x - min_x) / (float)(sample_count - 1);
    const float max_allowed_abs_error = 1.0e-4f;

    double max_abs_error[func_count];
    float  worst_x[func_count];
    float  worst_value[func_count];
    double worst_ref[func_count];

    for (int j = 0; j < func_count; j++) {
        max_abs_error[j] = 0.0;
        worst_x[j] = 0.0f;
        worst_value[j] = 0.0f;
        worst_ref[j] = 0.0;
    }

    for (int i = 0; i < sample_count; i++) {
        const float x = min_x + (float)i * step;
        const double ref = sin((double)x);

        for (int j = 0; j < func_count; j++) {
            const float val = sin_funcs[j].fn(x);
            const double err = fabs((double)val - ref);
            if (err > max_abs_error[j]) {
                max_abs_error[j] = err;
                worst_x[j] = x;
                worst_value[j] = val;
                worst_ref[j] = ref;
            }
        }
    }

    printf("Swept %d points from %.9f to %.9f\n", sample_count, min_x, max_x);

    for (int j = 0; j < func_count; j++) {
        double bits = -log2(max_abs_error[j]);
        printf("%-14s max abs error = %.9g  (%4.1f bits)  at x = %.9f (val=%.9f, ref=%.12f)\n",
            sin_funcs[j].name, max_abs_error[j], bits,
            worst_x[j], worst_value[j], worst_ref[j]);
    }

    for (int j = 1; j < func_count; j++) {
        if (max_abs_error[j] > (double)max_allowed_abs_error) {
            printf("FAIL: %s max abs error %.9g exceeds limit %.9g\n",
                sin_funcs[j].name, max_abs_error[j], max_allowed_abs_error);
            pass = false;
        }
    }

    return pass;
}

int main(void)
{
    const bool pass = runMathTests();

    if (pass)
        printf("All math tests passed.\n");
    else
        printf("Math tests failed.\n");

    return 0;
}
