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

#include "common/maths.h"
#include "drivers/system.h"

#include "bench.h"

#ifndef USE_STANDARD_MATH

mathBenchResults_t mathBenchResults;

void mathBenchRun(void)
{
    static float inputs[256];

    for (int i = 0; i < 256; i++)
        inputs[i] = (i / 256.0f) * M_2PIf;

    const int N = 10000;
    volatile float sink = 0;
    volatile sincosf_t sc = { 0, 0 };
    uint32_t t0, t1;

#define BENCH(field, expr) \
    t0 = getCycleCounter(); \
    for (int i = 0; i < N; i++) { expr; } \
    t1 = getCycleCounter(); \
    mathBenchResults.field = (t1 - t0) / N;

    BENCH(sinf_lib,       sink = sinf(inputs[i & 255]))
    BENCH(sin_approx,     sink = sin_approx(inputs[i & 255]))
    BENCH(sin_approx2,    sink = sin_approx2(inputs[i & 255]))
    BENCH(sin_approx3,    sink = sin_approx3(inputs[i & 255]))
    BENCH(cosf_lib,       sink = cosf(inputs[i & 255]))
    BENCH(cos_approx,     sink = cos_approx(inputs[i & 255]))
    BENCH(cos_approx3,    sink = cos_approx3(inputs[i & 255]))
    BENCH(sincos_approx3, sc   = sincos_approx3(inputs[i & 255]))
    BENCH(tan_approx,     sink = tan_approx(inputs[i & 255]))
    BENCH(tan_approx3,    sink = tan_approx3(inputs[i & 255]))

#undef BENCH

    (void)sink;
    (void)sc;
}

#endif
