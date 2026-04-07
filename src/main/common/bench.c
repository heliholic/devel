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
#include "common/trig.h"
#include "drivers/system.h"

#include "bench.h"

#ifdef USE_MATH_BENCH

static double sin_ref[256];
static double cos_ref[256];

mathBenchEntry_t mathBenchEntries[] = {
    { .name = "sinf       ", .fn = sinf,        .ref = sin_ref },
    { .name = "cosf       ", .fn = cosf,        .ref = cos_ref },
    { .name = NULL },
    { .name = "sin_approx ", .fn = sin_approx,  .ref = sin_ref },
    { .name = "cos_approx ", .fn = cos_approx,  .ref = cos_ref },
    { .name = NULL },
    { .name = "sin_approx2", .fn = sin_approx2, .ref = sin_ref },
    { .name = "cos_approx2", .fn = cos_approx2, .ref = cos_ref },
    { .name = NULL },
    { .name = "sin_approx3", .fn = sin_approx3, .ref = sin_ref },
    { .name = "cos_approx3", .fn = cos_approx3, .ref = cos_ref },
    { .name = NULL },
    { .name = "sin_approx4", .fn = sin_approx4, .ref = sin_ref },
    { .name = "cos_approx4", .fn = cos_approx4, .ref = cos_ref },
    { .name = NULL },
    { .name = "sin_fast", .fn = sin_fast, .ref = NULL },
    { .name = "cos_fast", .fn = cos_fast, .ref = NULL },
    { .name = NULL },
    { .name = "sin_quickflash", .fn = sin_quickflash, .ref = sin_ref },
    { .name = "cos_quickflash", .fn = cos_quickflash, .ref = cos_ref },
};

const int mathBenchEntryCount = sizeof(mathBenchEntries) / sizeof(mathBenchEntries[0]);


void mathBenchRun(void)
{
    static float inputs[256];

    for (int i = 0; i < 256; i++)
        inputs[i] = (i / 256.0f) * M_2PIf;

    for (int i = 0; i < 256; i++) {
        sin_ref[i] = sin(inputs[i]);
        cos_ref[i] = cos(inputs[i]);
    }

    const int N = 10000;
    volatile float sink = 0;

    for (int j = 0; j < mathBenchEntryCount; j++) {
        mathBenchEntry_t *e = &mathBenchEntries[j];

        if (!e->name)
            continue;

        uint32_t cycles = 0;

        for (int i = 0; i < N; i++) {
            float (*func)(float) = e->fn;
            float input = inputs[i & 255];
            uint32_t t0 = getCycleCounter();
            sink = func(input);
            uint32_t t1 = getCycleCounter();
            cycles += t1 - t0;
        }
        e->cycles = cycles / N;

        if (e->ref) {
            double maxErr = 0;
            for (int i = 0; i < 256; i++) {
                if (!isfinite(e->ref[i]))
                    continue;
                double err = fabs((double)e->fn(inputs[i]) - e->ref[i]);
                if (err > maxErr)
                    maxErr = err;
            }
            e->bits = (maxErr > 0 && maxErr < 1) ? (uint8_t)(-log2(maxErr)) : 0;
        }
    }

    (void)sink;
}

#endif
