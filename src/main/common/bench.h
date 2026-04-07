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

#ifndef USE_STANDARD_MATH

typedef struct {
    // Timing — cycles per call
    uint32_t sinf_lib;
    uint32_t cosf_lib;
    uint32_t tanf_lib;
    uint32_t sin_approx;
    uint32_t cos_approx;
    uint32_t tan_approx;
    uint32_t sin_approx2;
    uint32_t sin_approx3;
    uint32_t cos_approx3;
    uint32_t tan_approx3;
    uint32_t sincos_approx3;
    uint32_t sin_approx4;
    uint32_t cos_approx4;
    uint32_t tan_approx4;
    uint32_t sincos_approx4;
    uint32_t sin_precise;
    uint32_t cos_precise;
    uint32_t sincos_precise;
    // Precision — max absolute error vs sin_precise / cos_precise reference
    float sin_approx_err;
    float cos_approx_err;
    float sin_approx2_err;
    float sin_approx3_err;
    float cos_approx3_err;
    float sin_approx4_err;
    float cos_approx4_err;
} mathBenchResults_t;

extern mathBenchResults_t mathBenchResults;

void mathBenchRun(void);

#endif
