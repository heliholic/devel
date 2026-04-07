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

// 1/π
#define M_1_PIf   0.31830988618379067154f

// 2/π
#define M_2_PIf   0.63661977236758134308f

// 4/π
#define M_4_PIf   1.27323954473516268616f

// π/2
#define M_PI_2f   1.57079632679489661923f

float sin_fast(float x);
float cos_fast(float x);
float sin_taylor(float x);
float cos_taylor(float x);

float sin_approx2(float x);
float cos_approx2(float x);

float sin_approx3(float x);
float cos_approx3(float x);

float sin_approx4(float x);
float cos_approx4(float x);

float sin_betaflight(float x);
float cos_betaflight(float x);

