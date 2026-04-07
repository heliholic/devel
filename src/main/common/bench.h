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

#ifdef USE_MATH_BENCH

#include "common/trig.h"

typedef struct {
    const char    *name;        // NULL = blank-line separator in output
    float        (*fn)(float);  // function under test; NULL for separator entries
    const double  *ref;         // precision reference array, or NULL for no precision check
    // filled by mathBenchRun():
    uint32_t      cycles;
    uint8_t       bits;
} mathBenchEntry_t;

extern mathBenchEntry_t mathBenchEntries[];
extern const int        mathBenchEntryCount;

void mathBenchRun(void);

#endif
