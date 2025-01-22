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

#include "pg/pg.h"

typedef enum {
    CONFIGURATION_STATE_UNCONFIGURED = 0,
    CONFIGURATION_STATE_CONFIGURED,
} configurationState_e;

typedef struct systemConfig_s {
    uint8_t pidProfileIndex;
    uint8_t activeRateProfile;
    uint8_t debug_mode;
    uint8_t debug_axis;
    uint8_t task_statistics;
    uint8_t cpu_overclock;
    uint8_t powerOnArmingGraceTime; // in seconds
    char boardIdentifier[5];
    uint8_t hseMhz;                 // Only used for F4 and G4 targets
    uint8_t configurationState;     // The state of the configuration (defaults / configured)
    uint8_t enableStickArming; // boolean that determines whether stick arming can be used
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

