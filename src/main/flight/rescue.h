/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "pg/pg.h"

#include "flight/pid.h"

#define RESCUE_EXIT_TIME 2000

enum {
    RESCUE_MODE_OFF = 0,
    RESCUE_MODE_NORMAL,
    RESCUE_MODE_ALT_HOLD,
    RESCUE_MODE_POS_HOLD,
};

enum {
    RESCUE_STATE_OFF = 0,
    RESCUE_PULL_UP,
    RESCUE_FLIP_OVER,
    RESCUE_CLIMB,
    RESCUE_ALT_HOLD,
    RESCUE_POS_HOLD,
    RESCUE_EXIT,
};


float rescueApply(uint8_t axis, float setpoint);

void rescueUpdate();

void rescueInit(const pidProfile_t *pidProfile);
void rescueInitProfile(const pidProfile_t *pidProfile);
