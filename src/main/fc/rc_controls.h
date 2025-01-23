/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/axis.h"
#include "common/filter.h"

#include "pg/pg.h"
#include "pg/arming.h"
#include "pg/rc_controls.h"

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    COLLECTIVE,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

#define PRIMARY_CHANNEL_COUNT 5

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define ROL_LO    (1 << 0)
#define ROL_HI    (2 << 0)
#define ROL_CE    (3 << 0)
#define PIT_LO    (1 << 2)
#define PIT_HI    (2 << 2)
#define PIT_CE    (3 << 2)
#define YAW_LO    (1 << 4)
#define YAW_HI    (2 << 4)
#define YAW_CE    (3 << 4)
#define THR_LO    (1 << 6)
#define THR_HI    (2 << 6)
#define THR_CE    (3 << 6)
#define THR_MASK  (3 << 6)

#define CONTROL_RATE_CONFIG_RC_EXPO_MAX  100

#define CONTROL_RATE_CONFIG_RC_RATES_MAX  255

#define CONTROL_RATE_CONFIG_RATE_LIMIT_MIN  200
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX  1998

// (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
#define CONTROL_RATE_CONFIG_RATE_MAX  255

extern float rcCommand[5];

bool areUsingSticksToArm(void);

throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions();

bool isUsingSticksForArming(void);

void rcControlsInit(void);
