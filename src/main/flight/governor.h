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

#include "pg/governor.h"

#include "flight/pid.h"

typedef enum {
    GOV_TYPE_NONE = 0,
    GOV_TYPE_EXTERNAL,
    GOV_TYPE_ELECTRIC,
    GOV_TYPE_NITRO,
} govType_e;

typedef enum {
    GOV_STATE_THROTTLE_OFF,
    GOV_STATE_THROTTLE_LOW,
    GOV_STATE_THROTTLE_IDLE,
    GOV_STATE_SPOOLUP,
    GOV_STATE_RECOVERY,
    GOV_STATE_ACTIVE,
    GOV_STATE_FALLBACK,
    GOV_STATE_AUTOROTATION,
    GOV_STATE_BAILOUT,
    GOV_STATE_DIRECT,
} govState_e;

enum {
    GOV_FLAG_PRECOMP            = BIT(0),
    GOV_FLAG_FALLBACK_PRECOMP   = BIT(1),
    GOV_FLAG_VOLTAGE_COMP       = BIT(2),
    GOV_FLAG_PID_SPOOLUP        = BIT(3),
    GOV_FLAG_HS_ON_THROTTLE     = BIT(4),
    GOV_FLAG_3POS_THROTTLE      = BIT(5),
    GOV_FLAG_PID_WITH_THROTTLE  = BIT(6),
};


void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate(void);

int getGovernorState(void);

float getGovernorOutput(void);

float getFullHeadSpeedRatio(void);
float getSpoolUpRatio(void);

float getTTAIncrease(void);

bool isSpooledUp(void);

