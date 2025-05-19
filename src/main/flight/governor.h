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
    GOV_THROTTLE_PASSTHRU = 0,
    GOV_THROTTLE_3POS,
    GOV_THROTTLE_DIRECT,
    GOV_THROTTLE_PIDCTRL,
} govThrMode_e;

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
} govState_e;

enum {
    GOV_FLAG_PRECOMP      = BIT(0),
    GOV_FLAG_VOLT_CORR    = BIT(1),
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

