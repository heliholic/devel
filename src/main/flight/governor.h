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
    GOV_MODE_NONE = 0,
    GOV_MODE_EXTERNAL,
    GOV_MODE_ELECTRIC,
    GOV_MODE_NITRO,
} govMode_e;

typedef enum {
    GOV_STATE_THROTTLE_OFF,
    GOV_STATE_THROTTLE_IDLE,
    GOV_STATE_SPOOLUP,
    GOV_STATE_RECOVERY,
    GOV_STATE_ACTIVE,
    GOV_STATE_THROTTLE_CUT,
    GOV_STATE_FALLBACK,
    GOV_STATE_AUTOROTATION,
    GOV_STATE_BAILOUT,
    GOV_STATE_DIRECT,
} govState_e;

enum {
    GOV_FLAG_DIRECT_THROTTLE,
    GOV_FLAG_3POS_THROTTLE,
    GOV_FLAG_PRECOMP,
    GOV_FLAG_FALLBACK_PRECOMP,
    GOV_FLAG_DIRECT_PRECOMP,
    GOV_FLAG_VOLTAGE_COMP,
    GOV_FLAG_PID_SPOOLUP,
    GOV_FLAG_HS_ON_THROTTLE,
    GOV_FLAG_AUTOROTATION,
};

typedef enum {
    GOV_CURVE_LINEAR = 0,
    GOV_CURVE_SEMI_QUADRATIC,
    GOV_CURVE_QUADRATIC,
    GOV_CURVE_SEMI_CUBIC,
    GOV_CURVE_CUBIC,
} govCurve_e;

void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate(void);

bool getGovernerEnabled(void);
void setGovernorEnabled(bool enabled);

int getGovernorState(void);

float getGovernorOutput(void);

float getFullHeadSpeedRatio(void);
float getSpoolUpRatio(void);

float getTTAIncrease(void);

bool isSpooledUp(void);

