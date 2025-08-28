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
    GM_OFF = 0,
    GM_PASSTHROUGH,
    GM_STANDARD,
    GM_MODE1,
    GM_MODE2,
} govMode_e;

typedef enum {
    GS_THROTTLE_OFF,
    GS_THROTTLE_IDLE,
    GS_SPOOLING_UP,
    GS_RECOVERY,
    GS_ACTIVE,
    GS_ZERO_THROTTLE,
    GS_LOST_HEADSPEED,
    GS_AUTOROTATION,
    GS_AUTOROTATION_BAILOUT,
} govState_e;


 int adjustmentGet_GOV_GAIN(int adjFunc);
void adjustmentSet_GOV_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_P_GAIN(int adjFunc);
void adjustmentSet_GOV_P_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_I_GAIN(int adjFunc);
void adjustmentSet_GOV_I_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_D_GAIN(int adjFunc);
void adjustmentSet_GOV_D_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_F_GAIN(int adjFunc);
void adjustmentSet_GOV_F_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_TTA_GAIN(int adjFunc);
void adjustmentSet_GOV_TTA_GAIN(int adjFunc, int value);
 int adjustmentGet_GOV_CYCLIC_FF(int adjFunc);
void adjustmentSet_GOV_CYCLIC_FF(int adjFunc, int value);
 int adjustmentGet_GOV_COLLECTIVE_FF(int adjFunc);
void adjustmentSet_GOV_COLLECTIVE_FF(int adjFunc, int value);


void governorInit(const pidProfile_t *pidProfile);
void governorInitProfile(const pidProfile_t *pidProfile);

void governorUpdate(void);

int getGovernorState(void);

float getGovernorOutput(void);

float getFullHeadSpeedRatio(void);
float getSpoolUpRatio(void);

float getTTAIncrease(void);

bool isSpooledUp(void);

