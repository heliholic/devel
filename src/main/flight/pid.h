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
#include "common/pwl.h"
#include "common/rtc.h"

#include "pg/pid.h"

#define MAX_PID_PROCESS_DENOM       16

#define DTERM_LPF1_HZ_DEFAULT 75
#define DTERM_LPF2_HZ_DEFAULT 150

#define G_ACCELERATION 9.80665f // gravitational acceleration in m/s^2

typedef enum {
    TERM_P,
    TERM_I,
    TERM_D,
    TERM_F,
    TERM_S,
} term_e;

typedef enum {
    YAW_TYPE_RUDDER,
    YAW_TYPE_DIFF_THRUST,
} yawType_e;

void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

void resetPidProfile(pidProfile_t *profile);

float pidGetPreviousSetpoint(int axis);
float pidGetPidFrequency();

void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitConfig(const pidProfile_t *pidProfile);
void pidInit(const pidProfile_t *pidProfile);

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);
