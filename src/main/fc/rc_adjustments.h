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

#include <stdbool.h>

#include "fc/rc_modes.h"
#include "flight/pid.h"
#include "pg/pg.h"

typedef enum {
    ADJUSTMENT_NONE = 0,

    // Profile change
    ADJUSTMENT_RATE_PROFILE,                // 1
    ADJUSTMENT_PID_PROFILE,
    ADJUSTMENT_LED_PROFILE,
    ADJUSTMENT_OSD_PROFILE,

    // Rates
    ADJUSTMENT_PITCH_RATE,                  // 5
    ADJUSTMENT_ROLL_RATE,
    ADJUSTMENT_YAW_RATE,
    ADJUSTMENT_PITCH_RC_RATE,
    ADJUSTMENT_ROLL_RC_RATE,
    ADJUSTMENT_YAW_RC_RATE,
    ADJUSTMENT_PITCH_RC_EXPO,
    ADJUSTMENT_ROLL_RC_EXPO,
    ADJUSTMENT_YAW_RC_EXPO,

    // PID
    ADJUSTMENT_PITCH_P_GAIN,                // 14
    ADJUSTMENT_PITCH_I_GAIN,
    ADJUSTMENT_PITCH_D_GAIN,
    ADJUSTMENT_PITCH_F_GAIN,
    ADJUSTMENT_ROLL_P_GAIN,                 // 18
    ADJUSTMENT_ROLL_I_GAIN,
    ADJUSTMENT_ROLL_D_GAIN,
    ADJUSTMENT_ROLL_F_GAIN,
    ADJUSTMENT_YAW_P_GAIN,                  // 22
    ADJUSTMENT_YAW_I_GAIN,
    ADJUSTMENT_YAW_D_GAIN,
    ADJUSTMENT_YAW_F_GAIN,

    ADJUSTMENT_YAW_CW_GAIN,                 // 26
    ADJUSTMENT_YAW_CCW_GAIN,
    ADJUSTMENT_YAW_CYCLIC_FF,
    ADJUSTMENT_YAW_COLLECTIVE_FF,
    ADJUSTMENT_YAW_COLLECTIVE_DF,
    ADJUSTMENT_PITCH_COLLECTIVE_FF,

    // PID cutoffs
    ADJUSTMENT_PITCH_GYRO_CUTOFF,           // 32
    ADJUSTMENT_PITCH_DTERM_CUTOFF,
    ADJUSTMENT_ROLL_GYRO_CUTOFF,
    ADJUSTMENT_ROLL_DTERM_CUTOFF,
    ADJUSTMENT_YAW_GYRO_CUTOFF,
    ADJUSTMENT_YAW_DTERM_CUTOFF,

    // Rescue
    ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE,     // 38
    ADJUSTMENT_RESCUE_HOVER_COLLECTIVE,
    ADJUSTMENT_RESCUE_ALT_P_GAIN,
    ADJUSTMENT_RESCUE_ALT_I_GAIN,
    ADJUSTMENT_RESCUE_ALT_D_GAIN,

    // Leveling
    ADJUSTMENT_ANGLE_LEVEL_GAIN,            // 43
    ADJUSTMENT_HORIZON_LEVEL_GAIN,
    ADJUSTMENT_ACRO_TRAINER_GAIN,

    // Governor
    ADJUSTMENT_GOV_GAIN,                    // 46
    ADJUSTMENT_GOV_P_GAIN,
    ADJUSTMENT_GOV_I_GAIN,
    ADJUSTMENT_GOV_D_GAIN,
    ADJUSTMENT_GOV_F_GAIN,
    ADJUSTMENT_GOV_TTA_GAIN,
    ADJUSTMENT_GOV_CYCLIC_FF,
    ADJUSTMENT_GOV_COLLECTIVE_FF,

    // Mixer
    ADJUSTMENT_TAIL_MOTOR_IDLE,             // 55
    ADJUSTMENT_SWASH_PHASE,

    // Temporary for evaluation
    ADJUSTMENT_WAY_P_GAIN,                  // 57
    ADJUSTMENT_WAY_I_GAIN,
    ADJUSTMENT_WAY_D_GAIN,
    ADJUSTMENT_WAY_F_GAIN,

    ADJUSTMENT_FUNCTION_COUNT               // 61
} adjustmentFunction_e;

typedef enum {
    ADJUSTMENT_TYPE_NONE  = 0,
    ADJUSTMENT_TYPE_RATE  = BIT(0),
    ADJUSTMENT_TYPE_PROF  = BIT(1),
    ADJUSTMENT_TYPE_GOV   = BIT(2),
    ADJUSTMENT_TYPE_MIX   = BIT(3),
} adjustmentType_e;

typedef struct {
    timeMs_t deadTime;
    timeMs_t trigTime;
    int32_t  adjValue;
    int32_t  chValue;
} adjustmentState_t;

typedef struct {
    char *  cfgName;
    uint8_t cfgType;
    int16_t cfgMin;
    int16_t cfgMax;
} adjustmentConfig_t;

typedef struct {
    uint8_t function;
    uint8_t enaChannel;
    channelRange_t enaRange;
    uint8_t adjChannel;
    channelRange_t adjRange1;
    channelRange_t adjRange2;
    int16_t adjMin;
    int16_t adjMax;
    uint8_t adjStep;
} adjustmentRange_t;


#define MAX_ADJUSTMENT_RANGE_COUNT 32

PG_DECLARE_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges);


void adjustmentRangeInit(void);
void adjustmentRangeReset(int index);

void processRcAdjustments(void);

const char *getAdjustmentsRangeName(void);
int getAdjustmentsRangeFunc(void);
int getAdjustmentsRangeValue(void);

