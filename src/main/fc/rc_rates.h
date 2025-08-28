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

#include <stdint.h>

#include "pg/rates.h"

#define CONTROL_RATE_CONFIG_RC_EXPO_MAX         100
#define CONTROL_RATE_CONFIG_RC_RATES_MAX        255
#define CONTROL_RATE_CONFIG_SUPER_RATE_MAX      255

#define SETPOINT_RATE_LIMIT                     2000

typedef enum {
    RATES_TYPE_NONE = 0,
    RATES_TYPE_BETAFLIGHT,
    RATES_TYPE_RACEFLIGHT,
    RATES_TYPE_KISS,
    RATES_TYPE_ACTUAL,
    RATES_TYPE_QUICK,
    RATES_TYPE_COUNT
} ratesType_e;

typedef struct ratesSettingsLimits_s {
    uint8_t rc_rate_limit;
    uint8_t srate_limit;
    uint8_t expo_limit;
} ratesSettingsLimits_t;

extern controlRateConfig_t * currentControlRateProfile;
extern const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT];

 int adjustmentGet_PITCH_RATE(int adjFunc);
void adjustmentSet_PITCH_RATE(int adjFunc, int value);
 int adjustmentGet_ROLL_RATE(int adjFunc);
void adjustmentSet_ROLL_RATE(int adjFunc, int value);
 int adjustmentGet_YAW_RATE(int adjFunc);
void adjustmentSet_YAW_RATE(int adjFunc, int value);
 int adjustmentGet_PITCH_RC_RATE(int adjFunc);
void adjustmentSet_PITCH_RC_RATE(int adjFunc, int value);
 int adjustmentGet_ROLL_RC_RATE(int adjFunc);
void adjustmentSet_ROLL_RC_RATE(int adjFunc, int value);
 int adjustmentGet_YAW_RC_RATE(int adjFunc);
void adjustmentSet_YAW_RC_RATE(int adjFunc, int value);
 int adjustmentGet_PITCH_RC_EXPO(int adjFunc);
void adjustmentSet_PITCH_RC_EXPO(int adjFunc, int value);
 int adjustmentGet_ROLL_RC_EXPO(int adjFunc);
void adjustmentSet_ROLL_RC_EXPO(int adjFunc, int value);
 int adjustmentGet_YAW_RC_EXPO(int adjFunc);
void adjustmentSet_YAW_RC_EXPO(int adjFunc, int value);
 int adjustmentGet_PITCH_SP_BOOST_GAIN(int adjFunc);
void adjustmentSet_PITCH_SP_BOOST_GAIN(int adjFunc, int value);
 int adjustmentGet_ROLL_SP_BOOST_GAIN(int adjFunc);
void adjustmentSet_ROLL_SP_BOOST_GAIN(int adjFunc, int value);
 int adjustmentGet_YAW_SP_BOOST_GAIN(int adjFunc);
void adjustmentSet_YAW_SP_BOOST_GAIN(int adjFunc, int value);
 int adjustmentGet_COLL_SP_BOOST_GAIN(int adjFunc);
void adjustmentSet_COLL_SP_BOOST_GAIN(int adjFunc, int value);
 int adjustmentGet_YAW_DYN_CEILING_GAIN(int adjFunc);
void adjustmentSet_YAW_DYN_CEILING_GAIN(int adjFunc, int value);
 int adjustmentGet_YAW_DYN_DEADBAND_GAIN(int adjFunc);
void adjustmentSet_YAW_DYN_DEADBAND_GAIN(int adjFunc, int value);
 int adjustmentGet_YAW_DYN_DEADBAND_FILTER(int adjFunc);
void adjustmentSet_YAW_DYN_DEADBAND_FILTER(int adjFunc, int value);

float applyRatesCurve(const int axis, float rcCommandf);

void loadControlRateProfile(void);
void changeControlRateProfile(uint8_t controlRateProfileIndex);
void copyControlRateProfile(uint8_t dstControlRateProfileIndex, uint8_t srcControlRateProfileIndex);

