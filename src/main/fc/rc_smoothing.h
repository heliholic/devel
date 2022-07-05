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

#include "drivers/time.h"


#define RC_SMOOTHING_AUTO_FACTOR_MIN 0
#define RC_SMOOTHING_AUTO_FACTOR_MAX 250

typedef struct rcSmoothingFilterTraining_s {
    float sum;
    int count;
    uint16_t min;
    uint16_t max;
} rcSmoothingFilterTraining_t;

typedef struct rcSmoothingFilter_s {
    bool filterInitialized;
    pt3Filter_t filter[4];
    pt3Filter_t filterDeflection[2];
    pt3Filter_t setpointDeltaFilter[3];
    uint8_t setpointCutoffSetting;
    uint8_t throttleCutoffSetting;
    uint16_t setpointCutoffFrequency;
    uint16_t throttleCutoffFrequency;
    uint8_t setpointDeltaCutoffSetting;
    uint16_t setpointDeltaCutoffFrequency;
    int averageFrameTimeUs;
    rcSmoothingFilterTraining_t training;
    uint8_t debugAxis;
    uint8_t autoSmoothnessFactorSetpoint;
    uint8_t autoSmoothnessFactorThrottle;
} rcSmoothingFilter_t;


rcSmoothingFilter_t *getRcSmoothingData(void);

float getSetpointRate(int axis);

bool rcSmoothingAutoCalculate(void);
bool rcSmoothingInitializationComplete(void);

void processRcSmoothingFilter(bool isRxDataNew, bool isRxRateValid, uint16_t currentRxRefreshRate);

float rcSmoothingApplySetpointDeltaFilter(int axis, float pidSetpointDelta);
