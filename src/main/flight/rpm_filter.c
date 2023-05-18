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

#include <math.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "scheduler/scheduler.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "drivers/dshot.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "pg/motor.h"

#include "rpm_filter.h"


typedef struct rpmFilterBank_s
{
    uint8_t  motor;

    float    ratio;
    float    minHz;
    float    maxHz;
    float    Q;

    biquadFilter_t notch[XYZ_AXIS_COUNT];

} rpmFilterBank_t;


FAST_DATA_ZERO_INIT static rpmFilterBank_t filterBank[RPM_FILTER_BANK_COUNT];

FAST_DATA_ZERO_INIT static uint8_t activeBankCount;
FAST_DATA_ZERO_INIT static uint8_t filterDebugBank;



void rpmFilterInit(const rpmFilterConfig_t *config)
{
    const int mainMotorIndex = 1;
    const int tailMotorIndex = mixerMotorizedTail() ? 2 : 1;

    float mainGearRatio = getMainGearRatio();
    float tailGearRatio = getTailGearRatio();

    const bool enable1x = (getMotorCount() >= mainMotorIndex);
    const bool enable10 = (enable1x && mainGearRatio != 1.0f);
    const bool enable2x = (getMotorCount() >= tailMotorIndex);
    const bool enable20 = (enable2x && tailGearRatio != 1.0f);

    filterDebugBank = gyroConfig()->rpmFilterDebugBank;

    activeBankCount = 0;

    for (int bank = 0; bank < RPM_FILTER_BANK_COUNT; bank++)
    {
        unsigned source = config->filter_bank_rpm_source[bank];

        rpmFilterBank_t *filt = &filterBank[activeBankCount];

        if (config->filter_bank_rpm_source[bank] == 0 ||
            config->filter_bank_rpm_ratio[bank] == 0 ||
            config->filter_bank_notch_q[bank] == 0)
            continue;

        /*
         * NOTE!  rpm_min has different meaning depending on the rpm_source.
         *
         * 1-4      Minimum RPM of the RPM source
         * 10-18    Minimum RPM of the Main ROTOR
         * 20-28    Minimum RPM of the Tail ROTOR
         *
         */

        // Manually configured filters
        if (source >= 1 && source <= getMotorCount()) {
            filt->motor  = source;
            filt->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[bank], 1, 50000) / 1000) * 60);
            filt->minHz  = constrainf(config->filter_bank_rpm_min[bank] * filt->ratio, 10, 1000);
            filt->maxHz  = 0.45f * gyro.filterRateHz;
            filt->Q      = constrainf(config->filter_bank_notch_q[bank], 5, 100) / 10;
            activeBankCount++;
        }
        // Motor#1 (main)
        else if (source == 10 && enable10) {
            filt->motor  = mainMotorIndex;
            filt->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[bank], 1, 50000) / 10000) * 60);
            filt->minHz  = constrainf((config->filter_bank_rpm_min[bank] / mainGearRatio) / 60, 10, 1000);
            filt->maxHz  = 0.45f * gyro.filterRateHz;
            filt->Q      = constrainf(config->filter_bank_notch_q[bank], 5, 100) / 10;
            activeBankCount++;
        }
        // Main Rotor harmonics
        else if (source >= 11 && source <= 18 && enable1x) {
            unsigned harmonic = source - 10;
            filt->motor  = mainMotorIndex;
            filt->ratio  = mainGearRatio * harmonic / ((constrainf(config->filter_bank_rpm_ratio[bank], 1, 50000) / 10000) * 60);
            filt->minHz  = constrainf((config->filter_bank_rpm_min[bank] * harmonic) / 60, 10, 1000);
            filt->maxHz  = 0.45f * gyro.filterRateHz;
            filt->Q      = constrainf(config->filter_bank_notch_q[bank], 5, 100) / 10;
            activeBankCount++;
        }
        // Motor#2 (tail)
        else if (source == 20 && enable20) {
            filt->motor  = tailMotorIndex;
            filt->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[bank], 1, 50000) / 10000) * 60);
            filt->minHz  = constrainf((config->filter_bank_rpm_min[bank] / tailGearRatio) / 60, 10, 1000);
            filt->maxHz  = 0.45f * gyro.filterRateHz;
            filt->Q      = constrainf(config->filter_bank_notch_q[bank], 5, 100) / 10;
            activeBankCount++;
        }
        // Tail Rotor harmonics
        else if (source >= 21 && source <= 28 && enable2x) {
            unsigned harmonic = source - 20;
            filt->motor  = tailMotorIndex;
            filt->ratio  = tailGearRatio * harmonic / ((constrainf(config->filter_bank_rpm_ratio[bank], 1, 50000) / 10000) * 60);
            filt->minHz  = constrainf((config->filter_bank_rpm_min[bank] * harmonic) / 60, 10, 1000);
            filt->maxHz  = 0.45f * gyro.filterRateHz;
            filt->Q      = constrainf(config->filter_bank_notch_q[bank], 5, 100) / 10;
            activeBankCount++;
        }
    }

    // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
    for (int bank = 0; bank < activeBankCount; bank++) {
        rpmFilterBank_t *filt = &filterBank[bank];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&filt->notch[axis], filt->minHz, gyro.filterRateHz, filt->Q, BIQUAD_NOTCH);
        }
    }
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    for (int bank=0; bank<activeBankCount; bank++) {
        value = biquadFilterApplyDF1(&filterBank[bank].notch[axis], value);
    }
    return value;
}

void rpmFilterUpdate()
{
    // Adjust the loop rate to actual gyro speed
    float loopRate = gyro.filterRateHz * schedulerGetCycleTimeMultiplier();

    for (int bank=0; bank<activeBankCount; bank++) {

        rpmFilterBank_t *filt = &filterBank[bank];

        // Calculate filter frequency
        const float rpm = getMotorRPM(filt->motor - 1);
        const float freq = rpm * filt->ratio;
        const float center = constrainf(freq, filt->minHz, filt->maxHz);

        // Notches for Roll,Pitch,Yaw
        biquadFilter_t *R = &filt->notch[0];
        biquadFilter_t *P = &filt->notch[1];
        biquadFilter_t *Y = &filt->notch[2];

        // Update the filter coefficients
        biquadFilterUpdate(R, center, loopRate, filt->Q, BIQUAD_NOTCH);

        // Transfer the filter coefficients from Roll axis filter into Pitch and Yaw
        P->b0 = Y->b0 = R->b0;
        P->b1 = Y->b1 = R->b1;
        P->b2 = Y->b2 = R->b2;
        P->a1 = Y->a1 = R->a1;
        P->a2 = Y->a2 = R->a2;

        // Set debug if debugBank matches
        if (bank == filterDebugBank) {
            DEBUG(RPM_FILTER, 0, rpm);
            DEBUG(RPM_FILTER, 1, freq * 10);
            DEBUG(RPM_FILTER, 2, center * 10);
            DEBUG(RPM_FILTER, 3, loopRate * 10);
            DEBUG(RPM_FILTER, 4, filt->motor);
            DEBUG(RPM_FILTER, 5, filt->Q * 10);
            DEBUG(RPM_FILTER, 6, filt->minHz * 10);
            DEBUG(RPM_FILTER, 7, filt->maxHz * 10);
        }
    }
}

#endif

