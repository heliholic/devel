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


void rpmFilterInit(void)
{
    const rpmFilterConfig_t *config = rpmFilterConfig();

    const int mainMotorIndex = 0;
    const int tailMotorIndex = mixerMotorizedTail() ? 1 : 0;

    const float mainGearRatio = getMainGearRatio();
    const float tailGearRatio = getTailGearRatio();

    const bool enable1x = (getMotorCount() > mainMotorIndex);
    const bool enable10 = (enable1x && mainGearRatio != 1.0f);
    const bool enable2x = (getMotorCount() > tailMotorIndex);
    const bool enable20 = (enable2x && tailGearRatio != 1.0f);

    activeBankCount = 0;

    for (int index = 0; index < RPM_FILTER_BANK_COUNT; index++)
    {
        const unsigned source = config->filter_bank_rpm_source[index];

        rpmFilterBank_t *bank = &filterBank[activeBankCount];

        if (config->filter_bank_rpm_source[index] == 0 ||
            config->filter_bank_rpm_ratio[index] == 0 ||
            config->filter_bank_notch_q[index] == 0)
            continue;

        /*
         * NOTE!  rpm_min has different meaning depending on rpm_source value.
         *
         *    1-4     Minimum RPM of the motor
         *   10-18    Minimum RPM of the Main ROTOR
         *   20-28    Minimum RPM of the Tail ROTOR
         */

        // Manually configured filters
        if (source >= 1 && source <= getMotorCount()) {
            bank->motor  = source - 1;
            bank->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[index], 1, 50000) / 1000) * 60);
            bank->minHz  = constrainf(config->filter_bank_rpm_min[index] * bank->ratio, 10, 1000);
            bank->maxHz  = 0.45f * gyro.filterRateHz;
            bank->Q      = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;
            activeBankCount++;
        }
        // Motor#1 (main)
        else if (source == 10 && enable10) {
            bank->motor  = mainMotorIndex;
            bank->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[index], 1, 50000) / 10000) * 60);
            bank->minHz  = constrainf((config->filter_bank_rpm_min[index] / mainGearRatio) / 60, 10, 1000);
            bank->maxHz  = 0.45f * gyro.filterRateHz;
            bank->Q      = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;
            activeBankCount++;
        }
        // Main Rotor harmonics
        else if (source >= 11 && source <= 18 && enable1x) {
            unsigned harmonic = source - 10;
            bank->motor  = mainMotorIndex;
            bank->ratio  = mainGearRatio * harmonic / ((constrainf(config->filter_bank_rpm_ratio[index], 1, 50000) / 10000) * 60);
            bank->minHz  = constrainf((config->filter_bank_rpm_min[index] * harmonic) / 60, 10, 1000);
            bank->maxHz  = 0.45f * gyro.filterRateHz;
            bank->Q      = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;
            activeBankCount++;
        }
        // Motor#2 (tail)
        else if (source == 20 && enable20) {
            bank->motor  = tailMotorIndex;
            bank->ratio  = 1.0f / ((constrainf(config->filter_bank_rpm_ratio[index], 1, 50000) / 10000) * 60);
            bank->minHz  = constrainf((config->filter_bank_rpm_min[index] / tailGearRatio) / 60, 10, 1000);
            bank->maxHz  = 0.45f * gyro.filterRateHz;
            bank->Q      = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;
            activeBankCount++;
        }
        // Tail Rotor harmonics
        else if (source >= 21 && source <= 28 && enable2x) {
            unsigned harmonic = source - 20;
            bank->motor  = tailMotorIndex;
            bank->ratio  = tailGearRatio * harmonic / ((constrainf(config->filter_bank_rpm_ratio[index], 1, 50000) / 10000) * 60);
            bank->minHz  = constrainf((config->filter_bank_rpm_min[index] * harmonic) / 60, 10, 1000);
            bank->maxHz  = 0.45f * gyro.filterRateHz;
            bank->Q      = constrainf(config->filter_bank_notch_q[index], 5, 100) / 10;
            activeBankCount++;
        }
    }

    // Init all filters @minHz. As soon as the motor is running, the filters are updated to the real RPM.
    for (int index = 0; index < activeBankCount; index++) {
        rpmFilterBank_t *bank = &filterBank[index];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&bank->notch[axis], bank->minHz, gyro.filterRateHz, bank->Q, BIQUAD_NOTCH);
        }
    }
}

FAST_CODE float rpmFilterGyro(int axis, float value)
{
    for (int index = 0; index < activeBankCount; index++) {
        value = biquadFilterApplyDF1(&filterBank[index].notch[axis], value);
    }
    return value;
}

void rpmFilterUpdate()
{
    // Actual gyro loop rate
    float loopRate = gyro.filterRateHz * schedulerGetCycleTimeMultiplier();

    // Update all banks
    for (int index = 0; index < activeBankCount; index++) {

        rpmFilterBank_t *bank = &filterBank[index];

        // Calculate filter frequency
        const float rpm = getMotorRPM(bank->motor);
        const float freq = rpm * bank->ratio;
        const float center = constrainf(freq, bank->minHz, bank->maxHz);

        // Notches for Roll,Pitch,Yaw
        biquadFilter_t *R = &bank->notch[0];
        biquadFilter_t *P = &bank->notch[1];
        biquadFilter_t *Y = &bank->notch[2];

        // Update the filter coefficients
        biquadFilterUpdate(R, center, loopRate, bank->Q, BIQUAD_NOTCH);

        // Transfer the filter coefficients from Roll axis filter into Pitch and Yaw
        P->b0 = Y->b0 = R->b0;
        P->b1 = Y->b1 = R->b1;
        P->b2 = Y->b2 = R->b2;
        P->a1 = Y->a1 = R->a1;
        P->a2 = Y->a2 = R->a2;

        // Set debug if bank number matches
        if (index == debugAxis) {
            DEBUG(RPM_FILTER, 0, rpm);
            DEBUG(RPM_FILTER, 1, freq * 10);
            DEBUG(RPM_FILTER, 2, center * 10);
            DEBUG(RPM_FILTER, 3, loopRate * 10);
            DEBUG(RPM_FILTER, 4, bank->motor);
            DEBUG(RPM_FILTER, 5, bank->Q * 10);
            DEBUG(RPM_FILTER, 6, bank->minHz * 10);
            DEBUG(RPM_FILTER, 7, bank->maxHz * 10);
        }
    }
}

#endif

