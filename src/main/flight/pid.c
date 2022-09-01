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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc_rates.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/rpm_filter.h"

#include "pid.h"


static FAST_DATA_ZERO_INIT pid_t pid;


float pidGetDT()
{
    return pid.dT;
}

float pidGetPidFrequency()
{
    return pid.freq;
}

float pidGetSetpoint(int axis)
{
    return pid.axis[axis].setPoint;
}

float pidGetOutput(int axis)
{
    return pid.axis[axis].pidSum;
}

float pidGetCollective(void)
{
    return pid.collective;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.axis;
}

void INIT_CODE pidReset(void)
{
    memset(pid.axis, 0, sizeof(pid.axis));
}

void INIT_CODE pidResetIterm(int axis)
{
    pid.axis[axis].I = 0;
}

void INIT_CODE pidResetIterms(void)
{
    pid.axis[PID_ROLL].I  = 0;
    pid.axis[PID_PITCH].I = 0;
    pid.axis[PID_YAW].I   = 0;
}


static void INIT_CODE pidSetLooptime(uint32_t pidLooptime)
{
    pid.dT = pidLooptime * 1e-6f;
    pid.freq = 1.0f / pid.dT;

#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidSetLooptime(gyro.targetLooptime);
    pidInitProfile(pidProfile);
}

void INIT_CODE pidInitProfile(const pidProfile_t *pidProfile)
{
    pid.mode = constrain(pidProfile->mode, 1, 6);

    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = constrain(pidProfile->error_limit[i], 0, 360);

    pid.errorDecay = 1.0f - (pidProfile->error_decay) ? pid.dT * 10 / pidProfile->error_decay : 0;

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}


/*
 * 2D Rotation matrix
 *
 *        | cos(r)   -sin r |
 *    R = |                 |
 *        | sin(r)    cos r |
 *
 *
 *                3     5     7     9
 *               x     x     x     x
 * sin(x) = x - --- + --- - --- + --- - ...
 *               3!    5!    7!    9!
 *
 *                2     4     6     8
 *               x     x     x     x
 * cos(x) = 1 - --- + --- - --- + --- - ...
 *               2!    4!    6!    8!
 *
 *
 * For very small values of x, sin(x) ~= x and cos(x) ~= 1.
 *
 * In the use case below, using two first terms gives nearly 24bits of
 * resolution, which is close to what can be stored in a float.
 */

static inline void rotateAxisError(void)
{
    if (pid.errorRotation) {
        const float x = pid.axis[PID_ROLL].axisError;
        const float y = pid.axis[PID_PITCH].axisError;
        const float r = gyro.gyroADCf[Z] * RAD * pid.dT;

        const float t = r * r / 2;
        const float s = r * (1 - t / 3);
        const float c = 1 - t;

        pid.axis[PID_ROLL].axisError  = x * c + y * s;
        pid.axis[PID_PITCH].axisError = y * c - x * s;
    }
}

static FAST_CODE void pidApplyPrecomp(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

    // Yaw precompensation direction
    const float rotSign = mixerRotationSign();

    // Get stick throws (from previous cycle)
    const float cyclicDeflection = getCyclicDeflection();
    const float collectiveDeflection = getCollectiveDeflection();

    // Collective pitch impulse filter - TODO replace with proper filter
    pid.precomp.collectiveDeflectionLPF += (collectiveDeflection - pid.precomp.collectiveDeflectionLPF) * pid.precomp.collectiveImpulseFilterGain;
    const float collectiveDeflectionHPF = collectiveDeflection - pid.precomp.collectiveDeflectionLPF;

    // Pitch precomp
    const float pitchCollectiveFF = collectiveDeflection * pid.precomp.pitchCollectiveFFGain;
    const float pitchCollectiveImpulseFF = collectiveDeflectionHPF * pid.precomp.pitchCollectiveImpulseFFGain;

    // Total pitch precomp
    const float pitchPrecomp = pitchCollectiveFF + pitchCollectiveImpulseFF;

    // Add to PITCH feedforward
    pid.axis[FD_PITCH].F += pitchPrecomp;
    pid.axis[FD_PITCH].pidSum += pitchPrecomp;

#if 0
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 0, lrintf(collectiveDeflectionHPF * 1000));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 1, lrintf(pitchCollectiveFF));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 2, lrintf(pitchCollectiveImpulseFF));
    DEBUG_SET(DEBUG_PITCH_PRECOMP, 3, lrintf(pitchPrecomp));

    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 0, collectiveDeflection * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 1, collectiveDeflectionLPF * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 2, collectiveDeflectionHPF * 1000);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 3, pitchCollectiveFF * 10);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 4, pitchCollectiveImpulseFF * 10);
    DEBUG32_SET(DEBUG_PITCH_PRECOMP, 5, pitchPrecomp * 10);

    // Collective components
    float tailCollectiveFF = fabsf(collectiveDeflection) * tailCollectiveFFGain;
    float tailCollectiveImpulseFF = fabsf(collectiveDeflectionHPF) * tailCollectiveImpulseFFGain;

    // Cyclic component
    float tailCyclicFF = fabsf(cyclicDeflection) * tailCyclicFFGain;

    // Calculate total precompensation
    float tailPrecomp = (tailCollectiveFF + tailCollectiveImpulseFF + tailCyclicFF + tailCenterOffset) * rotSign;

    // Add to YAW feedforward
    pid.axis[FD_YAW].F += tailPrecomp;
    pid.axis[FD_YAW].pidSum += tailPrecomp;

    DEBUG_SET(DEBUG_YAW_PRECOMP, 0, lrintf(tailCyclicFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 1, lrintf(tailCollectiveFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 2, lrintf(tailCollectiveImpulseFF));
    DEBUG_SET(DEBUG_YAW_PRECOMP, 3, lrintf(tailPrecomp));

    DEBUG32_SET(DEBUG_YAW_PRECOMP, 0, cyclicDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 1, tailCyclicFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 2, collectiveDeflection * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 3, tailCollectiveFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 4, collectiveDeflectionHPF * 1000);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 5, tailCollectiveImpulseFF * 10);
    DEBUG32_SET(DEBUG_YAW_PRECOMP, 6, tailPrecomp * 10);
#endif
}

static FAST_CODE void pidApplyCollective(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

    pid.collective = getRcSetpoint(FD_COLL);
}

static FAST_CODE void pidApplyCyclicMode1(const pidProfile_t *pidProfile, uint8_t axis)
{
    // Rate setpoint
    float setpoint = getRcSetpoint(axis);

#ifdef USE_ACC_XXX
    // Apply leveling
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | RESCUE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        setpoint = pidLevelApply(axis, setpoint);
    }
#ifdef USE_ACRO_TRAINER
    else {
        // Apply trainer
        setpoint = acroTrainerApply(axis, setpoint);
    }
#endif
#endif

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }


  //// P-term

    // Calculate P-component
    pid.axis[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dError = pt1FilterApply(&pid.dtermFilter[axis], errorRate);
    float dTerm = (dError - pid.axis[axis].prevError) * pid.freq;
    pid.axis[axis].prevError = dError;

    // Calculate D-component
    pid.axis[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (pid.axis[axis].axisError * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component
    pid.axis[axis].axisError = constrainf(pid.axis[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.axis[axis].I = pid.coef[axis].Ki * pid.axis[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.axis[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.axis[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.axis[axis].pidSum = pid.axis[axis].P + pid.axis[axis].I + pid.axis[axis].D + pid.axis[axis].F;

    // Save setpoint
    pid.axis[axis].setPoint = setpoint;
}


static FAST_CODE void pidApplyYawMode1(const pidProfile_t *pidProfile)
{
    const uint8_t axis = FD_YAW;

    // Rate setpoint
    float setpoint = getRcSetpoint(axis);

    // Get filtered gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Calculate error rate
    float errorRate = setpoint - gyroRate;

    // Limit error bandwidth
    if (pidProfile->error_cutoff[axis]) {
        errorRate = pt1FilterApply(&pid.errorFilter[axis], errorRate);
    }

    // Select stop gain
    float stopGain = (errorRate > 0) ? pid.yawCWStopGain : pid.yawCCWStopGain;


  //// P-term

    // Calculate P-component
    pid.axis[axis].P = pid.coef[axis].Kp * errorRate;


  //// D-term

    // Calculate D-term with bandwidth limit
    float dTerm = (errorRate - pid.axis[axis].prevError) * pid.freq;
    pid.axis[axis].prevError = errorRate;

    // Filter D-term * stopGain
    dTerm = pt1FilterApply(&pid.dtermFilter[axis], dTerm * stopGain);

    // Calculate D-component
    pid.axis[axis].D = pid.coef[axis].Kd * dTerm;


  //// I-term

    // I-term change
    float itermDelta = errorRate * pid.dT;

    // No accumulation if axis saturated
    if (pidAxisSaturated(axis)) {
        if (pid.axis[axis].axisError * itermDelta > 0) // Same sign and not zero
            itermDelta = 0;
    }

    // Calculate I-component
    pid.axis[axis].axisError = constrainf(pid.axis[axis].axisError + itermDelta, -pid.errorLimit[axis], pid.errorLimit[axis]);
    pid.axis[axis].I = pid.coef[axis].Ki * pid.axis[axis].axisError;

    // Apply I-term error decay
    if (!isSpooledUp()) {
        pid.axis[axis].axisError *= pid.errorDecay;
    }


  //// F-term

    // Calculate feedforward component
    float fTerm = setpoint;
    if (pidProfile->fterm_cutoff[axis]) {
        fTerm -= pt1FilterApply(&pid.ftermFilter[axis], setpoint);
    }
    pid.axis[axis].F = pid.coef[axis].Kf * fTerm;


  //// PID Sum

    // Calculate PID sum
    pid.axis[axis].pidSum = pid.axis[axis].P + pid.axis[axis].I + pid.axis[axis].D + pid.axis[axis].F;

    // Save setpoint
    pid.axis[axis].setPoint = setpoint;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Apply PID for each axis
    switch (pid.mode) {
        default:
            pidApplyCyclicMode1(pidProfile, PID_ROLL);
            pidApplyCyclicMode1(pidProfile, PID_PITCH);
            pidApplyYawMode1(pidProfile);
            break;
    }

    // Calculate stabilized collective
    pidApplyCollective(pidProfile);

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp(pidProfile);

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
