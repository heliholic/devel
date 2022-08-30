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
    return pid.data[axis].setPoint;
}

float pidGetPidSum(int axis)
{
    return pid.data[axis].pidSum;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));
}

void pidResetIterm(int axis)
{
    pid.data[axis].I = 0;
}

void pidResetIterms(void)
{
    pid.data[PID_ROLL].I  = 0;
    pid.data[PID_PITCH].I = 0;
    pid.data[PID_YAW].I   = 0;
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

static inline void rotateVector(float *x, float *y, float r)
{
    float t = r * r / 2;
    float s = r * (1 - t / 3);
    float c = 1 - t;

    float a = *x*c + *y*s;
    float b = *y*c - *x*s;

    *x = a;
    *y = b;
}

static inline void rotateAxisError(void)
{
    if (pid.errorRotation) {
        rotateVector(&pid.data[PID_ROLL].axisError, &pid.data[PID_PITCH].axisError, gyro.gyroADCf[Z]*pid.dT*RAD);
    }
}

static FAST_CODE void pidApplyCyclic(const pidProfile_t *pidProfile, uint8_t axis)
{
    UNUSED(pidProfile);
    UNUSED(axis);

}

static FAST_CODE void pidApplyYaw(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

}

static FAST_CODE void pidApplyPrecomp(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

}

static FAST_CODE void pidApplyCollective(const pidProfile_t *pidProfile)
{
    UNUSED(pidProfile);

}

FAST_CODE void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    // Apply PID for each axis
    pidApplyCyclic(pidProfile, FD_ROLL);
    pidApplyCyclic(pidProfile, FD_PITCH);
    pidApplyYaw(pidProfile);

    // Calculate cyclic/collective precompensation
    pidApplyPrecomp(pidProfile);

    // Calculate stabilized collective
    pidApplyCollective(pidProfile);

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
