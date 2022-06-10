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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "drivers/dshot_command.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/feedforward.h"
#include "flight/pid.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/rpm_filter.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pid_init.h"

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 1e-6f;
    pidRuntime.pidFrequency = 1.0f / pidRuntime.dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
#endif
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitFeedforwardLpf(uint16_t filterCutoff, uint8_t debugAxis)
{
    pidRuntime.rcSmoothingDebugAxis = debugAxis;
    if (filterCutoff > 0) {
        pidRuntime.feedforwardLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            pt3FilterInit(&pidRuntime.feedforwardPt3[axis], pt3FilterGain(filterCutoff, pidRuntime.dT));
        }
    }
}

void pidUpdateFeedforwardLpf(uint16_t filterCutoff)
{
    if (filterCutoff > 0) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            pt3FilterUpdateCutoff(&pidRuntime.feedforwardPt3[axis], pt3FilterGain(filterCutoff, pidRuntime.dT));
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

void pidInitConfig(const pidProfile_t *pidProfile)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
    }

    pidRuntime.maxVelocity[FD_ROLL] = pidRuntime.maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.itermLimit = pidProfile->itermLimit;
    pidRuntime.itermRotation = pidProfile->iterm_rotation;

#ifdef USE_ACC
    pidLevelInit(pidProfile);
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif

#ifdef USE_FEEDFORWARD
    if (pidProfile->feedforward_transition == 0) {
        pidRuntime.feedforwardTransitionFactor = 0;
    } else {
        pidRuntime.feedforwardTransitionFactor = 100.0f / pidProfile->feedforward_transition;
    }
    pidRuntime.feedforwardAveraging = pidProfile->feedforward_averaging;
    pidRuntime.feedforwardSmoothFactor = 1.0f;
    if (pidProfile->feedforward_smooth_factor) {
        pidRuntime.feedforwardSmoothFactor = 1.0f - ((float)pidProfile->feedforward_smooth_factor) / 100.0f;
    }
    pidRuntime.feedforwardJitterFactor = pidProfile->feedforward_jitter_factor;
    pidRuntime.feedforwardBoostFactor = (float)pidProfile->feedforward_boost / 10.0f;

    feedforwardInit(pidProfile);
#endif
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

