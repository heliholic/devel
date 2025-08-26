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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/config.h"

#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/governor.h"

#include "io/beeper.h"
#include "io/ledstrip.h"

#include "drivers/time.h"

#include "osd/osd.h"

#include "pg/rx.h"
#include "pg/adjustments.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "rc_adjustments.h"


//// Internal Types

typedef int     (*getVal_f)(int adjFunc);
typedef void    (*setVal_f)(int adjFunc, int value);

typedef struct {
    char *      cfgName;
    getVal_f    cfgGet;
    setVal_f    cfgSet;
    int         cfgMin;
    int         cfgMax;
} adjustmentConfig_t;

typedef struct {
    timeMs_t deadTime;
    timeMs_t trigTime;
    int      adjValue;
    int      chValue;
} adjustmentState_t;


//// Internal Data

static adjustmentState_t adjustmentState[MAX_ADJUSTMENT_RANGE_COUNT];

static timeMs_t       adjustmentTime   = 0;
static const char *   adjustmentName   = NULL;
static int            adjustmentFunc   = 0;
static int            adjustmentValue  = 0;



//// Internal Functions

static int adjNullGet(int __unused adjFunc)
{
    return 0;
}

static void adjNullSet(int __unused adjFunc, int __unused value)
{
    // Nothing
}

static int adjustProfileGet(int adjFunc)
{
    int value = 0;

    switch (adjFunc) {
        case ADJUSTMENT_RATE_PROFILE:
            value = getCurrentControlRateProfileIndex() + 1;
            break;
        case ADJUSTMENT_PID_PROFILE:
            value = getCurrentPidProfileIndex() + 1;
            break;
        case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
            value = getLedProfile() + 1;
#endif
            break;
        case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
            value = getCurrentOsdProfileIndex();
#endif
            break;
    }

    return value;
}

static void adjustProfileSet(int adjFunc, int value)
{
    switch (adjFunc) {
        case ADJUSTMENT_RATE_PROFILE:
            changeControlRateProfile(value - 1);
            break;
        case ADJUSTMENT_PID_PROFILE:
            changePidProfile(value - 1);
            break;
        case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
            setLedProfile(value - 1);
#endif
            break;
        case ADJUSTMENT_OSD_PROFILE:
#ifdef USE_OSD_PROFILES
            changeOsdProfileIndex(value);
#endif
            break;
    }
}

static int adjPidsGet(int adjFunc)
{
    int value = 0;

    switch (adjFunc) {
        case ADJUSTMENT_PITCH_P_GAIN:
            value = currentPidProfile->pid[PID_PITCH].P;
            break;
        case ADJUSTMENT_ROLL_P_GAIN:
            value = currentPidProfile->pid[PID_ROLL].P;
            break;
        case ADJUSTMENT_YAW_P_GAIN:
            value = currentPidProfile->pid[PID_YAW].P;
            break;
        case ADJUSTMENT_PITCH_I_GAIN:
            value = currentPidProfile->pid[PID_PITCH].I;
            break;
        case ADJUSTMENT_ROLL_I_GAIN:
            value = currentPidProfile->pid[PID_ROLL].I;
            break;
        case ADJUSTMENT_YAW_I_GAIN:
            value = currentPidProfile->pid[PID_YAW].I;
            break;
        case ADJUSTMENT_PITCH_D_GAIN:
            value = currentPidProfile->pid[PID_PITCH].D;
            break;
        case ADJUSTMENT_ROLL_D_GAIN:
            value = currentPidProfile->pid[PID_ROLL].D;
            break;
        case ADJUSTMENT_YAW_D_GAIN:
            value = currentPidProfile->pid[PID_YAW].D;
            break;
        case ADJUSTMENT_PITCH_F_GAIN:
            value = currentPidProfile->pid[PID_PITCH].F;
            break;
        case ADJUSTMENT_ROLL_F_GAIN:
            value = currentPidProfile->pid[PID_ROLL].F;
            break;
        case ADJUSTMENT_YAW_F_GAIN:
            value = currentPidProfile->pid[PID_YAW].F;
            break;
        case ADJUSTMENT_YAW_CW_GAIN:
            value = currentPidProfile->yaw_cw_stop_gain;
            break;
        case ADJUSTMENT_YAW_CCW_GAIN:
            value = currentPidProfile->yaw_ccw_stop_gain;
            break;
        case ADJUSTMENT_YAW_CYCLIC_FF:
            value = currentPidProfile->yaw_cyclic_ff_gain;
            break;
        case ADJUSTMENT_YAW_COLLECTIVE_FF:
            value = currentPidProfile->yaw_collective_ff_gain;
            break;
        case ADJUSTMENT_PITCH_COLLECTIVE_FF:
            value = currentPidProfile->pitch_collective_ff_gain;
            break;
        case ADJUSTMENT_PITCH_GYRO_CUTOFF:
            value = currentPidProfile->gyro_cutoff[PID_PITCH];
            break;
        case ADJUSTMENT_ROLL_GYRO_CUTOFF:
            value = currentPidProfile->gyro_cutoff[PID_ROLL];
            break;
        case ADJUSTMENT_YAW_GYRO_CUTOFF:
            value = currentPidProfile->gyro_cutoff[PID_YAW];
            break;
        case ADJUSTMENT_PITCH_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_PITCH];
            break;
        case ADJUSTMENT_ROLL_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_ROLL];
            break;
        case ADJUSTMENT_YAW_DTERM_CUTOFF:
            value = currentPidProfile->dterm_cutoff[PID_YAW];
            break;
        case ADJUSTMENT_ANGLE_LEVEL_GAIN:
            value = currentPidProfile->angle.level_strength;
            break;
        case ADJUSTMENT_HORIZON_LEVEL_GAIN:
            value = currentPidProfile->horizon.level_strength;
            break;
        case ADJUSTMENT_ACRO_TRAINER_GAIN:
            value = currentPidProfile->trainer.gain;
            break;
        case ADJUSTMENT_PITCH_B_GAIN:
            value = currentPidProfile->pid[PID_PITCH].B;
            break;
        case ADJUSTMENT_ROLL_B_GAIN:
            value = currentPidProfile->pid[PID_ROLL].B;
            break;
        case ADJUSTMENT_YAW_B_GAIN:
            value = currentPidProfile->pid[PID_YAW].B;
            break;
        case ADJUSTMENT_PITCH_O_GAIN:
            value = currentPidProfile->pid[PID_PITCH].O;
            break;
        case ADJUSTMENT_ROLL_O_GAIN:
            value = currentPidProfile->pid[PID_ROLL].O;
            break;
        case ADJUSTMENT_CROSS_COUPLING_GAIN:
            value = currentPidProfile->cyclic_cross_coupling_gain;
            break;
        case ADJUSTMENT_CROSS_COUPLING_RATIO:
            value = currentPidProfile->cyclic_cross_coupling_ratio;
            break;
        case ADJUSTMENT_CROSS_COUPLING_CUTOFF:
            value = currentPidProfile->cyclic_cross_coupling_cutoff;
            break;
        case ADJUSTMENT_INERTIA_PRECOMP_GAIN:
            value = currentPidProfile->yaw_inertia_precomp_gain;
            break;
        case ADJUSTMENT_INERTIA_PRECOMP_CUTOFF:
            value = currentPidProfile->yaw_inertia_precomp_cutoff;
            break;
        case ADJUSTMENT_YAW_PRECOMP_CUTOFF:
            value = currentPidProfile->yaw_precomp_cutoff;
            break;
    }

    return value;
}

static void adjPidsSet(int adjFunc, int value)
{
    switch (adjFunc) {
        case ADJUSTMENT_PITCH_P_GAIN:
            currentPidProfile->pid[PID_PITCH].P = value;
            break;
        case ADJUSTMENT_ROLL_P_GAIN:
            currentPidProfile->pid[PID_ROLL].P = value;
            break;
        case ADJUSTMENT_YAW_P_GAIN:
            currentPidProfile->pid[PID_YAW].P = value;
            break;
        case ADJUSTMENT_PITCH_I_GAIN:
            currentPidProfile->pid[PID_PITCH].I = value;
            break;
        case ADJUSTMENT_ROLL_I_GAIN:
            currentPidProfile->pid[PID_ROLL].I = value;
            break;
        case ADJUSTMENT_YAW_I_GAIN:
            currentPidProfile->pid[PID_YAW].I = value;
            break;
        case ADJUSTMENT_PITCH_D_GAIN:
            currentPidProfile->pid[PID_PITCH].D = value;
            break;
        case ADJUSTMENT_ROLL_D_GAIN:
            currentPidProfile->pid[PID_ROLL].D = value;
            break;
        case ADJUSTMENT_YAW_D_GAIN:
            currentPidProfile->pid[PID_YAW].D = value;
            break;
        case ADJUSTMENT_PITCH_F_GAIN:
            currentPidProfile->pid[PID_PITCH].F = value;
            break;
        case ADJUSTMENT_ROLL_F_GAIN:
            currentPidProfile->pid[PID_ROLL].F = value;
            break;
        case ADJUSTMENT_YAW_F_GAIN:
            currentPidProfile->pid[PID_YAW].F = value;
            break;
        case ADJUSTMENT_YAW_CW_GAIN:
            currentPidProfile->yaw_cw_stop_gain = value;
            break;
        case ADJUSTMENT_YAW_CCW_GAIN:
            currentPidProfile->yaw_ccw_stop_gain = value;
            break;
        case ADJUSTMENT_YAW_CYCLIC_FF:
            currentPidProfile->yaw_cyclic_ff_gain = value;
            break;
        case ADJUSTMENT_YAW_COLLECTIVE_FF:
            currentPidProfile->yaw_collective_ff_gain = value;
            break;
        case ADJUSTMENT_PITCH_COLLECTIVE_FF:
            currentPidProfile->pitch_collective_ff_gain = value;
            break;
        case ADJUSTMENT_PITCH_GYRO_CUTOFF:
            currentPidProfile->gyro_cutoff[PID_PITCH] = value;
            break;
        case ADJUSTMENT_ROLL_GYRO_CUTOFF:
            currentPidProfile->gyro_cutoff[PID_ROLL] = value;
            break;
        case ADJUSTMENT_YAW_GYRO_CUTOFF:
            currentPidProfile->gyro_cutoff[PID_YAW] = value;
            break;
        case ADJUSTMENT_PITCH_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_PITCH] = value;
            break;
        case ADJUSTMENT_ROLL_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_ROLL] = value;
            break;
        case ADJUSTMENT_YAW_DTERM_CUTOFF:
            currentPidProfile->dterm_cutoff[PID_YAW] = value;
            break;
        case ADJUSTMENT_ANGLE_LEVEL_GAIN:
            currentPidProfile->angle.level_strength = value;
            break;
        case ADJUSTMENT_HORIZON_LEVEL_GAIN:
            currentPidProfile->horizon.level_strength = value;
            break;
        case ADJUSTMENT_ACRO_TRAINER_GAIN:
            currentPidProfile->trainer.gain = value;
            break;
        case ADJUSTMENT_PITCH_B_GAIN:
            currentPidProfile->pid[PID_PITCH].B = value;
            break;
        case ADJUSTMENT_ROLL_B_GAIN:
            currentPidProfile->pid[PID_ROLL].B = value;
            break;
        case ADJUSTMENT_YAW_B_GAIN:
            currentPidProfile->pid[PID_YAW].B = value;
            break;
        case ADJUSTMENT_PITCH_O_GAIN:
            currentPidProfile->pid[PID_PITCH].O = value;
            break;
        case ADJUSTMENT_ROLL_O_GAIN:
            currentPidProfile->pid[PID_ROLL].O = value;
            break;
        case ADJUSTMENT_CROSS_COUPLING_GAIN:
            currentPidProfile->cyclic_cross_coupling_gain = value;
            break;
        case ADJUSTMENT_CROSS_COUPLING_RATIO:
            currentPidProfile->cyclic_cross_coupling_ratio = value;
            break;
        case ADJUSTMENT_CROSS_COUPLING_CUTOFF:
            currentPidProfile->cyclic_cross_coupling_cutoff = value;
            break;
        case ADJUSTMENT_INERTIA_PRECOMP_GAIN:
            currentPidProfile->yaw_inertia_precomp_gain = value;
            break;
        case ADJUSTMENT_INERTIA_PRECOMP_CUTOFF:
            currentPidProfile->yaw_inertia_precomp_cutoff = value;
            break;
        case ADJUSTMENT_YAW_PRECOMP_CUTOFF:
            currentPidProfile->yaw_precomp_cutoff = value;
            break;
    }

    pidLoadProfile(currentPidProfile);
}

static int adjRescueGet(int adjFunc)
{
    int value = 0;

    switch (adjFunc) {
        case ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE:
            value = currentPidProfile->rescue.climb_collective;
            break;
        case ADJUSTMENT_RESCUE_HOVER_COLLECTIVE:
            value = currentPidProfile->rescue.hover_collective;
            break;
        case ADJUSTMENT_RESCUE_HOVER_ALTITUDE:
            value = currentPidProfile->rescue.hover_altitude;
            break;
        case ADJUSTMENT_RESCUE_ALT_P_GAIN:
            value = currentPidProfile->rescue.alt_p_gain;
            break;
        case ADJUSTMENT_RESCUE_ALT_I_GAIN:
            value = currentPidProfile->rescue.alt_i_gain;
            break;
        case ADJUSTMENT_RESCUE_ALT_D_GAIN:
            value = currentPidProfile->rescue.alt_d_gain;
            break;
    }

    return value;
}

static void adjRescueSet(int adjFunc, int value)
{
    switch (adjFunc) {
        case ADJUSTMENT_RESCUE_CLIMB_COLLECTIVE:
            currentPidProfile->rescue.climb_collective = value;
            break;
        case ADJUSTMENT_RESCUE_HOVER_COLLECTIVE:
            currentPidProfile->rescue.hover_collective = value;
            break;
        case ADJUSTMENT_RESCUE_HOVER_ALTITUDE:
            currentPidProfile->rescue.hover_altitude = value;
            break;
        case ADJUSTMENT_RESCUE_ALT_P_GAIN:
            currentPidProfile->rescue.alt_p_gain = value;
            break;
        case ADJUSTMENT_RESCUE_ALT_I_GAIN:
            currentPidProfile->rescue.alt_i_gain = value;
            break;
        case ADJUSTMENT_RESCUE_ALT_D_GAIN:
            currentPidProfile->rescue.alt_d_gain = value;
            break;
    }
}

static int adjustMiscGet(int adjFunc)
{
    int value = 0;

    switch (adjFunc) {
        case ADJUSTMENT_ACC_TRIM_PITCH:
            value = accelerometerConfig()->accelerometerTrims.values.pitch;
            break;
        case ADJUSTMENT_ACC_TRIM_ROLL:
            value = accelerometerConfig()->accelerometerTrims.values.roll;
            break;
    }

    return value;
}

static void adjustMiscSet(int adjFunc, int value)
{
    switch (adjFunc) {
        case ADJUSTMENT_ACC_TRIM_PITCH:
            accelerometerConfigMutable()->accelerometerTrims.values.pitch = value;
            break;
        case ADJUSTMENT_ACC_TRIM_ROLL:
            accelerometerConfigMutable()->accelerometerTrims.values.roll = value;
            break;
    }
}


#define ADJ_CONFIG(id, fun, min, max)  [ADJUSTMENT_##id] = \
    {                                                       \
        .cfgName  = #id,                                    \
        .cfgGet   = fun##Get,                               \
        .cfgSet   = fun##Set,                               \
        .cfgMin   = (min),                                  \
        .cfgMax   = (max),                                  \
    }

static const adjustmentConfig_t adjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT] =
{
    ADJ_CONFIG(NONE,                    adjNull,                0, 0),

    ADJ_CONFIG(RATE_PROFILE,            adjustProfile,          1, 6),
    ADJ_CONFIG(PID_PROFILE,             adjustProfile,          1, 6),
    ADJ_CONFIG(LED_PROFILE,             adjustProfile,          1, 4),
    ADJ_CONFIG(OSD_PROFILE,             adjustProfile,          1, 3),

    ADJ_CONFIG(PITCH_RATE,              adjustControlRate,      0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(ROLL_RATE,               adjustControlRate,      0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(YAW_RATE,                adjustControlRate,      0, CONTROL_RATE_CONFIG_SUPER_RATE_MAX),
    ADJ_CONFIG(PITCH_RC_RATE,           adjustControlRate,      1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(ROLL_RC_RATE,            adjustControlRate,      1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(YAW_RC_RATE,             adjustControlRate,      1, CONTROL_RATE_CONFIG_RC_RATES_MAX),
    ADJ_CONFIG(PITCH_RC_EXPO,           adjustControlRate,      0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(ROLL_RC_EXPO,            adjustControlRate,      0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),
    ADJ_CONFIG(YAW_RC_EXPO,             adjustControlRate,      0, CONTROL_RATE_CONFIG_RC_EXPO_MAX),

    ADJ_CONFIG(PITCH_SP_BOOST_GAIN,     adjustControlRate,      0, 255),
    ADJ_CONFIG(ROLL_SP_BOOST_GAIN,      adjustControlRate,      0, 255),
    ADJ_CONFIG(YAW_SP_BOOST_GAIN,       adjustControlRate,      0, 255),
    ADJ_CONFIG(COLL_SP_BOOST_GAIN,      adjustControlRate,      0, 255),

    ADJ_CONFIG(YAW_DYN_CEILING_GAIN,    adjustControlRate,      0, 250),
    ADJ_CONFIG(YAW_DYN_DEADBAND_GAIN,   adjustControlRate,      0, 250),
    ADJ_CONFIG(YAW_DYN_DEADBAND_FILTER, adjustControlRate,      0, 250),

    ADJ_CONFIG(PITCH_P_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(PITCH_I_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(PITCH_D_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(PITCH_F_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_P_GAIN,             adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_I_GAIN,             adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_D_GAIN,             adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_F_GAIN,             adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(YAW_P_GAIN,              adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(YAW_I_GAIN,              adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(YAW_D_GAIN,              adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(YAW_F_GAIN,              adjPids,                0, PID_GAIN_MAX),

    ADJ_CONFIG(PITCH_B_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_B_GAIN,             adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(YAW_B_GAIN,              adjPids,                0, PID_GAIN_MAX),

    ADJ_CONFIG(PITCH_O_GAIN,            adjPids,                0, PID_GAIN_MAX),
    ADJ_CONFIG(ROLL_O_GAIN,             adjPids,                0, PID_GAIN_MAX),

    ADJ_CONFIG(YAW_CW_GAIN,             adjPids,                25, 250),
    ADJ_CONFIG(YAW_CCW_GAIN,            adjPids,                25, 250),
    ADJ_CONFIG(YAW_CYCLIC_FF,           adjPids,                0, 250),
    ADJ_CONFIG(YAW_COLLECTIVE_FF,       adjPids,                0, 250),
    ADJ_CONFIG(YAW_COLLECTIVE_DYN,      adjPids,                -125, 125),
    ADJ_CONFIG(YAW_COLLECTIVE_DECAY,    adjPids,                1, 250),

    ADJ_CONFIG(PITCH_COLLECTIVE_FF,     adjPids,                0, 250),

    ADJ_CONFIG(PITCH_GYRO_CUTOFF,       adjPids,                0, 250),
    ADJ_CONFIG(ROLL_GYRO_CUTOFF,        adjPids,                0, 250),
    ADJ_CONFIG(YAW_GYRO_CUTOFF,         adjPids,                0, 250),

    ADJ_CONFIG(PITCH_DTERM_CUTOFF,      adjPids,                0, 250),
    ADJ_CONFIG(ROLL_DTERM_CUTOFF,       adjPids,                0, 250),
    ADJ_CONFIG(YAW_DTERM_CUTOFF,        adjPids,                0, 250),

    ADJ_CONFIG(YAW_PRECOMP_CUTOFF,      adjPids,                0, 250),

    ADJ_CONFIG(INERTIA_PRECOMP_GAIN,    adjPids,                0, 250),
    ADJ_CONFIG(INERTIA_PRECOMP_CUTOFF,  adjPids,                0, 250),

    ADJ_CONFIG(CROSS_COUPLING_GAIN,     adjPids,                0, 250),
    ADJ_CONFIG(CROSS_COUPLING_RATIO,    adjPids,                0, 200),
    ADJ_CONFIG(CROSS_COUPLING_CUTOFF,   adjPids,                1, 250),

    ADJ_CONFIG(ANGLE_LEVEL_GAIN,        adjPids,                0, 200),
    ADJ_CONFIG(HORIZON_LEVEL_GAIN,      adjPids,                0, 200),
    ADJ_CONFIG(ACRO_TRAINER_GAIN,       adjPids,                25, 255),

    ADJ_CONFIG(RESCUE_CLIMB_COLLECTIVE, adjRescue,              0, 1000),
    ADJ_CONFIG(RESCUE_HOVER_COLLECTIVE, adjRescue,              0, 1000),
    ADJ_CONFIG(RESCUE_HOVER_ALTITUDE,   adjRescue,              0, 2500),
    ADJ_CONFIG(RESCUE_ALT_P_GAIN,       adjRescue,              0, 1000),
    ADJ_CONFIG(RESCUE_ALT_I_GAIN,       adjRescue,              0, 1000),
    ADJ_CONFIG(RESCUE_ALT_D_GAIN,       adjRescue,              0, 1000),

    ADJ_CONFIG(GOV_GAIN,                adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_P_GAIN,              adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_I_GAIN,              adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_D_GAIN,              adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_F_GAIN,              adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_TTA_GAIN,            adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_CYCLIC_FF,           adjustGovernor,         0, 250),
    ADJ_CONFIG(GOV_COLLECTIVE_FF,       adjustGovernor,         0, 250),

    ADJ_CONFIG(ACC_TRIM_PITCH,          adjustMisc,             -300, 300),
    ADJ_CONFIG(ACC_TRIM_ROLL,           adjustMisc,             -300, 300),

};


static void blackboxAdjustmentEvent(int adjFunc, int value)
{
#ifndef USE_BLACKBOX
    UNUSED(adjFunc);
    UNUSED(value);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjFunc;
        eventData.newValue = value;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

#define ADJUSTMENT_LATENCY_MS 3000

static void updateAdjustmentData(int adjFunc, int value)
{
    const timeMs_t now = millis();

    if (adjFunc != ADJUSTMENT_NONE &&
#ifdef USE_OSD_PROFILES
        adjFunc != ADJUSTMENT_OSD_PROFILE &&
#endif
        adjFunc != ADJUSTMENT_PID_PROFILE &&
        adjFunc != ADJUSTMENT_RATE_PROFILE)
    {
        adjustmentTime   = now;
        adjustmentName   = adjustmentConfigs[adjFunc].cfgName;
        adjustmentFunc   = adjFunc;
        adjustmentValue  = value;
    }

    if (cmp32(now, adjustmentTime) > ADJUSTMENT_LATENCY_MS) {
        adjustmentName = NULL;
    }
}

#define REPEAT_DELAY     500
#define TRIGGER_DELAY    100

void processRcAdjustments(void)
{
    bool changed = false;

    if (rxIsReceivingSignal())
    {
        for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++)
        {
            const adjustmentRange_t * adjRange = adjustmentRanges(index);
            const uint8_t adjFunc = adjRange->function;

            if (adjRange->enaChannel == 0xff || isRangeActive(adjRange->enaChannel, &adjRange->enaRange))
            {
                const adjustmentConfig_t * adjConfig = &adjustmentConfigs[adjFunc];
                adjustmentState_t * adjState = &adjustmentState[index];
                const timeMs_t now = millis();

                int adjval = adjState->adjValue;

                if (cmp32(now, adjState->deadTime) < 0)
                    continue;

                const int chValue = rcInput[adjRange->adjChannel + CONTROL_CHANNEL_COUNT];

                // Stepped adjustment
                if (adjRange->adjStep) {
                    if (abs(chValue - adjState->chValue) > 2) {
                        adjState->trigTime = now + TRIGGER_DELAY;
                        adjState->chValue = chValue;
                        continue;
                    }
                    if (cmp32(now, adjState->trigTime) < 0) {
                        continue;
                    }
                    if (isRangeActive(adjRange->adjChannel, &adjRange->adjRange1)) {
                        adjval = adjConfig->cfgGet(adjFunc) - adjRange->adjStep;
                    }
                    else if (isRangeActive(adjRange->adjChannel, &adjRange->adjRange2)) {
                        adjval = adjConfig->cfgGet(adjFunc) + adjRange->adjStep;
                    }
                    else {
                        continue;
                    }
                }
                // Continuous adjustment
                else {
                    const int rangeLower = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.startStep);
                    const int rangeUpper = STEP_TO_CHANNEL_VALUE(adjRange->adjRange1.endStep);
                    const int rangeWidth = rangeUpper - rangeLower;
                    const int valueWidth = adjRange->adjMax - adjRange->adjMin;

                    if (rangeWidth > 0 && valueWidth > 0) {
                        const int rangeMargin = MAX(5, rangeWidth / (valueWidth * 2));
                        if (chValue > rangeLower - rangeMargin && chValue < rangeUpper + rangeMargin) {
                            const int offset = rangeWidth / 2;
                            adjval = adjRange->adjMin + ((chValue - rangeLower) * valueWidth + offset) / rangeWidth;
                        }
                    }
                }

                adjval = constrain(adjval, adjConfig->cfgMin, adjConfig->cfgMax);
                adjval = constrain(adjval, adjRange->adjMin, adjRange->adjMax);

                if (adjval != adjState->adjValue) {
                    adjConfig->cfgSet(adjFunc, adjval);

                    updateAdjustmentData(adjFunc, adjval);
                    blackboxAdjustmentEvent(adjFunc, adjval);

                    // PID profile change does it's own confirmation, no of beeps eq profile no,
                    // a single beep here will kill that.
                    if (adjFunc != ADJUSTMENT_PID_PROFILE) {
                      beeperConfirmationBeeps(1);
                    }
                    setConfigDirty();

                    adjState->deadTime = now + REPEAT_DELAY;
                    adjState->adjValue = adjval;

                    changed = true;
                }
            }
        }
    }

    if (!changed) {
        updateAdjustmentData(ADJUSTMENT_NONE, 0);
    }
}

const char * getAdjustmentsRangeName(void)
{
    return adjustmentName;
}

int getAdjustmentsRangeFunc(void)
{
    return adjustmentFunc;
}

int getAdjustmentsRangeValue(void)
{
    return adjustmentValue;
}

void adjustmentRangeInit(void)
{
    for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        adjustmentRangeReset(index);
    }
}

void adjustmentRangeReset(int index)
{
    const int adjFunc = adjustmentRanges(index)->function;
    const adjustmentConfig_t * adjConfig = &adjustmentConfigs[adjFunc];

    adjustmentState[index].adjValue = adjConfig->cfgGet(adjFunc);
    adjustmentState[index].deadTime = 0;
    adjustmentState[index].trigTime = 0;
    adjustmentState[index].chValue  = 0;
}
