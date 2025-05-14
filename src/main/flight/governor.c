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

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rx/rx.h"

#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"


// Throttle mapping in IDLE state
#define GOV_THROTTLE_OFF_LIMIT          0.05f

// Headspeed quality levels
#define GOV_HS_DETECT_DELAY             200
#define GOV_HS_DETECT_RATIO             0.05f

// RPM glitch ratio
#define GOV_HS_GLITCH_DELTA             0.25f
#define GOV_HS_GLITCH_LIMIT             2.0f

// Lost headspeed levels
#define GOV_HS_INVALID_RATIO            0.01f
#define GOV_HS_INVALID_THROTTLE         0.10f

// Nominal battery cell voltage
#define GOV_NOMINAL_CELL_VOLTAGE        3.70f


//// Internal Data

typedef struct {

    // Governor type
    govType_e       govType;

    // Features
    bool            precompEnabled;
    bool            vCompEnabled;
    bool            autoEnabled;
    bool            ttaEnabled;

    // State machine
    govState_e      state;
    timeMs_t        stateEntryTime;

    // Output throttle
    float           throttleOutput;

    // Input throttle mode
    govThrMode_e    throttleMode;

    // Input throttle
    float           throttleInput;
    bool            throttleInputOff;
    float           throttlePrevInput;

    // Gov Active throttle limits
    float           minActiveThrottle;
    float           maxActiveThrottle;

    // Spoolup throttle levels
    float           maxIdleThrottle;
    float           minSpoolupThrottle;
    float           maxSpoolupThrottle;

    // Current headspeed
    float           currentHeadSpeed;

    // Headspeed config
    float           fullHeadSpeed;
    float           targetHeadSpeed;
    float           requestedHeadSpeed;

    // Proportial headspeeds
    float           requestRatio;
    float           fullHeadSpeedRatio;

    // Main gear ratio (motor/head)
    float           mainGearRatio;

    // RPM Signal Flags
    bool            motorRPMError;
    bool            motorRPMPresent;

    // RPM filter & detection
    filter_t        motorRPMFilter;
    float           motorRPMGlitchDelta;
    float           motorRPMGlitchLimit;
    uint32_t        motorRPMDetectTime;

    // Battery voltage & current
    float           motorVoltage;
    filter_t        motorVoltageFilter;
    float           motorCurrent;
    filter_t        motorCurrentFilter;

    // Nominal battery voltage
    float           nominalVoltage;

    // Voltage compensation gain
    float           vCompGain;

    // PID terms
    float           P;
    float           I;
    float           C;
    float           D;
    float           F;
    float           pidSum;

    // Differentiator with bandwidth limiter
    difFilter_t     differentiator;

    // PID Gains
    float           K;
    float           Kp;
    float           Ki;
    float           Kd;
    float           Kf;

    // PID Limits
    float           Lp;
    float           Li;
    float           Ld;
    float           Lf;

    // Feedforward
    float           yawWeight;
    float           cyclicWeight;
    float           collectiveWeight;
    filter_t        FFFilter;

    // Tail Torque Assist
    float           TTAAdd;
    float           TTAGain;
    float           TTALimit;
    filter_t        TTAFilter;

    // Autorotation
    long            autoMinEntry;
    long            autoTimeout;

    // Timeouts
    long            zeroThrottleTimeout;
    long            lostHeadspeedTimeout;

    // Headspeed change rates
    float           headSpeedSpoolupRate;
    float           headSpeedBailoutRate;
    float           headSpeedRecoveryRate;
    float           headSpeedTrackingRate;

    // Throttle change rates
    float           throttleStartupRate;
    float           throttleSpoolupRate;
    float           throttleBailoutRate;
    float           throttleRecoveryRate;
    float           throttleTrackingRate;

} govData_t;

static FAST_DATA_ZERO_INIT govData_t gov;


//// Handler functions

typedef void  (*govVoidFn)(void);
typedef float (*govFloatFn)(void);

static FAST_DATA_ZERO_INIT govVoidFn   govStateUpdate;

static FAST_DATA_ZERO_INIT govVoidFn   govSpoolupInit;
static FAST_DATA_ZERO_INIT govFloatFn  govSpoolupCalc;

static FAST_DATA_ZERO_INIT govVoidFn   govActiveInit;
static FAST_DATA_ZERO_INIT govFloatFn  govActiveCalc;


//// Prototypes

static void govPIDInit(void);
static float govPIDControl(void);

static void governorUpdateState(void);
static void governorUpdateExternal(void);



//// Access functions

int getGovernorState(void)
{
    return gov.state;
}

float getGovernorOutput(void)
{
    return gov.throttleOutput;
}

float getTTAIncrease(void)
{
    return gov.TTAAdd;
}

float getFullHeadSpeedRatio(void)
{
    if (gov.govType  > GOV_TYPE_EXTERNAL) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLING_UP:
            case GOV_STATE_ZERO_THROTTLE:
            case GOV_STATE_LOST_HEADSPEED:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_AUTOROTATION_BAILOUT:
                return gov.fullHeadSpeedRatio;
            default:
                return 1.0f;
        }
    }

    return 1.0f;
}

float getSpoolUpRatio(void)
{
    if (!ARMING_FLAG(ARMED))
        return 0;

    if (gov.govType ) {
        switch (gov.state)
        {
            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_IDLE:
            case GOV_STATE_ZERO_THROTTLE:
            case GOV_STATE_AUTOROTATION:
                return 0;

            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_LOST_HEADSPEED:
            case GOV_STATE_AUTOROTATION_BAILOUT:
                return 1.0f;

            case GOV_STATE_SPOOLING_UP:
                return gov.requestRatio;
        }
        return 0;
    }
    else {
        return 1.0f;
    }

    return 0;
}

bool isSpooledUp(void)
{
    if (!ARMING_FLAG(ARMED))
        return false;

    if (gov.govType ) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_AUTOROTATION_BAILOUT:
                return true;

            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLING_UP:
            case GOV_STATE_LOST_HEADSPEED:
                return (gov.throttleOutput > 0.333f);

            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_IDLE:
            case GOV_STATE_ZERO_THROTTLE:
                return false;
        }
        return false;
    }
    else {
        return (gov.throttleOutput > 0.333f);
    }

    return false;
}


//// Internal functions

static void govChangeState(govState_e futureState)
{
    gov.state = futureState;
    gov.stateEntryTime = millis();
}

static long govStateTime(void)
{
    return cmp32(millis(),gov.stateEntryTime);
}

static void govDebugStats(void)
{
    DEBUG(GOVERNOR, 0, gov.requestedHeadSpeed);
    DEBUG(GOVERNOR, 1, gov.targetHeadSpeed);
    DEBUG(GOVERNOR, 2, gov.currentHeadSpeed);
    DEBUG(GOVERNOR, 3, gov.pidSum * 1000);
    DEBUG(GOVERNOR, 4, gov.P * 1000);
    DEBUG(GOVERNOR, 5, gov.I * 1000);
    DEBUG(GOVERNOR, 6, gov.D * 1000);
    DEBUG(GOVERNOR, 7, gov.F * 1000);
}

static float idleMap(float throttle)
{

    // Map throttle in IDLE state (N is maxIdleThrottle)
    //     0%..5%    => 0%
    //     5%..N%    => 0%..N%
    //     >N%       => N%
    throttle = (throttle - GOV_THROTTLE_OFF_LIMIT) * gov.maxIdleThrottle /
        (gov.maxIdleThrottle - GOV_THROTTLE_OFF_LIMIT);

    return constrainf(throttle, 0, gov.maxIdleThrottle);
}

static void govGetInputThrottle(void)
{
    float throttle = getThrottle();
    bool throff = (getThrottleStatus() == THROTTLE_LOW);

    // Throttle modes
    switch (gov.throttleMode) {
        case GOV_THROTTLE_NONE:
            break;
        case GOV_THROTTLE_3POS:
            if (throttle < 0.333f) {
                throttle = 0;
                throff = true;
            }
            else if (throttle < 0.666f) {
                throttle = gov.maxIdleThrottle;
            }
            else {
                throttle = 1.0f;
            }
            break;
        case GOV_THROTTLE_STICK:
            if (throttle < gov.minActiveThrottle) {
                throttle = 0;
            }
            break;

        // FIXME
    }

    gov.throttleInput = throttle;
    gov.throttleInputOff = throff;

}

static void govActiveUpdate(void)
{
    // Assume motor[0]
    const float motorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    const float filteredRPM = filterApply(&gov.motorRPMFilter, motorRPM);

    // Calculate headspeed from filtered motor speed
    gov.currentHeadSpeed = filteredRPM * gov.mainGearRatio;

    // Calculate current HS vs FullHS ratio
    gov.fullHeadSpeedRatio = gov.currentHeadSpeed / gov.fullHeadSpeed;

    // Update headspeed target
    gov.requestedHeadSpeed = gov.throttleInput * gov.fullHeadSpeed;

    // Detect stuck motor / startup problem
    const bool rpmError = ((gov.fullHeadSpeedRatio < GOV_HS_INVALID_RATIO || motorRPM < 10) && gov.throttleOutput > GOV_HS_INVALID_THROTTLE);

    // Detect RPM glitches
    const bool rpmGlitch = (motorRPM - filteredRPM > gov.motorRPMGlitchDelta || motorRPM > gov.motorRPMGlitchLimit);

    // Error cases
    gov.motorRPMError = rpmError || rpmGlitch;

    // Evaluate RPM signal quality
    if (!gov.motorRPMError && motorRPM > 0 && gov.fullHeadSpeedRatio > GOV_HS_DETECT_RATIO) {
        if (!gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = millis();
        }
    }
    else {
        if (gov.motorRPMDetectTime) {
            gov.motorRPMDetectTime = 0;
        }
    }

    // Headspeed is present if RPM is stable long enough
    gov.motorRPMPresent = (gov.motorRPMDetectTime && cmp32(millis(),gov.motorRPMDetectTime) > GOV_HS_DETECT_DELAY);

    // Battery state - zero when battery unplugged
    gov.nominalVoltage = getBatteryCellCount() * GOV_NOMINAL_CELL_VOLTAGE;

    // Voltage & current filters
    gov.motorVoltage = filterApply(&gov.motorVoltageFilter, getBatteryVoltageSample() * 0.01f);
    gov.motorCurrent = filterApply(&gov.motorCurrentFilter, getBatteryCurrentSample() * 0.01f);

    // Voltage compensation gain
    gov.vCompGain = (gov.vCompEnabled && gov.motorVoltage > 1) ? gov.nominalVoltage / gov.motorVoltage : 1;

    // Calculate request ratio (HS or throttle)
    if (gov.throttleInput > 0) {
        if (gov.govType  == GOV_TYPE_EXTERNAL)
            gov.requestRatio = gov.throttleOutput / gov.throttleInput;
        else
            gov.requestRatio = gov.currentHeadSpeed / gov.requestedHeadSpeed;
    }
    else {
        gov.requestRatio = 0;
    }

    // All precomps and feedforwards
    if (gov.precompEnabled) {
        // Calculate feedforward from collective deflection
        const float collectiveFF = gov.collectiveWeight * getCollectiveDeflectionAbs();

        // Calculate feedforward from cyclic deflection
        const float cyclicFF = gov.cyclicWeight * getCyclicDeflection();

        // Calculate feedforward from yaw deflection
        const float yawFF = gov.yawWeight * getYawDeflectionAbs();

        // Total feedforward / precomp
        float totalFF = collectiveFF + cyclicFF + yawFF;

        // Filtered FeedForward
        totalFF = filterApply(&gov.FFFilter, totalFF);

        // F-term
        gov.F = gov.K * gov.Kf * totalFF;
    }
    else {
        gov.F = 0;
    }

    // Tail Torque Assist
    if (gov.ttaEnabled) {
        float YAW = mixerGetInput(MIXER_IN_STABILIZED_YAW);
        float TTA = filterApply(&gov.TTAFilter, YAW) * getSpoolUpRatio() * gov.TTAGain;
        float headroom = 0;

        if (gov.govType  == GOV_TYPE_ELECTRIC)
            headroom = 2 * fmaxf(1.0f + gov.TTALimit - gov.fullHeadSpeedRatio, 0);
        else
            headroom = gov.TTALimit;

        gov.TTAAdd = constrainf(TTA, 0, headroom);

        DEBUG(TTA, 0, YAW * 1000);
        DEBUG(TTA, 1, TTA * 1000);
        DEBUG(TTA, 2, headroom * 1000);
        DEBUG(TTA, 3, gov.TTAAdd * 1000);

        DEBUG(TTA, 4, gov.P * 1000);
        DEBUG(TTA, 5, gov.I * 1000);
        DEBUG(TTA, 6, gov.pidSum * 1000);
        DEBUG(TTA, 7, gov.targetHeadSpeed);
    }
    else {
        gov.TTAAdd = 0;
    }

    // Normalized RPM error
    const float newError = (gov.targetHeadSpeed - gov.currentHeadSpeed) / gov.fullHeadSpeed + gov.TTAAdd;
    const float newDiff = difFilterApply(&gov.differentiator, newError);

    // Update PIDF terms
    gov.P = gov.K * gov.Kp * newError;
    gov.C = gov.K * gov.Ki * newError * pidGetDT();
    gov.D = gov.K * gov.Kd * newDiff;

}


/*
 * Throttle passthrough with rampup limits and extra stats.
 */

static void governorUpdateExternal(void)
{
    float throttleInput = fminf(gov.throttleInput, gov.maxActiveThrottle);
    float govPrev = gov.throttlePrevInput;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                govMain = govPrev = 0;
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > idleThrottle, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleStartupRate);
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (throttleInput > gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_SPOOLING_UP);
                break;

            // Follow the throttle, with a limited rampup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < idleThrottle, stay in spoolup
            //  -- Once throttle > idleThrottle and not ramping up, move to ACTIVE
            case GOV_STATE_SPOOLING_UP:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleSpoolupRate);
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (throttleInput <= gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (govMain >= throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle <20%, move to AUTO or SPOOLING_UP
            case GOV_STATE_ACTIVE:
                govPrev = slewLimit(govPrev, throttleInput, gov.throttleTrackingRate);
                govMain = govPrev + govPrev * gov.TTAAdd;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (throttleInput <= gov.maxIdleThrottle) {
                    if (gov.autoEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GOV_STATE_ZERO_THROTTLE:
                govMain = govPrev = 0;
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            case GOV_STATE_AUTOROTATION:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleTrackingRate);
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (throttleInput > gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move back to AUTO
            case GOV_STATE_AUTOROTATION_BAILOUT:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleBailoutRate);
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (throttleInput <= gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (govMain >= throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move to IDLE
            case GOV_STATE_RECOVERY:
                govMain = govPrev = slewUpLimit(govPrev, throttleInput, gov.throttleRecoveryRate);
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (throttleInput <= gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (govMain >= throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Update output variables
    gov.throttleOutput = govMain;
    gov.throttlePrevInput = govPrev;

    // Set debug
    govDebugStats();
}


/*
 * State machine for governed speed control
 */

static inline void govEnterSpoolupState(govState_e state)
{
    govChangeState(state);
    govSpoolupInit();
}

static inline void govEnterActiveState(govState_e state)
{
    govChangeState(state);
    govActiveInit();
}

static void governorUpdateState(void)
{
    float govPrev = gov.throttle;
    float govMain = 0;

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || (getBatteryCellCount() == 0 && gov.mode == GM_MODE2)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                govMain = 0;
                gov.targetHeadSpeed = 0;
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- Map throttle to motor output
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleStartupRate);
                gov.targetHeadSpeed = gov.currentHeadSpeed;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle <20%, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            //  -- If NO throttle, move to THROTTLE_OFF
            case GOV_STATE_SPOOLING_UP:
                govMain = govPrev;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > 0.95f)
                    govEnterActiveState(GOV_STATE_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedSpoolupRate);
                }
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed signal, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to AUTOROTATION or IDLE
            case GOV_STATE_ACTIVE:
                govMain = govPrev;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle) {
                    if (gov.autoEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                } else {
                    govMain = govActiveCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedTrackingRate);
                }
                break;

            // Throttle is off or low. If it is a mistake, give a chance to recover
            //  -- When throttle and *headspeed* returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GOV_STATE_ZERO_THROTTLE:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleTrackingRate);
                gov.targetHeadSpeed = gov.currentHeadSpeed;
                if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                break;

            // No headspeed signal. Ramp down throttle slowly.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GOV_STATE_LOST_HEADSPEED:
                govMain = slewLimit(govPrev, idleMap(gov.throttleInput), gov.throttleSpoolupRate);
                gov.targetHeadSpeed = gov.currentHeadSpeed;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (gov.motorRPMPresent) {
                    if (gov.throttleInput > gov.maxIdleThrottle)
                        govEnterSpoolupState(GOV_STATE_RECOVERY);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                else if (govStateTime() > gov.lostHeadspeedTimeout)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                break;

            // Throttle passthrough with rampup limit
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GOV_STATE_AUTOROTATION:
                govMain = slewUpLimit(govPrev, idleMap(gov.throttleInput), gov.throttleTrackingRate);
                gov.targetHeadSpeed = gov.currentHeadSpeed;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.maxIdleThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_AUTOROTATION_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_AUTOROTATION_BAILOUT:
                govMain = govPrev;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > gov.maxActiveThrottle * 0.99f)
                    govEnterActiveState(GOV_STATE_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedBailoutRate);
                }
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_RECOVERY:
                govMain = govPrev;
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_ZERO_THROTTLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_LOST_HEADSPEED);
                else if (gov.throttleInput < gov.maxIdleThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || govMain > gov.maxActiveThrottle * 0.99f)
                    govEnterActiveState(GOV_STATE_ACTIVE);
                else {
                    govMain = govSpoolupCalc();
                    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, gov.headSpeedRecoveryRate);
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Update output variables
    gov.throttleOutput = fminf(govMain, gov.maxActiveThrottle);

    // Set debug
    govDebugStats();
}


/*
 * Spoolup PI controller
 */

static void govSpoolUpInit(void)
{
    // Expected PID output
    float pidTarget = gov.throttle;

    // PID limits
    gov.P = constrainf(gov.P, -SPOOLUP_P_TERM_LIMIT, SPOOLUP_P_TERM_LIMIT);

    // Use gov.I to reach the target
    gov.I = pidTarget - gov.P;

    // Limited range
    gov.I = constrainf(gov.I, 0, gov.Li);
}

static float govSpoolUpControl(void)
{
    // PID limits
    gov.P = constrainf(gov.P, -SPOOLUP_P_TERM_LIMIT, SPOOLUP_P_TERM_LIMIT);
    gov.I = constrainf(gov.I, 0, gov.Li);
    gov.D = 0;
    gov.F = 0;

    // Governor PI sum
    gov.pidSum = gov.P + gov.I + gov.C;

    // Throttle value
    float output = gov.pidSum;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxSpoolupThrottle && gov.C > 0) || (output < gov.minSpoolupThrottle && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, gov.minSpoolupThrottle, gov.maxSpoolupThrottle);

    return output;
}


/*
 * Full PIDF controller
 */

static void govPIDInit(void)
{
    // Normalized battery voltage gain
    float pidGain = gov.vCompEnabled ? gov.nominalVoltage / gov.motorVoltage : 1;

    // Expected PID output
    float pidTarget = gov.throttleOutput / pidGain;

    // PID limits
    gov.P = constrainf(gov.P, -gov.Lp, gov.Lp);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Use gov.I to reach the target
    gov.I = pidTarget - (gov.P + gov.D + gov.F);

    // Limited range
    gov.I = constrainf(gov.I, 0, gov.Li);
}

static float govPIDControl(void)
{
    float output;

    // Normalized battery voltage gain
    float pidGain = gov.vCompEnabled ? gov.nominalVoltage / gov.motorVoltage : 1;

    // PID limits
    gov.P = constrainf(gov.P, -gov.Lp, gov.Lp);
    gov.I = constrainf(gov.I,       0, gov.Li);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    output = gov.pidSum * pidGain;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxActiveThrottle && gov.C > 0) || (output < gov.minActiveThrottle && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, gov.minActiveThrottle, gov.maxActiveThrottle);

    return output;
}


static inline float govCalcRate(uint16_t param, uint16_t min, uint16_t max)
{
    if (param)
        return 10 * pidGetDT() / constrain(param,min,max);
    else
        return 0;
}


//// Interface functions

void governorUpdate(void)
{
    // Governor is active
    if (gov.govType )
    {
        // Update internal data
        govActiveUpdate();

        // Run state machine
        govStateUpdate();
    }
    else
    {
        // Straight passthrough
        if (gov.throttleInputOff)
            gov.throttleOutput = 0;
        else
            gov.throttleOutput = gov.throttleInput;
    }
}

void INIT_CODE governorInitProfile(const pidProfile_t *pidProfile)
{
    if (gov.govType )
    {
        gov.K  = pidProfile->governor.gain / 100.0f;
        gov.Kp = pidProfile->governor.p_gain / 10.0f;
        gov.Ki = pidProfile->governor.i_gain / 10.0f;
        gov.Kd = pidProfile->governor.d_gain / 1000.0f;
        gov.Kf = pidProfile->governor.f_gain / 100.0f;

        gov.Lp = pidProfile->governor.p_limit / 100.0f;
        gov.Li = pidProfile->governor.i_limit / 100.0f;
        gov.Ld = pidProfile->governor.d_limit / 100.0f;
        gov.Lf = pidProfile->governor.f_limit / 100.0f;

        gov.vCompEnabled = ((pidProfile->gov.flags & GOV_FLAG_VOLT_CORR) && gov.govType > GOV_TYPE_EXTERNAL && isBatteryVoltageConfigured());
        gov.precompEnabled = (pidProfile->gov.flags & GOV_FLAG_PRECOMP);

        if (pidProfile->governor.tta_gain) {
            gov.TTAGain   = mixerRotationSign() * pidProfile->governor.tta_gain / -125.0f;
            gov.TTALimit  = pidProfile->governor.tta_limit / 100.0f;

            if (gov.govType  == GOV_TYPE_ELECTRIC)
                gov.TTAGain /= gov.K * gov.Kp;
        }
        else {
            gov.TTAGain   = 0;
            gov.TTALimit  = 0;
        }

        gov.yawWeight = pidProfile->governor.yaw_ff_weight / 100.0f;
        gov.cyclicWeight = pidProfile->governor.cyclic_ff_weight / 100.0f;
        gov.collectiveWeight = pidProfile->governor.collective_ff_weight / 100.0f;

        gov.maxActiveThrottle = pidProfile->governor.max_throttle / 100.0f;
        gov.minActiveThrottle = pidProfile->governor.min_throttle / 100.0f;

        gov.fullHeadSpeed = constrainf(pidProfile->governor.headspeed, 100, 50000);
        gov.fullHeadSpeedRatio = 1;

        gov.headSpeedSpoolupRate  = gov.throttleSpoolupRate  * gov.fullHeadSpeed;
        gov.headSpeedTrackingRate = gov.throttleTrackingRate * gov.fullHeadSpeed;
        gov.headSpeedRecoveryRate = gov.throttleRecoveryRate * gov.fullHeadSpeed;
        gov.headSpeedBailoutRate  = gov.throttleBailoutRate  * gov.fullHeadSpeed;

        gov.motorRPMGlitchDelta = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_DELTA;
        gov.motorRPMGlitchLimit = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_LIMIT;
    }
}

void INIT_CODE governorInit(const pidProfile_t *pidProfile)
{
    // Must have at least one motor
    if (getMotorCount() > 0)
    {
        gov.govType   = governorConfig()->gov_type;
        gov.state = GOV_STATE_THROTTLE_OFF;

        // Check RPM input
        if (gov.govType  > GOV_TYPE_EXTERNAL) {
            if (!isMotorFastRpmSourceActive(0)) {
                setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                setArmingDisabled(ARMING_DISABLED_RPM_SIGNAL);
                gov.govType  = GOV_TYPE_NONE;
            }
        }

        // Mode specific handler functions
        switch (gov.govType ) {
            case GOV_TYPE_EXTERNAL:
                govStateUpdate = governorUpdateExternal;
                govSpoolupInit = NULL;
                govSpoolupCalc = NULL;
                govActiveInit  = NULL;
                govActiveCalc  = NULL;
                break;
            case GOV_TYPE_ELECTRIC:
            case GOV_TYPE_NITRO:
                govStateUpdate = governorUpdateState;
                govSpoolupInit = govSpoolUpInit;
                govSpoolupCalc = govSpoolUpControl;
                govActiveInit  = govPIDInit;
                govActiveCalc  = govPIDControl;
                break;
            default:
                break;
        }

        gov.mainGearRatio = getMainGearRatio();

        gov.autoEnabled  = (governorConfig()->gov_autorotation_timeout > 0 &&
                           governorConfig()->gov_autorotation_bailout_time > 0);
        gov.autoTimeout  = governorConfig()->gov_autorotation_timeout * 100;
        gov.autoMinEntry = governorConfig()->gov_autorotation_min_entry_time * 100;

        gov.throttleStartupRate  = govCalcRate(governorConfig()->gov_startup_time, 1, 600);
        gov.throttleSpoolupRate  = govCalcRate(governorConfig()->gov_spoolup_time, 1, 600);
        gov.throttleTrackingRate = govCalcRate(governorConfig()->gov_tracking_time, 1, 100);
        gov.throttleRecoveryRate = govCalcRate(governorConfig()->gov_recovery_time, 1, 100);
        gov.throttleBailoutRate  = govCalcRate(governorConfig()->gov_autorotation_bailout_time, 1, 100);

        gov.headSpeedSpoolupRate  = gov.throttleSpoolupRate  * gov.fullHeadSpeed;
        gov.headSpeedTrackingRate = gov.throttleTrackingRate * gov.fullHeadSpeed;
        gov.headSpeedRecoveryRate = gov.throttleRecoveryRate * gov.fullHeadSpeed;
        gov.headSpeedBailoutRate  = gov.throttleBailoutRate  * gov.fullHeadSpeed;

        gov.zeroThrottleTimeout  = governorConfig()->gov_zero_throttle_timeout * 100;
        gov.lostHeadspeedTimeout = governorConfig()->gov_lost_headspeed_timeout * 100;

        gov.maxIdleThrottle = constrain(governorConfig()->gov_handover_throttle, 10, 50) / 100.0f;
        gov.minSpoolupThrottle = constrain(governorConfig()->gov_spoolup_min_throttle, 0, 50) / 100.0f;
        gov.maxSpoolupThrottle = constrain(governorConfig()->gov_spoolup_max_throttle, 50, 100) / 100.0f;

        const float diff_cutoff = governorConfig()->gov_rpm_filter ?
            constrainf(governorConfig()->gov_rpm_filter, 1, 50) : 10;

        difFilterInit(&gov.differentiator, diff_cutoff, gyro.targetRateHz);

        lowpassFilterInit(&gov.motorVoltageFilter, LPF_PT2, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.motorCurrentFilter, LPF_PT2, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.motorRPMFilter, LPF_PT2, governorConfig()->gov_rpm_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.TTAFilter, LPF_PT2, governorConfig()->gov_tta_filter, gyro.targetRateHz, 0);
        lowpassFilterInit(&gov.FFFilter, LPF_PT2, governorConfig()->gov_ff_filter, gyro.targetRateHz, 0);

        governorInitProfile(pidProfile);
    }
}
