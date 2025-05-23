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

// P-term limit for spoolup
#define GOV_SPOOLUP_P_LIMIT             0.10f


//// Internal Data

typedef struct {

    // Governor type
    govMode_e       govMode;

    // Master governor enable
    bool            govEnabled;

    // Governor features
    bool            useThreePosThrottle;
    bool            useHsAdjustment;
    bool            usePidSpoolup;
    bool            useVoltageComp;
    bool            usePrecomp;
    bool            useFallbackPrecomp;
    bool            useDirectPrecomp;
    bool            autoRotationEnabled;
    bool            ttaEnabled;

    // State machine
    govState_e      state;
    timeMs_t        stateEntryTime;

    // Output throttle
    float           throttleOutput;

    // Input throttle
    float           throttleInput;
    bool            throttleInputOff;
    float           throttlePrevInput;

    // Fallback base throttle
    float           baseThrottle;

    // Idle throttle level
    float           idleThrottle;

    // Handover level
    float           handoverThrottle;

    // Startup/spoolup throttle levels
    float           minSpoolupThrottle;
    float           maxSpoolupThrottle;

    // Active throttle limits
    float           minActiveThrottle;
    float           maxActiveThrottle;

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

    // Battery voltage
    float           motorVoltage;
    filter_t        motorVoltageFilter;

    // Nominal battery voltage
    float           nominalVoltage;

    // Voltage compensation gain
    float           voltageCompGain;

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
    filter_t        precompFilter;

    // Tail Torque Assist
    float           TTAAdd;
    float           ttaGain;
    float           ttaLimit;
    filter_t        TTAFilter;

    // Autorotation
    long            autoMinEntry;
    long            autoTimeout;

    // Timeouts
    long            zeroThrottleTimeout;

    // Throttle change rates
    float           throttleStartupRate;
    float           throttleSpoolupRate;
    float           throttleRecoveryRate;
    float           throttleTrackingRate;

} govData_t;

static FAST_DATA_ZERO_INIT govData_t gov;


//// Handler functions

typedef void  (*govStateFn)(void);
typedef void  (*govInitFn)(void);
typedef float (*govCtrlFn)(float rate);

static FAST_DATA_ZERO_INIT govStateFn  govStateUpdate;

static FAST_DATA_ZERO_INIT govInitFn   govSpoolupInit;
static FAST_DATA_ZERO_INIT govCtrlFn   govSpoolupControl;


//// Prototypes

static void govVoidInit(void);
static void govPIDInit(void);

static float govPIDControl(float rate);
static float govFallbackControl(float rate);
static float govThrottleSpoolupControl(float rate);

static void governorUpdateExternalState(void);
static void governorUpdateElectricState(void);


//// Access functions

bool getGovernerEnabled(void)
{
    return gov.govEnabled;
}

void setGovernorEnabled(bool enabled)
{
    gov.govEnabled = enabled && gov.govMode;
}

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
    if (gov.govMode > GOV_MODE_EXTERNAL) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLUP:
            case GOV_STATE_THROTTLE_LOW:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
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

    if (gov.govMode ) {
        switch (gov.state)
        {
            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_LOW:
            case GOV_STATE_THROTTLE_IDLE:
            case GOV_STATE_AUTOROTATION:
                return 0;

            case GOV_STATE_ACTIVE:
            case GOV_STATE_RECOVERY:
            case GOV_STATE_FALLBACK:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_DIRECT:
                return 1.0f;

            case GOV_STATE_SPOOLUP:
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

    if (gov.govMode ) {
        switch (gov.state)
        {
            case GOV_STATE_ACTIVE:
            case GOV_STATE_AUTOROTATION:
            case GOV_STATE_BAILOUT:
            case GOV_STATE_DIRECT:
                return true;

            case GOV_STATE_RECOVERY:
            case GOV_STATE_SPOOLUP:
            case GOV_STATE_FALLBACK:
                return (gov.throttleOutput > 0.333f);

            case GOV_STATE_THROTTLE_OFF:
            case GOV_STATE_THROTTLE_LOW:
            case GOV_STATE_THROTTLE_IDLE:
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

static inline void govChangeState(govState_e futureState)
{
    if (gov.state != futureState) {
        gov.state = futureState;
        gov.stateEntryTime = millis();
    }
}

static inline long govStateTime(void)
{
    return cmp32(millis(),gov.stateEntryTime);
}

static void govGetInputThrottle(void)
{
    bool throloff = (getThrottleStatus() == THROTTLE_LOW);
    float throttle = getThrottle();

    if (gov.useThreePosThrottle) {
        if (throttle < 0.333f) {
            throttle = 0;
            throloff = true;
        }
        else if (throttle < 0.666f) {
            throttle = gov.idleThrottle;
        }
        else {
            throttle = 1.0f;
        }
    }

    gov.throttleInput = throttle;
    gov.throttleInputOff = throloff;
}

static void govDataUpdate(void)
{
    // Calculate effective throttle
    govGetInputThrottle();

    // Assume motor[0]
    const float motorRPM = getMotorRawRPMf(0);

    // RPM signal is noisy - filtering is required
    const float filteredRPM = filterApply(&gov.motorRPMFilter, motorRPM);

    // Calculate headspeed from filtered motor speed
    gov.currentHeadSpeed = filteredRPM * gov.mainGearRatio;

    // Calculate current HS vs FullHS ratio
    gov.fullHeadSpeedRatio = gov.currentHeadSpeed / gov.fullHeadSpeed;

    // Update headspeed target
    if (gov.useHsAdjustment)
        gov.requestedHeadSpeed = gov.throttleInput * gov.fullHeadSpeed;

    // Detect stuck motor / startup problem
    const bool rpmError = ((gov.fullHeadSpeedRatio < GOV_HS_INVALID_RATIO || motorRPM < 10) && gov.throttleOutput > GOV_HS_INVALID_THROTTLE);

    // Detect RPM glitches
    const bool rpmGlitch = (fabsf(motorRPM - filteredRPM) > gov.motorRPMGlitchDelta || motorRPM > gov.motorRPMGlitchLimit);

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

    // Filtered voltage
    gov.motorVoltage = filterApply(&gov.motorVoltageFilter, getBatteryVoltageSample() * 0.01f);

    // Voltage compensation gain
    gov.voltageCompGain = (gov.useVoltageComp && gov.motorVoltage > 1) ? gov.nominalVoltage / gov.motorVoltage : 1;

    // Calculate request ratio (HS or throttle)
    if (gov.throttleInput > 0) {
        if (gov.govMode == GOV_MODE_EXTERNAL)
            gov.requestRatio = gov.throttleOutput / gov.throttleInput;
        else
            gov.requestRatio = gov.currentHeadSpeed / gov.requestedHeadSpeed;
    }
    else {
        gov.requestRatio = 0;
    }

    // All precomps and feedforwards
    if (gov.usePrecomp) {
        if (gov.useDirectPrecomp) {
            // Use throttle input directly as F-term
            gov.F = gov.throttleInput;
        }
        else {
            // Calculate feedforward from collective deflection
            const float collectiveFF = gov.collectiveWeight * getCollectiveDeflectionAbs();

            // Calculate feedforward from cyclic deflection
            const float cyclicFF = gov.cyclicWeight * getCyclicDeflection();

            // Calculate feedforward from yaw deflection
            const float yawFF = gov.yawWeight * getYawDeflectionAbs();

            // Total feedforward / precomp
            float totalFF = collectiveFF + cyclicFF + yawFF;

            // Filtered FeedForward
            totalFF = filterApply(&gov.precompFilter, totalFF);

            // F-term
            gov.F = gov.K * gov.Kf * totalFF;
        }
    }
    else {
        gov.F = 0;
    }

    // Tail Torque Assist
    if (gov.ttaEnabled) {
        float YAW = mixerGetInput(MIXER_IN_STABILIZED_YAW);
        float TTA = filterApply(&gov.TTAFilter, YAW) * getSpoolUpRatio() * gov.ttaGain;
        float headroom = 0;

        if (gov.govMode  == GOV_MODE_ELECTRIC)
            headroom = 2 * fmaxf(1.0f + gov.ttaLimit - gov.fullHeadSpeedRatio, 0);
        else
            headroom = gov.ttaLimit;

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
 * Void init function
 */

static void govVoidInit(void) { }


/*
 * Motor start up controller
 */

 static float govStartupControl(float rate)
{
    // Update headspeed target
    gov.targetHeadSpeed = gov.currentHeadSpeed;

    // Throttle value
    float output = slewUpLimit(gov.throttleOutput, gov.throttleInput, rate);

    // Limit output
    output = constrainf(output, gov.idleThrottle, gov.handoverThrottle);

    return output;
}


/*
 * Throttle ramp up controller
 */

static float govThrottleSpoolupControl(float rate)
{
    // Update headspeed target
    gov.targetHeadSpeed = gov.currentHeadSpeed;

    // Throttle value
    float output = slewUpLimit(gov.throttleOutput, gov.throttleInput, rate);

    // Limit output
    output = constrainf(output, gov.minSpoolupThrottle, gov.maxSpoolupThrottle);

    return output;
}


/*
 * Headspeed PI ramp up controller
 */

static void govHeadspeedSpoolUpInit(void)
{
    // Expected PID output
    float pidTarget = gov.throttleOutput;

    // PID limits
    gov.P = constrainf(gov.P, -GOV_SPOOLUP_P_LIMIT, GOV_SPOOLUP_P_LIMIT);

    // Use gov.I to reach the target
    gov.I = pidTarget - gov.P;

    // Limited range
    gov.I = constrainf(gov.I, 0, gov.Li);
}

static float govHeadspeedSpoolUpControl(float rate)
{
    // Update headspeed target
    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, rate * gov.fullHeadSpeed);

    // PID limits
    gov.P = constrainf(gov.P, -GOV_SPOOLUP_P_LIMIT, GOV_SPOOLUP_P_LIMIT);
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
    // Expected PID output
    float pidTarget = gov.throttleOutput / gov.voltageCompGain;

    // PID limits
    gov.P = constrainf(gov.P, -gov.Lp, gov.Lp);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Use gov.I to reach the target
    gov.I = pidTarget - (gov.P + gov.D + gov.F);

    // Limited range
    gov.I = constrainf(gov.I, 0, gov.Li);
}

static float govPIDControl(float rate)
{
    // Update headspeed target
    gov.targetHeadSpeed = slewLimit(gov.targetHeadSpeed, gov.requestedHeadSpeed, rate * gov.fullHeadSpeed);

    // PID limits
    gov.P = constrainf(gov.P, -gov.Lp, gov.Lp);
    gov.I = constrainf(gov.I,       0, gov.Li);
    gov.D = constrainf(gov.D, -gov.Ld, gov.Ld);
    gov.F = constrainf(gov.F,       0, gov.Lf);

    // Governor PIDF sum
    gov.pidSum = gov.P + gov.I + gov.C + gov.D + gov.F;

    // Generate throttle signal
    float output = gov.pidSum * gov.voltageCompGain;

    // Apply gov.C if output not saturated
    if (!((output > gov.maxActiveThrottle && gov.C > 0) || (output < gov.minActiveThrottle && gov.C < 0)))
        gov.I += gov.C;

    // Limit output
    output = constrainf(output, gov.minActiveThrottle, gov.maxActiveThrottle);

    return output;
}


/*
 * Fallback controller - Base throttle + precomp
 */

static float govFallbackControl(float __unused rate)
{
    // Precomp limits
    gov.F = constrainf(gov.F, 0, gov.Lf);

    // Governor "PID sum"
    gov.pidSum = gov.baseThrottle + (gov.useFallbackPrecomp ? gov.F : 0);

    // Generate throttle signal
    float output = gov.pidSum * gov.voltageCompGain;

    // Limit output
    output = constrainf(output, gov.minActiveThrottle, gov.maxActiveThrottle);

    return output;
}


/*
 * External throttle control (governor)
 */

static void governorUpdateExternalThrottle(void)
{
    float throttle = 0;

    switch (gov.state)
    {
        case GOV_STATE_THROTTLE_OFF:
        case GOV_STATE_THROTTLE_LOW:
            throttle = 0;
            break;
        case GOV_STATE_THROTTLE_IDLE:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleStartupRate);
            break;
        case GOV_STATE_SPOOLUP:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
        case GOV_STATE_FALLBACK:
        case GOV_STATE_AUTOROTATION:
            throttle = slewLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            throttle = slewUpLimit(gov.throttlePrevInput, gov.throttleInput, gov.throttleRecoveryRate);
            break;
        case GOV_STATE_DIRECT:
            throttle = gov.throttleInput;
            break;
        default:
            break;
    }

    gov.throttlePrevInput = throttle;

    if (gov.state == GOV_STATE_ACTIVE && gov.ttaEnabled) {
        throttle += throttle * gov.TTAAdd;
    }

    gov.throttleOutput = throttle;
}

static void governorUpdateExternalState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        // Handle governor master ON/OFF here
        if (!gov.govEnabled) {
            govChangeState(GOV_STATE_DIRECT);
        }

        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleOutput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_SPOOLUP);
                break;

            // Follow the throttle, with a limited ramp up rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < handover, stay in spoolup
            //  -- Once throttle > handover and not ramping up, move to ACTIVE
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle < handover, move to AUTO or SPOOLING_UP
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (gov.autoRotationEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GOV_STATE_THROTTLE_LOW:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move to IDLE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput > gov.handoverThrottle)
                    govChangeState(GOV_STATE_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle < handover, move back to AUTO
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.throttleOutput >= gov.throttleInput)
                    govChangeState(GOV_STATE_ACTIVE);
                break;

            // Governor disabled: Direct throttle to output
            //  -- If enabled, move back to approriate state
            case GOV_STATE_DIRECT:
                if (gov.govEnabled) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else if (gov.throttleInput > gov.handoverThrottle)
                        govChangeState(GOV_STATE_SPOOLUP);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Get throttle
    governorUpdateExternalThrottle();

    // Set debug
    govDebugStats();
}


/*
 * Electric motor speed control
 */

static inline void govEnterSpoolupState(govState_e state)
{
    govChangeState(state);
    govSpoolupInit();
}

static inline void govEnterActiveState(void)
{
    govChangeState(GOV_STATE_ACTIVE);
    govPIDInit();
}

static inline void govEnterFallbackState(void)
{
    govChangeState(GOV_STATE_FALLBACK);
}

static void governorUpdateElectricThrottle(void)
{
    float throttle = 0;

    switch (gov.state)
    {
        case GOV_STATE_THROTTLE_OFF:
        case GOV_STATE_THROTTLE_LOW:
            throttle = 0;
            break;
        case GOV_STATE_THROTTLE_IDLE:
            throttle = govStartupControl(gov.throttleStartupRate);
            break;
        case GOV_STATE_SPOOLUP:
            throttle = govSpoolupControl(gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
            throttle = govPIDControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_FALLBACK:
            throttle = govFallbackControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_AUTOROTATION:
            throttle = govThrottleSpoolupControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            throttle = govSpoolupControl(gov.throttleRecoveryRate);
            break;
        case GOV_STATE_DIRECT:
            throttle = gov.throttleInput;
            break;
        default:
            break;
    }

    gov.throttleOutput = throttle;
}

static void governorUpdateElectricState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        // Handle governor master ON/OFF here
        if (!gov.govEnabled) {
            govChangeState(GOV_STATE_DIRECT);
        }

        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_SPOOLUP);
                break;

            // Ramp up throttle until headspeed target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle < handover, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            //  -- If NO throttle, move to THROTTLE_OFF
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.95f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed signal, move to FALLBACK
            //  -- If throttle < handover, move to AUTOROTATION or IDLE
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (gov.autoRotationEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Fallback: No headspeed signal. Use curves.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GOV_STATE_FALLBACK:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_RECOVERY);
                break;

            // Throttle is off or low. If it is a mistake, give a chance to recover
            //  -- When throttle and *headspeed* returns, move to RECOVERY
            //  -- When timer expires, move to IDLE
            case GOV_STATE_THROTTLE_LOW:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to FALLBACK
            //  -- If throttle < handover, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.95f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Throttle passthrough with ramp up limit
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to FALLBACK
            //  -- If throttle < handover, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.95f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Governor disabled: Direct throttle to output
            //  -- If enabled, move back to approriate state
            case GOV_STATE_DIRECT:
                if (gov.govEnabled) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                        govEnterSpoolupState(GOV_STATE_SPOOLUP);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Get throttle
    governorUpdateElectricThrottle();

    // Set debug
    govDebugStats();
}


/*
 * Nitro/I.C. speed control
 */

static void governorUpdateNitroThrottle(void)
{
    float throttle = 0;

    switch (gov.state)
    {
        case GOV_STATE_THROTTLE_OFF:
        case GOV_STATE_THROTTLE_LOW:
            throttle = 0;
            break;
        case GOV_STATE_THROTTLE_IDLE:
            throttle = govThrottleSpoolupControl(gov.throttleStartupRate);
            break;
        case GOV_STATE_SPOOLUP:
            throttle = govThrottleSpoolupControl(gov.throttleSpoolupRate);
            break;
        case GOV_STATE_ACTIVE:
            throttle = govPIDControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_FALLBACK:
            throttle = govFallbackControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_AUTOROTATION:
            throttle = govThrottleSpoolupControl(gov.throttleTrackingRate);
            break;
        case GOV_STATE_RECOVERY:
        case GOV_STATE_BAILOUT:
            throttle = govThrottleSpoolupControl(gov.throttleRecoveryRate);
            break;
        case GOV_STATE_DIRECT:
            throttle = gov.throttleInput;
            break;
        default:
            break;
    }

    gov.throttleOutput = throttle;
}

static void governorUpdateNitroState(void)
{
    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GOV_STATE_THROTTLE_OFF);
    }
    else {
        // Handle governor master ON/OFF here
        if (!gov.govEnabled) {
            govChangeState(GOV_STATE_DIRECT);
        }

        switch (gov.state)
        {
            // Throttle is OFF
            case GOV_STATE_THROTTLE_OFF:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited startup rate
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > handover, move to SPOOLUP
            case GOV_STATE_THROTTLE_IDLE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_SPOOLUP);
                break;

            // Ramp up throttle until headspeed target is reached
            //  -- Once 99% headspeed reached, move to ACTIVE
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE
            //  -- If throttle < handover, move back to IDLE
            //  -- If no headspeed detected, move to IDLE
            //  -- If NO throttle, move to THROTTLE_OFF
            case GOV_STATE_SPOOLUP:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxSpoolupThrottle * 0.95f)
                    govEnterActiveState();
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed signal, move to FALLBACK
            //  -- If throttle < handover, move to AUTOROTATION or IDLE
            case GOV_STATE_ACTIVE:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.throttleInput < gov.handoverThrottle) {
                    if (gov.autoRotationEnabled && govStateTime() > gov.autoMinEntry)
                        govChangeState(GOV_STATE_AUTOROTATION);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Fallback: No headspeed signal. Use curves.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY or IDLE
            //  -- When timer expires, move to OFF
            case GOV_STATE_FALLBACK:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMPresent) {
                    if (gov.throttleInput > gov.handoverThrottle)
                        govEnterSpoolupState(GOV_STATE_RECOVERY);
                }
                break;

            // Throttle is off or low. If it is a mistake, give a chance to recover
            //  -- When throttle and *headspeed* returns, move to RECOVERY
            //  -- When timer expires, move to IDLE
            case GOV_STATE_THROTTLE_LOW:
                if (!gov.throttleInputOff)
                    govChangeState(GOV_STATE_RECOVERY);
                else if (govStateTime() > gov.zeroThrottleTimeout) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to FALLBACK
            //  -- If throttle < handover, move to IDLE
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_RECOVERY:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxActiveThrottle * 0.99f)
                    govEnterActiveState();
                break;

            // Throttle passthrough with ramp up limit
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If throttle > handover, move to BAILOUT
            //  -- If timer expires, move to IDLE
            //  -- Map throttle to motor output
            case GOV_STATE_AUTOROTATION:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_OFF);
                else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                    govEnterSpoolupState(GOV_STATE_BAILOUT);
                else if (govStateTime() > gov.autoTimeout)
                    govChangeState(GOV_STATE_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) ramp up.
            //  -- If NO throttle, move to ZERO_THROTTLE
            //  -- If no headspeed detected, move to FALLBACK
            //  -- If throttle < handover, move back to AUTOROTATION
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 99% headspeed reached, move to ACTIVE
            case GOV_STATE_BAILOUT:
                if (gov.throttleInputOff)
                    govChangeState(GOV_STATE_THROTTLE_LOW);
                else if (gov.throttleInput < gov.handoverThrottle)
                    govChangeState(GOV_STATE_AUTOROTATION);
                else if (gov.motorRPMError)
                    govEnterFallbackState();
                else if (gov.currentHeadSpeed > gov.requestedHeadSpeed * 0.99f || gov.throttleOutput > gov.maxActiveThrottle * 0.99f)
                    govEnterActiveState();
                break;

            // Governor disabled: Direct throttle to output
            //  -- If enabled, move back to approriate state
            case GOV_STATE_DIRECT:
                if (gov.govEnabled) {
                    if (gov.throttleInputOff)
                        govChangeState(GOV_STATE_THROTTLE_OFF);
                    else if (gov.throttleInput > gov.handoverThrottle && gov.motorRPMPresent)
                        govEnterSpoolupState(GOV_STATE_SPOOLUP);
                    else
                        govChangeState(GOV_STATE_THROTTLE_IDLE);
                }
                break;

            // Should not be here
            default:
                govChangeState(GOV_STATE_THROTTLE_OFF);
                break;
        }
    }

    // Get throttle
    governorUpdateNitroThrottle();

    // Set debug
    govDebugStats();
}


//// Interface functions

void governorUpdate(void)
{
    // Governor is active
    if (gov.govMode)
    {
        // Update internal data
        govDataUpdate();

        // Run state machine
        govStateUpdate();
    }
    else
    {
        // Straight passthrough
        gov.throttleOutput = gov.throttleInput;
    }
}


void INIT_CODE governorInitProfile(const pidProfile_t *pidProfile)
{
    if (gov.govMode)
    {
        gov.useThreePosThrottle = (pidProfile->governor.flags & GOV_FLAG_3POS_THROTTLE);
        gov.useHsAdjustment = (pidProfile->governor.flags & GOV_FLAG_HS_ON_THROTTLE) && !gov.useThreePosThrottle;
        gov.usePidSpoolup = (pidProfile->governor.flags & GOV_FLAG_PID_SPOOLUP) && (gov.govMode == GOV_MODE_ELECTRIC);
        gov.useVoltageComp = (pidProfile->governor.flags & GOV_FLAG_VOLTAGE_COMP) && (gov.govMode == GOV_MODE_ELECTRIC) && isBatteryVoltageConfigured();
        gov.usePrecomp = (pidProfile->governor.flags & GOV_FLAG_PRECOMP);
        gov.useFallbackPrecomp = (pidProfile->governor.flags & GOV_FLAG_FALLBACK_PRECOMP) && gov.usePrecomp;
        gov.useDirectPrecomp = (pidProfile->governor.flags & GOV_FLAG_DIRECT_PRECOMP) && gov.usePrecomp;

        gov.K  = pidProfile->governor.gain / 100.0f;
        gov.Kp = pidProfile->governor.p_gain / 10.0f;
        gov.Ki = pidProfile->governor.i_gain / 10.0f;
        gov.Kd = pidProfile->governor.d_gain / 1000.0f;
        gov.Kf = pidProfile->governor.f_gain / 100.0f;

        gov.Lp = pidProfile->governor.p_limit / 100.0f;
        gov.Li = pidProfile->governor.i_limit / 100.0f;
        gov.Ld = pidProfile->governor.d_limit / 100.0f;
        gov.Lf = pidProfile->governor.f_limit / 100.0f;

        gov.idleThrottle = pidProfile->governor.idle_throttle / 100.0f;
        gov.baseThrottle = pidProfile->governor.base_throttle / 100.0f;

        gov.minActiveThrottle = pidProfile->governor.min_throttle / 100.0f;
        gov.maxActiveThrottle = pidProfile->governor.max_throttle / 100.0f;

        gov.minSpoolupThrottle = gov.idleThrottle;
        gov.maxSpoolupThrottle = gov.maxActiveThrottle;

        if (gov.usePidSpoolup) {
            govSpoolupInit = govHeadspeedSpoolUpInit;
            govSpoolupControl = govHeadspeedSpoolUpControl;
        }
        else {
            govSpoolupInit = govVoidInit;
            govSpoolupControl = govThrottleSpoolupControl;
        }

        if (pidProfile->governor.tta_gain) {
            gov.ttaGain = mixerRotationSign() * pidProfile->governor.tta_gain / -125.0f;
            gov.ttaLimit = pidProfile->governor.tta_limit / 100.0f;
            gov.ttaEnabled = true;

            if (gov.govMode == GOV_MODE_ELECTRIC)
                gov.ttaGain /= gov.K * gov.Kp;
        }
        else {
            gov.ttaGain = 0;
            gov.ttaLimit = 0;
            gov.ttaEnabled = false;
        }

        gov.yawWeight = pidProfile->governor.yaw_weight / 100.0f;
        gov.cyclicWeight = pidProfile->governor.cyclic_weight / 100.0f;
        gov.collectiveWeight = pidProfile->governor.collective_weight / 100.0f;

        gov.fullHeadSpeed = constrainf(pidProfile->governor.headspeed, 100, 50000);
        gov.fullHeadSpeedRatio = 1;

        gov.requestedHeadSpeed = gov.fullHeadSpeed;

        gov.motorRPMGlitchDelta = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_DELTA;
        gov.motorRPMGlitchLimit = (gov.fullHeadSpeed / gov.mainGearRatio) * GOV_HS_GLITCH_LIMIT;
    }
}

static inline float govCalcRate(uint16_t param, uint16_t min, uint16_t max)
{
    if (param)
        return 10 * pidGetDT() / constrain(param,min,max);
    else
        return 0;
}

void INIT_CODE governorInit(const pidProfile_t *pidProfile)
{
    // Must have at least one motor
    if (getMotorCount() > 0)
    {
        gov.state = GOV_STATE_THROTTLE_OFF;

        gov.govMode = governorConfig()->gov_mode;

        if (gov.govMode > GOV_MODE_EXTERNAL) {
            if (!isMotorFastRpmSourceActive(0)) {
                setArmingDisabled(ARMING_DISABLED_GOVERNOR);
                setArmingDisabled(ARMING_DISABLED_RPM_SIGNAL);
                gov.govMode = GOV_MODE_NONE;
            }
        }

        switch (gov.govMode)
        {
            case GOV_MODE_EXTERNAL:
                govStateUpdate = governorUpdateExternalState;
                break;
            case GOV_MODE_ELECTRIC:
                govStateUpdate = governorUpdateElectricState;
                break;
            case GOV_MODE_NITRO:
                govStateUpdate = governorUpdateNitroState;
                break;
            default:
                break;
        }

        if (gov.govMode)
        {
            gov.govEnabled = true;

            gov.mainGearRatio = getMainGearRatio();

            gov.autoRotationEnabled  = governorConfig()->gov_autorotation_timeout > 0;
            gov.autoTimeout  = governorConfig()->gov_autorotation_timeout * 100;
            gov.autoMinEntry = governorConfig()->gov_autorotation_min_entry_time * 100;

            gov.throttleStartupRate  = govCalcRate(governorConfig()->gov_startup_time, 1, 600);
            gov.throttleSpoolupRate  = govCalcRate(governorConfig()->gov_spoolup_time, 1, 600);
            gov.throttleTrackingRate = govCalcRate(governorConfig()->gov_tracking_time, 1, 100);
            gov.throttleRecoveryRate = govCalcRate(governorConfig()->gov_recovery_time, 1, 100);

            gov.zeroThrottleTimeout  = governorConfig()->gov_zero_throttle_timeout * 100;

            gov.handoverThrottle = constrain(governorConfig()->gov_handover_throttle, 1, 100) / 100.0f;

            const float diff_cutoff = governorConfig()->gov_rpm_filter ?
                constrainf(governorConfig()->gov_rpm_filter, 1, 50) : 10;

            difFilterInit(&gov.differentiator, diff_cutoff, gyro.targetRateHz);

            lowpassFilterInit(&gov.motorVoltageFilter, LPF_PT2, governorConfig()->gov_pwr_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.motorRPMFilter, LPF_PT2, governorConfig()->gov_rpm_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.TTAFilter, LPF_PT2, governorConfig()->gov_tta_filter, gyro.targetRateHz, 0);
            lowpassFilterInit(&gov.precompFilter, LPF_PT2, governorConfig()->gov_ff_filter, gyro.targetRateHz, 0);

            governorInitProfile(pidProfile);
        }
    }
}
