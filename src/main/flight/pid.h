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

#include <stdbool.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/pwl.h"
#include "common/rtc.h"

#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16
#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

#define PID_GAIN_MAX 250
#define F_GAIN_MAX 1000

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
#define FEEDFORWARD_SCALE 0.013754f

// Anti gravity I constant
#define ANTIGRAVITY_KI 0.34f; // if AG gain is 6, about 6 times iTerm will be added
#define ANTIGRAVITY_KP 0.0034f; // one fifth of the I gain on P by default
#define ITERM_ACCELERATOR_GAIN_OFF 0
#define ITERM_ACCELERATOR_GAIN_MAX 250

#define PID_ROLL_DEFAULT  { 45, 80, 30, 120 }
#define PID_PITCH_DEFAULT { 47, 84, 34, 125 }
#define PID_YAW_DEFAULT   { 45, 80,  0, 120 }

#define DTERM_LPF1_DYN_MIN_HZ_DEFAULT 75
#define DTERM_LPF1_DYN_MAX_HZ_DEFAULT 150
#define DTERM_LPF2_HZ_DEFAULT 150

#define G_ACCELERATION 9.80665f // gravitational acceleration in m/s^2

typedef enum {
    TERM_P,
    TERM_I,
    TERM_D,
    TERM_F,
    TERM_S,
} term_e;

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
} pidSuperExpoYaw_e;

typedef enum {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
} pidStabilisationState_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

typedef enum feedforwardAveraging_e {
    FEEDFORWARD_AVERAGING_OFF,
    FEEDFORWARD_AVERAGING_2_POINT,
    FEEDFORWARD_AVERAGING_3_POINT,
    FEEDFORWARD_AVERAGING_4_POINT,
} feedforwardAveraging_t;

typedef enum {
    YAW_TYPE_RUDDER,
    YAW_TYPE_DIFF_THRUST,
} yawType_e;

#define MAX_PROFILE_NAME_LENGTH 8u

typedef struct pidProfile_s {
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_lpf1_type;                // Filter type for dterm lowpass 1
    uint8_t itermWindup;                    // iterm windup threshold, percentage of pidSumLimit within which to limit iTerm
    uint16_t pidSumLimit;                   // pidSum limit value for pitch and roll
    uint16_t pidSumLimitYaw;                // pidSum limit value for yaw
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t angle_limit;                    // Max angle in degrees in Angle mode

    uint8_t horizon_limit_degrees;          // in Horizon mode, zero levelling when the quad's attitude exceeds this angle
    uint8_t horizon_ignore_sticks;          // 0 = default, meaning both stick and attitude attenuation; 1 = only attitude attenuation

    // Betaflight PID controller parameters
    uint8_t anti_gravity_gain;              // AntiGravity Gain (was itermAcceleratorGain)
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t itermLimit;
    uint16_t dterm_lpf2_static_hz;          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system
    uint8_t dterm_lpf2_type;                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf1_dyn_min_hz;         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz;         // Dterm lowpass filter 1 max hz when in dynamic mode
    uint8_t transient_throttle_limit;       // Maximum DC component of throttle change to mix into throttle to prevent airmode mirroring noise
    char profileName[MAX_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile

    uint8_t dterm_lpf1_dyn_expo;            // set the curve for dynamic dterm lowpass filter

    uint8_t anti_gravity_cutoff_hz;
    uint8_t anti_gravity_p_gain;
    uint8_t angle_earth_ref;                // Control amount of "co-ordination" from yaw into roll while pitched forward in angle mode
    uint16_t horizon_delay_ms;              // delay when Horizon Strength increases, 50 = 500ms time constant

    uint8_t ez_landing_threshold;           // Threshold stick position below which motor output is limited
    uint8_t ez_landing_limit;               // Maximum motor output when all sticks centred and throttle zero
    uint8_t ez_landing_speed;               // Speed below which motor output is limited
    uint8_t landing_disarm_threshold;            // Accelerometer vector delta (jerk) threshold with disarms if exceeded

    uint8_t yaw_type;                   // For wings: type of yaw (rudder or differential thrust)
    int16_t angle_pitch_offset;         // For wings: pitch offset for angle modes; in decidegrees; positive values tilting the wing down
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;                   // Processing denominator for PID controller vs gyro sampling rate
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
} dtermLowpass_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef struct tpaSpeedParams_s {
    float maxSpeed;
    float dragMassRatio;
    float inversePropMaxSpeed;
    float twr;
    float speed;
    float maxVoltage;
    float pitchOffset;
} tpaSpeedParams_t;

typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpassApplyFn;
    dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
    bool antiGravityEnabled;
    pt2Filter_t antiGravityLpf;
    float antiGravityOsdCutoff;
    float antiGravityThrottleD;
    float itermAccelerator;
    uint8_t antiGravityGain;
    float antiGravityPGain;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float angleGain;
    float horizonGain;
    float horizonLimitSticks;
    float horizonLimitSticksInv;
    float horizonLimitDegrees;
    float horizonLimitDegreesInv;
    float horizonIgnoreSticks;
    float maxVelocity[XYZ_AXIS_COUNT];
    float itermLimit;
    float itermLimitYaw;
    bool itermRotation;
    bool zeroThrottleItermReset;
    bool useEzDisarm;
    float landingDisarmThreshold;

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_ACC
    pt3Filter_t attitudeFilter[RP_AXIS_COUNT];  // Only for ROLL and PITCH
    pt1Filter_t horizonSmoothingPt1;
    uint16_t horizonDelayMs;
    float angleYawSetpoint;
    float angleEarthRef;
    float angleTarget[RP_AXIS_COUNT];
    bool axisInAngleMode[3];
#endif

} pidRuntime_t;

extern pidRuntime_t pidRuntime;

extern const char pidNames[];

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern pt1Filter_t throttleLpf;

void resetPidProfile(pidProfile_t *profile);

void pidResetIterm(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void pidSetItermAccelerator(float newItermAccelerator);
void pidUpdateAntiGravityThrottleFilter(float throttle);
bool pidOsdAntiGravityActive(void);
void pidSetAntiGravityState(bool newState);
bool pidAntiGravityEnabled(void);

#ifdef UNIT_TEST
#include "sensors/acceleration.h"
extern float axisError[XYZ_AXIS_COUNT];
void rotateItermAndAxisError();
float pidLevel(int axis, const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim, float rawSetpoint, float horizonLevelStrength);
float calcHorizonLevelStrength(void);
#endif

void dynLpfDTermUpdate(float throttle);
void pidSetItermReset(bool enabled);
float pidGetPreviousSetpoint(int axis);
float pidGetDT();
float pidGetPidFrequency();

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);
