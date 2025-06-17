# Rotorflight Governor

The purpose of the governor is to keep the headspeed constant, regardless of the flight conditions, amount of collective applied, the battery voltage level, and so on.

It is also ensuring safe operation by limiting the spool up and down speeds, and dealing
with failure situations.


# Basic Operation


# States

The governor operates in multiple states. Each state is used for a specific purpose, and
has its own parameters.

### State `THROTTLE_OFF`

The motor is turned off.

### State `THROTTLE_IDLE`

The throttle is below _handover_, and the motor is either off or running slow.
The input throttle is passed to the output, but change speed is limited by
`gov_startup_time`.

The helicopter is expected to be on the ground.

### State `THROTTLE_HOLD`

The input throttle was switched off while in the `ACTIVE` state. The motor is spooling
down. If the throttle is resumed within `gov_throttle_hold_timeout`, a fast recovery
is performed.

This is a safeguard against an accidential throttle cut/hold activation.

### State `SPOOLUP`

The input throttle was increased above _handover_, and the motor is spooling up to
the required headspeed. The change speed is limited by `gov_spoolup_time`.

Once the target headspeed is reached, the governor moves to the `ACTIVE` state.

### State `RECOVERY`

The governor is recoving from `FALLBACK` or `THROTTLE_HOLD`, i.e. going back to `ACTIVE`.
This is done with a much faster spoolup speed than in `SPOOLUP`, as the helicopter is likely
airborne. The throttle change is limited by `gov_recovery time`.

### State `ACTIVE`

The governor is locked on the target headspeed, and the throttle is controlled by
the PID loop. Precomps from collective, cyclic, and yaw are applied.

If the target headspeed is altered, the change speed is limited by `gov_tracking_time`.

### State `FALLBACK`

RPM signal is unreliable or lost, and the headspeed can't be controlled.
Instead, a throttle value `gov_base_throttle` is applied, with optional precomps, so
that the pilot can land the helicopter safely.

### State `AUTOROTATION`

Throttle is below _handover_, and _Autorotation_ is enabled. Similar to `THROTTLE_IDLE`,
but the helicopter is expected to be airborne. If the throttle is returned, a fast
`BAILOUT` is performed.

After a succesful landing, the pilot is expected to disable _Autorotation_, and proceed
with a normal `SPOOLUP`.

### State `BAILOUT`

The autorotation attempt was abandoned, and an _autorotation bailout_ is performed.
Throttle is ramped up quickly.

### State `PASSTHRU`

The governor is temporarily disabled, and the throttle is passed to the ESC unaltered.


# Settings

The governor has both global and profile settings. Global settings are always valid.
Profile settings are applied only when a profile is active.

## Global settings

### Governor Modes

The governor mode setting (`gov_mode`) chooses what type of governor is used, if any.

#### Governor Mode `OFF`

The governor is disabled. The throttle level is passed through unmodified.

#### Governor Mode `EXTERNAL`

An external governor, or throttle curves can be used. The slow spoolup and failure handling
is still done in Rotorflight, but the headspeed stabilisation is left out.

#### Governor Mode `ELECTRIC`

A governor for electric motors is chosen. Rotorflight is taking care of the headspeed
stabilisation and soft start, among other things.

#### Governor Mode `NITRO`

A governor for I.C. / nitro motors is chosen. Rotorflight is taking care of the headspeed
stabilisation and soft start, among other things.

### Startup time `gov_startup_time`

The throttle change speed limit for `IDLE` state.

### Spoolup time `gov_spoolup_time`

The throttle change speed limit for `SPOOLUP` state.

### Tracking time `gov_tracking_time`

The throttle change speed limit for `ACTIVE` state.

### Recovery time `gov_recovery_time`

The throttle change speed limit for `RECOVERY` and `BAILOUT` states.

### Throttle Hold timeout `gov_throttle_hold_timeout`

The timeout for the `THROTTLE_HOLD` state.

### Handover Throttle `gov_handover_throttle`

The throttle level required for the governor to activate.

### Voltage Filter `gov_pwr_filter`

A cutoff frequency for the battery voltage filter.

### RPM Filter `gov_rpm_filter`

A cutoff frequency for the headspeed filter.

### TTA Filter `gov_tta_filter`

A cutoff frequency for the TTA control filter.

### Feedforward Filter `gov_ff_filter`

A cutoff frequency for the feedforward/precomp filter.

### D-term Cutoff Frequency `gov_d_cutoff`

A cutoff frequency for the D-term internal filter.


## Profile Settings

### Target headspeed `gov_headspeed`

### Idle throttle value `gov_idle_throttle`

The minimum throttle value in `IDLE` state.

### Base throttle value `gov_base_throttle`

The throttle value used in the `FALLBACK`.

### Autorotation throttle value `gov_auto_throttle`

The minimum throttle value in `AUTOROTATION` state.

### Minimum throttle while active `gov_min_throttle`

The minimum throttle value in `ACTIVE` state.

### Maximum throttle used `gov_max_throttle`

The maximum throttle value, in any state.

### Master Gain `gov_gain`

The master PID gain.

### P-Gain `gov_p_gain`

### I-Gain `gov_i_gain`

### D-Gain `gov_d_gain`

### F-gain `gov_f_gain`

Total _feedforward_ gain, i.e. a common gain for all precomps.

### P-limits `gov_p_limit`

P-term positive and negative limits (array).

### I-limit `gov_i_limit`

### D-limit `gov_d_limit`

### F-limit `gov_f_limit`

### Yaw Precomp Weight `gov_yaw_ff_weight`

Yaw precomp relative weight.

### Cyclic Precomp Weight `gov_cyclic_ff_weight`

Cyclic precomp relative weight.

### Collective Precomp Weight `gov_collective_ff_weight`

Collective precomp relative weight.

### Collective Precomp Curve `gov_collective_curve`

Collective curve selector 0..5.

### TTA-gain `gov_tta_gain`

### TTA-limit `gov_tta_limig`


## Profile Flags

### Flag `gov_use_passthrough`

This is a passthrough profile. The throttle is passed to the output unaltered.

### Flag `gov_use_dyn_min_throttle`

Use _Dynamic Minimum Throttle_ in `ACTIVE` state.

### Flag `gov_use_three_pos_switch`

The input throttle channel is a three-position switch: OFF/IDLE/ACTIVE.

### Flag `gov_use_precomp`

Enable the _precompensations_ in the governor.
The precomp level is calculated in the FC, unless `gov_use_direct_precomp` is enabled.

### Flag `gov_use_fallback_precomp`

Enable the _precompensations_ in `FALLBACK` state.

### Flag `gov_use_direct_precomp`

Use the throttle input as the precompensation level, instead of calculating it in the FC.

### Flag `gov_use_voltage_comp`

Use _battery voltage compensation_.

### Flat `gov_use_pid_spoolup`

Use a PID controlled spoolup in `SPOOLUP` state.

### Flag `gov_use_hs_on_throttle`

Headspeed target is controlled by the input throttle. The target headspeed is set to
`input_throttle * gov_headspeed`, while the throttle is above `gov_handover_throttle`.

### Flag `gov_use_autorotation`

This is an autorotation profile. Throttle cut/hold always enables autorotation.


## Mode Switches

### `AUTOROTATION`

Enable _autorotation_ temporarily.

### `RPMERROR`

Trigger an _RPM error_ temporarily, for testing the `FALLBACK` behaviour.

### `PASSTHRU`

Enable _throttle passthrough_ temporarily. Can be activated in `THROTTLE_OFF` or `THROTTLE_IDLE` states, for safety.
