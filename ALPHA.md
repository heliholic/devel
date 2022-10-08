
# Alpha Development version of Rotorflight-2.0 (4.3.x)

This is a snapshot of the ongoing Rotorflight 4.3 development.
It is called _ALPHA_ because lots of things are broken.

# Configuration

## Hardware Config

Should be the same as RF-1.0

All hardware related CLI commands (dump hardware) should work.

## Features


## Modes / AUX

The CLI commands are the same, but the range (PWM) resolution
has been incresed from 25 to 5us.


## Adjustment Functions

Adjustment functions have been totally rewritten, and have
very different functionality.

TBD

## Gyro Filterings

Most gyro filtering related CLI commands have been renamed in BF4.3.
There are further changes in Rotorflight.

CLI commands listed below.

```
set gyro_decimation_hz = 250
set gyro_lpf1_type = BIQUAD
set gyro_lpf1_static_hz = 150
set gyro_lpf1_dyn_min_hz = 25
set gyro_lpf1_dyn_max_hz = 200
set gyro_lpf2_type = PT1
set gyro_lpf2_static_hz = 0
set gyro_notch1_hz = 0
set gyro_notch1_cutoff = 0
set gyro_notch2_hz = 0
set gyro_notch2_cutoff = 0
set dterm_lpf1_type = PT1
set dterm_lpf1_static_hz = 15
set dterm_lpf1_dyn_min_hz = 0
set dterm_lpf1_dyn_max_hz = 0
set dterm_lpf2_type = PT1
set dterm_lpf2_static_hz = 0
set dterm_notch_hz = 0
set dterm_notch_cutoff = 0
```

## Blackbox

Blackbox has been refactored in BF4.3, and further in RF2.0.

### blackbox_mode

Refactored.

```
OFF    = No logging
NORMAL = logging when ARMED and BLACKBOX mode active
ARMED  = logging always when ARMED
ALWAYS = logging always (required reboot for storing the log)
```

### `blackbox_sample_rate = 1/N`

Blackbox sampling is now locked to the PID loop, and can be reduced
by a factor of 2,4,8,16,32,etc.


### `blackbox_log_XYZ = ON/OFF`

Each blackbox field can be now separately enabled/disabled.

```
set blackbox_log_command = ON
set blackbox_log_setpoint = ON
set blackbox_log_control = ON
set blackbox_log_pid = ON
set blackbox_log_gyro_raw = ON
set blackbox_log_gyro = ON
set blackbox_log_acc = ON
set blackbox_log_mag = OFF
set blackbox_log_baro = ON
set blackbox_log_battery = ON
set blackbox_log_rssi = ON
set blackbox_log_rpm = ON
set blackbox_log_motors = ON
set blackbox_log_servos = ON
set blackbox_log_gps = OFF
```

## Debug

Debug has now 8x 32bit values, rather than 4x 16bit values.

Some debug modes are logging only one axis, which is selected
by `debug_axis` parameter.


## Sensors

New settings for sensor update speed:

```
set vbat_update_hz = 100
set ibat_update_hz = 100
set esc_sensor_update_hz = 100
```


## Profile (PID Profile)

PID control has been totally rewritten.

There are now multiple PID controllers to choose from.

#### `pid_mode = 0`

This is a test mode with the FF term only. Useful for debugging the firmware.

#### `pid_mode = 1`

Rotorflight-1.0 compatibility mode. This is exactly the same as RF-1.0.
Parameters have different names, unfortunately.

#### `pid_mode = 2`

New PID mode under development. This is the current idea what RF-2.0 could be.

#### `pid_mode = 9`

Test PID mode with ridiculous number of parameters. Used for testing new ideas.

This mode has totally separate PID gains for yaw CW & CCW rotation.

The "other" direction gains are called "way" gains (as in yaw wrong way around...)

```
set way_p_gain = 50
set way_i_gain = 60
set way_d_gain = 10
set way_f_gain = 0
```


### PID Gains

The PID gain parameters have been renamed. Should be obvious.


#### `pitch_d_cutoff = <Hz>`

D-term bandwidth limit, separate for each axis.

Available in modes 2 and 9. Equivalent to RF10 D-term PT1 filter.
Must be set to a reasonable value for D-term to work.

#### `pitch_e_cutoff = <Hz>`

Error term bandwidth limit. 0 = disabled.

Available in modes 1, 2 and 9.

#### `pitch_f_cutoff = <Hz>`

FF term highpass filter cutoff. 0 = disabled.

New idea about how to do FF. Needs evaluation.

Available in modes 2 and 9.

#### `pitch_bandwidth = <Hz>`

Gyro signal bandwidth limit, separate for each axis. 0 = disabled.
This is an extra PT1 filter for each gyro axis.

Available in modes 2 and 9.

#### `yaw_[c]cw_stop_gain = <N>`

Stop gain for yaw. Works differently in each mode.

