
# TEST

1. AIRBORNE

2. PID
    * modes & gains
    * gain transition
    * Yaw flags
    * filters

3. Baro & GPS
    * Altitude fusion

4. Rescue
    * climb mode
    * alt-hold mode

5. Task speed
    * RPM filter function



# ISSUES

- GPS altitude initial convergency is slow

- Altitude filter convergency is slow



# NEXT

1. ATTITUDE hold
    * Delayed control

2. RESCUE / ALTITUDE HOLD
    * Baro + GPS

3. CRSF telemetry



# FIRMWARE

- PID
    * Attitude control
    * D-gain selector

- ALT-HOLD mode
    * Altitude stabilisation

- GPS RESCUE
    * throttle/collective handling

- CRSF telemetry
    * Custom sensors

- Filter architecture
    * Refactor

- Vibration analysis
    * Main rotor & tail rotor levels

- TTA
    * Bidir tail motor
    * Motor control mode

- Motorised tail Idle Gov

- Blackbox
    * More fields
    * High resolution (x10)

- Governor
    * Main motor idle
    * D-term filtering
    * Consider intertia effect
    * MODE3: KV estimation & min throttle

- Voltage ADC
    * Allow Vbat = External BEC
    * Vbec measurement

- Autotune
    * PID AGC autotune
    * Tail Feedforward autotune
    * Collective to Pitch FF autotune
    * Gov feedforward autotune

- MSP
    * MSP_CF_SERIAL_CONFIG
    * MSP2_COMMON_SERIAL_CONFIG

- Better trig approx
    * use splines

- Separate protocols for each motor
    * output groups

- Antigravity
    * requires acc level

- Leveling offset
    * RC adjustment for acc_trim_

- Smith Predictor
    * with EKF system ID

- Freq sensor
    * 32bit mode for T2 and T5

- SITL target
    * Get Simu tests running

- Persistent error flags
    * Too low gov speed
    * high vibrations
    * Current
    * Voltage

- Telemetry protocols
    * HobbyWing
    * Scorpion
    * Kontronik
    * Castle
    * YGE
    * MultiPlex
    * ExBUS

- FEATURE flags
    * CMS
    * VTX
    * RC device
    * Camera Control

- FIX
    * Check rcCommand[] access
    * Check rcData[] access
    * Check DEBUG_CYCLETIME in core.c
    * Use abs/fabsf instead of ABS




# CONFIGURATOR

- FIX Adjustments

- FIX blackbox

- Warnings and checks
    * Mixer settings
    * Dynamic filter
    * RPM filter
    * Very high rates

- Backup & Restore
    * using CLI

- Disable SAVE when ARMed

- Mixer
    * Refactor
    * Trailing edge control

- Reboot improvements

- Visuals
    * Blackbox hide/unhide boxes
    * Input font size
    * Input / Select disable / colours
    * Dialog style
    * General HTML & CSS improvements
    * noUi slider disabled/enabled colours

- Code improvements
    * <span class="warning">
    * <span class="error">

- Icons
    * GPS tab pointer drone
    * Landing tab drone

- Use RF config database
    * YAML file to describe the configs

- Status tab
    * PID values
    * Servo outputs
    * Motor outputs
    * Battery state

- Serial Ports tab
    * Move into Config tab

- Logging tab
    * Total rewrite

- Receiver tab
    * Channel order refactor

- Virtual FC

- SQUASH & REBASE



# BLACK BOX

- Show headspeed & tailspeed in FFT

- Use Headspeed in FFT waterfall

- Add TailSpeed

- Add motor RPMs

- Parameter report

