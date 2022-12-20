
# TEST

1. Baro & GPS
    * Altitude calc
    * GPS altitude

2. Rescue
    * climb mode
    * alt-hold mode

3. Blackbox
    * speed selector
    * flags (combinations)

4. PID
    * modes & gains
    * gain transition
    * Yaw flags
    * filters

5. Motorised tail
    * Function
    * TTA

6. Task speed
    * Speed stability
    * RPM filter stability

7. AIRBORNE
    * Tune filters
    * Need for LPFs?



# BUGS




# NEXT

1. ATTITUDE hold
    * "Absolute Control"

2. ALTITUDE HOLD
    * Baro + GPS

3. GPS RESCUE


- Filter refactor

- Vibration analysis

- CRSF telemetry



# FIRMWARE

- PID
    * Attitude control
    * Yaw D-gain selector

- ALT-HOLD mode
    * Altitude stabilisation

- GPS RESCUE
    * throttle/collective handling

- Filter architecture
    * Refactor

- Vibration analysis
    * Main rotor & tail rotor levels

- CRSF telemetry
    * Custom sensors

- TTA
    * Bidir tail motor
    * Motor control mode

- Motor tail IDLE GOV

- Blackbox
    * More fields
    * High resolution (x10)

- Governor
    * D-term filtering
    * Consider intertia effect
    * MODE3: KV estimation & min throttle

- Voltage ADC
    * Allow Vbat = External BEC
    * Vbec measurement

- PID AGC autotune
    * Roll & Pitch

- Tail Feedforward autotune
    * cyclic => yaw
    * collective => yaw

- Collective to Pitch FF autotune
    * nodding compensation

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
    * Check function

- Consider elev filter
    * What is causing this?
    * High D-term is enough?

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

