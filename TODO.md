
# TEST

1. Baro & GPS
    o Altitude calc
    o GPS altitude

2. Rescue
    o climb mode
    o alt-hold mode

3. Blackbox
    o speed selector
    o flags (combinations)

4. PID 
    o modes & gains
    o Yaw flags
    o filters

5. Motorised tail
    o Function
    o TTA

6. Task speed
    o Speed stability
    o RPM filter stability




# BUGS





# NEXT

1. ATTITUDE hold
    o "Absolute Control"

2. Adjustment functions
    o New RESCUE
    o ALT-HOLD


- Blackbox review

- Vibration analysis

- Motor mode

- CRSF telemetry

- Filter stack
    o Better filters
    o Bessel / damped filter
    o Filter task speed



# FIRMWARE

- FIX
    o Check rcCommand[] access
    o Check rcData[] access
    o Check DEBUG_CYCLETIME in core.c
    o Use abs/fabsf instead of ABS

- FEATURE flags
    o CMS
    o VTX
    o RC device
    o Camera Control

- PID
    o Attitude control

- ALTHOLD mode
    o Altitude stabilisation

- GPS RESCUE
    o throttle/collective handling

- Vibration analysis
    o Main rotor & tail rotor levels

- TTA for bidir tail motor

- Motor tail IDLE GOV

- Yaw Stop Gain transition
    o Right range?

- Filter architecture
    o Rewrite as SOS stack

- Blackbox
    o More fields
    o Check names
    o Higher resolution (x10)

- CRSF telemetry
    o Custom sensors

- Voltage ADC
    o BEC & others

- PID AGC autotune
    o Roll & Pitch

- Tail Feedforward autotune
    o cyclic => yaw
    o collective => yaw

- Collective to Pitch FF autotune
    o nodding compensation

- Governor
    o D-term filtering
    o Consider intertia effect
    o MODE3: KV estimation & min throttle

- MSP
    o MSP_CF_SERIAL_CONFIG
    o MSP2_COMMON_SERIAL_CONFIG
    o motor_mode

- Persistent error flags
    o Too low gov speed
    o high vibrations
    o Current
    o Voltage

- Better trig approx
    o use splines

- Separate protocols for each motor
    o output groups

- Antigravity
    o requires acc level

- Leveling offset
    o RC adjustment for acc_trim_
    o Check function

- Voltage ADC
    o Allow Vbat = External BEC
    o Vbec measurement

- Detect IN-THE-AIR
    o acc & gyro limits
    o use I-term decay

- GPS auto flight
    o GPS hold
    o GPS RTH

- Consider elev filter
    o What is causing this?
    o High D-term is enough?

- Smith Predictor
    o with EKF system ID

- Freq sensor
    o 32bit mode for T2 and T5

- Telemetry protocols
    x HobbyWing
    o Scorpion
    x Kontronik
    o Castle
    o YGE
    o MultiPlex
    o ExBUS

- Fix SITL target
    o Get Simu tests running




# CONFIGURATOR

- FIX Adjustments

- FIX blackbox

- Warnings and checks
    o Mixer settings
    o Dynamic filter
    o RPM filter
    o Very high rates

- Backup & Restore
    o using CLI

- Disable SAVE when ARMed

- Mixer
    o Refactor
    o Trailing edge control

- Reboot improvements

- Visuals
    o Blackbox hide/unhide boxes
    o Input font size
    o Input / Select disable / colours
    o Dialog style
    o General HTML & CSS improvements
    o noUi slider disabled/enabled colours

- Code improvements
    o <span class="warning">
    o <span class="error">

- Icons
    o GPS tab pointer drone
    o Landing tab drone

- Use RF config database
    o YAML file to describe the configs

- Status tab
    o PID values
    o Servo outputs
    o Motor outputs
    o Battery state

- Serial Ports tab
    o Move into Config tab

- Logging tab
    o Total rewrite

- Receiver tab
    o Channel order refactor

- Virtual FC

- SQUASH & REBASE




# BLACK BOX

- Show headspeed & tailspeed in FFT

- Use Headspeed in FFT waterfall

- Add TailSpeed

- Add motor RPMs

- Parameter report




---

# DONE

- CF: Armed config changes warning

- CF: Receiver config changes need reboot

- CF: Remove virtual FC

- CF: Warning about headspeed change

- CF: Receiver fallback Auto/Hold/Set translations

- BB: Datetime

- CF: Locales

- CF: Fix Analytics

- CF: compatiblity version check

- CF: Changelog update

- FW: Profile select adjustment range 1-6

- CF: Remove Logging tab

- CF: Add I-term limits to Profiles

- CF: Missing Gov parameters

- CF: BARE warning dialog

- CF: Servo config warnings

- CF: Add Servo Freq

- FW: Add Servo Freq

- CF: MSP SENSOR fixes

- FW: MSP SENSOR fixes

- CF: Disable RTH

- FW: CPU load check < 75%

- CF: Dirty profile change

- CF: Help messages & visuals

- BB: Window name shows logfile

- CF: Localization cleanup

- CF: Rates defaults

- CF: Null mixer detection

- CF: Hide expert mode for now

- CF: Time values to decimals

- CF: Override OFF send OFF

- CF: RPM filters

- CF: Custom mixing warning in Mixer

- CF: TTA to Yaw settings

- CF: Rescue delay in 0.1s

- CF: Flashing changes next tab to Status

- CF: Reboot after Gyro changes

- CF: Reboot after Mixer changes

- FW: Tail motor direction change

- CF: CLI tab exit / save

- CF: Refactor dead code

- CF: Use deg in mixer tab

- CF: Add warning dialog on dirty exit

- FW: Add firmware suffix in MSP

- BB: Log zoom

- CF: Mixer tab

- BB: Squash and rebase

- CF: Check welcome page links

- CF: Firmware flashing

- FW: semver versioning

- BB: Installer dmg name

- FW: Remove rateProfile6PosSwitch

- CF: Profiles tab heli parameters

- CF: MSP cleanup

- FW: MSP cleanup

- FW: CW & CCW gains for Yaw

- FW: DEBUG_YAW_PRECOMP

- FW: Move GOV parameters to PID profile

- CF: Config status

- CF: Dark colours

- FW: PID debug axis

- FW: FIX HID CDC

- FW: Split some params to R/P/Y

- FW: Rebase on 4.2.10

- FW: Remove PID_AUDIO?

- FW: Headspeed normalization without RPM

- FW: Docker build

- FW: Fix FF ok is smoothing OFF

- FW: MSC EMFAT files

- FW: PDK's rescue

- FW: Collective normalization

- BB: Adjustment Events

- CF: Motor info & override

- FW: TTA - Gov assisted tail

- CF: Servo overlord

- FW: Use servo speed directly from the config

- CF: Allow arming checkbox

- FW: Swashring

- Adjustments tab
    o new functionality
    o betterments

- Parameter adjustment
    o Refactor
    o Add new heli params

- Configurator Start-page links
    o Wiki
    o Help
    o Forum
    o Discussion

- Fix motors tab

- Rescue Mode
    o autolevel
    o collective punch

- Motorized tail
    o thrust linearization
    o PID normalization
    o idle offset

- Heli parameters in profile

- ff_boost setting
    o ifdef USE_INTERPOLATED_SP

- Motored tail
    o check old code

- Extra P filtering
    o Use Bessel for flat delay

- Bessel filter
    o for RC smoothing

- throttle smoothing
    o off when governor active

- PASSTHRU
    o implement RP passthrough
    o for flybar helis

- PID normalisation
    o headspeed

- yaw_lpf
    o Remove

- Saturation
    o back-propagation
    o latching

- DYN_LPF + DTERM_LPF
    o Check working

- Fix I-term rotation
    o only for RP

- Clean up pid.c
    o leveling modes to a file

- Check PID profile parameters
    o re-init heli settings

- Split heli stabilization into multiple commits
    o PID scaling
    o Tail feedforward
    o FF type change
    o Absolute Control enabler

- pid.c
    o Move D-term filtering to sensors/gyro.c

- DYN_LPF
    o throttle input -> headspeed input

- Remove feedforward transition

- Governor PID term limits

- Governor oscillations

- Governor transitions

- Governor output when PID control engaged
    o must be > 5% or so

- Number of motors & servos
    o From the HW config, not mixer

- ADC/ESC telemetry speed
    o depends on platform

- Mixer
    o 'mixer status'

- motor config & cli

- Remove derivative feedforward on tail

- TEST blackbox
    o general functions
    o motor values
    o servo values
    o debug32

- FIX governor MSP

- FIX mixer input channel naming

- Split collective commit from smoothing

- FIX mixer/servo/motor help arg order

- remove VIRTUAL_CURRENT_METER

- Exclude DYN_LPF

- Exclude GYRO_DATA_ANAL

