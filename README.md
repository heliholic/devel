# Rotorflight

Rotorflight is a Flight Control software suite for single-rotor helicopters.

It is based on Betaflight 4.3, with many advanced features added for helicopters.
Rotorflight does **NOT** support multi-rotor crafts, nor airplanes; it is only for RC helicopters.


## Information

For more information, please visit [Rotorflight Website](https://www.rotorflight.org/)


## Features

Rotorflight has many features:

* Many Rx protocols: CRSF, S.BUS, F.Port, DSM, IBUS, XBUS, EXBUS, GHOST, CPPM
* RX telemetry protocols: CSRF, FrSky, HoTT, etc.
* ESC telemetry protocols: BLHeli32, Hobbywing, Scorpion, Kontronik, OMP Hobby, ZTW, APD, YGE
* Advanced PID control tuned for helicopters
* Rotor speed governor
* Motorised tail support with TTA
* Advanced gyro filtering
  - Dynamic RPM based notch filters
  - Dynamic notch filters based on FFT
  - Dynamic LPF
* Extra servo/motor outputs for AUX functions
* Fully customisable servo/motor mixer
* Sensors for battery voltage, current, BEC, etc.
* Stabilisation modes (6D)
* High-speed Blackbox logging

Plus lots of features inherited from Betaflight:

* Configuration profiles for changing various tuning parameters
* Rates profiles for changing the stick feel and agility
* Multiple ESC protocols: PWM, DShot, Multishot, etc.
* Configurable buzzer sounds
* Multi-color RGB LEDs
* GPS support

And many more...


## Hardware support

The best hardware for Rotorflight is any Flight Controller especially designed for it.

Please see [What board is suitable?](https://www.rotorflight.org/docs/Tutorial-Quickstart/What-Board)

Otherwise, Rotorflight supports all flight controller boards that are supported by Betaflight.
Assuming that the board has enough suitable I/O pins for connecting all the servos and motors required.

Also, the Betaflight boards are labeled for multi-rotor use - thus the user needs to understand how
these functions can be used for a different purpose with helicopters. Usually this is just about using
the motor outputs for servos, but in some cases a more advanced remapping may be needed.

Rotorflight supports STM32G4, STM32F4, STM32F7 and STM32H7 MCUs from ST.

It is highly recommended to use an STM32F7 based flight controller, as Rotorflight greatly benefits
from the latest control and filtering algorithms and other new features that are all CPU intensive.

An absolute minimum is an STM32G4 based board, but it probably won't be able to run all the new
features later on. The older STM32F411 should be avoided if possible.


## Installation

The firmware should be _always_ flashed with the
[Rotorflight Configurator](https://github.com/rotorflight/rotorflight-configurator/releases).

Usually there is no need to download or flash the firmware manually.

Please see the [website](https://www.rotorflight.org/) for further instructions how to configure
and use Rotorflight.


## Contributing

Rotorflight is a open-source community project. Anybody can join in and help to make it better by:

* helping other users on Rotorflight Discord or other online forums
* [reporting](https://github.com/rotorflight?tab=repositories) bugs and issues, and suggesting improvements
* testing new software versions, new features and fixes; and providing feedback
* participating in discussions on new features
* create or update content on the [Website](https://www.rotorflight.org)
* [contributing](https://www.rotorflight.org/docs/Contributing/intro) to the software development - fixing bugs, implementing new features and improvements
* [translating](https://www.rotorflight.org/docs/Contributing/intro#translations) Rotorflight Configurator into a new language, or helping to maintain an existing translation


## Origins

Rotorflight is software that is **open source** and is available free of charge without warranty.

Rotorflight is forked from [Betaflight](https://github.com/betaflight), which in turn is forked from [Cleanflight](https://github.com/cleanflight).
Rotorflight borrows ideas and code also from [HeliFlight3D](https://github.com/heliflight3d/), another Betaflight fork for helicopters.

Huge thanks to all those whom have contributed along the way!


## Contact

Rotorflight team can be contacted by email at rotorflightfc@gmail.com.
