
## THIS PAGE IS WORK-IN-PROGRESS

Please read the page [Rotorflight FC Design Requirements](https://github.com/rotorflight/rotorflight/wiki/Rotorflight-FC-Design-Requirements) first. It is explaining the generic requirements for all Rotorflight designs.

__For an FC to be fully supported by Rotorflight-2 (RF2), it _must_ use one of the designs on this page.__

RF2 will offer more features and easier configurability for designs that are fully compatible.

Lots of effort has been put into these designs for making sure they are as flexible as possible, while not imposing any hardware limitations on the functionality. The manufacturer can choose not to implement any features that are marked _optional_. All other features must be implemented.

The reference designs are considering only the aspects that have effect on the software support, mostly the STM32 resource allocation and the minimum set of supported features. The rest - like size, form factor, connector locations - are left for the manufacturer to decide.


# Reference Design F7A

The following design is for the STM32F722RET (64 pins LQFP) chip.


## Ports


### Servo and Motor Port

The servos and motors are connected to standard 0.1" pin headers.
The pin headers are organised as two blocks, of size 4x3 and 3x3 pins.

The cyclic servos and the tail servo/ESC connect to the 4x3 block.

The main ESC connects to the 3x3 block.

A receiver can be connected to the 3x3 block in some configurations.

All pin headers in these blocks have a common GND and a common VX (BEC power).

| Label    | Pin1 | Pin2 | Pin3 | MCU Pin |
| -------- | ---- | ---- | ---- | ------- |
| S1       | GND  | VX   | PWM  | PB0     |
| S2       | GND  | VX   | PWM  | PB1     |
| S3       | GND  | VX   | PWM  | PB4     |
| TAIL     | GND  | VX   | PWM  | PB3     |
|          |      |      |      |         |
| ESC      | GND  | VX   | SIG  | PB6     |
| TELEM    | GND  | VX   | SIG  | PA3     |
| RX/RPM   | GND  | VX   | SIG  | PA2     |

The ESC pin headers are designed to accommodate all variations of traditional and drone ESCs.
The following combinations are possible:

| Type                  | ESC Header     | TELEM Header  | RX/RPM Header | Example ESC                     |
| --------------------- | -------------- | ------------- | ------------- | ------------------------------- |
| Large heli ESC        | PWM + BEC      | Tele + BEC    | RPM + BEC     | Hobbywing Platinum V4 120A      |
| Small heli ESC        | PWM + BEC      | RPM + BEC     | Receiver⁶     | Hobbywing Platinum V3 50A       |
| BLHeli32 ESC          | DShot          | Telemetry     | BEC           | IFlight BLITZ E80               |
| BLHeli-S ESC          | DShot          | BEC           | Receiver⁶     | Holybro 20A                     |

A one-wire receiver can be connected to the RX/RPM header, if the ESC Telemetry feature is not in use.
The receiver must be high voltage compatible, e.g. up to 8.4V.


### Receiver Port

A dedicated receiver port is _mandatory_.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3 | Pin4 |
| ---- | ---- | ---- | ---- |
| GND  | 5V   | RX   | TX   |

The receiver port shall be labelled as "Receiver".

The TX/RX are connected to the MCU pins PA0/PA1 (UART4).


### DSM Port

A port for connecting a Spektrum DSM satellite is _optional_.

The connector type is 3-pin JST-ZH, with the following pins:

| Pin1 | Pin2 | Pin3 |
| ---- | ---- | ---- |
| 3.3V | GND  | RX   |

The DSM port shall be labelled as "DSM".

The RX is connected to the MCU pin PA9 (UART1).


### Expansion Port A

The expansion port A for external peripherals is _optional_.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3     | Pin4     |
| ---- | ---- | -------- | -------- |
| GND  | 5V   | RX/SDA   | TX/SCL   |

This port can act as a serial port, or an I2C port.

It shall be labelled as "Port A"

The signal pins (TX/RX) are connected to the MCU pins PB10/PB11.


### Expansion Port B

The expansion port B for external peripherals is _optional_.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3  | Pin4  |
| ---- | ---- | ----- | ----- |
| GND  | 5V   | RX    | TX    |

This port shall be labelled as "Port B"

The signal pins (TX/RX) are connected to the MCU pins PC6/PC7.


### Expansion Port C

The expansion port C for external peripherals is _optional_.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3  | Pin4  |
| ---- | ---- | ----- | ----- |
| GND  | 5V   | RX    | TX    |

This port shall be labelled as "Port C"

The signal pins (TX/RX) are connected to the MCU pins PC12/PD2.


### Expansion Port D

The expansion port D for external peripherals is _optional_.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3  | Pin4  |
| ---- | ---- | ----- | ----- |
| GND  | 5V   | RX    | TX    |

This port shall be labelled as "Port D"

The TX pin is shared with the DSM port. Usually this port is not
implemented if the DSM Port is present, unless the best possible
expansion capability is required.

The signal pins (TX/RX) are connected to the MCU pins PA9/PA10.


### Expansion Port E

The expansion port E for external peripherals is _optional_,
but it must be implemented together with Port G.

The connector type is 4-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3  | Pin4  |
| ---- | ---- | ----- | ----- |
| GND  | 5V   | RX    | TX    |

This port shall be labelled as "Port E".

The signal pins (TX/RX) are connected to the MCU pins PC10/PC11.


### Expansion Port G

The expansion port G for a GPS and a Compass is _optional_,
but it must be implemented together with Port E.

The Port G replaces Ports A and B.

The connector type is 6-pin JST-GH, with the following pins:

| Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 |
| ---- | ---- | ---- | ---- | ---- | ---- |
| GND  | SDA  | SCL  | TX   | RX   | 5V   |

This socket is Pixhawk GPS compatible.

The MCU pins are PB10, PB11, PC6, PC7.


## MCU Resource Allocation

| Function      | PIN    | ALT1   | ALT2   | ALT3   | ALT4   | Notes     |
| ------------- | ------ | ------ | ------ | ------ | ------ | ----------|
| Servo1        | PB4    | T3Ch1  |
| Servo2        | PB5    | T3Ch2  |
| Servo3        | PB0    | T3Ch3  |
| Servo4        | PB1    | T3Ch4  |||| Optional: Fourth cyclic servo¹ |
| Tail          | PB3    | T2Ch2  |||| Tail servo or motor |
| Motor1        | PB6    | T4Ch1  |||| Main motor |
| Motor2        | PB7    | T4Ch2  |||| Optional: Second motor¹ |
|               |        |
| UART1 Tx      | PA9    | TX1    | T1Ch2⁹ ||| Optional: UART, LED strip, CC |
| UART1 Rx      | PA10   | RX1    | T1Ch3⁹ ||| Optional: UART, LED strip, CC |
| UART2 Tx      | PA2    | TX2    | T5Ch3 | T9Ch1 | A2 | ESC Telem, RPM, CPPM, ADC, LED strip⁵ |
| UART2 Rx      | PA3    | RX2    | T5Ch4 | T9Ch2 | A3 | ESC Telem, RPM, CPPM, ADC, LED strip⁵ |
| UART3 Tx      | PC10   | TX3    | TX4 |
| UART3 Rx      | PC11   | RX3    | RX4 |
| UART4 Tx      | PA0    | TX4    | T5Ch1 | A0 || Dedicated receiver port |
| UART4 Rx      | PA1    | RX4    | T5Ch2 | A1 || Dedicated reciever port |
| UART5 Tx      | PC12   | TX5    |
| UART5 Rx      | PD2    | RX5    |
| UART6 Tx      | PC6    | TX6    | T8Ch1⁹ ||| Optional: UART, LED strip, CC, GPS |
| UART6 Rx      | PC7    | RX6    | T8Ch2⁹ ||| Optional: UART, LED strip, CC, GPS |
|               |        |
| SCL1          | PB8    | SCL1   |||| Internal baro |
| SDA1          | PB9    | SDA1   |||| Internal baro |
|               |        |
| SCL2          | PB10   | SCL2   | TX3 | T2Ch3 || Optional: External compass, UART3, PWM |
| SDA2          | PB11   | SDA2   | RX3 | T2Ch4 || Optional: External compass, UART3, PWM |
|               |        |
| NSS           | PA4    | NSS1   |||| Gyro SPI NSS |
| SCK           | PA5    | SCK1   |||| Gyro SPI SCK |
| MISO          | PA6    | MISO1  |||| Gyro SPI MISO |
| MOSI          | PA7    | MOSI1  |||| Gyro SPI MOSI |
| GEXT          | PA15   | EXTI   |||| Gyro INT |
|               |        |
| NSS           | PB12   | NSS2   |||| Flash SPI NSS |
| SCK           | PB13   | SCK2   |||| Flash SPI SCK |
| MISO          | PB14   | MISO2  |||| Flash SPI MISO |
| MOSI          | PB15   | MOSI2  |||| Flash SPI MOSI |
|               |        |
| ADC1          | PC0    | A10 |||| Vx voltage sensor⁷ |
| ADC2          | PC1    | A11 |||| 5V voltage sensor⁷ |
| ADC3          | PC4    | A14 |||| Optional: Gyro 3.3V sensor⁷ |
| ADC4          | PC2    | A12 |||| Optional: Vx current sensor⁸ |
| ADC5          | PC3    | A13 |||| Optional: 5V current sensor⁸ |
|               |        |
| LED STRIP     | PA8    | T1Ch1⁹ |||| Optional: Dedicated LED strip |
|               |        |
| BEEPER        | PC13   | GPIO |||| Optional: buzzer |
| LED1          | PC14   | GPIO |||| Green status LED³ |
| LED2          | PC15   | GPIO |||| Red status LED³ |
|               |        |
| USB DM        | PA11   | DM |||| USB data- |
| USB DP        | PA12   | DP |||| USB data+ |
|               |        |
| SWDIO         | PA13   | SWDIO |||| Debugger test pad⁴ |
| SWCLK         | PA14   | SWCLK |||| Debugger test pad⁴ |
|               |        |
| Free¹⁰        | PB2    |
| Free¹⁰        | PC8    |
| Free¹⁰        | PC9    |

¹ The optional motors and servos should have solder pads on the PCB.

² PC10/PC11 can be configured as UART3 or UART4, or a combination of RX3/TX4, allowing S.BUS and S.Port on the same socket. This is ideal for a dedicated receiver socket.

³ A dual colour LED is preferred.

⁴ For easy debugging, GND,3V3,SWDIO,SWCLK solder pads should be available on the PCB.

⁵ PA2/PA3 is used for a combined ESC telemetry / RPM / Receiver pin header block.

⁶ A receiver can be connected if ESC telemetry is not used. PA2 and PA3 also support CPPM receivers.

⁷ A voltage divider and a filter cap is needed on each ADC input. The cutoff frequency for the input filter should be ~ 100Hz.

⁸ A high side current sensor is preferred, like the INA139. Low side ground shunts should be avoided.

⁹ TIM1 can be used only if TIM8 is unused; and vice versa.

¹⁰ If space permits, all unused/free pins should have solder pads on the PCB.

