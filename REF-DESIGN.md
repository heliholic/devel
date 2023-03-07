
## THIS PAGE IS WORK-IN-PROGRESS

Please read the page [Rotorflight FC Design Requirements](https://github.com/rotorflight/rotorflight/wiki/Rotorflight-FC-Design-Requirements) first. It is explaining the generic requirements for all Rotorflight designs.

__For an FC to be fully supported by Rotorflight-2 (RF2), it _must_ use one of the designs on this page.__

RF2 will offer more features and easier configurability for designs that are fully compatible.

Lots of effort has been put into these designs for making sure they are as flexible as possible, while not imposing any hardware limitations on the functionality. The manufacturer can choose not to implement any features that are marked _optional_. All other features must be implemented.
 
The reference designs are considering only the aspects that have effect on the software support, mostly the STM32 resource allocation and the minimum set of supported features. The rest - like size, form factor, connector locations - are left for the manufacturer to decide.


# Reference Design F7A

The following design is for the STM32F722RET (64 pins LQFP) chip.


### Servos and Motors

The servos and motors are connected to standard 0.1" pin headers, all with VX (BEC) power.

| Pin1 | Pin2 | Pin3 | MCU Pin |
| ---- | ---- | ---- | ------- |
| GND  | VX   | S1   | PB0     |
| GND  | VX   | S2   | PB1     |
| GND  | VX   | S3   | PB4     |
| GND  | VX   | TAIL | PB3     |
|      |      |      |         |
| GND  | VX   | ESC  | PB6     |
| GND  | VX   | TELE/RPM |  PA3 |
| GND  | VX   | RPM/RX   |  PA2 |


The ESC pin headers are designed to accommodate all variations of traditional ESCs and drone ESCs.
The following combinations are possible:

| Type                  | Header1 (PB6)  | Header2 (PA3) | Header3 (PA2) | Example ESC                     |
| --------------------- | -------------- | ------------- | ------------- | ------------------------------- |
| Large heli ESC        | PWM + BEC      | Tele + BEC    | RPM + BEC     | Hobbywing Platinum V4 120A      |
| Small heli ESC        | PWM + BEC      | RPM + BEC     | Receiver⁶     | Hobbywing Platinum V3 50A       |
| BLHeli32 ESC          | DShot          | Telemetry     | BEC           | IFlight BLITZ E80               |      
| BlueJay ESC           | DShot          | BEC           | Receiver⁶     |                                 |


### AUX Connectors

The AUX connectors are for peripherals, including the receiver. The connector type is a 4-pin JST-GH.

| Pin1 | Pin2 | Pin3 | Pin4 | Resource  | Notes                                   |
| ---- | ---- | ---- | ---- | --------- | --------------------------------------- |
| GND  | 5V   | RX3  | TX3  | PC10,PC11 | UART: ELRS,CRSF,S.BUS,S.Port,F.Port²    |
| GND  | 5V   | RX4  | TX4  | PA0,PA1   | UART, RPM, ADC                          |
| GND  | 5V   | RX6  | TX6  | PC6,PC7   | Optional: UART, CC, LEDS                |
| GND  | 5V   | RX1  | TX1  | PA9,PA10  | Optional: UART                          |


The optional GPS connector is 6-pin JST-GH (Pixhawk compatible).

| Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 | Resource             |
| ---- | ---- | ---- | ---- | ---- | ---- | -------------------- |
| GND  | SDA2 | SCL2 | TX5  | RX5  | 5V   | PB11,PB10,PC12,PD2   |


### Resource Allocation

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
| UART1 Tx      | PA9    | TX1    | T1Ch2⁹ ||| Optional: UART, LED strip |
| UART1 Rx      | PA10   | RX1    | T1Ch3⁹ ||| Optional: UART, LED strip |
| UART2 Tx      | PA2    | TX2    | T5Ch3 | T9Ch1 | A2 | ESC Telem, RPM, CPPM, ADC, LED strip⁵ |
| UART2 Rx      | PA3    | RX2    | T5Ch4 | T9Ch2 | A3 | ESC Telem, RPM, CPPM, ADC, LED strip⁵ |
| UART3 Tx      | PC10   | TX3    | TX4 ||| Dedicated Receiver port² |
| UART3 Rx      | PC11   | RX3    | RX4 ||| Dedicated Receiver port² |
| UART4 Tx      | PA0    | TX4    | T5Ch1 | A0 || RPM, ADC, LED strip |
| UART4 Rx      | PA1    | RX4    | T5Ch2 | A1 || RPM, ADC, LED strip |
| UART5 Tx      | PC12   | TX5    |||| Optional: GPS header |
| UART5 Rx      | PD2    | RX5    |||| Optional: GPS header |
| UART6 Tx      | PC6    | TX6    | T8Ch1⁹ ||| Optional: UART, LED strip, CC, PWM |
| UART6 Rx      | PC7    | RX6    | T8Ch2⁹ ||| Optional: UART, LED strip, CC, PWM |
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

⁵ PA2/PA3 are ideal for a combined ESC telemetry / RPM / Receiver pin header block, for one-wire receivers. For two-wire (UART) receivers, PC10/PC11 is the best choice.

⁶ A receiver can be connected if ESC telemetry is not used. PA2 and PA3 also support CPPM receivers.

⁷ A voltage divider and a filter cap is needed on each ADC input. The cutoff frequency for the input filter should be ~ 100Hz.

⁸ A high side current sensor is preferred, like the INA139. Low side ground shunts should be avoided.

⁹ TIM1 can be used only if TIM8 is unused; and vice versa.

¹⁰ If space permits, all unused/free pins should have solder pads on the PCB.

