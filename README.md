# CBUS&reg; CANBlock Module

This repository contains firmware for the Raspberry PICO to build a CBUS block instrument.

The pin mapping used by CANBlock is follows.

| Pico Pin | Function      |/| Pico Pin | Function      |
|----------|---------------|-|----------|---------------|
| 1        | GP0           | | 40       | VBUS          |
| 2        | GP0           | | 39       | VSYS          |
| 3        | GND           | | 38       | GND           |
| 4        | GP2 BUZZER    | | 37       | 3V3_EN        |
| 5        | GP3 BELL      | | 36       | 3V3           |
| 6        | GP4 LED TOT   | | 35       | ADC_VREF      |
| 7        | GP5 LED TOT   | | 34       | GP28          |
| 8        | GND           | | 33       | GND           |
| 9        | GP6 LED NRM   | | 32       | GP27          |
| 10       | GP7 LED NRM   | | 31       | GP26          |
| 11       | GP8 LED LC    | | 30       | RUN           |
| 12       | GP9 LED LC    | | 29       | GP22 LED R    |
| 13       | GND           | | 28       | GND           |
| 14       | GP10          | | 27       | GP21 LED G    |
| 15       | CAN RX        | | 26       | GP20 LED Y    |
| 16       | CAN TX        | | 25       | GP19          |
| 17       | GP13          | | 24       | GP18 BELL SW  |
| 18       | GND           | | 23       | GND           |
| 19       | GP14          | | 22       | GP17 FLiM BTN |
| 20       | GP15          | | 21       | GP16          |

CANBlock uses the soft PIO based CAN2040 CAN controller, so no external CAN controller is required, however a CAN2562 transceiver or similar MUST be connected to the Pico in order to communicate on CAN.

\attention CBUS&reg; is a registered trademark of Dr. Michael Bolton.  See [CBUS](https://cbus-traincontrol.com/)
