### Timer-Driven Serial, I²C Sensor & PWM Audio on BBC micro\:bit


A single bare-metal **`CW2.cpp`** file that explores three classic embedded-systems chores on the nRF52-powered BBC micro\:bit **without using high-level CODAL helpers or interrupts**: precise timers, software serial, and PWM sound. The coursework is worth 33 % of the module and builds directly on the week-5 lecture.&#x20;

---

#### Subtasks

| #                                | What you build                                                                                                                                                      | Core API / behaviour                                                                                                                   |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| **1 — Bit-bang 115 200 baud**    | Software UART TX that writes the same byte stream on **P0.06 (USB-CDC)** *and* **edge pin P2**. A 240 ms hardware timer schedules each message.                     | `void bitBangSerial(char *string);`   `void voteForChocolate(void);` – streams a literal chocolate-bar vote every 240 ms until reset.  |
| **2 — I²C accelerometer logger** | Minimal driver for the **LSM303AGR** over the micro\:bit’s internal TWI; samples X/Y/Z \~5 Hz and dumps `[X: …] [Y: …] [Z: …]` lines over your software UART.       | `int getAccelerometerSample(char axis);`   `void showAccelerometerSamples(void);`                                                      |
| **3 — PWM buzzer synth**         | Configures the nRF52 **PWM** peripheral on the on-board speaker pin. Press A or B → 1 kHz square wave; bonus marks: map pitch 500 Hz – 5 kHz to live Y-axis tilt. | `void makeNoise(void);` – loops forever, reading accelerometer and updating duty cycle.                                                |

---

#### Build & flash

```bash
# Clone your coursework scaffold (CODAL & nRF SDK assumed)
yotta build                 # or mbed compile -t GCC_ARM
cp build/bbc-microbit-classic-gcc/source/CW2-combined.hex /media/MICROBIT
```

No additional hardware is needed; just connect USB for power & serial.
*Serial timing is cycle-counted on 16 MHz HFCLK, so build **Release** for accuracy.*

---

#### Implementation highlights

* **Pure register-level C** – only `#include "MicroBit.h"` plus nRF SDK typedefs.
* **First-call initialisation guards** ensure peripherals are configured exactly once.
* **Timer vs. busy-wait comparison** shows why hardware timers beat cycle loops.
* **Readable, test-friendly APIs** match the marker’s `MainSubtaskRunnerCW2.cpp`.
* Tight code, no “magic numbers”: all baud, bit-times, and duty-cycles derived from `TIMER0->PRESCALER` and `LFCLK/RTC` constants.

---

