# Schematic

Wiring for the btreceiver build. Pin numbers are Pico 2 W **GPIO**
numbers (not physical pin numbers on the board).

## Power rails

```
   USB 5V ─┬─ Pico VBUS (pin 40)
           └─ PCM5102A VCC      (onboard LDO drops to 3.3 V for the chip)

  Pico 3V3(OUT) (pin 36) ─┬─ SSD1306 VCC
                          ├─ ADS1115 #1 VDD
                          ├─ ADS1115 #2 VDD
                          ├─ Pot tops (3V3 end of each pot)
                          └─ Toggle commons (3V3 side of each SPST)

         GND ─── common to all peripherals (PCM5102A has two GND pins;
                                            tie both together)
```

## I2C bus (shared)

One bus, three slaves. Pull-ups are on the breakout boards; no extra
externals needed at typical breadboard lengths.

```
                ┌─────────────────────────────┐
                │       Pico 2 W (I2C0)       │
                │                             │
                │   GPIO 8 (SDA) ─────────────┼──┬──────┬──────┬──────────
                │   GPIO 9 (SCL) ───────────┐ │  │      │      │
                │                           │ │  │      │      │
                └───────────────────────────┼─┼──┼──────┼──────┼──────────
                                            │ │  │      │      │
                                       ┌────┘ │  │      │      │
                                       │      │  │      │      │
                              ┌────────┴──────┴──┴──┐ ┌─┴──────┴────┐
                              │ SSD1306  OLED       │ │  ADS1115 #1 │
                              │  addr 0x3C          │ │  addr 0x48  │
                              │  SDA / SCL          │ │  SDA / SCL  │
                              └─────────────────────┘ └─────────────┘
                                                                │
                                                       ┌────────┴────┐
                                                       │  ADS1115 #2 │
                                                       │  addr 0x49  │
                                                       │  SDA / SCL  │
                                                       │  ADDR → SDA │
                                                       └─────────────┘
```

I2C runs at **1 MHz** (set in [btreceiver.cpp:197](btreceiver.cpp#L197))
so a full SSD1306 framebuffer flush stays under ~10 ms and doesn't
starve the audio refill timer.

## I2S audio out (PCM5102A "mk2" module)

```
   Pico 2 W                          PCM5102A
   ─────────                         ────────
   GPIO 26 (DIN) ─────────────────► DIN
   GPIO 27 (BCK) ─────────────────► BCK
   GPIO 28 (LCK) ─────────────────► LCK / LRCK

   5V (VBUS)     ─────────────────► VCC      (onboard 3.3 V LDO)
   GND           ─────────────────► GND      (both GND pins tied)

                                    SCK  ──► GND (strapped on board: internal PLL)
                                    XSMT ──► 3V3 (held high: mute disabled)
                                    FMT  ──► GND (I2S mode)
```

LCK is `BCK + 1` (= GPIO 28), set up in
[CMakeLists.txt:67-68](CMakeLists.txt#L67-L68) via the pico-extras
`pico_audio_i2s` PIO program.

## Pots (8 channels, 2x ADS1115)

Each pot is a 3-terminal 10 kΩ linear part: top to 3V3, bottom to GND,
wiper to its ADS1115 input.

```
                  3V3 ──┬── pot top
                        │
                       ┌┴┐
                       │ │  10 kΩ linear pot
                       │◄├──── wiper ─────► ADS1115 An
                       │ │
                       └┬┘
                        │
                  GND ──┴── pot bottom
```

Channel assignment:

| Pot | ADS1115 | Channel |
|-----|---------|---------|
| Bass     | #1 (0x48) | A0 |
| Mid      | #1 (0x48) | A1 |
| Treble   | #1 (0x48) | A2 |
| Presence | #1 (0x48) | A3 |
| Drive    | #2 (0x49) | A0 |
| Warmth   | #2 (0x49) | A1 |
| X-talk   | #2 (0x49) | A2 |
| Volume   | #2 (0x49) | A3 |

ADS1115 #2 is differentiated from #1 by tying its `ADDR` pin to **SDA**
instead of GND. Both chips run in single-shot mode at 860 SPS; the
driver issues a parallel-trigger / parallel-read sweep across both
chips per channel (~1.5 ms per channel pair).

## Front-panel toggles (4x SPST)

All four are wired identically: SPST switch with one terminal to 3V3
and the other to the GPIO, with a **10 kΩ external pull-down to GND**.
The internal pull-down is too weak to fight breadboard leakage.

```
                3V3 ──┬─┐
                        \
                         \  SPST toggle (closed = active HIGH)
                          \
                       ┌─┴───────── GPIO n
                       │
                      ┌┴┐
                      │ │  10 kΩ external pull-down
                      │ │
                      └┬┘
                       │
                GND ───┴
```

| GPIO | Toggle |
|------|--------|
| 14 | Tone bypass (skip all 4 EQ bands) |
| 15 | Colour bypass (skip drive + warmth + x-talk) |
| 18 | Volume policy (ignore phone slider, force AVRCP to 127) |
| 19 | Visualisation tap (pre-DSP when closed) |

GPIO 19 is wired with reversed polarity (chassis switch was mounted
that way) and inverted in software at
[btreceiver.cpp:86](btreceiver.cpp#L86).

## SSD1306 OLED

Standard 4-pin I2C breakout — no extras beyond the shared I2C bus.

```
   Pico 2 W              SSD1306
   ─────────             ───────
   3V3(OUT) ─────────► VCC
   GND      ─────────► GND
   GPIO 8   ─────────► SDA
   GPIO 9   ─────────► SCL
                       (I2C address 0x3C)
```

## VU meters (2x analog movement)

Driven directly off the Pico's PWM with no transistor or op-amp —
each meter coil is wired between its GPIO and GND. Both meters share
**PWM slice 8** (GPIO 16 = ch A, GPIO 17 = ch B), so they share clock
and wrap; only the duty cycle differs.

```
   Pico 2 W                   Analog VU meter (L)
   ─────────                  ───────────────────
   GPIO 16 (PWM slice 8 A) ──┬── (+) meter coil ──┐
                             │                    │
                             │     [calibration   │  ~146 kHz PWM,
                             │      resistor      │  10-bit duty
                             │      if needed]    │
                             │                    │
   GND ──────────────────────┴──────── (-) ───────┘

   GPIO 17 (PWM slice 8 B) ──── same wiring, right meter
```

PWM rate is well above audio so no extra filtering is needed; the
meter's mechanical inertia integrates the duty cycle.

## Debug probe (Raspberry Pi Debug Probe)

Two ribbons from the probe to the target:

```
   Probe (D connector)            Pico 2 W (SWD)
   ───────────────────            ──────────────
   SWCLK ─────────────────────►  SWCLK
   SWDIO ─────────────────────►  SWDIO
   GND   ─────────────────────►  GND


   Probe (U connector)            Pico 2 W (UART0)
   ───────────────────            ────────────────
   Probe TX ──────────────────►  GPIO 1 (UART0 RX)
   Probe RX ◄──────────────────  GPIO 0 (UART0 TX)
   GND      ──────────────────►  GND
```

`printf` lands on the probe's USB CDC (`/dev/ttyACM0`), independent
of the target's USB — the target's USB stack is not even initialised
([CMakeLists.txt:87](CMakeLists.txt#L87) sets
`pico_enable_stdio_usb(btreceiver 0)`).

## Full pin map (summary)

| GPIO | Direction | Function | Goes to |
|------|-----------|----------|---------|
| 0  | OUT | UART0 TX | Probe RX |
| 1  | IN  | UART0 RX | Probe TX (optional, stdin) |
| 8  | I/O | I2C0 SDA | SSD1306 + both ADS1115s |
| 9  | OUT | I2C0 SCL | SSD1306 + both ADS1115s |
| 14 | IN  | Tone-bypass toggle | SPST + 10 kΩ pull-down |
| 15 | IN  | Colour-bypass toggle | SPST + 10 kΩ pull-down |
| 16 | OUT (PWM) | VU meter L | Meter coil to GND |
| 17 | OUT (PWM) | VU meter R | Meter coil to GND |
| 18 | IN  | Volume-policy toggle | SPST + 10 kΩ pull-down |
| 19 | IN  | Vis pre/post toggle | SPST + 10 kΩ pull-down (inverted in SW) |
| 26 | OUT | I2S DIN | PCM5102A DIN |
| 27 | OUT | I2S BCK | PCM5102A BCK |
| 28 | OUT | I2S LCK | PCM5102A LCK |
