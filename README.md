<img width="800" height="600" alt="b3b8c003-c8e0-4a8e-9995-777f12522749" src="https://github.com/user-attachments/assets/34ae444f-40ee-4c03-a1b1-b5d1a443ceac" />

# btreceiver

A Bluetooth A2DP audio receiver on the Raspberry Pi Pico 2 W. Streams
SBC audio from a phone or PC, runs it through an 8-stage float32 DSP
chain on the M33 FPU, and plays it back through a PCM5102A I2S DAC.
Pairs on boot as **"2B Speaker"** with "just works" SSP; Class of
Device `0x240414` (Audio / Loudspeaker).

## Features

- **A2DP sink** with AVRCP (track metadata, transport controls,
  absolute-volume support so the source doesn't pre-attenuate PCM).
- **Float32 DSP chain** (see [dsp.cpp](dsp.cpp)):
  1. **Drive** — 4:1 soft-knee compressor, linked stereo envelope.
  2. **Warmth** — `tanhf` saturator with small DC bias for 2nd-harmonic
     "tube glow".
  3. **X-talk** — Bauer/Linkwitz-style 700 Hz headphone crossfeed.
  4. **Bass** — 150 Hz low-shelf, ±12 dB.
  5. **Mid** — 1 kHz peaking, ±12 dB.
  6. **Presence** — 4 kHz peaking, ±12 dB.
  7. **Treble** — 8 kHz high-shelf, ±12 dB.
  8. **Master volume** + always-on soft limiter (-1 dBFS, 1/50 ms).

  Order is colour-first, EQ-second. Bit-perfect passthrough when both
  the AVRCP slider and the master pot are at max.
- **8 physical pots** on two ADS1115 ADCs (one per DSP stage), polled
  at 20 Hz with EMA smoothing.
- **4 front-panel toggles**: tone bypass, colour bypass, volume policy
  (ignore phone slider), visualisation tap (pre/post-DSP).
- **SSD1306 OLED** showing a persistent volume bar with pot-popup
  override, plus a mirrored stereo spectrum (16 log-spaced FFT bands
  per channel with peak-hold).
- **Analog VU meters** driven directly from PWM (GPIO 16/17), fed from
  the post-volume MAV envelope.

## Hardware

- **Raspberry Pi Pico 2 W** (RP2350, ARM Cortex-M33 cores — not the
  RISC-V Hazard3 — so the FPU is available for float DSP).
- **PCM5102A "mk2" module** as the I2S DAC. Needs 5 V on VCC; SCK
  strapped to GND; XSMT held high.
- **2x ADS1115** on I2C (0x48, 0x49) for the pots.
- **SSD1306 128x64 OLED** on the same I2C bus (0x3C).
- **Raspberry Pi Debug Probe** for SWD + UART bridge (`printf` lands
  on the probe's USB CDC, independent of the target's USB).

### Pin assignment

| GPIO | Function |
|------|----------|
| 0 / 1 | UART0 TX / RX (to probe) |
| 8 / 9 | I2C0 SDA / SCL (OLED + both ADS1115s) |
| 14 | Tone-bypass toggle (SPST to 3V3, 10 kΩ pull-down) |
| 15 | Colour-bypass toggle |
| 16 / 17 | VU meter L / R (PWM, same slice) |
| 18 | Volume-policy toggle |
| 19 | Visualisation pre/post-DSP toggle |
| 26 | I2S DIN |
| 27 | I2S BCK |
| 28 | I2S LRCK |

I2C runs at **1 MHz** so a full SSD1306 framebuffer flush (~10 ms)
doesn't starve the audio refill timer.

See [SCHEMATIC.md](SCHEMATIC.md) for per-subsystem wiring diagrams
(power rails, I2C bus topology, I2S to the PCM5102A, pot ladders,
toggle pull-downs, VU meter PWM, debug probe).

## Build

Requires pico-sdk 2.2.0 and ARM toolchain `14_2_Rel1` with
`PICO_BOARD=pico2_w`. pico-extras is fetched automatically by
[CMakeLists.txt](CMakeLists.txt) via `FetchContent` (it must register
itself before `pico_sdk_init()`, which is why the dance at the top of
the file looks the way it does).

```sh
cmake -B build -G Ninja
ninja -C build
```

Output: `build/btreceiver.uf2` / `.elf`.

## Flash

Pick one:

- Drag-drop `build/btreceiver.uf2` to the BOOTSEL mass-storage mount.
- `picotool load build/btreceiver.uf2 -fx`
- Via debug probe:
  ```sh
  openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
          -c "adapter speed 5000" \
          -c "program build/btreceiver.elf verify reset exit"
  ```
  (Note: `target/rp2350.cfg`, not `target/rp2350-riscv.cfg` — this
  project runs on the ARM cores.)

## Debug

Serial monitor over the probe's UART bridge:

```sh
picocom -b 115200 /dev/ttyACM0   # Ctrl-A Ctrl-X to exit
```

The probe needs both ribbons connected to the target: SWD (D
connector) for stepping, and UART (U connector) for `printf`.

## Module layout

| File | Responsibility |
|------|----------------|
| [btreceiver.cpp](btreceiver.cpp) | `main()`, I2C / timer wiring, singleton display, front-panel toggle polling. |
| [bt.cpp](bt.cpp) / [bt.hpp](bt.hpp) | BTstack handlers (HCI, A2DP sink, AVRCP), `bt_setup()`, SDP records, GAP pairing config. |
| [audio.cpp](audio.cpp) / [audio.hpp](audio.hpp) | SBC decoder, ring buffers, drift-compensation resampler, I2S playback handler, scope tap. |
| [dsp.cpp](dsp.cpp) / [dsp.hpp](dsp.hpp) | Float32 DSP chain: drive, warmth, x-talk, 4-band EQ, master volume. |
| [pots.cpp](pots.cpp) / [pots.hpp](pots.hpp) | ADS1115 driver, EMA-smoothed pot polling, mapping to DSP setters + display popups. |
| [vu.cpp](vu.cpp) / [vu.hpp](vu.hpp) | PWM-direct VU meter driver. |
| [display.hpp](display.hpp) | Header-only SSD1306 driver, status strip, mirrored stereo spectrum renderer. |
| [spectrum.hpp](spectrum.hpp) | Header-only 256-pt real FFT in Q15 + bars + peak-hold. |
| [btstack_audio_pico.c](btstack_audio_pico.c) | I2S backend implementing BTstack's `btstack_audio_sink_t` HAL on top of pico-extras `pico_audio_i2s`. |
| [btstack_config.h](btstack_config.h) | BTstack sizing/features (Classic only, Bluedroid SBC decoder, no BLE). |

## Signal path

```
Phone (A2DP source)
    │ SBC over Bluetooth Classic (L2CAP)
    ▼
audio_handle_l2cap_media_data_packet()   [BTstack run loop]
    │ parse RTP+SBC; push frames; nudge resampler from backlog depth
    ▼
SBC ring buffer ──► SBC decoder ──► handle_pcm_data() (resample)
    ▼
playback_handler() runs dsp_run():
    int16 → float
      → drive → warmth → x-talk
      → bass → mid → presence → treble
      → master volume → soft limiter
      → float → int16 (saturated)
    ▼
btstack_audio_pico_sink (pico_audio_i2s, PIO + DMA)
    ▼
PCM5102A DAC → amp → speakers
```

See [AGENT.md](AGENT.md) for deeper design notes (DSP coefficient
formulas, run-loop integration, why the colour-first ordering, etc.).
