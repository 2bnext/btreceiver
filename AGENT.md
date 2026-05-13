# btspkr ("2B Speaker")

A Bluetooth A2DP audio receiver built on a Raspberry Pi Pico 2 W,
streaming to a PCM5102A I2S DAC, with an all-software DSP chain
(float32 on the M33 FPU) controlled by physical pots.

## Status

- A2DP sink works end-to-end: pair-on-boot ("just works" SSP), SBC
  decode, I2S out to a PCM5102A "mk2" module, audio plays through an
  external amp. Local name is "2B Speaker"; CoD = 0x240414
  (Audio / Loudspeaker).
- AVRCP wired up: track changes, title/album/artist metadata, the
  source's volume slider all reach us. We declare absolute-volume
  support so the source doesn't pre-attenuate, keeping incoming PCM
  at full resolution.
- **Float32 DSP chain on the Cortex-M33 FPU** (`dsp.cpp`). The int16
  buffer from the SBC decoder is promoted to normalised float
  (±1.0), every active stage runs in float, then saturated back to
  int16 for I2S. ~144 dB of dynamic range, no chained-stage overflow
  the way Q31 had with the presence boost.
- **DSP chain** (full 8-stage signal path -- all live):
    1. **Drive** -- soft-knee 4:1 compressor with linked stereo
       envelope. threshold = -24 dB × amount, makeup = +12 dB ×
       amount, 5/100 ms attack/release. Linear-domain gain reduction
       via `(threshold/env)^0.75 = sqrt(t/e) * sqrt(sqrt(t/e))` to
       avoid `powf`.
    2. **Warmth** -- real `tanhf` saturator with a small DC bias
       (`drive ∈ [1, 2]`, `bias ∈ [0, 0.1]` × amount²) for 2nd-
       harmonic "tube glow". DC subtracted, output normalised so
       peak amplitude stays roughly constant across drive settings.
       Uses `tanhf` (~80 cyc/sample × 2 ch) rather than a Padé
       approximation -- the Padé is faster (~10 cyc) but unbounded
       for large inputs, which clipped audibly on loud signal.
    3. **X-talk** -- Bauer/Linkwitz-style headphone crossfeed: a
       1st-order 700 Hz LPF on each channel, mixed into the opposite
       ear at up to ~50% level. Approximates the head-shadow effect
       of speakers in a real room so hard-panned mixes don't
       ping-pong on headphones. Direct/cross gains normalised to
       preserve per-ear level.
    4. **Bass** -- 150 Hz low-shelf, Q = 0.7, ±12 dB.
    5. **Mid** -- 1 kHz peaking, Q = 0.7, ±12 dB.
    6. **Presence** -- 4 kHz peaking, Q = 1.0, ±12 dB.
    7. **Treble** -- 8 kHz high-shelf, Q = 0.7, ±12 dB.
    8. **Master volume** -- (AVRCP slider × pot) in Q15. Bit-perfect
       passthrough when both are at max so full-scale audio reaches
       the DAC untouched.
    9. **Soft limiter** -- always-on, end-of-chain safety net. Linked
       stereo peak detector, threshold = -1 dBFS, attack = 1 ms,
       release = 50 ms. Engages only when stacked drive + EQ pushes
       peaks above threshold; idle on normal material. The downstream
       int16 saturation is the brick-wall backstop for sub-millisecond
       overshoots that slip through the attack ramp.

  Order is colour-first / EQ-second by user preference: the
  saturator shapes the raw audio without seeing EQ-boosted peaks,
  x-talk applies headphone room simulation to the coloured signal,
  EQ runs last so the user can tame any saturator harshness on the
  final output.

  All four EQ bands share a Direct Form I biquad implementation
  (`apply_band` in `dsp.cpp`); they differ only in the coefficient
  formula (low-shelf for bass, peaking for mid/presence, high-shelf
  for treble) and the centre frequency / Q. RBJ Cookbook EQ.

  Each stage has a small dead zone around its "neutral" pot
  position (centre detent for EQ, 0 for drive/warmth/x-talk) so the
  detent is a true bypass.

- **ADS1115 pot scanning** (`pots.cpp`): full 8-channel sweep every
  50 ms (20 Hz per pot). The two chips are converted in parallel --
  trigger 0x48 then 0x49 on the same channel, sleep once for
  ~1.5 ms, read both results back -- so a sweep is 4 channels x
  ~1.5 ms = ~6 ms blocking per pot tick. EMA smoothing (alpha=1/4)
  gives ~250 ms settling. Channels configured in single-shot mode
  at 860 SPS (~1.16 ms per conversion).

  Channel allocation:
  - **#1 (0x48)**: A0 = bass, A1 = mid, A2 = treble, A3 = presence
  - **#2 (0x49)**: A0 = drive, A1 = warmth, A2 = x-talk, A3 = volume

  Pot values are read on boot (no flash persistence -- pots are
  mechanical, the wiper position is the source of truth).
- **SSD1306 OLED** (`display.hpp`, header-only C++) -- single
  permanent layout, no mode cycling:
    - **Top status strip** (page 0, rows 0..7): power icon at far
      left (always solid; the device is on); Bluetooth runic-B icon
      next to it, drawn only while AVRCP is connected; then a
      3-character pot label and a 1-px-framed horizontal bar that
      defaults to "VOL" + the master-volume Q15. Any pot motion
      makes the bar take that pot's label/value for ~1.5 s, then
      it auto-reverts to volume -- so the level indicator is always
      visible at a glance.
    - **Spectrum** (rows 9..63, 55 px tall): mirrored stereo bar
      view. 16 log-spaced FFT bands per channel + peak-hold lines
      (3 s hold, ~8.5 s decay), 256-pt real FFT in Q15
      (`spectrum.hpp`), Hann window. L draws on cols 0..62 with
      *low frequencies on the inside* (near the centre gap) and
      highs reaching the left edge; R draws on cols 65..127 with
      lows on the inside and highs on the right edge. Cols 63..64
      are a deliberate 2 px blank centre gap -- no separator line,
      bass thump pulses outward from the middle on both sides.
    - **Row 8** is a 1-px breathing margin between the top strip
      and the spectrum.
- **Track metadata** (AVRCP title / album / artist / device name) is
  no longer rendered on the OLED. `bt.cpp` still prints it to the
  serial console for debugging; if an info panel is reinstated the
  data is one AVRCP-getter call away.
- **Scope mode** was deleted along with the multi-mode cycling. The
  post-DSP L/R sample tap in `audio.cpp` is preserved -- it's what
  feeds the spectrum FFT.

## Hardware

### Target

- **Raspberry Pi Pico 2 W** (RP2350). We use the **dual ARM
  Cortex-M33 cores** (toolchain `14_2_Rel1`) -- not the RISC-V
  Hazard3 cores -- so we have a hardware FPU for the float DSP.
  Switched from RISC-V mid-project after stacked DSP stages started
  saturating Q31 audibly.
- **PCM5102A "mk2" module** as the I2S DAC.
  - **Needs 5 V on VCC** (onboard LDO drops to 3.3 V for the chip).
  - Two GND pins exposed; both go to common ground.
  - SCK strapped to GND on-board (internal PLL); XSMT held high.
- **2x ADS1115** breakouts on the I2C bus for the pots (0x48, 0x49).
  3 more spares.
- **SSD1306 128x64 OLED** at 0x3C on the same I2C bus.
- **Raspberry Pi Debug Probe** for SWD single-stepping and the UART
  serial bridge that carries `printf` output (probe's CDC, not the
  target's USB).

### Current pin assignment

| GPIO | Function | Notes |
|------|----------|-------|
| 0 | UART0 TX -> probe RX | `printf` output via probe's CDC |
| 1 | UART0 RX | optional, for stdin |
| 8 | I2C0 SDA | OLED + both ADS1115s |
| 9 | I2C0 SCL | OLED + both ADS1115s |
| 14 | Tone-bypass toggle | SPST to 3V3, 10 kΩ external pull-down to GND; closed = bypass on |
| 15 | Colour-bypass toggle | SPST to 3V3, 10 kΩ external pull-down to GND; closed = bypass on |
| 16 | VU meter L (PWM) | slice 8 ch A; ~146 kHz, 10-bit duty drives the coil directly |
| 17 | VU meter R (PWM) | slice 8 ch B; same slice as L so they share clock + wrap |
| 18 | Volume-policy toggle | SPST to 3V3, 10 kΩ pull-down; closed = ignore phone slider, force 127 |
| 19 | Vis pre/post toggle | SPST to 3V3, 10 kΩ pull-down; closed = meters/spectrum tap pre-DSP |
| 26 | I2S DIN | to PCM5102 DIN |
| 27 | I2S BCK | to PCM5102 BCK |
| 28 | I2S LCK / LRCK | to PCM5102 LCK |

I2S pins are configured in `CMakeLists.txt` via `BTSPKR_I2S_DATA_PIN`
and `BTSPKR_I2S_CLOCK_PIN_BASE` (BCK is the base; LCK = base + 1).

I2C runs at **1 MHz** so the SSD1306 framebuffer flush doesn't block
the BTstack run loop long enough to starve the audio refill timer.

## Build, flash, debug

- SDK: pico-sdk 2.2.0, ARM toolchain `14_2_Rel1` (`PICO_BOARD=pico2_w`).
- pico-extras is fetched via `FetchContent` in `CMakeLists.txt` (must
  be added before `pico_sdk_init()` so its `post_init.cmake` registers
  correctly).
- Build: `ninja` from `build/`.
- Flash: drag-drop UF2, or `picotool load build/btspkr.uf2 -fx`, or
  via probe with `openocd -f interface/cmsis-dap.cfg -f
  target/rp2350.cfg -c "adapter speed 5000" -c "program
  build/btspkr.elf verify reset exit"`. (Note: target/rp2350.cfg --
  not target/rp2350-riscv.cfg, since we're back on ARM.)
- Serial monitor: `picocom -b 115200 /dev/ttyACM0` (the probe's UART
  bridge -- exit with `Ctrl-A Ctrl-X`).
- The probe needs **two** ribbons connected to the target: SWD (D
  connector) for stepping, and UART (U connector, three wires:
  Probe TX/RX/GND <-> Target GPIO 1/0/GND) for printf.

## Software architecture

```
Phone (A2DP source)
     |   SBC over Bluetooth Classic (L2CAP)
     v
audio_handle_l2cap_media_data_packet()      [BTstack run loop]
     |   parse RTP+SBC headers; push SBC frames into ring buffer;
     |   nudge resampler factor based on backlog depth (drift comp)
     v
sbc_frame_ring_buffer_
     |   pulled by playback_handler() when I2S asks for samples
     v
sbc_decoder_instance_->decode_signed_16()
     |   calls handle_pcm_data() per decoded frame
     v
handle_pcm_data()
     |   resample, split between request buffer and overflow ring
     v
playback_handler() runs the full DSP chain on the request buffer:
     int16 -> float
       -> drive (compressor) -> warmth (saturator) -> x-talk (crossfeed)
       -> bass -> mid -> presence -> treble
       -> master volume -> sat -> int16
     v
btstack_audio_pico_sink (pico_audio_i2s, PIO + DMA)
     v
PCM5102A DAC -> amp -> speakers
```

### Module layout

`btspkr.cpp` was split into focused modules in March 2026; the boot
file is now small and the heavy lifting lives in dedicated TUs.

| File | Responsibility |
|------|----------------|
| `btspkr.cpp` | `main()` + I2C/timer wiring + the singleton `Display display(I2C_PORT)` definition. |
| `bt.hpp/.cpp` | All BTstack handlers (HCI / A2DP sink / AVRCP top + controller + target) and `bt_setup()` (SDP records, GAP "just works" pairing, local name, CoD). |
| `audio.hpp/.cpp` | SBC decoder, ring buffers, drift-compensation resampler, I2S `playback_handler`, scope-tap. |
| `dsp.hpp/.cpp` | Float32 DSP chain: `dsp_run()`, drive/warmth/x-talk stages, 4-band EQ (shared `BandState` helper), master volume; coefficient recompute on Fs change. |
| `pots.hpp/.cpp` | ADS1115 driver, EMA-smoothed pot polling, mapping pot values to DSP setters + display popups. |
| `vu.hpp/.cpp` | PWM-direct VU meter driver (GPIO 16/17), pulls the post-volume MAV envelope from `dsp.cpp` and maps to duty via the meter calibration constants. |
| `display.hpp` | Header-only SSD1306 driver: 5x7 font, 7x7 power/BT icons, top status strip (persistent volume + pot popup), mirrored stereo spectrum, embedded `Spectrum` instance. |
| `spectrum.hpp` | Header-only 256-pt real FFT in Q15 + bars+peak-hold renderer. |
| `btstack_audio_pico.c` | I2S backend implementing BTstack's `btstack_audio_sink_t` HAL on top of pico-extras `pico_audio_i2s`. |
| `btstack_config.h` | BTstack feature/sizing config (Classic only, Bluedroid SBC decoder, no BLE, no stdin commands). |
| `CMakeLists.txt` | Links `pico_btstack_classic`, `pico_btstack_sbc_decoder`, `pico_btstack_cyw43`, `pico_cyw43_arch_threadsafe_background`, `pico_audio_i2s`, `hardware_pwm`. lwIP intentionally not linked (`CYW43_LWIP=0`). |

### Why this run-loop integration

- `cyw43_arch_init()` (with `pico_btstack_cyw43` linked) is the single
  init call: it brings up the CYW43439 radio AND calls
  `btstack_memory_init()` + `btstack_run_loop_init()` on the cyw43
  async_context.
- `btstack_run_loop_execute()` is what `main()` blocks in; the run
  loop is interrupt-driven (timer + IRQ from the cyw43 path) so no
  busy waiting.
- Two periodic timers run on the BTstack loop:
  - **Display tick** at ~30 Hz (33 ms) -- `display.tick()` advances
    the pot-popup countdown, runs the L/R FFT update, and redraws
    the full panel (top strip + mirrored spectrum) every tick.
  - **Pot poll** at 20 Hz (50 ms) -- `pots_poll_tick()` (ADS1115
    read, EMA smoothing, DSP setters, popup trigger).
- We use `pico_cyw43_arch_threadsafe_background` (no lwIP) -- pure
  Bluetooth-only build.

## Code style preferences (current owner)

- Descriptive comments throughout (not "why-only"). Comments explain
  intent and surface non-obvious gotchas.
- **No single-line `if` bodies**; always braced and on their own line.
- **Blank line before and after** every `if` / `while` / `case` block,
  except where directly bordered by `{` or `}`.
- One statement per line in `case` bodies; no trailing `; break;` on
  the same line as the case label.
- `extern "C"` only where strictly needed for C linkage (e.g. linking
  to the C-built `btstack_audio_pico.c`).

## DSP chain + control surface (live)

All 8 pots wired, all 8 DSP stages live (drive / warmth / x-talk /
4-band EQ + master volume + soft limiter). The limiter is the only
stage without a pot -- it's safety, not colour.

### Pot layout (8 controls)

| Pot | ADC | Function | Filter / behaviour |
|-----|-----|----------|--------------------|
| **Bass** | #1 A0 | low-shelf | 150 Hz, Q = 0.7, ±12 dB |
| **Mid** | #1 A1 | peaking | 1 kHz, Q = 0.7, ±12 dB |
| **Treble** | #1 A2 | high-shelf | 8 kHz, Q = 0.7, ±12 dB |
| **Presence** | #1 A3 | peaking | 4 kHz, Q = 1.0, ±12 dB |
| **Drive** | #2 A0 | 4:1 compressor | threshold = -24 dB × amount; makeup = +12 dB × amount; 5/100 ms attack/release; linked stereo envelope |
| **Warmth** | #2 A1 | `tanhf` saturator | drive ∈ [1, 2], bias ∈ [0, 0.1], both scaled by amount²; output peak-normalised |
| **X-talk** | #2 A2 | crossfeed (head shadow) | 700 Hz LPF on each channel mixed into opposite ear; up to ~50% mix at amount = 1 |
| **Volume** | #2 A3 | master | linear pot + square-law audio taper in software |

Plus four **physical SPST toggles** on the front panel:
- Tone bypass (skip all 4 EQ bands)
- Colour bypass (skip drive + warmth + x-talk)
- Volume policy (ignore phone slider, force AVRCP to 127)
- Visualisation tap (move VU + spectrum to pre-DSP for analysis)

### Signal chain (as built)

```
SBC -> resample
    -> drive (4:1 compressor, linked stereo envelope)
    -> warmth (tanhf saturator + DC bias for 2nd-harmonic)
    -> x-talk (700 Hz LP'd opposite-channel crossfeed)
    -> bass -> mid -> presence -> treble
    -> volume
    -> soft limiter (-1 dBFS / 1 ms / 50 ms, linked stereo)
    -> [scope + spectrum tap, future VU meter tap]
    -> I2S
```

Order is **colour-first, EQ-second** by user preference. The
saturator shapes the raw decoded audio without seeing EQ-boosted
peaks (which was important: an earlier EQ-first arrangement fed
the warmth saturator with above-±1.0 peaks and the original Padé-
approximation tanh overshot the ±1 asymptote and clipped audibly).
The audio.cpp scope/spectrum tap sits *after* `dsp_run` so it
always shows post-everything signal.

Order rationale:

- Colour first so the saturator sees raw audio bounded by ±1, and
  so EQ can tame any saturator harshness on the post-everything
  signal.
- Compressor (Drive) before saturation -- "studio channel strip"
  order; reduces peaks before the soft clipper sees them.
- X-talk after warmth: crossfeed mixes the *finished* per-channel
  signal so the head-shadow simulation sees the same content the
  ear would on speakers.
- Volume last so adjusting it doesn't change tone, dynamics, or
  stereo image.
- The global soft limiter (-1 dBFS / 1 ms / 50 ms, linked stereo)
  sits just before the I2S sat and catches the rare overshoot from
  stacked EQ + colour gain so the saturation step isn't doing the
  limiting via hard clipping.

### Drive (compressor) curve

```
amount in [0, 1]                      (pot Q15 / 32767, 0 = bypass)
ratio       = 4:1                     (fixed)
threshold   = -24 dBFS * amount
makeup_gain = +12 dB    * amount
attack/release = 5 ms / 100 ms        (fixed)
detector    = max(|L|, |R|)           (linked stereo)
gain_red    = (threshold/env) ^ 0.75  for env > threshold (else 1.0)
```

Linear-domain gain reduction via `sqrt(t/e) * sqrt(sqrt(t/e))` --
two FPU sqrts beats `powf` and a log/exp pair on the M33.
`amount = 0` is a hard bypass (small dead zone). Threshold and
makeup linked so peak level stays roughly constant -- only RMS /
loudness rises.

### Warmth (tube) curve

```
y = (tanhf(drive * (x + bias)) - tanhf(drive * bias)) / norm
curve  = amount * amount      (squared so the knob feels gradual at low
                               positions; tanh is most sensitive near 0)
drive  = 1 + 1 * curve        in [1, 2]
bias   = 0.1 * curve          in [0, 0.1]    (asymmetric -> 2nd-harmonic)
norm   = tanhf(drive * (1 + bias)) - tanhf(drive * bias)
```

The drive ceiling started at 5 (drive ∈ [1, 5]); user reported even
small pot motions felt heavy, so we squared the amount mapping AND
dropped the ceiling to 2. Result: a much subtler, "tape warmth" feel
across the full pot range rather than "tube overdrive at the top".

We use real `tanhf` (~80 cyc/sample × 2 ch) rather than a Padé
approximation. The Padé is faster (~10 cyc) but unbounded for
large arguments -- with EQ ahead of warmth the saturator was fed
peaks > ±1.0 and the Padé overshot the ±1 asymptote, clipping
audibly downstream. Real `tanhf` is properly bounded; the curve
shape at moderate inputs (where most music lives) is nearly
identical anyway.

### X-talk (headphone crossfeed) curve

Bauer/Linkwitz-style crossfeed: each channel's signal is low-passed
at ~700 Hz (the head-shadow knee -- highs are attenuated more than
lows when crossing the head) and mixed into the *opposite* ear at a
controlled level. Cures the "ping-pong stereo" headache that hard-
panned mixes cause on headphones; the user can dial it back to zero
on speakers where the room already provides crosstalk.

```
amount in [0, 1]                          (pot Q15 / 32767, 0 = bypass)

LPF cutoff = 700 Hz, fixed                # 1st-order, no resonance
mix        = 0.5 * amount                 # max ~50% LP'd opposite channel
direct     = 1 / (1 + mix)
cross      = mix / (1 + mix)              # so direct + cross = 1

out_L = direct * L + cross * lpf(R)
out_R = direct * R + cross * lpf(L)
```

The 1/(1+mix) normalisation keeps total per-ear level constant as
the crossfeed amount rises -- the mix shifts the *image* without
changing the loudness. Single LPF state pair per channel; the LPF
on each channel is reused as the source for the opposite channel's
crossfeed.

### CPU budget (RP2350 @ 150 MHz, ARM M33 + FPU)

Single-precision float on the M33 FPU. Measured / estimated:

- 4 biquad bands, stereo, float: ~3 % of one core.
- Drive (compressor) -- linked envelope + sqrt(sqrt) gain reduction:
  ~2 %.
- Warmth -- real `tanhf` × 2 ch: ~5 %. Slightly more than the Padé
  alternative (~1 %), but the Padé is unbounded for large arguments
  and clipped audibly at loud signal -- the +4 % is buying correct
  saturation behaviour.
- X-talk -- 2 LPFs (one per channel) + 4 multiplies + 2 adds: ~1 %.
- Volume: <1 %.
- SBC decode + resample: ~5-10 % under load.

Total ~20 % of one core under load. Plenty of headroom on the
single core; the second M33 is idle and available if we ever want
to split signal-path DSP from the BT/UI loop.

User reported a +10 mA current bump after enabling float DSP +
spectrum FFT. ~50 mW @ 5 V; the FPU is the headline (active power
while constantly running biquads + tanhf), plus the spectrum FFT
hitting the float pipeline ~30× per second, plus ADS1115 polling
traffic. Acceptable trade for what it bought.

## Toggle switches

Four physical SPST toggles, all live. Each is wired active-HIGH:
one terminal to 3V3, the other to a GPIO. A **10 kΩ external
pull-down** to GND holds the pin firmly LOW when the switch is
open -- the internal pull-down (~50-80 kΩ) is too weak to fight
the 30-50 µA of leakage observed (likely capacitive coupling from
neighbouring I2C / I2S clock lines on the breadboard), which left
the pin floating around 2 V. The external pull-down brings the
open-state reading to ~0.3 V -- comfortably below VIL. The internal
pull-down is left enabled as a no-cost soft fallback. Polled at
20 Hz on the same `pots_poll_tick` cadence -- well above mechanical
bounce, so naive sampling is glitch-free.

| Toggle | GPIO | When closed (active) |
|--------|------|----------------------|
| **Tone bypass**          | 14 | Skip all 4 EQ bands (bass / mid / presence / treble); colour stages still run. |
| **Colour bypass**        | 15 | Skip drive + warmth + x-talk; tone EQ still runs. |
| **Volume policy**        | 18 | Ignore phone-side `set_absolute_volume`; force AVRCP slider to 127 on every change and re-assert every 5 s so the source delivers bit-perfect audio (we attenuate locally). |
| **Visualisation tap**    | 19 | Move VU envelope follower + OLED spectrum tap to *before* the on-board DSP chain (so the meters / bars show the raw decoded source rather than the post-EQ output). **Wired with inverted polarity** -- the chassis switch was already mounted that way, so `toggles_poll` in `btspkr.cpp` reads `!gpio_get(GPIO19)`. Closed switch is still "active" from the user's perspective. |

Tone / colour bypasses skip the relevant `apply_band` / `apply_drive`
/ `apply_warmth` / `apply_xtalk` calls in `dsp_run`; coefficients
and per-stage state stay current while bypassed so flipping back
picks up cleanly at the live pot positions.

The volume policy lives in `bt.cpp`: `bt_set_volume_policy_ignore`
gates the AVRCP target's incoming `VOLUME_CHANGED` handler and
pushes 127 back to the phone via `avrcp_target_volume_changed` on
engagement, on connect, on every inbound change, and on a 5-second
re-assertion timer (driven from `bt_volume_policy_tick` in the pot
timer).

The visualisation toggle is read by both the VU env follower
(`apply_vu_tap` in `dsp.cpp`, gated on `visualisation_pre_`) and
the scope tap in `audio.cpp` (which checks `dsp_get_visualisation_pre`
to decide whether to capture L/R samples before or after `dsp_run`).
Audio path itself is unaffected; only the meter / spectrum tap
point moves.

## VU meters

Two moving-coil VU meters (stereo L/R), driven by **PWM only** --
no analog buffer, no transistor, no RC filter. The meter coil's own
mechanical inertia + inductance integrates the PWM into a clean
average voltage.

- **Pins**: GPIO 16 (L) and GPIO 17 (R) -- same PWM slice so they
  share clock + wrap and stay in lockstep.
- **PWM frequency**: ~146 kHz (10-bit wrap of 1023 at default
  150 MHz / clkdiv 1.0). Well above audible so no coil whine; 1024
  steps is plenty for needle ballistics.
- **Tap**: post-volume, pre-limiter (`apply_vu_tap` in `dsp.cpp`,
  between `apply_volume` and `apply_limiter`). The needle reflects
  music level, not the limiter's response to occasional overshoots.
- **Ballistics**: per-channel one-pole IIR follower, ~300 ms time
  constant: `alpha = 1 - exp(-1/(fs * 0.3))`; `env += (|x| - env) *
  alpha`. Default coefficient baked for 44.1 kHz so the follower
  works on cold start; `dsp_set_sample_rate` updates it for other
  rates.
- **Update cadence**: `vu_update()` called from the 50 ms pot timer
  (20 Hz). Slow compared to audio rate, fast compared to the needle's
  mechanical response.
- **CPU**: env follower is ~0.5 % of one core; PWM update is just
  two register writes.

### Calibration (vu.cpp)

Reference: **0 VU = -18 dBFS RMS sine** (per the user's chosen
loudness reference, leaves 18 dB of headroom for peaks before the
needle pegs). Mean absolute value of a sine = (2/π) × peak, so
-18 dBFS sine has MAV = 0.0801 in the float-normalised audio.

The user measured the physical meter: **0.2 V across the coil drives
the needle to the 0 VU mark**. With PWM at 3.3 V supply, that's
duty = 0.2 / 3.3 = 6.06 % = 62 counts of a 1024-wrap PWM.

So the env-to-duty mapping is `duty = env * (62 / 0.0801) = env *
~775`, clamped at the wrap value. Hot material that drives env past
~1.32 saturates the PWM but the meter has its own mechanical stop
anyway.

If a different meter is used, only the `kVuVMeterAt0VU` constant
needs to change -- everything else recalculates from it at compile
time.

## Notes on audio quality

- The PCM5102A is genuinely good: 112 dB DNR, -93 dB THD+N. Used in
  many hi-fi Pi HATs.
- The current quality ceiling is **SBC**, not the DAC. SBC over A2DP
  is lossy and capped at ~328 kbps with audible artifacts on cymbals
  and high-frequency content.
- Future codec upgrades worth knowing about:
  - **AAC** -- best universal upgrade (covers iPhone *and* Android).
    Decoder: Fraunhofer FDK-AAC (mature but large).
  - **LDAC** -- Sony, open-source decoder, up to 990 kbps. Most
    CPU-heavy.
  - **aptX** -- Qualcomm, license-required for legitimate
    distribution; reverse-engineered decoders exist (legally grey).
  - **LC3 (LE Audio)** -- newer, requires Bluetooth 5.2+ Isochronous
    Channels. BTstack supports it but it's a much bigger lift.
- The cheap OLED has a visible row-32 page seam and weak column
  drivers -- vertical scope strokes look like single columns rather
  than full bars. Not a software issue; the pixels are doing what
  they're told. A nicer OLED swap would clean it up.

## Open issues

- **Linux/Ubuntu source doesn't connect** reliably (Android works).
  Not yet investigated -- likely a PIN/SSP capability mismatch on
  the laptop side or a profile-priority quirk.

## Roadmap

1. ~~**Software volume**~~ -- done. Q15 multiply, bit-perfect at max.
2. ~~**OLED display**~~ -- done. Single-layout view: persistent
   top status strip (power + BT icons + always-on volume bar with
   pot-popup takeover) above a mirrored stereo FFT spectrum.
3. ~~**ADS1115 driver + EMA pot loop**~~ -- done. Two ADCs at 20 Hz.
4. ~~**Switch to ARM + float32 DSP**~~ -- done. Cleared the Q31
   crackles when stages stack.
5. ~~**Presence band**~~ -- done. 4 kHz peaking biquad on a real pot.
6. ~~**Code split**~~ -- done. `btspkr.cpp` 1551 → ~115 lines,
   logic moved to `bt.cpp` / `audio.cpp` / `dsp.cpp` / `pots.cpp`.
7. ~~**Cold-boot reliability**~~ -- done. SSD1306 POR window +
   triple "display off" flush in `display.init()`; I2C bus recovery
   in `btspkr.cpp` for debugger-halted-mid-transaction.
8. ~~**Alive icons**~~ -- done. Power symbol always solid + a
   Bluetooth runic-B that appears when AVRCP is connected. Both
   live in the top-left of the status strip.
9. ~~**3 more EQ bands**~~ -- done. Bass low-shelf, mid peaking,
   treble high-shelf, all on real pots, RBJ Cookbook coefficients.
10. ~~**8-channel pot scanning**~~ -- done. Round-robin mux scan in
    single-shot mode; full sweep every 200 ms.
11. ~~**Drive (compressor)**~~ -- done. 4:1, linked stereo envelope.
12. ~~**Warmth (tube saturation)**~~ -- done. `tanhf` + DC bias.
13. ~~**FM (transmission character)**~~ -- shipped, then dropped
    in favour of X-talk (the FM stage was a band-limit + M/S
    narrow that didn't earn its slot once headphone listening
    became part of the use case).
14. ~~**X-talk (headphone crossfeed)**~~ -- done. 700 Hz LPF
    crossfeed in the same pot/chain slot the FM stage occupied.
15. ~~**Toggle switches**~~ -- done. Four live: tone bypass (GPIO 14),
    colour bypass (GPIO 15), volume policy (GPIO 18), visualisation
    pre/post-DSP tap (GPIO 19).
16. ~~**VU meters**~~ -- done. Stereo PWM-direct drive on GPIO 16/17,
    ~300 ms IIR ballistics, calibrated to 0.2 V at 0 VU = -18 dBFS.
17. ~~**Soft limiter**~~ -- done. Always-on, -1 dBFS / 1 ms / 50 ms,
    linked stereo, sits just before the int16 sat.
18. **Higher-quality codec** (likely AAC) -- big project.
19. **Dual-core split** -- DSP on one M33, BT/UI on the other.
    Optional; we have plenty of headroom on a single core.

## Display performance notes

- 1 MHz I2C so framebuffer flushes don't block the BTstack run loop
  long enough to starve the audio refill timer.
- The unified tick runs at **~30 Hz (33 ms)**. The full 1024-byte
  framebuffer is re-rendered and flushed every tick (8 pages ×
  ~1.2 ms = ~10 ms blocking @ 30 Hz). Well under the audio backend's
  ~35 ms buffer reserve, so no underruns. With the multi-mode logic
  gone, the dirty-page tracking that used to gate marquee redraws
  is no longer needed -- everything redraws every tick anyway.
- Pot popup behaviour: any pot motion >0.4% of full scale calls
  `display.show_pot(label, q15)`, which sets the top bar's label/
  value for `kPotPopupTicks` (~1.5 s) before it auto-reverts to
  the persistent "VOL" + master-volume display.

If audio ever needs more headroom, the next step is **DMA-driven I2C**
(~50 LOC) so the CPU returns from `flush_pages` immediately. Currently
not necessary.

## Things to leave alone

- The BTstack a2dp_sink demo provenance is preserved at the top of
  `bt.cpp`. The `0x240414` Class of Device (Audio / Loudspeaker)
  and "just works" SSP behaviour are intentional.
- `lwipopts.h` is left in the tree even though it's unused now -- it's
  cheap insurance if Wi-Fi is ever turned on for OTA firmware updates
  or remote control.
- Pot values are **not** persisted to flash. Pots are mechanical: the
  wiper position *is* the saved state, read on boot before audio
  starts. Earlier flash-persistence work was deliberately reverted
  after it caused audio plops on save.
- Do not re-add stdin / console UI code from the original a2dp_sink
  demo; it was stripped intentionally.
