// dsp.hpp -- audio DSP chain. Public API; state lives in dsp.cpp.
//
// Pipeline: int16 PCM -> normalised float buffer
//        -> [drive -> warmth -> x-talk -> 4-band EQ -> volume -> limiter]
//        -> saturate back to int16 in place.
//
// Floats give ~144 dB dynamic range so chained EQ / drive / warmth /
// drive/warmth boosts can't blow past the buffer; the FPU on the M33 makes
// each stage cheap.

#pragma once

#include <stdint.h>

// Run all active DSP stages on a stereo int16 buffer in place.
// Called from the audio backend's playback callback.
void dsp_run(int16_t * buffer, uint16_t frames);

// Recompute Fs-dependent coefficients (currently just the presence
// biquad). Called from media_processing_init when SBC negotiates a
// new sampling frequency.
void dsp_set_sample_rate(uint32_t fs);

// Master volume contributors. Master = avrcp * pot in Q15 (so either at
// max is "transparent", either at zero mutes). Both setters also recompute
// the combined master and the bit-perfect-passthrough flag.
void dsp_set_avrcp_volume_127(uint8_t volume_0_to_127);
void dsp_set_pot_volume_q15(int16_t already_audio_tapered);

// 4-band EQ pots. Each is +/- 12 dB; q15 == 0 -> -12 dB, 16384 -> 0 dB
// (bypassed via dead zone), 32767 -> +12 dB. Recomputes coefficients
// on every call (cheap; the audio path doesn't notice).
//   bass     -- 150 Hz low-shelf,   Q = 0.7
//   mid      -- 1 kHz peaking,      Q = 0.7
//   presence -- 4 kHz peaking,      Q = 1.0
//   treble   -- 8 kHz high-shelf,   Q = 0.7
void dsp_set_bass_q15(int16_t q15);
void dsp_set_mid_q15(int16_t q15);
void dsp_set_presence_q15(int16_t q15);
void dsp_set_treble_q15(int16_t q15);

// "Colour" + spatial stages, chained ahead of the EQ in this order:
//
//   drive   -- soft-knee 4:1 compressor. q15 == 0 is bypass; up to
//              full = -24 dB threshold + 12 dB makeup gain. 5/100 ms
//              attack/release.
//   warmth  -- tanh-style saturator with a small DC bias for 2nd-
//              harmonic flavour (tube glow). q15 == 0 is bypass.
//   x-talk  -- Bauer/Linkwitz-style headphone crossfeed: a 700 Hz
//              LP'd copy of each channel mixed into the opposite
//              ear to approximate speaker-in-room head-shadow.
//              q15 == 0 is bypass (full discrete stereo).
//
// These pots map 0..32767 -> "amount" 0..1, *not* a +/- bipolar range.
void dsp_set_drive_q15(int16_t q15);
void dsp_set_warmth_q15(int16_t q15);
void dsp_set_xtalk_q15(int16_t q15);

// Front-panel bypass toggles. When engaged the corresponding stages are
// skipped wholesale in dsp_run -- coefficients/state are kept up to date
// in the background so flipping the switch off picks up immediately at
// the current pot positions.
//   tone   -- skip all four EQ bands (bass / mid / presence / treble).
//   colour -- skip drive + warmth + x-talk.
// Master volume and the soft limiter always run.
void dsp_set_tone_bypass(bool bypass);
void dsp_set_colour_bypass(bool bypass);

// VU meter envelope getters. Returns the current per-channel
// mean-absolute-value follower (~300 ms time constant). Tap location
// (pre- or post-DSP) is selected by dsp_set_visualisation_pre(). In
// post mode the env still runs post-volume / pre-limiter so the
// limiter's rare gain reduction doesn't make the needle bob.
// Range 0.0..~1.0 in float-normalised audio units; physical-meter
// calibration (env -> PWM duty) lives in vu.cpp.
float dsp_get_vu_env_l();
float dsp_get_vu_env_r();

// Visualisation tap selector. Affects both the VU envelope follower
// and the audio.cpp scope ring (which feeds the OLED spectrum):
//   false (default): tap post-DSP -- shows what's actually being sent
//                    to the DAC after EQ / colour / volume.
//   true           : tap pre-DSP  -- shows the raw decoded audio
//                    before any of the on-board processing.
// The audio path itself is unaffected; only the meters + spectrum
// move their tap point.
void dsp_set_visualisation_pre(bool pre);
bool dsp_get_visualisation_pre();
