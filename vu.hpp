// vu.hpp -- analog VU meter driver.
//
// Two moving-coil VU meters (stereo L / R) driven directly from the
// RP2350's PWM peripheral. No analog buffer, no transistor, no RC
// filter -- the meter coils take the raw PWM output and the coil's
// own mechanical inertia + inductance integrate it.
//
// Audio envelope (mean absolute value, ~300 ms time constant) is
// computed in dsp.cpp and pulled via dsp_get_vu_env_{l,r}() at the
// 20 Hz pot-poll cadence. Calibration constants in vu.cpp map env
// to PWM duty so the needle reads 0 VU when the audio sits at the
// reference -18 dBFS.

#pragma once

void vu_init();
void vu_update();
