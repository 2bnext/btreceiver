// vu.cpp -- analog VU meter PWM driver + calibration.
//
// Two PWM outputs on the same slice (so they share clock + wrap and
// stay in lockstep) drive moving-coil meters directly. The meter's
// mechanical/electrical low-pass response integrates the PWM into a
// smooth average voltage; we just need PWM frequency above audible
// (>20 kHz) so the coil doesn't buzz. We run at ~146 kHz which is
// comfortably above and gives 10-bit resolution.
//
// Calibration:
//   Reference audio level: 0 VU = -18 dBFS RMS sine (per AGENT.md
//   roadmap). For a sine, mean absolute value = (2/pi) * peak; at
//   -18 dBFS the peak is 10^(-18/20) = 0.1259, so MAV = 0.0801.
//
//   Reference meter level: the user measured the physical meter at
//   0.2 V for the 0 VU deflection. With PWM at 3.3 V supply, that's
//   duty = 0.2 / 3.3 = 6.06 % = 62 counts of a 1024-wrap PWM.
//
// So when the env follower reads 0.0801 we want 62 PWM counts, which
// means duty = env * (62 / 0.0801) = env * ~774. Hot material clamped
// to the wrap value -- the meter has a mechanical stop anyway.

#include "vu.hpp"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "dsp.hpp"

namespace {

// PWM pin assignment. Pico 2 W: GPIO 16 = PWM slice 8 channel A,
// GPIO 17 = slice 8 channel B -- same slice, so the two channels share
// the wrap value and the clock divisor and stay in sync automatically.
constexpr unsigned kGpioVuL = 16;
constexpr unsigned kGpioVuR = 17;

// 10-bit PWM. clkdiv = 1.0 -> freq = 150 MHz / 1024 = ~146 kHz, well
// above audible so no coil whine. 1024 levels gives 0.1 % steps, more
// than enough resolution for needle ballistics.
constexpr uint16_t kPwmWrap = 1023;
constexpr int      kPwmLevels = (int) kPwmWrap + 1;  // 1024

// Calibration -- see file-level comment.
constexpr float kVuVMeterAt0VU  = 0.2f;
constexpr float kVuVSupply      = 3.3f;
constexpr float kVuDutyAt0VU    = (kVuVMeterAt0VU / kVuVSupply) * (float) kPwmLevels;
constexpr float kVuEnvAt0VU     = 0.0801f;  // (2/pi) * 10^(-18/20)

// Empirical trim against music played through the device. Started
// at 0.80 (visually matched the OLED spectrum), then dialled down a
// further -3 dB (factor 0.7079) so loud commercial music sits near
// 0 VU instead of pegging the needles. Net trim 0.80 * 0.7079 = ~0.566.
// Tweak this single constant if a different perception target is wanted.
constexpr float kVuTrim         = 0.566f;
constexpr float kVuEnvToDuty    = (kVuDutyAt0VU / kVuEnvAt0VU) * kVuTrim;  // ~438

// Pre-DSP visualisation tap sees audio before master volume + any EQ
// cuts, so the env follower runs hotter for the same listening level.
// User reported the needles sat ~5 dB high in pre-DSP mode; apply a
// -5 dB compensation (factor 10^(-5/20) = 0.5623) when that toggle is
// engaged so both modes land in roughly the same place.
constexpr float kVuPreModeAttn  = 0.5623f;

int env_to_duty(float env) {
    if (dsp_get_visualisation_pre()) {
        env *= kVuPreModeAttn;
    }

    int duty = (int) (env * kVuEnvToDuty);

    if (duty < 0) {
        return 0;
    }

    if (duty > (int) kPwmWrap) {
        return (int) kPwmWrap;
    }

    return duty;
}

}  // namespace

void vu_init() {
    gpio_set_function(kGpioVuL, GPIO_FUNC_PWM);
    gpio_set_function(kGpioVuR, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(kGpioVuL);

    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_wrap(slice, kPwmWrap);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    pwm_set_enabled(slice, true);
}

void vu_update() {
    const uint slice = pwm_gpio_to_slice_num(kGpioVuL);

    pwm_set_chan_level(slice, PWM_CHAN_A, (uint16_t) env_to_duty(dsp_get_vu_env_l()));
    pwm_set_chan_level(slice, PWM_CHAN_B, (uint16_t) env_to_duty(dsp_get_vu_env_r()));
}
