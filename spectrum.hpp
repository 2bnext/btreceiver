// spectrum.hpp -- 256-point real-input FFT + log-band aggregator with
// peak-hold, in fixed-point Q15. Designed for the SSD1306 spectrum
// visualisation: feed it a window of recent mono samples and read out
// 32 band magnitudes (0..63) plus 32 peak-hold values (0..63).
//
// CPU cost on RP2350 (ARM Cortex-M33 + FPU): well under 1 % of one
// core at 30 Hz. The implementation stays in fixed-point Q15 (the FPU
// is reserved for the audio-path DSP) so it'd be just as cheap on the
// RISC-V Hazard3 cores, which is what it was originally measured on
// before the project moved back to ARM.
//
// Memory: ~2.7 KB of LUTs and scratch.
//
// The trig / bit-reversal LUTs are filled at boot in init() using libc
// trig (sinf/cosf); runs once and is fine even on a softfloat target.

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

class Spectrum {
public:
    static constexpr size_t kFftN     = 256;       // FFT length (must be power of two)
    static constexpr size_t kFftLog2  = 8;         // log2(kFftN)
    static constexpr size_t kBands    = 16;        // bars per channel (L or R half)
    static constexpr int    kBarMaxPx = 64;        // bar can fill the full screen height
    // Peak-hold ballistics. New peaks freeze for kPeakHoldTicks
    // before they're allowed to decay; decay then drops 1 pixel every
    // kPeakDecayPeriod ticks. At 30 Hz that gives ~3 s hold + ~8.5 s
    // fall = ~11.5 s total visible peak.
    static constexpr int    kPeakHoldTicks   = 90;
    static constexpr int    kPeakDecayPeriod = 4;

    Spectrum() = default;

    // One-time setup. Builds the Hann window, FFT twiddle factors,
    // bit-reversal indices, and log-spaced band -> bin mapping. Cheap
    // (a few ms at 150 MHz, even via software trig) so call from main().
    void init() {
        // Hann window: 0.5 * (1 - cos(2*pi*n/(N-1)))
        for (size_t n = 0; n < kFftN; n++) {
            double w = 0.5 * (1.0 - cos(2.0 * M_PI * (double) n / (double) (kFftN - 1)));
            window_[n] = (int16_t) (w * 32767.0);
        }

        // Twiddle factors W_k = exp(-j*2*pi*k/N) for k = 0..N/2-1.
        for (size_t k = 0; k < kFftN / 2; k++) {
            double angle = -2.0 * M_PI * (double) k / (double) kFftN;
            twiddle_cos_[k] = (int16_t) (cos(angle) * 32767.0);
            twiddle_sin_[k] = (int16_t) (sin(angle) * 32767.0);
        }

        // Bit-reversal permutation indices.
        for (size_t i = 0; i < kFftN; i++) {
            uint16_t r = 0;

            for (size_t b = 0; b < kFftLog2; b++) {
                if (i & (1u << b)) {
                    r |= (uint16_t) (1u << (kFftLog2 - 1 - b));
                }
            }

            bit_rev_[i] = r;
        }

        // Log-spaced band -> FFT-bin mapping. Skip bin 0 (DC) and use
        // bins 1..N/2-1. Each band gets one or more bins; lower bands
        // are narrow (often just one bin), upper bands span many.
        const int    first_bin = 1;
        const int    last_bin  = (int) (kFftN / 2 - 1);
        const double ratio     = pow((double) last_bin / (double) first_bin,
                                     1.0 / (double) kBands);
        double cur = (double) first_bin;

        for (size_t b = 0; b < kBands; b++) {
            int lo = (int) cur;
            cur *= ratio;
            int hi = (int) cur - 1;

            if (b == kBands - 1) {
                hi = last_bin;
            }

            if (hi < lo) {
                hi = lo;
            }

            band_first_[b] = (uint8_t) lo;
            band_last_[b]  = (uint8_t) hi;
        }

        memset(bands_, 0, sizeof(bands_));
        memset(peaks_, 0, sizeof(peaks_));
    }

    // Process the most recent kFftN samples from a circular ring buffer.
    // `read_pos` is the index of the *oldest* sample in the ring (the
    // write head position).
    void update(const int16_t * samples, size_t count, size_t read_pos) {
        if (samples == nullptr || count < kFftN) {
            return;
        }

        // The newest sample is at (read_pos - 1) mod count; we want the
        // last kFftN samples ending there. Start = (read_pos - kFftN) mod count.
        size_t start = (read_pos + count - kFftN) % count;

        for (size_t n = 0; n < kFftN; n++) {
            int16_t s        = samples[(start + n) % count];
            int32_t windowed = ((int32_t) s * window_[n]) >> 15;
            real_[n] = (int16_t) windowed;
            imag_[n] = 0;
        }

        fft();

        // Magnitudes per bin via cheap "max + min/2" approximation
        // (accurate to ~5 % vs sqrt(re^2+im^2), fast enough that magnitude
        // costs nothing in the time budget).
        uint16_t mag[kFftN / 2];

        for (size_t k = 0; k < kFftN / 2; k++) {
            int32_t r  = real_[k];
            int32_t i  = imag_[k];
            int32_t ar = (r < 0) ? -r : r;
            int32_t ai = (i < 0) ? -i : i;
            int32_t mx = (ar > ai) ? ar : ai;
            int32_t mn = (ar > ai) ? ai : ar;
            mag[k] = (uint16_t) (mx + (mn >> 1));
        }

        // Aggregate bins per band, then sqrt-compress so quiet detail
        // stays visible. Empirical scale tuned so loud peaks fill the
        // screen height; tweak kBarScaleShift to taste.
        for (size_t b = 0; b < kBands; b++) {
            uint32_t sum = 0;

            for (uint8_t k = band_first_[b]; k <= band_last_[b]; k++) {
                sum += mag[k];
            }

            uint32_t scaled = isqrt(sum);                      // sqrt compression
            uint32_t pixels = scaled >> kBarScaleShift;        // map to bar pixels

            if (pixels > (uint32_t) (kBarMaxPx - 1)) {
                pixels = (uint32_t) (kBarMaxPx - 1);
            }

            bands_[b] = (uint8_t) pixels;

            // Peak hold: rises instantly, freezes for kPeakHoldTicks,
            // then drops 1 px every kPeakDecayPeriod ticks.
            if (bands_[b] > peaks_[b]) {
                peaks_[b]     = bands_[b];
                peak_hold_[b] = kPeakHoldTicks;
            } else if (peak_hold_[b] > 0) {
                peak_hold_[b]--;
            } else if ((decay_counter_ % kPeakDecayPeriod) == 0 && peaks_[b] > 0) {
                peaks_[b]--;
            }
        }

        decay_counter_++;
    }

    uint8_t band(size_t i) const { return bands_[i]; }
    uint8_t peak(size_t i) const { return peaks_[i]; }

private:
    // Tunable: bigger shift attenuates the bar height (loud signals
    // never fill the screen), smaller/zero lets peaks saturate. With
    // sqrt-compression already applied, 0 gives a good "music fills
    // the screen" feel: typical bins land around half-height, peaks
    // saturate to full.
    static constexpr int kBarScaleShift = 0;

    // Integer square root (Newton-style binary). Used once per band per
    // update -- 32 calls/frame, negligible cost.
    static uint32_t isqrt(uint32_t x) {
        uint32_t res = 0;
        uint32_t one = 1u << 30;

        while (one > x) {
            one >>= 2;
        }

        while (one != 0) {
            if (x >= res + one) {
                x  -= res + one;
                res = (res >> 1) + one;
            } else {
                res >>= 1;
            }

            one >>= 2;
        }

        return res;
    }

    // Iterative radix-2 Cooley-Tukey FFT in Q15 with per-stage scaling
    // (>>1 each butterfly) to keep results in int16 range. Loses 1 bit
    // per stage = 8 bits over 8 stages; remaining 8 bits are plenty
    // for visualisation.
    void fft() {
        // Bit-reversal permutation.
        for (size_t i = 0; i < kFftN; i++) {
            size_t j = bit_rev_[i];

            if (j > i) {
                int16_t tr = real_[i]; real_[i] = real_[j]; real_[j] = tr;
                int16_t ti = imag_[i]; imag_[i] = imag_[j]; imag_[j] = ti;
            }
        }

        // Butterfly stages: 2, 4, 8, ..., kFftN.
        for (size_t len = 2; len <= kFftN; len <<= 1) {
            size_t step = kFftN / len;
            size_t half = len >> 1;

            for (size_t i = 0; i < kFftN; i += len) {
                for (size_t k = 0; k < half; k++) {
                    size_t  tw = k * step;
                    int32_t cr = twiddle_cos_[tw];
                    int32_t ci = twiddle_sin_[tw];

                    int32_t r1 = real_[i + k + half];
                    int32_t i1 = imag_[i + k + half];

                    int32_t tr = (cr * r1 - ci * i1) >> 15;
                    int32_t ti = (cr * i1 + ci * r1) >> 15;

                    int32_t ur = real_[i + k];
                    int32_t ui = imag_[i + k];

                    real_[i + k]        = (int16_t) ((ur + tr) >> 1);
                    imag_[i + k]        = (int16_t) ((ui + ti) >> 1);
                    real_[i + k + half] = (int16_t) ((ur - tr) >> 1);
                    imag_[i + k + half] = (int16_t) ((ui - ti) >> 1);
                }
            }
        }
    }

    int16_t  real_[kFftN]            = {};
    int16_t  imag_[kFftN]            = {};
    int16_t  window_[kFftN]          = {};
    int16_t  twiddle_cos_[kFftN / 2] = {};
    int16_t  twiddle_sin_[kFftN / 2] = {};
    uint16_t bit_rev_[kFftN]         = {};

    uint8_t  band_first_[kBands]     = {};
    uint8_t  band_last_[kBands]      = {};
    uint8_t  bands_[kBands]          = {};
    uint8_t  peaks_[kBands]          = {};
    uint8_t  peak_hold_[kBands]      = {};
    uint32_t decay_counter_          = 0;
};
