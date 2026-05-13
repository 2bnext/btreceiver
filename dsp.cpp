// dsp.cpp -- DSP chain implementation. Float pipeline on the M33 FPU.
//
// Chain order (each stage runs in float, normalised to +/-1.0):
//   int16 in -> drive -> warmth -> x-talk
//            -> bass -> mid -> presence -> treble
//            -> master volume -> soft limiter -> int16 out
//
// Colour first, EQ second: the saturator shapes the raw audio without
// seeing EQ-boosted peaks; x-talk applies headphone room simulation
// (Bauer/Linkwitz crossfeed) to the coloured signal; EQ runs last so
// the user can tame any saturator harshness on the final output.
//
// All four EQ bands share a Direct Form I biquad implementation; they
// differ only in the coefficient calculation (low-shelf for bass,
// peaking for mid/presence, high-shelf for treble) and in the centre
// frequency / Q. Each band has a +/-12 dB range with a small dead zone
// around 0 dB so the pot's centre detent is a true bypass.

#include "dsp.hpp"

#include <math.h>
#include <stdint.h>

namespace {

// ---------------------------------------------------------------------
// DSP working buffer (float, normalised to +/-1.0).
// Sized for the audio backend's max buffer (SAMPLES_PER_BUFFER from
// pico_audio_i2s = 512 frames stereo = 1024 floats = 4 KB).
// ---------------------------------------------------------------------
constexpr size_t kDspMaxFrames = 512;
constexpr int    kNumChannels  = 2;
float            dsp_buf_[kDspMaxFrames * kNumChannels] = {};

constexpr float kInt16ToFloat = 1.0f / 32768.0f;
constexpr float kFloatToInt16 = 32767.0f;
constexpr float kPi           = 3.14159265358979323846f;

uint32_t sample_rate_ = 44100;

// Front-panel bypass toggles (SPST switch active-HIGH; internal
// pull-down holds the GPIO low when open). Per-stage state stays
// current while bypassed so flipping the switch off picks up
// cleanly at the pot positions of the moment.
bool tone_bypass_   = false;
bool colour_bypass_ = false;

// Visualisation tap selector. When true, the VU env-follower runs
// on the just-promoted-from-int16 float buffer (raw decoded audio);
// audio.cpp also moves its scope tap to before dsp_run. When false,
// both run post-DSP / post-volume.
bool visualisation_pre_ = false;

// ---------------------------------------------------------------------
// Master volume = avrcp * pot in Q15. Either at max keeps the chain
// passthrough (bit-perfect) so no volume scaling is applied.
// ---------------------------------------------------------------------
int16_t avrcp_vol_q15_          = 32767;  // phone slider (linear)
int16_t pot_vol_q15_            = 32767;  // pot, after audio-taper curve
int16_t master_vol_q15_         = 32767;  // (avrcp * pot) >> 15
bool    master_vol_passthrough_ = true;

void recompute_master_volume() {
    int32_t combined = ((int32_t) avrcp_vol_q15_ * (int32_t) pot_vol_q15_) >> 15;

    if (avrcp_vol_q15_ >= 32767 && pot_vol_q15_ >= 32767) {
        master_vol_passthrough_ = true;
        master_vol_q15_         = 32767;
        return;
    }

    if (combined < 0) {
        combined = 0;
    } else if (combined > 32767) {
        combined = 32767;
    }

    master_vol_passthrough_ = false;
    master_vol_q15_         = (int16_t) combined;
}

void apply_volume(float * buf, uint16_t frames) {
    if (master_vol_passthrough_) {
        return;
    }

    const float vol = (float) master_vol_q15_ * (1.0f / 32767.0f);

    for (uint16_t i = 0; i < frames; i++) {
        buf[i * 2]     *= vol;
        buf[i * 2 + 1] *= vol;
    }
}

// ---------------------------------------------------------------------
// Per-band EQ: pot value, biquad coefficients, biquad state.
// ---------------------------------------------------------------------
//   pot_q15 == 0      -> -kEqMaxDb (full cut)
//   pot_q15 == 16384  -> 0 dB (flat, bypassed by the dead-zone check)
//   pot_q15 == 32767  -> +kEqMaxDb (full boost)
constexpr float kEqMaxDb  = 12.0f;
constexpr float kEqDeadDb = 0.3f;

enum class BandKind : uint8_t {
    LowShelf,
    Peaking,
    HighShelf,
};

struct BandState {
    BandKind kind;
    float    freq_hz;
    float    Q;
    int16_t  pot_q15  = 16384;
    bool     bypass   = true;

    float    b0       = 1.0f;
    float    b1       = 0.0f;
    float    b2       = 0.0f;
    float    a1       = 0.0f;
    float    a2       = 0.0f;

    float    lx1      = 0.0f, lx2 = 0.0f;
    float    ly1      = 0.0f, ly2 = 0.0f;
    float    rx1      = 0.0f, rx2 = 0.0f;
    float    ry1      = 0.0f, ry2 = 0.0f;
};

// Frequencies + Qs from the AGENT.md signal chain. Mid and treble use
// the classic "wide" Q=0.7; presence is a tighter Q=1.0 since it is
// the only band intended to carve a specific narrow region.
BandState bass_     = { BandKind::LowShelf,   150.0f, 0.7f, 16384, true };
BandState mid_      = { BandKind::Peaking,   1000.0f, 0.7f, 16384, true };
BandState presence_ = { BandKind::Peaking,   4000.0f, 1.0f, 16384, true };
BandState treble_   = { BandKind::HighShelf, 8000.0f, 0.7f, 16384, true };

// RBJ Cookbook EQ. Same structure for every band kind; the b/a numerator
// and denominator polynomials differ. Branch is taken once per pot motion
// (rare), not per audio sample, so it costs nothing in the audio path.
void compute_band_coefficients(BandState & s, float gain_db) {
    if (gain_db > -kEqDeadDb && gain_db < kEqDeadDb) {
        s.bypass = true;
        return;
    }

    float A     = powf(10.0f, gain_db / 40.0f);
    float omega = 2.0f * kPi * s.freq_hz / (float) sample_rate_;
    float sin_o = sinf(omega);
    float cos_o = cosf(omega);
    float alpha = sin_o / (2.0f * s.Q);

    float a0;
    float b0, b1, b2, a1, a2;

    switch (s.kind) {
        case BandKind::Peaking: {
            a0 = 1.0f + alpha / A;
            b0 = 1.0f + alpha * A;
            b1 = -2.0f * cos_o;
            b2 = 1.0f - alpha * A;
            a1 = -2.0f * cos_o;
            a2 = 1.0f - alpha / A;
            break;
        }

        case BandKind::LowShelf: {
            float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;
            a0 =        (A + 1.0f) + (A - 1.0f) * cos_o + two_sqrtA_alpha;
            b0 =     A * ((A + 1.0f) - (A - 1.0f) * cos_o + two_sqrtA_alpha);
            b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_o);
            b2 =     A * ((A + 1.0f) - (A - 1.0f) * cos_o - two_sqrtA_alpha);
            a1 =-2.0f *     ((A - 1.0f) + (A + 1.0f) * cos_o);
            a2 =            (A + 1.0f) + (A - 1.0f) * cos_o - two_sqrtA_alpha;
            break;
        }

        case BandKind::HighShelf: {
            float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;
            a0 =            (A + 1.0f) - (A - 1.0f) * cos_o + two_sqrtA_alpha;
            b0 =      A *   ((A + 1.0f) + (A - 1.0f) * cos_o + two_sqrtA_alpha);
            b1 =-2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_o);
            b2 =      A *   ((A + 1.0f) + (A - 1.0f) * cos_o - two_sqrtA_alpha);
            a1 = 2.0f *     ((A - 1.0f) - (A + 1.0f) * cos_o);
            a2 =            (A + 1.0f) - (A - 1.0f) * cos_o - two_sqrtA_alpha;
            break;
        }
    }

    float a0_inv = 1.0f / a0;
    s.b0 = b0 * a0_inv;
    s.b1 = b1 * a0_inv;
    s.b2 = b2 * a0_inv;
    s.a1 = a1 * a0_inv;
    s.a2 = a2 * a0_inv;
    s.bypass = false;
}

// Direct Form I peaking/shelving biquad in float, applied to the
// normalised stereo buffer in place. Bypassed when the band is at 0 dB.
void apply_band(BandState & s, float * buf, uint16_t frames) {
    if (s.bypass) {
        return;
    }

    const float b0 = s.b0;
    const float b1 = s.b1;
    const float b2 = s.b2;
    const float a1 = s.a1;
    const float a2 = s.a2;

    float lx1 = s.lx1, lx2 = s.lx2;
    float ly1 = s.ly1, ly2 = s.ly2;
    float rx1 = s.rx1, rx2 = s.rx2;
    float ry1 = s.ry1, ry2 = s.ry2;

    for (uint16_t i = 0; i < frames; i++) {
        float l  = buf[i * 2];
        float lo = b0 * l + b1 * lx1 + b2 * lx2 - a1 * ly1 - a2 * ly2;
        lx2 = lx1; lx1 = l;
        ly2 = ly1; ly1 = lo;
        buf[i * 2] = lo;

        float r  = buf[i * 2 + 1];
        float ro = b0 * r + b1 * rx1 + b2 * rx2 - a1 * ry1 - a2 * ry2;
        rx2 = rx1; rx1 = r;
        ry2 = ry1; ry1 = ro;
        buf[i * 2 + 1] = ro;
    }

    s.lx1 = lx1; s.lx2 = lx2;
    s.ly1 = ly1; s.ly2 = ly2;
    s.rx1 = rx1; s.rx2 = rx2;
    s.ry1 = ry1; s.ry2 = ry2;
}

void clear_band_state(BandState & s) {
    s.lx1 = s.lx2 = s.ly1 = s.ly2 = 0.0f;
    s.rx1 = s.rx2 = s.ry1 = s.ry2 = 0.0f;
}

// ---------------------------------------------------------------------
// Drive: soft-knee 4:1 compressor with linked stereo envelope.
// ---------------------------------------------------------------------
// pot_q15 -> amount in [0, 1]. amount == 0 is hard bypass.
// threshold = -24 dB * amount, makeup = +12 dB * amount.
// Attack 5 ms, release 100 ms. Linked detector (max of L|R abs) so
// stereo content doesn't pull the image around.
//
// Gain reduction in linear domain via x^0.75 = sqrt(x) * sqrt(sqrt(x))
// -- two FPU sqrts beats a powf or log/exp pair on the M33.
constexpr float kCompKneeAmount = 0.02f;  // dead zone

struct CompressorState {
    // Defaults precomputed for 44.1 kHz so the env follower has sane
    // coefficients on cold start. dsp_set_sample_rate short-circuits when
    // fs already matches 44100, so without these baked-in defaults the
    // coefs would stay at zero, `env += (peak - env) * 0` would freeze env
    // at 0, and the compressor would silently never engage for the first
    // 44.1 kHz source. The limiter and VU follower bake their own defaults
    // for the same reason.
    int16_t pot_q15      = 0;
    bool    bypass       = true;
    float   threshold    = 1.0f;       // linear, 0..1
    float   makeup       = 1.0f;       // linear
    float   env          = 0.0f;       // linked detector
    float   attack_coef  = 0.0045247f; // 1 - exp(-1 / (44100 * 0.005))
    float   release_coef = 0.00022673f;// 1 - exp(-1 / (44100 * 0.100))
};

CompressorState comp_;

void recompute_compressor_time_coefs() {
    // 1 - exp(-1 / (fs * tau))
    float fs = (float) sample_rate_;
    comp_.attack_coef  = 1.0f - expf(-1.0f / (fs * 0.005f));
    comp_.release_coef = 1.0f - expf(-1.0f / (fs * 0.100f));
}

void compute_drive(CompressorState & s, int16_t q15) {
    s.pot_q15 = q15;

    if (q15 < 0) q15 = 0;
    float amount = (float) q15 / 32767.0f;

    if (amount < kCompKneeAmount) {
        s.bypass = true;
        s.env    = 0.0f;
        return;
    }

    float threshold_db = -24.0f * amount;
    float makeup_db    =  12.0f * amount;
    s.threshold = powf(10.0f, threshold_db / 20.0f);
    s.makeup    = powf(10.0f, makeup_db    / 20.0f);
    s.bypass    = false;
}

void apply_drive(CompressorState & s, float * buf, uint16_t frames) {
    if (s.bypass) {
        return;
    }

    const float threshold = s.threshold;
    const float makeup    = s.makeup;
    const float a_coef    = s.attack_coef;
    const float r_coef    = s.release_coef;
    float       env       = s.env;

    for (uint16_t i = 0; i < frames; i++) {
        float l = buf[i * 2];
        float r = buf[i * 2 + 1];

        float al = fabsf(l);
        float ar = fabsf(r);
        float peak = (al > ar) ? al : ar;

        float coef = (peak > env) ? a_coef : r_coef;
        env += (peak - env) * coef;

        float gain = 1.0f;

        if (env > threshold) {
            // gain = (threshold/env)^0.75 = sqrt(t/e) * sqrt(sqrt(t/e))
            float ratio = threshold / env;
            float s1    = sqrtf(ratio);
            gain = s1 * sqrtf(s1);
        }

        gain *= makeup;
        buf[i * 2]     = l * gain;
        buf[i * 2 + 1] = r * gain;
    }

    s.env = env;
}

// ---------------------------------------------------------------------
// Warmth: tanh-style saturator with small DC bias for 2nd-harmonic.
// ---------------------------------------------------------------------
// pot_q15 -> amount in [0, 1]. amount == 0 is hard bypass.
// drive in [1, 2], bias in [0, 0.1] (asymmetric clipping = even
// harmonics, "tube glow"). Output normalised so peak amplitude stays
// roughly constant across drive settings.
//
// Uses real tanhf so the output is properly bounded by +/-1 even when
// EQ boosts push peaks well above unity into the saturator. (A Pade
// approximation was tried first; it's faster but unbounded for large
// arguments, which clipped audibly at loud signals after the int16
// saturation step.)
constexpr float kWarmthKneeAmount = 0.02f;

struct WarmthState {
    int16_t pot_q15 = 0;
    bool    bypass  = true;
    float   drive   = 1.0f;
    float   bias    = 0.0f;
    float   dc      = 0.0f;   // tanh(drive * bias) -- subtracted to remove DC
    float   norm    = 1.0f;   // 1 / peak so output stays at +/-1
};

WarmthState warmth_;

void compute_warmth(WarmthState & s, int16_t q15) {
    s.pot_q15 = q15;

    if (q15 < 0) q15 = 0;
    float amount = (float) q15 / 32767.0f;

    if (amount < kWarmthKneeAmount) {
        s.bypass = true;
        return;
    }

    // Square the pot amount before mapping to drive/bias. tanh is most
    // sensitive near zero, so a linear knob feels top-heavy ("a tiny
    // bit gives a huge effect"). Squaring stretches the gentle region
    // across the bottom half of the pot's range and keeps the strong
    // saturation at the top.
    float curve = amount * amount;

    s.drive = 1.0f + 1.0f * curve;     // 1..2
    s.bias  = 0.1f * curve;             // 0..0.1
    s.dc    = tanhf(s.drive * s.bias);

    // peak output occurs at x=+1: tanh(drive*(1+bias)) - dc
    float peak = tanhf(s.drive * (1.0f + s.bias)) - s.dc;
    s.norm    = (peak > 1e-6f) ? (1.0f / peak) : 1.0f;
    s.bypass  = false;
}

void apply_warmth(WarmthState & s, float * buf, uint16_t frames) {
    if (s.bypass) {
        return;
    }

    const float drive = s.drive;
    const float bias  = s.bias;
    const float dc    = s.dc;
    const float norm  = s.norm;

    for (uint16_t i = 0; i < frames; i++) {
        float l = buf[i * 2];
        float r = buf[i * 2 + 1];
        buf[i * 2]     = (tanhf(drive * (l + bias)) - dc) * norm;
        buf[i * 2 + 1] = (tanhf(drive * (r + bias)) - dc) * norm;
    }
}

// ---------------------------------------------------------------------
// X-talk: Bauer/Linkwitz-style stereo crossfeed for headphone room sim.
// ---------------------------------------------------------------------
// Headphones present each channel to one ear only; speakers in a real
// room reach both ears with a delayed/low-passed contribution from the
// opposite channel (head-shadow attenuates highs more than lows). This
// stage approximates that head-shadow effect: a 1st-order LPF at
// ~700 Hz on each channel, mixed into the opposite channel at a level
// the user controls. Cures the "ping-pong stereo" headache on hard-
// panned mixes without any audible width loss on speakers.
//
// pot_q15 -> amount in [0, 1]. amount == 0 is hard bypass (no
// crossfeed, full discrete stereo). amount == 1 is roughly the
// classic Linkwitz mix (~50% LP'd opposite channel into each ear).
//
// Mix is normalised by 1/(1+mix) so total per-ear level is preserved.
constexpr float kXtalkKneeAmount = 0.02f;
constexpr float kXtalkLpFc       = 700.0f;     // head-shadow knee
constexpr float kXtalkMaxMix     = 0.5f;       // amount=1 -> 50% crossfeed

struct XtalkState {
    int16_t pot_q15  = 0;
    bool    bypass   = true;
    float   lp_alpha = 0.0f;
    float   direct   = 1.0f;   // 1 / (1 + mix)
    float   cross    = 0.0f;   // mix / (1 + mix)
    float   lp_l     = 0.0f;
    float   lp_r     = 0.0f;
};

XtalkState xtalk_;

void recompute_xtalk_filter(XtalkState & s) {
    if (s.bypass) {
        return;
    }

    constexpr float kPi2 = 2.0f * kPi;
    s.lp_alpha = 1.0f - expf(-kPi2 * kXtalkLpFc / (float) sample_rate_);
}

void compute_xtalk(XtalkState & s, int16_t q15) {
    s.pot_q15 = q15;

    if (q15 < 0) q15 = 0;
    float amount = (float) q15 / 32767.0f;

    if (amount < kXtalkKneeAmount) {
        s.bypass = true;
        s.lp_l   = 0.0f;
        s.lp_r   = 0.0f;
        return;
    }

    float mix = kXtalkMaxMix * amount;
    float inv = 1.0f / (1.0f + mix);
    s.direct = inv;
    s.cross  = mix * inv;
    s.bypass = false;
    recompute_xtalk_filter(s);
}

void apply_xtalk(XtalkState & s, float * buf, uint16_t frames) {
    if (s.bypass) {
        return;
    }

    const float a      = s.lp_alpha;
    const float direct = s.direct;
    const float cross  = s.cross;
    float       lp_l   = s.lp_l;
    float       lp_r   = s.lp_r;

    for (uint16_t i = 0; i < frames; i++) {
        float l = buf[i * 2];
        float r = buf[i * 2 + 1];

        // 1st-order LPF on each channel; we feed the LP'd signal into
        // the *opposite* channel as the crossfeed.
        lp_l += (l - lp_l) * a;
        lp_r += (r - lp_r) * a;

        buf[i * 2]     = direct * l + cross * lp_r;
        buf[i * 2 + 1] = direct * r + cross * lp_l;
    }

    s.lp_l = lp_l;
    s.lp_r = lp_r;
}

// ---------------------------------------------------------------------
// Soft limiter: end-of-chain safety net.
// ---------------------------------------------------------------------
// The chain ahead of this point can stack ~30 dB of cumulative gain
// (drive makeup +12 dB, four EQ bands at +12 dB each), so a track
// already near full scale played with bass + presence both pushed can
// land peaks several dB above +/-1.0. Without this stage those peaks
// would hard-clip at the int16 cast at the end of dsp_run -- audible
// crunch on the rare overshoot.
//
// Always-on (no pot, no bypass): it's safety, not colour. Engages only
// when the linked-stereo peak exceeds -1 dBFS; at typical levels with
// EQ near unity it sits idle (gain stays at 1.0).
//
// Algorithm: per-sample peak detection (max of |L|, |R|), gain ramps
// down with 1 ms attack when a peak exceeds threshold, recovers with
// 50 ms release. Sub-millisecond overshoots that slip through during
// the attack ramp are still caught by the int16 saturation behind us,
// so this is genuinely "soft" -- the brick-wall backstop is downstream.
constexpr float kLimiterThresholdDb = -1.0f;
constexpr float kLimiterAttackMs    = 1.0f;
constexpr float kLimiterReleaseMs   = 50.0f;

struct LimiterState {
    // Defaults precomputed for 44.1 kHz so the limiter is functional on
    // cold start. dsp_set_sample_rate short-circuits when fs matches the
    // default (44100), so without these the coefs would stay at zero and
    // the limiter would freeze its gain at 1.0 for 44.1 kHz sources.
    float threshold    = 0.8912509f;    // 10^(-1/20)
    float gain         = 1.0f;
    float attack_coef  = 0.0224025f;    // 1 - exp(-1 / (44100 * 0.001))
    float release_coef = 0.0004536f;    // 1 - exp(-1 / (44100 * 0.050))
};

LimiterState limiter_;

void recompute_limiter_time_coefs() {
    float fs = (float) sample_rate_;
    limiter_.attack_coef  = 1.0f - expf(-1.0f / (fs * kLimiterAttackMs  * 0.001f));
    limiter_.release_coef = 1.0f - expf(-1.0f / (fs * kLimiterReleaseMs * 0.001f));
    limiter_.threshold    = powf(10.0f, kLimiterThresholdDb / 20.0f);
}

// ---------------------------------------------------------------------
// VU meter envelope follower: per-channel mean-absolute-value with a
// ~300 ms time constant. Tapped post-volume / pre-limiter so the
// limiter's rare gain reduction doesn't make the needle bob.
//
// Defaults precomputed for 44.1 kHz so the follower has a sane alpha
// even when dsp_set_sample_rate's same-rate short-circuit prevents a
// recompute. fs change updates it via recompute_vu_alpha.
// ---------------------------------------------------------------------
constexpr float kVuTauSeconds = 0.3f;

float vu_env_l_  = 0.0f;
float vu_env_r_  = 0.0f;
float vu_alpha_  = 7.5550e-5f;     // 1 - exp(-1 / (44100 * 0.3))

void recompute_vu_alpha() {
    float fs   = (float) sample_rate_;
    vu_alpha_  = 1.0f - expf(-1.0f / (fs * kVuTauSeconds));
}

void apply_vu_tap(float * buf, uint16_t frames) {
    const float a = vu_alpha_;
    float       l = vu_env_l_;
    float       r = vu_env_r_;

    for (uint16_t i = 0; i < frames; i++) {
        l += (fabsf(buf[i * 2])     - l) * a;
        r += (fabsf(buf[i * 2 + 1]) - r) * a;
    }

    vu_env_l_ = l;
    vu_env_r_ = r;
}

void apply_limiter(LimiterState & s, float * buf, uint16_t frames) {
    const float threshold = s.threshold;
    const float a_coef    = s.attack_coef;
    const float r_coef    = s.release_coef;
    float       gain      = s.gain;

    for (uint16_t i = 0; i < frames; i++) {
        float l = buf[i * 2];
        float r = buf[i * 2 + 1];

        float al   = fabsf(l);
        float ar   = fabsf(r);
        float peak = (al > ar) ? al : ar;

        // Target gain: 1.0 when under threshold (no reduction); otherwise
        // exactly enough to bring this peak down to the threshold.
        float target_gain = (peak > threshold) ? (threshold / peak) : 1.0f;

        // Gain falls fast on rising peaks (attack), recovers slowly on
        // quieter content (release). Asymmetric coefficients are what
        // make this sound smooth rather than zipper-y.
        float coef = (target_gain < gain) ? a_coef : r_coef;
        gain += (target_gain - gain) * coef;

        buf[i * 2]     = l * gain;
        buf[i * 2 + 1] = r * gain;
    }

    s.gain = gain;
}

void set_band_from_pot(BandState & s, int16_t q15) {
    if (q15 < 0) {
        q15 = 0;
    } else if (q15 > 32767) {
        q15 = 32767;
    }

    s.pot_q15 = q15;

    float gain_db = ((float) q15 - 16383.5f) / 16383.5f * kEqMaxDb;
    compute_band_coefficients(s, gain_db);
}

void recompute_band_for_current_pot(BandState & s) {
    float gain_db = ((float) s.pot_q15 - 16383.5f) / 16383.5f * kEqMaxDb;
    compute_band_coefficients(s, gain_db);
}

}  // namespace

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

void dsp_run(int16_t * buffer, uint16_t frames) {
    if (frames > kDspMaxFrames) {
        frames = (uint16_t) kDspMaxFrames;
    }

    // Promote int16 -> normalised float (range +/-1.0).
    for (uint16_t i = 0; i < frames; i++) {
        dsp_buf_[i * 2]     = (float) buffer[i * 2]     * kInt16ToFloat;
        dsp_buf_[i * 2 + 1] = (float) buffer[i * 2 + 1] * kInt16ToFloat;
    }

    // Pre-DSP visualisation tap: env follower runs on the raw decoded
    // audio (before any on-board colour / EQ / volume) when the tap
    // selector is in "pre" mode. The audio path itself is unaffected.
    if (visualisation_pre_) {
        apply_vu_tap(dsp_buf_, frames);
    }

    // Chain: colour first (drive/warmth) so the saturator sees raw audio
    // without EQ-boosted peaks; x-talk applies headphone room simulation
    // to the coloured signal; EQ runs last so the user can shape the
    // post-colour, post-crossfeed signal. Front-panel bypass toggles
    // gate each block as a whole.
    if (!colour_bypass_) {
        apply_drive(comp_,    dsp_buf_, frames);
        apply_warmth(warmth_, dsp_buf_, frames);
        apply_xtalk(xtalk_,   dsp_buf_, frames);
    }

    if (!tone_bypass_) {
        apply_band(bass_,     dsp_buf_, frames);
        apply_band(mid_,      dsp_buf_, frames);
        apply_band(presence_, dsp_buf_, frames);
        apply_band(treble_,   dsp_buf_, frames);
    }

    apply_volume(dsp_buf_, frames);

    // Post-DSP visualisation tap: env follower sees the post-volume,
    // pre-limiter signal so the needle reflects the music's level,
    // not the limiter's response to stacked-EQ overshoots.
    if (!visualisation_pre_) {
        apply_vu_tap(dsp_buf_, frames);
    }

    // Soft limiter just before the int16 sat: catches the rare overshoot
    // from stacked drive + EQ gain so the saturation step below isn't
    // doing the limiting via hard clipping.
    apply_limiter(limiter_, dsp_buf_, frames);

    // Saturate float -> int16 in place.
    for (uint16_t i = 0; i < frames; i++) {
        float lf = dsp_buf_[i * 2]     * kFloatToInt16;
        float rf = dsp_buf_[i * 2 + 1] * kFloatToInt16;

        if (lf >  32767.0f) lf =  32767.0f;
        if (lf < -32768.0f) lf = -32768.0f;
        if (rf >  32767.0f) rf =  32767.0f;
        if (rf < -32768.0f) rf = -32768.0f;

        buffer[i * 2]     = (int16_t) lf;
        buffer[i * 2 + 1] = (int16_t) rf;
    }
}

void dsp_set_sample_rate(uint32_t fs) {
    if (fs == 0 || fs == sample_rate_) {
        return;
    }

    sample_rate_ = fs;

    clear_band_state(bass_);
    clear_band_state(mid_);
    clear_band_state(presence_);
    clear_band_state(treble_);

    recompute_band_for_current_pot(bass_);
    recompute_band_for_current_pot(mid_);
    recompute_band_for_current_pot(presence_);
    recompute_band_for_current_pot(treble_);

    // Drive's attack/release time constants, X-talk's LPF alpha,
    // the limiter's attack/release coefs, and the VU env-follower's
    // alpha all depend on Fs. Warmth's saturator is sample-rate-
    // independent.
    recompute_compressor_time_coefs();
    recompute_xtalk_filter(xtalk_);
    recompute_limiter_time_coefs();
    recompute_vu_alpha();
}

void dsp_set_avrcp_volume_127(uint8_t v) {
    if (v >= 127) {
        avrcp_vol_q15_ = 32767;
    } else {
        avrcp_vol_q15_ = (int16_t) (((int32_t) v * 32768 + 63) / 127);
    }

    recompute_master_volume();
}

void dsp_set_pot_volume_q15(int16_t v) {
    if (v < 0) {
        v = 0;
    }

    pot_vol_q15_ = v;
    recompute_master_volume();
}

void dsp_set_bass_q15(int16_t q15)     { set_band_from_pot(bass_,     q15); }
void dsp_set_mid_q15(int16_t q15)      { set_band_from_pot(mid_,      q15); }
void dsp_set_presence_q15(int16_t q15) { set_band_from_pot(presence_, q15); }
void dsp_set_treble_q15(int16_t q15)   { set_band_from_pot(treble_,   q15); }

void dsp_set_drive_q15(int16_t q15)    { compute_drive(comp_,    q15); }
void dsp_set_warmth_q15(int16_t q15)   { compute_warmth(warmth_, q15); }
void dsp_set_xtalk_q15(int16_t q15)    { compute_xtalk(xtalk_,   q15); }

void dsp_set_tone_bypass(bool bypass)   { tone_bypass_   = bypass; }
void dsp_set_colour_bypass(bool bypass) { colour_bypass_ = bypass; }

float dsp_get_vu_env_l() { return vu_env_l_; }
float dsp_get_vu_env_r() { return vu_env_r_; }

void dsp_set_visualisation_pre(bool pre) { visualisation_pre_ = pre; }
bool dsp_get_visualisation_pre()         { return visualisation_pre_; }
