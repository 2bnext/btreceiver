// pots.cpp -- ADS1115 driver + pot polling.
//
// Two ADS1115s share the I2C bus (#1 = 0x48 EQ pots, #2 = 0x49 colour
// + volume). Each ADS1115 has 4 analog inputs but only one ADC, so we
// scan via the chip's MUX in single-shot mode at 860 SPS (~1.16 ms per
// conversion -- fast enough to read both chips on every pot tick).
//
// Channel allocation (matches the analog control surface):
//   #1 0x48: A0 = bass     A1 = mid       A2 = treble   A3 = presence
//   #2 0x49: A0 = drive    A1 = warmth    A2 = x-talk   A3 = volume
//
// EMA smoothing per channel rejects the bottom couple of LSBs of noise;
// any meaningful pot motion triggers an OLED popup with the band's
// label and a horizontal bar.

#include "pots.hpp"

#include <limits.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "display.hpp"  // for `Display display` extern + show_pot()
#include "dsp.hpp"

namespace {

constexpr i2c_inst_t * kI2cPort  = i2c0;
constexpr uint8_t      kAddrEq   = 0x48;  // EQ pots (#1)
constexpr uint8_t      kAddrVol  = 0x49;  // drive/warmth/x-talk/volume (#2)

constexpr uint8_t      kRegConv  = 0x00;
constexpr uint8_t      kRegCfg   = 0x01;

// ADS1115 single-shot config skeleton:
//   OS=1 (start conversion), MUX=set per call, PGA=001 (+/-4.096V),
//   MODE=1 (single-shot), DR=111 (860 SPS, ~1.16 ms), comparator off.
//
// Layout:  bit15  14:12  11:9   8     7:5     4:0
//          OS     MUX    PGA    MODE  DR      COMP
//          1      vary   001    1     111     00011
//
// kCfgBase has MUX=000; we OR ((4 + ch) << 12) to set MUX = 100..111
// for AIN0..AIN3 single-ended.
constexpr uint16_t     kCfgBase  = 0b1000'0011'1110'0011;  // 0x83E3

// Pot wired GND..3V3 -> ADC range [0, ~26395]. Top of the range maps
// to Q15 max with a small deadband at the very top so the pot has a
// firm "10/10" stop.
constexpr int32_t      kPotFullScale = 25000;

// Significant-change threshold for the OLED popup (~0.4% of full scale).
constexpr int32_t      kPotPopupDelta = 128;

// 860 SPS -> ~1.16 ms per conversion. Sleep a bit longer to leave
// margin for the trigger I2C write itself.
constexpr uint32_t     kConversionUs = 1500;

// Per-pot smoothing + last-displayed state. EMA in Q24 (8 sub-LSB
// bits) so the filter actually moves on tiny pot motions; alpha = 1/4
// gives ~5-sample / 250 ms settling at 20 Hz polling.
struct PotState {
    int32_t ema_q24        = 0;
    bool    ema_init       = false;
    int32_t last_announced = -1;
};

// ---------------------------------------------------------------------
// ADS1115 driver
// ---------------------------------------------------------------------

// Kick off a single-shot conversion on `addr` for AIN`channel`. Does
// NOT wait for the result -- callers fire one chip, fire the other,
// then sleep once for both to convert in parallel before reading
// back. That parallelism is the whole reason the per-channel sample
// rate is fast enough to feel responsive.
void ads1115_trigger(uint8_t addr, uint8_t channel) {
    uint16_t cfg = kCfgBase | ((uint16_t) (4 + (channel & 3)) << 12);

    const uint8_t cfg_msg[3] = {
        kRegCfg,
        (uint8_t) (cfg >> 8),
        (uint8_t) (cfg & 0xFF),
    };

    i2c_write_blocking(kI2cPort, addr, cfg_msg, 3, false);
}

// Read the most recent conversion result. Caller must have triggered
// the conversion and waited >= kConversionUs first. Returns INT32_MIN
// on I2C error so the caller can keep its previous value.
int32_t ads1115_read_result(uint8_t addr) {
    const uint8_t ptr_msg = kRegConv;

    if (i2c_write_blocking(kI2cPort, addr, &ptr_msg, 1, false) != 1) {
        return INT32_MIN;
    }

    uint8_t rx[2];

    if (i2c_read_blocking(kI2cPort, addr, rx, 2, false) != 2) {
        return INT32_MIN;
    }

    int16_t raw = (int16_t) ((rx[0] << 8) | rx[1]);
    return (raw < 0) ? 0 : (int32_t) raw;
}

// ---------------------------------------------------------------------
// Pot processing
// ---------------------------------------------------------------------

// Advance the EMA, return the smoothed Q15 value mapped from the ADC
// range. INT32_MIN means I2C error; caller should bail.
int32_t smooth_to_q15(int32_t raw, PotState & state) {
    if (raw == INT32_MIN) {
        return INT32_MIN;
    }

    if (!state.ema_init) {
        state.ema_q24  = raw << 8;
        state.ema_init = true;
    } else {
        state.ema_q24 += ((raw << 8) - state.ema_q24) >> 2;
    }

    int32_t smoothed = state.ema_q24 >> 8;
    int32_t q15      = (smoothed * 32767) / kPotFullScale;

    if (q15 < 0) {
        q15 = 0;
    } else if (q15 > 32767) {
        q15 = 32767;
    }

    return q15;
}

// Trigger an OLED popup on the first reading and on any subsequent
// change >= kPotPopupDelta.
void announce(PotState & state, const char * label, int16_t q15) {
    if (state.last_announced < 0) {
        state.last_announced = q15;
        display.show_pot(label, q15);
        return;
    }

    int32_t diff = (int32_t) q15 - state.last_announced;
    if (diff < 0) diff = -diff;

    if (diff > kPotPopupDelta) {
        display.show_pot(label, q15);
        state.last_announced = q15;
    }
}

// ---------------------------------------------------------------------
// Pot table
// ---------------------------------------------------------------------
// One entry per (chip, channel). The dispatch function maps the
// smoothed Q15 reading to the right DSP setter (or no-op for the
// drive/warmth/x-talk channels.

enum class PotId : uint8_t {
    Bass     = 0,  // 0x48 A0
    Mid      = 1,  // 0x48 A1
    Treble   = 2,  // 0x48 A2
    Presence = 3,  // 0x48 A3
    Drive    = 4,  // 0x49 A0
    Warmth   = 5,  // 0x49 A1
    Xtalk    = 6,  // 0x49 A2
    Volume   = 7,  // 0x49 A3
    Count    = 8,
};

constexpr size_t kNumPots = (size_t) PotId::Count;

PotState pot_state_[kNumPots];

const char * pot_label(PotId id) {
    switch (id) {
        case PotId::Bass:     return "BAS";
        case PotId::Mid:      return "MID";
        case PotId::Treble:   return "TRE";
        case PotId::Presence: return "PRS";
        case PotId::Drive:    return "DRV";
        case PotId::Warmth:   return "WRM";
        case PotId::Xtalk:    return "XTL";
        case PotId::Volume:   return "VOL";
        default:              return "?";
    }
}

// Push the smoothed Q15 to the appropriate DSP setter. Volume gets the
// audio-taper square-law curve here so dsp_set_pot_volume_q15 stays
// dumb -- everything else passes through unchanged.
int16_t dispatch_to_dsp(PotId id, int16_t q15) {
    switch (id) {
        case PotId::Bass:     dsp_set_bass_q15(q15);     return q15;
        case PotId::Mid:      dsp_set_mid_q15(q15);      return q15;
        case PotId::Treble:   dsp_set_treble_q15(q15);   return q15;
        case PotId::Presence: dsp_set_presence_q15(q15); return q15;

        case PotId::Volume: {
            // Square-law audio taper: a *linear* pot feels logarithmic
            // to the ear (half rotation -> -12 dB). Skip the squaring
            // at max so passthrough stays exactly 32767 (bit-perfect).
            int16_t curved = q15;

            if (q15 < 32767) {
                curved = (int16_t) (((int32_t) q15 * q15) >> 15);
            }

            dsp_set_pot_volume_q15(curved);

            // Push the curved value to the OLED's persistent top-bar
            // default, so when no pot popup is showing the bar still
            // reflects the live volume position.
            display.set_volume_q15(curved);
            return curved;
        }

        case PotId::Drive:    dsp_set_drive_q15(q15);    return q15;
        case PotId::Warmth:   dsp_set_warmth_q15(q15);   return q15;
        case PotId::Xtalk:    dsp_set_xtalk_q15(q15);    return q15;

        default:
            return q15;
    }
}

// Smooth a raw ADC reading, dispatch to DSP, fire popup if it moved.
void process_pot(PotId id, int32_t raw) {
    PotState & state = pot_state_[(size_t) id];

    int32_t q15 = smooth_to_q15(raw, state);

    if (q15 == INT32_MIN) {
        return;
    }

    int16_t announced_q15 = dispatch_to_dsp(id, (int16_t) q15);
    announce(state, pot_label(id), announced_q15);
}

// Per-channel pot identity for each chip. Indexed by ADS1115 AIN
// channel (0..3). pots on chip #1 are EQ; chip #2 are colour + volume.
constexpr PotId kEqPots [4] = { PotId::Bass,  PotId::Mid,    PotId::Treble, PotId::Presence };
constexpr PotId kVolPots[4] = { PotId::Drive, PotId::Warmth, PotId::Xtalk,  PotId::Volume   };

// Scan one channel from BOTH chips concurrently: trigger conversions
// on 0x48 and 0x49, sleep once for them to finish in parallel, then
// read both results back. 4 channels per chip x 1 wait of 1500 us =
// ~6.8 ms per full sweep, run every pot tick (50 ms) so each pot is
// sampled at 20 Hz instead of the round-robin 5 Hz this used to be.
void scan_channel_pair(uint8_t channel) {
    ads1115_trigger(kAddrEq,  channel);
    ads1115_trigger(kAddrVol, channel);
    sleep_us(kConversionUs);

    int32_t raw_eq  = ads1115_read_result(kAddrEq);
    int32_t raw_vol = ads1115_read_result(kAddrVol);

    process_pot(kEqPots [channel], raw_eq);
    process_pot(kVolPots[channel], raw_vol);
}

void scan_all_channels() {
    for (uint8_t ch = 0; ch < 4; ch++) {
        scan_channel_pair(ch);
    }
}

}  // namespace

void pots_init() {
    // Take a full sweep at boot so the audio path starts at the wiper-
    // correct values for every band and the master volume. Single-shot
    // mode needs no up-front configuration -- ads1115_trigger() writes
    // a fresh config each time.
    scan_all_channels();
}

void pots_poll_tick() {
    scan_all_channels();
}
