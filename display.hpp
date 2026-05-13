// display.hpp -- SSD1306 128x64 OLED driver for btspkr.
//
// Single-header, header-only C++ implementation. Owns its own framebuffer
// and embeds a 5x7 ASCII font + 7x7 status icons.
//
// Layout (single permanent screen, no mode cycling):
//
//   Page 0 (rows 0..7) -- top status strip:
//     cols 0..6   : power icon         (always solid)
//     cols 8..14  : Bluetooth icon     (drawn only while AVRCP connected)
//     cols 22..39 : 3-char pot label   ("VOL" by default; takes over with
//                                       any moving pot for ~1.5 s, then
//                                       reverts to volume)
//     cols 44..124: framed horizontal bar (q15 fill, 1 px frame so the
//                                          empty position stays visible)
//     cols 125..127: right margin
//
//   Row 8 -- 1 px breathing margin (intentionally blank).
//
//   Rows 9..63 -- mirrored stereo FFT spectrum:
//     16 bands per channel, 4 px slot (3 px bar + 1 px gap).
//     L on cols 0..62 with low frequencies near the centre and highs
//     reaching the left edge; R on cols 65..127 with low frequencies
//     near the centre and highs reaching the right edge. Cols 63..64
//     are a deliberate 2 px blank centre gap (no separator line).
//     Bars grow upward from row 63, capped at 55 px so they can't
//     bleed into the top strip.
//
// Everything redraws every tick at ~30 Hz; there is no dirty-page
// tracking. AVRCP track metadata is not currently rendered -- bt.cpp
// still prints it to the serial console for debugging.
//
// I2C: assumes the bus has already been initialised (we share GPIO 8/9 with
// the ADS1115 pot ADCs). Display lives at 0x3C.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hardware/i2c.h"
#include "pico/time.h"  // sleep_ms for the SSD1306 cold-boot reset window

#include "spectrum.hpp"

class Display {
public:
    static constexpr uint8_t kI2cAddr      = 0x3C;
    static constexpr int     kWidth        = 128;
    static constexpr int     kHeight       = 64;
    static constexpr int     kPages        = kHeight / 8;       // 8 horizontal stripes
    static constexpr int     kCharW        = 6;                  // 5 px glyph + 1 px gap

    // Top status bar -- page 0, drawn every tick.
    static constexpr int     kBarPage          = 0;             // rows 0..7
    static constexpr int     kBarPowerIconX    = 0;             // 7 cols wide
    static constexpr int     kBarBtIconX       = 8;             // 7 cols wide, 1 px gap

    // 4 toggle-status letter indicators (T / C / V / P), drawn only
    // when their switch is active. Standard 6-px char pitch from the
    // 5x7 font puts them at 16, 22, 28, 34 -- ending at col 38.
    static constexpr int     kBarToggleX0      = 16;
    static constexpr int     kBarToggleStride  = kCharW;        // 6
    static constexpr int     kBarToggleEnd     = kBarToggleX0 + 4 * kBarToggleStride;  // 40

    static constexpr int     kBarLabelX        = kBarToggleEnd + 2;  // 42, 3-char pot label
    static constexpr int     kBarLabelChars    = 3;
    static constexpr int     kBarBoxLeft       = kBarLabelX + kBarLabelChars * kCharW + 4;  // 64
    static constexpr int     kBarBoxRight      = kWidth - 4;    // 124
    static constexpr int     kBarBoxTopRow     = 0;
    static constexpr int     kBarBoxBotRow     = 6;             // 7 px tall framed bar

    // Spectrum lives below the bar with a 1 px breathing margin: page 0
    // is rows 0..7 (the header), row 8 is the deliberate gap, and the
    // spectrum starts at row 9. That leaves 55 px for the bars.
    static constexpr int     kSpectrumTopRow   = 9;
    static constexpr int     kSpectrumMaxBars  = kHeight - kSpectrumTopRow;  // 55

    // Pot-value popup: how long the moving pot's label/value owns the top
    // bar after the last show_pot() call before reverting to volume, and
    // the longest label we keep room for.
    static constexpr uint32_t kPotPopupTicks   = 45;  // ~1.5 s at 33 ms tick
    static constexpr size_t   kPotPopupNameMax = 8;

    explicit Display(i2c_inst_t * i2c) : i2c_(i2c) {}

    // Send the SSD1306 init sequence and clear the screen.
    // Call after i2c_init() in main().
    void init() {
        // Cold-boot guard. The SSD1306's internal POR can race with the
        // MCU coming out of reset on a slow-ramping external supply: if
        // we start sending init commands before the controller has
        // finished its own reset, they land on a half-initialised state
        // machine and the panel stays dark. Symptom: power-off-on fails,
        // but a reflash with power maintained works (because the OLED
        // is already up and stable). 100 ms covers the datasheet's VCC
        // stabilisation window. Then send 0xAE three times: if the
        // controller was mid multi-byte command at boot the first one
        // or two get eaten as data, the third is guaranteed a fresh
        // "display off" that flushes the parser to a known state.
        sleep_ms(100);
        send_cmd(0xAE);
        send_cmd(0xAE);
        send_cmd(0xAE);

        // Standard 128x64 init sequence, broken out so it's easy to read.
        // Each command is sent as a separate I2C transaction with control
        // byte 0x00 ("next byte is a command"). Could be batched into one
        // long stream with control byte 0x00 prefix, but per-command sends
        // are small and run once at boot -- not worth the complexity.
        static const uint8_t kInit[] = {
            0xAE,           // display off
            0xD5, 0x80,     // clock divide / oscillator
            0xA8, 0x3F,     // multiplex ratio = 64
            0xD3, 0x00,     // display offset = 0
            0x40,           // start line = 0
            0x8D, 0x14,     // charge pump on
            0x20, 0x00,     // memory mode = horizontal addressing
            0xA1,           // segment remap (column 127 mapped to SEG0)
            0xC8,           // COM scan direction reversed
            0xDA, 0x12,     // COM pins hardware config (alternative, no remap)
            0x81, 0xCF,     // contrast
            0xD9, 0xF1,     // pre-charge period
            0xDB, 0x40,     // VCOMH deselect level
            0xA4,           // resume from RAM (vs. all-pixels-on)
            0xA6,           // normal display (not inverted)
            0x2E,           // deactivate scrolling
            0xAF,           // display on
        };

        for (uint8_t c : kInit) {
            send_cmd(c);
        }

        spectrum_l_.init();
        spectrum_r_.init();

        // Single full-screen wipe so the panel doesn't briefly show
        // VRAM garbage before the first tick lands.
        clear_buffer();
        flush_all();
    }

    // Stereo sample sources for the spectrum view.
    void set_spectrum_source(const int16_t * left, const int16_t * right,
                             size_t count, const size_t * read_pos) {
        spectrum_l_samples_ = left;
        spectrum_r_samples_ = right;
        spectrum_count_     = count;
        spectrum_read_pos_  = read_pos;
    }

    // A pot moved -- take over the top bar with this label/value for
    // ~1.5 s, then auto-revert to volume. Calling again restarts the
    // timer, so a continuous turn keeps the label on screen.
    void show_pot(const char * name, int16_t q15) {
        size_t n = strlen(name);

        if (n > kPotPopupNameMax - 1) {
            n = kPotPopupNameMax - 1;
        }

        memcpy(pot_popup_name_, name, n);
        pot_popup_name_[n]    = 0;
        pot_popup_q15_        = q15;
        pot_popup_ticks_left_ = kPotPopupTicks;
    }

    // Persistent "default" value for the top bar, shown whenever no pot
    // popup is active. Plumbed in from pots.cpp's volume dispatch case so
    // the bar always reflects the current master volume.
    void set_volume_q15(int16_t q15) {
        volume_q15_ = q15;
    }

    // Connection state for the BT icon. render_top_bar draws the icon
    // only when this is true; the slot stays blank until a phone pairs.
    void set_bt_connected(bool connected) {
        bt_connected_ = connected;
    }

    // Front-panel toggle states, rendered as four letter indicators
    // (T / C / V / P) in the top bar -- each only drawn while its
    // switch is active. Set from toggles_poll in btspkr.cpp.
    void set_toggle_states(bool tone_bypass, bool colour_bypass,
                           bool vol_policy_max, bool vis_pre) {
        tone_bypass_on_    = tone_bypass;
        colour_bypass_on_  = colour_bypass;
        vol_policy_max_on_ = vol_policy_max;
        vis_pre_on_        = vis_pre;
    }

    // Periodic tick (~30 Hz). Pull fresh FFT data, redraw the whole
    // panel, push it out. No dirty-page tracking: at 30 Hz the full
    // 1 KB flush is well inside the audio backend's buffer reserve.
    void tick() {
        if (pot_popup_ticks_left_ > 0) {
            pot_popup_ticks_left_--;
        }

        if (spectrum_l_samples_ && spectrum_r_samples_ && spectrum_read_pos_) {
            spectrum_l_.update(spectrum_l_samples_, spectrum_count_, *spectrum_read_pos_);
            spectrum_r_.update(spectrum_r_samples_, spectrum_count_, *spectrum_read_pos_);
        }

        clear_buffer();
        render_top_bar();
        render_spectrum();
        flush_all();
    }

private:
    // I2C control bytes for the SSD1306 protocol.
    static constexpr uint8_t kCtrlCmd  = 0x00;
    static constexpr uint8_t kCtrlData = 0x40;

    void clear_buffer() {
        memset(framebuffer_, 0, sizeof(framebuffer_));
    }

    // Top status strip (page 0):
    //   power icon | [BT icon if connected] | label | framed bar
    //
    // Power icon is always drawn (the device is on). The BT icon is
    // drawn only when bt_connected_ is true; until a phone pairs, the
    // BT slot stays blank. Label + bar default to volume; if a pot is
    // being turned the popup state takes over for kPotPopupTicks then
    // reverts.
    void render_top_bar() {
        // 7x7 power icon (circle + power-button stroke at the top).
        static constexpr uint8_t kPower[7] = {
            0x1C,  // col 0: rows 2,3,4    -- left side of circle
            0x22,  // col 1: rows 1,5
            0x40,  // col 2: row  6        -- circle bottom
            0x43,  // col 3: rows 0,1,6    -- power stroke + circle bottom
            0x40,  // col 4: row  6
            0x22,  // col 5: rows 1,5
            0x1C,  // col 6: rows 2,3,4    -- right side of circle
        };
        // 7x7 Bluetooth runic-B (spine + bowtie tips at rows 2 and 4,
        // left waist at row 3).
        static constexpr uint8_t kBluetooth[7] = {
            0x00,
            0x08,
            0x08,
            0x7F,
            0x36,
            0x14,
            0x00,
        };

        for (int i = 0; i < 7; i++) {
            framebuffer_[kBarPage * kWidth + kBarPowerIconX + i] = kPower[i];
        }

        if (bt_connected_) {
            for (int i = 0; i < 7; i++) {
                framebuffer_[kBarPage * kWidth + kBarBtIconX + i] = kBluetooth[i];
            }
        }

        // Toggle-status indicators: T, C, V, P. Each appears only while
        // its switch is active so the default look stays clean. Same
        // page as the rest of the strip; uses the standard 5x7 font.
        if (tone_bypass_on_) {
            draw_char(kBarToggleX0 + 0 * kBarToggleStride, kBarPage, 'T');
        }

        if (colour_bypass_on_) {
            draw_char(kBarToggleX0 + 1 * kBarToggleStride, kBarPage, 'C');
        }

        if (vol_policy_max_on_) {
            draw_char(kBarToggleX0 + 2 * kBarToggleStride, kBarPage, 'V');
        }

        if (vis_pre_on_) {
            draw_char(kBarToggleX0 + 3 * kBarToggleStride, kBarPage, 'P');
        }

        // Pick what to show: pot popup if the timer's still running,
        // otherwise the master volume. Falling through to volume after
        // a popup expires is what makes the bar "always visible" --
        // there's never a frame with nothing on the right side.
        const char * label;
        int16_t      q15;

        if (pot_popup_ticks_left_ > 0) {
            label = pot_popup_name_;
            q15   = pot_popup_q15_;
        } else {
            label = "VOL";
            q15   = volume_q15_;
        }

        // 3-char label, left-aligned in its slot. Pad short labels
        // with spaces so the previous label's tail doesn't bleed
        // through (the framebuffer was cleared, but anything we don't
        // overwrite stays black -- which is fine; padding here keeps
        // the spacing predictable if the label width changes later).
        char buf[kBarLabelChars + 1];
        int  i = 0;

        while (i < kBarLabelChars && label[i] != 0) {
            buf[i] = label[i];
            i++;
        }
        while (i < kBarLabelChars) {
            buf[i++] = ' ';
        }
        buf[kBarLabelChars] = 0;
        draw_text(kBarLabelX, kBarPage, buf);

        // 1-px frame around the bar so the empty position is still
        // visible (a fully-empty progress bar with no border vanishes,
        // which looks broken). Frame is rows 0 (top) and 6 (bottom),
        // and 1-col verticals at the left/right edges.

        // Top + bottom edges: a row of pixels along the inside top
        // and bottom of the bar. Encoded as bit 0 (row 0) and bit 6
        // (row 6) of each column byte in page 0.
        for (int x = kBarBoxLeft; x <= kBarBoxRight; x++) {
            framebuffer_[kBarPage * kWidth + x] |=
                (uint8_t) ((1u << kBarBoxTopRow) | (1u << kBarBoxBotRow));
        }

        // Left + right edges: full-height stubs across rows 0..6
        // (mask 0x7F covers rows 0..6 in this page byte).
        constexpr uint8_t kFrameMask = 0x7F;
        framebuffer_[kBarPage * kWidth + kBarBoxLeft]  |= kFrameMask;
        framebuffer_[kBarPage * kWidth + kBarBoxRight] |= kFrameMask;

        // Inner fill: rows 2..5 (4 px tall solid bar), grows from the
        // left edge of the inner box (col kBarBoxLeft + 1) toward the
        // right edge (col kBarBoxRight - 1).
        const int inner_left  = kBarBoxLeft  + 1;
        const int inner_right = kBarBoxRight - 1;
        const int inner_w     = inner_right - inner_left + 1;
        const int filled      = clamp_filled(q15, inner_w);

        constexpr uint8_t kFillMask = 0x3C;  // rows 2..5

        for (int x = 0; x < filled; x++) {
            framebuffer_[kBarPage * kWidth + inner_left + x] |= kFillMask;
        }
    }

    // Stereo FFT bars + peak holds, clipped to the spectrum area
    // (rows kSpectrumTopRow..kHeight-1). 16 bands per side, 4 px slot
    // (3 px bar + 1 px gap). The two halves are *mirrored* around the
    // centre 2 px gap (cols 63..64): L runs cols 0..62 with low
    // frequencies on the inside (near the gap) growing toward high on
    // the outside; R runs cols 65..127 with low frequencies on the
    // inside and high on the outside. Bass thump pulses out from the
    // centre toward both edges -- visually symmetric, intuitive at a
    // glance.
    void render_spectrum() {
        constexpr int kSlot    = 4;
        constexpr int kBarW    = 3;
        constexpr int kRightX0 = 65;

        for (size_t b = 0; b < Spectrum::kBands; b++) {
            // L mirrors: low band (b == 0) draws nearest the gap (cols
            // 60..62), high band (b == kBands-1) draws at the far left
            // (cols 0..2).
            int x_l = (int) (Spectrum::kBands - 1 - b) * kSlot;
            int x_r = kRightX0 + (int) b * kSlot;
            draw_band(x_l, kBarW, spectrum_l_.band(b), spectrum_l_.peak(b));
            draw_band(x_r, kBarW, spectrum_r_.band(b), spectrum_r_.peak(b));
        }
    }

    // Draw a single band's body + peak-hold pixel into the framebuffer
    // at columns x0..x0+w-1. Heights are clipped to the spectrum area
    // (rows kSpectrumTopRow..kHeight-1) so tall peaks never bleed into
    // the top status strip.
    void draw_band(int x0, int w, int height, int peak) {
        if (height > kSpectrumMaxBars) {
            height = kSpectrumMaxBars;
        }

        if (height > 0) {
            int top_row = kHeight - height;
            int bot_row = kHeight - 1;

            for (int r = top_row; r <= bot_row; r++) {
                int page = r >> 3;
                int bit  = r & 7;

                for (int x = x0; x < x0 + w; x++) {
                    framebuffer_[page * kWidth + x] |= (uint8_t) (1u << bit);
                }
            }
        }

        if (peak > 0) {
            int row = kHeight - peak;

            if (row < kSpectrumTopRow) {
                row = kSpectrumTopRow;
            }

            int page = row >> 3;
            int bit  = row & 7;

            for (int x = x0; x < x0 + w; x++) {
                framebuffer_[page * kWidth + x] |= (uint8_t) (1u << bit);
            }
        }
    }

    static int clamp_filled(int16_t q15, int max) {
        if (q15 < 0) {
            return 0;
        }

        int filled = (int) (((int32_t) q15 * max) / 32767);

        if (filled < 0) {
            return 0;
        }

        if (filled > max) {
            return max;
        }

        return filled;
    }

    // Draw plain text starting at (col, page). max_chars caps how many
    // glyphs we'll lay down -- defaults to "as many as fit on screen".
    void draw_text(int col, int page, const char * s, int max_chars = -1) {
        int x = col;
        int n = 0;

        while (*s && x + kCharW <= kWidth && (max_chars < 0 || n < max_chars)) {
            draw_char(x, page, *s++);
            x += kCharW;
            n++;
        }
    }

    // Plot one 5x7 glyph into the framebuffer at (col, page).
    // Each font byte is one column (LSB at top).
    void draw_char(int col, int page, char c) {
        if (c < 32 || c > 126) {
            c = '?';
        }

        const uint8_t * glyph = &kFont[(c - 32) * 5];

        for (int i = 0; i < 5; i++) {
            framebuffer_[page * kWidth + col + i] = glyph[i];
        }

        // 6th column is the inter-character gap; explicitly zeroed because
        // we don't memset between draws.
        framebuffer_[page * kWidth + col + 5] = 0;
    }

    void send_cmd(uint8_t c) {
        uint8_t buf[2] = { kCtrlCmd, c };
        i2c_write_blocking(i2c_, kI2cAddr, buf, 2, false);
    }

    // Push the entire framebuffer to the panel, page by page (each
    // page is one I2C transaction of ~130 bytes). At 30 Hz the eight
    // back-to-back transactions take ~10 ms blocking, which sits well
    // inside the audio backend's ~35 ms buffer reserve.
    void flush_all() {
        for (int page = 0; page < kPages; page++) {
            send_cmd(0x21); send_cmd(0); send_cmd(kWidth - 1);
            send_cmd(0x22); send_cmd((uint8_t) page); send_cmd((uint8_t) page);

            uint8_t buf[1 + kWidth];
            buf[0] = kCtrlData;
            memcpy(&buf[1], &framebuffer_[page * kWidth], kWidth);
            i2c_write_blocking(i2c_, kI2cAddr, buf, sizeof(buf), false);
        }
    }

    i2c_inst_t * i2c_;
    uint8_t framebuffer_[kWidth * kPages] = {};

    Spectrum spectrum_l_;
    Spectrum spectrum_r_;

    // Stereo sample sources for the FFT spectrum view. L and R share
    // spectrum_count_ / spectrum_read_pos_ since they're written
    // sample-aligned by audio.cpp.
    const int16_t * spectrum_l_samples_ = nullptr;
    const int16_t * spectrum_r_samples_ = nullptr;
    size_t          spectrum_count_     = 0;
    const size_t  * spectrum_read_pos_  = nullptr;

    // Pot popup state. ticks_left counts down each tick; while non-zero,
    // the top bar shows the pot's label + value. When it expires, the
    // bar reverts to "VOL" + volume_q15_.
    char     pot_popup_name_[kPotPopupNameMax] = {};
    int16_t  pot_popup_q15_                    = 0;
    uint32_t pot_popup_ticks_left_             = 0;

    // Default top-bar value when no pot is being turned. Set from
    // pots.cpp's volume dispatch case so the bar always reflects the
    // current master volume.
    int16_t  volume_q15_                       = 32767;

    // BT connection state. The BT icon is drawn only while connected;
    // until a phone pairs the icon slot stays blank.
    bool     bt_connected_                     = false;

    // Front-panel toggle states. Rendered as the T / C / V / P letter
    // indicators in the top bar; each letter is only drawn while its
    // switch is active.
    bool     tone_bypass_on_                   = false;
    bool     colour_bypass_on_                 = false;
    bool     vol_policy_max_on_                = false;
    bool     vis_pre_on_                       = false;

    // 5x7 ASCII font, columns 0..4, LSB at top. Covers printable ASCII
    // 32..126; everything else renders as '?'. Roughly the classic LCD
    // 5x7 glyph set used in countless embedded projects.
    static constexpr uint8_t kFont[(127 - 32) * 5] = {
        0x00, 0x00, 0x00, 0x00, 0x00,  // 32  ' '
        0x00, 0x00, 0x5F, 0x00, 0x00,  // 33  '!'
        0x00, 0x07, 0x00, 0x07, 0x00,  // 34  '"'
        0x14, 0x7F, 0x14, 0x7F, 0x14,  // 35  '#'
        0x24, 0x2A, 0x7F, 0x2A, 0x12,  // 36  '$'
        0x23, 0x13, 0x08, 0x64, 0x62,  // 37  '%'
        0x36, 0x49, 0x55, 0x22, 0x50,  // 38  '&'
        0x00, 0x05, 0x03, 0x00, 0x00,  // 39  '\''
        0x00, 0x1C, 0x22, 0x41, 0x00,  // 40  '('
        0x00, 0x41, 0x22, 0x1C, 0x00,  // 41  ')'
        0x14, 0x08, 0x3E, 0x08, 0x14,  // 42  '*'
        0x08, 0x08, 0x3E, 0x08, 0x08,  // 43  '+'
        0x00, 0x50, 0x30, 0x00, 0x00,  // 44  ','
        0x08, 0x08, 0x08, 0x08, 0x08,  // 45  '-'
        0x00, 0x60, 0x60, 0x00, 0x00,  // 46  '.'
        0x20, 0x10, 0x08, 0x04, 0x02,  // 47  '/'
        0x3E, 0x51, 0x49, 0x45, 0x3E,  // 48  '0'
        0x00, 0x42, 0x7F, 0x40, 0x00,  // 49  '1'
        0x42, 0x61, 0x51, 0x49, 0x46,  // 50  '2'
        0x21, 0x41, 0x45, 0x4B, 0x31,  // 51  '3'
        0x18, 0x14, 0x12, 0x7F, 0x10,  // 52  '4'
        0x27, 0x45, 0x45, 0x45, 0x39,  // 53  '5'
        0x3C, 0x4A, 0x49, 0x49, 0x30,  // 54  '6'
        0x01, 0x71, 0x09, 0x05, 0x03,  // 55  '7'
        0x36, 0x49, 0x49, 0x49, 0x36,  // 56  '8'
        0x06, 0x49, 0x49, 0x29, 0x1E,  // 57  '9'
        0x00, 0x36, 0x36, 0x00, 0x00,  // 58  ':'
        0x00, 0x56, 0x36, 0x00, 0x00,  // 59  ';'
        0x08, 0x14, 0x22, 0x41, 0x00,  // 60  '<'
        0x14, 0x14, 0x14, 0x14, 0x14,  // 61  '='
        0x00, 0x41, 0x22, 0x14, 0x08,  // 62  '>'
        0x02, 0x01, 0x51, 0x09, 0x06,  // 63  '?'
        0x32, 0x49, 0x79, 0x41, 0x3E,  // 64  '@'
        0x7E, 0x11, 0x11, 0x11, 0x7E,  // 65  'A'
        0x7F, 0x49, 0x49, 0x49, 0x36,  // 66  'B'
        0x3E, 0x41, 0x41, 0x41, 0x22,  // 67  'C'
        0x7F, 0x41, 0x41, 0x22, 0x1C,  // 68  'D'
        0x7F, 0x49, 0x49, 0x49, 0x41,  // 69  'E'
        0x7F, 0x09, 0x09, 0x09, 0x01,  // 70  'F'
        0x3E, 0x41, 0x49, 0x49, 0x7A,  // 71  'G'
        0x7F, 0x08, 0x08, 0x08, 0x7F,  // 72  'H'
        0x00, 0x41, 0x7F, 0x41, 0x00,  // 73  'I'
        0x20, 0x40, 0x41, 0x3F, 0x01,  // 74  'J'
        0x7F, 0x08, 0x14, 0x22, 0x41,  // 75  'K'
        0x7F, 0x40, 0x40, 0x40, 0x40,  // 76  'L'
        0x7F, 0x02, 0x0C, 0x02, 0x7F,  // 77  'M'
        0x7F, 0x04, 0x08, 0x10, 0x7F,  // 78  'N'
        0x3E, 0x41, 0x41, 0x41, 0x3E,  // 79  'O'
        0x7F, 0x09, 0x09, 0x09, 0x06,  // 80  'P'
        0x3E, 0x41, 0x51, 0x21, 0x5E,  // 81  'Q'
        0x7F, 0x09, 0x19, 0x29, 0x46,  // 82  'R'
        0x46, 0x49, 0x49, 0x49, 0x31,  // 83  'S'
        0x01, 0x01, 0x7F, 0x01, 0x01,  // 84  'T'
        0x3F, 0x40, 0x40, 0x40, 0x3F,  // 85  'U'
        0x1F, 0x20, 0x40, 0x20, 0x1F,  // 86  'V'
        0x7F, 0x20, 0x18, 0x20, 0x7F,  // 87  'W'
        0x63, 0x14, 0x08, 0x14, 0x63,  // 88  'X'
        0x03, 0x04, 0x78, 0x04, 0x03,  // 89  'Y'
        0x61, 0x51, 0x49, 0x45, 0x43,  // 90  'Z'
        0x00, 0x7F, 0x41, 0x41, 0x00,  // 91  '['
        0x02, 0x04, 0x08, 0x10, 0x20,  // 92  '\\'
        0x00, 0x41, 0x41, 0x7F, 0x00,  // 93  ']'
        0x04, 0x02, 0x01, 0x02, 0x04,  // 94  '^'
        0x40, 0x40, 0x40, 0x40, 0x40,  // 95  '_'
        0x00, 0x01, 0x02, 0x04, 0x00,  // 96  '`'
        0x20, 0x54, 0x54, 0x54, 0x78,  // 97  'a'
        0x7F, 0x48, 0x44, 0x44, 0x38,  // 98  'b'
        0x38, 0x44, 0x44, 0x44, 0x20,  // 99  'c'
        0x38, 0x44, 0x44, 0x48, 0x7F,  // 100 'd'
        0x38, 0x54, 0x54, 0x54, 0x18,  // 101 'e'
        0x08, 0x7E, 0x09, 0x01, 0x02,  // 102 'f'
        0x0C, 0x52, 0x52, 0x52, 0x3E,  // 103 'g'
        0x7F, 0x08, 0x04, 0x04, 0x78,  // 104 'h'
        0x00, 0x44, 0x7D, 0x40, 0x00,  // 105 'i'
        0x20, 0x40, 0x44, 0x3D, 0x00,  // 106 'j'
        0x7F, 0x10, 0x28, 0x44, 0x00,  // 107 'k'
        0x00, 0x41, 0x7F, 0x40, 0x00,  // 108 'l'
        0x7C, 0x04, 0x18, 0x04, 0x78,  // 109 'm'
        0x7C, 0x08, 0x04, 0x04, 0x78,  // 110 'n'
        0x38, 0x44, 0x44, 0x44, 0x38,  // 111 'o'
        0x7C, 0x14, 0x14, 0x14, 0x08,  // 112 'p'
        0x08, 0x14, 0x14, 0x18, 0x7C,  // 113 'q'
        0x7C, 0x08, 0x04, 0x04, 0x08,  // 114 'r'
        0x48, 0x54, 0x54, 0x54, 0x20,  // 115 's'
        0x04, 0x3F, 0x44, 0x40, 0x20,  // 116 't'
        0x3C, 0x40, 0x40, 0x20, 0x7C,  // 117 'u'
        0x1C, 0x20, 0x40, 0x20, 0x1C,  // 118 'v'
        0x3C, 0x40, 0x30, 0x40, 0x3C,  // 119 'w'
        0x44, 0x28, 0x10, 0x28, 0x44,  // 120 'x'
        0x0C, 0x50, 0x50, 0x50, 0x3C,  // 121 'y'
        0x44, 0x64, 0x54, 0x4C, 0x44,  // 122 'z'
        0x00, 0x08, 0x36, 0x41, 0x00,  // 123 '{'
        0x00, 0x00, 0x7F, 0x00, 0x00,  // 124 '|'
        0x00, 0x41, 0x36, 0x08, 0x00,  // 125 '}'
        0x02, 0x01, 0x02, 0x04, 0x02,  // 126 '~'
    };
};

// Singleton OLED instance, defined in btspkr.cpp. Modules that need
// to push status / track / popup updates pull this in via display.hpp.
extern Display display;
