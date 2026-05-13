// btreceiver -- Bluetooth A2DP audio receiver for the Raspberry Pi Pico 2 W.
// Streams audio from a phone/PC over Bluetooth Classic and plays it back
// through a PCM5102 I2S DAC. Pairs on boot with "just works" SSP.
//
// This file is the boot wiring + main loop. Functionality is split into:
//   - bt.hpp/.cpp      : BTstack handlers + A2DP/AVRCP/SDP/GAP setup
//   - audio.hpp/.cpp   : SBC decoder, ring buffers, I2S playback handler
//   - dsp.hpp/.cpp     : float32 DSP chain (presence biquad + master volume)
//   - pots.hpp/.cpp    : ADS1115 ADC scanning + pot -> DSP/display routing
//   - display.hpp      : SSD1306 OLED + track/scope/spectrum UI
//
// Audio path:
//   phone -> A2DP -> SBC ring -> decoder -> resample -> dsp_run() -> I2S DAC
//   (drift-compensation resampler nudged by SBC backlog depth).

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"     // brings up the CYW43439 (Wi-Fi + BT radio)
#include "hardware/i2c.h"

#include "btstack.h"

#include "audio.hpp"
#include "bt.hpp"
#include "display.hpp"
#include "dsp.hpp"
#include "pots.hpp"
#include "vu.hpp"

// I2S backend lives in btstack_audio_pico.c.
extern "C" const btstack_audio_sink_t * btstack_audio_pico_sink_get_instance(void);

// I2C bus shared by the OLED and both ADS1115 ADCs.
#define I2C_PORT i2c0
#define I2C_SDA  8
#define I2C_SCL  9

// Singleton OLED instance. extern declaration is in display.hpp so the
// audio / pot / BT modules can push updates via `display.foo()`.
Display display(I2C_PORT);

namespace {

btstack_timer_source_t display_timer_;
btstack_timer_source_t pot_timer_;

// 33 ms tick = ~30 Hz scope refresh. Each full-screen flush is ~10 ms
// at 1 MHz I2C, leaving ~23 ms of free run-loop time per cycle for the
// audio refill timer (5 ms cadence, ~35 ms buffer reserve).
constexpr uint32_t kDisplayTickMs = 33;

// 50 ms = 20 Hz pot polling. With EMA alpha=1/4 that gives ~250 ms
// settling for the audio-taper / presence smoothing.
constexpr uint32_t kPotPollMs = 50;

// Front-panel SPST front-panel toggles. Each is wired active-HIGH:
// one terminal to 3V3, the other to the GPIO with a 10 kΩ external
// pull-down to GND (the internal pull-down is too weak to fight
// breadboard leakage). Closed = HIGH = active.
//
// Polled on the pot timer; the 50 ms cadence is well above mechanical
// bounce (~10 ms) so naive sampling is glitch-free.
constexpr uint kGpioToneBypass    = 14;  // skip 4-band EQ
constexpr uint kGpioColourBypass  = 15;  // skip drive + warmth + x-talk
constexpr uint kGpioVolumePolicy  = 18;  // ignore phone slider, force 127
constexpr uint kGpioVisPreDsp     = 19;  // tap meters/spectrum pre-DSP

void toggles_init() {
    const uint pins[] = {
        kGpioToneBypass, kGpioColourBypass, kGpioVolumePolicy, kGpioVisPreDsp,
    };

    for (uint p : pins) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_down(p);
    }
}

void toggles_poll() {
    bool tone     = gpio_get(kGpioToneBypass);
    bool colour   = gpio_get(kGpioColourBypass);
    bool vol_pol  = gpio_get(kGpioVolumePolicy);
    // Visualisation toggle is wired with the inverse polarity from
    // the others (chassis switch was already mounted that way), so
    // we invert the read here rather than rewiring.
    bool vis_pre  = !gpio_get(kGpioVisPreDsp);

    static bool tone_prev    = false;
    static bool colour_prev  = false;
    static bool vol_pol_prev = false;
    static bool vis_pre_prev = false;

    if (tone != tone_prev) {
        printf("tone bypass %s\n",        tone    ? "ON" : "off");
        tone_prev = tone;
    }

    if (colour != colour_prev) {
        printf("colour bypass %s\n",      colour  ? "ON" : "off");
        colour_prev = colour;
    }

    if (vol_pol != vol_pol_prev) {
        printf("volume policy %s\n",      vol_pol ? "IGNORE phone slider (force 127)"
                                                  : "follow phone slider");
        vol_pol_prev = vol_pol;
    }

    if (vis_pre != vis_pre_prev) {
        printf("visualisation tap %s\n",  vis_pre ? "PRE-DSP" : "post-DSP");
        vis_pre_prev = vis_pre;
    }

    dsp_set_tone_bypass        (tone);
    dsp_set_colour_bypass      (colour);
    bt_set_volume_policy_ignore(vol_pol);
    dsp_set_visualisation_pre  (vis_pre);

    // Surface the four states on the OLED top bar (T / C / V / P
    // letter indicators).
    display.set_toggle_states(tone, colour, vol_pol, vis_pre);
}

// If the previous firmware was halted mid-I2C-transaction (e.g. by a
// debugger flash that resets the RP2350 but leaves the SSD1306 alone),
// the slave can be left holding SDA low and wedge the bus across the
// soft reset. Bit-bang up to 9 SCL pulses with SDA released so the
// stuck slave clocks through whatever byte it was mid-way through,
// then issue a manual STOP. Cheap, safe to run unconditionally.
void i2c_bus_recovery() {
    gpio_init(I2C_SDA);
    gpio_init(I2C_SCL);
    gpio_set_dir(I2C_SDA, GPIO_IN);
    gpio_set_dir(I2C_SCL, GPIO_IN);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    sleep_us(10);

    if (gpio_get(I2C_SDA)) {
        return;
    }

    // Drive SCL low / release alternately. SDA stays as input so any
    // slave can release it as soon as it sees enough clocks.
    for (int i = 0; i < 9 && !gpio_get(I2C_SDA); i++) {
        gpio_set_dir(I2C_SCL, GPIO_OUT);
        gpio_put(I2C_SCL, 0);
        sleep_us(5);
        gpio_set_dir(I2C_SCL, GPIO_IN);
        sleep_us(5);
    }

    // Manual STOP: SDA low while SCL high, then SDA released.
    gpio_set_dir(I2C_SDA, GPIO_OUT);
    gpio_put(I2C_SDA, 0);
    sleep_us(5);
    gpio_set_dir(I2C_SDA, GPIO_IN);
    sleep_us(5);
}

void display_timer_handler(btstack_timer_source_t * ts) {
    display.tick();
    btstack_run_loop_set_timer(ts, kDisplayTickMs);
    btstack_run_loop_add_timer(ts);
}

void pot_timer_handler(btstack_timer_source_t * ts) {
    pots_poll_tick();
    toggles_poll();
    bt_volume_policy_tick();
    vu_update();
    btstack_run_loop_set_timer(ts, kPotPollMs);
    btstack_run_loop_add_timer(ts);
}

}  // namespace

int main(void) {
    stdio_init_all();

    // cyw43_arch_init brings up the radio AND (because pico_btstack_cyw43
    // is linked) initialises BTstack on top of it: btstack_memory_init()
    // and the run loop bound to the cyw43 async_context.
    if (cyw43_arch_init()) {
        printf("cyw43_arch_init failed\n");
        return -1;
    }

    // Unstick any I2C slave that was caught mid-transaction by the
    // previous firmware's reset (otherwise the OLED stays dark after a
    // debugger flash).
    i2c_bus_recovery();

    // 1 MHz I2C so the SSD1306 framebuffer flush doesn't block the run
    // loop long enough to starve the audio refill timer (~10 ms full
    // flush at 1 MHz vs ~25 ms at 400 kHz).
    i2c_init(I2C_PORT, 1000 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // OLED first so the splash appears before anything else can block.
    display.init();
    display.set_spectrum_source(audio_scope_l_buf(), audio_scope_r_buf(),
                                audio_scope_buf_size(), audio_scope_pos());

    // ADS1115s up + first reads, so master volume / presence start at
    // the wiper-correct values rather than zero. Pots are mechanical:
    // no flash persistence needed.
    pots_init();

    // Front-panel bypass toggles -- read once at boot so we honour the
    // switch positions immediately rather than waiting for the first
    // pot timer tick.
    toggles_init();
    toggles_poll();

    // VU meter PWM outputs -- start at 0 duty (needles parked left).
    // The pot timer drives vu_update() once audio starts flowing.
    vu_init();

    // Plug our PIO-based I2S backend into BTstack's audio HAL before
    // anything in bt_setup can call btstack_audio_sink_get_instance().
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());

    bt_setup();

    btstack_run_loop_set_timer_handler(&display_timer_, &display_timer_handler);
    btstack_run_loop_set_timer(&display_timer_, kDisplayTickMs);
    btstack_run_loop_add_timer(&display_timer_);

    btstack_run_loop_set_timer_handler(&pot_timer_, &pot_timer_handler);
    btstack_run_loop_set_timer(&pot_timer_, kPotPollMs);
    btstack_run_loop_add_timer(&pot_timer_);

    printf("Starting BTstack...\n");
    hci_power_control(HCI_POWER_ON);

    // Driven by the cyw43 async_context (interrupt-backed); never returns.
    btstack_run_loop_execute();
    return 0;
}
