// audio.hpp -- SBC decode + ring buffers + I2S playback handler +
// post-DSP L/R sample tap for the OLED spectrum view.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "btstack.h"  // for btstack_sbc_channel_mode_t / allocation_method_t

// Shared SBC config struct. Filled in by the BT module from
// A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION; consumed by
// audio_media_processing_init() to spin up the decoder + ring buffers
// for the negotiated stream.
typedef struct {
    uint8_t  reconfigure;          // true if A2DP told us this is a reconfig
    uint8_t  num_channels;
    uint16_t sampling_frequency;
    uint8_t  block_length;
    uint8_t  subbands;
    uint8_t  min_bitpool_value;
    uint8_t  max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} media_codec_configuration_sbc_t;

// Capabilities advertised on our SBC stream endpoint. Defined in
// audio.cpp; bt.cpp passes the pointer + length to BTstack.
extern uint8_t       audio_sbc_codec_capabilities[4];
constexpr size_t     audio_sbc_codec_capabilities_size = 4;

// Registered with a2dp_sink_register_media_handler from bt_setup().
void audio_handle_l2cap_media_data_packet(uint8_t seid, uint8_t * packet, uint16_t size);

// Stream lifecycle, called by the A2DP packet handler in bt.cpp.
int  audio_media_processing_init(media_codec_configuration_sbc_t * cfg);
void audio_media_processing_start();
void audio_media_processing_pause();
void audio_media_processing_close();

// Post-DSP L/R sample tap for the OLED spectrum view. main() wires
// these into the Display via set_spectrum_source(). Both buffers
// share the same write index so they stay sample-aligned.
const int16_t * audio_scope_l_buf();
const int16_t * audio_scope_r_buf();
size_t          audio_scope_buf_size();
const size_t  * audio_scope_pos();
