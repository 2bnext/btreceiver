// audio.cpp -- SBC decode, ring buffers, I2S playback callback, scope tap.

#include "audio.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "btstack_resample.h"
#include "btstack_ring_buffer.h"

#include "dsp.hpp"

namespace {

// A2DP/SBC always delivers stereo 16-bit PCM after decoding.
constexpr int    kNumChannels      = 2;
constexpr int    kBytesPerFrame    = 2 * kNumChannels;
constexpr int    kMaxSbcFrameSize  = 120;

// Ring-buffer policy (in units of SBC frames):
//   < kOptimalFramesMin  -> stretch (slightly slower playback)
//   in [min..max]        -> nominal
//   > kOptimalFramesMax  -> compress (slightly faster playback)
constexpr int    kOptimalFramesMin = 60;
constexpr int    kOptimalFramesMax = 80;
constexpr int    kAdditionalFrames = 30;

uint8_t                          sbc_frame_storage_[(kOptimalFramesMax + kAdditionalFrames) * kMaxSbcFrameSize];
btstack_ring_buffer_t            sbc_frame_ring_buffer_;
unsigned int                     sbc_frame_size_ = 0;

uint8_t                          decoded_audio_storage_[(128 + 16) * kBytesPerFrame];
btstack_ring_buffer_t            decoded_audio_ring_buffer_;

const btstack_sbc_decoder_t *    sbc_decoder_instance_ = nullptr;
btstack_sbc_decoder_bluedroid_t  sbc_decoder_context_;

bool                             media_initialized_   = false;
bool                             audio_stream_started_ = false;
btstack_resample_t               resample_instance_;

int16_t *                        request_buffer_ = nullptr;
int                              request_frames_ = 0;

// Spectrum tap: 1024 samples each of L and R, captured at the I2S
// boundary (post-DSP). playback_handler() writes both in lockstep;
// the OLED's stereo FFT reads via audio_scope_l_buf / audio_scope_r_buf.
// (Name is "scope" for historical reasons -- scope mode itself was
// dropped along with multi-mode display cycling.)
constexpr size_t                 kScopeBufSamples = 1024;
int16_t                          scope_l_buf_[kScopeBufSamples] = {};
int16_t                          scope_r_buf_[kScopeBufSamples] = {};
size_t                           scope_write_pos_ = 0;

int read_media_data_header(uint8_t * packet, int size, int * offset,
                           avdtp_media_packet_header_t * media_header) {
    int media_header_len = 12;
    int pos              = *offset;

    if (size - pos < media_header_len) {
        return 0;
    }

    media_header->version              = packet[pos] & 0x03;
    media_header->padding              = get_bit16(packet[pos], 2);
    media_header->extension            = get_bit16(packet[pos], 3);
    media_header->csrc_count           = (packet[pos] >> 4) & 0x0F;
    pos++;
    media_header->marker               = get_bit16(packet[pos], 0);
    media_header->payload_type         = (packet[pos] >> 1) & 0x7F;
    pos++;
    media_header->sequence_number      = big_endian_read_16(packet, pos); pos += 2;
    media_header->timestamp            = big_endian_read_32(packet, pos); pos += 4;
    media_header->synchronization_source = big_endian_read_32(packet, pos); pos += 4;
    *offset = pos;
    return 1;
}

int read_sbc_header(uint8_t * packet, int size, int * offset,
                    avdtp_sbc_codec_header_t * sbc_header) {
    int sbc_header_len = 12;
    int pos            = *offset;

    if (size - pos < sbc_header_len) {
        return 0;
    }

    sbc_header->fragmentation   = get_bit16(packet[pos], 7);
    sbc_header->starting_packet = get_bit16(packet[pos], 6);
    sbc_header->last_packet     = get_bit16(packet[pos], 5);
    sbc_header->num_frames      = packet[pos] & 0x0f;
    pos++;
    *offset = pos;
    return 1;
}

// Called by the SBC decoder for every decoded frame. Resamples (clock
// drift compensation) and writes to the I2S request buffer set by
// playback_handler(); leftover goes to the decoded-PCM overflow ring.
void handle_pcm_data(int16_t * data, int num_audio_frames, int num_channels,
                     int sample_rate, void * context) {
    UNUSED(sample_rate);
    UNUSED(context);
    UNUSED(num_channels);

    if (!btstack_audio_sink_get_instance()) {
        return;
    }

    int16_t  output_buffer[(128 + 16) * kNumChannels];
    uint32_t resampled_frames = btstack_resample_block(&resample_instance_, data,
                                                      num_audio_frames, output_buffer);

    int frames_to_copy = btstack_min(resampled_frames, (uint32_t) request_frames_);
    memcpy(request_buffer_, output_buffer, frames_to_copy * kBytesPerFrame);
    request_frames_ -= frames_to_copy;
    request_buffer_ += frames_to_copy * kNumChannels;

    int frames_to_store = resampled_frames - frames_to_copy;

    if (frames_to_store) {
        int status = btstack_ring_buffer_write(&decoded_audio_ring_buffer_,
                                               (uint8_t *) &output_buffer[frames_to_copy * kNumChannels],
                                               frames_to_store * kBytesPerFrame);

        if (status) {
            printf("PCM ring buffer overflow (decoder running ahead of I2S)\n");
        }
    }
}

// Called from the I2S backend when it needs more PCM. Drains the
// decoded-PCM ring, decodes more SBC frames into whatever space
// remains, runs the DSP chain in place, and taps the result for
// the OLED scope.
void playback_handler(int16_t * buffer, uint16_t num_audio_frames) {
    int16_t * const orig_buffer = buffer;
    uint16_t        orig_frames = num_audio_frames;

    if (sbc_frame_size_ == 0) {
        memset(buffer, 0, num_audio_frames * kBytesPerFrame);
    } else {
        uint32_t bytes_read;
        btstack_ring_buffer_read(&decoded_audio_ring_buffer_, (uint8_t *) buffer,
                                 num_audio_frames * kBytesPerFrame, &bytes_read);
        buffer          += bytes_read / kNumChannels;
        num_audio_frames -= bytes_read / kBytesPerFrame;

        request_buffer_ = buffer;
        request_frames_ = num_audio_frames;

        while (request_frames_ &&
               btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer_) >= sbc_frame_size_) {
            uint8_t sbc_frame[kMaxSbcFrameSize];
            btstack_ring_buffer_read(&sbc_frame_ring_buffer_, sbc_frame, sbc_frame_size_, &bytes_read);
            sbc_decoder_instance_->decode_signed_16(&sbc_decoder_context_, 0, sbc_frame, sbc_frame_size_);
        }
    }

    // Visualisation tap: pre- or post-DSP, controlled by the front-panel
    // toggle (dsp_set_visualisation_pre). Pre snapshots the raw decoded
    // audio so the meters / spectrum show what the source is delivering;
    // post (default) shows what's actually heading to the DAC after the
    // on-board chain.
    const bool pre   = dsp_get_visualisation_pre();
    size_t     pos   = scope_write_pos_;

    if (pre) {
        for (uint16_t i = 0; i < orig_frames; i++) {
            scope_l_buf_[pos] = orig_buffer[i * 2];
            scope_r_buf_[pos] = orig_buffer[i * 2 + 1];
            pos = (pos + 1) % kScopeBufSamples;
        }
    }

    // DSP chain runs unconditionally; only the tap point moves.
    dsp_run(orig_buffer, orig_frames);

    if (!pre) {
        for (uint16_t i = 0; i < orig_frames; i++) {
            scope_l_buf_[pos] = orig_buffer[i * 2];
            scope_r_buf_[pos] = orig_buffer[i * 2 + 1];
            pos = (pos + 1) % kScopeBufSamples;
        }
    }

    scope_write_pos_ = pos;
}

}  // namespace

// SBC capabilities exposed to the BT setup. 0xFF = "we accept any
// value the source proposes"; bitpool range 2..53.
uint8_t audio_sbc_codec_capabilities[4] = {
    0xFF,
    0xFF,
    2, 53,
};

void audio_handle_l2cap_media_data_packet(uint8_t seid, uint8_t * packet, uint16_t size) {
    UNUSED(seid);
    int pos = 0;

    avdtp_media_packet_header_t media_header;

    if (!read_media_data_header(packet, size, &pos, &media_header)) {
        return;
    }

    avdtp_sbc_codec_header_t sbc_header;

    if (!read_sbc_header(packet, size, &pos, &sbc_header)) {
        return;
    }

    int      packet_length = size - pos;
    uint8_t * packet_begin = packet + pos;

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();

    if (!audio) {
        sbc_decoder_instance_->decode_signed_16(&sbc_decoder_context_, 0, packet_begin, packet_length);
        return;
    }

    // Defensive: a malformed source could send num_frames == 0. The
    // division below would otherwise be undefined behavior; drop the
    // packet instead.
    if (sbc_header.num_frames == 0) {
        return;
    }

    sbc_frame_size_ = packet_length / sbc_header.num_frames;
    int status = btstack_ring_buffer_write(&sbc_frame_ring_buffer_, packet_begin, packet_length);

    if (status != ERROR_CODE_SUCCESS) {
        printf("SBC ring buffer overflow (source pushing faster than we drain)\n");
    }

    int sbc_frames_in_buffer = btstack_ring_buffer_bytes_available(&sbc_frame_ring_buffer_) / sbc_frame_size_;

    uint32_t nominal_factor = 0x10000;
    uint32_t compensation   = 0x00100;
    uint32_t resampling_factor;

    if (sbc_frames_in_buffer < kOptimalFramesMin) {
        resampling_factor = nominal_factor - compensation;
    } else if (sbc_frames_in_buffer <= kOptimalFramesMax) {
        resampling_factor = nominal_factor;
    } else {
        resampling_factor = nominal_factor + compensation;
    }

    btstack_resample_set_factor(&resample_instance_, resampling_factor);

    if (!audio_stream_started_ && sbc_frames_in_buffer >= kOptimalFramesMin) {
        audio_media_processing_start();
    }
}

int audio_media_processing_init(media_codec_configuration_sbc_t * cfg) {
    if (media_initialized_) {
        return 0;
    }

    sbc_decoder_instance_ = btstack_sbc_decoder_bluedroid_init_instance(&sbc_decoder_context_);
    sbc_decoder_instance_->configure(&sbc_decoder_context_, SBC_MODE_STANDARD, handle_pcm_data, NULL);

    btstack_ring_buffer_init(&sbc_frame_ring_buffer_, sbc_frame_storage_, sizeof(sbc_frame_storage_));
    btstack_ring_buffer_init(&decoded_audio_ring_buffer_, decoded_audio_storage_, sizeof(decoded_audio_storage_));
    btstack_resample_init(&resample_instance_, cfg->num_channels);

    // EQ filter coefficients are Fs-dependent.
    dsp_set_sample_rate(cfg->sampling_frequency);

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();

    if (audio) {
        audio->init(kNumChannels, cfg->sampling_frequency, &playback_handler);
    }

    audio_stream_started_ = false;
    media_initialized_    = true;
    return 0;
}

void audio_media_processing_start() {
    if (!media_initialized_) {
        return;
    }

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();

    if (audio) {
        audio->start_stream();
    }

    audio_stream_started_ = true;
}

void audio_media_processing_pause() {
    if (!media_initialized_) {
        return;
    }

    audio_stream_started_ = false;
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();

    if (audio) {
        audio->stop_stream();
    }

    btstack_ring_buffer_reset(&decoded_audio_ring_buffer_);
    btstack_ring_buffer_reset(&sbc_frame_ring_buffer_);
}

void audio_media_processing_close() {
    if (!media_initialized_) {
        return;
    }

    media_initialized_    = false;
    audio_stream_started_ = false;
    sbc_frame_size_       = 0;

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();

    if (audio) {
        audio->close();
    }
}

const int16_t * audio_scope_l_buf()    { return scope_l_buf_;     }
const int16_t * audio_scope_r_buf()    { return scope_r_buf_;     }
size_t          audio_scope_buf_size() { return kScopeBufSamples; }
const size_t  * audio_scope_pos()      { return &scope_write_pos_; }
