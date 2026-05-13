// bt.cpp -- BTstack handlers + setup.
//
// Ported from BTstack's a2dp_sink_demo.c (BlueKitchen GmbH); the stdin
// console UI, cover-art client, and WAV-file output paths were stripped.
// The device pairs on boot with "just works" Secure Simple Pairing.

#include "bt.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "btstack.h"

#include "audio.hpp"
#include "display.hpp"
#include "dsp.hpp"

namespace {

// -----------------------------------------------------------------
// SDP records, registration nodes, and per-connection state.
// -----------------------------------------------------------------
btstack_packet_callback_registration_t hci_event_callback_registration_;

uint8_t sdp_avdtp_sink_service_buffer_[150];
uint8_t sdp_avrcp_target_service_buffer_[150];
uint8_t sdp_avrcp_controller_service_buffer_[200];
uint8_t device_id_sdp_service_buffer_[100];

// AVRCP target state we report to the source. The device is mains-
// powered (no battery), so we advertise EXTERNAL to phones that surface
// the source's battery state in their UI -- WARNING was misleading.
int                    volume_percentage_ = 0;
avrcp_battery_status_t battery_status_    = AVRCP_BATTERY_STATUS_EXTERNAL;

// Volume policy toggle. When true: ignore phone-side slider changes,
// keep DSP at AVRCP-127 (passthrough), and periodically tell the phone
// "you're at 127" so its slider parks at the top.
bool volume_policy_ignore_ = false;

// Tick counter for the periodic re-assertion (pot timer fires at 50 ms,
// so 100 ticks = 5 s between re-assertions).
uint32_t volume_policy_tick_count_ = 0;
constexpr uint32_t kVolumePolicyTicksPerAssertion = 100;

typedef enum {
    STREAM_STATE_CLOSED,
    STREAM_STATE_OPEN,
    STREAM_STATE_PLAYING,
    STREAM_STATE_PAUSED,
} stream_state_t;

typedef struct {
    uint8_t a2dp_local_seid;
    uint8_t media_sbc_codec_configuration[4];
} a2dp_sink_stream_endpoint_t;
a2dp_sink_stream_endpoint_t a2dp_sink_stream_endpoint_;

typedef struct {
    bd_addr_t addr;
    uint16_t  a2dp_cid;
    uint8_t   a2dp_local_seid;
    stream_state_t stream_state;
    media_codec_configuration_sbc_t sbc_configuration;
} a2dp_sink_state_t;
a2dp_sink_state_t a2dp_sink_state_;

typedef struct {
    bd_addr_t addr;
    uint16_t  avrcp_cid;
    bool      playing;
    uint16_t  notifications_supported_by_target;
} avrcp_state_t;
avrcp_state_t avrcp_state_;

void dump_sbc_configuration(media_codec_configuration_sbc_t * cfg) {
    printf("SBC config   : %d ch, %d Hz, bitpool [%d..%d]\n",
           cfg->num_channels, cfg->sampling_frequency,
           cfg->min_bitpool_value, cfg->max_bitpool_value);
}

// -----------------------------------------------------------------
// HCI events: stack-state announce, legacy PIN pairing, remote name
// resolution for the OLED status line.
// -----------------------------------------------------------------
void hci_packet_handler(uint8_t packet_type, uint16_t channel,
                        uint8_t * packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE: {
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) {
                break;
            }

            bd_addr_t local_addr;
            gap_local_bd_addr(local_addr);
            printf("BTstack up on %s -- ready to pair as \"2B Speaker\".\n", bd_addr_to_str(local_addr));
            break;
        }

        case HCI_EVENT_PIN_CODE_REQUEST: {
            // Legacy pairing fallback for devices that don't speak SSP.
            bd_addr_t address;
            printf("Pin code request -- using '0000'\n");
            hci_event_pin_code_request_get_bd_addr(packet, address);
            gap_pin_code_response(address, "0000");
            break;
        }

        case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
            uint8_t status = hci_event_remote_name_request_complete_get_status(packet);

            if (status != ERROR_CODE_SUCCESS) {
                printf("Remote name request failed: 0x%02x\n", status);
                break;
            }

            const char * name = hci_event_remote_name_request_complete_get_remote_name(packet);
            printf("Remote name: %s\n", name);
            break;
        }

        default:
            break;
    }
}

// -----------------------------------------------------------------
// AVRCP top-level: connection up/down. AVRCP carries Controller +
// Target on the same channel; this handler just bookkeeps the cid.
// -----------------------------------------------------------------
void avrcp_packet_handler(uint8_t packet_type, uint16_t channel,
                          uint8_t * packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) {
        return;
    }

    avrcp_state_t * conn = &avrcp_state_;

    switch (packet[2]) {
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
            uint8_t status = avrcp_subevent_connection_established_get_status(packet);

            if (status != ERROR_CODE_SUCCESS) {
                printf("AVRCP: connect failed, status 0x%02x\n", status);
                conn->avrcp_cid = 0;
                return;
            }

            conn->avrcp_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            bd_addr_t addr;
            avrcp_subevent_connection_established_get_bd_addr(packet, addr);
            printf("AVRCP: connected to %s (cid 0x%02x)\n", bd_addr_to_str(addr), conn->avrcp_cid);
            display.set_bt_connected(true);

            gap_remote_name_request(addr, 0, 0);

            avrcp_target_support_event(conn->avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            avrcp_target_support_event(conn->avrcp_cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
            avrcp_target_battery_status_changed(conn->avrcp_cid, battery_status_);
            avrcp_controller_get_supported_events(conn->avrcp_cid);

            // If the volume policy was engaged before this connection
            // came up, immediately push 127 so the phone's slider parks
            // at the top instead of waiting for the next periodic tick.
            if (volume_policy_ignore_) {
                avrcp_target_volume_changed(conn->avrcp_cid, 127);
                dsp_set_avrcp_volume_127(127);
            }
            return;
        }

        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("AVRCP: released (cid 0x%02x)\n",
                   avrcp_subevent_connection_released_get_avrcp_cid(packet));
            conn->avrcp_cid = 0;
            conn->notifications_supported_by_target = 0;
            display.set_bt_connected(false);
            return;

        default:
            break;
    }
}

// -----------------------------------------------------------------
// AVRCP Controller: events flowing from the source. Track metadata
// (title/artist/album) and play/pause status.
// -----------------------------------------------------------------
void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel,
                                     uint8_t * packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) {
        return;
    }

    avrcp_state_t * conn = &avrcp_state_;

    if (conn->avrcp_cid == 0) {
        return;
    }

    uint8_t value[256];
    memset(value, 0, sizeof(value));

    switch (packet[2]) {
        case AVRCP_SUBEVENT_GET_CAPABILITY_EVENT_ID:
            conn->notifications_supported_by_target |= (1 << avrcp_subevent_get_capability_event_id_get_event_id(packet));
            break;

        case AVRCP_SUBEVENT_GET_CAPABILITY_EVENT_ID_DONE:
            avrcp_controller_enable_notification(conn->avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            avrcp_controller_enable_notification(conn->avrcp_cid, AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED);
            avrcp_controller_enable_notification(conn->avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            break;

        case AVRCP_SUBEVENT_NOTIFICATION_PLAYBACK_STATUS_CHANGED: {
            uint8_t st = avrcp_subevent_notification_playback_status_changed_get_play_status(packet);
            printf("AVRCP: playback %s\n", avrcp_play_status2str(st));
            conn->playing = (st == AVRCP_PLAYBACK_STATUS_PLAYING);
            break;
        }

        case AVRCP_SUBEVENT_NOTIFICATION_TRACK_CHANGED:
            printf("AVRCP: track changed\n");
            avrcp_controller_get_now_playing_info(conn->avrcp_cid);
            break;

        case AVRCP_SUBEVENT_NOTIFICATION_NOW_PLAYING_CONTENT_CHANGED:
            printf("AVRCP: now-playing content changed\n");
            avrcp_controller_get_now_playing_info(conn->avrcp_cid);
            break;

        case AVRCP_SUBEVENT_NOW_PLAYING_TITLE_INFO:
            if (avrcp_subevent_now_playing_title_info_get_value_len(packet) > 0) {
                memcpy(value, avrcp_subevent_now_playing_title_info_get_value(packet),
                       avrcp_subevent_now_playing_title_info_get_value_len(packet));
                printf("AVRCP: title  %s\n", value);
            }
            break;

        case AVRCP_SUBEVENT_NOW_PLAYING_ARTIST_INFO:
            if (avrcp_subevent_now_playing_artist_info_get_value_len(packet) > 0) {
                memcpy(value, avrcp_subevent_now_playing_artist_info_get_value(packet),
                       avrcp_subevent_now_playing_artist_info_get_value_len(packet));
                printf("AVRCP: artist %s\n", value);
            }
            break;

        case AVRCP_SUBEVENT_NOW_PLAYING_ALBUM_INFO:
            if (avrcp_subevent_now_playing_album_info_get_value_len(packet) > 0) {
                memcpy(value, avrcp_subevent_now_playing_album_info_get_value(packet),
                       avrcp_subevent_now_playing_album_info_get_value_len(packet));
                printf("AVRCP: album  %s\n", value);
            }
            break;

        default:
            break;
    }
}

// -----------------------------------------------------------------
// AVRCP Target: commands flowing to us. Phone slider lands here as
// absolute volume (0..127), forwarded to the DSP module.
// -----------------------------------------------------------------
void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel,
                                 uint8_t * packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) {
        return;
    }

    switch (packet[2]) {
        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED: {
            uint8_t volume = avrcp_subevent_notification_volume_changed_get_absolute_volume(packet);

            if (volume_policy_ignore_) {
                // Drop the phone's slider value. Force DSP to passthrough
                // (AVRCP = 127) and push 127 back to the phone so its
                // slider snaps to the top -- after this round-trip the
                // phone delivers full-resolution audio.
                printf("AVRCP: volume %d ignored (policy = max); pushing 127 back\n", volume);
                dsp_set_avrcp_volume_127(127);

                if (avrcp_state_.avrcp_cid != 0) {
                    avrcp_target_volume_changed(avrcp_state_.avrcp_cid, 127);
                }

                volume_percentage_ = 100;
            } else {
                volume_percentage_ = volume * 100 / 127;
                printf("AVRCP: volume %d%% (%d)\n", volume_percentage_, volume);
                dsp_set_avrcp_volume_127(volume);
            }
            break;
        }

        default:
            break;
    }
}

// -----------------------------------------------------------------
// A2DP Sink: signalling + stream lifecycle. Codec config arrives
// here; audio data comes via audio_handle_l2cap_media_data_packet
// (registered separately).
// -----------------------------------------------------------------
void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel,
                              uint8_t * packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) {
        return;
    }

    a2dp_sink_state_t * conn = &a2dp_sink_state_;

    switch (packet[2]) {
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("A2DP: non-SBC codec offered -- rejecting (only SBC is supported)\n");
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
            conn->sbc_configuration.reconfigure        = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            conn->sbc_configuration.num_channels       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            conn->sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            conn->sbc_configuration.block_length       = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            conn->sbc_configuration.subbands           = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            conn->sbc_configuration.min_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            conn->sbc_configuration.max_bitpool_value  = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);

            // A2DP spec numbers allocation methods 1..2 ({SNR, Loudness});
            // SBC encoder API expects 0..1.
            uint8_t allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            conn->sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);

            switch (a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet)) {
                case AVDTP_CHANNEL_MODE_JOINT_STEREO:
                    conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
                    break;

                case AVDTP_CHANNEL_MODE_STEREO:
                    conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
                    break;

                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
                    conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
                    break;

                case AVDTP_CHANNEL_MODE_MONO:
                    conn->sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
                    break;

                default:
                    btstack_assert(false);
                    break;
            }

            dump_sbc_configuration(&conn->sbc_configuration);
            break;
        }

        case A2DP_SUBEVENT_STREAM_ESTABLISHED: {
            uint8_t status = a2dp_subevent_stream_established_get_status(packet);

            if (status != ERROR_CODE_SUCCESS) {
                printf("A2DP: streaming connection failed, status 0x%02x\n", status);
                break;
            }

            a2dp_subevent_stream_established_get_bd_addr(packet, conn->addr);
            conn->a2dp_cid        = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            conn->a2dp_local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
            conn->stream_state    = STREAM_STATE_OPEN;
            printf("A2DP: stream established with %s (cid 0x%02x)\n",
                   bd_addr_to_str(conn->addr), conn->a2dp_cid);
            break;
        }

        case A2DP_SUBEVENT_STREAM_STARTED:
            printf("A2DP: stream started\n");
            conn->stream_state = STREAM_STATE_PLAYING;

            if (conn->sbc_configuration.reconfigure) {
                audio_media_processing_close();
            }

            audio_media_processing_init(&conn->sbc_configuration);
            break;

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            printf("A2DP: stream paused\n");
            conn->stream_state = STREAM_STATE_PAUSED;
            audio_media_processing_pause();
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            printf("A2DP: stream released\n");
            conn->stream_state = STREAM_STATE_CLOSED;
            audio_media_processing_close();
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            // Source disconnected entirely. media_processing_close() is
            // idempotent, so calling it after a STREAM_RELEASED earlier
            // in this sequence is fine.
            printf("A2DP: signaling connection released\n");
            conn->a2dp_cid = 0;
            audio_media_processing_close();
            break;

        default:
            break;
    }
}

}  // namespace

void bt_setup() {
    l2cap_init();
    sdp_init();

    a2dp_sink_init();
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();

    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&audio_handle_l2cap_media_data_packet);

    avdtp_stream_endpoint_t * ep = a2dp_sink_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC,
        audio_sbc_codec_capabilities, audio_sbc_codec_capabilities_size,
        a2dp_sink_stream_endpoint_.media_sbc_codec_configuration,
        sizeof(a2dp_sink_stream_endpoint_.media_sbc_codec_configuration));
    btstack_assert(ep != NULL);
    a2dp_sink_stream_endpoint_.a2dp_local_seid = avdtp_local_seid(ep);

    avrcp_register_packet_handler(&avrcp_packet_handler);
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    // SDP records.
    memset(sdp_avdtp_sink_service_buffer_, 0, sizeof(sdp_avdtp_sink_service_buffer_));
    a2dp_sink_create_sdp_record(sdp_avdtp_sink_service_buffer_,
                                sdp_create_service_record_handle(),
                                AVDTP_SINK_FEATURE_MASK_SPEAKER, NULL, NULL);
    sdp_register_service(sdp_avdtp_sink_service_buffer_);

    memset(sdp_avrcp_controller_service_buffer_, 0, sizeof(sdp_avrcp_controller_service_buffer_));
    uint16_t controller_features = 1 << AVRCP_CONTROLLER_SUPPORTED_FEATURE_CATEGORY_PLAYER_OR_RECORDER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer_,
                                       sdp_create_service_record_handle(),
                                       controller_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_service_buffer_);

    memset(sdp_avrcp_target_service_buffer_, 0, sizeof(sdp_avrcp_target_service_buffer_));
    uint16_t target_features = 1 << AVRCP_TARGET_SUPPORTED_FEATURE_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer_,
                                   sdp_create_service_record_handle(),
                                   target_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer_);

    memset(device_id_sdp_service_buffer_, 0, sizeof(device_id_sdp_service_buffer_));
    device_id_create_sdp_record(device_id_sdp_service_buffer_,
                                sdp_create_service_record_handle(),
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH,
                                BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer_);

    // GAP -- discoverability and pairing UX.
    gap_set_local_name("2B Speaker");
    gap_discoverable_control(1);
    gap_connectable_control(1);
    gap_set_class_of_device(0x240414);  // Audio / Loudspeaker

    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(1);

    hci_event_callback_registration_.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration_);
}

void bt_set_volume_policy_ignore(bool ignore) {
    if (ignore == volume_policy_ignore_) {
        return;
    }

    volume_policy_ignore_     = ignore;
    volume_policy_tick_count_ = 0;

    if (ignore) {
        // Engage immediately: force DSP passthrough and tell the phone
        // its slider is at 127 (no wait for the next periodic tick).
        dsp_set_avrcp_volume_127(127);

        if (avrcp_state_.avrcp_cid != 0) {
            avrcp_target_volume_changed(avrcp_state_.avrcp_cid, 127);
        }
    }
    // No "disengage" cleanup: the phone is free to send a new volume
    // and we'll honour it next time it does.
}

void bt_volume_policy_tick() {
    if (!volume_policy_ignore_ || avrcp_state_.avrcp_cid == 0) {
        return;
    }

    // Re-assert 127 on a slow cadence (5 s at the 50 ms pot timer) so
    // any drift / reset on the phone side gets corrected.
    if (++volume_policy_tick_count_ < kVolumePolicyTicksPerAssertion) {
        return;
    }

    volume_policy_tick_count_ = 0;
    avrcp_target_volume_changed(avrcp_state_.avrcp_cid, 127);
}
