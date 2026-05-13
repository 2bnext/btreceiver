// bt.hpp -- BTstack init: SDP / GAP / A2DP sink / AVRCP packet handlers.

#pragma once

// One-time setup. Initialises L2CAP / SDP / A2DP sink / AVRCP, registers
// every packet handler, publishes the SDP records, and configures GAP
// for "just works" pairing as a Bluetooth speaker.
//
// Call once from main(), after cyw43_arch_init() and after the audio
// sink instance has been registered with btstack_audio_sink_set_instance.
void bt_setup();

// Volume policy toggle. When true, incoming AVRCP set_absolute_volume
// commands from the phone are dropped (the local volume pot remains in
// charge), and we push 127 back to the phone so its slider parks at the
// top -- the phone then sends us full-resolution audio without any
// pre-attenuation. When false, the phone's slider scales the local
// AVRCP gain in dsp.cpp as usual.
void bt_set_volume_policy_ignore(bool ignore);

// Periodic re-assertion: when ignore mode is active and AVRCP is
// connected, push 127 to the phone so any drift in the slider is
// corrected. Cheap to call -- internally a no-op when not connected
// or when the policy is off. Called from the pot timer.
void bt_volume_policy_tick();
