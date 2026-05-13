#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H

// BTstack features that can be enabled
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

#ifdef ENABLE_CLASSIC
#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE
#endif

// BTstack configuration. buffers, sizes, ...
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_AVDTP_CONNECTIONS 1
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 1
#define MAX_NR_AVRCP_CONNECTIONS 2
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES  2
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_CHANNELS  4
#define MAX_NR_L2CAP_SERVICES  3
#define MAX_NR_RFCOMM_CHANNELS 1
#define MAX_NR_RFCOMM_MULTIPLEXERS 1
#define MAX_NR_RFCOMM_SERVICES 1
#define MAX_NR_SERVICE_RECORD_ITEMS 4
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 16

// Limit number of ACL/SCO Buffers used by the stack to avoid cyw43 shared bus overrun
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// Enable and configure HCI Controller to Host Flow Control to avoid cyw43 shared bus overrun
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// Link Key DB using TLV on top of Flash Sector interface
#define NVM_NUM_LINK_KEYS 16

// BTstack HAL configuration
#define HAVE_EMBEDDED_TIME_MS

// map btstack_assert onto Pico SDK assert()
#define HAVE_ASSERT

#define HCI_RESET_RESEND_TIMEOUT_MS 1000

#define ENABLE_SOFTWARE_AES128

#endif // _PICO_BTSTACK_BTSTACK_CONFIG_H
