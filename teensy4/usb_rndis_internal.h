#pragma once

#include <stdint.h>

#define RNDIS_STATUS_SUCCESS              0x00000000
#define RNDIS_STATUS_ERROR                0xC0000001
#define NDIS_STATUS_MULTICAST_FULL	      0xC0010009
#define NDIS_STATUS_MULTICAST_EXISTS      0xC001000A
#define NDIS_STATUS_MULTICAST_NOT_FOUND   0xC001000B

/* from drivers/net/sk98lin/h/skgepnmi.h */
#define OID_PNP_CAPABILITIES                    0xFD010100
#define OID_PNP_SET_POWER                       0xFD010101
#define OID_PNP_QUERY_POWER                     0xFD010102
#define OID_PNP_ADD_WAKE_UP_PATTERN             0xFD010103
#define OID_PNP_REMOVE_WAKE_UP_PATTERN          0xFD010104
#define OID_PNP_ENABLE_WAKE_UP                  0xFD010106

enum NDIS_DEVICE_POWER_STATE {
	NdisDeviceStateUnspecified = 0,
	NdisDeviceStateD0,
	NdisDeviceStateD1,
	NdisDeviceStateD2,
	NdisDeviceStateD3,
	NdisDeviceStateMaximum
};

struct NDIS_PM_WAKE_UP_CAPABILITIES {
	enum NDIS_DEVICE_POWER_STATE  MinMagicPacketWakeUp;
	enum NDIS_DEVICE_POWER_STATE  MinPatternWakeUp;
	enum NDIS_DEVICE_POWER_STATE  MinLinkChangeWakeUp;
};

/* NDIS_PNP_CAPABILITIES.Flags constants */
#define NDIS_DEVICE_WAKE_UP_ENABLE                0x00000001
#define NDIS_DEVICE_WAKE_ON_PATTERN_MATCH_ENABLE  0x00000002
#define NDIS_DEVICE_WAKE_ON_MAGIC_PACKET_ENABLE   0x00000004

/* Required Object IDs (OIDs) */
#define OID_GEN_SUPPORTED_LIST            0x00010101
#define OID_GEN_HARDWARE_STATUS           0x00010102
#define OID_GEN_MEDIA_SUPPORTED           0x00010103
#define OID_GEN_MEDIA_IN_USE              0x00010104
#define OID_GEN_MAXIMUM_LOOKAHEAD         0x00010105
#define OID_GEN_MAXIMUM_FRAME_SIZE        0x00010106
#define OID_GEN_LINK_SPEED                0x00010107
#define OID_GEN_TRANSMIT_BUFFER_SPACE     0x00010108
#define OID_GEN_RECEIVE_BUFFER_SPACE      0x00010109
#define OID_GEN_TRANSMIT_BLOCK_SIZE       0x0001010A
#define OID_GEN_RECEIVE_BLOCK_SIZE        0x0001010B
#define OID_GEN_VENDOR_ID                 0x0001010C
#define OID_GEN_VENDOR_DESCRIPTION        0x0001010D
#define OID_GEN_CURRENT_PACKET_FILTER     0x0001010E
#define OID_GEN_CURRENT_LOOKAHEAD         0x0001010F
#define OID_GEN_DRIVER_VERSION            0x00010110
#define OID_GEN_MAXIMUM_TOTAL_SIZE        0x00010111
#define OID_GEN_PROTOCOL_OPTIONS          0x00010112
#define OID_GEN_MAC_OPTIONS               0x00010113
#define OID_GEN_MEDIA_CONNECT_STATUS      0x00010114
#define OID_GEN_MAXIMUM_SEND_PACKETS      0x00010115
#define OID_GEN_VENDOR_DRIVER_VERSION     0x00010116
#define OID_GEN_SUPPORTED_GUIDS           0x00010117
#define OID_GEN_NETWORK_LAYER_ADDRESSES   0x00010118
#define OID_GEN_TRANSPORT_HEADER_OFFSET   0x00010119
#define OID_GEN_MACHINE_NAME              0x0001021A
#define OID_GEN_RNDIS_CONFIG_PARAMETER    0x0001021B
#define OID_GEN_VLAN_ID                   0x0001021C

/* Optional OIDs */
#define OID_GEN_MEDIA_CAPABILITIES        0x00010201
#define OID_GEN_PHYSICAL_MEDIUM           0x00010202

/* Required statistics OIDs */
#define OID_GEN_XMIT_OK                   0x00020101
#define OID_GEN_RCV_OK                    0x00020102
#define OID_GEN_XMIT_ERROR                0x00020103
#define OID_GEN_RCV_ERROR                 0x00020104
#define OID_GEN_RCV_NO_BUFFER             0x00020105

/* Optional statistics OIDs */
#define OID_GEN_DIRECTED_BYTES_XMIT       0x00020201
#define OID_GEN_DIRECTED_FRAMES_XMIT      0x00020202
#define OID_GEN_MULTICAST_BYTES_XMIT      0x00020203
#define OID_GEN_MULTICAST_FRAMES_XMIT     0x00020204
#define OID_GEN_BROADCAST_BYTES_XMIT      0x00020205
#define OID_GEN_BROADCAST_FRAMES_XMIT     0x00020206
#define OID_GEN_DIRECTED_BYTES_RCV        0x00020207
#define OID_GEN_DIRECTED_FRAMES_RCV       0x00020208
#define OID_GEN_MULTICAST_BYTES_RCV       0x00020209
#define OID_GEN_MULTICAST_FRAMES_RCV      0x0002020A
#define OID_GEN_BROADCAST_BYTES_RCV       0x0002020B
#define OID_GEN_BROADCAST_FRAMES_RCV      0x0002020C
#define OID_GEN_RCV_CRC_ERROR             0x0002020D
#define OID_GEN_TRANSMIT_QUEUE_LENGTH     0x0002020E
#define OID_GEN_GET_TIME_CAPS             0x0002020F
#define OID_GEN_GET_NETCARD_TIME          0x00020210
#define OID_GEN_NETCARD_LOAD              0x00020211
#define OID_GEN_DEVICE_PROFILE            0x00020212
#define OID_GEN_INIT_TIME_MS              0x00020213
#define OID_GEN_RESET_COUNTS              0x00020214
#define OID_GEN_MEDIA_SENSE_COUNTS        0x00020215
#define OID_GEN_FRIENDLY_NAME             0x00020216
#define OID_GEN_MINIPORT_INFO             0x00020217
#define OID_GEN_RESET_VERIFY_PARAMETERS   0x00020218

/* IEEE 802.3 (Ethernet) OIDs */
#define NDIS_802_3_MAC_OPTION_PRIORITY    0x00000001

#define OID_802_3_PERMANENT_ADDRESS       0x01010101
#define OID_802_3_CURRENT_ADDRESS         0x01010102
#define OID_802_3_MULTICAST_LIST          0x01010103
#define OID_802_3_MAXIMUM_LIST_SIZE       0x01010104
#define OID_802_3_MAC_OPTIONS             0x01010105
#define OID_802_3_RCV_ERROR_ALIGNMENT     0x01020101
#define OID_802_3_XMIT_ONE_COLLISION      0x01020102
#define OID_802_3_XMIT_MORE_COLLISIONS    0x01020103
#define OID_802_3_XMIT_DEFERRED           0x01020201
#define OID_802_3_XMIT_MAX_COLLISIONS     0x01020202
#define OID_802_3_RCV_OVERRUN             0x01020203
#define OID_802_3_XMIT_UNDERRUN           0x01020204
#define OID_802_3_XMIT_HEARTBEAT_FAILURE  0x01020205
#define OID_802_3_XMIT_TIMES_CRS_LOST     0x01020206
#define OID_802_3_XMIT_LATE_COLLISIONS    0x01020207

#define NDIS_MEDIUM_802_3		0x00000000
#define NDIS_MEDIUM_802_5		0x00000001
#define NDIS_MEDIUM_FDDI		0x00000002
#define NDIS_MEDIUM_WAN			0x00000003
#define NDIS_MEDIUM_LOCAL_TALK		0x00000004
#define NDIS_MEDIUM_DIX			0x00000005
#define NDIS_MEDIUM_ARCENT_RAW		0x00000006
#define NDIS_MEDIUM_ARCENT_878_2	0x00000007
#define NDIS_MEDIUM_ATM			0x00000008
#define NDIS_MEDIUM_WIRELESS_LAN	0x00000009
#define NDIS_MEDIUM_IRDA		0x0000000A
#define NDIS_MEDIUM_BPC			0x0000000B
#define NDIS_MEDIUM_CO_WAN		0x0000000C
#define NDIS_MEDIUM_1394		0x0000000D

#define NDIS_PACKET_TYPE_DIRECTED	0x00000001
#define NDIS_PACKET_TYPE_MULTICAST	0x00000002
#define NDIS_PACKET_TYPE_ALL_MULTICAST	0x00000004
#define NDIS_PACKET_TYPE_BROADCAST	0x00000008
#define NDIS_PACKET_TYPE_SOURCE_ROUTING	0x00000010
#define NDIS_PACKET_TYPE_PROMISCUOUS	0x00000020
#define NDIS_PACKET_TYPE_SMT		0x00000040
#define NDIS_PACKET_TYPE_ALL_LOCAL	0x00000080
#define NDIS_PACKET_TYPE_GROUP		0x00000100
#define NDIS_PACKET_TYPE_ALL_FUNCTIONAL	0x00000200
#define NDIS_PACKET_TYPE_FUNCTIONAL	0x00000400
#define NDIS_PACKET_TYPE_MAC_FRAME	0x00000800

#define NDIS_MEDIA_STATE_CONNECTED	0x00000000
#define NDIS_MEDIA_STATE_DISCONNECTED	0x00000001

#define NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA     0x00000001
#define NDIS_MAC_OPTION_RECEIVE_SERIALIZED      0x00000002
#define NDIS_MAC_OPTION_TRANSFERS_NOT_PEND      0x00000004
#define NDIS_MAC_OPTION_NO_LOOPBACK             0x00000008
#define NDIS_MAC_OPTION_FULL_DUPLEX             0x00000010
#define NDIS_MAC_OPTION_EOTX_INDICATION         0x00000020
#define NDIS_MAC_OPTION_8021P_PRIORITY          0x00000040
#define NDIS_MAC_OPTION_RESERVED                0x80000000


// RNDIS spec: https://docs.microsoft.com/en-us/previous-versions/ff570635(v=vs.85)

enum {
	RNDIS_MESSAGE_TYPE_PACKET_DATA = 0x01,
	RNDIS_MESSAGE_TYPE_INITIALIZE = 0x02,
	RNDIS_MESSAGE_TYPE_HALT = 0x03,
	RNDIS_MESSAGE_TYPE_QUERY = 0x04,
	RNDIS_MESSAGE_TYPE_SET = 0x05,
	RNDIS_MESSAGE_TYPE_RESET = 0x06,
	RNDIS_MESSAGE_TYPE_INDICATE = 0x07,
	RNDIS_MESSAGE_TYPE_KEEPALIVE = 0x08,

	RNDIS_MESSAGE_TYPE_INITIALIZE_COMPLETE = 0x80000002,
	RNDIS_MESSAGE_TYPE_QUERY_COMPLETE = 0x80000004,
	RNDIS_MESSAGE_TYPE_SET_COMPLETE = 0x80000005,
	RNDIS_MESSAGE_TYPE_RESET_COMPLETE = 0x80000006,
	RNDIS_MESSAGE_TYPE_KEEPALIVE_COMPLETE = 0x80000008,
};

struct rndis_message_header {
	uint32_t message_type;
	uint32_t message_length;
};

struct rndis_initalize_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t major_version;
	uint32_t minor_version;
	uint32_t max_transfer_size;
};

struct rndis_initalize_complete_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t status;
	uint32_t major_version;
	uint32_t minor_version;
	uint32_t device_flags;
	uint32_t medium;
	uint32_t max_packets_per_message;
	uint32_t max_transfer_size;
	uint32_t packet_alignment_factor;
	uint32_t reserved_AFListOffset;
	uint32_t reserved_AFListSize;
};

struct rndis_packet_message {
	struct rndis_message_header h;
	uint32_t data_offset; // relative to the start of this field member
	uint32_t data_len;
	uint32_t oobdata_offset;
	uint32_t oobdata_len;
	uint32_t num_oobdata_elements;
	uint32_t per_packet_info_offset;
	uint32_t per_packet_info_len;
	uint32_t vc_handle;
	uint32_t reserved;
};

struct rndis_halt_message {
	struct rndis_message_header h;
	uint32_t req_id;
};

struct rndis_keepalive_message {
	struct rndis_message_header h;
	uint32_t req_id;
};

struct rndis_keepalive_complete_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t status;
};

struct rndis_query_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t oid;
	uint32_t info_buffer_length;
	uint32_t info_buffer_offset;
	uint32_t reserved;
};

struct rndis_query_complete_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t status;
	uint32_t info_buffer_length;
	uint32_t info_buffer_offset;
};

struct rndis_set_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t oid;
	uint32_t info_buffer_length;
	uint32_t info_buffer_offset;
	uint32_t reserved;
};

struct rndis_set_complete_message {
	struct rndis_message_header h;
	uint32_t req_id;
	uint32_t status;
};

struct rndis_reset_complete_message {
	struct rndis_message_header h;
	uint32_t status;
	uint32_t addressing_reset;
};
