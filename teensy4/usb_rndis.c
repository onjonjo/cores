/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdbool.h>
#include "usb_dev.h"
#include "usb_desc.h"
#include "usb_rndis_internal.h"
#include "core_pins.h"// for delay()
#include <string.h> // for memcpy()
#include "avr/pgmspace.h" // for PROGMEM, DMAMEM, FASTRUN

#include "debug/printf.h"
#include "util/fifo.h"
#include "core_pins.h"

// defined by usb_dev.h -> usb_desc.h
#if defined(USB_RNDIS)

static const uint32_t OIDSupportedList[] = {
/* Required General */
OID_GEN_SUPPORTED_LIST,
OID_GEN_HARDWARE_STATUS,
OID_GEN_MEDIA_SUPPORTED,
OID_GEN_MEDIA_IN_USE,
OID_GEN_MAXIMUM_FRAME_SIZE,
OID_GEN_LINK_SPEED,
OID_GEN_TRANSMIT_BLOCK_SIZE,
OID_GEN_RECEIVE_BLOCK_SIZE,
OID_GEN_VENDOR_ID,
OID_GEN_VENDOR_DESCRIPTION,
OID_GEN_CURRENT_PACKET_FILTER,
OID_GEN_MAXIMUM_TOTAL_SIZE,
OID_GEN_MEDIA_CONNECT_STATUS,
OID_GEN_VENDOR_DRIVER_VERSION,
OID_GEN_PHYSICAL_MEDIUM,

/* Required Statistical */
OID_GEN_XMIT_OK,
OID_GEN_RCV_OK,
OID_GEN_XMIT_ERROR,
OID_GEN_RCV_ERROR,
OID_GEN_RCV_NO_BUFFER,

/* Please configure us        */
OID_GEN_RNDIS_CONFIG_PARAMETER,

/* IEEE 802.3 (Ethernet) OIDs */
OID_802_3_PERMANENT_ADDRESS,
OID_802_3_CURRENT_ADDRESS,
OID_802_3_MULTICAST_LIST,
OID_802_3_MAXIMUM_LIST_SIZE,
OID_802_3_MAC_OPTIONS,
OID_802_3_RCV_ERROR_ALIGNMENT,
OID_802_3_XMIT_ONE_COLLISION,
OID_802_3_XMIT_MORE_COLLISIONS,

/* Minimum power managment needed for USB */

OID_PNP_CAPABILITIES,
OID_PNP_QUERY_POWER,
OID_PNP_SET_POWER,

OID_PNP_ENABLE_WAKE_UP,
OID_PNP_ADD_WAKE_UP_PATTERN,
OID_PNP_REMOVE_WAKE_UP_PATTERN, };

#define OID_LIST_LENGTH sizeof(OIDSupportedList)/sizeof(*OIDSupportedList)

//#if F_CPU >= 20000000

// At very slow CPU speeds, the OCRAM just isn't fast enough for
// USB to work reliably.  But the precious/limited DTCM is.  So
// as an ugly workaround, undefine DMAMEM so all buffers which
// would normally be allocated in OCRAM are placed in DTCM.
#if defined(F_CPU) && F_CPU < 30000000
#undef DMAMEM
#define DMAMEM
#endif

extern volatile uint8_t usb_high_speed;
extern volatile uint8_t usb_configuration; // non-zero when USB host as configured device

#define MAX_RNDIS_DATA_BYTES (3 * 512)
#define MAX_USB_PACKETS_PER_RNDIS_DATA ((3 * 512) / CDC_TX_SIZE_12)
#define RX_NUM_RNDIS_PACKETS  8

struct rndis_data {
	uint16_t num_usb_packets;
	uint16_t usb_packets[MAX_USB_PACKETS_PER_RNDIS_DATA];
	size_t len;
	size_t read_offset;
};

static struct rndis_data rndis_data_rx_pool[RX_NUM_RNDIS_PACKETS];
static struct fifo_cnt rndis_data_rx_fifo =
		{ .capacity = RX_NUM_RNDIS_PACKETS, };

static transfer_t rx_transfer[RX_NUM_RNDIS_PACKETS
		* MAX_USB_PACKETS_PER_RNDIS_DATA] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[RX_NUM_RNDIS_PACKETS * MAX_RNDIS_DATA_BYTES] __attribute__ ((aligned(32)));
static uint16_t rx_packet_size = 0;
static size_t num_rx_transfers;

#define TX_NUM_RNDIS_PACKETS   4
static transfer_t tx_transfer[TX_NUM_RNDIS_PACKETS
		* MAX_USB_PACKETS_PER_RNDIS_DATA] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t txbuffer[TX_NUM_RNDIS_PACKETS * MAX_RNDIS_DATA_BYTES] __attribute__ ((aligned(32)));
static size_t num_tx_transfers;

static struct fifo_cnt tx_avail_fifo;
static struct fifo_cnt tx_used_fifo;

//   --- forward declarations ---

static void rx_queue_transfer(int i);
static void rx_event(transfer_t *t);
static void rndis_query_process(void);

//   --- end of forward declarations ---

static inline uint8_t* get_rx_buffer(uint16_t index) {
	return rx_buffer + (rx_packet_size * index);
}

void usb_rndis_configure(void) {
	int i;

	printf("usb_serial_configure\n");
	if (usb_high_speed) {
		rx_packet_size = CDC_RX_SIZE_480;
	} else {
		rx_packet_size = CDC_RX_SIZE_12;
	}

	num_rx_transfers = RX_NUM_RNDIS_PACKETS
			* (MAX_RNDIS_DATA_BYTES / rx_packet_size);
	memset(rx_transfer, 0, sizeof(rx_transfer));
	memset(&rndis_data_rx_fifo, 0, sizeof rndis_data_rx_fifo);
	rndis_data_rx_fifo = FIFO_INIT(RX_NUM_RNDIS_PACKETS);

	num_tx_transfers = TX_NUM_RNDIS_PACKETS
			* (MAX_RNDIS_DATA_BYTES / rx_packet_size);
	memset(tx_transfer, 0, sizeof(tx_transfer));
	tx_avail_fifo = FIFO_INIT(TX_NUM_RNDIS_PACKETS);
	tx_used_fifo = FIFO_INIT(TX_NUM_RNDIS_PACKETS);

	// all transfers are initially available
	fifo_add(&tx_avail_fifo, num_tx_transfers);

	usb_config_tx(CDC_ACM_ENDPOINT, CDC_ACM_SIZE, 0, NULL); // size same 12 & 480
	usb_config_rx(CDC_RX_ENDPOINT, rx_packet_size, 0, rx_event);
	usb_config_tx(CDC_TX_ENDPOINT, rx_packet_size, 1, NULL);
	for (i = 0; i < num_rx_transfers; i++)
		rx_queue_transfer(i);

}

/*************************************************************************/
/**                               Receive                               **/
/*************************************************************************/

static void rx_queue_transfer(int i) {
	NVIC_DISABLE_IRQ(IRQ_USB1);printf("rx queue i=%d\n", i);
	void *buffer = get_rx_buffer(i);
	usb_prepare_transfer(rx_transfer + i, buffer, rx_packet_size, i);
	arm_dcache_delete(buffer, rx_packet_size);
	usb_receive(CDC_RX_ENDPOINT, rx_transfer + i);
	NVIC_ENABLE_IRQ(IRQ_USB1);
}

static void rndis_data_clear(struct rndis_data *d) {

	for (uint16_t i = 0; i != d->num_usb_packets; i++) {
		// release USB buffers
		rx_queue_transfer(d->usb_packets[i]);
	}

	// reset data to make this ready for next use
	d->num_usb_packets = 0;
	d->len = 0;
	d->read_offset = 0;
}

// called by USB interrupt when any packet is received
static void rx_event(transfer_t *t) {
	int len = rx_packet_size - ((t->status >> 16) & 0x7FFF);
	int i = t->callback_param;
	printf("rx event, len=%d, i=%d\n", len, i);
	struct rndis_data *d = &rndis_data_rx_pool[fifo_write_index(
			&rndis_data_rx_fifo)];

	// append USB packet buffer
	d->len += len;
	d->usb_packets[d->num_usb_packets++] = i;

	if (len < rx_packet_size) {
		// rdnis frame complete
		fifo_add(&rndis_data_rx_fifo, 1);
		// if fifo is full, there shouldn't be any transfer buffers
	}
}

// copy linear range from all buffers
static int rndis_data_copy(const struct rndis_data *d, size_t offset, void *dst,
		size_t len) {

	if (offset + len > d->len)
		return -1;

	while (len) {
		size_t buffer = offset / rx_packet_size;
		size_t boffset = offset % rx_packet_size;
		size_t n = rx_packet_size - boffset;
		if (len < n)
			n = len;

		memcpy(dst, get_rx_buffer(buffer) + boffset, n);

		len -= n;
		offset += n;
	}

	return 0;
}

int usb_rndis_read(void *dst, size_t len) {

	int ret = -1;
	struct rndis_packet_message h;

	if (fifo_level(&rndis_data_rx_fifo) == 0) {
		return -1;
	}
	struct rndis_data *d = &rndis_data_rx_pool[fifo_read_index(
			&rndis_data_rx_fifo)];

	if (rndis_data_copy(d, d->read_offset, &h, sizeof(h)) != 0) {
		goto error;
	}

	if (h.h.message_type != RNDIS_MESSAGE_TYPE_PACKET_DATA) {
		goto error;
	}

	if (h.h.message_length > d->read_offset + d->len) {
		goto error;
	}

	size_t data_start = d->read_offset + h.data_offset
			+ offsetof(struct rndis_packet_message, data_offset);
	if (data_start > d->len || data_start + h.data_len > d->len) {
		goto error;
	}

	size_t min_len = h.data_len;
	if (len < min_len)
		min_len = len;

	if (rndis_data_copy(d, data_start, dst, len) != 0) { // copy payload
		goto error;
	}
	ret = len;

	d->read_offset += h.h.message_length; // move to next message

	if (d->read_offset + sizeof(h) < d->len) {
		// there can be more messages
		return ret; // stop before removing the current entry
	}

	error: // we are done with this one
	fifo_remove(&rndis_data_rx_fifo, 1);
	rndis_data_clear(d); // release this message buffer for more incoming data
	return ret;
}

/*************************************************************************/
/**                               Transmit                              **/
/*************************************************************************/

int rndis_write(const void *data, size_t len) {

	if (!usb_configuration)
		return 0;


	size_t message_len = len + sizeof(struct rndis_packet_message);
	size_t transfers_required = (message_len + rx_packet_size - 1)
			/ rx_packet_size;

	// try to release the required number of elements
	size_t avail = fifo_level(&tx_avail_fifo);

	if (avail < transfers_required) {
		// insufficient transfer buffers.
		size_t diff = transfers_required - avail;

		for (size_t i = 0; i != diff; i++) {

			if (fifo_level(&tx_used_fifo) == 0) {
				return -1;
			}

			size_t read_index = fifo_read_index(&tx_used_fifo);
			transfer_t *trans = &tx_transfer[read_index];
			uint32_t status = usb_transfer_status(trans);
			if (!(status & 0x80)) {
				if (status & 0x68) {
					// TODO: what if status has errors???
					printf("ERROR status = %x, i=%d, ms=%u\n",
							status, tx_head, systick_millis_count);
				}
				// move this element from used-fifo to avail-fifo
				fifo_remove(&tx_used_fifo, 1);
				fifo_add(&tx_avail_fifo, 1);
			} else {
				return -1;
			}
		}
	}

	// now we should have enough entries in tx_avail_fifo

	bool add_header = true;
	while (len) {
		size_t buf_index = fifo_read_index(&tx_avail_fifo);
		fifo_remove(&tx_avail_fifo, 1);
		fifo_add(&tx_used_fifo, 1);

		uint8_t *txbuf = txbuffer + (buf_index * rx_packet_size);
		transfer_t *trans = &tx_transfer[buf_index];

		void *p = txbuf;
		size_t p_cap = rx_packet_size;

		if (add_header) {
			struct rndis_packet_message *h = p;
			h->h.message_type = RNDIS_MESSAGE_TYPE_PACKET_DATA;
			h->h.message_length = sizeof(struct rndis_packet_message) + len;
			h->data_offset = sizeof(struct rndis_packet_message)
					- sizeof(struct rndis_message_header);
			h->data_len = len;
			h->oobdata_offset = 0;
			h->oobdata_len = 0;
			h->per_packet_info_offset = 0;
			h->per_packet_info_len = 0;
			h->vc_handle = 0;
			h->reserved = 0;
			p = h + 1; // data begins behind header
			p_cap -= sizeof(struct rndis_packet_message);
			add_header = false;
		}

		size_t chunk_size = len;
		if (p_cap < chunk_size)
			chunk_size = p_cap;

		memcpy(p, data, chunk_size);
		data += chunk_size;
		p += chunk_size;
		len -= chunk_size;
		size_t packet_len = (uint8_t *)p - txbuf;

		usb_prepare_transfer(trans, txbuf, packet_len, 0);
		arm_dcache_flush_delete(txbuf, packet_len);
		usb_transmit(CDC_TX_ENDPOINT, trans);
	}

	return 0;
}

void rndis_packetFilter(uint32_t newfilter);

/******** RNDIS ********/

#define ENC_BUF_SIZE    (OID_LIST_LENGTH + 8) // in u32

// Command buffer
DMAMEM uint32_t encapsulated_buffer[ENC_BUF_SIZE];

//Do we have data to send back?
bool data_to_send = false;
bool rndis_initialized = false;

/**
 * \brief Handles a "SEND ENCAPSULATED COMMAND" message.
 *
 * \return True on success, false on failure.
 */
bool rndis_send_encapsulated_command(void) {

	struct rndis_message_header *header =
			(struct rndis_message_header*) encapsulated_buffer;
	switch (header->message_type) {
	/* Requests remote intilization. Respond with complete,
	 eventually should probably do something */
	case RNDIS_MESSAGE_TYPE_INITIALIZE: {

		struct rndis_initalize_complete_message *m;
		m = (struct rndis_initalize_complete_message*) encapsulated_buffer;

		//m->MessageID is same as before
		m->h.message_type = RNDIS_MESSAGE_TYPE_INITIALIZE_COMPLETE;
		m->h.message_length = sizeof(struct rndis_initalize_complete_message);
		m->major_version = 1;
		m->minor_version = 0;
		m->status = 0 /* RNDIS_STATUS_SUCCESS */;
		m->device_flags = 0x10;
		m->medium = 0;
		m->max_packets_per_message = 1;
		m->max_transfer_size = 3 * 512; // TODO: make this one better
		m->packet_alignment_factor = 3;
		m->reserved_AFListOffset = 0;
		m->reserved_AFListSize = 0;
		rndis_initialized = true;

		data_to_send = true;
	}
		break;
	case RNDIS_MESSAGE_TYPE_HALT:
		break;

	case RNDIS_MESSAGE_TYPE_QUERY:
		rndis_query_process();
		break;

	case RNDIS_MESSAGE_TYPE_SET: {
		rndis_set_process();
	}
		break;

	case RNDIS_MESSAGE_TYPE_RESET: {
		struct rndis_reset_complete_message *m;
		m = (struct rndis_reset_complete_message*) encapsulated_buffer;

		rndis_initialized = false;

		m->h.message_type = REMOTE_NDIS_RESET_CMPLT;
		m->h.message_length = sizeof(*m);
		m->status = 0;
		m->addressing_reset = 1;

		data_to_send = true;
	}
		break;

	case RNDIS_MESSAGE_TYPE_KEEPALIVE: {
		struct rndis_keepalive_complete_message *m;
		m = (struct rndis_keepalive_complete_message*) encapsulated_buffer;
		m->h.message_type = REMOTE_NDIS_KEEPALIVE_CMPLT;
		m->h.message_length = sizeof(*m);
		m->status = 0;
		data_to_send = true;
		break;
	}

	default:
		return false;
	}

	rndis_send_interrupt();

	return true;
}

/**
 * \brief Send an interrupt over the interrupt endpoint to the host.
 */
void rndis_send_interrupt(void) {

	//Schedule the interrupt to take place next
	//time USB task is run
	schedule_interrupt = 1;
}



uint32_t oid_packet_filter = 0x0000000;

/**
 * \brief Function to handle a RNDIS "QUERY" command in the encapsulated_buffer
 */
static void rndis_query_process(void) {
	struct rndis_query_message *m;
	struct rndis_query_complete_message *c;
	rndis_Status_t status = RNDIS_STATUS_SUCCESS;

	m = (struct rndis_query_message*) encapsulated_buffer;
	c = (struct rndis_query_complete_message*) encapsulated_buffer;

	uint32_t oid = m->oid;
	/* We set up packet for sending one 4-byte response, which a lot of
	 these will do. If you need more or less just change the defaults in
	 the specific case */

	c->h.message_type = REMOTE_NDIS_QUERY_CMPLT;
	c->status = 0;
	c->info_buffer_length = 4;
	c->info_buffer_offset = 16; // relative to reqid
	uint32_t *info_buf = (uint32_t*) (encapsulated_buffer + sizeof(*c));

	switch (m->oid) {

	/**** GENERAL ****/
	case OID_GEN_SUPPORTED_LIST:
		c->info_buffer_length = 4 * OID_LIST_LENGTH;
		//Copy data to SRAM
		memcpy(info_buf, OIDSupportedList, c->info_buffer_length);
		break;

	case OID_GEN_HARDWARE_STATUS:
		*info_buf = 0x00000000; /* Ready and Willing */
		break;

	case OID_GEN_MEDIA_SUPPORTED:
	case OID_GEN_MEDIA_IN_USE:
	case OID_GEN_PHYSICAL_MEDIUM:
		*info_buf = NDIS_MEDIUM_802_3;
		break;

	case OID_GEN_MAXIMUM_FRAME_SIZE:
		*info_buf = (uint32_t) USB_ETH_MTU - 14; //1280 //102; /* Assume 25 octet header on 15.4 */
		break;

	case OID_GEN_LINK_SPEED:
		*info_buf = 250000 / 100; /* in 100 bytes/sec units.. this is kinda a lie */
		break;

	case OID_GEN_TRANSMIT_BLOCK_SIZE:
	case OID_GEN_RECEIVE_BLOCK_SIZE:
		*info_buf = (uint32_t) 102;
		break;

	case OID_GEN_VENDOR_ID:
		*info_buf = 0xFFFFFF; /* No vendor ID ! */
		break;

	case OID_GEN_VENDOR_DESCRIPTION:
		c->InformationBufferLength = 8;
		memcpy(info_buf, "Teensy\0\0", 8);
		break;

	case OID_GEN_CURRENT_PACKET_FILTER:
		*info_buf = oid_packet_filter;
		break;

	case OID_GEN_MAXIMUM_TOTAL_SIZE:
		*info_buf = (uint32_t) USB_ETH_MTU; //127;
		break;

	case OID_GEN_MEDIA_CONNECT_STATUS:
		*info_buf = usb_eth_is_active ?
		NDIS_MEDIA_STATE_CONNECTED :
										NDIS_MEDIA_STATE_DISCONNECTED;
		break;

	case OID_GEN_VENDOR_DRIVER_VERSION:
		*info_buf = 0x00001000;
		break;

	case OID_GEN_CURRENT_LOOKAHEAD:
		*info_buf = (uint32_t) USB_ETH_MTU - 14; //102;

//		case OID_GEN_RNDIS_CONFIG_PARAMETER:
//			break;

		/******* 802.3 (Ethernet) *******/

		/*The address of the NIC encoded in the hardware.*/
	case OID_802_3_PERMANENT_ADDRESS:
	case OID_802_3_CURRENT_ADDRESS:

		//Clear unused bytes
		info_buf[1] = 0;

		//get address
		memcpy(info_buf, MAC_ADDRESS, 6)

		c->InformationBufferLength = 6;
		break;

		/* The multicast address list on the NIC enabled for packet reception. */
	case OID_802_3_MULTICAST_LIST:
		*info_buf = 0xE000000;
		break;

		/* The maximum number of multicast addresses the NIC driver can manage. */
	case OID_802_3_MAXIMUM_LIST_SIZE:
		*info_buf = 1;
		break;

		/* Features supported by the underlying driver, which could be emulating Ethernet. */
	case OID_802_3_MAC_OPTIONS:
		*info_buf = 0;
		break;

		/* Frames received with alignment error */
	case OID_802_3_RCV_ERROR_ALIGNMENT:
		/* Frames transmitted with one collision */
	case OID_802_3_XMIT_ONE_COLLISION:
		/* Frames transmitted with more than one collision */
	case OID_802_3_XMIT_MORE_COLLISIONS:
		*info_buf = 0;
		break;

		/*** Statistical ***/

		/* Frames transmitted without errors */
	case OID_GEN_XMIT_OK:
		*info_buf = 0;
		break;

		/* Frames received without errors */
	case OID_GEN_RCV_OK:
		*info_buf = 0;
		break;

		/* Frames received with errors */
	case OID_GEN_RCV_ERROR:
		*info_buf = 0;
		break;

		/* Frames transmitted with errors */
	case OID_GEN_XMIT_ERROR:
		*info_buf = 0;
		break;

		/* Frames dropped due to lack of buffer space */
	case OID_GEN_RCV_NO_BUFFER:

		*info_buf = 0; /* Lies! */
		break;

		/*** Power Managment ***/
	case OID_PNP_CAPABILITIES:
		c->InformationBufferLength =
				sizeof(struct NDIS_PM_WAKE_UP_CAPABILITIES);
		memset(info_buf, 0, sizeof(struct NDIS_PM_WAKE_UP_CAPABILITIES));
		break;

	case OID_PNP_QUERY_POWER:
		c->InformationBufferLength = 0;
		break;

	case OID_PNP_ENABLE_WAKE_UP:
		*info_buf = 0; /* Nothing Supported */
		break;

	default:
		c->status = 1;
		c->info_buffer_length = 0;
		break;
	}

	//Calculate message size
	c->h.message_length = sizeof(*c) + c->InformationBufferLength;

	//Check if we are sending no information buffer
	if (c->info_buffer_length == 0) {
		c->info_buffer_offset = 0;
	}

	data_to_send = true;
}

/**
 * \brief Function to deal with a RNDIS "SET" command present in the
 *        encapsulated_buffer
 */
void rndis_set_process(void) {
	rndis_set_cmplt_t *c;
	rndis_set_msg_t *m;

	c = ((rndis_set_cmplt_t*) encapsulated_buffer);
	m = ((rndis_set_msg_t*) encapsulated_buffer);



	switch (m->Oid) {

	/* Parameters set up in 'Advanced' tab */
	case OID_GEN_RNDIS_CONFIG_PARAMETER:
		/* Parameter name: rawmode
		 Parameter desc: Enables or disable raw capture of 802.15.4 Packets
		 Parameter type: single octet
		 Parameter values: '0' = disabled, '1' = enabled
		 */
		rndis_handle_config_parm(parmname, PARMVALUE, PARMVALUELENGTH);
		break;

		/* Mandatory general OIDs */
	case OID_GEN_CURRENT_PACKET_FILTER:
		oid_packet_filter = *INFBUF;

		if (oid_packet_filter) {

			rndis_packetFilter(oid_packet_filter);

			rndis_state = rndis_data_initialized;
		} else {
			rndis_state = rndis_initialized;
		}

		break;

	case OID_GEN_CURRENT_LOOKAHEAD:
		break;

	case OID_GEN_PROTOCOL_OPTIONS:
		break;

		/* Mandatory 802_3 OIDs */
	case OID_802_3_MULTICAST_LIST:
		break;

		/* Power Managment: fails for now */
	case OID_PNP_ADD_WAKE_UP_PATTERN:
	case OID_PNP_REMOVE_WAKE_UP_PATTERN:
	case OID_PNP_ENABLE_WAKE_UP:

	default:
		//c->MessageID is same as before
		c->MessageType = REMOTE_NDIS_SET_CMPLT;
		c->MessageLength = sizeof(rndis_set_cmplt_t);
		c->Status = RNDIS_STATUS_FAILURE;
		data_to_send = c->MessageLength;
		return;

		break;
	}

	//c->MessageID is same as before
	c->MessageType = REMOTE_NDIS_SET_CMPLT;
	c->MessageLength = sizeof(rndis_set_cmplt_t);
	c->Status = RNDIS_STATUS_SUCCESS;
	data_to_send = c->MessageLength;
	return;
}


#endif
