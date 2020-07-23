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

#define USB_ETH_MTU 1440

uint8_t MAC_ADDRESS[6] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

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

int usb_eth_is_active = 1;

#define USB_PACKETS_PER_RNDIS_PACKET 4
#define MAX_RNDIS_DATA_BYTES (USB_PACKETS_PER_RNDIS_PACKET * 512)
#define MAX_USB_PACKETS_PER_RNDIS_DATA ((USB_PACKETS_PER_RNDIS_PACKET * 512) / RNDIS_TX_SIZE_12)
#define RX_NUM_RNDIS_PACKETS  8

struct rndis_data {
	uint16_t num_usb_packets;
	uint16_t usb_packets[MAX_USB_PACKETS_PER_RNDIS_DATA];
	size_t len;
	size_t read_offset;
};

static struct rndis_data rndis_data_rx_pool[RX_NUM_RNDIS_PACKETS];
static struct fifo_cnt rndis_data_rx_fifo = { .capacity = RX_NUM_RNDIS_PACKETS, };

static transfer_t rx_transfer[RX_NUM_RNDIS_PACKETS * MAX_USB_PACKETS_PER_RNDIS_DATA] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[RX_NUM_RNDIS_PACKETS * MAX_RNDIS_DATA_BYTES] __attribute__ ((aligned(32)));
static uint16_t usb_packet_size = 0;
static size_t num_rx_transfers;

#define TX_NUM_RNDIS_PACKETS   4
static transfer_t tx_transfer[TX_NUM_RNDIS_PACKETS * MAX_USB_PACKETS_PER_RNDIS_DATA] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t txbuffer[TX_NUM_RNDIS_PACKETS * MAX_RNDIS_DATA_BYTES] __attribute__ ((aligned(32)));
static size_t num_tx_transfers;

static struct fifo_cnt tx_transfer_fifo; // contains transfers in use

#define NUM_INT_XFERS 4
static transfer_t int_xfer[NUM_INT_XFERS] __attribute__ ((used, aligned(32)));
static size_t current_int_xfer;

//   --- forward declarations ---

static void rx_queue_transfer(int i);
static void rx_event(transfer_t *t);
static void rndis_query_process(void);
static void rndis_set_process(void);
void rndis_send_interrupt(void);

//   --- end of forward declarations ---

static inline uint8_t* get_rx_buffer(uint16_t index) {
	return rx_buffer + (usb_packet_size * index);
}

void usb_rndis_configure(void) {
	int i;

	printf("usb_rndis_configure\n");
	MAC_ADDRESS[0] = (HW_OCOTP_MAC1 >> 8) & 0xFE;
	MAC_ADDRESS[1] = HW_OCOTP_MAC1 >> 0;
	MAC_ADDRESS[2] = HW_OCOTP_MAC0 >> 24;
	MAC_ADDRESS[4] = HW_OCOTP_MAC0 >> 8;
	MAC_ADDRESS[5] = HW_OCOTP_MAC0 >> 8;

	if (usb_high_speed) {
		usb_packet_size = RNDIS_RX_SIZE_480;
	} else {
		usb_packet_size = RNDIS_RX_SIZE_12;
	}

	num_rx_transfers = RX_NUM_RNDIS_PACKETS * (MAX_RNDIS_DATA_BYTES / usb_packet_size);

	memset(rx_transfer, 0, sizeof(rx_transfer));
	memset(&rndis_data_rx_fifo, 0, sizeof rndis_data_rx_fifo);
	rndis_data_rx_fifo = FIFO_INIT(RX_NUM_RNDIS_PACKETS);

	num_tx_transfers = TX_NUM_RNDIS_PACKETS * (MAX_RNDIS_DATA_BYTES / usb_packet_size);
	memset(tx_transfer, 0, sizeof(tx_transfer));
	memset(int_xfer, 0, sizeof(int_xfer));

	tx_transfer_fifo = FIFO_INIT(num_tx_transfers);

	usb_config_tx(RNDIS_INT_ENDPOINT, RNDIS_ACM_SIZE, 1, NULL); // size same 12 & 480
	usb_config_rx(RNDIS_RX_ENDPOINT, usb_packet_size, 0, rx_event);
	usb_config_tx(RNDIS_TX_ENDPOINT, usb_packet_size, 1, NULL);
	for (i = 0; i < num_rx_transfers; i++)
		rx_queue_transfer(i);

}

/*************************************************************************/
/**                               Receive                               **/
/*************************************************************************/

static void rx_queue_transfer(int i) {
	NVIC_DISABLE_IRQ(IRQ_USB1);
	//printf("rx queue i=%d\n", i);
	void *buffer = get_rx_buffer(i);
	usb_prepare_transfer(rx_transfer + i, buffer, usb_packet_size, i);
	arm_dcache_delete(buffer, usb_packet_size);
	usb_receive(RNDIS_RX_ENDPOINT, rx_transfer + i);
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
	int len = usb_packet_size - ((t->status >> 16) & 0x7FFF);
	int i = t->callback_param;
	//printf("rx event, len=%d, i=%d\n", len, i);
	struct rndis_data *d = &rndis_data_rx_pool[fifo_write_index(&rndis_data_rx_fifo)];

	// append USB packet buffer
	d->len += len;
	d->usb_packets[d->num_usb_packets++] = i;

	if (len < usb_packet_size) {
		// rdnis frame complete
		if (fifo_level(&rndis_data_rx_fifo) != rndis_data_rx_fifo.capacity - 1) {
			fifo_add(&rndis_data_rx_fifo, 1);
			//printf("rxq + %d = %d\n", fifo_write_index(&rndis_data_rx_fifo),
//					fifo_level(&rndis_data_rx_fifo));
		} else {
			// fifo full, this entry will be reused
			rndis_data_clear(d);
		}
	}
}

// copy linear range from all buffers
static int rndis_data_copy(const struct rndis_data *d, size_t offset, void *dst, size_t len) {

	if (offset + len > d->len)
		return -1;

	while (len) {
		size_t buffer = offset / usb_packet_size;
		size_t boffset = offset % usb_packet_size;
		size_t n = usb_packet_size - boffset;
		if (len < n)
			n = len;

		const uint8_t *usb_buffer = get_rx_buffer(d->usb_packets[buffer]);
		memcpy(dst, usb_buffer + boffset, n);

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
	struct rndis_data *d = &rndis_data_rx_pool[fifo_read_index(&rndis_data_rx_fifo)];

	//printf("rxdeq@%d\n", fifo_read_index(&rndis_data_rx_fifo));
	if (rndis_data_copy(d, d->read_offset, &h, sizeof(h)) != 0) {
		goto error;
	}

	if (h.h.message_type != RNDIS_MESSAGE_TYPE_PACKET_DATA) {
		printf("wrong packet type\n");
		goto error;
	}

	if (h.h.message_length > d->read_offset + d->len) {
		printf("wrong packet length\n");
		goto error;
	}

	size_t
	data_start = d->read_offset + h.data_offset
	+ offsetof(struct rndis_packet_message, data_offset);
	if (data_start > d->len || data_start + h.data_len > d->len) {
		printf("wrong len\n");
		goto error;
	}

	size_t min_len = h.data_len;
	if (len < min_len)
		min_len = len;

	printf("copy %d %d %d \n", data_start, h.data_len, len);
	if (rndis_data_copy(d, data_start, dst, min_len) != 0) { // copy payload
		printf("copy error\n");
		goto error;
	}
	ret = min_len;

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

int usb_rndis_write(const void *data, size_t len) {

	if (!usb_configuration)
		return 0;

	size_t message_len = len + sizeof(struct rndis_packet_message);
	size_t padding_len = 0;
	if (message_len % usb_packet_size == 0) padding_len = 1; // add one byte to avoid empty usb packets
	message_len += padding_len;
	size_t transfers_required = (message_len + usb_packet_size - 1) / usb_packet_size;

	// try to release the required number of elements
	size_t avail = fifo_remaining(&tx_transfer_fifo);

	if (avail < transfers_required) {
		// insufficient transfer buffers.
		size_t diff = transfers_required - avail;

		for (size_t i = 0; i != diff; i++) {

			if (fifo_level(&tx_transfer_fifo) == 0) {
				return -1;
			}

			size_t read_index = fifo_read_index(&tx_transfer_fifo);
			transfer_t *trans = &tx_transfer[read_index];
			uint32_t status = usb_transfer_status(trans);
			if (!(status & 0x80)) {
				if (status & 0x68) {
					// TODO: what if status has errors???
					printf("ERROR status = %x, i=%d, ms=%u\n", status, i, systick_millis_count);
				}

				fifo_remove(&tx_transfer_fifo, 1);
			} else {
				printf("insufficient buffers\n");
				return -1; // we cannot release transfer buffers. Then we cannot transmit.
			}
		}
	}

	// now we should have enough entries in tx_avail_fifo

	bool add_header = true;
	while (message_len) {
		size_t buf_index = fifo_write_index(&tx_transfer_fifo);
		fifo_add(&tx_transfer_fifo, 1);

		uint8_t *txbuf = txbuffer + (buf_index * usb_packet_size);
		transfer_t *trans = &tx_transfer[buf_index];

		uint8_t *p = txbuf;
		uint8_t *p_end = p + usb_packet_size; // end of usb packet

		if (add_header) {
			struct rndis_packet_message *h = (struct rndis_packet_message *)p;
			h->h.message_type = RNDIS_MESSAGE_TYPE_PACKET_DATA;
			h->h.message_length = sizeof(struct rndis_packet_message) + len;
			h->data_offset = sizeof(struct rndis_packet_message) - sizeof(struct rndis_message_header);
			h->data_len = len;
			h->oobdata_offset = 0;
			h->oobdata_len = 0;
			h->num_oobdata_elements = 0;
			h->per_packet_info_offset = 0;
			h->per_packet_info_len = 0;
			h->vc_handle = 0;
			h->reserved = 0;
			p = h + 1; // data begins behind header
			message_len -= sizeof(struct rndis_packet_message);
			add_header = false;
		}

		size_t chunk_size = p_end - p;
		if (message_len < chunk_size) chunk_size = message_len;

		if (message_len == 1 && padding_len == 1) {
			*p = 0; // single zero padding byte
		} else {
			memcpy(p, data, chunk_size);
		}
		data += chunk_size;
		p += chunk_size;

		message_len -= chunk_size;
		size_t packet_len = (uint8_t*)p - txbuf;

		NVIC_DISABLE_IRQ(IRQ_USB1);
		usb_prepare_transfer(trans, txbuf, packet_len, 0);
		arm_dcache_flush_delete(txbuf, packet_len);
		usb_transmit(RNDIS_TX_ENDPOINT, trans);
		NVIC_ENABLE_IRQ(IRQ_USB1);
	}

	return 0;
}

void rndis_packetFilter(uint32_t newfilter);

/******** RNDIS ********/

#define ENC_BUF_SIZE    (OID_LIST_LENGTH + 8) // in u32

// Command buffer
__attribute__ ((section(".dmabuffers"), aligned(32))) uint32_t encapsulated_buffer[1024];
const size_t encapsulated_buffer_capacity = 1024 * sizeof(uint32_t);
size_t encapsulated_buffer_len;

//Do we have data to send back?
bool data_to_send = false;
bool rndis_initialized = false;

volatile int usb_rndis_irq = 0;

/**
 * \brief Handles a "SEND ENCAPSULATED COMMAND" message.
 *
 * \return True on success, false on failure.
 */
bool rndis_send_encapsulated_command(void) {

	struct rndis_message_header *header = (struct rndis_message_header*) encapsulated_buffer;
	//printf("C%x\n", header->message_type);
	switch (header->message_type) {
	/* Requests remote intilization. Respond with complete,
	 eventually should probably do something */
	case RNDIS_MESSAGE_TYPE_INITIALIZE: {

		struct rndis_initalize_complete_message *m = (struct rndis_initalize_complete_message*) encapsulated_buffer;

		//m->MessageID is same as before
		m->h.message_type = RNDIS_MESSAGE_TYPE_INITIALIZE_COMPLETE;
		m->h.message_length = sizeof(struct rndis_initalize_complete_message);
		m->major_version = 1;
		m->minor_version = 0;
		m->status = RNDIS_STATUS_SUCCESS;
		m->device_flags = 0x01;
		m->medium = 0;
		m->max_packets_per_message = 1;
		m->max_transfer_size = MAX_RNDIS_DATA_BYTES;
		m->packet_alignment_factor = 3;
		m->reserved_AFListOffset = 0;
		m->reserved_AFListSize = 0;
		rndis_initialized = true;

		//printf("rndis_init\n");
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

		m->h.message_type = RNDIS_MESSAGE_TYPE_RESET_COMPLETE;
		m->h.message_length = sizeof(*m);
		m->status = RNDIS_STATUS_SUCCESS;
		m->addressing_reset = 1;

		data_to_send = true;
	}
		break;

	case RNDIS_MESSAGE_TYPE_KEEPALIVE: {
		struct rndis_keepalive_complete_message *m;
		m = (struct rndis_keepalive_complete_message*) encapsulated_buffer;
		m->h.message_type = RNDIS_MESSAGE_TYPE_KEEPALIVE_COMPLETE;
		m->h.message_length = sizeof(*m);
		m->status = RNDIS_STATUS_SUCCESS;
		data_to_send = true;
		break;
	}

	default:
		return false;
	}

	//rndis_send_interrupt();
	usb_rndis_irq = 1;
	return true;
}

/**
 * \brief Send an interrupt over the interrupt endpoint to the host.
 */
void rndis_send_interrupt(void) {

	static DMAMEM __attribute__ ((aligned(32))) uint32_t resp_available[2];

	transfer_t *xfer = &int_xfer[current_int_xfer++];
	if (current_int_xfer == NUM_INT_XFERS)
		current_int_xfer = 0;

	uint32_t status = usb_transfer_status(xfer);
	if (!(status & 0x80)) {
		if (status & 0x68) {
			// TODO: what if status has errors???
			printf("ERROR send intr: status = %x, ms=%u\n", status, systick_millis_count);
		}
	} else {
		// previous transfer pending
		printf("previous transfer pending\n");
		return;
	}

//	printf("INTR\n");
	// dma memory must by initialized dynamically
	resp_available[0] = 0x01;
	resp_available[1] = 0x00;

	usb_prepare_transfer(xfer, resp_available, 8, 0);
	arm_dcache_flush_delete(resp_available, 8);
	usb_transmit(RNDIS_INT_ENDPOINT, xfer);
}

void check_rndis_irq(void) {
	if (usb_rndis_irq) {
		usb_rndis_irq = 0;
		rndis_send_interrupt();
	}
}

uint32_t oid_packet_filter = 0x0000000;

/**
 * \brief Function to handle a RNDIS "QUERY" command in the encapsulated_buffer
 */
static void rndis_query_process(void) {
	struct rndis_query_message *m;
	struct rndis_query_complete_message *c;

	m = (struct rndis_query_message*) encapsulated_buffer;
	c = (struct rndis_query_complete_message*) encapsulated_buffer;

	uint32_t oid = m->oid;
	/* We set up packet for sending one 4-byte response, which a lot of
	 these will do. If you need more or less just change the defaults in
	 the specific case */

	c->h.message_type = RNDIS_MESSAGE_TYPE_QUERY_COMPLETE;
	c->status = RNDIS_STATUS_SUCCESS;
	c->info_buffer_length = 4;
	c->info_buffer_offset = 16; // relative to reqid
	uint32_t *info_buf = (uint32_t*) ((uintptr_t) encapsulated_buffer + sizeof(*c));

	printf("Q%d\n", oid);
	switch (oid) {

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
		c->info_buffer_length = 8;
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
		memcpy(info_buf, MAC_ADDRESS, 6);

		c->info_buffer_length = 6;
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
		c->info_buffer_length = sizeof(struct NDIS_PM_WAKE_UP_CAPABILITIES);
		memset(info_buf, 0, sizeof(struct NDIS_PM_WAKE_UP_CAPABILITIES));
		break;

	case OID_PNP_QUERY_POWER:
		c->info_buffer_length = 0;
		break;

	case OID_PNP_ENABLE_WAKE_UP:
		*info_buf = 0; /* Nothing Supported */
		break;

	default:
		c->status = RNDIS_STATUS_ERROR;
		c->info_buffer_length = 0;
		break;
	}

	//Calculate message size
	c->h.message_length = sizeof(*c) + c->info_buffer_length;

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
static void rndis_set_process(void) {

	struct rndis_set_message *m = (struct rndis_set_message*) encapsulated_buffer;

	struct rndis_set_complete_message *c = (struct rndis_set_complete_message*) encapsulated_buffer;

	const uint32_t
	*info_buf = (uint32_t *)((uintptr_t)m + offsetof(struct rndis_set_message, req_id) + m->info_buffer_offset);

	switch (m->oid) {
	//printf("S%d\n", m->oid);
	/* Parameters set up in 'Advanced' tab */
	case OID_GEN_RNDIS_CONFIG_PARAMETER:
		break;
		/* Mandatory general OIDs */
	case OID_GEN_CURRENT_PACKET_FILTER:
		oid_packet_filter = *info_buf;
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
		c->h.message_type = RNDIS_MESSAGE_TYPE_SET_COMPLETE;
		c->h.message_length = sizeof(struct rndis_set_complete_message);
		c->status = RNDIS_STATUS_ERROR;
		data_to_send = true;
		return;

		break;
	}

	//c->MessageID is same as before
	c->h.message_type = RNDIS_MESSAGE_TYPE_SET_COMPLETE;
	c->h.message_length = sizeof(struct rndis_set_complete_message);
	c->status = RNDIS_STATUS_SUCCESS;
	data_to_send = true;
	return;
}

#endif
