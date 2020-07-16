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

#include "usb_dev.h"
#include "usb_serial.h"
#include "core_pins.h"// for delay()
#include <string.h> // for memcpy()
#include "avr/pgmspace.h" // for PROGMEM, DMAMEM, FASTRUN

#include "debug/printf.h"
#include "util/fifo.h"
#include "core_pins.h"

// defined by usb_dev.h -> usb_desc.h
#if defined(USB_RNDIS)

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
	uint32_t req_id;
	uint32_t major_version;
	uint32_t minor_version;
	uint32_t max_transfer_size;
};

struct rndis_initalize_complete_message {
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
static struct fifo_cnt rndis_data_rx_fifo = { .capacity = RX_NUM_RNDIS_PACKETS, };

static transfer_t rx_transfer[RX_NUM_RNDIS_PACKETS * MAX_USB_PACKETS_PER_RNDIS_DATA] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[RX_NUM_RNDIS_PACKETS * MAX_RNDIS_DATA_BYTES] __attribute__ ((aligned(32)));
static uint16_t rx_packet_size = 0;
static size_t num_rx_transfers;


static void rx_queue_transfer(int i);
static void rx_event(transfer_t *t);

static inline uint8_t* get_rx_buffer(uint16_t index) {
	return rx_buffer + (rx_packet_size * index);
}

void usb_rndis_configure(void) {
	int i;

	printf("usb_serial_configure\n");
	if (usb_high_speed) {
		tx_packet_size = CDC_TX_SIZE_480;
		rx_packet_size = CDC_RX_SIZE_480;
	} else {
		tx_packet_size = CDC_TX_SIZE_12;
		rx_packet_size = CDC_RX_SIZE_12;
	}

	num_rx_transfers = RX_NUM_RNDIS_PACKETS * (MAX_RNDIS_DATA_BYTES / rx_packet_size);
	memset(rx_transfer, 0, sizeof(rx_transfer));
	memset(&rndis_data_rx_fifo, 0, sizeof rndis_data_rx_fifo);
	rndis_data_rx_fifo.capacity = RX_NUM_RNDIS_PACKETS;


	memset(tx_transfer, 0, sizeof(tx_transfer));
	tx_head = 0;
	tx_available = 0;

	usb_config_tx(CDC_ACM_ENDPOINT, CDC_ACM_SIZE, 0, NULL); // size same 12 & 480
	usb_config_rx(CDC_RX_ENDPOINT, rx_packet_size, 0, rx_event);
	usb_config_tx(CDC_TX_ENDPOINT, tx_packet_size, 1, NULL);
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

ssize_t usb_rndis_read(void *dst, size_t len) {

	ssize_t ret = -1;
	struct rndis_packet_message h;

	if (fifo_level(&rndis_data_rx_fifo) == 0) {
		return -1;
	}
	struct rndis_data *d = &rndis_data_rx_pool[fifo_read_index(
			&rndis_data_rx_fifo)];

	struct rndis_packet_message h;

	if (rndis_data_copy(d, d->read_offset, &h, sizeof(h)) != 0) {
		goto error;
	}

	if (h->h.message_type != RNDIS_MESSAGE_TYPE_PACKET_DATA) {
		goto error;
	}

	if (h->h.message_length > d->read_offset + d->len) {
		goto error;
	}

	size_t data_start = d->read_offset + h->data_offset + offsetof(h, data_offset);
	if (data_start > d->len || data_start + h->data_len > d->len) {
		goto error;
	}

	size_t min_len = h->data_len;
	if (len < min_len) min_len = len;

	if (rndis_data_copy(d, data_start , dst, len) != 0) { // copy payload
		goto error;
	}
	ret = len;

	d->read_offset += h->h.message_length; // move to next message

	if (d->read_offset + sizeof(h) < d->len) {
		// there can be more messages
		return ret; // stop before removing the current entry
	}

	error: // we are done with this one
	fifo_remove(&rndis_data_rx_fifo, 1);
	rndis_data_clear(d); // release this message buffer for more incoming data
	return ret;
}


#if 0
//static int maxtimes=0;

// read a block of bytes to a buffer
int usb_serial_read(void *buffer, uint32_t size) {
	uint8_t *p = (uint8_t*) buffer;
	uint32_t count = 0;

	NVIC_DISABLE_IRQ(IRQ_USB1);
	//if (++maxtimes > 15) while (1) ;
	uint32_t tail = rx_tail;
	//printf("usb_serial_read, size=%d, tail=%d, head=%d\n", size, tail, rx_head);
	while (count < size && tail != rx_head) {
		if (++tail > RX_NUM_RNDIS_PACKETS)
			tail = 0;
		uint32_t i = rx_list[tail];
		uint32_t len = size - count;
		uint32_t avail = rx_count[i] - rx_index[i];
		//printf("usb_serial_read, count=%d, size=%d, i=%d, index=%d, len=%d, avail=%d, c=%c\n",
		//count, size, i, rx_index[i], len, avail, rx_buffer[i * CDC_RX_SIZE_480]);
		if (avail > len) {
			// partially consume this packet
			memcpy(p, rx_buffer + i * CDC_RX_SIZE_480 + rx_index[i], len);
			rx_available -= len;
			rx_index[i] += len;
			count += len;
		} else {
			// fully consume this packet
			memcpy(p, rx_buffer + i * CDC_RX_SIZE_480 + rx_index[i], avail);
			p += avail;
			rx_available -= avail;
			count += avail;
			rx_tail = tail;
			rx_queue_transfer(i);
		}
	}
	NVIC_ENABLE_IRQ(IRQ_USB1);
	return count;
}
#endif


/*************************************************************************/
/**                               Transmit                              **/
/*************************************************************************/

#define TX_NUM   4
#define TX_SIZE  2048 /* should be a multiple of CDC_TX_SIZE */
static transfer_t tx_transfer[TX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t txbuffer[TX_SIZE * TX_NUM] __attribute__ ((aligned(32)));
static uint8_t tx_head = 0;
static uint16_t tx_available = 0;
static uint16_t tx_packet_size = 0;

// When the PC isn't listening, how long do we wait before discarding data?  If this is
// too short, we risk losing data during the stalls that are common with ordinary desktop
// software.  If it's too long, we stall the user's program when no software is running.
#define TX_TIMEOUT_MSEC 120

// When we've suffered the transmit timeout, don't wait again until the computer
// begins accepting data.  If no software is running to receive, we'll just discard
// data as rapidly as Serial.print() can generate it, until there's something to
// actually receive it.
static uint8_t transmit_previous_timeout = 0;

// transmit a character.  0 returned on success, -1 on error
int usb_serial_putchar(uint8_t c) {
	return usb_serial_write(&c, 1);
}

extern volatile uint32_t systick_millis_count;

static void timer_config(void (*callback)(void), uint32_t microseconds);
static void timer_start_oneshot();
static void timer_stop();

static void timer_config(void (*callback)(void), uint32_t microseconds) {
	usb_timer0_callback = callback;
	USB1_GPTIMER0CTRL = 0;
	USB1_GPTIMER0LD = microseconds - 1;
	USB1_USBINTR |= USB_USBINTR_TIE0;
}

static void timer_start_oneshot(void) {
	// restarts timer if already running (retriggerable one-shot)
	USB1_GPTIMER0CTRL = USB_GPTIMERCTRL_GPTRUN | USB_GPTIMERCTRL_GPTRST;
}

static void timer_stop(void) {
	USB1_GPTIMER0CTRL = 0;
}

int usb_serial_write(const void *buffer, uint32_t size) {
	uint32_t sent = 0;
	const uint8_t *data = (const uint8_t*) buffer;

	if (!usb_configuration)
		return 0;
	while (size > 0) {
		transfer_t *xfer = tx_transfer + tx_head;
		int waiting = 0;
		uint32_t wait_begin_at = 0;
		while (!tx_available) {
			//digitalWriteFast(3, HIGH);
			uint32_t status = usb_transfer_status(xfer);
			if (!(status & 0x80)) {
				if (status & 0x68) {
					// TODO: what if status has errors???
					printf("ERROR status = %x, i=%d, ms=%u\n",
							status, tx_head, systick_millis_count);
				}
				tx_available = TX_SIZE;
				transmit_previous_timeout = 0;
				break;
			}
			if (!waiting) {
				wait_begin_at = systick_millis_count;
				waiting = 1;
			}
			if (transmit_previous_timeout)
				return sent;
			if (systick_millis_count - wait_begin_at > TX_TIMEOUT_MSEC) {
				// waited too long, assume the USB host isn't listening
				transmit_previous_timeout = 1;
				return sent;
				//printf("\nstop, waited too long\n");
				//printf("status = %x\n", status);
				//printf("tx head=%d\n", tx_head);
				//printf("TXFILLTUNING=%08lX\n", USB1_TXFILLTUNING);
				//usb_print_transfer_log();
				//while (1) ;
			}
			if (!usb_configuration)
				return sent;
			yield();
		}
		//digitalWriteFast(3, LOW);
		uint8_t *txdata = txbuffer + (tx_head * TX_SIZE)
				+ (TX_SIZE - tx_available);
		if (size >= tx_available) {
			memcpy(txdata, data, tx_available);
			//*(txbuffer + (tx_head * TX_SIZE)) = 'A' + tx_head; // to see which buffer
			//*(txbuffer + (tx_head * TX_SIZE) + 1) = ' '; // really see it
			uint8_t *txbuf = txbuffer + (tx_head * TX_SIZE);
			usb_prepare_transfer(xfer, txbuf, TX_SIZE, 0);
			arm_dcache_flush_delete(txbuf, TX_SIZE);
			usb_transmit(CDC_TX_ENDPOINT, xfer);
			if (++tx_head >= TX_NUM)
				tx_head = 0;
			size -= tx_available;
			sent += tx_available;
			data += tx_available;
			tx_available = 0;
			timer_stop();
		} else {
			memcpy(txdata, data, size);
			tx_available -= size;
			sent += size;
			size = 0;
			timer_start_oneshot();
		}
	}
	return sent;
}

int usb_serial_write_buffer_free(void) {
	uint32_t sum = 0;
	tx_noautoflush = 1;
	for (uint32_t i = 0; i < TX_NUM; i++) {
		if (i == tx_head)
			continue;
		if (!(usb_transfer_status(tx_transfer + i) & 0x80))
			sum += TX_SIZE;
	}
	tx_noautoflush = 0;
	return sum;
}

void usb_serial_flush_output(void) {

	if (!usb_configuration)
		return;
	if (tx_available == 0)
		return;
	tx_noautoflush = 1;
	transfer_t *xfer = tx_transfer + tx_head;
	uint8_t *txbuf = txbuffer + (tx_head * TX_SIZE);
	uint32_t txnum = TX_SIZE - tx_available;
	usb_prepare_transfer(xfer, txbuf, txnum, 0);
	arm_dcache_flush_delete(txbuf, txnum);
	usb_transmit(CDC_TX_ENDPOINT, xfer);
	if (++tx_head >= TX_NUM)
		tx_head = 0;
	tx_available = 0;
	tx_noautoflush = 0;
}

static void usb_serial_flush_callback(void) {
	if (tx_noautoflush)
		return;
	if (!usb_configuration)
		return;
	if (tx_available == 0)
		return;
	//printf("flush callback, %d bytes\n", TX_SIZE - tx_available);
	transfer_t *xfer = tx_transfer + tx_head;
	uint8_t *txbuf = txbuffer + (tx_head * TX_SIZE);
	uint32_t txnum = TX_SIZE - tx_available;
	usb_prepare_transfer(xfer, txbuf, txnum, 0);
	arm_dcache_flush_delete(txbuf, txnum);
	usb_transmit(CDC_TX_ENDPOINT, xfer);
	if (++tx_head >= TX_NUM)
		tx_head = 0;
	tx_available = 0;
}

//#endif // F_CPU
#endif // CDC_STATUS_INTERFACE && CDC_DATA_INTERFACE
