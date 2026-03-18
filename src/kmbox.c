// MAKD/MACKU command injection over LPUART6 (pins 0/1)

#include "kmbox.h"
#include "macku.h"
#include "smooth.h"
#include "imxrt.h"
#include "usb_device.h"
#include <string.h>

extern uint32_t millis(void);

#ifndef UART_BAUD
#define UART_BAUD  115200
#endif
#define UART_CLOCK 24000000

// Pin 1 = GPIO_AD_B0_02 ALT2 = LPUART6_TX
// Pin 0 = GPIO_AD_B0_03 ALT2 = LPUART6_RX
#undef  UART_BAUD
#define UART_BAUD          CMD_BAUD

#define KM_UART_BAUD       LPUART6_BAUD
#define KM_UART_CTRL       LPUART6_CTRL
#define KM_UART_STAT       LPUART6_STAT
#define KM_UART_DATA       LPUART6_DATA
#define KM_UART_FIFO       LPUART6_FIFO
#define KM_UART_WATER      LPUART6_WATER

#define KM_RX_SADDR        DMA_TCD3_SADDR
#define KM_RX_SOFF         DMA_TCD3_SOFF
#define KM_RX_ATTR         DMA_TCD3_ATTR
#define KM_RX_NBYTES       DMA_TCD3_NBYTES_MLNO
#define KM_RX_SLAST        DMA_TCD3_SLAST
#define KM_RX_DADDR        DMA_TCD3_DADDR
#define KM_RX_DOFF         DMA_TCD3_DOFF
#define KM_RX_CITER        DMA_TCD3_CITER_ELINKNO
#define KM_RX_BITER        DMA_TCD3_BITER_ELINKNO
#define KM_RX_DLASTSGA     DMA_TCD3_DLASTSGA
#define KM_RX_CSR          DMA_TCD3_CSR
#define KM_RX_DMAMUX       DMAMUX_CHCFG3
#define KM_RX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART6_RX
#define KM_RX_CH           3

#define KM_TX_SADDR        DMA_TCD4_SADDR
#define KM_TX_SOFF         DMA_TCD4_SOFF
#define KM_TX_ATTR         DMA_TCD4_ATTR
#define KM_TX_NBYTES       DMA_TCD4_NBYTES_MLNO
#define KM_TX_SLAST        DMA_TCD4_SLAST
#define KM_TX_DADDR        DMA_TCD4_DADDR
#define KM_TX_DOFF         DMA_TCD4_DOFF
#define KM_TX_CITER        DMA_TCD4_CITER_ELINKNO
#define KM_TX_BITER        DMA_TCD4_BITER_ELINKNO
#define KM_TX_DLASTSGA     DMA_TCD4_DLASTSGA
#define KM_TX_CSR          DMA_TCD4_CSR
#define KM_TX_DMAMUX       DMAMUX_CHCFG4
#define KM_TX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART6_TX
#define KM_TX_CH           4

// When BT_ENABLED, D30/D31 are used by HC-05 module — LEDs unavailable.
// D31 = LINK:  GPIO_EMC_37 = GPIO3[23] — toggles when UART data arriving
// D30 = STATE: GPIO_EMC_36 = GPIO3[22] — toggles on valid frame dispatch
// D24 = STATUS: GPIO_AD_B0_12 = GPIO1[12] — solid = UART OK, flickers on error
#if BT_ENABLED
#define LINK_LED_BIT   0  // D31 used by HC-05 EN
#define STATE_LED_BIT  0  // D30 used by HC-05 STA
#else
#define LINK_LED_BIT   (1u << 23)
#define STATE_LED_BIT  (1u << 22)
#endif
#define STATUS_LED_BIT (1u << 12)

static uint32_t link_last_rx_time;

#define DMA_RX_RING_SIZE 256
static uint8_t dma_rx_ring[DMA_RX_RING_SIZE]
	__attribute__((section(".dmabuffers"), aligned(DMA_RX_RING_SIZE)));
static volatile uint16_t rx_tail;

#define TX_RING_SIZE 256
static uint8_t tx_ring[TX_RING_SIZE];
static uint16_t tx_head;
static uint16_t tx_tail_pos;

#define DMA_TX_BUF_SIZE 64
static uint8_t dma_tx_buf[DMA_TX_BUF_SIZE]
	__attribute__((section(".dmabuffers"), aligned(4)));

typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     mouse_dirty;

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     kb_dirty;

	uint8_t  click_release_mask;
	uint32_t click_release_at; // ms, 0=off

	#define MAX_KB_RELEASES 6
	struct { uint8_t key; uint32_t at; } kb_releases[6];
	uint8_t kb_release_count;
} kmbox_inject_t;

static makd_parser_t parser;
static macku_parser_t macku_parser;

static kmbox_inject_t inject;
static uint32_t frames_ok;
static uint32_t frames_err;
static uint32_t rx_bytes_total;

static uint8_t detected_proto;

static uint32_t current_baud = CMD_BAUD;

static uint32_t uart_overrun_count;
static uint32_t uart_framing_count;
static uint32_t uart_noise_count;

static uint32_t tx_bytes_total;
static uint32_t tx_stuck_count;
static uint32_t pending_baud_rate;
#define TX_STUCK_THRESHOLD 5000       // ~5k polls ≈ 50ms at 100kHz poll rate

static uint8_t  cached_mouse_ep;
static uint16_t cached_mouse_maxpkt;
static uint8_t  cached_kb_ep;
static struct {
	uint16_t x_bit;
	uint16_t y_bit;
	uint16_t wheel_bit;     // 0xFFFF = none
	uint8_t  x_size;
	uint8_t  y_size;
	uint8_t  wheel_size;
	uint8_t  report_id;
	uint8_t  y_report_id;
	uint8_t  wheel_report_id;
	uint8_t  data_off;
	bool     valid;
	int16_t  x_max;
	int16_t  y_max;
	int16_t  w_max;
	bool     fast_path;
	uint8_t  x_byte;
	uint8_t  y_byte;
	uint8_t  w_byte;        // 0xFF = none
	bool     x_is16;
	bool     y_is16;
	bool     w_is16;
} mouse_layout;
static uint8_t cached_mouse_report_len; // actual report length from first real report

static bool merged_this_cycle;

static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel, bool use_smooth);
static void baud_change_apply(uint32_t baud);

static void tx_enqueue(uint8_t b)
{
	uint16_t next = (tx_head + 1) & (TX_RING_SIZE - 1);
	if (next == tx_tail_pos) return;
	tx_ring[tx_head] = b;
	tx_head = next;
}

static void uart_tx_frame(const uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
		tx_enqueue(data[i]);
}

static void tx_flush(void)
{
	if (!(KM_TX_CSR & DMA_TCD_CSR_DONE) &&
	    KM_TX_CITER != KM_TX_BITER) {
		// DMA still in progress — check for stuck transfer
		if (++tx_stuck_count >= TX_STUCK_THRESHOLD) {
			// Force-recover: disable channel, clear error, mark DONE
			DMA_CERQ = KM_TX_CH;
			if (DMA_ERR & (1u << KM_TX_CH))
				DMA_CERR = KM_TX_CH;
			KM_TX_CSR = DMA_TCD_CSR_DONE;
			// Flush UART TX FIFO to unblock TDRE
			KM_UART_FIFO |= LPUART_FIFO_TXFLUSH;
			tx_stuck_count = 0;
			// fall through to re-arm
		} else {
			return;
		}
	}
	tx_stuck_count = 0;
	DMA_CDNE = KM_TX_CH;
	uint8_t count = 0;
	while (tx_tail_pos != tx_head && count < DMA_TX_BUF_SIZE) {
		dma_tx_buf[count++] = tx_ring[tx_tail_pos];
		tx_tail_pos = (tx_tail_pos + 1) & (TX_RING_SIZE - 1);
	}
	if (count == 0) return;
	tx_bytes_total += count;
	KM_TX_SADDR = (volatile const void *)dma_tx_buf;
	KM_TX_CITER = count;
	KM_TX_BITER = count;
	KM_TX_CSR = DMA_TCD_CSR_DREQ;
	DMA_SERQ = KM_TX_CH;
}
void kmbox_init(void)
{
#if NET_ENABLED
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;
	tx_bytes_total = 0;
	tx_stuck_count = 0;
	uart_overrun_count = 0;
	uart_framing_count = 0;
	uart_noise_count = 0;
	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;
	return;
#endif
	CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART6_TX_SELECT_INPUT = 1;
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) |
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

#if !BT_ENABLED
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_37 = 5; // D31 LINK — ALT5 = GPIO3[23]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_37 = IOMUXC_PAD_DSE(6);
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_36 = 5; // D30 STATE — ALT5 = GPIO3[22]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_36 = IOMUXC_PAD_DSE(6);
	GPIO3_GDIR |= LINK_LED_BIT | STATE_LED_BIT;
	GPIO3_DR_CLEAR = LINK_LED_BIT | STATE_LED_BIT;
#endif

	// D24 STATUS LED — solid when UART OK, flickers on error
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 = 5; // D24 — ALT5 = GPIO1[12]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_12 = IOMUXC_PAD_DSE(6);
	GPIO1_GDIR |= STATUS_LED_BIT;
	GPIO1_DR_SET = STATUS_LED_BIT; // ON = UART configured OK

	// OSR so SBR >= 1
	uint32_t osr;
	if (UART_BAUD <= 460800) {
		osr = 15;
	} else {
		osr = UART_CLOCK / UART_BAUD - 1;
		if (osr < 4) osr = 4;
		if (osr > 31) osr = 31;
	}
	uint32_t sbr = UART_CLOCK / (UART_BAUD * (osr + 1));
	if (sbr == 0) sbr = 1;
	KM_UART_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
	KM_UART_CTRL = 0;
	KM_UART_FIFO = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE;
	KM_UART_FIFO |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_RXFLUSH;
	KM_UART_WATER = LPUART_WATER_RXWATER(1);

	KM_UART_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
	KM_RX_DMAMUX = 0; // disable before reconfiguring
	KM_RX_SADDR = (volatile const void *)&KM_UART_DATA;
	KM_RX_SOFF = 0;
	KM_RX_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	             DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	KM_RX_NBYTES = 1;
	KM_RX_SLAST = 0;
	KM_RX_DADDR = (volatile void *)dma_rx_ring;
	KM_RX_DOFF = 1;
	KM_RX_CITER = DMA_RX_RING_SIZE;
	KM_RX_BITER = DMA_RX_RING_SIZE;
	KM_RX_DLASTSGA = -DMA_RX_RING_SIZE;
	KM_RX_CSR = 0;
	KM_RX_DMAMUX = KM_RX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	DMA_SERQ = KM_RX_CH;
	KM_UART_BAUD |= LPUART_BAUD_RDMAE;
	rx_tail = 0;
	KM_TX_DMAMUX = 0;
	KM_TX_SADDR = (volatile const void *)dma_tx_buf;
	KM_TX_SOFF = 1;
	KM_TX_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	             DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	KM_TX_NBYTES = 1;
	KM_TX_SLAST = 0;
	KM_TX_DADDR = (volatile void *)&KM_UART_DATA;
	KM_TX_DOFF = 0;
	KM_TX_DLASTSGA = 0;
	KM_TX_CSR = DMA_TCD_CSR_DONE;
	KM_TX_DMAMUX = KM_TX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	KM_UART_BAUD |= LPUART_BAUD_TDMAE;

	tx_head = 0;
	tx_tail_pos = 0;
	makd_parser_reset(&parser);
	macku_parser_reset(&macku_parser);
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;
	tx_bytes_total = 0;
	tx_stuck_count = 0;
	uart_overrun_count = 0;
	uart_framing_count = 0;
	uart_noise_count = 0;

	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;

	link_last_rx_time = 0;

	makd_init();
	smooth_init(1000); // default 1kHz, main.c re-inits with actual rate
}

static void parse_mouse_layout(const uint8_t *rd, uint16_t rdlen)
{
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;

	uint16_t usage_page = 0;
	uint8_t  usages[16];
	uint8_t  num_usages = 0;
	uint16_t usage_min = 0, usage_max = 0;
	uint8_t  report_size = 0;
	uint8_t  report_count = 0;
	uint8_t  current_rid = 0;
	uint16_t bit_pos = 0;

	uint16_t i = 0;
	while (i < rdlen) {
		uint8_t b = rd[i];
		if (b == 0xFE) { // long item — skip
			if (i + 2 < rdlen) i += 3 + rd[i + 1];
			else break;
			continue;
		}

		uint8_t sz = b & 0x03;
		if (sz == 3) sz = 4;
		if (i + 1 + sz > rdlen) break;

		// Read unsigned data
		uint32_t val = 0;
		if (sz >= 1) val = rd[i + 1];
		if (sz >= 2) val |= (uint32_t)rd[i + 2] << 8;
		if (sz >= 4) val |= (uint32_t)rd[i + 3] << 16 | (uint32_t)rd[i + 4] << 24;

		switch (b & 0xFC) {
		case 0x04: usage_page = (uint16_t)val; break;   // Usage Page
		case 0x74: report_size = (uint8_t)val; break;    // Report Size
		case 0x94: report_count = (uint8_t)val; break;   // Report Count
		case 0x84:                                        // Report ID
			current_rid = (uint8_t)val;
			bit_pos = 0;
			break;

		case 0x08: // Usage
			if (num_usages < 16) usages[num_usages++] = (uint8_t)val;
			break;
		case 0x18: usage_min = (uint16_t)val; break;     // Usage Minimum
		case 0x28: usage_max = (uint16_t)val; break;     // Usage Maximum

		case 0x80: { // Input
			if (num_usages == 0 && usage_max >= usage_min) {
				for (uint16_t u = usage_min; u <= usage_max && num_usages < 16; u++)
					usages[num_usages++] = (uint8_t)u;
			}

			for (uint8_t f = 0; f < report_count; f++) {
				uint8_t u = (f < num_usages) ? usages[f] :
				            (num_usages > 0 ? usages[num_usages - 1] : 0);

				if (usage_page == 0x01) { // Generic Desktop
					if (u == 0x30) { // X
						mouse_layout.x_bit = bit_pos;
						mouse_layout.x_size = report_size;
						mouse_layout.report_id = current_rid;
					} else if (u == 0x31) { // Y
						mouse_layout.y_bit = bit_pos;
						mouse_layout.y_size = report_size;
						mouse_layout.y_report_id = current_rid;
					} else if (u == 0x38) { // Wheel
						mouse_layout.wheel_bit = bit_pos;
						mouse_layout.wheel_size = report_size;
						mouse_layout.wheel_report_id = current_rid;
					}
				}
				bit_pos += report_size;
			}
			// Clear local state after Main item
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		}
		case 0xA0: // Collection
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		case 0xC0: // End Collection
			num_usages = 0;
			break;
		}

		i += 1 + sz;
	}

	mouse_layout.data_off = mouse_layout.report_id ? 1 : 0;
	mouse_layout.valid = (mouse_layout.x_size > 0 && mouse_layout.y_size > 0);
	mouse_layout.x_max = mouse_layout.x_size > 0 ? (int16_t)((1 << (mouse_layout.x_size - 1)) - 1) : 0;
	mouse_layout.y_max = mouse_layout.y_size > 0 ? (int16_t)((1 << (mouse_layout.y_size - 1)) - 1) : 0;
	mouse_layout.w_max = mouse_layout.wheel_size > 0 ? (int16_t)((1 << (mouse_layout.wheel_size - 1)) - 1) : 0;

	mouse_layout.fast_path = false;
	mouse_layout.w_byte = 0xFF;

	if (mouse_layout.valid &&
	    (mouse_layout.x_bit & 7) == 0 &&
	    (mouse_layout.y_bit & 7) == 0 &&
	    (mouse_layout.x_size == 8 || mouse_layout.x_size == 16) &&
	    (mouse_layout.y_size == 8 || mouse_layout.y_size == 16) &&
	    mouse_layout.report_id == mouse_layout.y_report_id) {

		mouse_layout.x_byte = (uint8_t)(mouse_layout.x_bit / 8) + mouse_layout.data_off;
		mouse_layout.y_byte = (uint8_t)(mouse_layout.y_bit / 8) + mouse_layout.data_off;
		mouse_layout.x_is16 = (mouse_layout.x_size == 16);
		mouse_layout.y_is16 = (mouse_layout.y_size == 16);

		if (mouse_layout.wheel_bit != 0xFFFF &&
		    (mouse_layout.wheel_bit & 7) == 0 &&
		    (mouse_layout.wheel_size == 8 || mouse_layout.wheel_size == 16) &&
		    mouse_layout.wheel_report_id == mouse_layout.report_id) {
			mouse_layout.w_byte = (uint8_t)(mouse_layout.wheel_bit / 8) + mouse_layout.data_off;
			mouse_layout.w_is16 = (mouse_layout.wheel_size == 16);
		}
		mouse_layout.fast_path = true;
	}
}

static int32_t read_report_field(const uint8_t *buf, uint8_t buf_len,
                                 uint16_t bit_off,
                                 uint8_t bit_size, uint8_t data_off)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			if (byte_idx + 2 > buf_len) return 0;
			return (int16_t)(buf[byte_idx] | ((uint16_t)buf[byte_idx + 1] << 8));
		}
		if (bit_size == 8) {
			if (byte_idx + 1 > buf_len) return 0;
			return (int8_t)buf[byte_idx];
		}
	}

	uint32_t raw = 0;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	if (byte_idx + bytes_needed > buf_len) return 0;
	for (uint8_t b = 0; b < bytes_needed; b++)
		raw |= (uint32_t)buf[byte_idx + b] << (b * 8);
	raw = (raw >> bit_idx) & ((1u << bit_size) - 1);
	if (raw & (1u << (bit_size - 1)))
		raw |= ~((1u << bit_size) - 1); // sign extend
	return (int32_t)raw;
}

static void write_report_field(uint8_t *buf, uint16_t buf_len, uint16_t bit_off,
                               uint8_t bit_size, uint8_t data_off, int32_t value)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			if (byte_idx + 2 > buf_len) return;
			buf[byte_idx]     = (uint8_t)(value & 0xFF);
			buf[byte_idx + 1] = (uint8_t)((value >> 8) & 0xFF);
			return;
		}
		if (bit_size == 8) {
			if (byte_idx + 1 > buf_len) return;
			buf[byte_idx] = (uint8_t)(int8_t)value;
			return;
		}
	}

	uint32_t mask = ((1u << bit_size) - 1) << bit_idx;
	uint32_t val  = ((uint32_t)value & ((1u << bit_size) - 1)) << bit_idx;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	if (byte_idx + bytes_needed > buf_len) return;
	for (uint8_t b = 0; b < bytes_needed; b++) {
		uint8_t m = (mask >> (b * 8)) & 0xFF;
		uint8_t v = (val  >> (b * 8)) & 0xFF;
		buf[byte_idx + b] = (buf[byte_idx + b] & ~m) | v;
	}
}

void kmbox_cache_endpoints(const captured_descriptors_t *desc)
{
	cached_mouse_ep = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (desc->ifaces[i].interrupt_ep == 0) continue;
		uint8_t ep = desc->ifaces[i].interrupt_ep & 0x0F;
		if (desc->ifaces[i].iface_protocol == 2 && !cached_mouse_ep) {
			cached_mouse_ep = ep;
			cached_mouse_maxpkt = desc->ifaces[i].interrupt_maxpkt;
			parse_mouse_layout(desc->ifaces[i].hid_report_desc,
			                   desc->ifaces[i].hid_report_desc_len);
		} else if (desc->ifaces[i].iface_protocol == 1 && !cached_kb_ep) {
			cached_kb_ep = ep;
		}
	}
}

void kmbox_poll(void)
{
#if NET_ENABLED
	merged_this_cycle = false;
	return; // NET mode: commands come from kmnet_poll()
#endif
	merged_this_cycle = false;
	tx_flush();
	// Deferred baud change: apply only after TX DMA has finished sending response
	if (pending_baud_rate && (KM_TX_CSR & DMA_TCD_CSR_DONE)) {
		baud_change_apply(pending_baud_rate);
		pending_baud_rate = 0;
	}
	uint32_t stat = KM_UART_STAT;
	if (__builtin_expect(stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF), 0)) {
		if (stat & LPUART_STAT_OR) uart_overrun_count++;
		if (stat & LPUART_STAT_FE) uart_framing_count++;
		if (stat & LPUART_STAT_NF) uart_noise_count++;
		KM_UART_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		if (stat & (LPUART_STAT_OR | LPUART_STAT_FE)) {
			makd_parser_reset(&parser);
			macku_parser_reset(&macku_parser);
		}
		GPIO1_DR_TOGGLE = STATUS_LED_BIT;
	}

	if (__builtin_expect(inject.click_release_at, 0) && millis() >= inject.click_release_at) {
		inject.mouse_buttons &= ~inject.click_release_mask;
		inject.mouse_dirty = true;
		inject.click_release_mask = 0;
		inject.click_release_at = 0;
	}

	if (__builtin_expect(inject.kb_release_count, 0)) {
		uint32_t now = millis();
		for (int r = 0; r < inject.kb_release_count; ) {
			if (now >= inject.kb_releases[r].at) {
				uint8_t key = inject.kb_releases[r].key;
				for (int i = 0; i < 6; i++) {
					if (inject.kb_keys[i] == key) {
						inject.kb_keys[i] = 0;
						break;
					}
				}
				inject.kb_dirty = true;
				inject.kb_releases[r] = inject.kb_releases[--inject.kb_release_count];
			} else {
				r++;
			}
		}
	}

	uint16_t head = ((uint32_t)KM_RX_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	if (head != rx_tail) {
		GPIO3_DR_TOGGLE = LINK_LED_BIT;
		link_last_rx_time = millis();
	} else if (link_last_rx_time && (millis() - link_last_rx_time) > 50) {
		GPIO3_DR_CLEAR = LINK_LED_BIT;
		link_last_rx_time = 0;
	}
	if (head == rx_tail && (parser.state != 0 || macku_parser.state != 0 || macku_parser.bin_state != 0)) {
		if (!link_last_rx_time || (millis() - link_last_rx_time) > 5) {
			makd_parser_reset(&parser);
			macku_parser_reset(&macku_parser);
		}
	}
	while (rx_tail != head) {
		uint8_t b = dma_rx_ring[rx_tail];
		rx_tail = (rx_tail + 1) & (DMA_RX_RING_SIZE - 1);
		rx_bytes_total++;

		if (detected_proto == 0 || detected_proto == 1) {
			if (makd_parser_feed(&parser, b)) {
				makd_dispatch(parser.cmd, parser.buf, parser.len, uart_tx_frame);
				tx_flush();
				frames_ok++;
				detected_proto = 1;
				GPIO3_DR_TOGGLE = STATE_LED_BIT;
				continue;
			}
		}
		if (detected_proto == 0 || detected_proto >= 2) {
			uint8_t r = macku_parser_feed(&macku_parser, b);
			if (r == 1) {
				macku_dispatch(macku_parser.cmd, macku_parser.cmd_len,
				               macku_parser.arg, macku_parser.arg_len,
				               uart_tx_frame);
				tx_flush();
				frames_ok++;
				detected_proto = g_identity_mode ? 3 : 2; // 2=MACKU, 3=Ferrum
				GPIO3_DR_TOGGLE = STATE_LED_BIT;
			} else if (r == 2) {
				macku_dispatch_bin(macku_parser.bin_buf, macku_parser.bin_len,
				                   uart_tx_frame);
				tx_flush();
				frames_ok++;
				detected_proto = g_identity_mode ? 3 : 2;
			}
		}
	}
}

__attribute__((section(".fastrun")))
void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len)
{
	if (iface_protocol == 2) {
		if (__builtin_expect(cached_mouse_report_len == 0, 0))
			cached_mouse_report_len = len;

		if (mouse_layout.valid && inject.mouse_dirty) {
			uint8_t doff = mouse_layout.data_off;
			uint8_t rid = doff ? report[0] : 0;

			if (mouse_layout.fast_path && rid == mouse_layout.report_id) {
				report[doff] |= inject.mouse_buttons;

				if (mouse_layout.x_is16) {
					int32_t rx = (int16_t)(report[mouse_layout.x_byte] |
					             ((uint16_t)report[mouse_layout.x_byte + 1] << 8));
					int32_t mx = rx + inject.mouse_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte]     = (uint8_t)(mx & 0xFF);
					report[mouse_layout.x_byte + 1] = (uint8_t)(mx >> 8);
				} else {
					int32_t rx = (int8_t)report[mouse_layout.x_byte];
					int32_t mx = rx + inject.mouse_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte] = (uint8_t)(int8_t)mx;
				}

				if (mouse_layout.y_is16) {
					int32_t ry = (int16_t)(report[mouse_layout.y_byte] |
					             ((uint16_t)report[mouse_layout.y_byte + 1] << 8));
					int32_t my = ry + inject.mouse_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte]     = (uint8_t)(my & 0xFF);
					report[mouse_layout.y_byte + 1] = (uint8_t)(my >> 8);
				} else {
					int32_t ry = (int8_t)report[mouse_layout.y_byte];
					int32_t my = ry + inject.mouse_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte] = (uint8_t)(int8_t)my;
				}

				if (mouse_layout.w_byte != 0xFF && inject.mouse_wheel != 0) {
					if (mouse_layout.w_is16) {
						int32_t rw = (int16_t)(report[mouse_layout.w_byte] |
						             ((uint16_t)report[mouse_layout.w_byte + 1] << 8));
						int32_t mw = rw + inject.mouse_wheel;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte]     = (uint8_t)(mw & 0xFF);
						report[mouse_layout.w_byte + 1] = (uint8_t)(mw >> 8);
					} else {
						int32_t rw = (int8_t)report[mouse_layout.w_byte];
						int32_t mw = rw + inject.mouse_wheel;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte] = (uint8_t)(int8_t)mw;
					}
					inject.mouse_wheel = 0;
				}

				inject.mouse_dx    = 0;
				inject.mouse_dy    = 0;
				inject.mouse_dirty = (inject.mouse_buttons != 0 ||
				                      inject.mouse_wheel != 0);
			} else {
				bool wheel_consumed = false;

				if (rid == mouse_layout.report_id) {
					report[doff] |= inject.mouse_buttons;

					int32_t rx = read_report_field(report, len, mouse_layout.x_bit,
					                               mouse_layout.x_size, doff);
					int32_t mx = rx + inject.mouse_dx;
					if (mx > mouse_layout.x_max) mx = mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					write_report_field(report, len, mouse_layout.x_bit,
					                   mouse_layout.x_size, doff, mx);

					if (rid == mouse_layout.y_report_id) {
						int32_t ry = read_report_field(report, len, mouse_layout.y_bit,
						                               mouse_layout.y_size, doff);
						int32_t my = ry + inject.mouse_dy;
						if (my > mouse_layout.y_max) my = mouse_layout.y_max;
						if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
						write_report_field(report, len, mouse_layout.y_bit,
						                   mouse_layout.y_size, doff, my);
					}
				}

				if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
				    rid == mouse_layout.wheel_report_id) {
					int32_t rw = read_report_field(report, len, mouse_layout.wheel_bit,
					                               mouse_layout.wheel_size, doff);
					int32_t mw = rw + inject.mouse_wheel;
					if (mw > mouse_layout.w_max) mw = mouse_layout.w_max;
					if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
					write_report_field(report, len, mouse_layout.wheel_bit,
					                   mouse_layout.wheel_size, doff, mw);
					wheel_consumed = true;
				}

				inject.mouse_dx = 0;
				inject.mouse_dy = 0;
				if (wheel_consumed)
					inject.mouse_wheel = 0;
				inject.mouse_dirty = (inject.mouse_buttons != 0 ||
				                      inject.mouse_wheel != 0);
			}
		}
		merged_this_cycle = true;
	} else if (iface_protocol == 1 && inject.kb_dirty) {
		if (len >= 8) {
			report[0] |= inject.kb_modifier;
			for (int i = 0; i < 6; i++) {
				if (inject.kb_keys[i] == 0) continue;
				bool found = false;
				for (int j = 2; j < 8; j++) {
					if (report[j] == inject.kb_keys[i]) {
						found = true;
						break;
					}
				}
				if (!found) {
					for (int j = 2; j < 8; j++) {
						if (report[j] == 0) {
							report[j] = inject.kb_keys[i];
							break;
						}
					}
				}
			}
		}
		merged_this_cycle = true;
	}
}

__attribute__((section(".fastrun")))
void kmbox_send_pending(void)
{
	// Flush unconsumed wheel on a separate report ID even when merged
	if (merged_this_cycle && inject.mouse_wheel != 0 &&
	    cached_mouse_ep && mouse_layout.valid &&
	    mouse_layout.wheel_bit != 0xFFFF &&
	    mouse_layout.wheel_report_id != mouse_layout.report_id) {
		uint8_t synth[16];
		memset(synth, 0, sizeof(synth));
		uint8_t doff = mouse_layout.data_off;
		if (doff) synth[0] = mouse_layout.wheel_report_id;
		int32_t w = inject.mouse_wheel;
		if (w > mouse_layout.w_max) w = mouse_layout.w_max;
		if (w < -mouse_layout.w_max) w = -mouse_layout.w_max;
		write_report_field(synth, sizeof(synth), mouse_layout.wheel_bit,
		                   mouse_layout.wheel_size, doff, w);
		uint8_t rlen = cached_mouse_report_len;
		if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
		usb_device_send_report(cached_mouse_ep, synth, rlen);
		inject.mouse_wheel = 0;
		inject.mouse_dirty = (inject.mouse_buttons != 0);
	}

	if (merged_this_cycle) return;
	if (inject.mouse_dirty && cached_mouse_ep && mouse_layout.valid) {
		uint8_t synth[16];
		memset(synth, 0, sizeof(synth));
		uint8_t doff = mouse_layout.data_off;
		if (doff) synth[0] = mouse_layout.report_id;
		synth[doff] = inject.mouse_buttons;
		int32_t dx = inject.mouse_dx;
		int32_t dy = inject.mouse_dy;
		if (dx > mouse_layout.x_max) dx = mouse_layout.x_max;
		if (dx < -mouse_layout.x_max) dx = -mouse_layout.x_max;
		if (dy > mouse_layout.y_max) dy = mouse_layout.y_max;
		if (dy < -mouse_layout.y_max) dy = -mouse_layout.y_max;

		write_report_field(synth, sizeof(synth), mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, dx);
		write_report_field(synth, sizeof(synth), mouse_layout.y_bit,
		                   mouse_layout.y_size, doff, dy);

		if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
		    mouse_layout.wheel_report_id == mouse_layout.report_id) {
			int32_t w = inject.mouse_wheel;
			if (w > mouse_layout.w_max) w = mouse_layout.w_max;
			if (w < -mouse_layout.w_max) w = -mouse_layout.w_max;
			write_report_field(synth, sizeof(synth), mouse_layout.wheel_bit,
			                   mouse_layout.wheel_size, doff, w);
		}
		uint8_t rlen = cached_mouse_report_len;
		if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
		usb_device_send_report(cached_mouse_ep, synth, rlen);
		inject.mouse_dx = 0;
		inject.mouse_dy = 0;
		inject.mouse_wheel = 0;
		inject.mouse_dirty = (inject.mouse_buttons != 0);
	}
	if (inject.kb_dirty && cached_kb_ep) {
		uint8_t synth[8];
		synth[0] = inject.kb_modifier;
		synth[1] = 0;
		memcpy(&synth[2], inject.kb_keys, 6);
		usb_device_send_report(cached_kb_ep, synth, 8);
		static const uint8_t zeros[6] = {0};
		inject.kb_dirty = (inject.kb_modifier != 0 ||
		                    memcmp(inject.kb_keys, zeros, 6) != 0);
	}
}

void kmbox_inject_smooth(int16_t dx, int16_t dy)
{
	inject.mouse_dx += dx;
	inject.mouse_dy += dy;
	inject.mouse_dirty = true;
}

static void baud_change_apply(uint32_t baud)
{
	KM_UART_CTRL &= ~(LPUART_CTRL_TE | LPUART_CTRL_RE);

	uint32_t osr;
	if (baud <= 460800) {
		osr = 15;
	} else {
		osr = UART_CLOCK / baud - 1;
		if (osr < 4) osr = 4;
		if (osr > 31) osr = 31;
	}
	uint32_t sbr = UART_CLOCK / (baud * (osr + 1));
	if (sbr == 0) sbr = 1;

	KM_UART_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr)
	             | LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE;

	KM_UART_CTRL |= LPUART_CTRL_TE | LPUART_CTRL_RE;

	current_baud = baud;

	g_identity_mode = (baud >= 2000000) ? 1 : 0;
	g_echo = (baud > 1000000) ? 1 : 0;

	detected_proto = 0;
}

void kmbox_set_baud(uint32_t baud)
{
	if (baud < 9600 || baud > 6000000) return;
	pending_baud_rate = baud;
}

uint32_t kmbox_current_baud(void) { return current_baud; }

uint32_t kmbox_frame_count(void) { return frames_ok; }
uint32_t kmbox_error_count(void) { return frames_err; }
uint32_t kmbox_rx_byte_count(void) { return rx_bytes_total; }
uint32_t kmbox_tx_byte_count(void) { return tx_bytes_total; }
uint32_t kmbox_uart_overrun(void) { return uart_overrun_count; }
uint32_t kmbox_uart_framing(void) { return uart_framing_count; }
uint32_t kmbox_uart_noise(void) { return uart_noise_count; }
uint8_t  kmbox_protocol_mode(void) { return detected_proto; }
__attribute__((section(".fastrun")))
static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel, bool use_smooth)
{
	inject.mouse_buttons = buttons;
	inject.mouse_wheel += wheel;

	if (use_smooth && (dx != 0 || dy != 0)) {
		smooth_inject(dx, dy);
		if (buttons != 0 || wheel != 0)
			inject.mouse_dirty = true;
	} else {
		inject.mouse_dx += dx;
		inject.mouse_dy += dy;
		inject.mouse_dirty = true;
	}
}

void kmbox_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                        int8_t wheel, bool use_smooth)
{
	apply_mouse_result(dx, dy, buttons, wheel, use_smooth);
}

void kmbox_inject_keyboard(uint8_t modifier, const uint8_t keys[6])
{
	inject.kb_modifier = modifier;
	memcpy(inject.kb_keys, keys, 6);
	inject.kb_dirty = true;
}

void kmbox_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
	inject.click_release_mask = button_mask;
	inject.click_release_at = millis() + delay_ms;
}

void kmbox_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
	uint32_t at = millis() + delay_ms;
	// Replace existing entry for same key
	for (int i = 0; i < inject.kb_release_count; i++) {
		if (inject.kb_releases[i].key == key) {
			inject.kb_releases[i].at = at;
			return;
		}
	}
	// Add new entry
	if (inject.kb_release_count < MAX_KB_RELEASES) {
		inject.kb_releases[inject.kb_release_count].key = key;
		inject.kb_releases[inject.kb_release_count].at = at;
		inject.kb_release_count++;
	}
}
