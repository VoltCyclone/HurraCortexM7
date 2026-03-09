// Multi-protocol command injection over LPUART6
// Auto-detects and handles three protocols on the same UART:
//   - KMBox B binary: sync 0x57 0xAB, cmd, len, payload, checksum
//   - Makcu binary:   sync 0x50, cmd, len_lo, len_hi, payload
//   - Ferrum text:    km.command(args)\n  (any printable ASCII start)
//
// Integrates smooth injection queue with FPU-based humanization tremor.
// Merges injected keyboard/mouse inputs additively with real device HID reports.

#include "kmbox.h"
#include "smooth.h"
#include "ferrum.h"
#include "makcu.h"
#include "imxrt.h"
#include "usb_device.h"
#include <string.h>

extern uint32_t millis(void);

// ---- UART constants ----
#ifndef UART_BAUD
#define UART_BAUD  2000000
#endif
#define UART_CLOCK 24000000

// ---- eDMA RX ring buffer (non-cacheable via MPU region 10) ----
#define DMA_RX_RING_SIZE 256
static uint8_t dma_rx_ring[DMA_RX_RING_SIZE]
	__attribute__((section(".dmabuffers"), aligned(DMA_RX_RING_SIZE)));
static volatile uint16_t rx_tail;

// ---- eDMA TX: staging ring -> DMA channel 2 -> LPUART6_DATA ----
// Software ring for staging; DMA copies contiguous chunks to UART hardware.
#define TX_RING_SIZE 128
static uint8_t tx_ring[TX_RING_SIZE];
static uint8_t tx_head;
static uint8_t tx_tail_pos;

// DMA TX linear buffer — non-cacheable for DMA coherency
#define DMA_TX_BUF_SIZE 64
static uint8_t dma_tx_buf[DMA_TX_BUF_SIZE]
	__attribute__((section(".dmabuffers"), aligned(4)));

// ---- Multi-protocol dispatcher state ----
typedef enum {
	PROTO_IDLE,       // waiting for first byte to determine protocol
	PROTO_KMBOX,      // in KMBox B binary frame
	PROTO_MAKCU,      // in Makcu binary frame
	PROTO_FERRUM      // accumulating text line
} proto_mode_t;

// ---- KMBox B parser states ----
typedef enum {
	KB_SYNC2,
	KB_CMD,
	KB_LEN,
	KB_PAYLOAD,
	KB_CHECKSUM
} kb_state_t;

// ---- Makcu parser states ----
typedef enum {
	MK_CMD,
	MK_LEN_LO,
	MK_LEN_HI,
	MK_PAYLOAD
} mk_state_t;

// ---- Injection state ----
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;
	int8_t   mouse_wheel;
	bool     mouse_dirty;

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     kb_dirty;

	// Click auto-release: mask of buttons to release after click_release_at
	uint8_t  click_release_mask;
	uint32_t click_release_at; // millis() timestamp, 0 = inactive
} kmbox_inject_t;

// ---- Static state ----

// Protocol dispatcher
static proto_mode_t proto_mode;

// KMBox B parser
static kb_state_t   kb_parse_state;
static uint8_t      frame_buf[KMBOX_MAX_PAYLOAD];
static uint8_t      frame_cmd;
static uint8_t      frame_len;
static uint8_t      frame_pos;
static uint8_t      frame_checksum;

// Makcu parser
static mk_state_t   mk_parse_state;
static uint8_t      mk_cmd;
static uint16_t     mk_len;
static uint16_t     mk_pos;
static uint8_t      mk_buf[MAKCU_MAX_PAYLOAD];

// Ferrum line buffer
static char         ferrum_line[FERRUM_MAX_LINE];
static uint8_t      ferrum_pos;

// Shared injection state
static kmbox_inject_t inject;
static uint32_t frames_ok;
static uint32_t frames_err;
static uint32_t rx_bytes_total;

// Cached EP info — set once at enumeration, avoids per-poll iface scanning
static uint8_t  cached_mouse_ep;    // EP number for mouse (0 = not found)
static uint16_t cached_mouse_maxpkt;
static uint8_t  cached_kb_ep;       // EP number for keyboard (0 = not found)
// Mouse report field layout — parsed from HID report descriptor
static struct {
	uint16_t x_bit;         // bit offset of X in data (after report ID byte)
	uint16_t y_bit;         // bit offset of Y
	uint16_t wheel_bit;     // bit offset of wheel (0xFFFF = none)
	uint8_t  x_size;        // X field size in bits (8 or 16 typically)
	uint8_t  y_size;        // Y field size in bits
	uint8_t  wheel_size;    // wheel field size in bits
	uint8_t  report_id;     // 0 = no report ID byte
	uint8_t  data_off;      // byte offset of data start (0 or 1)
	bool     valid;         // true if X and Y were found
	int16_t  x_max;         // precomputed (1 << (x_size-1)) - 1
	int16_t  y_max;         // precomputed (1 << (y_size-1)) - 1
	int16_t  w_max;         // precomputed (1 << (wheel_size-1)) - 1
} mouse_layout;
static uint8_t cached_mouse_report_len; // actual report length from first real report

// Track whether a merge happened this cycle (reset each poll)
static bool merged_this_cycle;

// ---- Forward declarations ----
static void dispatch_kmbox_frame(void);
static void dispatch_makcu_frame(void);
static void dispatch_ferrum_line(void);
static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel, bool use_smooth);

// ---- Non-blocking TX: enqueue to ring, DMA flushes to UART ----
static void tx_enqueue(uint8_t b)
{
	uint8_t next = (tx_head + 1) & (TX_RING_SIZE - 1);
	if (next == tx_tail_pos) return; // full — drop byte
	tx_ring[tx_head] = b;
	tx_head = next;
}

static void tx_enqueue_buf(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
		tx_enqueue(data[i]);
}

static void tx_enqueue_str(const char *s)
{
	while (*s) tx_enqueue((uint8_t)*s++);
}

// Kick DMA channel 2 to send a contiguous chunk from the ring buffer.
// Called from kmbox_poll. Copies pending ring bytes into the linear DMA
// staging buffer, then fires a single DMA transfer. Hardware drains one
// byte per TDRE — zero CPU until next poll.
static void tx_flush(void)
{
	// If previous transfer still in flight, let it finish
	if (!(DMA_TCD2_CSR & DMA_TCD_CSR_DONE) &&
	    DMA_TCD2_CITER_ELINKNO != DMA_TCD2_BITER_ELINKNO)
		return;
	DMA_CDNE = 2; // clear DONE flag for next transfer

	// Copy pending ring bytes into linear DMA buffer
	uint8_t count = 0;
	while (tx_tail_pos != tx_head && count < DMA_TX_BUF_SIZE) {
		dma_tx_buf[count++] = tx_ring[tx_tail_pos];
		tx_tail_pos = (tx_tail_pos + 1) & (TX_RING_SIZE - 1);
	}
	if (count == 0) return;

	// Update only the 3 TCD fields that change per flush (static fields set in kmbox_init)
	DMA_TCD2_SADDR = (volatile const void *)dma_tx_buf;
	DMA_TCD2_CITER_ELINKNO = count;
	DMA_TCD2_BITER_ELINKNO = count;
	DMA_TCD2_CSR = DMA_TCD_CSR_DREQ; // auto-disable requests on completion
	DMA_SERQ = 2; // enable channel 2 requests — DMA fires on each TDRE
}
void kmbox_init(void)
{
#if NET_ENABLED
	// NET mode: skip UART/DMA setup — commands come from Ethernet
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;
	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;
	return;
#endif
	// Enable LPUART6 clock gate
	CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);

	// Pin 1 (GPIO_AD_B0_02) = LPUART6_TX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART6_TX_SELECT_INPUT = 1;

	// Pin 0 (GPIO_AD_B0_03) = LPUART6_RX (ALT2)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) |
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

	// Baud: pick OSR so SBR >= 1 (e.g. 2M @ 24MHz -> OSR=11, SBR=1)
	uint32_t osr;
	if (UART_BAUD <= 460800) {
		osr = 15;
	} else {
		osr = UART_CLOCK / UART_BAUD - 1;
		if (osr < 4) osr = 4;
	}
	uint32_t sbr = UART_CLOCK / (UART_BAUD * (osr + 1));
	if (sbr == 0) sbr = 1;
	LPUART6_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);

	// Disable TE/RE before FIFO config (required by i.MX RT datasheet).
	// uart_init() may have already enabled them when UART_ENABLED=1.
	LPUART6_CTRL = 0;

	// Enable 4-entry RX and TX hardware FIFOs (must be set while TE/RE clear)
	LPUART6_FIFO = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE;
	LPUART6_FIFO |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_RXFLUSH;
	LPUART6_WATER = LPUART_WATER_RXWATER(1);

	LPUART6_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;

	// ---- eDMA channel 0: LPUART6 RX -> ring buffer ----
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
	DMAMUX_CHCFG0 = 0; // disable before reconfiguring
	DMA_TCD0_SADDR = (volatile const void *)&LPUART6_DATA;
	DMA_TCD0_SOFF = 0;
	DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	                DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	DMA_TCD0_NBYTES_MLNO = 1;
	DMA_TCD0_SLAST = 0;
	DMA_TCD0_DADDR = (volatile void *)dma_rx_ring;
	DMA_TCD0_DOFF = 1;
	DMA_TCD0_CITER_ELINKNO = DMA_RX_RING_SIZE;
	DMA_TCD0_BITER_ELINKNO = DMA_RX_RING_SIZE;
	DMA_TCD0_DLASTSGA = -DMA_RX_RING_SIZE;
	DMA_TCD0_CSR = 0; // no DREQ — continuous circular operation
	DMAMUX_CHCFG0 = DMAMUX_SOURCE_LPUART6_RX | DMAMUX_CHCFG_ENBL;
	DMA_SERQ = 0; // enable channel 0 requests
	LPUART6_BAUD |= LPUART_BAUD_RDMAE; // route RX to DMA
	rx_tail = 0;

	// ---- eDMA channel 2: dma_tx_buf -> LPUART6 TX ----
	// Pre-fill static TCD fields once; tx_flush only updates SADDR/CITER/BITER.
	DMAMUX_CHCFG2 = 0;
	DMA_TCD2_SADDR = (volatile const void *)dma_tx_buf;
	DMA_TCD2_SOFF = 1;
	DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	                DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	DMA_TCD2_NBYTES_MLNO = 1;
	DMA_TCD2_SLAST = 0;
	DMA_TCD2_DADDR = (volatile void *)&LPUART6_DATA;
	DMA_TCD2_DOFF = 0;
	DMA_TCD2_DLASTSGA = 0;
	DMA_TCD2_CSR = DMA_TCD_CSR_DONE; // mark idle so first tx_flush succeeds
	DMAMUX_CHCFG2 = DMAMUX_SOURCE_LPUART6_TX | DMAMUX_CHCFG_ENBL;
	LPUART6_BAUD |= LPUART_BAUD_TDMAE; // route TX FIFO ready to DMA

	tx_head = 0;
	tx_tail_pos = 0;
	proto_mode = PROTO_IDLE;
	kb_parse_state = KB_SYNC2;
	mk_parse_state = MK_CMD;
	ferrum_pos = 0;
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;

	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;

	ferrum_init();
	makcu_init();
	smooth_init(1000); // default 1kHz, main.c re-inits with actual rate
}

// ---- HID report descriptor parser ----

// Parse mouse HID report descriptor to find X, Y, wheel bit positions and sizes.
// This is essential because mice vary: 8-bit X/Y (boot-like), 16-bit X/Y (gaming),
// 12-bit fields, etc.  Without parsing, we'd guess wrong and corrupt reports.
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
					} else if (u == 0x38) { // Wheel
						mouse_layout.wheel_bit = bit_pos;
						mouse_layout.wheel_size = report_size;
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
}

// Read signed value from bit position in a report buffer.
// bit_off is relative to data start; data_off is byte offset of data (0 or 1).
static int32_t read_report_field(const uint8_t *buf, uint16_t bit_off,
                                 uint8_t bit_size, uint8_t data_off)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) return (int16_t)(buf[byte_idx] | ((uint16_t)buf[byte_idx + 1] << 8));
		if (bit_size == 8)  return (int8_t)buf[byte_idx];
	}

	uint32_t raw = 0;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	for (uint8_t b = 0; b < bytes_needed; b++)
		raw |= (uint32_t)buf[byte_idx + b] << (b * 8);
	raw = (raw >> bit_idx) & ((1u << bit_size) - 1);
	if (raw & (1u << (bit_size - 1)))
		raw |= ~((1u << bit_size) - 1); // sign extend
	return (int32_t)raw;
}

static void write_report_field(uint8_t *buf, uint16_t bit_off,
                               uint8_t bit_size, uint8_t data_off, int32_t value)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			buf[byte_idx]     = (uint8_t)(value & 0xFF);
			buf[byte_idx + 1] = (uint8_t)((value >> 8) & 0xFF);
			return;
		}
		if (bit_size == 8) {
			buf[byte_idx] = (uint8_t)(int8_t)value;
			return;
		}
	}

	uint32_t mask = ((1u << bit_size) - 1) << bit_idx;
	uint32_t val  = ((uint32_t)value & ((1u << bit_size) - 1)) << bit_idx;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
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
	uint32_t stat = LPUART6_STAT;
	if (__builtin_expect(stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF), 0)) {
		LPUART6_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		proto_mode = PROTO_IDLE;
		ferrum_pos = 0;
	}

	if (__builtin_expect(inject.click_release_at, 0) && millis() >= inject.click_release_at) {
		inject.mouse_buttons &= ~inject.click_release_mask;
		inject.mouse_dirty = true;
		inject.click_release_mask = 0;
		inject.click_release_at = 0;
	}

	uint16_t head = ((uint32_t)DMA_TCD0_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	while (rx_tail != head) {
		uint8_t b = dma_rx_ring[rx_tail];
		rx_tail = (rx_tail + 1) & (DMA_RX_RING_SIZE - 1);
		rx_bytes_total++;

		switch (proto_mode) {
		case PROTO_IDLE:
			if (b == KMBOX_SYNC1) {
				proto_mode = PROTO_KMBOX;
				frame_checksum = b;
				kb_parse_state = KB_SYNC2;
			} else if (b == MAKCU_SYNC_BYTE) {
				proto_mode = PROTO_MAKCU;
				mk_parse_state = MK_CMD;
			} else if (b >= 0x20 && b < 0x7F) {
				proto_mode = PROTO_FERRUM;
				ferrum_pos = 0;
				ferrum_line[ferrum_pos++] = (char)b;
			}
			break;
		case PROTO_KMBOX:
			switch (kb_parse_state) {
			case KB_SYNC2:
				if (b == KMBOX_SYNC2) {
					frame_checksum += b;
					kb_parse_state = KB_CMD;
				} else if (b == KMBOX_SYNC1) {
					frame_checksum = b;
				} else {
					proto_mode = PROTO_IDLE;
				}
				break;
			case KB_CMD:
				frame_cmd = b;
				frame_checksum += b;
				kb_parse_state = KB_LEN;
				break;
			case KB_LEN:
				frame_len = b;
				frame_checksum += b;
				if (frame_len > KMBOX_MAX_PAYLOAD) {
					frames_err++;
					proto_mode = PROTO_IDLE;
				} else if (frame_len == 0) {
					kb_parse_state = KB_CHECKSUM;
				} else {
					frame_pos = 0;
					kb_parse_state = KB_PAYLOAD;
				}
				break;
			case KB_PAYLOAD:
				frame_buf[frame_pos++] = b;
				frame_checksum += b;
				if (frame_pos >= frame_len) {
					kb_parse_state = KB_CHECKSUM;
				}
				break;
			case KB_CHECKSUM:
				if ((frame_checksum & 0xFF) == b) {
					dispatch_kmbox_frame();
					frames_ok++;
				} else {
					frames_err++;
				}
				proto_mode = PROTO_IDLE;
				break;
			}
			break;

		case PROTO_MAKCU:
			switch (mk_parse_state) {
			case MK_CMD:
				mk_cmd = b;
				mk_parse_state = MK_LEN_LO;
				break;
			case MK_LEN_LO:
				mk_len = b;
				mk_parse_state = MK_LEN_HI;
				break;
			case MK_LEN_HI:
				mk_len |= (uint16_t)b << 8;
				if (mk_len > MAKCU_MAX_PAYLOAD) {
					frames_err++;
					proto_mode = PROTO_IDLE;
				} else if (mk_len == 0) {
					dispatch_makcu_frame();
					frames_ok++;
					proto_mode = PROTO_IDLE;
				} else {
					mk_pos = 0;
					mk_parse_state = MK_PAYLOAD;
				}
				break;
			case MK_PAYLOAD:
				mk_buf[mk_pos++] = b;
				if (mk_pos >= mk_len) {
					dispatch_makcu_frame();
					frames_ok++;
					proto_mode = PROTO_IDLE;
				}
				break;
			}
			break;
		case PROTO_FERRUM:
			if (b == '\n' || b == '\r') {
				if (ferrum_pos > 0) {
					ferrum_line[ferrum_pos] = '\0';
					dispatch_ferrum_line();
				}
				ferrum_pos = 0;
				proto_mode = PROTO_IDLE;
			} else if (ferrum_pos < FERRUM_MAX_LINE - 1) {
				ferrum_line[ferrum_pos++] = (char)b;
			} else {
				ferrum_pos = 0;
				proto_mode = PROTO_IDLE;
			}
			break;
		}
	}

}

void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len)
{
	if (iface_protocol == 2) {
		if (__builtin_expect(cached_mouse_report_len == 0, 0))
			cached_mouse_report_len = len;

		if (mouse_layout.valid &&
		    (inject.mouse_dx || inject.mouse_dy ||
		     inject.mouse_buttons || inject.mouse_wheel)) {
			uint8_t doff = mouse_layout.data_off;

			report[doff] |= inject.mouse_buttons;

			int32_t rx = read_report_field(report, mouse_layout.x_bit,
			                               mouse_layout.x_size, doff);
			int32_t mx = rx + inject.mouse_dx;
			if (mx > mouse_layout.x_max) mx = mouse_layout.x_max;
			if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
			write_report_field(report, mouse_layout.x_bit,
			                   mouse_layout.x_size, doff, mx);

			int32_t ry = read_report_field(report, mouse_layout.y_bit,
			                               mouse_layout.y_size, doff);
			int32_t my = ry + inject.mouse_dy;
			if (my > mouse_layout.y_max) my = mouse_layout.y_max;
			if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
			write_report_field(report, mouse_layout.y_bit,
			                   mouse_layout.y_size, doff, my);

			if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0) {
				int32_t rw = read_report_field(report, mouse_layout.wheel_bit,
				                               mouse_layout.wheel_size, doff);
				int32_t mw = rw + inject.mouse_wheel;
				if (mw > mouse_layout.w_max) mw = mouse_layout.w_max;
				if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
				write_report_field(report, mouse_layout.wheel_bit,
				                   mouse_layout.wheel_size, doff, mw);
			}

			inject.mouse_dx = 0;
			inject.mouse_dy = 0;
			inject.mouse_wheel = 0;
			inject.mouse_dirty = false;
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

void kmbox_send_pending(void)
{
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

		write_report_field(synth, mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, dx);
		write_report_field(synth, mouse_layout.y_bit,
		                   mouse_layout.y_size, doff, dy);

		if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0) {
			int32_t w = inject.mouse_wheel;
			if (w > mouse_layout.w_max) w = mouse_layout.w_max;
			if (w < -mouse_layout.w_max) w = -mouse_layout.w_max;
			write_report_field(synth, mouse_layout.wheel_bit,
			                   mouse_layout.wheel_size, doff, w);
		}
		uint8_t rlen = cached_mouse_report_len;
		if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
		usb_device_send_report(cached_mouse_ep, synth, rlen);
		inject.mouse_dx = 0;
		inject.mouse_dy = 0;
		inject.mouse_wheel = 0;
		inject.mouse_dirty = false;
	}
	if (inject.kb_dirty && cached_kb_ep) {
		uint8_t synth[8];
		synth[0] = inject.kb_modifier;
		synth[1] = 0;
		memcpy(&synth[2], inject.kb_keys, 6);
		usb_device_send_report(cached_kb_ep, synth, 8);
	}
}

void kmbox_inject_smooth(int16_t dx, int16_t dy)
{
	inject.mouse_dx += dx;
	inject.mouse_dy += dy;
	inject.mouse_dirty = true;
}

uint32_t kmbox_frame_count(void) { return frames_ok; }
uint32_t kmbox_error_count(void) { return frames_err; }
uint32_t kmbox_rx_byte_count(void) { return rx_bytes_total; }
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

static void dispatch_kmbox_frame(void)
{
	switch (frame_cmd) {
	case KMBOX_CMD_MOUSE_MOVE:
		if (frame_len >= 4) {
			inject.mouse_dx += (int16_t)(frame_buf[0] | (frame_buf[1] << 8));
			inject.mouse_dy += (int16_t)(frame_buf[2] | (frame_buf[3] << 8));
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_BUTTON:
		if (frame_len >= 1) {
			inject.mouse_buttons = frame_buf[0];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_WHEEL:
		if (frame_len >= 1) {
			inject.mouse_wheel += (int8_t)frame_buf[0];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_MOUSE_ALL:
		if (frame_len >= 6) {
			inject.mouse_buttons = frame_buf[0];
			inject.mouse_dx += (int16_t)(frame_buf[1] | (frame_buf[2] << 8));
			inject.mouse_dy += (int16_t)(frame_buf[3] | (frame_buf[4] << 8));
			inject.mouse_wheel += (int8_t)frame_buf[5];
			inject.mouse_dirty = true;
		}
		break;

	case KMBOX_CMD_KEYBOARD:
		if (frame_len >= 8) {
			inject.kb_modifier = frame_buf[0];
			memcpy(inject.kb_keys, &frame_buf[2], 6);
			inject.kb_dirty = true;
		}
		break;

	case KMBOX_CMD_KEYBOARD_REL:
		inject.kb_modifier = 0;
		memset(inject.kb_keys, 0, 6);
		inject.kb_dirty = true; // must send zero-report to release keys on host
		break;

	case KMBOX_CMD_SMOOTH_MOVE:
		if (frame_len >= 4) {
			int16_t x = (int16_t)(frame_buf[0] | (frame_buf[1] << 8));
			int16_t y = (int16_t)(frame_buf[2] | (frame_buf[3] << 8));
			smooth_inject(x, y);
		}
		break;

	case KMBOX_CMD_SMOOTH_CONFIG:
		if (frame_len >= 1) {
			smooth_set_max_per_frame((int16_t)frame_buf[0]);
		}
		break;

	case KMBOX_CMD_SMOOTH_CLEAR:
		smooth_clear();
		break;

	case KMBOX_CMD_PING:
		tx_enqueue(0xFE);
		break;

	default:
		break;
	}
}
static void makcu_send_response(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
	tx_enqueue(MAKCU_SYNC_BYTE);
	tx_enqueue(cmd);
	tx_enqueue(plen & 0xFF);
	tx_enqueue((plen >> 8) & 0xFF);
	if (plen > 0)
		tx_enqueue_buf(payload, plen);
}
static void dispatch_makcu_frame(void)
{
	makcu_result_t result;
	if (makcu_parse_command(mk_cmd, mk_buf, mk_len, &result)) {
		if (result.has_mouse) {
			apply_mouse_result(result.mouse_dx, result.mouse_dy,
			                   result.mouse_buttons, result.mouse_wheel,
			                   true);
		}
		if (result.has_keyboard) {
			inject.kb_modifier = result.kb_modifier;
			memcpy(inject.kb_keys, result.kb_keys, 6);
			inject.kb_dirty = true;
		}
		if (result.click_release) {
			inject.click_release_mask = result.mouse_buttons;
			inject.click_release_at = millis() + 30;
		}
		if (result.needs_response) {
			makcu_send_response(result.resp_cmd, result.resp_payload,
			                    result.resp_payload_len);
		}
	}
}

static void dispatch_ferrum_line(void)
{
	ferrum_result_t result;
	if (ferrum_parse_line(ferrum_line, ferrum_pos, &result)) {
		if (result.has_mouse) {
			apply_mouse_result(result.mouse_dx, result.mouse_dy,
			                   result.mouse_buttons, result.mouse_wheel,
			                   true);
		}
		if (result.click_release) {
			inject.click_release_mask = result.mouse_buttons;
			inject.click_release_at = millis() + 30;
		}
		if (result.needs_response) {
			if (result.text_response)
				tx_enqueue_str(result.text_response);
			static const uint8_t resp[] = { '>', '>', '>', '\r', '\n' };
			tx_enqueue_buf(resp, 5);
		}
	}
}
