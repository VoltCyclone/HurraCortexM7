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
#define UART_BAUD  115200
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

	// Set up TCD2: dma_tx_buf[0..count-1] -> LPUART6_DATA, 1 byte per TDRE
	DMA_TCD2_SADDR = (volatile const void *)dma_tx_buf;
	DMA_TCD2_SOFF = 1;
	DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	                DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	DMA_TCD2_NBYTES_MLNO = 1;
	DMA_TCD2_SLAST = 0;
	DMA_TCD2_DADDR = (volatile void *)&LPUART6_DATA;
	DMA_TCD2_DOFF = 0;
	DMA_TCD2_CITER_ELINKNO = count;
	DMA_TCD2_BITER_ELINKNO = count;
	DMA_TCD2_DLASTSGA = 0;
	DMA_TCD2_CSR = DMA_TCD_CSR_DREQ; // auto-disable requests on completion
	DMA_SERQ = 2; // enable channel 2 requests — DMA fires on each TDRE
}

// ---- Public API ----

void kmbox_init(void)
{
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

	// Baud: dynamic OSR for high baud rates
	uint32_t osr = (UART_BAUD > 460800) ? 25 : 15;
	uint32_t sbr = UART_CLOCK / (UART_BAUD * (osr + 1));
	if (sbr == 0) sbr = 1;
	LPUART6_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);

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
	DMAMUX_CHCFG2 = 0;
	DMA_TCD2_CSR = 0;
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

	ferrum_init();
	makcu_init();
	smooth_init();
}

void kmbox_cache_endpoints(const captured_descriptors_t *desc)
{
	cached_mouse_ep = 0;
	cached_kb_ep = 0;
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (desc->ifaces[i].interrupt_ep == 0) continue;
		uint8_t ep = desc->ifaces[i].interrupt_ep & 0x0F;
		if (desc->ifaces[i].iface_protocol == 2 && !cached_mouse_ep) {
			cached_mouse_ep = ep;
			cached_mouse_maxpkt = desc->ifaces[i].interrupt_maxpkt;
		} else if (desc->ifaces[i].iface_protocol == 1 && !cached_kb_ep) {
			cached_kb_ep = ep;
		}
	}
}

void kmbox_poll(void)
{
	merged_this_cycle = false;

	// Kick DMA TX if there are pending bytes (non-blocking)
	tx_flush();

	// Clear line errors and reset parser if any occurred
	uint32_t stat = LPUART6_STAT;
	if (stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF)) {
		LPUART6_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		proto_mode = PROTO_IDLE;
		ferrum_pos = 0;
	}

	// Process pending click release timer
	if (inject.click_release_at && millis() >= inject.click_release_at) {
		inject.mouse_buttons &= ~inject.click_release_mask;
		inject.mouse_dirty = true;
		inject.click_release_mask = 0;
		inject.click_release_at = 0;
	}

	// Drain all DMA-received bytes from ring buffer
	uint16_t head = ((uint32_t)DMA_TCD0_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	while (rx_tail != head) {
		uint8_t b = dma_rx_ring[rx_tail];
		rx_tail = (rx_tail + 1) & (DMA_RX_RING_SIZE - 1);
		rx_bytes_total++;

		switch (proto_mode) {
		case PROTO_IDLE:
			// Auto-detect protocol from first byte
			if (b == KMBOX_SYNC1) {
				// KMBox B: first sync byte 0x57
				proto_mode = PROTO_KMBOX;
				frame_checksum = b;
				kb_parse_state = KB_SYNC2;
			} else if (b == MAKCU_SYNC_BYTE) {
				// Makcu: sync byte 0x50
				proto_mode = PROTO_MAKCU;
				mk_parse_state = MK_CMD;
			} else if (b >= 0x20 && b < 0x7F) {
				// Printable ASCII — start Ferrum text line
				proto_mode = PROTO_FERRUM;
				ferrum_pos = 0;
				ferrum_line[ferrum_pos++] = (char)b;
			}
			// else: discard non-printable, non-sync bytes
			break;

		// ---- KMBox B binary protocol ----
		case PROTO_KMBOX:
			switch (kb_parse_state) {
			case KB_SYNC2:
				if (b == KMBOX_SYNC2) {
					frame_checksum += b;
					kb_parse_state = KB_CMD;
				} else if (b == KMBOX_SYNC1) {
					// Could be restart — keep sync1 checksum
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

		// ---- Makcu binary protocol ----
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

		// ---- Ferrum text protocol ----
		case PROTO_FERRUM:
			if (b == '\n' || b == '\r') {
				// End of line — dispatch
				if (ferrum_pos > 0) {
					ferrum_line[ferrum_pos] = '\0';
					dispatch_ferrum_line();
				}
				ferrum_pos = 0;
				proto_mode = PROTO_IDLE;
			} else if (ferrum_pos < FERRUM_MAX_LINE - 1) {
				ferrum_line[ferrum_pos++] = (char)b;
			} else {
				// Line too long — discard
				ferrum_pos = 0;
				proto_mode = PROTO_IDLE;
			}
			break;
		}
	}

}

void kmbox_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len)
{
	if (iface_protocol == 2 && inject.mouse_dirty) {
		// Mouse merge — buttons OR, deltas add
		report[0] |= inject.mouse_buttons;

		if (len == 3) {
			// Boot protocol: X and Y are int8
			int16_t mx = (int8_t)report[1] + inject.mouse_dx;
			int16_t my = (int8_t)report[2] + inject.mouse_dy;
			if (mx > 127) mx = 127;
			if (mx < -127) mx = -127;
			if (my > 127) my = 127;
			if (my < -127) my = -127;
			report[1] = (uint8_t)(int8_t)mx;
			report[2] = (uint8_t)(int8_t)my;
		} else if (len >= 5) {
			// Extended: X and Y are int16 LE at bytes 1-4
			int32_t rx = (int16_t)(report[1] | (report[2] << 8));
			int32_t ry = (int16_t)(report[3] | (report[4] << 8));
			int32_t mx = rx + inject.mouse_dx;
			int32_t my = ry + inject.mouse_dy;
			if (mx > 32767) mx = 32767;
			if (mx < -32767) mx = -32767;
			if (my > 32767) my = 32767;
			if (my < -32767) my = -32767;
			report[1] = mx & 0xFF;
			report[2] = (mx >> 8) & 0xFF;
			report[3] = my & 0xFF;
			report[4] = (my >> 8) & 0xFF;

			if (len >= 6 && inject.mouse_wheel != 0) {
				int16_t w = (int8_t)report[5] + inject.mouse_wheel;
				if (w > 127) w = 127;
				if (w < -127) w = -127;
				report[5] = (uint8_t)(int8_t)w;
			}
		}

		// Clear one-shot deltas after merge
		inject.mouse_dx = 0;
		inject.mouse_dy = 0;
		inject.mouse_wheel = 0;
		inject.mouse_dirty = false;
		merged_this_cycle = true;

	} else if (iface_protocol == 1 && inject.kb_dirty) {
		// Keyboard merge (8-byte boot report)
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

	// Send synthetic mouse report if dirty (direct EP, no iface scan)
	if (inject.mouse_dirty && cached_mouse_ep) {
		uint8_t synth[8];
		memset(synth, 0, sizeof(synth));
		synth[0] = inject.mouse_buttons;

		uint8_t rlen;
		if (cached_mouse_maxpkt <= 3) {
			int16_t dx = inject.mouse_dx;
			int16_t dy = inject.mouse_dy;
			if (dx > 127) dx = 127;
			if (dx < -127) dx = -127;
			if (dy > 127) dy = 127;
			if (dy < -127) dy = -127;
			synth[1] = (uint8_t)(int8_t)dx;
			synth[2] = (uint8_t)(int8_t)dy;
			rlen = 3;
		} else {
			synth[1] = inject.mouse_dx & 0xFF;
			synth[2] = (inject.mouse_dx >> 8) & 0xFF;
			synth[3] = inject.mouse_dy & 0xFF;
			synth[4] = (inject.mouse_dy >> 8) & 0xFF;
			synth[5] = (uint8_t)inject.mouse_wheel;
			rlen = (cached_mouse_maxpkt < 8) ? (uint8_t)cached_mouse_maxpkt : 8;
		}

		usb_device_send_report(cached_mouse_ep, synth, rlen);

		inject.mouse_dx = 0;
		inject.mouse_dy = 0;
		inject.mouse_wheel = 0;
		inject.mouse_dirty = false;
	}

	// Send synthetic keyboard report if dirty (direct EP, no iface scan)
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

// ---- Internal helpers ----

// Apply a mouse command result to the inject state.
// If use_smooth is true, movement goes through smooth queue.
static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel, bool use_smooth)
{
	inject.mouse_buttons = buttons;
	inject.mouse_wheel += wheel;
	inject.mouse_dirty = true;

	if (use_smooth && (dx != 0 || dy != 0)) {
		smooth_inject(dx, dy);
	} else {
		inject.mouse_dx += dx;
		inject.mouse_dy += dy;
	}
}

// ---- KMBox B frame dispatch ----
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

// Enqueue a Makcu binary response: [0x50][cmd][len_lo][len_hi][payload...]
static void makcu_send_response(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
	tx_enqueue(MAKCU_SYNC_BYTE);
	tx_enqueue(cmd);
	tx_enqueue(plen & 0xFF);
	tx_enqueue((plen >> 8) & 0xFF);
	if (plen > 0)
		tx_enqueue_buf(payload, plen);
}

// ---- Makcu frame dispatch ----
static void dispatch_makcu_frame(void)
{
	makcu_result_t result;
	if (makcu_parse_command(mk_cmd, mk_buf, mk_len, &result)) {
		if (result.has_mouse) {
			apply_mouse_result(result.mouse_dx, result.mouse_dy,
			                   result.mouse_buttons, result.mouse_wheel,
			                   false);
		}
		if (result.has_keyboard) {
			inject.kb_modifier = result.kb_modifier;
			memcpy(inject.kb_keys, result.kb_keys, 6);
			inject.kb_dirty = true;
		}
		if (result.click_release) {
			// Schedule button release ~30ms after press
			inject.click_release_mask = result.mouse_buttons;
			inject.click_release_at = millis() + 30;
		}
		if (result.needs_response) {
			makcu_send_response(result.resp_cmd, result.resp_payload,
			                    result.resp_payload_len);
		}
	}
}

// ---- Ferrum line dispatch ----
static void dispatch_ferrum_line(void)
{
	ferrum_result_t result;
	if (ferrum_parse_line(ferrum_line, ferrum_pos, &result)) {
		if (result.has_mouse) {
			apply_mouse_result(result.mouse_dx, result.mouse_dy,
			                   result.mouse_buttons, result.mouse_wheel,
			                   false);
		}
		if (result.click_release) {
			// Schedule button release ~30ms after click press
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
