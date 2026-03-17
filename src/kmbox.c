// Multi-protocol command injection over UART (LPUART6 — pins 0/1)
// Auto-detects and handles three protocols on the same UART:
//   - KMBox B binary: sync 0x57 0xAB, cmd, len, payload, checksum
//   - MAKCU text:     km.move(dx,dy)\r\n  km.left(1)\r\n  (makcu-rs format)
//   - Ferrum text:    km.mouse_move(dx,dy)\r  km.mouse_button_press(code)\r
//
// MAKCU and Ferrum are both text protocols starting with km.* — auto-detected
// by command name.  The text line accumulator handles both; dispatch tries
// Ferrum first (data commands don't overlap), then MAKCU.
// km.version() matches both — Ferrum wins to return the correct identity.
//
// Status LEDs on carrier board BT module footprint (unpopulated):
//   D31 = LINK  (GPIO_EMC_37 / GPIO3[23]) — solid when receiving UART data
//   D30 = STATE (GPIO_EMC_36 / GPIO3[22]) — toggles on valid frame dispatch
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

// ---- Hardware: LPUART6 on Teensy pins 0 (RX) / 1 (TX) ----
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

// ---- Status LEDs (carrier board BT footprint, accent LEDs) ----
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

static uint32_t link_last_rx_time; // millis() of last RX byte (for link timeout)

// ---- eDMA RX ring buffer (non-cacheable via MPU region 10) ----
#define DMA_RX_RING_SIZE 256
static uint8_t dma_rx_ring[DMA_RX_RING_SIZE]
	__attribute__((section(".dmabuffers"), aligned(DMA_RX_RING_SIZE)));
static volatile uint16_t rx_tail;

// ---- eDMA TX: staging ring -> DMA TX channel -> UART DATA ----
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
	PROTO_TEXT         // accumulating text line (MAKCU or Ferrum — detected by command name)
} proto_mode_t;

// ---- KMBox B parser states ----
typedef enum {
	KB_SYNC2,
	KB_CMD,
	KB_LEN,
	KB_PAYLOAD,
	KB_CHECKSUM
} kb_state_t;

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

	// Keyboard press auto-release: key to release after kb_release_at
	uint8_t  kb_release_key;
	uint32_t kb_release_at;   // millis() timestamp, 0 = inactive
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

// Text line buffer (shared by MAKCU and Ferrum text protocols)
static char         text_line[FERRUM_MAX_LINE];
static uint8_t      text_pos;

// Shared injection state
static kmbox_inject_t inject;
static uint32_t frames_ok;
static uint32_t frames_err;
static uint32_t rx_bytes_total;

// Detected protocol: 0=none, 1=KMBox B, 2=Makcu, 3=Ferrum
static uint8_t detected_proto;

// Hardware UART error counters (diagnose signal quality vs overrun)
static uint32_t uart_overrun_count;   // OR: DMA couldn't drain FIFO fast enough
static uint32_t uart_framing_count;   // FE: baud mismatch or signal integrity
static uint32_t uart_noise_count;     // NF: electrical noise on line

// TX DMA watchdog: detect stuck transfers and force-recover
static uint32_t tx_bytes_total;       // total bytes sent via DMA
static uint32_t tx_stuck_count;       // consecutive polls where DMA was busy
#define TX_STUCK_THRESHOLD 5000       // ~5k polls ≈ 50ms at 100kHz poll rate

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
	uint8_t  report_id;     // report ID that owns X (0 = no report ID byte)
	uint8_t  y_report_id;   // report ID that owns Y
	uint8_t  wheel_report_id; // report ID that owns wheel
	uint8_t  data_off;      // byte offset of data start (0 or 1)
	bool     valid;         // true if X and Y were found
	int16_t  x_max;         // precomputed (1 << (x_size-1)) - 1
	int16_t  y_max;         // precomputed (1 << (y_size-1)) - 1
	int16_t  w_max;         // precomputed (1 << (wheel_size-1)) - 1
	// Precomputed fast-path offsets (valid when fast_path == true)
	bool     fast_path;     // true = X,Y,W are byte-aligned 8 or 16-bit, same report_id
	uint8_t  x_byte;        // byte offset of X (absolute, includes data_off)
	uint8_t  y_byte;        // byte offset of Y
	uint8_t  w_byte;        // byte offset of wheel (0xFF = none)
	bool     x_is16;        // true = 16-bit LE, false = 8-bit signed
	bool     y_is16;
	bool     w_is16;
} mouse_layout;
static uint8_t cached_mouse_report_len; // actual report length from first real report

// Track whether a merge happened this cycle (reset each poll)
static bool merged_this_cycle;

// ---- Forward declarations ----
static void dispatch_kmbox_frame(void);
static void dispatch_text_line(void);
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
	DMA_CDNE = KM_TX_CH; // clear DONE flag for next transfer
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
	KM_TX_CSR = DMA_TCD_CSR_DREQ; // auto-disable requests on completion
	DMA_SERQ = KM_TX_CH; // enable TX DMA channel requests
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

	// Baud: pick OSR so SBR >= 1 (e.g. 2M @ 24MHz -> OSR=11, SBR=1)
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
	KM_RX_CSR = 0; // no DREQ — continuous circular operation
	KM_RX_DMAMUX = KM_RX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	DMA_SERQ = KM_RX_CH;
	KM_UART_BAUD |= LPUART_BAUD_RDMAE; // route RX to DMA
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
	KM_TX_CSR = DMA_TCD_CSR_DONE; // mark idle so first tx_flush succeeds
	KM_TX_DMAMUX = KM_TX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	KM_UART_BAUD |= LPUART_BAUD_TDMAE; // route TX FIFO ready to DMA

	tx_head = 0;
	tx_tail_pos = 0;
	proto_mode = PROTO_IDLE;
	kb_parse_state = KB_SYNC2;
	text_pos = 0;
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

	ferrum_init();
	makcu_init();
	smooth_init(1000); // default 1kHz, main.c re-inits with actual rate
}

// ---- HID report descriptor parser ----
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

	// Precompute fast-path: byte-aligned 8 or 16-bit X/Y on same report_id
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

// Read signed value from bit position in a report buffer.
// bit_off is relative to data start; data_off is byte offset of data (0 or 1).
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
	uint32_t stat = KM_UART_STAT;
	if (__builtin_expect(stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF), 0)) {
		if (stat & LPUART_STAT_OR) uart_overrun_count++;
		if (stat & LPUART_STAT_FE) uart_framing_count++;
		if (stat & LPUART_STAT_NF) uart_noise_count++;
		KM_UART_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		if (stat & (LPUART_STAT_OR | LPUART_STAT_FE)) {
			proto_mode = PROTO_IDLE;
			text_pos = 0;
		}
		GPIO1_DR_TOGGLE = STATUS_LED_BIT;
	}

	if (__builtin_expect(inject.click_release_at, 0) && millis() >= inject.click_release_at) {
		inject.mouse_buttons &= ~inject.click_release_mask;
		inject.mouse_dirty = true;
		inject.click_release_mask = 0;
		inject.click_release_at = 0;
	}

	if (__builtin_expect(inject.kb_release_at, 0) && millis() >= inject.kb_release_at) {
		for (int i = 0; i < 6; i++) {
			if (inject.kb_keys[i] == inject.kb_release_key) {
				inject.kb_keys[i] = 0;
				break;
			}
		}
		inject.kb_dirty = true;
		inject.kb_release_key = 0;
		inject.kb_release_at = 0;
	}

	uint16_t head = ((uint32_t)KM_RX_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	if (head != rx_tail) {
		GPIO3_DR_TOGGLE = LINK_LED_BIT;
		link_last_rx_time = millis();
	} else if (link_last_rx_time && (millis() - link_last_rx_time) > 50) {
		GPIO3_DR_CLEAR = LINK_LED_BIT;
		link_last_rx_time = 0;
	}
	if (proto_mode != PROTO_IDLE && head == rx_tail) {
		if (!link_last_rx_time || (millis() - link_last_rx_time) > 5) {
			proto_mode = PROTO_IDLE;
			text_pos = 0;
		}
	}
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
			} else if (b >= 0x20 && b < 0x7F) {
				// Printable ASCII → text protocol (MAKCU or Ferrum)
				proto_mode = PROTO_TEXT;
				text_pos = 0;
				text_line[text_pos++] = (char)b;
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
					detected_proto = 1;
					GPIO3_DR_TOGGLE = STATE_LED_BIT;
				} else {
					frames_err++;
				}
				proto_mode = PROTO_IDLE;
				break;
			}
			break;

		case PROTO_TEXT:
			if (b == '\n' || b == '\r') {
				if (text_pos > 0) {
					text_line[text_pos] = '\0';
					dispatch_text_line();
					GPIO3_DR_TOGGLE = STATE_LED_BIT;
				}
				text_pos = 0;
				proto_mode = PROTO_IDLE;
			} else if (text_pos < FERRUM_MAX_LINE - 1) {
				text_line[text_pos++] = (char)b;
			} else {
				text_pos = 0;
				proto_mode = PROTO_IDLE;
			}
			break;
		}
	}

	// Flush any enqueued response bytes immediately (don't wait for next poll)
	tx_flush();
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
				// Fast path: byte-aligned fields, no function call overhead
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
				// Generic bit-field path (fallback for exotic devices)
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
	inject.kb_release_key = key;
	inject.kb_release_at = millis() + delay_ms;
}

// Send a properly framed KMBox B response: SYNC1 SYNC2 cmd len [payload] checksum
static void kmbox_send_response(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
	uint8_t cksum = KMBOX_SYNC1 + KMBOX_SYNC2 + cmd + plen;
	tx_enqueue(KMBOX_SYNC1);
	tx_enqueue(KMBOX_SYNC2);
	tx_enqueue(cmd);
	tx_enqueue(plen);
	for (uint8_t i = 0; i < plen; i++) {
		tx_enqueue(payload[i]);
		cksum += payload[i];
	}
	tx_enqueue(cksum);
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
		kmbox_send_response(KMBOX_CMD_PING, NULL, 0);
		break;

	default:
		break;
	}
}
// Emit MAKCU tracked-command response: >>> OK#id\r\n  or  >>>\r\n for untracked
static void send_makcu_response(const makcu_result_t *r)
{
	if (r->text_response)
		tx_enqueue_str(r->text_response);
	if (r->track_id) {
		// Tracked: >>> OK#<id>\r\n
		char buf[32];
		uint8_t pos = 0;
		buf[pos++] = '>'; buf[pos++] = '>'; buf[pos++] = '>';
		buf[pos++] = ' '; buf[pos++] = 'O'; buf[pos++] = 'K';
		buf[pos++] = '#';
		// Convert track_id to decimal
		char digits[10];
		uint8_t nd = 0;
		uint32_t id = r->track_id;
		do { digits[nd++] = '0' + (id % 10); id /= 10; } while (id);
		while (nd--) buf[pos++] = digits[nd];
		buf[pos++] = '\r'; buf[pos++] = '\n';
		tx_enqueue_buf((const uint8_t *)buf, pos);
	} else {
		static const uint8_t resp[] = { '>', '>', '>', '\r', '\n' };
		tx_enqueue_buf(resp, 5);
	}
}

// Dispatch a complete text line — tries Ferrum first, then MAKCU.
// Both protocols share the km.* prefix but use different command names.
// Ferrum first is safe: data commands don't overlap (mouse_move vs move, etc.)
// and km.version() matches both — Ferrum must win to return the correct identity.
static void dispatch_text_line(void)
{
	// Try Ferrum text protocol first (km.mouse_move, km.mouse_button_press, etc.)
	ferrum_result_t fr;
	if (ferrum_parse_line(text_line, text_pos, &fr)) {
		if (fr.has_mouse) {
			apply_mouse_result(fr.mouse_dx, fr.mouse_dy,
			                   fr.mouse_buttons, fr.mouse_wheel,
			                   true);
		}
		if (fr.has_keyboard) {
			inject.kb_modifier = fr.kb_modifier;
			memcpy(inject.kb_keys, fr.kb_keys, 6);
			inject.kb_dirty = true;
		}
		if (fr.click_release) {
			inject.click_release_mask = fr.mouse_buttons;
			inject.click_release_at = millis() + 30;
		}
		if (fr.kb_click_release) {
			inject.kb_release_key = fr.kb_release_key;
			inject.kb_release_at = millis() + 30;
		}
		if (fr.needs_response) {
			if (fr.text_response)
				tx_enqueue_str(fr.text_response);
			static const uint8_t resp[] = { '>', '>', '>', '\r', '\n' };
			tx_enqueue_buf(resp, 5);
		}
		frames_ok++;
		detected_proto = 3;
		return;
	}

	// Try MAKCU text protocol (km.move, km.left, km.wheel, etc.)
	makcu_result_t mk;
	if (makcu_parse_line(text_line, text_pos, &mk)) {
		if (mk.has_mouse) {
			apply_mouse_result(mk.mouse_dx, mk.mouse_dy,
			                   mk.mouse_buttons, mk.mouse_wheel,
			                   true);
		}
		if (mk.has_keyboard) {
			inject.kb_modifier = mk.kb_modifier;
			memcpy(inject.kb_keys, mk.kb_keys, 6);
			inject.kb_dirty = true;
		}
		if (mk.click_release) {
			inject.click_release_mask = mk.mouse_buttons;
			inject.click_release_at = millis() + 30;
		}
		if (mk.kb_click_release) {
			inject.kb_release_key = mk.kb_release_key;
			inject.kb_release_at = millis() + 30;
		}
		if (mk.needs_response) {
			send_makcu_response(&mk);
		}
		frames_ok++;
		detected_proto = 2;
		return;
	}

	// Unrecognized text line
	frames_err++;
}
