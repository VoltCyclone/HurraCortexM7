// HC-05 Bluetooth SPP input over LPUART7 (Teensy pins D28 TX / D29 RX)
// D30 = STA pin (GPIO_EMC_36 / GPIO3[22]) — input, high when paired
//
// Same multi-protocol auto-detect as wired UART: KMBox B binary, text (Makcu/Ferrum).
// Parsed commands feed into the shared injection state via kmbox_inject_*().
//
// DMA channels: 5 (RX), 6 (TX)

#include "bt.h"
#include "kmbox.h"
#include "smooth.h"
#include "ferrum.h"
#include "makcu.h"
#include "imxrt.h"
#include <string.h>

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// ---- UART constants ----
#ifndef BT_BAUD
#define BT_BAUD 921600
#endif
#define BT_CLOCK 24000000
#define BT_AT_BAUD 38400  // HC-05 full AT mode uses fixed 38400

// ---- Hardware: LPUART7 on Teensy pins D28 (TX) / D29 (RX) ----
// D28 = GPIO_EMC_32 ALT2 = LPUART7_TX
// D29 = GPIO_EMC_31 ALT2 = LPUART7_RX

#define BT_UART_BAUD       LPUART7_BAUD
#define BT_UART_CTRL       LPUART7_CTRL
#define BT_UART_STAT       LPUART7_STAT
#define BT_UART_DATA       LPUART7_DATA
#define BT_UART_FIFO       LPUART7_FIFO
#define BT_UART_WATER      LPUART7_WATER

// ---- eDMA channel 5: BT RX ----
#define BT_RX_SADDR        DMA_TCD5_SADDR
#define BT_RX_SOFF         DMA_TCD5_SOFF
#define BT_RX_ATTR         DMA_TCD5_ATTR
#define BT_RX_NBYTES       DMA_TCD5_NBYTES_MLNO
#define BT_RX_SLAST        DMA_TCD5_SLAST
#define BT_RX_DADDR        DMA_TCD5_DADDR
#define BT_RX_DOFF         DMA_TCD5_DOFF
#define BT_RX_CITER        DMA_TCD5_CITER_ELINKNO
#define BT_RX_BITER        DMA_TCD5_BITER_ELINKNO
#define BT_RX_DLASTSGA     DMA_TCD5_DLASTSGA
#define BT_RX_CSR          DMA_TCD5_CSR
#define BT_RX_DMAMUX       DMAMUX_CHCFG5
#define BT_RX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART7_RX
#define BT_RX_CH           5

// ---- eDMA channel 6: BT TX ----
#define BT_TX_SADDR        DMA_TCD6_SADDR
#define BT_TX_SOFF         DMA_TCD6_SOFF
#define BT_TX_ATTR         DMA_TCD6_ATTR
#define BT_TX_NBYTES       DMA_TCD6_NBYTES_MLNO
#define BT_TX_SLAST        DMA_TCD6_SLAST
#define BT_TX_DADDR        DMA_TCD6_DADDR
#define BT_TX_DOFF         DMA_TCD6_DOFF
#define BT_TX_CITER        DMA_TCD6_CITER_ELINKNO
#define BT_TX_BITER        DMA_TCD6_BITER_ELINKNO
#define BT_TX_DLASTSGA     DMA_TCD6_DLASTSGA
#define BT_TX_CSR          DMA_TCD6_CSR
#define BT_TX_DMAMUX       DMAMUX_CHCFG6
#define BT_TX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART7_TX
#define BT_TX_CH           6

// ---- STA pin: D30 = GPIO_EMC_36 = GPIO3[22] (input) ----
#define BT_STA_BIT         (1u << 22)
// ---- EN pin: D31 = GPIO_EMC_37 = GPIO3[23] (output, controls AT mode) ----
#define BT_EN_BIT          (1u << 23)

// Actual operating baud after AT configuration (may differ from BT_BAUD if AT fails)
static uint32_t bt_active_baud;

// ---- eDMA RX ring buffer (non-cacheable via MPU region 10) ----
#define BT_RX_RING_SIZE 256
static uint8_t bt_rx_ring[BT_RX_RING_SIZE]
	__attribute__((section(".dmabuffers"), aligned(BT_RX_RING_SIZE)));
static volatile uint16_t bt_rx_tail;

// ---- eDMA TX: staging ring -> DMA TX channel -> UART DATA ----
#define BT_TX_RING_SIZE 128
static uint8_t bt_tx_ring[BT_TX_RING_SIZE];
static uint8_t bt_tx_head;
static uint8_t bt_tx_tail_pos;

#define BT_TX_BUF_SIZE 64
static uint8_t bt_tx_buf[BT_TX_BUF_SIZE]
	__attribute__((section(".dmabuffers"), aligned(4)));

// ---- Multi-protocol dispatcher state ----
typedef enum {
	BT_PROTO_IDLE,
	BT_PROTO_KMBOX,
	BT_PROTO_TEXT
} bt_proto_mode_t;

// KMBox B parser states
typedef enum {
	BT_KB_SYNC2,
	BT_KB_CMD,
	BT_KB_LEN,
	BT_KB_PAYLOAD,
	BT_KB_CHECKSUM
} bt_kb_state_t;

// ---- Static state ----
static bt_proto_mode_t bt_proto;
static bt_kb_state_t   bt_kb_state;
static uint8_t         bt_frame_buf[KMBOX_MAX_PAYLOAD];
static uint8_t         bt_frame_cmd;
static uint8_t         bt_frame_len;
static uint8_t         bt_frame_pos;
static uint8_t         bt_frame_cksum;

// Unified text line buffer (shared by MAKCU and Ferrum text protocols)
#define BT_TEXT_MAX_LINE 128
static char             bt_text_line[BT_TEXT_MAX_LINE];
static uint8_t          bt_text_pos;

static uint32_t bt_frames_ok;
static uint32_t bt_frames_err;
static uint32_t bt_rx_bytes;
static uint32_t bt_last_rx_time;

// Detected protocol: 0=none, 1=KMBox B, 2=Makcu, 3=Ferrum
static uint8_t bt_detected_proto;

// ---- TX helpers ----

static void bt_tx_enqueue(uint8_t b)
{
	uint8_t next = (bt_tx_head + 1) & (BT_TX_RING_SIZE - 1);
	if (next == bt_tx_tail_pos) return;
	bt_tx_ring[bt_tx_head] = b;
	bt_tx_head = next;
}

static void bt_tx_enqueue_buf(const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
		bt_tx_enqueue(data[i]);
}

static void bt_tx_enqueue_str(const char *s)
{
	while (*s) bt_tx_enqueue((uint8_t)*s++);
}

static void bt_tx_flush(void)
{
	if (!(BT_TX_CSR & DMA_TCD_CSR_DONE) &&
	    BT_TX_CITER != BT_TX_BITER)
		return;
	DMA_CDNE = BT_TX_CH;

	uint8_t count = 0;
	while (bt_tx_tail_pos != bt_tx_head && count < BT_TX_BUF_SIZE) {
		bt_tx_buf[count++] = bt_tx_ring[bt_tx_tail_pos];
		bt_tx_tail_pos = (bt_tx_tail_pos + 1) & (BT_TX_RING_SIZE - 1);
	}
	if (count == 0) return;

	BT_TX_SADDR = (volatile const void *)bt_tx_buf;
	BT_TX_CITER = count;
	BT_TX_BITER = count;
	BT_TX_CSR = DMA_TCD_CSR_DREQ;
	DMA_SERQ = BT_TX_CH;
}

// ---- Protocol dispatch ----

static void bt_send_kmbox_response(uint8_t cmd, const uint8_t *payload, uint8_t plen)
{
	uint8_t cksum = KMBOX_SYNC1 + KMBOX_SYNC2 + cmd + plen;
	bt_tx_enqueue(KMBOX_SYNC1);
	bt_tx_enqueue(KMBOX_SYNC2);
	bt_tx_enqueue(cmd);
	bt_tx_enqueue(plen);
	for (uint8_t i = 0; i < plen; i++) {
		bt_tx_enqueue(payload[i]);
		cksum += payload[i];
	}
	bt_tx_enqueue(cksum);
}

static void bt_dispatch_kmbox(void)
{
	switch (bt_frame_cmd) {
	case KMBOX_CMD_MOUSE_MOVE:
		if (bt_frame_len >= 4) {
			int16_t dx = (int16_t)(bt_frame_buf[0] | (bt_frame_buf[1] << 8));
			int16_t dy = (int16_t)(bt_frame_buf[2] | (bt_frame_buf[3] << 8));
			kmbox_inject_mouse(dx, dy, 0, 0, false);
		}
		break;

	case KMBOX_CMD_MOUSE_BUTTON:
		if (bt_frame_len >= 1)
			kmbox_inject_mouse(0, 0, bt_frame_buf[0], 0, false);
		break;

	case KMBOX_CMD_MOUSE_WHEEL:
		if (bt_frame_len >= 1)
			kmbox_inject_mouse(0, 0, 0, (int8_t)bt_frame_buf[0], false);
		break;

	case KMBOX_CMD_MOUSE_ALL:
		if (bt_frame_len >= 6) {
			uint8_t buttons = bt_frame_buf[0];
			int16_t dx = (int16_t)(bt_frame_buf[1] | (bt_frame_buf[2] << 8));
			int16_t dy = (int16_t)(bt_frame_buf[3] | (bt_frame_buf[4] << 8));
			int8_t wheel = (int8_t)bt_frame_buf[5];
			kmbox_inject_mouse(dx, dy, buttons, wheel, false);
		}
		break;

	case KMBOX_CMD_KEYBOARD:
		if (bt_frame_len >= 8)
			kmbox_inject_keyboard(bt_frame_buf[0], &bt_frame_buf[2]);
		break;

	case KMBOX_CMD_KEYBOARD_REL: {
		static const uint8_t zeros[6] = {0};
		kmbox_inject_keyboard(0, zeros);
		break;
	}

	case KMBOX_CMD_SMOOTH_MOVE:
		if (bt_frame_len >= 4) {
			int16_t x = (int16_t)(bt_frame_buf[0] | (bt_frame_buf[1] << 8));
			int16_t y = (int16_t)(bt_frame_buf[2] | (bt_frame_buf[3] << 8));
			smooth_inject(x, y);
		}
		break;

	case KMBOX_CMD_SMOOTH_CONFIG:
		if (bt_frame_len >= 1)
			smooth_set_max_per_frame((int16_t)bt_frame_buf[0]);
		break;

	case KMBOX_CMD_SMOOTH_CLEAR:
		smooth_clear();
		break;

	case KMBOX_CMD_PING:
		bt_send_kmbox_response(KMBOX_CMD_PING, NULL, 0);
		break;
	}
}

static void bt_send_makcu_response(uint32_t track_id)
{
	if (track_id) {
		// Tracked: >>> OK#id\r\n
		bt_tx_enqueue_str(">>> OK#");
		char num[12];
		int pos = 0;
		uint32_t v = track_id;
		char tmp[12];
		int tpos = 0;
		do { tmp[tpos++] = '0' + (v % 10); v /= 10; } while (v);
		while (tpos--) num[pos++] = tmp[tpos];
		num[pos] = '\0';
		bt_tx_enqueue_str(num);
		bt_tx_enqueue_str("\r\n");
	} else {
		bt_tx_enqueue_str(">>> OK\r\n");
	}
}

static void bt_dispatch_text_line(void)
{
	// Try MAKCU first (km.move, km.left, etc.)
	makcu_result_t mk;
	if (makcu_parse_line(bt_text_line, bt_text_pos, &mk)) {
		if (mk.has_mouse)
			kmbox_inject_mouse(mk.mouse_dx, mk.mouse_dy,
			                   mk.mouse_buttons, mk.mouse_wheel, true);
		if (mk.has_keyboard)
			kmbox_inject_keyboard(mk.kb_modifier, mk.kb_keys);
		if (mk.click_release)
			kmbox_schedule_click_release(mk.mouse_buttons, 30);
		if (mk.kb_click_release)
			kmbox_schedule_kb_release(mk.kb_release_key, 30);
		if (mk.needs_response) {
			if (mk.text_response)
				bt_tx_enqueue_str(mk.text_response);
			bt_send_makcu_response(mk.track_id);
		}
		bt_frames_ok++;
		bt_detected_proto = 2;
		return;
	}

	// Try Ferrum (km.mouse_move, km.mouse_button_press, etc.)
	ferrum_result_t fr;
	if (ferrum_parse_line(bt_text_line, bt_text_pos, &fr)) {
		if (fr.has_mouse)
			kmbox_inject_mouse(fr.mouse_dx, fr.mouse_dy,
			                   fr.mouse_buttons, fr.mouse_wheel, true);
		if (fr.has_keyboard)
			kmbox_inject_keyboard(fr.kb_modifier, fr.kb_keys);
		if (fr.click_release)
			kmbox_schedule_click_release(fr.mouse_buttons, 30);
		if (fr.kb_click_release)
			kmbox_schedule_kb_release(fr.kb_release_key, 30);
		if (fr.needs_response) {
			if (fr.text_response)
				bt_tx_enqueue_str(fr.text_response);
			static const uint8_t resp[] = { '>', '>', '>', '\r', '\n' };
			bt_tx_enqueue_buf(resp, 5);
		}
		bt_frames_ok++;
		bt_detected_proto = 3;
		return;
	}

	// Unrecognized text line
	bt_frames_err++;
}

// ---- Polled UART helpers for AT command mode (init-time only) ----

static void bt_uart_set_baud(uint32_t baud)
{
	BT_UART_CTRL = 0; // disable TE/RE before reconfiguring
	uint32_t osr;
	if (baud <= 115200) {
		osr = 15;
	} else {
		// For high baud rates, pick OSR to minimise error with SBR=1
		osr = BT_CLOCK / baud - 1;
		if (osr < 4) osr = 4;
		if (osr > 31) osr = 31;
	}
	uint32_t sbr = BT_CLOCK / (baud * (osr + 1));
	if (sbr == 0) sbr = 1;
	BT_UART_BAUD = LPUART_BAUD_OSR(osr) | LPUART_BAUD_SBR(sbr);
	BT_UART_FIFO = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE;
	BT_UART_FIFO |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_RXFLUSH;
	BT_UART_WATER = LPUART_WATER_RXWATER(1);
	BT_UART_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;
}

// Polled TX: send string, blocking
static void bt_at_send(const char *cmd)
{
	while (*cmd) {
		while (!(BT_UART_STAT & LPUART_STAT_TDRE)) {}
		BT_UART_DATA = (uint8_t)*cmd++;
	}
	// Wait for transmit complete
	while (!(BT_UART_STAT & LPUART_STAT_TC)) {}
}

// Polled RX: read until \n or timeout. Returns length read (0 on timeout).
static uint8_t bt_at_recv(char *buf, uint8_t max, uint32_t timeout_ms)
{
	uint8_t pos = 0;
	uint32_t start = millis();
	while ((millis() - start) < timeout_ms && pos < max - 1) {
		if (BT_UART_STAT & LPUART_STAT_RDRF) {
			char c = (char)(BT_UART_DATA & 0xFF);
			if (c == '\n') {
				// Skip empty lines (\r\nOK\r\n firmware variants)
				if (pos == 0) continue;
				buf[pos] = '\0';
				return pos;
			}
			if (c != '\r')
				buf[pos++] = c;
		}
	}
	buf[pos] = '\0';
	return pos;
}

// Drain any pending RX bytes (clear FIFO garbage from mode switch)
static void bt_at_drain(void)
{
	uint32_t start = millis();
	while ((millis() - start) < 50) {
		if (BT_UART_STAT & LPUART_STAT_RDRF)
			(void)BT_UART_DATA;
	}
}

// Send AT command and check for "OK" response. Returns true on success.
static bool bt_at_cmd(const char *cmd, uint32_t timeout_ms)
{
	bt_at_send(cmd);
	char resp[32];
	uint8_t len = bt_at_recv(resp, sizeof(resp), timeout_ms);
	return (len >= 2 && resp[0] == 'O' && resp[1] == 'K');
}

// AT mode sequence: enter AT mode via EN pin, configure target baud, exit.
// Returns the baud rate to use for data mode.
// If AT mode fails (no HC-05, or module unresponsive), returns BT_BAUD unchanged.
static uint32_t bt_at_configure(uint32_t target_baud)
{
	// D31 (EN) HIGH → HC-05 enters full AT mode on next boot/reset
	GPIO3_DR_SET = BT_EN_BIT;
	delay(600); // HC-05 needs ~500ms to boot into AT mode

	// AT mode is fixed 38400 baud
	bt_uart_set_baud(BT_AT_BAUD);
	bt_at_drain();

	// Verify AT mode with a simple AT command
	if (!bt_at_cmd("AT\r\n", 500)) {
		// No response — module not present or not in AT mode.
		// Try once more after a brief pause.
		delay(200);
		bt_at_drain();
		if (!bt_at_cmd("AT\r\n", 500)) {
			// Give up on AT mode, use target baud directly
			GPIO3_DR_CLEAR = BT_EN_BIT;
			delay(200);
			bt_uart_set_baud(target_baud);
			return target_baud;
		}
	}

	// Configure the target baud rate: AT+UART=<baud>,<stop>,<parity>
	// HC-05 supported bauds: 9600..1382400
	{
		char cmd[40] = "AT+UART=";
		char *p = cmd + 8;
		// Convert baud to decimal string
		char tmp[12];
		uint8_t n = 0;
		uint32_t v = target_baud;
		do { tmp[n++] = '0' + (v % 10); v /= 10; } while (v);
		while (n--) *p++ = tmp[n];
		*p++ = ','; *p++ = '0'; // 1 stop bit
		*p++ = ','; *p++ = '0'; // no parity
		*p++ = '\r'; *p++ = '\n'; *p = '\0';
		bt_at_cmd(cmd, 500);
	}

	// Exit AT mode: EN LOW → module resets into data mode
	GPIO3_DR_CLEAR = BT_EN_BIT;
	delay(500); // HC-05 reboots into data mode

	// Switch LPUART7 to the target baud for normal operation
	bt_uart_set_baud(target_baud);
	bt_at_drain();

	return target_baud;
}

// ---- Public API ----

void bt_init(void)
{
	// LPUART7 clock gate
	CCM_CCGR5 |= CCM_CCGR5_LPUART7(CCM_CCGR_ON);

	// Pin mux: D28 (GPIO_EMC_32) = LPUART7_TX ALT2
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_32 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_32 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART7_TX_SELECT_INPUT = 1;

	// Pin mux: D29 (GPIO_EMC_31) = LPUART7_RX ALT2
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_31 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_31 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) |
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART7_RX_SELECT_INPUT = 1;

	// D30 (GPIO_EMC_36) = STA input — GPIO3[22]
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_36 = 5; // ALT5 = GPIO3[22]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_36 =
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(0); // pull-down
	GPIO3_GDIR &= ~BT_STA_BIT; // input

	// D31 (GPIO_EMC_37) = EN output — GPIO3[23]
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_37 = 5; // ALT5 = GPIO3[23]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_37 = IOMUXC_PAD_DSE(6);
	GPIO3_GDIR |= BT_EN_BIT;
	GPIO3_DR_CLEAR = BT_EN_BIT; // start LOW (data mode default)

	// ---- AT command phase: configure HC-05 baud rate ----
	// Uses polled UART at 38400 (AT mode), then switches to target baud.
	bt_active_baud = bt_at_configure(BT_BAUD);

	// ---- eDMA RX: LPUART7 DATA -> ring buffer (ch 5) ----
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
	BT_RX_DMAMUX = 0;
	BT_RX_SADDR = (volatile const void *)&BT_UART_DATA;
	BT_RX_SOFF = 0;
	BT_RX_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	             DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	BT_RX_NBYTES = 1;
	BT_RX_SLAST = 0;
	BT_RX_DADDR = (volatile void *)bt_rx_ring;
	BT_RX_DOFF = 1;
	BT_RX_CITER = BT_RX_RING_SIZE;
	BT_RX_BITER = BT_RX_RING_SIZE;
	BT_RX_DLASTSGA = -BT_RX_RING_SIZE;
	BT_RX_CSR = 0;
	BT_RX_DMAMUX = BT_RX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	DMA_SERQ = BT_RX_CH;
	BT_UART_BAUD |= LPUART_BAUD_RDMAE;
	bt_rx_tail = 0;

	// ---- eDMA TX: bt_tx_buf -> LPUART7 DATA (ch 6) ----
	BT_TX_DMAMUX = 0;
	BT_TX_SADDR = (volatile const void *)bt_tx_buf;
	BT_TX_SOFF = 1;
	BT_TX_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_8BIT) |
	             DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_8BIT);
	BT_TX_NBYTES = 1;
	BT_TX_SLAST = 0;
	BT_TX_DADDR = (volatile void *)&BT_UART_DATA;
	BT_TX_DOFF = 0;
	BT_TX_DLASTSGA = 0;
	BT_TX_CSR = DMA_TCD_CSR_DONE;
	BT_TX_DMAMUX = BT_TX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	BT_UART_BAUD |= LPUART_BAUD_TDMAE;

	bt_tx_head = 0;
	bt_tx_tail_pos = 0;
	bt_proto = BT_PROTO_IDLE;
	bt_kb_state = BT_KB_SYNC2;
	bt_text_pos = 0;
	bt_frames_ok = 0;
	bt_frames_err = 0;
	bt_rx_bytes = 0;
	bt_last_rx_time = 0;
}

void bt_poll(void)
{
	bt_tx_flush();

	// Check UART errors
	uint32_t stat = BT_UART_STAT;
	if (__builtin_expect(stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF), 0)) {
		BT_UART_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		if (stat & (LPUART_STAT_OR | LPUART_STAT_FE)) {
			bt_proto = BT_PROTO_IDLE;
			bt_text_pos = 0;
		}
	}

	uint16_t head = ((uint32_t)BT_RX_DADDR - (uint32_t)bt_rx_ring) & (BT_RX_RING_SIZE - 1);
	if (head != bt_rx_tail)
		bt_last_rx_time = millis();

	// Inter-byte timeout: discard partial state after 5ms silence
	if (bt_proto != BT_PROTO_IDLE && head == bt_rx_tail) {
		if (!bt_last_rx_time || (millis() - bt_last_rx_time) > 5) {
			bt_proto = BT_PROTO_IDLE;
			bt_text_pos = 0;
		}
	}

	while (bt_rx_tail != head) {
		uint8_t b = bt_rx_ring[bt_rx_tail];
		bt_rx_tail = (bt_rx_tail + 1) & (BT_RX_RING_SIZE - 1);
		bt_rx_bytes++;

		switch (bt_proto) {
		case BT_PROTO_IDLE:
			if (b == KMBOX_SYNC1) {
				bt_proto = BT_PROTO_KMBOX;
				bt_frame_cksum = b;
				bt_kb_state = BT_KB_SYNC2;
			} else if (b >= 0x20 && b < 0x7F) {
				bt_proto = BT_PROTO_TEXT;
				bt_text_pos = 0;
				bt_text_line[bt_text_pos++] = (char)b;
			}
			break;

		case BT_PROTO_KMBOX:
			switch (bt_kb_state) {
			case BT_KB_SYNC2:
				if (b == KMBOX_SYNC2) {
					bt_frame_cksum += b;
					bt_kb_state = BT_KB_CMD;
				} else if (b == KMBOX_SYNC1) {
					bt_frame_cksum = b;
				} else {
					bt_proto = BT_PROTO_IDLE;
				}
				break;
			case BT_KB_CMD:
				bt_frame_cmd = b;
				bt_frame_cksum += b;
				bt_kb_state = BT_KB_LEN;
				break;
			case BT_KB_LEN:
				bt_frame_len = b;
				bt_frame_cksum += b;
				if (bt_frame_len > KMBOX_MAX_PAYLOAD) {
					bt_frames_err++;
					bt_proto = BT_PROTO_IDLE;
				} else if (bt_frame_len == 0) {
					bt_kb_state = BT_KB_CHECKSUM;
				} else {
					bt_frame_pos = 0;
					bt_kb_state = BT_KB_PAYLOAD;
				}
				break;
			case BT_KB_PAYLOAD:
				bt_frame_buf[bt_frame_pos++] = b;
				bt_frame_cksum += b;
				if (bt_frame_pos >= bt_frame_len)
					bt_kb_state = BT_KB_CHECKSUM;
				break;
			case BT_KB_CHECKSUM:
				if ((bt_frame_cksum & 0xFF) == b) {
					bt_dispatch_kmbox();
					bt_frames_ok++;
					bt_detected_proto = 1;
				} else {
					bt_frames_err++;
				}
				bt_proto = BT_PROTO_IDLE;
				break;
			}
			break;

		case BT_PROTO_TEXT:
			if (b == '\n' || b == '\r') {
				if (bt_text_pos > 0) {
					bt_text_line[bt_text_pos] = '\0';
					bt_dispatch_text_line();
				}
				bt_text_pos = 0;
				bt_proto = BT_PROTO_IDLE;
			} else if (bt_text_pos < BT_TEXT_MAX_LINE - 1) {
				bt_text_line[bt_text_pos++] = (char)b;
			} else {
				bt_text_pos = 0;
				bt_proto = BT_PROTO_IDLE;
			}
			break;
		}
	}

	bt_tx_flush();
}

bool bt_connected(void)
{
	return (GPIO3_DR & BT_STA_BIT) != 0;
}

uint32_t bt_frame_count(void)  { return bt_frames_ok; }
uint32_t bt_error_count(void)  { return bt_frames_err; }
uint32_t bt_rx_byte_count(void) { return bt_rx_bytes; }
uint32_t bt_get_baud(void)     { return bt_active_baud; }
uint8_t  bt_protocol_mode(void) { return bt_detected_proto; }
