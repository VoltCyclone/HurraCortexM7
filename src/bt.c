// bt.c — HC-05 SPP on LPUART7 (D28 TX / D29 RX), DMA ch 5/6

#include "bt.h"
#include "kmbox.h"
#include "makd.h"
#include "macku.h"
#include "imxrt.h"
#include <string.h>

extern uint32_t millis(void);
extern void delay(uint32_t msec);

#ifndef BT_BAUD
#define BT_BAUD 921600
#endif
#define BT_CLOCK 24000000
#define BT_AT_BAUD 38400  // HC-05 full AT mode uses fixed 38400

// D28 = GPIO_EMC_32 ALT2 = LPUART7_TX
// D29 = GPIO_EMC_31 ALT2 = LPUART7_RX
#define BT_UART_BAUD       LPUART7_BAUD
#define BT_UART_CTRL       LPUART7_CTRL
#define BT_UART_STAT       LPUART7_STAT
#define BT_UART_DATA       LPUART7_DATA
#define BT_UART_FIFO       LPUART7_FIFO
#define BT_UART_WATER      LPUART7_WATER

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

// D30 = GPIO_EMC_36 = GPIO3[22] (STA input)
#define BT_STA_BIT         (1u << 22)
// D31 = GPIO_EMC_37 = GPIO3[23] (EN output, AT mode)
#define BT_EN_BIT          (1u << 23)

// Actual operating baud after AT configuration (may differ from BT_BAUD if AT fails)
static uint32_t bt_active_baud;

#define BT_RX_RING_SIZE 256
static uint8_t bt_rx_ring[BT_RX_RING_SIZE]
	__attribute__((section(".dmabuffers"), aligned(BT_RX_RING_SIZE)));
static volatile uint16_t bt_rx_tail;

#define BT_TX_RING_SIZE 128
static uint8_t bt_tx_ring[BT_TX_RING_SIZE];
static uint8_t bt_tx_head;
static uint8_t bt_tx_tail_pos;

#define BT_TX_BUF_SIZE 64
static uint8_t bt_tx_buf[BT_TX_BUF_SIZE]
	__attribute__((section(".dmabuffers"), aligned(4)));

static makd_parser_t bt_parser;
static macku_parser_t bt_macku_parser;

static uint32_t bt_frames_ok;
static uint32_t bt_frames_err;
static uint32_t bt_rx_bytes;
static uint32_t bt_last_rx_time;

// Detected protocol: 0=none, 1=MAKD, 2=Macku
static uint8_t bt_detected_proto;

static void bt_tx_enqueue(uint8_t b)
{
	uint8_t next = (bt_tx_head + 1) & (BT_TX_RING_SIZE - 1);
	if (next == bt_tx_tail_pos) return;
	bt_tx_ring[bt_tx_head] = b;
	bt_tx_head = next;
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

static void bt_tx_frame(const uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
		bt_tx_enqueue(data[i]);
}

static void bt_uart_set_baud(uint32_t baud)
{
	BT_UART_CTRL = 0;
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

static void bt_at_send(const char *cmd)
{
	while (*cmd) {
		while (!(BT_UART_STAT & LPUART_STAT_TDRE)) {}
		BT_UART_DATA = (uint8_t)*cmd++;
	}
	while (!(BT_UART_STAT & LPUART_STAT_TC)) {}
}

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

static void bt_at_drain(void)
{
	uint32_t start = millis();
	while ((millis() - start) < 50) {
		if (BT_UART_STAT & LPUART_STAT_RDRF)
			(void)BT_UART_DATA;
	}
}

static bool bt_at_cmd(const char *cmd, uint32_t timeout_ms)
{
	bt_at_send(cmd);
	char resp[32];
	uint8_t len = bt_at_recv(resp, sizeof(resp), timeout_ms);
	return (len >= 2 && resp[0] == 'O' && resp[1] == 'K');
}

static uint32_t bt_at_configure(uint32_t target_baud)
{
	GPIO3_DR_SET = BT_EN_BIT; // EN HIGH → AT mode
	delay(600); // HC-05 needs ~500ms to boot into AT mode

	bt_uart_set_baud(BT_AT_BAUD);
	bt_at_drain();

	if (!bt_at_cmd("AT\r\n", 500)) {
		delay(200);
		bt_at_drain();
		if (!bt_at_cmd("AT\r\n", 500)) {
			GPIO3_DR_CLEAR = BT_EN_BIT;
			delay(200);
			bt_uart_set_baud(target_baud);
			return target_baud;
		}
	}

	{
		char cmd[40] = "AT+UART=";
		char *p = cmd + 8;
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

	GPIO3_DR_CLEAR = BT_EN_BIT; // EN LOW → data mode
	delay(500); // HC-05 reboots into data mode

	bt_uart_set_baud(target_baud);
	bt_at_drain();

	return target_baud;
}

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

	bt_active_baud = bt_at_configure(BT_BAUD);

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
	makd_parser_reset(&bt_parser);
	macku_parser_reset(&bt_macku_parser);
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
			makd_parser_reset(&bt_parser);
			macku_parser_reset(&bt_macku_parser);
		}
	}

	uint16_t head = ((uint32_t)BT_RX_DADDR - (uint32_t)bt_rx_ring) & (BT_RX_RING_SIZE - 1);
	if (head != bt_rx_tail)
		bt_last_rx_time = millis();

	// Inter-byte timeout: discard partial state after 5ms silence
	if (head == bt_rx_tail && (bt_parser.state != 0 || bt_macku_parser.state != 0 || bt_macku_parser.bin_state != 0)) {
		if (!bt_last_rx_time || (millis() - bt_last_rx_time) > 5) {
			makd_parser_reset(&bt_parser);
			macku_parser_reset(&bt_macku_parser);
		}
	}

	while (bt_rx_tail != head) {
		uint8_t b = bt_rx_ring[bt_rx_tail];
		bt_rx_tail = (bt_rx_tail + 1) & (BT_RX_RING_SIZE - 1);
		bt_rx_bytes++;

		if (bt_detected_proto == 0 || bt_detected_proto == 1) {
			if (makd_parser_feed(&bt_parser, b)) {
				makd_dispatch(bt_parser.cmd, bt_parser.buf, bt_parser.len,
				              bt_tx_frame);
				bt_tx_flush();
				bt_frames_ok++;
				bt_detected_proto = 1;
				continue;
			}
		}
		if (bt_detected_proto == 0 || bt_detected_proto >= 2) {
			uint8_t r = macku_parser_feed(&bt_macku_parser, b);
			if (r == 1) {
				macku_dispatch(bt_macku_parser.cmd, bt_macku_parser.cmd_len,
				               bt_macku_parser.arg, bt_macku_parser.arg_len,
				               bt_tx_frame);
				bt_tx_flush();
				bt_frames_ok++;
				bt_detected_proto = g_identity_mode ? 3 : 2;
			} else if (r == 2) {
				macku_dispatch_bin(bt_macku_parser.bin_buf, bt_macku_parser.bin_len,
				                   bt_tx_frame);
				bt_tx_flush();
				bt_frames_ok++;
				bt_detected_proto = g_identity_mode ? 3 : 2;
			}
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
