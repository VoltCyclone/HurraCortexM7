// Ferrum ASCII command injection over LPUART3 (Teensy MicroMod pins 16/17,
// exposed on the ATP carrier as UART_RX2/UART_TX2, M.2 pads 21/22).
// Moved off LPUART6 (pins 0/1) after suspected pad damage on D0/D1.

#include "kmbox.h"
#include "humanize.h"
#include "imxrt.h"
#include "usb_device.h"
#include "proto.h"
#include "actions.h"
#include "gpt_profile.h"
#include "synth_cadence.h"
#include <string.h>

extern uint32_t millis(void);

#define UART_CLOCK 24000000

// Pin 17 = GPIO_AD_B1_06 ALT2 = LPUART3_TX  (MicroMod UART_TX2, M.2 pad 22)
// Pin 16 = GPIO_AD_B1_07 ALT2 = LPUART3_RX  (MicroMod UART_RX2, M.2 pad 21)
#define UART_BAUD          CMD_BAUD

#define KM_UART_BAUD       LPUART3_BAUD
#define KM_UART_CTRL       LPUART3_CTRL
#define KM_UART_STAT       LPUART3_STAT
#define KM_UART_DATA       LPUART3_DATA
#define KM_UART_FIFO       LPUART3_FIFO
#define KM_UART_WATER      LPUART3_WATER

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
#define KM_RX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART3_RX
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
#define KM_TX_DMAMUX_SRC   DMAMUX_SOURCE_LPUART3_TX
#define KM_TX_CH           4

// D31 = LINK:  GPIO_EMC_37 = GPIO3[23] — toggles when UART data arriving
// D30 = STATE: GPIO_EMC_36 = GPIO3[22] — toggles on valid frame dispatch
// D24 = STATUS: GPIO_AD_B0_12 = GPIO1[12] — solid = UART OK, flickers on error
#define LINK_LED_BIT   (1u << 23)
#define STATE_LED_BIT  (1u << 22)
#define STATUS_LED_BIT (1u << 12)

static uint32_t link_last_rx_time;
// Separate from link_last_rx_time (which is cleared by the LED-timeout path
// in kmbox_poll_fast). Tracks any RX activity for the auto-baud-reset gate.
static uint32_t last_rx_activity_time;
#define BAUD_IDLE_RESET_MS 5000

// DMA RX major-loop ISR — fires every DMA_RX_RING_SIZE bytes. Just clears
// the channel-done flag; SEVONPEND propagates the NVIC-pending into a SEV
// so the main loop's WFE wakes promptly on a burst of UART data instead
// of waiting for the next PIT tick.
static void km_rx_dma_isr(void)
{
	DMA_CINT = KM_RX_CH;
	// Posted-write flush: without this the W1C may not have landed by
	// exception return and the ISR re-enters once (wasted cycles in the
	// latency-critical window between PIT ticks).
	__asm volatile("dsb" ::: "memory");
}

// LPUART IDLE-line ISR — fires after one idle character at the end of each
// CH343B USB-frame burst. Pairs with the DMA major-loop ISR to wake the
// main loop's WFE between bursts (DMA only fires every DMA_RX_RING_SIZE
// bytes, so without this the last partial burst would wait for the next
// PIT tick). Body just W1C's the IDLE flag; SEVONPEND handles the wake.
static void km_uart_idle_isr(void)
{
	KM_UART_STAT = LPUART_STAT_IDLE;
	__asm volatile("dsb" ::: "memory"); // flush W1C before exception return
}

// Ring sized to match CH343B's 1 KB USB IN buffer so a full burst lands in
// one transfer. At 2 Mbaud (~200 KB/s) this is ~5 ms of headroom vs the
// previous 256-byte ring's 1.3 ms.
#define DMA_RX_RING_SIZE 1024
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

static kmbox_inject_t inject;
static uint32_t frames_ok;
static uint32_t frames_err;
static uint32_t rx_bytes_total;

static uint32_t current_baud = CMD_BAUD;

static uint32_t uart_overrun_count;
static uint32_t uart_framing_count;
static uint32_t uart_noise_count;
static uint32_t rx_drv_overrun_count;

static uint32_t tx_bytes_total;
static uint32_t tx_stuck_count;
static uint32_t tx_overflow_count;
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
                               int8_t wheel);
static void baud_change_apply(uint32_t baud);

static void tx_enqueue(uint8_t b)
{
	uint16_t next = (tx_head + 1) & (TX_RING_SIZE - 1);
	if (next == tx_tail_pos) {
		tx_overflow_count++;
		return;
	}
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
// Compute LPUART BAUD register OSR/SBR/BOTHEDGE bits for the requested rate.
// Follows NXP fsl_lpuart.c: searches all oversampling divisors 4..32, uses
// round-to-nearest SBR, and rejects (returns 0) if no combo lands within 3%
// of the requested baud (the same tolerance the NXP driver enforces). The
// returned value is the bare OSR|SBR(|BOTHEDGE) bits — caller adds RDMAE/TDMAE.
static uint32_t compute_baud_reg(uint32_t baud)
{
	if (baud == 0) return 0;
	uint32_t best_osr_div = 0;
	uint32_t best_sbr = 0;
	uint32_t best_diff = 0xFFFFFFFFu;
	for (uint32_t osr_div = 4; osr_div <= 32; osr_div++) {
		// Round-to-nearest: (2*num/den + 1) / 2
		uint32_t sbr = (UART_CLOCK * 2u / (baud * osr_div) + 1u) / 2u;
		if (sbr == 0) sbr = 1;
		if (sbr > 0x1FFFu) sbr = 0x1FFFu; // SBR is 13 bits
		uint32_t calc = UART_CLOCK / (osr_div * sbr);
		uint32_t diff = (calc > baud) ? (calc - baud) : (baud - calc);
		// '<=' to match fsl_lpuart.c: on ties, prefer the higher OSR (more
		// samples per bit → better noise immunity, no BOTHEDGE needed).
		if (diff <= best_diff) {
			best_diff = diff;
			best_osr_div = osr_div;
			best_sbr = sbr;
		}
	}
	// 3% tolerance — matches NXP's kStatus_LPUART_BaudrateNotSupport gate.
	if (best_diff > (baud / 100u) * 3u) return 0;
	// BAUD[OSR] field stores divisor-1 (RM §49.4.4.4: value 0x3 → 4× OSR).
	uint32_t reg = LPUART_BAUD_OSR(best_osr_div - 1u) | LPUART_BAUD_SBR(best_sbr);
	// BOTHEDGE required when oversampling ratio is 4..7 (RM §49.4.4.4).
	if (best_osr_div >= 4 && best_osr_div <= 7) reg |= LPUART_BAUD_BOTHEDGE;
	return reg;
}

void kmbox_init(void)
{
	// Hardening: pin the LPUART root clock to the 24 MHz crystal oscillator
	// (UART_CLK_SEL=1) with no post-divider (UART_CLK_PODF=0) so compute_baud_reg's
	// UART_CLOCK=24e6 assumption is guaranteed rather than inherited from the
	// bootloader/core. RM §14 (CCM): CCM_CSCDR1[UART_CLK_SEL], [UART_CLK_PODF].
	CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL;
	CCM_CCGR0 |= CCM_CCGR0_LPUART3(CCM_CCGR_ON);
	// TX = pin 17 = GPIO_AD_B1_06 ALT2 = LPUART3_TX
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	IOMUXC_LPUART3_TX_SELECT_INPUT = 0; // DAISY=0 → GPIO_AD_B1_06_ALT2
	// RX = pin 16 = GPIO_AD_B1_07 ALT2 = LPUART3_RX (keep keeper/pull-up)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 2;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 =
		IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2) |
		IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
	IOMUXC_LPUART3_RX_SELECT_INPUT = 0; // DAISY=0 → GPIO_AD_B1_07_ALT2

	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_37 = 5; // D31 LINK — ALT5 = GPIO3[23]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_37 = IOMUXC_PAD_DSE(6);
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_36 = 5; // D30 STATE — ALT5 = GPIO3[22]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_36 = IOMUXC_PAD_DSE(6);
	GPIO3_GDIR |= LINK_LED_BIT | STATE_LED_BIT;
	GPIO3_DR_CLEAR = LINK_LED_BIT | STATE_LED_BIT;

	// D24 STATUS LED — solid when UART OK, flickers on error
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 = 5; // D24 — ALT5 = GPIO1[12]
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_12 = IOMUXC_PAD_DSE(6);
	GPIO1_GDIR |= STATUS_LED_BIT;
	GPIO1_DR_SET = STATUS_LED_BIT; // ON = UART configured OK

	uint32_t baud_reg = compute_baud_reg(UART_BAUD);
	if (baud_reg == 0) {
		// Fallback: 115200 with OSR=16 (field=15), SBR=13 → 115384 (-0.16%).
		baud_reg = LPUART_BAUD_OSR(15) | LPUART_BAUD_SBR(13);
	}
	KM_UART_BAUD = baud_reg;
	KM_UART_CTRL = 0;
	KM_UART_FIFO = LPUART_FIFO_RXFE | LPUART_FIFO_TXFE;
	KM_UART_FIFO |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_RXFLUSH;
	// RXWATER=0: DMA fires on first received byte (lowest command latency).
	// TXWATER=2: DMA refills when FIFO has 2 free slots (avoids underrun at
	// high baud; FIFO depth is 4).
	KM_UART_WATER = LPUART_WATER_RXWATER(0) | LPUART_WATER_TXWATER(2);

	KM_UART_CTRL = LPUART_CTRL_TE | LPUART_CTRL_RE;
	// Clear any pending IDLE (set by hardware when RE went on), then enable
	// the IDLE interrupt for end-of-burst wake. ISR is just W1C — SEVONPEND
	// propagates the NVIC pending into a SEV that wakes the WFE in main.
	KM_UART_STAT = LPUART_STAT_IDLE;
	KM_UART_CTRL |= LPUART_CTRL_ILIE;
	attachInterruptVector(IRQ_LPUART3, km_uart_idle_isr);
	NVIC_SET_PRIORITY(IRQ_LPUART3, 160); // same band as DMA RX
	NVIC_ENABLE_IRQ(IRQ_LPUART3);
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
	KM_RX_CSR = DMA_TCD_CSR_INTMAJOR;
	KM_RX_DMAMUX = KM_RX_DMAMUX_SRC | DMAMUX_CHCFG_ENBL;
	attachInterruptVector(IRQ_DMA_CH3, km_rx_dma_isr);
	NVIC_SET_PRIORITY(IRQ_DMA_CH3, 160); // below USB (144) and PIT (64)
	NVIC_ENABLE_IRQ(IRQ_DMA_CH3);
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
	// proto_reset() not called here — proto_init() below zeroes the same
	// fields (plus callback state).  Reset remains in error paths only.
	memset(&inject, 0, sizeof(inject));
	frames_ok = 0;
	frames_err = 0;
	tx_bytes_total = 0;
	tx_stuck_count = 0;
	tx_overflow_count = 0;
	uart_overrun_count = 0;
	uart_framing_count = 0;
	uart_noise_count = 0;
	rx_drv_overrun_count = 0;

	cached_mouse_ep = 0;
	cached_mouse_maxpkt = 0;
	cached_kb_ep = 0;
	memset(&mouse_layout, 0, sizeof(mouse_layout));
	mouse_layout.wheel_bit = 0xFFFF;
	cached_mouse_report_len = 0;

	link_last_rx_time = 0;
	last_rx_activity_time = 0;

	// Order matters: hurra_init() zeroes its TX pointer (ferrum_init() does
	// not), so the transport MUST be installed *after* proto_init() or the
	// Hurra build never transmits (TF_WriteImpl no-ops on a NULL s_tx).
	proto_init();
	proto_set_tx(uart_tx_frame);
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
		if (desc->ifaces[i].interrupt_in_ep == 0) continue;
		uint8_t ep = desc->ifaces[i].interrupt_in_ep & 0x0F;
		if (desc->ifaces[i].iface_protocol == 2 && !cached_mouse_ep) {
			cached_mouse_ep = ep;
			cached_mouse_maxpkt = desc->ifaces[i].interrupt_in_maxpkt;
			parse_mouse_layout(desc->ifaces[i].hid_report_desc,
			                   desc->ifaces[i].hid_report_desc_len);
		} else if (desc->ifaces[i].iface_protocol == 1 && !cached_kb_ep) {
			cached_kb_ep = ep;
		}
	}

}

bool kmbox_rx_pending(void)
{
	uint16_t head = ((uint32_t)KM_RX_DADDR - (uint32_t)dma_rx_ring)
	              & (DMA_RX_RING_SIZE - 1);
	if (head != rx_tail) return true;
	// Also gate heavy on a sticky LPUART error so it gets cleared. OR in
	// particular halts hardware reception until cleared — leaving it set
	// would lock UART forever (no bytes → rx_pending stays false → heavy
	// never runs → OR never cleared).
	return (KM_UART_STAT & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF)) != 0;
}

void kmbox_poll_fast(void)
{
	merged_this_cycle = false;

	// Call tx_flush when there's data to drain (ring non-empty), when
	// the previous DMA just finished (DONE bit set), or when a DMA is
	// in-flight — that last case keeps tx_stuck_count incrementing so
	// stuck-recovery still fires while the software ring is empty.
	if (tx_head != tx_tail_pos ||
	    (KM_TX_CSR & DMA_TCD_CSR_DONE) ||
	    KM_TX_CITER != KM_TX_BITER)
		tx_flush();

	// Auto-reset baud to the boot default (CMD_BAUD) after extended RX idle so a
	// host that closes and reopens the VCOM (without power-cycling) recovers at
	// the known default rate after any km.baud() bump. Targets CMD_BAUD, not a
	// hardcoded 115200, so a 4 Mbaud Hurra default isn't downgraded mid-session.
	if (__builtin_expect(current_baud != CMD_BAUD && pending_baud_rate == 0 &&
	                     last_rx_activity_time != 0 &&
	                     (millis() - last_rx_activity_time) > BAUD_IDLE_RESET_MS, 0)) {
		pending_baud_rate = CMD_BAUD;
	}

	// Deferred baud change: apply only when TX is fully idle. We can't use
	// CSR_DONE here — tx_flush clears it unconditionally via DMA_CDNE, so
	// after the first poll it stays 0 until another TX *completes*. Instead
	// check DMA_ERQ (auto-cleared on transfer completion with DREQ) plus
	// software ring empty.
	if (__builtin_expect(pending_baud_rate != 0, 0) &&
	    tx_head == tx_tail_pos &&
	    !(DMA_ERQ & (1u << KM_TX_CH))) {
		baud_change_apply(pending_baud_rate);
		pending_baud_rate = 0;
	}

	// Drive catch_xy deadline check even when no UART RX is arriving.
	proto_tick();

	// Step any in-flight motion program (automove/bezier). Emits the increment
	// toward the trajectory's position at this instant through the injection
	// path, so it composes with real-mouse passthrough and humanization.
	act_motion_tick();

	if (__builtin_expect(inject.click_release_at != 0, 0) && millis() >= inject.click_release_at) {
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

	// LINK-LED timeout lives in _fast (not _heavy) — _heavy stops being
	// called once the ring drains, so the LED would stick on otherwise.
	if (__builtin_expect(link_last_rx_time != 0, 0) &&
	    (millis() - link_last_rx_time) > 50) {
		GPIO3_DR_CLEAR = LINK_LED_BIT;
		link_last_rx_time = 0;
	}
}

// Called only when kmbox_rx_pending() reported bytes available. Reads
// UART STAT, drains the DMA ring, feeds the ferrum parser, updates LINK LED.
void kmbox_poll_heavy(void)
{
	uint32_t stat = KM_UART_STAT;
	if (__builtin_expect(stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF), 0)) {
		if (stat & LPUART_STAT_OR) uart_overrun_count++;
		if (stat & LPUART_STAT_FE) uart_framing_count++;
		if (stat & LPUART_STAT_NF) uart_noise_count++;
		KM_UART_STAT = stat & (LPUART_STAT_OR | LPUART_STAT_FE | LPUART_STAT_NF);
		if (stat & (LPUART_STAT_OR | LPUART_STAT_FE)) {
			proto_reset();
		}
		GPIO1_DR_TOGGLE = STATUS_LED_BIT;
	}

	uint16_t head = ((uint32_t)KM_RX_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	if (head != rx_tail) {
		GPIO3_DR_TOGGLE = LINK_LED_BIT;
		link_last_rx_time = millis();
		last_rx_activity_time = link_last_rx_time;
	}

	// Driver-overrun heuristic: if the HW write pointer has run >=3/4 of the
	// ring ahead of the SW read pointer, bytes were almost certainly lost (the
	// eDMA ring wraps silently). Count it, snap rp to wp to skip the stale
	// span, and reset the parser so the next valid frame realigns cleanly.
	{
		uint16_t gap = (uint16_t)((head - rx_tail) & (DMA_RX_RING_SIZE - 1));
		if (__builtin_expect(gap > (DMA_RX_RING_SIZE * 3u / 4u), 0)) {
			rx_drv_overrun_count++;
			rx_tail = head;
			proto_reset();
			GPIO1_DR_TOGGLE = STATUS_LED_BIT;
		}
	}

	if (rx_tail != head) {
		// Feed the whole burst in <=2 contiguous spans (batch TF_Accept).
		uint16_t count = (uint16_t)((head - rx_tail) & (DMA_RX_RING_SIZE - 1));
		if (head > rx_tail) {
			proto_feed(&dma_rx_ring[rx_tail], (uint16_t)(head - rx_tail));
		} else {
			proto_feed(&dma_rx_ring[rx_tail],
			           (uint16_t)(DMA_RX_RING_SIZE - rx_tail));
			if (head) proto_feed(&dma_rx_ring[0], head);
		}
		rx_bytes_total += count;
		frames_ok++;                       // counts RX bursts processed
		GPIO3_DR_TOGGLE = STATE_LED_BIT;
		rx_tail = head;
		// Immediate reply flush: any reply/telemetry the parser queued during
		// the feed leaves in one DMA TX on this same poll, instead of waiting
		// for the next poll_fast tick. Protocol-agnostic latency win.
		tx_flush();
	}
}

__attribute__((cold, noinline))
static void kmbox_merge_report_slow(uint8_t *report, uint8_t len,
                                    uint8_t rid, uint8_t doff);
__attribute__((cold, noinline))
static void kmbox_merge_keyboard(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void kmbox_phys_mouse(uint8_t *report, uint8_t len);
__attribute__((cold, noinline))
static void kmbox_phys_keyboard(uint8_t *report, uint8_t len);

// Output cadence tracking (GPT2 microseconds). last_merge_us = when a real
// mouse report last rode through (injection rides those). last_synth_us = last
// standalone synth frame. The cadence rule (see synth_cadence.h) keeps exactly
// one mouse report per *measured device poll interval*: injection rides merge
// reports while the mouse is active, and the synth path only fills in when the
// mouse has gone silent, at the same rate the merge path was running — not a
// fixed 1 kHz. The two paths never both emit in the same window.
// last_merge_us is WRITTEN by the main merge path (kmbox_merge_report) and READ
// by the PIT ISR (kmbox_emit_synth_isr's silence gate), so it must be volatile —
// otherwise LTO may cache a stale value in the ISR (the ISR never writes it) and
// the silence gate would fire early, synthing while the mouse is still active.
static volatile uint32_t last_merge_us;

// --- Part C: lock-free synth frame handoff (main builds, PIT ISR emits) ----
// Main loop builds the next synth report into the inactive buffer, then flips
// synth_pub_idx with a single aligned store (atomic publish). The ISR reads the
// published buffer and emits it — no FPU, no humanize filter, in interrupt
// context. All cross-context scalars are volatile: written in one context, read
// in the other, and the compiler must not cache or reorder them.
// synth_buf itself is intentionally NOT volatile: the producer fully writes it
// before the publish DSB, and the ISR's "memory"-clobber DMB (after the
// synth_armed check) forces the consumer to reload it — that barrier, not a
// volatile qualifier, is what orders the buffer reads. Keep the DMB if you ever
// touch this.
static uint8_t           synth_buf[2][16];
static volatile uint8_t  synth_buf_len[2];
static volatile uint8_t  synth_wheel_pending[2]; // 1 = published frame carries the wheel
static volatile uint8_t  synth_pub_idx;   // buffer the ISR should read
static volatile bool     synth_armed;     // a fresh frame is waiting to emit
// last_synth_us is WRITTEN by the ISR (kmbox_emit_synth_isr) and READ by the
// main builder, so it must be volatile (uint32 access is atomic on M7, but the
// compiler would otherwise cache the main-loop read across iterations).
static volatile uint32_t last_synth_us;

/* Pull this frame's injected delta from the pending accumulators and run it
 * through the humanization filter. The filter delivers in-frame and owns
 * conservation (sub-pixel residual + >127 cap-carry), so we just consume. */
static void kmbox_take_injection(int16_t *out_dx, int16_t *out_dy)
{
	int16_t dx = inject.mouse_dx;
	int16_t dy = inject.mouse_dy;
	inject.mouse_dx = 0;
	inject.mouse_dy = 0;
	humanize_filter(&dx, &dy);
	*out_dx = dx;
	*out_dy = dy;
}

__attribute__((section(".fastrun")))
void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len)
{
	if (iface_protocol == 2) {
		last_merge_us = gpt_profile_us();   // a real mouse report is riding through now
		if (__builtin_expect(cached_mouse_report_len == 0, 0))
			cached_mouse_report_len = len;

		// Feature B/C: emit physical-only telemetry (pre-merge, pre-mask) and
		// suppress masked physical inputs — before any injection is merged in.
		// Both off (the common case) → the predicate is false and we skip it.
		if (__builtin_expect((g_phys_mask || proto_phys_enabled()) &&
		                     mouse_layout.valid, 0))
			kmbox_phys_mouse(report, len);

		if (mouse_layout.valid && inject.mouse_dirty) {
			uint8_t doff = mouse_layout.data_off;
			uint8_t rid = doff ? report[0] : 0;

			if (__builtin_expect(mouse_layout.fast_path && rid == mouse_layout.report_id, 1)) {
				report[doff] |= inject.mouse_buttons;
				proto_notify_buttons(report[doff]);

				int16_t inj_dx, inj_dy;
				kmbox_take_injection(&inj_dx, &inj_dy);

				// Each axis adds humanized injection onto the mouse's own delta
				// and clamps to the report field as a hard safety bound only.
				// The filter's per-frame cap (127) means the field clamp rarely
				// fires; conservation is now owned by humanize_filter's internal
				// owed accumulator.
				int32_t done_w = 0;
				int32_t done_dx = 0, done_dy = 0;
				if (mouse_layout.x_is16) {
					int32_t rx = (int16_t)(report[mouse_layout.x_byte] |
					             ((uint16_t)report[mouse_layout.x_byte + 1] << 8));
					int32_t mx = rx + inj_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte]     = (uint8_t)(mx & 0xFF);
					report[mouse_layout.x_byte + 1] = (uint8_t)(mx >> 8);
					done_dx = mx - rx;
				} else {
					int32_t rx = (int8_t)report[mouse_layout.x_byte];
					int32_t mx = rx + inj_dx;
					if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
					if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
					report[mouse_layout.x_byte] = (uint8_t)(int8_t)mx;
					done_dx = mx - rx;
				}

				if (mouse_layout.y_is16) {
					int32_t ry = (int16_t)(report[mouse_layout.y_byte] |
					             ((uint16_t)report[mouse_layout.y_byte + 1] << 8));
					int32_t my = ry + inj_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte]     = (uint8_t)(my & 0xFF);
					report[mouse_layout.y_byte + 1] = (uint8_t)(my >> 8);
					done_dy = my - ry;
				} else {
					int32_t ry = (int8_t)report[mouse_layout.y_byte];
					int32_t my = ry + inj_dy;
					if (my >  mouse_layout.y_max) my =  mouse_layout.y_max;
					if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
					report[mouse_layout.y_byte] = (uint8_t)(int8_t)my;
					done_dy = my - ry;
				}

				if (mouse_layout.w_byte != 0xFF && inject.mouse_wheel != 0) {
					if (mouse_layout.w_is16) {
						int32_t rw = (int16_t)(report[mouse_layout.w_byte] |
						             ((uint16_t)report[mouse_layout.w_byte + 1] << 8));
						int32_t want = rw + inject.mouse_wheel;
						int32_t mw = want;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte]     = (uint8_t)(mw & 0xFF);
						report[mouse_layout.w_byte + 1] = (uint8_t)(mw >> 8);
						inject.mouse_wheel = (int8_t)(want - mw);
						done_w = mw - rw;
					} else {
						int32_t rw = (int8_t)report[mouse_layout.w_byte];
						int32_t want = rw + inject.mouse_wheel;
						int32_t mw = want;
						if (mw >  mouse_layout.w_max) mw =  mouse_layout.w_max;
						if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
						report[mouse_layout.w_byte] = (uint8_t)(int8_t)mw;
						inject.mouse_wheel = (int8_t)(want - mw);
						done_w = mw - rw;
					}
				}

				// For a wheel on a separate report ID (no field here) the scroll
				// is flushed later by kmbox_send_wheel_report, so report the full
				// pending value now to preserve its telemetry cadence.
				int8_t w_tlm = (mouse_layout.w_byte != 0xFF)
				             ? (int8_t)done_w : inject.mouse_wheel;
				proto_notify_axes((int16_t)done_dx, (int16_t)done_dy, w_tlm);
				// If the field clamp rejected part of the injected delta (e.g.
				// 8-bit field while the real mouse is also moving), return the
				// unfit injected portion so the filter redelivers it next frame.
				// Real-mouse motion keeps priority; nothing injected is dropped.
				humanize_return((int16_t)(inj_dx - done_dx),
				                (int16_t)(inj_dy - done_dy));
				inject.mouse_dirty = (inject.mouse_buttons != 0 ||
				                      inject.mouse_wheel != 0 ||
				                      humanize_pending());
			} else {
				kmbox_merge_report_slow(report, len, rid, doff);
			}
		}
		merged_this_cycle = true;
	} else if (iface_protocol == 1) {
		// Feature B/C (keyboard): telemetry + masking run even with nothing
		// injected (the user may be typing on a masked key). Gated so an idle
		// keyboard with no monitoring/mask pays only this branch test.
		if (__builtin_expect((proto_phys_enabled() || act_phys_kb_mask_active()) &&
		                     len >= 8, 0))
			kmbox_phys_keyboard(report, len);
		if (__builtin_expect(inject.kb_dirty, 0)) {
			kmbox_merge_keyboard(report, len);
			merged_this_cycle = true;
		}
	}
}

__attribute__((cold, noinline))
static void kmbox_merge_report_slow(uint8_t *report, uint8_t len,
                                    uint8_t rid, uint8_t doff)
{
	// Pull humanized injection once for this frame; conservation is owned by
	// the filter's internal owed accumulator.  Only the axes whose report ID
	// actually arrived are applied — if X and Y live on different report IDs
	// the caller re-enters with the other ID and the filter will emit again.
	int16_t inj_dx, inj_dy;
	kmbox_take_injection(&inj_dx, &inj_dy);
	int32_t done_dx = 0, done_dy = 0, done_w = 0;

	if (rid == mouse_layout.report_id) {
		report[doff] |= inject.mouse_buttons;
		proto_notify_buttons(report[doff]);

		int32_t rx = read_report_field(report, len, mouse_layout.x_bit,
		                               mouse_layout.x_size, doff);
		int32_t mx = rx + inj_dx;
		if (mx > mouse_layout.x_max) mx = mouse_layout.x_max;
		if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
		write_report_field(report, len, mouse_layout.x_bit,
		                   mouse_layout.x_size, doff, mx);
		done_dx = mx - rx;

		if (rid == mouse_layout.y_report_id) {
			int32_t ry = read_report_field(report, len, mouse_layout.y_bit,
			                               mouse_layout.y_size, doff);
			int32_t my = ry + inj_dy;
			if (my > mouse_layout.y_max) my = mouse_layout.y_max;
			if (my < -mouse_layout.y_max) my = -mouse_layout.y_max;
			write_report_field(report, len, mouse_layout.y_bit,
			                   mouse_layout.y_size, doff, my);
			done_dy = my - ry;
		}
	}

	if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
	    rid == mouse_layout.wheel_report_id) {
		int32_t rw = read_report_field(report, len, mouse_layout.wheel_bit,
		                               mouse_layout.wheel_size, doff);
		int32_t ww = rw + inject.mouse_wheel;
		int32_t mw = ww;
		if (mw > mouse_layout.w_max) mw = mouse_layout.w_max;
		if (mw < -mouse_layout.w_max) mw = -mouse_layout.w_max;
		write_report_field(report, len, mouse_layout.wheel_bit,
		                   mouse_layout.wheel_size, doff, mw);
		inject.mouse_wheel = (int8_t)(ww - mw);
		done_w = mw - rw;
	}

	proto_notify_axes((int16_t)done_dx, (int16_t)done_dy, (int8_t)done_w);
	// Return any injected motion not applied this frame — either field-clamped,
	// or (on split X/Y report-ID layouts) belonging to an axis whose report ID
	// didn't arrive this call. The filter redelivers it; nothing is dropped.
	humanize_return((int16_t)(inj_dx - done_dx), (int16_t)(inj_dy - done_dy));
	inject.mouse_dirty = (inject.mouse_buttons != 0 ||
	                      inject.mouse_wheel != 0 ||
	                      humanize_pending());
}

__attribute__((cold, noinline))
static void kmbox_merge_keyboard(uint8_t *report, uint8_t len)
{
	if (len < 8) return;
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
	proto_notify_keys(&report[2]);
}

// Feature B/C (mouse): runs before injection is merged. Reads the physical
// report fields, pushes them as TLM_PHYS_* telemetry (the user's TRUE input,
// observed before masking), then zeroes any masked physical contribution so it
// never reaches the downstream PC. Injected input is applied afterward by the
// normal merge, so an injected click on a masked button still passes.
// Only the fields whose report ID matches this report are touched.
__attribute__((cold, noinline))
static void kmbox_phys_mouse(uint8_t *report, uint8_t len)
{
	uint8_t doff = mouse_layout.data_off;
	uint8_t rid  = doff ? report[0] : 0;

	bool xy_here    = (rid == mouse_layout.report_id);
	bool wheel_here = (mouse_layout.wheel_bit != 0xFFFF &&
	                   rid == mouse_layout.wheel_report_id);

	uint8_t phys_btn = xy_here ? report[doff] : 0;
	int32_t phys_x = 0, phys_y = 0, phys_w = 0;
	if (xy_here) {
		phys_x = read_report_field(report, len, mouse_layout.x_bit,
		                           mouse_layout.x_size, doff);
		phys_y = read_report_field(report, len, mouse_layout.y_bit,
		                           mouse_layout.y_size, doff);
	}
	if (wheel_here)
		phys_w = read_report_field(report, len, mouse_layout.wheel_bit,
		                           mouse_layout.wheel_size, doff);

	// Telemetry: the true physical input, BEFORE masking (spec §5.3).
	if (proto_phys_enabled()) {
		if (xy_here)
			proto_notify_phys_buttons(phys_btn);
		if (xy_here || wheel_here)
			proto_notify_phys_axes((int16_t)phys_x, (int16_t)phys_y,
			                       (int8_t)phys_w);
	}

	// Masking: zero the masked physical contributions in the outgoing report.
	if (g_phys_mask) {
		if (xy_here) {
			uint8_t bmask = (uint8_t)(g_phys_mask & 0x1F); // ml,mr,mm,ms1,ms2
			if (bmask) report[doff] &= (uint8_t)~bmask;
			if (g_phys_mask & (1u << PHYS_MASK_MX))
				write_report_field(report, len, mouse_layout.x_bit,
				                   mouse_layout.x_size, doff, 0);
			if (g_phys_mask & (1u << PHYS_MASK_MY))
				write_report_field(report, len, mouse_layout.y_bit,
				                   mouse_layout.y_size, doff, 0);
		}
		if (wheel_here && (g_phys_mask & (1u << PHYS_MASK_WHEEL)))
			write_report_field(report, len, mouse_layout.wheel_bit,
			                   mouse_layout.wheel_size, doff, 0);
	}
}

// Feature B/C (keyboard): standard 8-byte boot report (modifier, reserved,
// 6 keycodes). Pushes the physical modifier+keys as TLM_PHYS_KB (pre-mask),
// then removes any masked keycodes from the outgoing report. Modifiers are not
// individually maskable in the KMBox API, so only keycodes are filtered.
__attribute__((cold, noinline))
static void kmbox_phys_keyboard(uint8_t *report, uint8_t len)
{
	(void)len;  // caller guarantees len >= 8
	if (proto_phys_enabled())
		proto_notify_phys_keys(report[0], &report[2]);

	if (act_phys_kb_mask_active()) {
		for (int j = 2; j < 8; j++) {
			if (report[j] && act_phys_key_masked(report[j]))
				report[j] = 0;
		}
	}
}

__attribute__((cold, noinline))
static void kmbox_send_wheel_report(void);
__attribute__((cold, noinline))
static void kmbox_send_keyboard_report(void);

// Build the next standalone synth report into the inactive buffer and publish
// it for the PIT ISR. Runs in the main loop (FPU + inject consumption here).
// Does NOT decide silence/cadence — that is the ISR's gate at emit time.
__attribute__((section(".fastrun")))
void kmbox_publish_synth(void)
{
	// A previously published frame is still waiting for the ISR to emit it.
	// Don't drain injection again or overwrite the pending buffer — the ISR owns
	// it until it emits (which disarms) or stays gated. This single guard closes
	// two races at once: (a) the ISR catching an already-true synth_armed across
	// a fresh index flip and emitting one frame twice; (b) re-draining inject
	// into a second buffer before the first was sent, stranding the first drain.
	// In steady state the ISR disarms each tick before this runs, so it never
	// blocks; it only holds when the ISR's own gate suppressed the emit.
	if (synth_armed) return;

	// Synth only fills in while the mouse is silent — when it's active the merge
	// path carries injection. Draining inject here during active use would steal
	// it from the merge path, and the ISR's silence gate would then suppress the
	// emit, losing that motion exactly when injection matters most. Gate publish
	// on the SAME silence rule the ISR uses so the two paths never compete for
	// inject. (synth_armed is guaranteed false past the guard above.)
	uint32_t now_us = gpt_profile_us();
	uint32_t measured_us = humanize_measured_interval_us();
	if (!synth_mouse_silent(now_us, last_merge_us, measured_us)) return;

	if (!(inject.mouse_dirty && cached_mouse_ep && mouse_layout.valid))
		return;
	uint8_t w = synth_pub_idx ^ 1u;          // inactive buffer
	uint8_t *synth = synth_buf[w];
	memset(synth, 0, 16);
	uint8_t doff = mouse_layout.data_off;
	if (doff) synth[0] = mouse_layout.report_id;
	synth[doff] = inject.mouse_buttons;

	int16_t inj_dx, inj_dy;
	kmbox_take_injection(&inj_dx, &inj_dy);  // humanize_filter runs here (FPU)
	int32_t dx = inj_dx, dy = inj_dy;
	if (dx >  mouse_layout.x_max) dx =  mouse_layout.x_max;
	if (dx < -mouse_layout.x_max) dx = -mouse_layout.x_max;
	if (dy >  mouse_layout.y_max) dy =  mouse_layout.y_max;
	if (dy < -mouse_layout.y_max) dy = -mouse_layout.y_max;
	write_report_field(synth, 16, mouse_layout.x_bit, mouse_layout.x_size, doff, dx);
	write_report_field(synth, 16, mouse_layout.y_bit, mouse_layout.y_size, doff, dy);

	// Snapshot the wheel into the buffer but DO NOT consume inject.mouse_wheel
	// here — it must only be cleared when the report is actually emitted, else a
	// frame that gets superseded (synth_armed re-published, or disarmed) would
	// silently drop the wheel delta. The ISR clears it on confirmed emit.
	synth_wheel_pending[w] = 0;
	if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
	    mouse_layout.wheel_report_id == mouse_layout.report_id) {
		int32_t wv = inject.mouse_wheel;
		if (wv >  mouse_layout.w_max) wv =  mouse_layout.w_max;
		if (wv < -mouse_layout.w_max) wv = -mouse_layout.w_max;
		write_report_field(synth, 16, mouse_layout.wheel_bit,
		                   mouse_layout.wheel_size, doff, wv);
		synth_wheel_pending[w] = 1;   // this published frame carries the wheel
	}
	uint8_t rlen = cached_mouse_report_len;
	if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
	synth_buf_len[w] = rlen;

	// dirty recompute mirrors the original synth block, but keep mouse_wheel in
	// the predicate since we have NOT consumed it yet — a pending wheel must keep
	// the path dirty so a later emit still flushes it.
	inject.mouse_dirty = (inject.mouse_buttons != 0 ||
	                      inject.mouse_wheel != 0 ||
	                      humanize_pending());

	synth_pub_idx = w;        // publish the index for this buffer
	__asm volatile("dsb" ::: "memory"); // commit buffer + index before arming
	synth_armed = true;       // arm last: ISR keys off this (see emit barrier)
}

// Emit the published synth frame from the PIT ISR if the mouse is silent and a
// frame is due. Returns true if it emitted. Pure consumer: no FPU and no
// humanize filter (the only inject write is a single-store wheel clear on
// confirmed emit). last_merge_us / measured interval are owned by the main loop.
__attribute__((section(".fastrun")))
bool kmbox_emit_synth_isr(uint32_t now_us)
{
	if (!synth_armed) return false;
	// Consumer-side barrier: pair with the publisher's DSB. Without it the M7's
	// out-of-order load unit may hoist the synth_pub_idx / synth_buf reads above
	// the synth_armed check and observe a stale index or half-built buffer.
	__asm volatile("dmb" ::: "memory");
	uint32_t measured_us = humanize_measured_interval_us();
	if (!synth_mouse_silent(now_us, last_merge_us, measured_us)) return false;
	if (!synth_due(now_us, last_synth_us, measured_us)) return false;
	uint8_t idx = synth_pub_idx;
	usb_device_send_report(cached_mouse_ep, synth_buf[idx], synth_buf_len[idx]);
	last_synth_us = now_us;
	// Consume the wheel ONLY now that the frame is actually on the wire (the
	// buffer for `idx` carried it). Deferring the clear to here is what prevents
	// a superseded/disarmed publish from dropping a wheel delta.
	if (synth_wheel_pending[idx]) inject.mouse_wheel = 0;
	synth_armed = false;
	return true;
}

__attribute__((section(".fastrun")))
void kmbox_send_pending(void)
{
	// Flush unconsumed wheel on a separate report ID even when merged
	if (__builtin_expect(merged_this_cycle && inject.mouse_wheel != 0 &&
	    cached_mouse_ep && mouse_layout.valid &&
	    mouse_layout.wheel_bit != 0xFFFF &&
	    mouse_layout.wheel_report_id != mouse_layout.report_id, 0)) {
		kmbox_send_wheel_report();
	}

	if (merged_this_cycle) return;
	// Only synthesize a standalone mouse report when the physical mouse has
	// gone silent — otherwise injection rides the next real report (merge),
	// so the two paths never both emit in the same poll window (a double-emit
	// would overwrite at the endpoint, since the host reads <=1 report per
	// bInterval). Silence and cadence both derive from the *measured* device
	// poll interval (synth_cadence.h), so on an 8 kHz mouse the synth path
	// fills in at 8 kHz too, matching the rate the merge path was running.
	uint32_t now_us = gpt_profile_us();
	uint32_t measured_us = humanize_measured_interval_us();
	bool mouse_silent = synth_mouse_silent(now_us, last_merge_us, measured_us);
	bool due = synth_due(now_us, last_synth_us, measured_us);
	if (inject.mouse_dirty && mouse_silent && due &&
	    cached_mouse_ep && mouse_layout.valid) {
		last_synth_us = now_us;
		uint8_t synth[16];
		memset(synth, 0, sizeof(synth));
		uint8_t doff = mouse_layout.data_off;
		if (doff) synth[0] = mouse_layout.report_id;
		synth[doff] = inject.mouse_buttons;
		int16_t inj_dx, inj_dy;
		kmbox_take_injection(&inj_dx, &inj_dy);
		int32_t dx = inj_dx;
		int32_t dy = inj_dy;
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
		inject.mouse_wheel = 0;
		inject.mouse_dirty = (inject.mouse_buttons != 0 ||
		                      humanize_pending());
	}
	if (__builtin_expect(inject.kb_dirty && cached_kb_ep, 0)) {
		kmbox_send_keyboard_report();
	}
}

__attribute__((cold, noinline))
static void kmbox_send_wheel_report(void)
{
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

__attribute__((cold, noinline))
static void kmbox_send_keyboard_report(void)
{
	uint8_t synth[8];
	synth[0] = inject.kb_modifier;
	synth[1] = 0;
	memcpy(&synth[2], inject.kb_keys, 6);
	usb_device_send_report(cached_kb_ep, synth, 8);
	static const uint8_t zeros[6] = {0};
	inject.kb_dirty = (inject.kb_modifier != 0 ||
	                    memcmp(inject.kb_keys, zeros, 6) != 0);
}

static void baud_change_apply(uint32_t baud)
{
	uint32_t baud_reg = compute_baud_reg(baud);
	if (baud_reg == 0) {
		// Rate unachievable within 3% — keep the current configuration so
		// the host can recover by issuing km.baud() with a supported rate.
		return;
	}

	KM_UART_CTRL &= ~(LPUART_CTRL_TE | LPUART_CTRL_RE);
	KM_UART_BAUD = baud_reg | LPUART_BAUD_RDMAE | LPUART_BAUD_TDMAE;
	// Clear pending IDLE before re-enabling RE so we don't take a spurious
	// IDLE-line IRQ from the disable/enable transition.
	KM_UART_STAT = LPUART_STAT_IDLE;
	KM_UART_CTRL |= LPUART_CTRL_TE | LPUART_CTRL_RE;

	current_baud = baud;
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
uint32_t kmbox_tx_overflow(void) { return tx_overflow_count; }
uint32_t kmbox_rx_drv_overrun(void) { return rx_drv_overrun_count; }

uint16_t kmbox_tx_room(void)
{
	// Bytes free in the TX ring. Telemetry emitters skip a frame when the ring
	// would overflow; input listeners never skip.
	uint16_t used = (uint16_t)((tx_head - tx_tail_pos) & (TX_RING_SIZE - 1));
	return (uint16_t)(TX_RING_SIZE - 1 - used);
}

__attribute__((section(".fastrun")))
static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel)
{
	inject.mouse_buttons = buttons;
	inject.mouse_wheel += wheel;
	inject.mouse_dx += dx;
	inject.mouse_dy += dy;
	inject.mouse_dirty = true;
}

void kmbox_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                        int8_t wheel)
{
	apply_mouse_result(dx, dy, buttons, wheel);
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
