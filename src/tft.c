#include "tft.h"
#include "imxrt.h"
#include <string.h>

extern void delay(uint32_t msec);

// RGB565 framebuffers — 2 × 150 KB in cached AXI SRAM.
// D-cache gives near-TCM read speed; CPU reads these in pack_row,
// DMA never touches framebuffers directly (it reads from txbuf in DTCM).
static uint16_t fb[2][TFT_MAX_FB_SIZE]
	__attribute__((section(".axicached"), aligned(32)));

// Double-buffered DMA row buffers in DTCM — ISR alternates [0]/[1]
// so the next row can be packed while the previous is still in the
// LPSPI FIFO.  Each buffer holds one row of packed 32-bit words.
static uint32_t txbuf[2][TFT_MAX_WIDTH / 2]
	__attribute__((section(".dmabuffers"), aligned(32)));

uint16_t *tft_input;
static uint16_t *tft_committed;

uint16_t tft_w = TFT_MAX_WIDTH;
uint16_t tft_h = TFT_MAX_HEIGHT;
tft_driver_t tft_detected_driver = TFT_DRIVER_NONE;

static void dma_ch1_isr(void);

static inline void gpio_cs_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_cs_high(void) { GPIO7_DR_SET   = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_dc_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_dc_high(void) { GPIO7_DR_SET   = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_rst_low(void) { GPIO7_DR_CLEAR = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_rst_high(void){ GPIO7_DR_SET   = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_bl_on(void)   { GPIO4_DR_SET   = (1u << TFT_BL_PIN_BIT); }

// ---- SPI blocking helpers ----

static void spi_write8_blocking(uint8_t b)
{
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_TDR = b;
	while (!(LPSPI4_SR & LPSPI_SR_TCF))
		;
	LPSPI4_SR = LPSPI_SR_TCF;
}

// Pipeline bytes into the FIFO for faster multi-byte sends.
static void spi_write_blocking(const uint8_t *data, size_t len)
{
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	for (size_t i = 0; i < len; i++) {
		while (!(LPSPI4_SR & LPSPI_SR_TDF))
			;
		LPSPI4_TDR = data[i];
	}
	while (LPSPI4_SR & LPSPI_SR_MBF)
		;
	LPSPI4_SR = LPSPI_SR_TCF;
}

static inline void spi_wait_idle(void)
{
	while (LPSPI4_SR & LPSPI_SR_MBF)
		;
}

void tft_command(uint8_t cmd, const uint8_t *data, size_t len)
{
	gpio_dc_low();
	gpio_cs_low();
	spi_write8_blocking(cmd);
	spi_wait_idle();

	if (len > 0) {
		gpio_dc_high();
		spi_write_blocking(data, len);
	}

	gpio_cs_high();
}

void tft_begin_sync(void)
{
	tft_command(0x2C, 0, 0);
}

// ---- MISO / detection ----

static void miso_pin_enable(void)
{
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(6) |
	                                     IOMUXC_PAD_SPEED(2) |
	                                     IOMUXC_PAD_PKE | IOMUXC_PAD_PUE |
	                                     IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
	IOMUXC_LPSPI4_SDI_SELECT_INPUT = 0;
}

static uint8_t spi_read_bytes(uint8_t cmd, uint8_t *buf, uint8_t len)
{
	LPSPI4_CR &= ~LPSPI_CR_MEN;
	uint32_t saved_cfgr1 = LPSPI4_CFGR1;
	LPSPI4_CFGR1 = LPSPI_CFGR1_MASTER;
	LPSPI4_CR = LPSPI_CR_MEN;
	LPSPI4_CR |= LPSPI_CR_RRF | LPSPI_CR_RTF;

	gpio_cs_low();
	gpio_dc_low();
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_TDR = cmd;
	while (!(LPSPI4_SR & LPSPI_SR_TCF))
		;
	LPSPI4_SR = LPSPI_SR_TCF;
	spi_wait_idle();

	gpio_dc_high();
	uint8_t total = 1 + len;
	uint8_t got = 0;

	for (uint8_t i = 0; i < total; i++) {
		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7);
		LPSPI4_TDR = 0x00;
		uint32_t timeout = 10000;
		while (!(LPSPI4_SR & LPSPI_SR_RDF) && --timeout)
			;
		if (timeout == 0) break;
		uint8_t rx = (uint8_t)(LPSPI4_RDR & 0xFF);
		if (i > 0)
			buf[got++] = rx;
	}
	spi_wait_idle();
	gpio_cs_high();

	LPSPI4_CR &= ~LPSPI_CR_MEN;
	LPSPI4_CFGR1 = saved_cfgr1;
	LPSPI4_CR = LPSPI_CR_MEN;
	LPSPI4_CR |= LPSPI_CR_RRF;

	return got;
}

// ---- Init ----

void tft_init(void)
{
	tft_input     = fb[0];
	tft_committed = fb[1];
	memset(fb[0], 0x00, sizeof fb[0]);
	memset(fb[1], 0xFF, sizeof fb[1]);

	CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);

	// LPSPI_CLK = PLL3_PFD0 / 6 = 120 MHz
	CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_CLK_SEL_MASK |
	             CCM_CBCMR_LPSPI_PODF_MASK))
	          | CCM_CBCMR_LPSPI_CLK_SEL(2)
	          | CCM_CBCMR_LPSPI_PODF(5);

	// SCK
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SCK_SELECT_INPUT = 0;
	// MOSI
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SDO_SELECT_INPUT = 0;
	// MISO
	miso_pin_enable();
	// CS
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_CS_PIN_BIT);
	gpio_cs_high();
	// DC
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_DC_PIN_BIT);
	gpio_dc_high();
	// RST
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_RST_PIN_BIT);
	gpio_rst_high();
	// BL — pin 5 (GPIO_EMC_08 = GPIO4 bit 8)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 5; // ALT5 = GPIO4_IO08
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_08 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO4_GDIR |= (1u << TFT_BL_PIN_BIT);
	gpio_bl_on();

	// LPSPI4: MASTER, no NOSTALL.  SCKDIV(2) → 30 MHz.
	LPSPI4_CR  = LPSPI_CR_RST;
	LPSPI4_CR  = 0;
	LPSPI4_CFGR1 = LPSPI_CFGR1_MASTER;
	LPSPI4_CCR   = LPSPI_CCR_SCKDIV(2);
	LPSPI4_FCR   = LPSPI_FCR_TXWATER(0);
	LPSPI4_TCR   = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_CR    = LPSPI_CR_MEN;

	// eDMA CH1: txbuf → LPSPI4_TDR, 32-bit words, one row per major loop.
	DMAMUX_CHCFG1 = 0;
	DMA_TCD1_SADDR        = txbuf[0];
	DMA_TCD1_SOFF         = 4;
	DMA_TCD1_ATTR         = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_32BIT) |
	                        DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_32BIT);
	DMA_TCD1_NBYTES_MLNO  = 4;
	DMA_TCD1_SLAST        = 0;
	DMA_TCD1_DADDR        = (volatile void *)&LPSPI4_TDR;
	DMA_TCD1_DOFF         = 0;
	DMA_TCD1_CITER_ELINKNO = TFT_MAX_WIDTH / 2;
	DMA_TCD1_BITER_ELINKNO = TFT_MAX_WIDTH / 2;
	DMA_TCD1_DLASTSGA     = 0;
	DMA_TCD1_CSR          = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
	DMAMUX_CHCFG1         = DMAMUX_SOURCE_LPSPI4_TX | DMAMUX_CHCFG_ENBL;
	LPSPI4_DER            = LPSPI_DER_TDDE;

	attachInterruptVector(IRQ_DMA_CH1, dma_ch1_isr);
	NVIC_SET_PRIORITY(IRQ_DMA_CH1, 96);
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);

	gpio_rst_low();
	delay(50);
	gpio_rst_high();
	delay(50);

	tft_detected_driver = TFT_DRIVER_ILI9341;
	tft_w = 240;
	tft_h = 320;

	ili9341_init_sequence();
}

void tft_swap_buffers(void)
{
	uint16_t *tmp = tft_committed;
	tft_committed = tft_input;
	tft_input     = tmp;
}

// ---- Non-blocking DMA state machine ----

typedef enum {
	TFT_DMA_IDLE,
	TFT_DMA_SENDING,
	TFT_DMA_SPAN_DONE,
} tft_dma_state_t;

#define MAX_SPANS 32
typedef struct { uint16_t ys, ye; } tft_span_t;

static volatile tft_dma_state_t tft_dma_state = TFT_DMA_IDLE;
static volatile uint16_t tft_dma_row;
static uint16_t          tft_dma_total;
static const uint16_t   *tft_dma_base;     // committed FB pointer for span start

static tft_span_t span_list[MAX_SPANS];
static uint8_t    span_count;
static uint8_t    span_idx;

// Pack one row of RGB565 pixels into a DMA tx buffer.
// LPSPI sends 32 bits MSB-first, so first pixel goes in the high half-word.
static inline void pack_row(const uint16_t *src, uint32_t *buf, uint16_t w)
{
	const uint16_t half = w / 2;
	for (uint16_t i = 0; i < half; i++) {
		buf[i] = ((uint32_t)src[0] << 16) | src[1];
		src += 2;
	}
}

// eDMA channel 1 completion ISR — chains rows within a dirty span.
// Runs at priority 96, below PIT0 (64) to avoid perturbing smooth timing.
// Typical execution: ~0.4 µs (pack 240 px + DMA kick).
static void dma_ch1_isr(void)
{
	DMA_CINT = 1;

	uint16_t next_row = tft_dma_row + 1;
	if (next_row >= tft_dma_total) {
		// Terminate the CONT transfer.  CONTC keeps us inside the
		// continuous transfer; CONT=0 makes this the final frame.
		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK
		           | LPSPI_TCR_CONTC;
		while ((LPSPI4_FSR & 0x1F) >= 16)
			;
		LPSPI4_TDR = 0;
		tft_dma_state = TFT_DMA_SPAN_DONE;
		return;
	}
	tft_dma_row = next_row;

	const uint16_t w = tft_w;
	uint32_t *buf = txbuf[next_row & 1];
	pack_row(tft_dma_base + next_row * w, buf, w);

	DMA_CDNE = 1;
	__asm volatile("dsb" ::: "memory");
	DMA_TCD1_SADDR = buf;
	DMA_TCD1_CITER_ELINKNO = w / 2;
	DMA_TCD1_BITER_ELINKNO = w / 2;
	__asm volatile("dsb" ::: "memory");
	DMA_SERQ = 1;
}

#define MAX_DIRTY_WORDS ((TFT_MAX_HEIGHT + 31) / 32)

static void build_span_list(void)
{
	const uint16_t w = tft_w;
	const uint16_t h = tft_h;

	// Each row is w uint16_t = w/2 uint32_t words.
	const int row_words  = w / 2;
	const int dirty_words = (h + 31) / 32;

	const uint32_t *np = (const uint32_t *)tft_committed;
	const uint32_t *op = (const uint32_t *)tft_input;

	uint32_t dirty[MAX_DIRTY_WORDS];
	for (int i = 0; i < dirty_words; i++) dirty[i] = 0;

	for (int y = 0; y < h; y++) {
		for (int wd = 0; wd < row_words; wd++) {
			if (np[wd] != op[wd]) {
				dirty[y >> 5] |= (1u << (y & 31));
				break;
			}
		}
		np += row_words;
		op += row_words;
	}

	span_count = 0;
	int y = 0;
	while (y < h && span_count < MAX_SPANS) {
		while (y < h) {
			uint32_t word = dirty[y >> 5] >> (y & 31);
			if (word) { y += __builtin_ctz(word); break; }
			y = (y | 31) + 1;
		}
		if (y >= h) break;

		int ys = y;
		while (y < h) {
			uint32_t word = dirty[y >> 5] >> (y & 31);
			if (word == 0) break;
			uint32_t inv = ~word;
			if (inv) { y += __builtin_ctz(inv); break; }
			y = (y | 31) + 1;
		}
		if (y > h) y = h;

		span_list[span_count].ys = (uint16_t)ys;
		span_list[span_count].ye = (uint16_t)(y - 1);
		span_count++;
	}
}

static void start_span(uint16_t ys, uint16_t ye)
{
	const uint16_t w = tft_w;

	uint8_t caset[] = { 0, 0, (uint8_t)((w - 1) >> 8), (uint8_t)(w - 1) };
	tft_command(0x2a, caset, 4);
	uint8_t raset[] = {
		(uint8_t)(ys >> 8), (uint8_t)ys,
		(uint8_t)(ye >> 8), (uint8_t)ye
	};
	tft_command(0x2b, raset, 4);

	// RAMWR command — CS stays low for pixel data DMA
	gpio_dc_low();
	gpio_cs_low();
	spi_write8_blocking(0x2C);
	spi_wait_idle();
	gpio_dc_high();

	// 32-bit DMA frames with CONT for streaming.
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK | LPSPI_TCR_CONT;

	tft_dma_base = tft_committed + ys * w;
	tft_dma_total = ye - ys + 1;
	tft_dma_row = 0;

	// Pack first row
	uint32_t *buf = txbuf[0];
	pack_row(tft_dma_base, buf, w);

	DMA_CDNE = 1;
	__asm volatile("dsb" ::: "memory");
	DMA_TCD1_SADDR = buf;
	DMA_TCD1_CITER_ELINKNO = w / 2;
	DMA_TCD1_BITER_ELINKNO = w / 2;
	__asm volatile("dsb" ::: "memory");
	tft_dma_state = TFT_DMA_SENDING;
	DMA_SERQ = 1;
}

void tft_sync_begin(void)
{
	build_span_list();
	span_idx = 0;
	if (span_count > 0)
		start_span(span_list[0].ys, span_list[0].ye);
}

bool tft_sync_continue(void)
{
	if (tft_dma_state == TFT_DMA_IDLE)
		return false;
	if (tft_dma_state != TFT_DMA_SPAN_DONE)
		return true;

	spi_wait_idle();
	gpio_cs_high();

	span_idx++;
	if (span_idx >= span_count) {
		tft_dma_state = TFT_DMA_IDLE;
		return false;
	}

	start_span(span_list[span_idx].ys, span_list[span_idx].ye);
	return true;
}

bool tft_sync_busy(void)
{
	return tft_dma_state != TFT_DMA_IDLE;
}

void tft_sync(void)
{
	tft_sync_begin();
	while (tft_sync_continue())
		;
}

void tft_swap_sync(void)
{
	tft_swap_buffers();
	tft_sync();
}

// ---- Draw primitives ----

void tft_fill(uint16_t color)
{
	uint32_t w32 = ((uint32_t)color << 16) | color;
	uint32_t *p  = (uint32_t *)tft_input;
	uint32_t  n  = ((uint32_t)tft_w * tft_h) / 2;
	for (uint32_t i = 0; i < n; i++)
		p[i] = w32;
}

void tft_draw_pixel(int x, int y, uint16_t color)
{
	if ((unsigned)x >= tft_w || (unsigned)y >= tft_h) return;
	tft_input[y * tft_w + x] = color;
}

void tft_draw_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= tft_w) x1 = tft_w - 1;
	if (y1 >= tft_h) y1 = tft_h - 1;
	if (x0 > x1 || y0 > y1) return;

	int width = x1 - x0 + 1;
	if (width >= 2) {
		uint32_t w32 = ((uint32_t)color << 16) | color;
		for (int y = y0; y <= y1; y++) {
			uint16_t *row = tft_input + y * tft_w + x0;
			int i = 0;
			if ((uintptr_t)row & 2) { *row++ = color; i = 1; }
			uint32_t *wp = (uint32_t *)row;
			int words = (width - i) / 2;
			for (int j = 0; j < words; j++) wp[j] = w32;
			if ((width - i) & 1) row[words * 2] = color;
		}
	} else {
		for (int y = y0; y <= y1; y++)
			tft_input[y * tft_w + x0] = color;
	}
}

void tft_draw_hline(int x0, int x1, int y, uint16_t color)
{
	if ((unsigned)y >= tft_h) return;
	if (x0 < 0) x0 = 0;
	if (x1 >= tft_w) x1 = tft_w - 1;
	if (x0 > x1) return;

	uint16_t *p = tft_input + y * tft_w + x0;
	int n = x1 - x0 + 1;
	uint32_t w32 = ((uint32_t)color << 16) | color;
	int i = 0;
	if ((uintptr_t)p & 2) { *p++ = color; i = 1; }
	uint32_t *wp = (uint32_t *)p;
	int words = (n - i) / 2;
	for (int j = 0; j < words; j++) wp[j] = w32;
	if ((n - i) & 1) p[words * 2] = color;
}

void tft_draw_glyph(int x, int y, uint16_t color, char c)
{
	// Fast path: fully on-screen
	if (x >= 0 && x + TFT_FONT_W <= tft_w &&
	    y >= 0 && y + TFT_FONT_H <= tft_h) {
		const uint8_t *glyph = tft_font + (unsigned char)c * TFT_FONT_H;
		uint16_t *row = tft_input + y * tft_w + x;
		for (int gy = 0; gy < TFT_FONT_H; gy++) {
			uint8_t bits = glyph[gy];
			if (bits) {
				if (bits & 0x80) row[0] = color;
				if (bits & 0x40) row[1] = color;
				if (bits & 0x20) row[2] = color;
				if (bits & 0x10) row[3] = color;
				if (bits & 0x08) row[4] = color;
				if (bits & 0x04) row[5] = color;
			}
			row += tft_w;
		}
		return;
	}

	// Slow path: clip each pixel
	const uint8_t *glyph = tft_font + (unsigned char)c * TFT_FONT_H;
	for (int gy = 0; gy < TFT_FONT_H; gy++) {
		uint8_t bits = glyph[gy];
		if (!bits) continue;
		int py = y + gy;
		if ((unsigned)py >= tft_h) continue;
		for (int gx = 0; gx < TFT_FONT_W; gx++) {
			if ((bits >> (7 - gx)) & 1) {
				int px = x + gx;
				if ((unsigned)px < tft_w)
					tft_input[py * tft_w + px] = color;
			}
		}
	}
}

void tft_draw_string(int x, int y, uint16_t color, const char *str)
{
	for (; *str; str++, x += TFT_FONT_W)
		tft_draw_glyph(x, y, color, *str);
}

void tft_draw_string_right(int x, int y, uint16_t color, const char *str)
{
	int len = 0;
	const char *p = str;
	while (*p++) len++;
	tft_draw_string(x - len * TFT_FONT_W, y, color, str);
}
