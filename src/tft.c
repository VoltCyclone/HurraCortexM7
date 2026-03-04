// tft.c — ST7735 TFT display driver for i.MX RT1062
// Bare-metal LPSPI4 + eDMA, double-buffered 8-bit indexed framebuffer
// Row-by-row palette lookup (8-bit→RGB565) + DMA transfer to SPI

#include "tft.h"
#include "imxrt.h"
#include <string.h>

extern void delay(uint32_t msec);

// ---- Double-buffered framebuffers in non-cacheable DMA region ----
static uint8_t fb[2][TFT_FB_SIZE]
	__attribute__((section(".dmabuffers"), aligned(32)));

// Per-row transfer buffers (ping-pong for DMA overlap)
// Each entry packs 2 RGB565 pixels for 32-bit SPI frames
static uint32_t txbuf[2][TFT_WIDTH / 2]
	__attribute__((section(".dmabuffers"), aligned(32)));

uint8_t *tft_input;
static uint8_t *tft_committed;

// Palette from RaspberryKMBox/bridge (ISC license)
uint16_t __attribute__((__aligned__(512))) tft_palette[256] = {
	0x0000, 0x1082, 0x2104, 0x31a6, 0x4228, 0x52aa, 0x632c, 0x73ae,
	0x8c51, 0x9cd3, 0xad55, 0xbdd7, 0xce59, 0xdefb, 0xef7d, 0xffff,
	0x20a2, 0x20e2, 0x2102, 0x1902, 0x1102, 0x1102, 0x1103, 0x1104,
	0x10c4, 0x10a4, 0x1084, 0x1884, 0x2084, 0x2083, 0x2083, 0x2082,
	0x2081, 0x20c1, 0x2101, 0x1101, 0x0901, 0x0902, 0x0903, 0x0904,
	0x08c4, 0x0884, 0x0844, 0x1044, 0x1844, 0x2043, 0x2042, 0x2041,
	0x2060, 0x20c0, 0x1900, 0x1100, 0x0900, 0x0101, 0x0102, 0x0104,
	0x00a4, 0x0044, 0x0004, 0x1004, 0x1804, 0x2003, 0x2002, 0x2000,
	0x41a6, 0x41e6, 0x4206, 0x3a06, 0x3206, 0x3206, 0x3207, 0x3208,
	0x31c8, 0x31a8, 0x3188, 0x3988, 0x3988, 0x4187, 0x4187, 0x4186,
	0x4164, 0x41c4, 0x3a04, 0x3204, 0x2204, 0x2205, 0x2206, 0x2208,
	0x21a8, 0x2148, 0x2108, 0x3108, 0x3908, 0x4107, 0x4105, 0x4104,
	0x4102, 0x41a2, 0x3a02, 0x2a02, 0x1a02, 0x1203, 0x1205, 0x1208,
	0x1168, 0x10e8, 0x1888, 0x2888, 0x3888, 0x4086, 0x4084, 0x4082,
	0x40c0, 0x4180, 0x3a00, 0x2200, 0x0a00, 0x0202, 0x0205, 0x0208,
	0x0148, 0x0088, 0x0808, 0x2008, 0x3808, 0x4006, 0x4003, 0x4000,
	0x834c, 0x83ac, 0x7c0c, 0x740c, 0x640c, 0x640d, 0x640e, 0x640f,
	0x63b0, 0x6350, 0x6310, 0x7310, 0x7b10, 0x830f, 0x830d, 0x830c,
	0x82a8, 0x8368, 0x7c08, 0x6408, 0x4c08, 0x440a, 0x440c, 0x440f,
	0x4350, 0x4290, 0x4a10, 0x6210, 0x7210, 0x820e, 0x820b, 0x8208,
	0x8204, 0x8324, 0x7404, 0x5404, 0x3404, 0x2407, 0x240b, 0x240f,
	0x22f0, 0x21d0, 0x2910, 0x5110, 0x7110, 0x810d, 0x8108, 0x8104,
	0x8160, 0x82e0, 0x7400, 0x4400, 0x1400, 0x0404, 0x0409, 0x040f,
	0x0290, 0x0110, 0x1010, 0x4010, 0x6810, 0x800c, 0x8006, 0x8000,
	0xfeb7, 0xff77, 0xf7f7, 0xdff7, 0xc7f7, 0xbff9, 0xbffc, 0xbfff,
	0xbf3f, 0xbe7f, 0xc5ff, 0xddff, 0xf5ff, 0xfdfd, 0xfdfa, 0xfdf7,
	0xfd70, 0xfef0, 0xeff0, 0xbff0, 0x97f0, 0x87f3, 0x87f9, 0x87ff,
	0x869f, 0x851f, 0x8c1f, 0xbc1f, 0xec1f, 0xfc1b, 0xfc16, 0xfc10,
	0xfc28, 0xfe48, 0xe7e8, 0xa7e8, 0x5fe8, 0x47ed, 0x47f6, 0x47ff,
	0x45df, 0x439f, 0x521f, 0x9a1f, 0xe21f, 0xfa1a, 0xfa11, 0xfa08,
	0xfae0, 0xfdc0, 0xe7e0, 0x87e0, 0x27e0, 0x07e7, 0x07f3, 0x07ff,
	0x051f, 0x023f, 0x181f, 0x781f, 0xd81f, 0xf818, 0xf80c, 0xf800,
};

// ---- GPIO helpers ----
// All TFT GPIO pins live on GPIO7 (fast GPIO for GPIO_B0_xx and GPIO_B1_xx)

static inline void gpio_cs_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_cs_high(void) { GPIO7_DR_SET   = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_dc_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_dc_high(void) { GPIO7_DR_SET   = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_rst_low(void) { GPIO7_DR_CLEAR = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_rst_high(void){ GPIO7_DR_SET   = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_bl_on(void)   { GPIO7_DR_SET   = (1u << TFT_BL_PIN_BIT); }

// ---- SPI helpers (blocking, used for init commands only) ----

// Write a single byte over LPSPI4 in 8-bit mode, blocking
static void spi_write8_blocking(uint8_t b)
{
	// Set 8-bit frame, RX masked, no continuous
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_TDR = b;
	while (!(LPSPI4_SR & LPSPI_SR_TCF))
		;
	LPSPI4_SR = LPSPI_SR_TCF; // clear flag
}

// Write data bytes over LPSPI4 in 8-bit mode
static void spi_write_blocking(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		spi_write8_blocking(data[i]);
}

// Wait for SPI module to be idle (no transfer in progress)
static inline void spi_wait_idle(void)
{
	while (LPSPI4_SR & LPSPI_SR_MBF)
		;
}

// ---- Command interface ----

void tft_command(uint8_t cmd, const uint8_t *data, size_t len)
{
	gpio_dc_low();   // command mode
	gpio_cs_low();
	spi_write8_blocking(cmd);
	spi_wait_idle();

	if (len > 0) {
		gpio_dc_high(); // data mode
		spi_write_blocking(data, len);
		spi_wait_idle();
	}

	gpio_cs_high();
}

// ---- Initialization ----

void tft_init(void)
{
	tft_input = fb[0];
	tft_committed = fb[1];
	memset(fb[0], 0, TFT_FB_SIZE);
	memset(fb[1], 0xFF, TFT_FB_SIZE); // differ from fb[0] to force first full sync

	// ---- Clock gating ----
	CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON); // already on, harmless

	// LPSPI clock source: PLL2 (528 MHz), divider = 3+1 = 4 → 132 MHz
	CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_CLK_SEL_MASK |
	             CCM_CBCMR_LPSPI_PODF_MASK))
	          | CCM_CBCMR_LPSPI_CLK_SEL(2)
	          | CCM_CBCMR_LPSPI_PODF(3);

	// ---- Pin mux ----
	// Pin 13 (GPIO_B0_03) = LPSPI4_SCK (ALT3)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SCK_SELECT_INPUT = 0; // GPIO_B0_03

	// Pin 11 (GPIO_B0_02) = LPSPI4_SDO (ALT3)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SDO_SELECT_INPUT = 0; // GPIO_B0_02

	// Pin 10 (GPIO_B0_00) = CS as GPIO (ALT5)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_CS_PIN_BIT);
	gpio_cs_high();

	// Pin 9 (GPIO_B0_11) = DC as GPIO (ALT5)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_DC_PIN_BIT);
	gpio_dc_high();

	// Pin 6 (GPIO_B0_10) = RST as GPIO (ALT5)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_RST_PIN_BIT);
	gpio_rst_high();

	// Pin 7 (GPIO_B1_01) = Backlight as GPIO (ALT5)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_01 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_BL_PIN_BIT);
	gpio_bl_on();

	// ---- LPSPI4 configuration ----
	LPSPI4_CR = LPSPI_CR_RST; // software reset
	LPSPI4_CR = 0;
	// Master mode, no stall on TX underrun
	LPSPI4_CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_NOSTALL;
	// SCK = lpspi_clk / (SCKDIV+2) = 132 MHz / (0+2) = 66 MHz
	LPSPI4_CCR = LPSPI_CCR_SCKDIV(0) | LPSPI_CCR_SCKPCS(1) | LPSPI_CCR_PCSSCK(1);
	// TX FIFO watermark: request DMA when ≥1 entry free
	LPSPI4_FCR = LPSPI_FCR_TXWATER(0);
	// Default TCR: 8-bit, RX masked (set per-transfer)
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	// Enable module
	LPSPI4_CR = LPSPI_CR_MEN;

	// ---- eDMA channel 1: txbuf[] → LPSPI4_TDR (32-bit packed pixel pairs) ----
	DMAMUX_CHCFG1 = 0; // disable before config
	DMA_TCD1_SADDR = txbuf[0];
	DMA_TCD1_SOFF = 4; // 32-bit increments
	DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_32BIT) |
	                DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_32BIT);
	DMA_TCD1_NBYTES_MLNO = 4; // 1 word (2 pixels) per minor loop
	DMA_TCD1_SLAST = 0; // don't adjust source after major
	DMA_TCD1_DADDR = (volatile void *)&LPSPI4_TDR;
	DMA_TCD1_DOFF = 0; // fixed destination
	DMA_TCD1_CITER_ELINKNO = TFT_WIDTH / 2;
	DMA_TCD1_BITER_ELINKNO = TFT_WIDTH / 2;
	DMA_TCD1_DLASTSGA = 0;
	DMA_TCD1_CSR = DMA_TCD_CSR_DREQ; // auto-disable request on major complete
	DMAMUX_CHCFG1 = DMAMUX_SOURCE_LPSPI4_TX | DMAMUX_CHCFG_ENBL;
	// Enable DMA request from LPSPI4 TX FIFO
	LPSPI4_DER = LPSPI_DER_TDDE;

	// ---- Hardware reset of ST7735 ----
	gpio_rst_low();
	delay(50);
	gpio_rst_high();
	delay(50);

	// Run controller init sequence (in st7735.c)
	tft_preflight();
}

// ---- Double buffering ----

void tft_swap_buffers(void)
{
	uint8_t *tmp = tft_committed;
	tft_committed = tft_input;
	tft_input = tmp;
}

// ---- Display sync: dirty-row partial update + DMA ----
// Compares committed (new) vs input (old) framebuffer to find changed rows.
// Only sends dirty row spans via RASET+RAMWR, skipping identical rows.
// Saves ~75% SPI bandwidth on a stats dashboard where most rows are static.

void tft_sync(void)
{
	// Compare old vs new framebuffers — build 160-bit dirty row bitmask
	// Each row is 128 bytes = 32 words; early-exit on first difference per row
	const uint32_t *np = (const uint32_t *)tft_committed;
	const uint32_t *op = (const uint32_t *)tft_input;
	uint32_t dirty[5] = {0, 0, 0, 0, 0};

	for (int y = 0; y < TFT_HEIGHT; y++) {
		for (int w = 0; w < 32; w++) {
			if (np[w] != op[w]) {
				dirty[y >> 5] |= (1u << (y & 31));
				break;
			}
		}
		np += 32;
		op += 32;
	}

	// Scan dirty bitmask for contiguous spans
	int y = 0;
	while (y < TFT_HEIGHT) {
		// Skip clean rows
		while (y < TFT_HEIGHT && !(dirty[y >> 5] & (1u << (y & 31))))
			y++;
		if (y >= TFT_HEIGHT) break;

		int ys = y;
		// Extend to end of contiguous dirty rows
		while (y < TFT_HEIGHT && (dirty[y >> 5] & (1u << (y & 31))))
			y++;
		int ye = y - 1;

		// Set row window for this dirty span
		uint8_t raset[] = { 0, (uint8_t)ys, 0, (uint8_t)ye };
		tft_command(0x2b, raset, 4);

		// RAMWR — send command byte, keep CS low for pixel data
		gpio_dc_low();
		gpio_cs_low();
		spi_write8_blocking(0x2c);
		spi_wait_idle();
		gpio_dc_high();

		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK | LPSPI_TCR_CONT;

		const uint8_t *inptr = tft_committed + ys * TFT_WIDTH;
		int span_rows = ye - ys + 1;

		for (int r = 0; r < span_rows; r++) {
			uint32_t *buf = txbuf[r & 1];

			// 4-wide palette lookup: pack 2 RGB565 pixels per 32-bit word
			for (int x = 0; x < TFT_WIDTH / 2; x += 2) {
				uint32_t idx4 = *(const uint32_t *)inptr;
				uint16_t p0 = tft_palette[idx4 & 0xFF];
				uint16_t p1 = tft_palette[(idx4 >> 8) & 0xFF];
				uint16_t p2 = tft_palette[(idx4 >> 16) & 0xFF];
				uint16_t p3 = tft_palette[idx4 >> 24];
				buf[x]     = ((uint32_t)p0 << 16) | p1;
				buf[x + 1] = ((uint32_t)p2 << 16) | p3;
				inptr += 4;
			}

			// Wait for previous row DMA (skip on first row of span)
			if (r > 0) {
				while (!(DMA_TCD1_CSR & DMA_TCD_CSR_DONE))
					;
			}

			DMA_CDNE = 1;
			__asm volatile("dsb" ::: "memory");
			DMA_TCD1_SADDR = buf;
			DMA_TCD1_CITER_ELINKNO = TFT_WIDTH / 2;
			DMA_TCD1_BITER_ELINKNO = TFT_WIDTH / 2;
			__asm volatile("dsb" ::: "memory");
			DMA_SERQ = 1;
		}

		// Wait for final row of this span
		while (!(DMA_TCD1_CSR & DMA_TCD_CSR_DONE))
			;

		// End continuous SPI mode and release CS
		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK;
		spi_wait_idle();
		gpio_cs_high();
	}
}

void tft_swap_sync(void)
{
	tft_swap_buffers();
	tft_sync();
}

// ---- Drawing primitives ----

void tft_fill(uint8_t color)
{
	memset(tft_input, color, TFT_FB_SIZE);
}

void tft_draw_pixel(int x, int y, uint8_t color)
{
	if ((unsigned)x >= TFT_WIDTH || (unsigned)y >= TFT_HEIGHT)
		return;
	tft_input[y * TFT_WIDTH + x] = color;
}

void tft_draw_rect(int x0, int y0, int x1, int y1, uint8_t color)
{
	// Clamp to display bounds
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= TFT_WIDTH)  x1 = TFT_WIDTH - 1;
	if (y1 >= TFT_HEIGHT) y1 = TFT_HEIGHT - 1;
	if (x0 > x1 || y0 > y1) return;

	for (int y = y0; y <= y1; y++)
		memset(&tft_input[y * TFT_WIDTH + x0], color, x1 - x0 + 1);
}

void tft_draw_hline(int x0, int x1, int y, uint8_t color)
{
	if ((unsigned)y >= TFT_HEIGHT) return;
	if (x0 < 0) x0 = 0;
	if (x1 >= TFT_WIDTH) x1 = TFT_WIDTH - 1;
	if (x0 > x1) return;
	memset(&tft_input[y * TFT_WIDTH + x0], color, x1 - x0 + 1);
}

void tft_draw_glyph(int x, int y, uint8_t color, char c)
{
	// Fast path: fully on-screen
	if (x >= 0 && x + TFT_FONT_W <= TFT_WIDTH &&
	    y >= 0 && y + TFT_FONT_H <= TFT_HEIGHT) {
		const uint8_t *glyph = tft_font + (unsigned char)c * TFT_FONT_H;
		uint8_t *row = &tft_input[y * TFT_WIDTH + x];
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
			row += TFT_WIDTH;
		}
		return;
	}

	// Slow path: clipped
	const uint8_t *glyph = tft_font + (unsigned char)c * TFT_FONT_H;
	for (int gy = 0; gy < TFT_FONT_H; gy++) {
		uint8_t bits = glyph[gy];
		if (!bits) continue;
		int py = y + gy;
		if ((unsigned)py >= TFT_HEIGHT) continue;
		for (int gx = 0; gx < TFT_FONT_W; gx++) {
			if ((bits >> (7 - gx)) & 1) {
				int px = x + gx;
				if ((unsigned)px < TFT_WIDTH)
					tft_input[py * TFT_WIDTH + px] = color;
			}
		}
	}
}

void tft_draw_string(int x, int y, uint8_t color, const char *str)
{
	for (; *str; str++, x += TFT_FONT_W)
		tft_draw_glyph(x, y, color, *str);
}

void tft_draw_string_right(int x, int y, uint8_t color, const char *str)
{
	int len = 0;
	const char *p = str;
	while (*p++) len++;
	tft_draw_string(x - len * TFT_FONT_W, y, color, str);
}

