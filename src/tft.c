#include "tft.h"
#include "imxrt.h"
#include <string.h>

extern void delay(uint32_t msec);

// Framebuffers in cached AXI RAM — frees ~150 KB of DTCM.
// D-cache gives near-TCM read speed; the buffers are only CPU-accessed
// (palette expansion reads them, DMA never touches them directly).
static uint8_t fb[2][TFT_MAX_FB_SIZE]
	__attribute__((section(".axicached"), aligned(32)));

static uint32_t txbuf[2][TFT_MAX_WIDTH / 2]
	__attribute__((section(".dmabuffers"), aligned(32)));

uint8_t *tft_input;
static uint8_t *tft_committed;

uint16_t tft_w = TFT_MAX_WIDTH;
uint16_t tft_h = TFT_MAX_HEIGHT;
tft_driver_t tft_detected_driver = TFT_DRIVER_NONE;

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

static void dma_ch1_isr(void); // forward decl — defined after tft_swap_buffers

static inline void gpio_cs_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_cs_high(void) { GPIO7_DR_SET   = (1u << TFT_CS_PIN_BIT); }
static inline void gpio_dc_low(void)  { GPIO7_DR_CLEAR = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_dc_high(void) { GPIO7_DR_SET   = (1u << TFT_DC_PIN_BIT); }
static inline void gpio_rst_low(void) { GPIO7_DR_CLEAR = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_rst_high(void){ GPIO7_DR_SET   = (1u << TFT_RST_PIN_BIT); }
static inline void gpio_bl_on(void)   { GPIO7_DR_SET   = (1u << TFT_BL_PIN_BIT); }

// ---- SPI helpers (matching original proven patterns) ----

static void spi_write8_blocking(uint8_t b)
{
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_TDR = b;
	while (!(LPSPI4_SR & LPSPI_SR_TCF))
		;
	LPSPI4_SR = LPSPI_SR_TCF;
}

static void spi_write_blocking(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		spi_write8_blocking(data[i]);
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
		spi_wait_idle();
	}

	gpio_cs_high();
}

void tft_begin_sync(void)
{
	tft_command(0x2C, 0, 0);
}

// ---- MISO pin and SPI read for display detection ----

static void miso_pin_enable(void)
{
	// Pin 12 = GPIO_B0_01, ALT3 = LPSPI4_SDI
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(6) |
	                                     IOMUXC_PAD_SPEED(2) |
	                                     IOMUXC_PAD_PKE | IOMUXC_PAD_PUE |
	                                     IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
	IOMUXC_LPSPI4_SDI_SELECT_INPUT = 0;
}

// Read `len` bytes from display after sending a command byte.
// Must be called at reduced SPI clock (~6 MHz for display reads).
static uint8_t spi_read_bytes(uint8_t cmd, uint8_t *buf, uint8_t len)
{
	// Disable NOSTALL for reads (TX must wait for RX consumption)
	LPSPI4_CR &= ~LPSPI_CR_MEN;
	uint32_t saved_cfgr1 = LPSPI4_CFGR1;
	LPSPI4_CFGR1 = LPSPI_CFGR1_MASTER; // no NOSTALL
	LPSPI4_CR = LPSPI_CR_MEN;

	// Flush FIFOs
	LPSPI4_CR |= LPSPI_CR_RRF | LPSPI_CR_RTF;

	// Send command byte — blocking, single frame
	gpio_cs_low();
	gpio_dc_low();
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_TDR = cmd;
	while (!(LPSPI4_SR & LPSPI_SR_TCF))
		;
	LPSPI4_SR = LPSPI_SR_TCF;
	spi_wait_idle();

	// Read phase: 1 dummy byte + len data bytes
	gpio_dc_high();
	uint8_t total = 1 + len;
	uint8_t got = 0;

	for (uint8_t i = 0; i < total; i++) {
		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7); // RX enabled, no CONT
		LPSPI4_TDR = 0x00; // clock out dummy
		uint32_t timeout = 10000;
		while (!(LPSPI4_SR & LPSPI_SR_RDF) && --timeout)
			;
		if (timeout == 0) break;
		uint8_t rx = (uint8_t)(LPSPI4_RDR & 0xFF);
		if (i > 0) // skip dummy byte
			buf[got++] = rx;
	}
	spi_wait_idle();
	gpio_cs_high();

	// Restore write-optimized config
	LPSPI4_CR &= ~LPSPI_CR_MEN;
	LPSPI4_CFGR1 = saved_cfgr1;
	LPSPI4_CR = LPSPI_CR_MEN;
	LPSPI4_CR |= LPSPI_CR_RRF; // flush residual RX

	return got;
}

static tft_driver_t tft_detect(void)
{
	// Slow SPI for reads: 120 MHz / (18+2) = 6 MHz
	LPSPI4_CR &= ~LPSPI_CR_MEN;
	uint32_t saved_ccr = LPSPI4_CCR;
	LPSPI4_CCR = LPSPI_CCR_SCKDIV(18) | LPSPI_CCR_SCKPCS(4) | LPSPI_CCR_PCSSCK(4);
	LPSPI4_CR = LPSPI_CR_MEN;

	uint8_t id[3] = {0};
	uint8_t n = spi_read_bytes(0x04, id, 3); // RDDID

	// Restore clock
	LPSPI4_CR &= ~LPSPI_CR_MEN;
	LPSPI4_CCR = saved_ccr;
	LPSPI4_CR = LPSPI_CR_MEN;

	if (n < 3) return TFT_DRIVER_NONE;

	// ILI9341: ID bytes are [0x00, 0x93, 0x41]
	if (id[1] == 0x93 && id[2] == 0x41)
		return TFT_DRIVER_ILI9341;

	// ST7735: ID1 often 0x7C or 0x89
	if (id[0] == 0x7C || id[0] == 0x89)
		return TFT_DRIVER_ST7735;

	// Any non-zero response — assume ST7735 (clones vary)
	if (id[0] != 0 || id[1] != 0 || id[2] != 0)
		return TFT_DRIVER_ST7735;

	return TFT_DRIVER_NONE;
}

// ---- Init ----

void tft_init(void)
{
	tft_input = fb[0];
	tft_committed = fb[1];
	memset(fb[0], 0, TFT_MAX_FB_SIZE);
	memset(fb[1], 0xFF, TFT_MAX_FB_SIZE);

	CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
	CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
	// LPSPI_CLK = PLL3_PFD0 / 6 = 720 / 6 = 120 MHz
	CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_CLK_SEL_MASK |
	             CCM_CBCMR_LPSPI_PODF_MASK))
	          | CCM_CBCMR_LPSPI_CLK_SEL(2)
	          | CCM_CBCMR_LPSPI_PODF(5);
	// SCK — fast slew required at 40 MHz
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SCK_SELECT_INPUT = 0; // GPIO_B0_03
	// MOSI — fast slew
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 3;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(3);
	IOMUXC_LPSPI4_SDO_SELECT_INPUT = 0; // GPIO_B0_02
	// MISO — for display ID reads
	miso_pin_enable();
	// CS — GPIO (Pin 10 = GPIO_B0_00 ALT5)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 5; // ALT5 = GPIO
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_CS_PIN_BIT);
	gpio_cs_high();
	// DC — GPIO
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_DC_PIN_BIT);
	gpio_dc_high();
	// RST — GPIO
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_RST_PIN_BIT);
	gpio_rst_high();
	// BL — GPIO
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 5;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_01 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);
	GPIO7_GDIR |= (1u << TFT_BL_PIN_BIT);
	gpio_bl_on();

	// LPSPI4 — start at ILI9341 write speed, will adjust after detection
	LPSPI4_CR = LPSPI_CR_RST;
	LPSPI4_CR = 0;
	LPSPI4_CFGR1 = LPSPI_CFGR1_MASTER;  // no NOSTALL — clock stalls on empty FIFO
	LPSPI4_CCR = LPSPI_CCR_SCKDIV(2);
	LPSPI4_FCR = LPSPI_FCR_TXWATER(0);
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK;
	LPSPI4_CR = LPSPI_CR_MEN;

	// eDMA channel 1: txbuf[] → LPSPI4_TDR (32-bit packed pixel pairs)
	// INTMAJOR generates a completion interrupt; the ISR chains rows.
	DMAMUX_CHCFG1 = 0;
	DMA_TCD1_SADDR = txbuf[0];
	DMA_TCD1_SOFF = 4;
	DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_32BIT) |
	                DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_32BIT);
	DMA_TCD1_NBYTES_MLNO = 4;
	DMA_TCD1_SLAST = 0;
	DMA_TCD1_DADDR = (volatile void *)&LPSPI4_TDR;
	DMA_TCD1_DOFF = 0;
	DMA_TCD1_CITER_ELINKNO = TFT_MAX_WIDTH / 2; // will update after detect
	DMA_TCD1_BITER_ELINKNO = TFT_MAX_WIDTH / 2;
	DMA_TCD1_DLASTSGA = 0;
	DMA_TCD1_CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
	DMAMUX_CHCFG1 = DMAMUX_SOURCE_LPSPI4_TX | DMAMUX_CHCFG_ENBL;
	LPSPI4_DER = LPSPI_DER_TDDE;

	// Install eDMA CH1 completion ISR for non-blocking row chaining
	attachInterruptVector(IRQ_DMA_CH1, dma_ch1_isr);
	NVIC_SET_PRIORITY(IRQ_DMA_CH1, 96); // below PIT0 (64)
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);

	// Hardware reset
	gpio_rst_low();
	delay(50);
	gpio_rst_high();
	delay(50);

	// Hardcoded ILI9341 — detection via SPI reads disturbs LPSPI state.
	tft_detected_driver = TFT_DRIVER_ILI9341;
	tft_w = 240;
	tft_h = 320;
	ili9341_init_sequence();
}

void tft_swap_buffers(void)
{
	uint8_t *tmp = tft_committed;
	tft_committed = tft_input;
	tft_input = tmp;
}

// ---- Non-blocking DMA state machine ----
// The eDMA CH1 completion ISR chains row-by-row SPI transfers within
// a dirty span.  The main loop drives span-to-span transitions via
// tft_sync_continue(), freeing the CPU for USB work between rows.

typedef enum {
	TFT_DMA_IDLE,
	TFT_DMA_SENDING,    // ISR driving row-by-row DMA within a span
	TFT_DMA_SPAN_DONE,  // ISR finished last row; main code cleans up SPI
} tft_dma_state_t;

#define MAX_SPANS 32
typedef struct { uint16_t ys, ye; } tft_span_t;

static volatile tft_dma_state_t tft_dma_state = TFT_DMA_IDLE;
static volatile uint16_t tft_dma_row;      // current row within span
static uint16_t          tft_dma_total;    // total rows in current span
static const uint8_t    *tft_dma_base;     // committed FB pointer for span start

static tft_span_t span_list[MAX_SPANS];
static uint8_t    span_count;
static uint8_t    span_idx;

// Palette-expand one framebuffer row into a DMA tx buffer.
// Reads 8-bit indexed pixels, writes 32-bit packed RGB565 pairs.
static inline void expand_row(const uint8_t *inptr, uint32_t *buf, uint16_t w)
{
	for (int x = 0; x < w / 2; x += 2) {
		uint32_t idx4 = *(const uint32_t *)inptr;
		uint16_t p0 = tft_palette[idx4 & 0xFF];
		uint16_t p1 = tft_palette[(idx4 >> 8) & 0xFF];
		uint16_t p2 = tft_palette[(idx4 >> 16) & 0xFF];
		uint16_t p3 = tft_palette[idx4 >> 24];
		buf[x]     = ((uint32_t)p0 << 16) | p1;
		buf[x + 1] = ((uint32_t)p2 << 16) | p3;
		inptr += 4;
	}
}

// eDMA channel 1 completion ISR — chains rows within a dirty span.
// Runs at priority 96, below PIT0 (64) to avoid perturbing smooth timing.
// Typical execution: ~1.5 µs (palette expand 240 px + DMA kick).
static void dma_ch1_isr(void)
{
	DMA_CINT = 1; // clear interrupt for channel 1

	uint16_t next_row = tft_dma_row + 1;
	if (next_row >= tft_dma_total) {
		// Terminate the CONT transfer from within the ISR so that
		// zero-stuffing doesn't pump garbage between now and the
		// main loop.  CONTC keeps us inside the continuous transfer
		// (no MBF toggle); CONT=0 makes this the final frame.
		LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK
		           | LPSPI_TCR_CONTC;            // CONT=0
		while ((LPSPI4_FSR & 0x1F) >= 16)        // wait for FIFO slot
			;
		LPSPI4_TDR = 0;                          // dummy — ends transfer
		tft_dma_state = TFT_DMA_SPAN_DONE;
		return;
	}
	tft_dma_row = next_row;

	const uint16_t w = tft_w;
	uint32_t *buf = txbuf[next_row & 1];
	expand_row(tft_dma_base + next_row * w, buf, w);

	DMA_CDNE = 1;
	__asm volatile("dsb" ::: "memory");
	DMA_TCD1_SADDR = buf;
	DMA_TCD1_CITER_ELINKNO = w / 2;
	DMA_TCD1_BITER_ELINKNO = w / 2;
	__asm volatile("dsb" ::: "memory");
	DMA_SERQ = 1;
}

// Start DMA for one dirty span: sends SPI address-window commands
// (blocking, ~5 µs), expands row 0, kicks first DMA transfer.
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

	// Switch to 32-bit DMA frames with CONT for streaming.
	// The ISR terminates CONT after the last row's DMA completes.
	LPSPI4_TCR = LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK | LPSPI_TCR_CONT;

	tft_dma_base = tft_committed + ys * w;
	tft_dma_total = ye - ys + 1;
	tft_dma_row = 0;

	// Expand row 0
	uint32_t *buf = txbuf[0];
	expand_row(tft_dma_base, buf, w);

	DMA_CDNE = 1;
	__asm volatile("dsb" ::: "memory");
	DMA_TCD1_SADDR = buf;
	DMA_TCD1_CITER_ELINKNO = w / 2;
	DMA_TCD1_BITER_ELINKNO = w / 2;
	__asm volatile("dsb" ::: "memory");
	tft_dma_state = TFT_DMA_SENDING;
	DMA_SERQ = 1;
}

#define MAX_DIRTY_WORDS ((TFT_MAX_HEIGHT + 31) / 32)

// Build the dirty span list by comparing committed vs input framebuffers.
static void build_span_list(void)
{
	const uint16_t w = tft_w;
	const uint16_t h = tft_h;
	const int row_words = w / 4;
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
		// Find first dirty row
		while (y < h) {
			uint32_t word = dirty[y >> 5] >> (y & 31);
			if (word) {
				y += __builtin_ctz(word);
				break;
			}
			y = (y | 31) + 1;
		}
		if (y >= h) break;

		int ys = y;
		// Find end of dirty run
		while (y < h) {
			uint32_t word = dirty[y >> 5] >> (y & 31);
			if (word == 0) break;
			uint32_t inv = ~word;
			if (inv) {
				y += __builtin_ctz(inv);
				break;
			}
			y = (y | 31) + 1;
		}
		if (y > h) y = h;

		span_list[span_count].ys = (uint16_t)ys;
		span_list[span_count].ye = (uint16_t)(y - 1);
		span_count++;
	}
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
		return true; // ISR still driving rows

	// Last row was sent without CONT (ISR dropped it), so the module
	// goes idle after the final word.  Just wait and deassert CS.
	spi_wait_idle();
	gpio_cs_high();

	span_idx++;
	if (span_idx >= span_count) {
		tft_dma_state = TFT_DMA_IDLE;
		return false; // all spans done
	}

	start_span(span_list[span_idx].ys, span_list[span_idx].ye);
	return true;
}

bool tft_sync_busy(void)
{
	return tft_dma_state != TFT_DMA_IDLE;
}

// Blocking sync — used by init/error paths.
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


void tft_fill(uint8_t color)
{
	memset(tft_input, color, (uint32_t)tft_w * tft_h);
}

void tft_draw_pixel(int x, int y, uint8_t color)
{
	if ((unsigned)x >= tft_w || (unsigned)y >= tft_h)
		return;
	tft_input[y * tft_w + x] = color;
}

void tft_draw_rect(int x0, int y0, int x1, int y1, uint8_t color)
{
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= tft_w)  x1 = tft_w - 1;
	if (y1 >= tft_h) y1 = tft_h - 1;
	if (x0 > x1 || y0 > y1) return;

	for (int y = y0; y <= y1; y++)
		memset(&tft_input[y * tft_w + x0], color, x1 - x0 + 1);
}

void tft_draw_hline(int x0, int x1, int y, uint8_t color)
{
	if ((unsigned)y >= tft_h) return;
	if (x0 < 0) x0 = 0;
	if (x1 >= tft_w) x1 = tft_w - 1;
	if (x0 > x1) return;
	memset(&tft_input[y * tft_w + x0], color, x1 - x0 + 1);
}

void tft_draw_glyph(int x, int y, uint8_t color, char c)
{
	if (x >= 0 && x + TFT_FONT_W <= tft_w &&
	    y >= 0 && y + TFT_FONT_H <= tft_h) {
		const uint8_t *glyph = tft_font + (unsigned char)c * TFT_FONT_H;
		uint8_t *row = &tft_input[y * tft_w + x];
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
