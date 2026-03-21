#include "tft.h"

extern void delay(uint32_t msec);

void ili9341_init_sequence(void)
{
	// Software reset
	tft_command(0x01, 0, 0);
	delay(120);

	// Extended power init — must be sent in this exact order before normal
	// init. Omitting causes instability at >40 MHz SPI clock.
	uint8_t ext[] = { 0x03, 0x80, 0x02 };
	tft_command(0xEF, ext, sizeof ext);

	uint8_t pwrb[] = { 0x00, 0xC1, 0x30 };
	tft_command(0xCF, pwrb, sizeof pwrb);

	uint8_t pwrseq[] = { 0x64, 0x03, 0x12, 0x81 };
	tft_command(0xED, pwrseq, sizeof pwrseq);

	uint8_t dtca[] = { 0x85, 0x00, 0x78 };
	tft_command(0xE8, dtca, sizeof dtca);

	uint8_t pwra[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
	tft_command(0xCB, pwra, sizeof pwra);

	uint8_t prc[] = { 0x20 };
	tft_command(0xF7, prc, sizeof prc);

	uint8_t dtcb[] = { 0x00, 0x00 };
	tft_command(0xEA, dtcb, sizeof dtcb);

	// Power control — tuned for Adafruit 2.8" panel
	uint8_t pwr1[] = { 0x23 }; // VRH = 4.60V
	tft_command(0xC0, pwr1, sizeof pwr1);
	uint8_t pwr2[] = { 0x10 }; // SAP, BT step-up
	tft_command(0xC1, pwr2, sizeof pwr2);

	// VCOM — panel-specific for correct contrast
	uint8_t vcom1[] = { 0x3E, 0x28 };
	tft_command(0xC5, vcom1, sizeof vcom1);
	uint8_t vcom2[] = { 0x86 };
	tft_command(0xC7, vcom2, sizeof vcom2);

	// MADCTL: MX=1 BGR=1 (portrait default)
	uint8_t madctl = 0x48;
	tft_command(0x36, &madctl, 1);

	// 16-bit RGB565
	uint8_t colmod = 0x55;
	tft_command(0x3A, &colmod, 1);

	// Frame rate: DIVA=0 (fosc), RTNA=0x13 → ~76 Hz internal refresh
	uint8_t frmctr1[] = { 0x00, 0x13 };
	tft_command(0xB1, frmctr1, sizeof frmctr1);

	// Display function control
	uint8_t dfc[] = { 0x08, 0x82, 0x27 };
	tft_command(0xB6, dfc, sizeof dfc);

	// 3-gamma off
	uint8_t enable3g[] = { 0x00 };
	tft_command(0xF2, enable3g, sizeof enable3g);
	uint8_t gamset[] = { 0x01 };
	tft_command(0x26, gamset, sizeof gamset);

	// Positive gamma
	uint8_t pgamctrl[] = {
		0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
		0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00
	};
	tft_command(0xE0, pgamctrl, sizeof pgamctrl);

	// Negative gamma
	uint8_t ngamctrl[] = {
		0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
		0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F
	};
	tft_command(0xE1, ngamctrl, sizeof ngamctrl);

	// Sleep out — must come after all config, before pixel writes.
	// Display On (0x29) is deferred until after the first frame is
	// written so the panel doesn't briefly show undefined RAM.
	tft_command(0x11, 0, 0);
	delay(120);

	// Set initial window
	uint8_t caset[] = { 0, 0, (tft_w - 1) >> 8, (tft_w - 1) & 0xFF };
	tft_command(0x2A, caset, sizeof caset);
	uint8_t paset[] = { 0, 0, (tft_h - 1) >> 8, (tft_h - 1) & 0xFF };
	tft_command(0x2B, paset, sizeof paset);
}
