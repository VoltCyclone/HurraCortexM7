#include "tft.h"

#if TFT_DRIVER == TFT_DRIVER_ILI9341

extern void delay(uint32_t msec);

void tft_preflight(void)
{
	tft_command(0x01, 0, 0);
	delay(120);
	tft_command(0x11, 0, 0);
	delay(120);
	uint8_t pwra[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
	tft_command(0xCB, pwra, sizeof pwra);
	uint8_t pwrb[] = { 0x00, 0x83, 0x30 };
	tft_command(0xCF, pwrb, sizeof pwrb);
	uint8_t dtca[] = { 0x85, 0x01, 0x79 };
	tft_command(0xE8, dtca, sizeof dtca);
	uint8_t dtcb[] = { 0x00, 0x00 };
	tft_command(0xEA, dtcb, sizeof dtcb);
	uint8_t pwrseq[] = { 0x64, 0x03, 0x12, 0x81 };
	tft_command(0xED, pwrseq, sizeof pwrseq);
	uint8_t prc[] = { 0x20 };
	tft_command(0xF7, prc, sizeof prc);
	uint8_t pwr1[] = { 0x26 };
	tft_command(0xC0, pwr1, sizeof pwr1);
	uint8_t pwr2[] = { 0x11 };
	tft_command(0xC1, pwr2, sizeof pwr2);
	uint8_t vcom1[] = { 0x35, 0x3E };
	tft_command(0xC5, vcom1, sizeof vcom1);
	uint8_t vcom2[] = { 0xBE };
	tft_command(0xC7, vcom2, sizeof vcom2);
	uint8_t madctl = (1 << 6) /* MX */ | (1 << 3) /* BGR */;
	tft_command(0x36, &madctl, 1);
	uint8_t colmod = 0x55;
	tft_command(0x3a, &colmod, 1);
	uint8_t dfc[] = { 0x0A, 0x82, 0x27, 0x00 };
	tft_command(0xB6, dfc, sizeof dfc);
	uint8_t enable3g[] = { 0 };
	tft_command(0xF2, enable3g, sizeof enable3g);
	uint8_t gamset[] = { 1 };
	tft_command(0x26, gamset, sizeof gamset);
	uint8_t pgamctrl[] = {
		0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1,
		0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00
	};
	tft_command(0xE0, pgamctrl, sizeof pgamctrl);
	uint8_t ngamctrl[] = {
		0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1,
		0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f
	};
	tft_command(0xE1, ngamctrl, sizeof ngamctrl);
	tft_command(0x29, 0, 0);
	delay(120);
	uint8_t etmod[] = { 0x07 };
	tft_command(0xB7, etmod, sizeof etmod);
	uint8_t frmctr1[] = { 0, 24 };
	tft_command(0xB1, frmctr1, sizeof frmctr1);
	uint8_t caset[] = { 0, 0, (TFT_WIDTH - 1) >> 8, (TFT_WIDTH - 1) & 0xFF };
	tft_command(0x2A, caset, sizeof caset);
	uint8_t paset[] = { 0, 0, (TFT_HEIGHT - 1) >> 8, (TFT_HEIGHT - 1) & 0xFF };
	tft_command(0x2B, paset, sizeof paset);
}

void tft_begin_sync(void)
{
	tft_command(0x2C, 0, 0);
}

#endif // TFT_DRIVER == TFT_DRIVER_ILI9341
