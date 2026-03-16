// st7735.c — ST7735 TFT controller init sequence for i.MX RT1062
// 128x160 RGB565 display over SPI

#include "tft.h"

extern void delay(uint32_t msec);

void st7735_init_sequence(void)
{
	// SWRESET
	tft_command(0x01, 0, 0);
	delay(150);

	// SLPOUT
	tft_command(0x11, 0, 0);
	delay(150);

	// FRMCTR1: frame rate control (normal mode)
	uint8_t frmctr1[] = { 0x01, 0x2C, 0x2D };
	tft_command(0xB1, frmctr1, 3);

	// FRMCTR2: frame rate control (idle mode)
	uint8_t frmctr2[] = { 0x01, 0x2C, 0x2D };
	tft_command(0xB2, frmctr2, 3);

	// FRMCTR3: frame rate control (partial mode)
	uint8_t frmctr3[] = { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D };
	tft_command(0xB3, frmctr3, 6);

	// INVCTR: display inversion control
	uint8_t invctr = 0x07;
	tft_command(0xB4, &invctr, 1);

	// PWCTR1: power control
	uint8_t pwctr1[] = { 0xA2, 0x02, 0x84 };
	tft_command(0xC0, pwctr1, 3);

	// PWCTR2: power control
	uint8_t pwctr2 = 0xC5;
	tft_command(0xC1, &pwctr2, 1);

	// PWCTR3: power control (normal mode)
	uint8_t pwctr3[] = { 0x0A, 0x00 };
	tft_command(0xC2, pwctr3, 2);

	// PWCTR4: power control (idle mode)
	uint8_t pwctr4[] = { 0x8A, 0x2A };
	tft_command(0xC3, pwctr4, 2);

	// PWCTR5: power control (partial mode)
	uint8_t pwctr5[] = { 0x8A, 0xEE };
	tft_command(0xC4, pwctr5, 2);

	// VMCTR1: VCOM control
	uint8_t vmctr1 = 0x0E;
	tft_command(0xC5, &vmctr1, 1);

	// INVOFF: display inversion off
	tft_command(0x20, 0, 0);

	// MADCTL: no rotation
	uint8_t madctl = 0x00;
	tft_command(0x36, &madctl, 1);

	// COLMOD: 16-bit RGB565
	uint8_t colmod = 0x05;
	tft_command(0x3A, &colmod, 1);

	// CASET: 0 to width-1
	uint8_t caset[] = { 0, 0, 0, (uint8_t)(tft_w - 1) };
	tft_command(0x2A, caset, 4);

	// RASET: 0 to height-1
	uint8_t raset[] = { 0, 0, 0, (uint8_t)(tft_h - 1) };
	tft_command(0x2B, raset, 4);

	// GMCTRP1: positive gamma correction
	uint8_t gmctrp1[] = {
		0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
		0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10
	};
	tft_command(0xE0, gmctrp1, 16);

	// GMCTRN1: negative gamma correction
	uint8_t gmctrn1[] = {
		0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
		0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10
	};
	tft_command(0xE1, gmctrn1, 16);

	// NORON: normal display mode on
	tft_command(0x13, 0, 0);
	delay(10);

	// DISPON
	tft_command(0x29, 0, 0);
	delay(100);
}
