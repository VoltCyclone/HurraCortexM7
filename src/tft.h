// tft.h — TFT display driver for i.MX RT1062
// LPSPI4 + eDMA, double-buffered RGB565 framebuffer
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
	TFT_DRIVER_NONE    = 0,
	TFT_DRIVER_ST7735  = 1,
	TFT_DRIVER_ILI9341 = 3,
} tft_driver_t;

extern tft_driver_t tft_detected_driver;

#define TFT_MAX_WIDTH    240
#define TFT_MAX_HEIGHT   320
#define TFT_MAX_FB_SIZE  (TFT_MAX_WIDTH * TFT_MAX_HEIGHT)

extern uint16_t tft_w;
extern uint16_t tft_h;

#define TFT_WIDTH   tft_w
#define TFT_HEIGHT  tft_h
#define TFT_FB_SIZE ((uint32_t)tft_w * tft_h)
#define TFT_COLS    (tft_w / TFT_FONT_W)
#define TFT_ROWS    (tft_h / TFT_FONT_H)

#define TFT_CS_PIN_BIT    0   // GPIO_B0_00  → GPIO7 bit 0   (pin 10)
#define TFT_DC_PIN_BIT   11   // GPIO_B0_11  → GPIO7 bit 11  (pin 32)
#define TFT_RST_PIN_BIT  10   // GPIO_B0_10  → GPIO7 bit 10  (pin 31)
#define TFT_BL_PIN_BIT    8   // GPIO_EMC_08 → GPIO4 bit 8   (pin 5)
#define TFT_FONT_W   6
#define TFT_FONT_H   8

// RGB565 color constants
#define COL_BG       ((uint16_t)0x0000)  // black
#define COL_WHITE    ((uint16_t)0xFFFF)  // white
#define COL_GREEN    ((uint16_t)0x07E0)  // bright green
#define COL_YELLOW   ((uint16_t)0xFFE0)  // yellow
#define COL_RED      ((uint16_t)0xF800)  // red
#define COL_CYAN     ((uint16_t)0x07FF)  // cyan
#define COL_GRAY     ((uint16_t)0x7BEF)  // medium gray
#define COL_DARK     ((uint16_t)0x39E7)  // dark gray
#define COL_DIM      ((uint16_t)0x18C6)  // dim gray

extern uint8_t  tft_font[256 * TFT_FONT_H];
extern uint16_t *tft_input;

void tft_init(void);
void tft_fill(uint16_t color);
void tft_swap_buffers(void);
void tft_sync(void);
void tft_swap_sync(void);
void tft_sync_begin(void);
bool tft_sync_continue(void);
bool tft_sync_busy(void);

void tft_draw_pixel(int x, int y, uint16_t color);
void tft_draw_rect(int x0, int y0, int x1, int y1, uint16_t color);
void tft_draw_glyph(int x, int y, uint16_t color, char c);
void tft_draw_string(int x, int y, uint16_t color, const char *str);
void tft_draw_string_right(int x, int y, uint16_t color, const char *str);
void tft_draw_hline(int x0, int x1, int y, uint16_t color);
void tft_command(uint8_t cmd, const uint8_t *data, size_t len);
void tft_begin_sync(void);

void ili9341_init_sequence(void);
void st7735_init_sequence(void);
