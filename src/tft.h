// tft.h — TFT display driver for i.MX RT1062
// LPSPI4 + eDMA, double-buffered 8-bit indexed framebuffer
// Supports ST7735 (128x160) and ILI9341 (240x320) via runtime auto-detection

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---- Driver IDs ----
typedef enum {
	TFT_DRIVER_NONE    = 0,
	TFT_DRIVER_ST7735  = 1,
	TFT_DRIVER_ILI9341 = 3,
} tft_driver_t;

extern tft_driver_t tft_detected_driver;

// ---- Compile-time buffer capacity (sized for largest display: ILI9341) ----
#define TFT_MAX_WIDTH    240
#define TFT_MAX_HEIGHT   320
#define TFT_MAX_FB_SIZE  (TFT_MAX_WIDTH * TFT_MAX_HEIGHT)

// ---- Runtime display dimensions (set during tft_init after detection) ----
extern uint16_t tft_w;    // actual display width  (128 or 240)
extern uint16_t tft_h;    // actual display height (160 or 320)

// Convenience macros — expand to runtime variables, NOT constants.
// Do NOT use in static array sizes or initializers.
#define TFT_WIDTH   tft_w
#define TFT_HEIGHT  tft_h
#define TFT_FB_SIZE ((uint32_t)tft_w * tft_h)
#define TFT_COLS    (tft_w / TFT_FONT_W)
#define TFT_ROWS    (tft_h / TFT_FONT_H)

#define TFT_CS_PIN_BIT    0  // Pin 10 = GPIO_B0_00 = GPIO7 bit 0
#define TFT_DC_PIN_BIT   11  // Pin 9  = GPIO_B0_11 = GPIO7 bit 11
#define TFT_RST_PIN_BIT  10  // Pin 6  = GPIO_B0_10 = GPIO7 bit 10
#define TFT_BL_PIN_BIT   17
#define TFT_FONT_W   6
#define TFT_FONT_H   8
extern uint16_t tft_palette[256];
extern uint8_t  tft_font[256 * TFT_FONT_H];
extern uint8_t *tft_input;
void tft_init(void);
void tft_fill(uint8_t color);
void tft_swap_buffers(void);
void tft_sync(void);
void tft_swap_sync(void);

// Non-blocking DMA sync — call from main loop to overlap TFT refresh
// with USB work.  tft_sync_begin() starts DMA for all dirty rows and
// returns immediately; tft_sync_continue() advances the state machine
// (fast no-op when idle); tft_sync_busy() returns true while DMA is
// still running.
void tft_sync_begin(void);
bool tft_sync_continue(void);
bool tft_sync_busy(void);
void tft_draw_pixel(int x, int y, uint8_t color);
void tft_draw_rect(int x0, int y0, int x1, int y1, uint8_t color);
void tft_draw_glyph(int x, int y, uint8_t color, char c);
void tft_draw_string(int x, int y, uint8_t color, const char *str);
void tft_draw_string_right(int x, int y, uint8_t color, const char *str);
void tft_draw_hline(int x0, int x1, int y, uint8_t color);
void tft_command(uint8_t cmd, const uint8_t *data, size_t len);
void tft_begin_sync(void);

// Driver-specific init sequences (called by tft_init after detection)
void ili9341_init_sequence(void);
void st7735_init_sequence(void);

#define COL_BG       0x00  // black
#define COL_WHITE    0x0F  // white
#define COL_GREEN    0xF4  // bright green
#define COL_YELLOW   0xF1  // yellow-orange
#define COL_RED      0xFF  // red
#define COL_CYAN     0xF7  // cyan
#define COL_GRAY     0x0A  // medium gray
#define COL_DARK     0x06  // dark gray
#define COL_DIM      0x04  // dim gray
