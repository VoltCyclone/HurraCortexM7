// tft_display.c — Stats + settings rendering for USB proxy TFT display
// Supports ST7735 (21×20 grid) and ILI9341 (40×40 grid) via TFT_DRIVER
// Touch-driven settings menu on ILI9341 when TOUCH_ENABLED

#include "tft_display.h"
#include "tft.h"
#include <string.h>

// ---- Formatting helpers ----

static char *u32_to_str(char *buf, uint32_t v)
{
	char tmp[10];
	int i = 0;
	if (v == 0) { *buf++ = '0'; return buf; }
	while (v) { tmp[i++] = '0' + (v % 10); v /= 10; }
	while (i--) *buf++ = tmp[i];
	return buf;
}

static char *u32_to_str02(char *buf, uint32_t v)
{
	*buf++ = '0' + (v / 10) % 10;
	*buf++ = '0' + v % 10;
	return buf;
}

static int fmt_done(char *start, char *end)
{
	*end = '\0';
	return (int)(end - start);
}

static char *u16_to_hex4(char *buf, uint16_t v)
{
	static const char hex[] = "0123456789ABCDEF";
	*buf++ = hex[(v >> 12) & 0xF];
	*buf++ = hex[(v >> 8) & 0xF];
	*buf++ = hex[(v >> 4) & 0xF];
	*buf++ = hex[v & 0xF];
	return buf;
}

static char *i8_to_str(char *buf, int8_t v)
{
	if (v < 0) { *buf++ = '-'; v = -v; }
	if (v >= 100) { *buf++ = '0' + v / 100; v %= 100; }
	if (v >= 10 || buf[-1] != '-') *buf++ = '0' + v / 10;
	*buf++ = '0' + v % 10;
	return buf;
}

// ---- Layout constants ----
#define FW   TFT_FONT_W
#define FH   TFT_FONT_H
#define LINE(n)  ((n) * FH)
#define COL(n)   ((n) * FW)

static void draw_separator(int y)
{
	tft_draw_hline(2, TFT_WIDTH - 3, y + FH / 2, COL_DIM);
}

// ---- View/settings state ----
static tft_view_t     g_view = TFT_VIEW_STATS;
static tft_settings_t g_settings = {
	.smooth_enabled   = true,
	.smooth_max       = 127,
	.humanize_enabled = true,
	.backlight        = true,
};
static uint8_t g_selected_setting = 0;

static void draw_stats(const tft_proxy_stats_t *s)
{
	char buf[24];
	char *p;

	tft_fill(COL_BG);

	int title_col = (TFT_COLS - 8) / 2;
	tft_draw_string(COL(title_col), LINE(0), COL_CYAN, "IMXRTNSY");

	draw_separator(LINE(1));

	tft_draw_string(COL(0), LINE(2), COL_GRAY, "Host:");
	tft_draw_string(COL(5), LINE(2),
		s->host_connected ? COL_GREEN : COL_RED,
		s->host_connected ? "OK" : "--");
	tft_draw_string(COL(9), LINE(2), COL_GRAY, "Dev:");
	tft_draw_string(COL(13), LINE(2),
		s->device_configured ? COL_GREEN : COL_RED,
		s->device_configured ? "OK" : "--");

	{
		const char *proto[] = { "None", "KMBox", "Makcu", "Ferrum" };
		uint8_t pm = s->protocol_mode;
		if (pm > 3) pm = 0;
		tft_draw_string(COL(0), LINE(3), COL_GRAY, "Proto:");
		tft_draw_string(COL(6), LINE(3),
			s->kmbox_active ? COL_GREEN : COL_DARK,
			proto[pm]);
	}

	{
		const char *spd[] = { "Full", "Low", "High" };
		uint8_t sp = s->device_speed;
		if (sp > 2) sp = 0;
		tft_draw_string(COL(0), LINE(4), COL_GRAY, "Spd:");
		tft_draw_string(COL(4), LINE(4), COL_WHITE, spd[sp]);
		tft_draw_string(COL(12), LINE(4), COL_GRAY, "EPs:");
		p = buf;
		p = u32_to_str(p, s->num_endpoints);
		fmt_done(buf, p);
		tft_draw_string(COL(16), LINE(4), COL_WHITE, buf);
	}

	draw_separator(LINE(5));

	tft_draw_string(COL(0), LINE(6), COL_GRAY, "Reports:");
	p = buf;
	p = u32_to_str(p, s->report_count);
	fmt_done(buf, p);
	tft_draw_string(COL(9), LINE(6), COL_WHITE, buf);

	tft_draw_string(COL(0), LINE(7), COL_GRAY, "Rate:");
	p = buf;
	p = u32_to_str(p, s->reports_per_sec);
	*p++ = '/'; *p++ = 's';
	fmt_done(buf, p);
	tft_draw_string(COL(5), LINE(7), COL_GREEN, buf);

	tft_draw_string(COL(0), LINE(8), COL_GRAY, "Drops:");
	p = buf;
	p = u32_to_str(p, s->drop_count);
	fmt_done(buf, p);
	tft_draw_string(COL(6), LINE(8),
		s->drop_count > 0 ? COL_RED : COL_DARK, buf);

	tft_draw_string(COL(0), LINE(9), COL_GRAY, "KMBox:");
	p = buf;
	p = u32_to_str(p, s->kmbox_frames_ok);
	*p++ = ' '; *p++ = 'o'; *p++ = 'k';
	fmt_done(buf, p);
	tft_draw_string(COL(6), LINE(9), COL_WHITE, buf);

	if (s->kmbox_frames_err > 0) {
		tft_draw_string(COL(14), LINE(9), COL_GRAY, "E:");
		p = buf;
		p = u32_to_str(p, s->kmbox_frames_err);
		fmt_done(buf, p);
		tft_draw_string(COL(16), LINE(9), COL_RED, buf);
	}

	draw_separator(LINE(10));

	// Line 11: Smooth injection
	tft_draw_string(COL(0), LINE(11), COL_GRAY, "Smooth:");
	if (s->smooth_active) {
		uint8_t filled = 0;
		if (s->smooth_queue_max > 0)
			filled = (s->smooth_queue_depth * 8) / s->smooth_queue_max;
		buf[0] = '[';
		for (int i = 0; i < 8; i++)
			buf[1 + i] = (i < filled) ? '=' : ' ';
		buf[9] = ']';
		p = buf + 10;
		p = u32_to_str(p, s->smooth_queue_depth);
		fmt_done(buf, p);
		tft_draw_string(COL(7), LINE(11), COL_YELLOW, buf);
	} else {
		tft_draw_string(COL(7), LINE(11), COL_DARK, "idle");
	}

	draw_separator(LINE(12));

#if NET_ENABLED
	// Lines 13-16: Network stats
	{
		// IP address: x.x.x.x
		uint32_t ip = s->net_ip;
		tft_draw_string(COL(0), LINE(13), COL_GRAY, "IP:");
		p = buf;
		p = u32_to_str(p, (ip >> 24) & 0xFF); *p++ = '.';
		p = u32_to_str(p, (ip >> 16) & 0xFF); *p++ = '.';
		p = u32_to_str(p, (ip >> 8) & 0xFF);  *p++ = '.';
		p = u32_to_str(p, ip & 0xFF);
		fmt_done(buf, p);
		tft_draw_string(COL(3), LINE(13), COL_WHITE, buf);
	}

	tft_draw_string(COL(0), LINE(14), COL_GRAY, "Port:");
	p = buf;
	p = u32_to_str(p, s->net_port);
	fmt_done(buf, p);
	tft_draw_string(COL(5), LINE(14), COL_WHITE, buf);

	tft_draw_string(COL(11), LINE(14), COL_GRAY, "Link:");
	tft_draw_string(COL(16), LINE(14),
		s->net_link_up ? COL_GREEN : COL_RED,
		s->net_link_up ? "UP" : "DN");

	{
		tft_draw_string(COL(0), LINE(15), COL_GRAY, "UUID:");
		p = buf;
		p = u16_to_hex4(p, (uint16_t)(s->net_uuid >> 16));
		p = u16_to_hex4(p, (uint16_t)(s->net_uuid & 0xFFFF));
		fmt_done(buf, p);
		tft_draw_string(COL(5), LINE(15), COL_CYAN, buf);

		tft_draw_string(COL(14), LINE(15),
			s->net_connected ? COL_GREEN : COL_DARK,
			s->net_connected ? "CONN" : "WAIT");
	}

	tft_draw_string(COL(0), LINE(16), COL_GRAY, "RX:");
	p = buf;
	p = u32_to_str(p, s->net_rx_count);
	fmt_done(buf, p);
	tft_draw_string(COL(3), LINE(16), COL_WHITE, buf);

	tft_draw_string(COL(11), LINE(16), COL_GRAY, "TX:");
	p = buf;
	p = u32_to_str(p, s->net_tx_count);
	fmt_done(buf, p);
	tft_draw_string(COL(14), LINE(16), COL_WHITE, buf);

	draw_separator(LINE(17));
#else
	// Line 13: UART RX activity
	tft_draw_string(COL(0), LINE(13), COL_GRAY, "UART RX:");
	if (s->uart_rx_bytes > 0) {
		p = buf;
		p = u32_to_str(p, s->uart_rx_bytes);
		*p++ = 'B';
		fmt_done(buf, p);
		tft_draw_string(COL(9), LINE(13), COL_GREEN, buf);
	} else {
		tft_draw_string(COL(9), LINE(13), COL_RED, "no data");
	}

	draw_separator(LINE(14));
#endif

	// Line offsets depend on whether NET stats took extra rows
#if NET_ENABLED
#define UPTIME_LINE  18
#else
#define UPTIME_LINE  15
#endif

	{
		uint32_t sec = s->uptime_sec;
		uint32_t hr  = sec / 3600;
		uint32_t min = (sec / 60) % 60;
		uint32_t s2  = sec % 60;
		tft_draw_string(COL(0), LINE(UPTIME_LINE), COL_GRAY, "Up:");
		p = buf;
		p = u32_to_str02(p, hr);
		*p++ = ':';
		p = u32_to_str02(p, min);
		*p++ = ':';
		p = u32_to_str02(p, s2);
		fmt_done(buf, p);
		tft_draw_string(COL(3), LINE(UPTIME_LINE), COL_WHITE, buf);

		tft_draw_string(COL(14), LINE(UPTIME_LINE), COL_GRAY, "CPU:");
		p = buf;
		p = i8_to_str(p, s->cpu_temp_c);
		*p++ = 'C';
		fmt_done(buf, p);
		uint8_t tcol = (s->cpu_temp_c > 80) ? COL_RED :
		               (s->cpu_temp_c > 60) ? COL_YELLOW : COL_GREEN;
		tft_draw_string(COL(18), LINE(UPTIME_LINE), tcol, buf);
	}

#if !NET_ENABLED
	draw_separator(LINE(16));

	{
		tft_draw_string(COL(0), LINE(17), COL_GRAY, "USB:");
		p = buf;
		p = u16_to_hex4(p, s->usb_vid);
		*p++ = ':';
		p = u16_to_hex4(p, s->usb_pid);
		fmt_done(buf, p);
		tft_draw_string(COL(4), LINE(17), COL_WHITE, buf);
	}

	if (s->usb_product[0]) {
		tft_draw_string(COL(0), LINE(18), COL_DARK, s->usb_product);
	}
#endif

#undef UPTIME_LINE

#if TFT_DRIVER == TFT_DRIVER_ILI9341
	// Bottom touch zone indicator (line 39)
	draw_separator(LINE(37));
	int settings_col = (TFT_COLS - 14) / 2;
	tft_draw_string(COL(settings_col), LINE(38), COL_DIM, "Tap for Setup");
#endif
}

#if TFT_DRIVER == TFT_DRIVER_ILI9341

typedef struct {
	const char *label;
	bool        is_bool;
} setting_info_t;

static const setting_info_t setting_info[SETTING_COUNT] = {
	{ "Smooth",    true  },
	{ "Max/Frame", false },
	{ "Humanize",  true  },
	{ "Backlight", true  },
};

#define MENU_START_Y   3  // first setting starts at line 3
#define MENU_ITEM_H    4  // lines per menu item
#define MENU_BACK_Y    (MENU_START_Y + SETTING_COUNT * MENU_ITEM_H + 1)

static void draw_settings(void)
{
	char buf[24];
	char *p;

	tft_fill(COL_BG);

	// Title
	int title_col = (TFT_COLS - 8) / 2;
	tft_draw_string(COL(title_col), LINE(0), COL_CYAN, "SETTINGS");
	draw_separator(LINE(1));

	for (uint8_t i = 0; i < SETTING_COUNT; i++) {
		int y = LINE(MENU_START_Y + i * MENU_ITEM_H);
		uint8_t label_col = (i == g_selected_setting) ? COL_YELLOW : COL_GRAY;

		if (i == g_selected_setting) {
			tft_draw_glyph(COL(0), y, COL_YELLOW, '>');
		}

		tft_draw_string(COL(2), y, label_col, setting_info[i].label);

		const char *val_str = NULL;
		uint8_t val_col = COL_WHITE;
		switch ((setting_id_t)i) {
		case SETTING_SMOOTH_ENABLED:
			val_str = g_settings.smooth_enabled ? "ON" : "OFF";
			val_col = g_settings.smooth_enabled ? COL_GREEN : COL_RED;
			break;
		case SETTING_SMOOTH_MAX:
			p = buf;
			p = u32_to_str(p, (uint32_t)g_settings.smooth_max);
			fmt_done(buf, p);
			val_str = buf;
			break;
		case SETTING_HUMANIZE_ENABLED:
			val_str = g_settings.humanize_enabled ? "ON" : "OFF";
			val_col = g_settings.humanize_enabled ? COL_GREEN : COL_RED;
			break;
		case SETTING_BACKLIGHT:
			val_str = g_settings.backlight ? "ON" : "OFF";
			val_col = g_settings.backlight ? COL_GREEN : COL_RED;
			break;
		default:
			break;
		}
		if (val_str) {
			tft_draw_string_right(TFT_WIDTH - COL(2), y, val_col, val_str);
		}

		if (i == g_selected_setting) {
			int hint_y = y + FH;
			if (setting_info[i].is_bool) {
				int hint_col = (TFT_COLS - 12) / 2;
				tft_draw_string(COL(hint_col), hint_y, COL_DIM, "Tap to toggle");
			} else {
				tft_draw_string(COL(2), hint_y, COL_DIM, "[-]");
				tft_draw_string_right(TFT_WIDTH - COL(2), hint_y, COL_DIM, "[+]");
			}
		}

		draw_separator(LINE(MENU_START_Y + i * MENU_ITEM_H + MENU_ITEM_H - 1));
	}

	int back_col = (TFT_COLS - 4) / 2;
	tft_draw_string(COL(back_col), LINE(MENU_BACK_Y), COL_CYAN, "Back");
}
#endif // TFT_DRIVER == TFT_DRIVER_ILI9341

// ---- Touch handling ----

bool tft_display_touch(uint16_t x, uint16_t y)
{
#if TFT_DRIVER == TFT_DRIVER_ILI9341
	if (g_view == TFT_VIEW_STATS) {
		// Tap bottom zone to enter settings
		if (y >= (uint16_t)LINE(37)) {
			g_view = TFT_VIEW_SETTINGS;
			return false;
		}
		return false;
	}

	// Settings view
	// Check "Back" button
	int back_y = LINE(MENU_BACK_Y);
	if (y >= (uint16_t)back_y && y < (uint16_t)(back_y + FH * 2)) {
		g_view = TFT_VIEW_STATS;
		return false;
	}

	// Check which setting was tapped
	for (uint8_t i = 0; i < SETTING_COUNT; i++) {
		int item_y = LINE(MENU_START_Y + i * MENU_ITEM_H);
		int item_end = item_y + FH * MENU_ITEM_H;
		if (y >= (uint16_t)item_y && y < (uint16_t)item_end) {
			if (i != g_selected_setting) {
				g_selected_setting = i;
				return false; // just select, don't change value yet
			}
			// Already selected — modify value
			bool left_half = (x < TFT_WIDTH / 2);
			switch ((setting_id_t)i) {
			case SETTING_SMOOTH_ENABLED:
				g_settings.smooth_enabled = !g_settings.smooth_enabled;
				return true;
			case SETTING_SMOOTH_MAX:
				if (left_half) {
					g_settings.smooth_max -= 10;
					if (g_settings.smooth_max < 1) g_settings.smooth_max = 1;
				} else {
					g_settings.smooth_max += 10;
					if (g_settings.smooth_max > 127) g_settings.smooth_max = 127;
				}
				return true;
			case SETTING_HUMANIZE_ENABLED:
				g_settings.humanize_enabled = !g_settings.humanize_enabled;
				return true;
			case SETTING_BACKLIGHT:
				g_settings.backlight = !g_settings.backlight;
				return true;
			default:
				break;
			}
		}
	}
#else
	(void)x; (void)y;
#endif
	return false;
}

tft_view_t tft_display_get_view(void)
{
	return g_view;
}

const tft_settings_t *tft_display_get_settings(void)
{
	return &g_settings;
}


void tft_display_init(void)
{
	tft_init();

	tft_fill(COL_BG);
	int title_col = (TFT_COLS - 8) / 2;
	tft_draw_string(COL(title_col), LINE(TFT_ROWS / 2 - 2), COL_CYAN, "IMXRTNSY");
	int sub_col = (TFT_COLS - 13) / 2;
	tft_draw_string(COL(sub_col), LINE(TFT_ROWS / 2), COL_GRAY, "USB HID Proxy");
#if TFT_DRIVER == TFT_DRIVER_ILI9341
	int drv_col = (TFT_COLS - 11) / 2;
	tft_draw_string(COL(drv_col), LINE(TFT_ROWS / 2 + 2), COL_DARK, "ILI9341 TFT");
#else
	int drv_col = (TFT_COLS - 10) / 2;
	tft_draw_string(COL(drv_col), LINE(TFT_ROWS / 2 + 2), COL_DARK, "ST7735 TFT");
#endif
	tft_swap_sync();
}

void tft_display_update(const tft_proxy_stats_t *stats)
{
#if TFT_DRIVER == TFT_DRIVER_ILI9341
	if (g_view == TFT_VIEW_SETTINGS) {
		draw_settings();
	} else {
		draw_stats(stats);
	}
#else
	draw_stats(stats);
#endif
	tft_swap_sync();
}

void tft_display_error(const char *msg)
{
	tft_fill(COL_BG);
	int err_col = (TFT_COLS - 5) / 2;
	tft_draw_string(COL(err_col), LINE(TFT_ROWS / 2 - 2), COL_RED, "ERROR");

	int y = LINE(TFT_ROWS / 2);
	int max_chars = TFT_WIDTH / FW;
	while (*msg && y < TFT_HEIGHT - FH) {
		int len = 0;
		const char *p = msg;
		while (*p && len < max_chars) { p++; len++; }
		for (int i = 0; i < len; i++)
			tft_draw_glyph(i * FW, y, COL_WHITE, msg[i]);
		msg += len;
		y += FH;
	}
	tft_swap_sync();
}
