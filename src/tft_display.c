// tft_display.c — TFT stats + settings rendering

#include "tft_display.h"
#include "tft.h"
#include <string.h>

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

#define FW   TFT_FONT_W
#define FH   TFT_FONT_H
#define LINE(n)  ((n) * FH)
#define COL(n)   ((n) * FW)

static void draw_separator(int y)
{
	tft_draw_hline(2, TFT_WIDTH - 3, y + FH / 2, COL_DIM);
}

static tft_view_t     g_view = TFT_VIEW_STATS;
static tft_settings_t g_settings = {
	.identity         = (CMD_BAUD >= 2000000) ? IDENTITY_FERRUM : IDENTITY_MACKU,
	.baud             = CMD_BAUD,
	.smooth_enabled   = true,
	.smooth_max       = 127,
	.humanize_enabled = true,
	.backlight        = true,
};

static const uint32_t baud_presets[] = {
	115200, 921600, 1000000, 2000000, 3000000, 4000000
};
#define BAUD_PRESET_COUNT (sizeof(baud_presets) / sizeof(baud_presets[0]))
static uint8_t g_selected_setting = 0;

#define RATE_HIST_LEN  34
static uint32_t g_rate_hist[RATE_HIST_LEN];
static uint8_t  g_rate_hist_wr;
static uint8_t  g_graph_tick;
static uint32_t g_peak_rate;

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
#if BT_ENABLED
	tft_draw_string(COL(16), LINE(2), COL_GRAY, "BT:");
	tft_draw_string(COL(19), LINE(2),
		s->bt_connected ? COL_GREEN : COL_RED,
		s->bt_connected ? "OK" : "--");
#endif

	{
		const char *proto[] = { "None", "MAKD", "MACKU", "Ferrum" };
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

	tft_draw_string(COL(0), LINE(9), COL_GRAY, "MAKD:");
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

	// Line 10: UART hardware error breakdown (only if errors present)
	{
		uint32_t hw_total = s->uart_overrun + s->uart_framing + s->uart_noise;
		if (hw_total > 0) {
			tft_draw_string(COL(0), LINE(10), COL_GRAY, "OR:");
			p = buf; p = u32_to_str(p, s->uart_overrun); fmt_done(buf, p);
			tft_draw_string(COL(3), LINE(10),
				s->uart_overrun > 0 ? COL_RED : COL_DARK, buf);

			tft_draw_string(COL(8), LINE(10), COL_GRAY, "FE:");
			p = buf; p = u32_to_str(p, s->uart_framing); fmt_done(buf, p);
			tft_draw_string(COL(11), LINE(10),
				s->uart_framing > 0 ? COL_RED : COL_DARK, buf);

			tft_draw_string(COL(16), LINE(10), COL_GRAY, "NF:");
			p = buf; p = u32_to_str(p, s->uart_noise); fmt_done(buf, p);
			tft_draw_string(COL(19), LINE(10),
				s->uart_noise > 0 ? COL_YELLOW : COL_DARK, buf);
		}
	}

	draw_separator(LINE(11));

	// Line 12: Smooth injection
	tft_draw_string(COL(0), LINE(12), COL_GRAY, "Smooth:");
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
		tft_draw_string(COL(7), LINE(12), COL_YELLOW, buf);
	} else {
		tft_draw_string(COL(7), LINE(12), COL_DARK, "idle");
	}

	draw_separator(LINE(13));

#if NET_ENABLED
	// Lines 14-17: Network stats
	{
		uint32_t ip = s->net_ip;
		tft_draw_string(COL(0), LINE(14), COL_GRAY, "IP:");
		p = buf;
		p = u32_to_str(p, (ip >> 24) & 0xFF); *p++ = '.';
		p = u32_to_str(p, (ip >> 16) & 0xFF); *p++ = '.';
		p = u32_to_str(p, (ip >> 8) & 0xFF);  *p++ = '.';
		p = u32_to_str(p, ip & 0xFF);
		fmt_done(buf, p);
		tft_draw_string(COL(3), LINE(14), COL_WHITE, buf);
	}

	tft_draw_string(COL(0), LINE(15), COL_GRAY, "Port:");
	p = buf;
	p = u32_to_str(p, s->net_port);
	fmt_done(buf, p);
	tft_draw_string(COL(5), LINE(15), COL_WHITE, buf);

	tft_draw_string(COL(11), LINE(15), COL_GRAY, "Link:");
	tft_draw_string(COL(16), LINE(15),
		s->net_link_up ? COL_GREEN : COL_RED,
		s->net_link_up ? "UP" : "DN");

	{
		tft_draw_string(COL(0), LINE(16), COL_GRAY, "UUID:");
		p = buf;
		p = u16_to_hex4(p, (uint16_t)(s->net_uuid >> 16));
		p = u16_to_hex4(p, (uint16_t)(s->net_uuid & 0xFFFF));
		fmt_done(buf, p);
		tft_draw_string(COL(5), LINE(16), COL_CYAN, buf);

		tft_draw_string(COL(14), LINE(16),
			s->net_connected ? COL_GREEN : COL_DARK,
			s->net_connected ? "CONN" : "WAIT");
	}

	tft_draw_string(COL(0), LINE(17), COL_GRAY, "RX:");
	p = buf;
	p = u32_to_str(p, s->net_rx_count);
	fmt_done(buf, p);
	tft_draw_string(COL(3), LINE(17), COL_WHITE, buf);

	tft_draw_string(COL(11), LINE(17), COL_GRAY, "TX:");
	p = buf;
	p = u32_to_str(p, s->net_tx_count);
	fmt_done(buf, p);
	tft_draw_string(COL(14), LINE(17), COL_WHITE, buf);

	draw_separator(LINE(18));
#else
	// Line 14: UART RX/TX activity
	tft_draw_string(COL(0), LINE(14), COL_GRAY, "RX:");
	if (s->uart_rx_bytes > 0) {
		p = buf;
		p = u32_to_str(p, s->uart_rx_bytes);
		*p++ = 'B';
		fmt_done(buf, p);
		tft_draw_string(COL(3), LINE(14), COL_GREEN, buf);
	} else {
		tft_draw_string(COL(3), LINE(14), COL_RED, "no data");
	}
	tft_draw_string(COL(11), LINE(14), COL_GRAY, "TX:");
	if (s->uart_tx_bytes > 0) {
		p = buf;
		p = u32_to_str(p, s->uart_tx_bytes);
		*p++ = 'B';
		fmt_done(buf, p);
		tft_draw_string(COL(14), LINE(14), COL_GREEN, buf);
	} else {
		tft_draw_string(COL(14), LINE(14), COL_RED, "0");
	}

#if BT_ENABLED
	tft_draw_string(COL(12), LINE(14), COL_GRAY, "BT:");
	tft_draw_string(COL(15), LINE(14),
		s->bt_connected ? COL_GREEN : COL_DARK,
		s->bt_connected ? "CONN" : "----");

	// Line 15: BT baud + frame stats
	tft_draw_string(COL(0), LINE(15), COL_GRAY, "BT:");
	p = buf;
	p = u32_to_str(p, s->bt_baud);
	fmt_done(buf, p);
	tft_draw_string(COL(3), LINE(15), COL_CYAN, buf);

	p = buf;
	p = u32_to_str(p, s->bt_frames_ok);
	*p++ = 'f';
	fmt_done(buf, p);
	tft_draw_string(COL(12), LINE(15), COL_WHITE, buf);

	// Line 16: BT errors (if any)
	if (s->bt_frames_err > 0) {
		tft_draw_string(COL(0), LINE(16), COL_GRAY, "BT E:");
		p = buf;
		p = u32_to_str(p, s->bt_frames_err);
		fmt_done(buf, p);
		tft_draw_string(COL(5), LINE(16), COL_RED, buf);
	}

	draw_separator(LINE(17));
#else
	draw_separator(LINE(15));
#endif
#endif

	// Line offsets depend on whether NET/BT stats took extra rows
#if NET_ENABLED
#define UPTIME_LINE  19
#elif BT_ENABLED
#define UPTIME_LINE  18
#else
#define UPTIME_LINE  16
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
	draw_separator(LINE(UPTIME_LINE + 1));

	{
		tft_draw_string(COL(0), LINE(UPTIME_LINE + 2), COL_GRAY, "USB:");
		p = buf;
		p = u16_to_hex4(p, s->usb_vid);
		*p++ = ':';
		p = u16_to_hex4(p, s->usb_pid);
		fmt_done(buf, p);
		tft_draw_string(COL(4), LINE(UPTIME_LINE + 2), COL_WHITE, buf);
	}

	if (s->usb_product[0]) {
		tft_draw_string(COL(0), LINE(UPTIME_LINE + 3), COL_DARK, s->usb_product);
	}
#endif

	// Compute graph start line based on where preceding content ends
#if NET_ENABLED
#define GRAPH_LINE  (UPTIME_LINE + 1)
#else
#define GRAPH_LINE  (UPTIME_LINE + 4)
#endif

	if (tft_detected_driver == TFT_DRIVER_ILI9341) {
		// Sample rate into ring buffer every ~1s (30 frames at 30Hz)
		g_graph_tick++;
		if (g_graph_tick >= 30) {
			g_graph_tick = 0;
			g_rate_hist[g_rate_hist_wr] = s->reports_per_sec;
			g_rate_hist_wr = (g_rate_hist_wr + 1) % RATE_HIST_LEN;
			if (s->reports_per_sec > g_peak_rate)
				g_peak_rate = s->reports_per_sec;
		}

		draw_separator(LINE(GRAPH_LINE));

		tft_draw_string(COL(1), LINE(GRAPH_LINE + 1), COL_CYAN, "THROUGHPUT");
		p = buf;
		*p++ = 'p'; *p++ = 'k'; *p++ = ' ';
		p = u32_to_str(p, g_peak_rate);
		fmt_done(buf, p);
		tft_draw_string_right(TFT_WIDTH - COL(1), LINE(GRAPH_LINE + 1), COL_DIM, buf);

		uint32_t mx = 10;
		for (int i = 0; i < RATE_HIST_LEN; i++)
			if (g_rate_hist[i] > mx) mx = g_rate_hist[i];
		if (mx < 100)
			mx = ((mx + 9) / 10) * 10;
		else
			mx = ((mx + 99) / 100) * 100;

		int gx0 = COL(5);
		int gx1 = TFT_WIDTH - COL(1);
		int gy0 = LINE(GRAPH_LINE + 3);
		int gy1 = LINE(34);
		int gh  = gy1 - gy0;

		p = buf; p = u32_to_str(p, mx); fmt_done(buf, p);
		tft_draw_string_right(gx0 - 2, gy0, COL_DIM, buf);
		tft_draw_string_right(gx0 - 2, gy1 - FH, COL_DIM, "0");

		// Mid-line dotted grid + label
		int mid_y = gy0 + gh / 2;
		for (int x = gx0; x <= gx1; x += 4)
			tft_draw_pixel(x, mid_y, COL_DARK);
		p = buf; p = u32_to_str(p, mx / 2); fmt_done(buf, p);
		tft_draw_string_right(gx0 - 2, mid_y - FH / 2, COL_DIM, buf);

		// Left axis (vertical) + bottom axis (horizontal)
		tft_draw_rect(gx0 - 1, gy0, gx0 - 1, gy1, COL_DIM);
		tft_draw_hline(gx0 - 1, gx1, gy1 + 1, COL_DIM);

		int bar_step = (gx1 - gx0) / RATE_HIST_LEN;
		int bar_w = bar_step - 1;
		if (bar_w < 1) bar_w = 1;
		int newest = (g_rate_hist_wr + RATE_HIST_LEN - 1) % RATE_HIST_LEN;

		for (int i = 0; i < RATE_HIST_LEN; i++) {
			int idx = (g_rate_hist_wr + i) % RATE_HIST_LEN;
			uint32_t val = g_rate_hist[idx];
			if (val == 0) continue;

			int bh = (int)((val * (uint32_t)gh) / mx);
			if (bh < 1) bh = 1;
			if (bh > gh) bh = gh;

			int bx = gx0 + i * bar_step;
			uint8_t col = (idx == newest) ? COL_CYAN : COL_GREEN;

			tft_draw_rect(bx, gy1 - bh, bx + bar_w - 1, gy1 - 1, col);
		}

		// Time scale hint
		tft_draw_string_right(TFT_WIDTH - COL(1), LINE(35), COL_DIM,
			"~34s");

		draw_separator(LINE(37));
		int settings_col = (TFT_COLS - 14) / 2;
		tft_draw_string(COL(settings_col), LINE(38), COL_DIM, "Tap for Setup");
	}
#undef GRAPH_LINE
#undef UPTIME_LINE
}

typedef struct {
	const char *label;
	bool        is_bool;
} setting_info_t;

static const setting_info_t setting_info[SETTING_COUNT] = {
	{ "Identity",  false },
	{ "Baud",      false },
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
		case SETTING_IDENTITY: {
			const char *names[] = { "MACKU", "Ferrum" };
			uint8_t id = (uint8_t)g_settings.identity;
			if (id >= IDENTITY_COUNT) id = 0;
			val_str = names[id];
			val_col = COL_CYAN;
			break;
		}
		case SETTING_BAUD: {
			// Show baud as compact string: "115K", "921K", "1M", "2M", etc.
			uint32_t b = g_settings.baud;
			p = buf;
			if (b >= 1000000) {
				p = u32_to_str(p, b / 1000000);
				*p++ = 'M';
			} else {
				p = u32_to_str(p, b / 1000);
				*p++ = 'K';
			}
			fmt_done(buf, p);
			val_str = buf;
			val_col = COL_YELLOW;
			break;
		}
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
			if (setting_info[i].is_bool || (setting_id_t)i == SETTING_IDENTITY) {
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

bool tft_display_touch(uint16_t x, uint16_t y)
{
	if (tft_detected_driver != TFT_DRIVER_ILI9341) {
		(void)x; (void)y;
		return false;
	}

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
			case SETTING_IDENTITY:
				// Toggle, but Ferrum only allowed at >= 2Mbaud
				if (g_settings.identity == IDENTITY_MACKU) {
					if (g_settings.baud >= 2000000)
						g_settings.identity = IDENTITY_FERRUM;
				} else {
					g_settings.identity = IDENTITY_MACKU;
				}
				return true;
			case SETTING_BAUD: {
				// Find current index in presets, step [-] or [+]
				int idx = -1;
				for (int j = 0; j < (int)BAUD_PRESET_COUNT; j++) {
					if (baud_presets[j] == g_settings.baud) { idx = j; break; }
				}
				if (idx < 0) idx = 0; // snap to first if custom
				if (left_half) {
					if (idx > 0) idx--;
				} else {
					if (idx < (int)BAUD_PRESET_COUNT - 1) idx++;
				}
				g_settings.baud = baud_presets[idx];
				// Auto-update identity: force MACKU if dropping below 2M
				if (g_settings.baud < 2000000)
					g_settings.identity = IDENTITY_MACKU;
				return true;
			}
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
	if (tft_detected_driver == TFT_DRIVER_ILI9341) {
		int drv_col = (TFT_COLS - 11) / 2;
		tft_draw_string(COL(drv_col), LINE(TFT_ROWS / 2 + 2), COL_DARK, "ILI9341 TFT");
	} else {
		int drv_col = (TFT_COLS - 10) / 2;
		tft_draw_string(COL(drv_col), LINE(TFT_ROWS / 2 + 2), COL_DARK, "ST7735 TFT");
	}
	tft_swap_sync();
}

void tft_display_update(const tft_proxy_stats_t *stats)
{
	if (tft_detected_driver == TFT_DRIVER_ILI9341 &&
	    g_view == TFT_VIEW_SETTINGS) {
		draw_settings();
	} else {
		draw_stats(stats);
	}
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
