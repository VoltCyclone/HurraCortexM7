// Ferrum KM API text protocol parser (matches FerrumAPI.py wire format)
// Wire format: km.mouse_move(dx,dy)\r  km.mouse_button_press(code)\r
// Button codes are bitmasks: 1=left, 2=right, 4=middle, 8=side1, 16=side2
// Keyboard: km.key_press(hid), km.key_down(hid), km.key_up(hid) (alias key_release)
//           km.key_click(hid) = press + auto-release
//           km.key_block(hid,state) = acknowledged
// Response: >>>\r\n  (with optional text payload before >>>)
// Version: km.version() → "km.ver(V2.0.0)\r\n>>>\r\n"

#include "ferrum.h"
#include <string.h>

// HID button bitmask values (Ferrum uses bitmask directly, not index)
#define BTN_LEFT    0x01
#define BTN_RIGHT   0x02
#define BTN_MIDDLE  0x04
#define BTN_BACK    0x08
#define BTN_FORWARD 0x10

// Persistent button state (sticky press/release)
static uint8_t g_buttons;

// Persistent keyboard state
static uint8_t g_kb_modifier;
static uint8_t g_kb_keys[6];

// ---- Helpers ----

static bool parse_int(const char **p, int32_t *out)
{
	while (**p == ' ' || **p == '\t') (*p)++;
	if (**p == '\0' || **p == ')') return false;

	bool neg = false;
	if (**p == '-') { neg = true; (*p)++; }
	else if (**p == '+') { (*p)++; }

	if (**p < '0' || **p > '9') return false;

	int32_t val = 0;
	while (**p >= '0' && **p <= '9') {
		val = val * 10 + (**p - '0');
		(*p)++;
	}
	*out = neg ? -val : val;
	return true;
}

static void skip_sep(const char **p)
{
	while (**p == ' ' || **p == '\t' || **p == ',') (*p)++;
}

static bool starts_with(const char *s, const char *prefix)
{
	while (*prefix) {
		if (*s++ != *prefix++) return false;
	}
	return true;
}

static void skip_open(const char **p)
{
	if (**p == '(') (*p)++;
	while (**p == ' ' || **p == '\t') (*p)++;
}

// ---- Public API ----

void ferrum_init(void)
{
	g_buttons = 0;
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
}

bool ferrum_parse_line(const char *line, uint8_t len, ferrum_result_t *out)
{
	memset(out, 0, sizeof(*out));

	if (len < 4 || !starts_with(line, "km.")) return false;
	const char *p = line + 3;

	// km.mouse_move(dx, dy)
	if (starts_with(p, "mouse_move(")) {
		p += 10;
		skip_open(&p);
		int32_t x, y;
		if (!parse_int(&p, &x)) return false;
		skip_sep(&p);
		if (!parse_int(&p, &y)) return false;

		out->mouse_dx = (int16_t)x;
		out->mouse_dy = (int16_t)y;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.mouse_button_press(bitmask) — press button(s)
	if (starts_with(p, "mouse_button_press(")) {
		p += 18;
		skip_open(&p);
		int32_t code;
		if (!parse_int(&p, &code)) return false;

		g_buttons |= (uint8_t)code;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.mouse_button_release(bitmask) — release button(s)
	if (starts_with(p, "mouse_button_release(")) {
		p += 20;
		skip_open(&p);
		int32_t code;
		if (!parse_int(&p, &code)) return false;

		g_buttons &= ~(uint8_t)code;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// km.mouse_button_is_pressed(bitmask) — query button state
	if (starts_with(p, "mouse_button_is_pressed(")) {
		p += 23;
		skip_open(&p);
		int32_t code;
		if (!parse_int(&p, &code)) return false;

		out->text_response = (g_buttons & (uint8_t)code) ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}

	// km.key_down(hid_code) / km.key_press(hid_code) — press key
	if (starts_with(p, "key_down(") || starts_with(p, "key_press(")) {
		uint8_t skip = starts_with(p, "key_down(") ? 8 : 9;
		p += skip;
		skip_open(&p);
		int32_t key;
		if (!parse_int(&p, &key)) return false;

		uint8_t hid = (uint8_t)key;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == hid) break; // already pressed
			if (g_kb_keys[i] == 0) {
				g_kb_keys[i] = hid;
				break;
			}
		}
		out->kb_modifier = g_kb_modifier;
		memcpy(out->kb_keys, g_kb_keys, 6);
		out->has_keyboard = true;
		out->needs_response = true;
		return true;
	}

	// km.key_up(hid_code) / km.key_release(hid_code) — release key
	if (starts_with(p, "key_up(") || starts_with(p, "key_release(")) {
		uint8_t skip = starts_with(p, "key_up(") ? 6 : 11;
		p += skip;
		skip_open(&p);
		int32_t key;
		if (!parse_int(&p, &key)) return false;

		uint8_t hid = (uint8_t)key;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == hid) {
				g_kb_keys[i] = 0;
				break;
			}
		}
		out->kb_modifier = g_kb_modifier;
		memcpy(out->kb_keys, g_kb_keys, 6);
		out->has_keyboard = true;
		out->needs_response = true;
		return true;
	}

	// km.key_click(hid_code) — press + auto-release
	if (starts_with(p, "key_click(")) {
		p += 9;
		skip_open(&p);
		int32_t key;
		if (!parse_int(&p, &key)) return false;

		uint8_t hid = (uint8_t)key;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == hid) break;
			if (g_kb_keys[i] == 0) {
				g_kb_keys[i] = hid;
				break;
			}
		}
		out->kb_modifier = g_kb_modifier;
		memcpy(out->kb_keys, g_kb_keys, 6);
		out->has_keyboard = true;
		out->kb_click_release = true;
		out->kb_release_key = hid;
		out->needs_response = true;
		return true;
	}

	// km.key_block(hid_code, state) — key masking (acknowledged, no action)
	if (starts_with(p, "key_block(")) {
		out->needs_response = true;
		return true;
	}

	// km.key_is_pressed(hid_code) — query key state
	if (starts_with(p, "key_is_pressed(")) {
		p += 14;
		skip_open(&p);
		int32_t key;
		if (!parse_int(&p, &key)) return false;

		uint8_t hid = (uint8_t)key;
		bool pressed = false;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == hid) { pressed = true; break; }
		}
		out->text_response = pressed ? "1\r\n" : "0\r\n";
		out->needs_response = true;
		return true;
	}

	// km.version() — KMBox-compatible identity string
	if (starts_with(p, "version")) {
		out->text_response = "km.ver(V2.0.0)\r\n";
		out->needs_response = true;
		return true;
	}

	// km.monitor(enable) / km.reboot() / km.baud(rate) — acknowledged
	if (starts_with(p, "monitor(") || starts_with(p, "reboot(") ||
	    starts_with(p, "baud(")) {
		out->needs_response = true;
		return true;
	}

	return false;
}
