// MAKCU text protocol parser (matches makcu-rs wire format)
// Wire: km.move(dx,dy)\r\n  km.left(1)\r\n  km.wheel(d)\r\n
// Tracked commands: km.move(dx,dy)#42\r\n → respond with >>> OK#42\r\n
// Button names: left, right, middle, ms1, ms2
// Lock commands: km.lock_mx(1), km.lock_ml(1), etc. (acknowledged, no action)
// System: km.buttons(1), km.serial('x'), km.version()

#include "makcu.h"
#include <string.h>

// HID button masks
#define BTN_LEFT    0x01
#define BTN_RIGHT   0x02
#define BTN_MIDDLE  0x04
#define BTN_BACK    0x08
#define BTN_FORWARD 0x10

// Persistent button state
static uint8_t g_buttons;

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

// Extract tracked command ID from line. Scans for )#<number> pattern.
// Returns 0 if untracked. Writes the position of '#' to hash_pos if found.
static uint32_t extract_track_id(const char *line, uint8_t len)
{
	// Scan backwards for '#'
	for (int i = len - 1; i >= 0; i--) {
		if (line[i] == '#') {
			const char *p = &line[i + 1];
			uint32_t id = 0;
			bool found_digit = false;
			while (*p >= '0' && *p <= '9') {
				id = id * 10 + (*p - '0');
				found_digit = true;
				p++;
			}
			if (found_digit) return id;
		}
	}
	return 0;
}

// ---- Public API ----

void makcu_init(void)
{
	g_buttons = 0;
}

bool makcu_parse_line(const char *line, uint8_t len, makcu_result_t *out)
{
	memset(out, 0, sizeof(*out));

	if (len < 4 || !starts_with(line, "km.")) return false;
	const char *p = line + 3;

	// Extract tracking ID before parsing command
	out->track_id = extract_track_id(line, len);

	// km.move(dx, dy)
	if (starts_with(p, "move(") || starts_with(p, "move ")) {
		p += 4;
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

	// km.wheel(delta)
	if (starts_with(p, "wheel(") || starts_with(p, "wheel ")) {
		p += 5;
		skip_open(&p);
		int32_t w;
		if (!parse_int(&p, &w)) return false;

		out->mouse_wheel = (int8_t)w;
		out->mouse_buttons = g_buttons;
		out->has_mouse = true;
		out->needs_response = true;
		return true;
	}

	// Button commands: km.left(state), km.right(state), km.middle(state),
	//                  km.ms1(state), km.ms2(state)
	// No-arg form: km.left() = click (press + auto-release)
	struct { const char *name; uint8_t nlen; uint8_t mask; } btns[] = {
		{ "left",   4, BTN_LEFT },
		{ "right",  5, BTN_RIGHT },
		{ "middle", 6, BTN_MIDDLE },
		{ "ms1",    3, BTN_BACK },
		{ "ms2",    3, BTN_FORWARD },
	};

	for (uint8_t i = 0; i < 5; i++) {
		if (starts_with(p, btns[i].name) &&
		    (p[btns[i].nlen] == '(' || p[btns[i].nlen] == ' ')) {
			p += btns[i].nlen;
			skip_open(&p);

			// Check for no-arg click: km.left() or km.left()#id
			if (*p == ')' || *p == '#' || *p == '\0') {
				// Click: press + schedule release
				g_buttons |= btns[i].mask;
				out->mouse_buttons = g_buttons;
				out->has_mouse = true;
				out->click_release = true;
				out->needs_response = true;
				return true;
			}

			int32_t st;
			if (!parse_int(&p, &st)) return false;

			if (st != 0)
				g_buttons |= btns[i].mask;
			else
				g_buttons &= ~btns[i].mask;

			out->mouse_buttons = g_buttons;
			out->has_mouse = true;
			out->needs_response = true;
			return true;
		}
	}

	// km.lock_* commands — acknowledged, no action on our side
	if (starts_with(p, "lock_")) {
		out->needs_response = true;
		return true;
	}

	// km.buttons(state) — enable/disable button state broadcasting
	if (starts_with(p, "buttons(")) {
		out->needs_response = true;
		return true;
	}

	// km.serial('value') — set serial identifier (acknowledged)
	if (starts_with(p, "serial(")) {
		out->needs_response = true;
		return true;
	}

	// km.version() — KMBox-compatible identity string
	if (starts_with(p, "version")) {
		out->text_response = "km.ver(V2.0.0)\r\n";
		out->needs_response = true;
		return true;
	}

	return false;
}
