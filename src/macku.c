// Macku text protocol — MACKU / Ferrum-compatible command dispatch and handlers

#include "macku.h"
#include "makd.h"
#include "kmbox.h"
#include "imxrt.h"
#include <string.h>

extern uint32_t millis(void);

#define FW_VERSION_MACKU  "km.MAKCU"
#define FW_VERSION_FERRUM "kmbox: Ferrum"

// Set before calling handlers — used by reply_get/reply_set for echoing.

static const char *cur_cmd;   // original parsed command name
static const char *cur_args;  // original args string (NUL-terminated)

static void tx_str(const char *s, macku_tx_fn tx)
{
	uint16_t len = 0;
	while (s[len]) len++;
	if (len) tx((const uint8_t *)s, len);
}

// Getter response:
//  Software: km.<cmd>(<args>)\r\n<result>\r\n>>>
//  Legacy:   <result>\r\n>>>
static void reply_get(const char *result, macku_tx_fn tx)
{
	if (g_echo) {
		tx((const uint8_t *)"km.", 3);
		tx_str(cur_cmd, tx);
		tx((const uint8_t *)"(", 1);
		tx_str(cur_args, tx);
		tx((const uint8_t *)")\r\n", 3);
	}
	if (result && result[0]) {
		tx_str(result, tx);
		tx((const uint8_t *)"\r\n", 2);
	}
	tx((const uint8_t *)">>> ", 4);
}

// Setter response:
//  Software: km.<cmd>(<args>)\r\n>>>
//  Legacy:   >>>
static void reply_set(macku_tx_fn tx)
{
	if (g_echo) {
		tx((const uint8_t *)"km.", 3);
		tx_str(cur_cmd, tx);
		tx((const uint8_t *)"(", 1);
		tx_str(cur_args, tx);
		tx((const uint8_t *)")\r\n", 3);
	}
	tx((const uint8_t *)">>> ", 4);
}

static char *i32_to_str(char *buf, int32_t v)
{
	if (v < 0) { *buf++ = '-'; v = -v; }
	if (v == 0) { *buf++ = '0'; return buf; }
	char tmp[11];
	int n = 0;
	while (v > 0) { tmp[n++] = '0' + (v % 10); v /= 10; }
	while (n > 0) *buf++ = tmp[--n];
	return buf;
}

static char *u32_to_str(char *buf, uint32_t v)
{
	if (v == 0) { *buf++ = '0'; return buf; }
	char tmp[11];
	int n = 0;
	while (v > 0) { tmp[n++] = '0' + (v % 10); v /= 10; }
	while (n > 0) *buf++ = tmp[--n];
	return buf;
}

// Parse comma-separated int32 args, returns count parsed.
// Stops at NUL or when max reached.  Skips whitespace around commas.
// Handles trailing comma: ".move(1,1,)" → 2 args.
static uint8_t parse_int_args(const char *args, uint16_t len, int32_t *out, uint8_t max)
{
	uint8_t count = 0;
	uint16_t i = 0;
	while (i < len && count < max) {
		// skip whitespace/commas
		while (i < len && (args[i] == ' ' || args[i] == ',')) i++;
		if (i >= len) break;
		// check for quote (key arg, not int)
		if (args[i] == '\'' || args[i] == '"') break;
		// check for non-numeric (boolean args like "true"/"false")
		if (args[i] != '-' && args[i] != '+' && (args[i] < '0' || args[i] > '9')) break;
		// parse integer
		bool neg = false;
		if (args[i] == '-') { neg = true; i++; }
		else if (args[i] == '+') i++;
		if (i >= len || args[i] < '0' || args[i] > '9') break;
		int32_t v = 0;
		while (i < len && args[i] >= '0' && args[i] <= '9')
			v = v * 10 + (args[i++] - '0');
		out[count++] = neg ? -v : v;
	}
	return count;
}

typedef struct {
	const char *name;
	uint8_t hid;
} key_entry_t;

// Sorted alphabetically for binary search
static const key_entry_t key_table[] = {
	{"alt",            0xE2},
	{"back",           0x2A},
	{"backslash",      0x31},
	{"backspace",      0x2A},
	{"backtick",       0x35},
	{"break",          0x48},
	{"bslash",         0x31},
	{"caps",           0x39},
	{"capslock",       0x39},
	{"closebracket",   0x30},
	{"cmd",            0xE3},
	{"comma",          0x36},
	{"command",        0xE3},
	{"control",        0xE0},
	{"ctrl",           0xE0},
	{"dash",           0x2D},
	{"del",            0x4C},
	{"delete",         0x4C},
	{"dot",            0x37},
	{"down",           0x51},
	{"downarrow",      0x51},
	{"end",            0x4D},
	{"enter",          0x28},
	{"equal",          0x2E},
	{"equals",         0x2E},
	{"esc",            0x29},
	{"escape",         0x29},
	{"f1",             0x3A},
	{"f10",            0x43},
	{"f11",            0x44},
	{"f12",            0x45},
	{"f2",             0x3B},
	{"f3",             0x3C},
	{"f4",             0x3D},
	{"f5",             0x3E},
	{"f6",             0x3F},
	{"f7",             0x40},
	{"f8",             0x41},
	{"f9",             0x42},
	{"forwardslash",   0x38},
	{"fslash",         0x38},
	{"grave",          0x35},
	{"gui",            0xE3},
	{"home",           0x4A},
	{"hyphen",         0x2D},
	{"ins",            0x49},
	{"insert",         0x49},
	{"kp0",            0x62},
	{"kp1",            0x59},
	{"kp2",            0x5A},
	{"kp3",            0x5B},
	{"kp4",            0x5C},
	{"kp5",            0x5D},
	{"kp6",            0x5E},
	{"kp7",            0x5F},
	{"kp8",            0x60},
	{"kp9",            0x61},
	{"kpdivide",       0x54},
	{"kpdot",          0x63},
	{"kpenter",        0x58},
	{"kpminus",        0x56},
	{"kpmultiply",     0x55},
	{"kpperiod",       0x63},
	{"kpplus",         0x57},
	{"lalt",           0xE2},
	{"lbracket",       0x2F},
	{"lcontrol",       0xE0},
	{"lctrl",          0xE0},
	{"left",           0x50},
	{"leftalt",        0xE2},
	{"leftarrow",      0x50},
	{"leftbracket",    0x2F},
	{"leftcontrol",    0xE0},
	{"leftctrl",       0xE0},
	{"leftgui",        0xE3},
	{"leftshift",      0xE1},
	{"leftwin",        0xE3},
	{"lgui",           0xE3},
	{"lshift",         0xE1},
	{"lwin",           0xE3},
	{"meta",           0xE3},
	{"minus",          0x2D},
	{"np0",            0x62},
	{"np1",            0x59},
	{"np2",            0x5A},
	{"np3",            0x5B},
	{"np4",            0x5C},
	{"np5",            0x5D},
	{"np6",            0x5E},
	{"np7",            0x5F},
	{"np8",            0x60},
	{"np9",            0x61},
	{"npdivide",       0x54},
	{"npdot",          0x63},
	{"npenter",        0x58},
	{"npminus",        0x56},
	{"npmultiply",     0x55},
	{"npperiod",       0x63},
	{"npplus",         0x57},
	{"num",            0x53},
	{"numlock",        0x53},
	{"openbracket",    0x2F},
	{"pagedown",       0x4E},
	{"pageup",         0x4B},
	{"pause",          0x48},
	{"period",         0x37},
	{"pgdn",           0x4E},
	{"pgdown",         0x4E},
	{"pgup",           0x4B},
	{"print",          0x46},
	{"printscreen",    0x46},
	{"prtsc",          0x46},
	{"quote",          0x34},
	{"ralt",           0xE6},
	{"rbracket",       0x30},
	{"rcontrol",       0xE4},
	{"rctrl",          0xE4},
	{"return",         0x28},
	{"right",          0x4F},
	{"rightalt",       0xE6},
	{"rightarrow",     0x4F},
	{"rightbracket",   0x30},
	{"rightcontrol",   0xE4},
	{"rightctrl",      0xE4},
	{"rightgui",       0xE7},
	{"rightshift",     0xE5},
	{"rightwin",       0xE7},
	{"rgui",           0xE7},
	{"rshift",         0xE5},
	{"rwin",           0xE7},
	{"scroll",         0x47},
	{"scrolllock",     0x47},
	{"semi",           0x33},
	{"semicolon",      0x33},
	{"shift",          0xE1},
	{"singlequote",    0x34},
	{"slash",          0x38},
	{"space",          0x2C},
	{"spacebar",       0x2C},
	{"super",          0xE3},
	{"tab",            0x2B},
	{"tilde",          0x35},
	{"up",             0x52},
	{"uparrow",        0x52},
	{"win",            0xE3},
	{"windows",        0xE3},
};
#define KEY_TABLE_SIZE (sizeof(key_table) / sizeof(key_table[0]))

static int key_cmp(const char *a, const char *b)
{
	while (*a && *b) {
		char ca = *a, cb = *b;
		if (ca >= 'A' && ca <= 'Z') ca += 32;
		if (cb >= 'A' && cb <= 'Z') cb += 32;
		if (ca != cb) return (int)ca - (int)cb;
		a++; b++;
	}
	return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

// Resolve a key argument: integer, single char, or quoted multi-char name.
// Returns HID code 0-255 on success, -1 on error.
// *needs_shift is set if the key requires shift modifier (uppercase / symbols).
static int16_t resolve_key(const char *arg, uint16_t len, bool *needs_shift)
{
	*needs_shift = false;
	if (len == 0) return -1;

	// Integer literal?
	if (arg[0] >= '0' && arg[0] <= '9') {
		int32_t v = 0;
		for (uint16_t i = 0; i < len; i++) {
			if (arg[i] < '0' || arg[i] > '9') break;
			v = v * 10 + (arg[i] - '0');
		}
		return (v <= 255) ? (int16_t)v : -1;
	}

	// Single char (may be in quotes or not)?
	const char *s = arg;
	uint16_t slen = len;
	if ((s[0] == '\'' || s[0] == '"') && slen >= 3 && s[slen - 1] == s[0]) {
		s++;
		slen -= 2;
	}

	if (slen == 1) {
		char c = s[0];
		if (c >= 'a' && c <= 'z') return c - 'a' + 4;
		if (c >= 'A' && c <= 'Z') { *needs_shift = true; return c - 'A' + 4; }
		if (c >= '1' && c <= '9') return c - '1' + 0x1E;
		if (c == '0') return 0x27;
		switch (c) {
		case '-': return 0x2D;
		case '=': return 0x2E;
		case '[': return 0x2F;
		case ']': return 0x30;
		case '\\': return 0x31;
		case ';': return 0x33;
		case '\'': return 0x34;
		case '`': return 0x35;
		case ',': return 0x36;
		case '.': return 0x37;
		case '/': return 0x38;
		case ' ': return 0x2C;
		case '!': *needs_shift = true; return 0x1E;
		case '@': *needs_shift = true; return 0x1F;
		case '#': *needs_shift = true; return 0x20;
		case '$': *needs_shift = true; return 0x21;
		case '%': *needs_shift = true; return 0x22;
		case '^': *needs_shift = true; return 0x23;
		case '&': *needs_shift = true; return 0x24;
		case '*': *needs_shift = true; return 0x25;
		case '(': *needs_shift = true; return 0x26;
		case '_': *needs_shift = true; return 0x2D;
		case '+': *needs_shift = true; return 0x2E;
		case '{': *needs_shift = true; return 0x2F;
		case '}': *needs_shift = true; return 0x30;
		case '|': *needs_shift = true; return 0x31;
		case ':': *needs_shift = true; return 0x33;
		case '"': *needs_shift = true; return 0x34;
		case '~': *needs_shift = true; return 0x35;
		case '<': *needs_shift = true; return 0x36;
		case '>': *needs_shift = true; return 0x37;
		case '?': *needs_shift = true; return 0x38;
		}
		return -1;
	}

	// Multi-char name — binary search key_table
	char lower[32];
	if (slen >= sizeof(lower)) return -1;
	for (uint16_t i = 0; i < slen; i++) {
		char c = s[i];
		if (c >= 'A' && c <= 'Z') c += 32;
		lower[i] = c;
	}
	lower[slen] = '\0';

	int lo = 0, hi = (int)KEY_TABLE_SIZE - 1;
	while (lo <= hi) {
		int mid = (lo + hi) / 2;
		int cmp = key_cmp(lower, key_table[mid].name);
		if (cmp == 0) return key_table[mid].hid;
		if (cmp < 0) hi = mid - 1;
		else lo = mid + 1;
	}
	return -1;
}

// Parse a key argument from the args string starting at position *pos.
// Handles: integer, 'x', "name", unquoted_name
// Returns HID code or -1. Advances *pos past the argument and trailing comma.
static int16_t parse_key_arg(const char *args, uint16_t len, uint16_t *pos,
                             bool *needs_shift)
{
	uint16_t i = *pos;
	while (i < len && (args[i] == ' ' || args[i] == ',')) i++;
	if (i >= len) return -1;

	if (args[i] == '\'' || args[i] == '"') {
		char q = args[i];
		uint16_t start = i;
		i++;
		while (i < len && args[i] != q) i++;
		if (i < len) i++;
		*pos = i;
		return resolve_key(&args[start], i - start, needs_shift);
	}

	uint16_t start = i;
	while (i < len && args[i] != ',' && args[i] != ' ' && args[i] != '\0') i++;
	*pos = i;
	return resolve_key(&args[start], i - start, needs_shift);
}

typedef struct {
	uint8_t hid;
	uint8_t shift;
} ascii_hid_t;

static const ascii_hid_t ascii_to_hid[95] = {
	{0x2C, 0}, // 32 ' '
	{0x1E, 1}, // 33 '!'
	{0x34, 1}, // 34 '"'
	{0x20, 1}, // 35 '#'
	{0x21, 1}, // 36 '$'
	{0x22, 1}, // 37 '%'
	{0x24, 1}, // 38 '&'
	{0x34, 0}, // 39 '\''
	{0x26, 1}, // 40 '('
	{0x27, 1}, // 41 ')'
	{0x25, 1}, // 42 '*'
	{0x2E, 1}, // 43 '+'
	{0x36, 0}, // 44 ','
	{0x2D, 0}, // 45 '-'
	{0x37, 0}, // 46 '.'
	{0x38, 0}, // 47 '/'
	{0x27, 0}, // 48 '0'
	{0x1E, 0}, // 49 '1'
	{0x1F, 0}, // 50 '2'
	{0x20, 0}, // 51 '3'
	{0x21, 0}, // 52 '4'
	{0x22, 0}, // 53 '5'
	{0x23, 0}, // 54 '6'
	{0x24, 0}, // 55 '7'
	{0x25, 0}, // 56 '8'
	{0x26, 0}, // 57 '9'
	{0x33, 1}, // 58 ':'
	{0x33, 0}, // 59 ';'
	{0x36, 1}, // 60 '<'
	{0x2E, 0}, // 61 '='
	{0x37, 1}, // 62 '>'
	{0x38, 1}, // 63 '?'
	{0x1F, 1}, // 64 '@'
	{0x04, 1}, // 65 'A'
	{0x05, 1}, // 66 'B'
	{0x06, 1}, // 67 'C'
	{0x07, 1}, // 68 'D'
	{0x08, 1}, // 69 'E'
	{0x09, 1}, // 70 'F'
	{0x0A, 1}, // 71 'G'
	{0x0B, 1}, // 72 'H'
	{0x0C, 1}, // 73 'I'
	{0x0D, 1}, // 74 'J'
	{0x0E, 1}, // 75 'K'
	{0x0F, 1}, // 76 'L'
	{0x10, 1}, // 77 'M'
	{0x11, 1}, // 78 'N'
	{0x12, 1}, // 79 'O'
	{0x13, 1}, // 80 'P'
	{0x14, 1}, // 81 'Q'
	{0x15, 1}, // 82 'R'
	{0x16, 1}, // 83 'S'
	{0x17, 1}, // 84 'T'
	{0x18, 1}, // 85 'U'
	{0x19, 1}, // 86 'V'
	{0x1A, 1}, // 87 'W'
	{0x1B, 1}, // 88 'X'
	{0x1C, 1}, // 89 'Y'
	{0x1D, 1}, // 90 'Z'
	{0x2F, 0}, // 91 '['
	{0x31, 0}, // 92 '\\'
	{0x30, 0}, // 93 ']'
	{0x23, 1}, // 94 '^'
	{0x2D, 1}, // 95 '_'
	{0x35, 0}, // 96 '`'
	{0x04, 0}, // 97 'a'
	{0x05, 0}, // 98 'b'
	{0x06, 0}, // 99 'c'
	{0x07, 0}, // 100 'd'
	{0x08, 0}, // 101 'e'
	{0x09, 0}, // 102 'f'
	{0x0A, 0}, // 103 'g'
	{0x0B, 0}, // 104 'h'
	{0x0C, 0}, // 105 'i'
	{0x0D, 0}, // 106 'j'
	{0x0E, 0}, // 107 'k'
	{0x0F, 0}, // 108 'l'
	{0x10, 0}, // 109 'm'
	{0x11, 0}, // 110 'n'
	{0x12, 0}, // 111 'o'
	{0x13, 0}, // 112 'p'
	{0x14, 0}, // 113 'q'
	{0x15, 0}, // 114 'r'
	{0x16, 0}, // 115 's'
	{0x17, 0}, // 116 't'
	{0x18, 0}, // 117 'u'
	{0x19, 0}, // 118 'v'
	{0x1A, 0}, // 119 'w'
	{0x1B, 0}, // 120 'x'
	{0x1C, 0}, // 121 'y'
	{0x1D, 0}, // 122 'z'
	{0x2F, 1}, // 123 '{'
	{0x31, 1}, // 124 '|'
	{0x30, 1}, // 125 '}'
	{0x35, 1}, // 126 '~'
};

// ======== LOCK / CATCH TARGET RESOLUTION ========

typedef struct {
	const char *name;
	uint8_t id;
} lock_target_t;

static const lock_target_t lock_targets[] = {
	{"ml",  MAKD_LOCK_ML},
	{"mm",  MAKD_LOCK_MM},
	{"mr",  MAKD_LOCK_MR},
	{"ms1", MAKD_LOCK_MS1},
	{"ms2", MAKD_LOCK_MS2},
	{"mw",  MAKD_LOCK_MW},
	{"mw+", MAKD_LOCK_MW_POS},
	{"mw-", MAKD_LOCK_MW_NEG},
	{"mx",  MAKD_LOCK_MX},
	{"mx+", MAKD_LOCK_MX_POS},
	{"mx-", MAKD_LOCK_MX_NEG},
	{"my",  MAKD_LOCK_MY},
	{"my+", MAKD_LOCK_MY_POS},
	{"my-", MAKD_LOCK_MY_NEG},
};
#define LOCK_TARGET_COUNT (sizeof(lock_targets) / sizeof(lock_targets[0]))

static int8_t resolve_lock_target(const char *suffix)
{
	for (uint8_t i = 0; i < LOCK_TARGET_COUNT; i++) {
		const char *a = suffix, *b = lock_targets[i].name;
		while (*a && *b && *a == *b) { a++; b++; }
		if (*a == '\0' && *b == '\0') return lock_targets[i].id;
	}
	return -1;
}

static const lock_target_t catch_targets[] = {
	{"ml",  MAKD_LOCK_ML},
	{"mm",  MAKD_LOCK_MM},
	{"mr",  MAKD_LOCK_MR},
	{"ms1", MAKD_LOCK_MS1},
	{"ms2", MAKD_LOCK_MS2},
};
#define CATCH_TARGET_COUNT (sizeof(catch_targets) / sizeof(catch_targets[0]))

static int8_t resolve_catch_target(const char *suffix)
{
	for (uint8_t i = 0; i < CATCH_TARGET_COUNT; i++) {
		const char *a = suffix, *b = catch_targets[i].name;
		while (*a && *b && *a == *b) { a++; b++; }
		if (*a == '\0' && *b == '\0') return catch_targets[i].id;
	}
	return -1;
}

// ======== BUTTON HELPERS ========

static uint8_t btn_name_to_mask(const char *name)
{
	if (name[0] == 'l') return 0x01; // left
	if (name[0] == 'r') return 0x02; // right
	if (name[0] == 'm') return 0x04; // middle
	if (name[0] == 's') {
		if (name[4] == '1') return 0x08; // side1
		if (name[4] == '2') return 0x10; // side2
	}
	return 0;
}

// ---- Mouse buttons: left/right/middle/side1/side2 ----
// GET: returns 0 (released) or 1 (pressed)
// SET: 0=release, 1=press

static void handle_button(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint8_t mask = btn_name_to_mask(cur_cmd);
	if (!mask) return;

	if (args_len == 0) {
		char buf[4] = { (g_buttons & mask) ? '1' : '0', '\0' };
		reply_get(buf, tx);
		return;
	}

	int32_t vals[1];
	if (parse_int_args(args, args_len, vals, 1) < 1) {
		reply_set(tx);
		return;
	}
	makd_action_button_set(mask, (uint8_t)vals[0]);
	reply_set(tx);
}

// ---- Click scheduling ----
// Ferrum: km.click(button) where button is 0-4 (0=left,1=right,2=mid,3=side1,4=side2)

static void handle_click(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[3] = {0, 1, 0};
	uint8_t n = parse_int_args(args, args_len, vals, 3);
	if (n < 1) { reply_set(tx); return; }

	uint8_t button = (uint8_t)vals[0]; // 0-indexed: 0=left .. 4=side2
	uint8_t count = (n >= 2) ? (uint8_t)vals[1] : 1;
	uint8_t delay = (n >= 3) ? (uint8_t)vals[2] : 0;

	if (button > 4) { reply_set(tx); return; }

	uint32_t d = delay ? delay : (75 + (millis() % 51));
	makd_action_click(button + 1, count, d); // convert 0-indexed to 1-indexed
	reply_set(tx);
}

// ---- Turbo ----

static void handle_turbo(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[64];
		char *p = buf;
		bool first = true;
		for (int i = 0; i < 5; i++) {
			if (g_turbo_delay[i]) {
				if (!first) *p++ = ',';
				*p++ = 'm';
				*p++ = '1' + i;
				*p++ = '=';
				p = u32_to_str(p, g_turbo_delay[i]);
				first = false;
			}
		}
		*p = '\0';
		reply_get(buf, tx);
		return;
	}

	int32_t vals[2];
	uint8_t n = parse_int_args(args, args_len, vals, 2);
	if (n < 1) { reply_set(tx); return; }

	uint8_t button = (uint8_t)vals[0];
	if (button == 0) {
		memset(g_turbo_delay, 0, sizeof(g_turbo_delay));
	} else if (button >= 1 && button <= 5) {
		uint16_t delay = 0;
		if (n >= 2) delay = (uint16_t)vals[1];
		g_turbo_delay[button - 1] = delay ? delay : 1;
	}
	reply_set(tx);
}

// ---- Mouse movement ----

static void handle_move(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[7];
	uint8_t n = parse_int_args(args, args_len, vals, 7);
	if (n < 2) { reply_set(tx); return; }
	makd_action_move((int16_t)vals[0], (int16_t)vals[1], n >= 3 && vals[2] > 0);
	reply_set(tx);
}

static void handle_moveto(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[7];
	uint8_t n = parse_int_args(args, args_len, vals, 7);
	if (n < 2) { reply_set(tx); return; }
	makd_action_moveto((int16_t)vals[0], (int16_t)vals[1], n >= 3 && vals[2] > 0);
	reply_set(tx);
}

static void handle_wheel(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[1];
	if (parse_int_args(args, args_len, vals, 1) < 1) {
		reply_set(tx);
		return;
	}
	int8_t delta = (int8_t)vals[0];
	if (delta > 1) delta = 1;
	if (delta < -1) delta = -1;
	kmbox_inject_mouse(0, 0, g_buttons, delta, false);
	reply_set(tx);
}

static void handle_pan(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[8];
		char *p = i32_to_str(buf, g_pending_pan);
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_pending_pan = (int8_t)vals[0];
		reply_set(tx);
	}
}

static void handle_tilt(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[8];
		char *p = i32_to_str(buf, g_pending_tilt);
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_pending_tilt = (int8_t)vals[0];
		reply_set(tx);
	}
}

static void handle_getpos(macku_tx_fn tx)
{
	char buf[24];
	char *p = i32_to_str(buf, (int32_t)g_pos_x);
	*p++ = ',';
	p = i32_to_str(p, (int32_t)g_pos_y);
	*p = '\0';
	reply_get(buf, tx);
}

static void handle_silent(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[2];
	if (parse_int_args(args, args_len, vals, 2) < 2) {
		reply_set(tx);
		return;
	}
	makd_action_silent((int16_t)vals[0], (int16_t)vals[1]);
	reply_set(tx);
}

static void handle_mo(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	int32_t vals[6];
	uint8_t n = parse_int_args(args, args_len, vals, 6);
	if (n < 1) { reply_set(tx); return; }

	uint8_t buttons = (uint8_t)vals[0];
	int16_t x       = (n >= 2) ? (int16_t)vals[1] : 0;
	int16_t y       = (n >= 3) ? (int16_t)vals[2] : 0;
	int8_t  wheel   = (n >= 4) ? (int8_t)vals[3]  : 0;
	int8_t  pan     = (n >= 5) ? (int8_t)vals[4]  : 0;
	int8_t  tilt    = (n >= 6) ? (int8_t)vals[5]  : 0;

	// Ferrum: all-zero resets pan/tilt too
	if (buttons == 0 && x == 0 && y == 0 && wheel == 0) {
		pan = 0; tilt = 0;
	}
	makd_action_mo(buttons, x, y, wheel, pan, tilt);
	reply_set(tx);
}

// ---- Lock ----

static void handle_lock(const char *target_name, const char *args,
                        uint16_t args_len, macku_tx_fn tx)
{
	int8_t target = resolve_lock_target(target_name);
	if (target < 0) return;

	if (args_len == 0) {
		uint8_t locked = (g_lock_mask & (1u << target)) ? 1 : 0;
		char buf[4] = { '0' + locked, '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1) {
			if (vals[0])
				g_lock_mask |= (1u << target);
			else
				g_lock_mask &= ~(1u << target);
		}
		reply_set(tx);
	}
}

// ---- Catch ----

static void handle_catch(const char *target_name, const char *args,
                         uint16_t args_len, macku_tx_fn tx)
{
	int8_t target = resolve_catch_target(target_name);
	if (target < 0) return;

	if (args_len == 0) {
		char buf[4] = { '0' + g_catch_mode, '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_catch_mode = (uint8_t)vals[0];
		reply_set(tx);
	}
}

// ---- Catch XY (Ferrum: km.catch_xy(duration[, include_sw])) ----
// Returns (x, y) summed input over last duration ms.
// Stub: returns current accumulated position delta.

static void handle_catch_xy(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	(void)args; (void)args_len;
	// TODO: proper time-windowed accumulation
	char buf[32];
	char *p = buf;
	*p++ = '(';
	p = i32_to_str(p, g_pos_x);
	*p++ = ',';
	*p++ = ' ';
	p = i32_to_str(p, g_pos_y);
	*p++ = ')';
	*p = '\0';
	reply_get(buf, tx);
}

// ---- Remap button ----

static void handle_remap_button(const char *args, uint16_t args_len,
                                macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[64];
		char *p = buf;
		bool first = true;
		const char *names[] = {"left","right","middle","side1","side2"};
		for (int i = 0; i < 5; i++) {
			if (g_button_remap[i] && g_button_remap[i] != (uint8_t)(i + 1)) {
				if (!first) *p++ = ',';
				const char *sn = names[i];
				while (*sn) *p++ = *sn++;
				*p++ = ':';
				const char *dn = (g_button_remap[i] >= 1 && g_button_remap[i] <= 5)
					? names[g_button_remap[i] - 1] : "?";
				while (*dn) *p++ = *dn++;
				first = false;
			}
		}
		*p = '\0';
		reply_get(buf, tx);
		return;
	}

	int32_t vals[2];
	uint8_t n = parse_int_args(args, args_len, vals, 2);
	if (n >= 1 && vals[0] == 0) {
		memset(g_button_remap, 0, sizeof(g_button_remap));
	} else if (n >= 2) {
		uint8_t src = (uint8_t)vals[0];
		uint8_t dst = (uint8_t)vals[1];
		if (src >= 1 && src <= 5)
			g_button_remap[src - 1] = dst;
	}
	reply_set(tx);
}

// ---- Remap axis ----

static void handle_remap_axis(const char *args, uint16_t args_len,
                              macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[48];
		char *p = buf;
		const char *s = "invert_x=";
		while (*s) *p++ = *s++;
		*p++ = '0' + g_invert_x;
		*p++ = ',';
		s = "invert_y=";
		while (*s) *p++ = *s++;
		*p++ = '0' + g_invert_y;
		*p++ = ',';
		s = "swap_xy=";
		while (*s) *p++ = *s++;
		*p++ = '0' + g_swap_xy;
		*p = '\0';
		reply_get(buf, tx);
		return;
	}

	int32_t vals[3];
	uint8_t n = parse_int_args(args, args_len, vals, 3);
	if (n == 1 && vals[0] == 0) {
		g_invert_x = 0; g_invert_y = 0; g_swap_xy = 0;
	} else if (n >= 3) {
		g_invert_x = vals[0] ? 1 : 0;
		g_invert_y = vals[1] ? 1 : 0;
		g_swap_xy  = vals[2] ? 1 : 0;
	}
	reply_set(tx);
}

// ---- Invert / Swap ----

static void handle_flag(uint8_t *flag, const char *args, uint16_t args_len,
                        macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[4] = { (char)('0' + *flag), '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			*flag = vals[0] ? 1 : 0;
		reply_set(tx);
	}
}

// ---- Keyboard ----

static void handle_kb_down(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool needs_shift;
	int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
	if (key < 0) { reply_set(tx); return; }
	if (needs_shift) g_kb_modifier |= (1u << 1);
	makd_action_kb_down((uint8_t)key);
	reply_set(tx);
}

static void handle_kb_up(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool needs_shift;
	int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
	if (key < 0) { reply_set(tx); return; }
	if (needs_shift) g_kb_modifier &= ~(1u << 1);
	makd_action_kb_up((uint8_t)key);
	reply_set(tx);
}

static void handle_kb_press(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool needs_shift;
	int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
	if (key < 0) { reply_set(tx); return; }

	int32_t timing[2] = {0, 0};
	parse_int_args(&args[pos], args_len - pos, timing, 2);

	if (needs_shift) g_kb_modifier |= (1u << 1);

	uint32_t delay = (uint32_t)timing[0];
	if (timing[1] > 0) delay += millis() % (uint32_t)timing[1];
	if (delay == 0) delay = 75 + (millis() % 51);
	makd_action_kb_press((uint8_t)key, delay);
	reply_set(tx);
}

// ---- Multi-key variants (Ferrum: multidown, multiup, multipress) ----

static void handle_kb_multidown(const char *args, uint16_t args_len,
                                macku_tx_fn tx)
{
	uint16_t pos = 0;
	while (pos < args_len) {
		bool needs_shift;
		int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
		if (key < 0) break;
		if (needs_shift) g_kb_modifier |= (1u << 1);
		makd_action_kb_down((uint8_t)key);
	}
	reply_set(tx);
}

static void handle_kb_multiup(const char *args, uint16_t args_len,
                              macku_tx_fn tx)
{
	uint16_t pos = 0;
	while (pos < args_len) {
		bool needs_shift;
		int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
		if (key < 0) break;
		if (needs_shift) g_kb_modifier &= ~(1u << 1);
		makd_action_kb_up((uint8_t)key);
	}
	reply_set(tx);
}

static void handle_kb_multipress(const char *args, uint16_t args_len,
                                 macku_tx_fn tx)
{
	uint16_t pos = 0;
	while (pos < args_len) {
		bool needs_shift;
		int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
		if (key < 0) break;
		if (needs_shift) g_kb_modifier |= (1u << 1);
		makd_action_kb_press((uint8_t)key, 75 + (millis() % 51));
	}
	reply_set(tx);
}

// ---- String typing ----

static void handle_kb_string(const char *args, uint16_t args_len,
                             macku_tx_fn tx)
{
	uint16_t start = 0;
	while (start < args_len && args[start] != '"' && args[start] != '\'') start++;
	if (start >= args_len) { reply_set(tx); return; }

	char q = args[start++];
	uint16_t end = start;
	while (end < args_len && args[end] != q) end++;

	for (uint16_t i = start; i < end && i < start + 256; i++) {
		uint8_t c = (uint8_t)args[i];
		if (c < 32 || c > 126) continue;
		const ascii_hid_t *entry = &ascii_to_hid[c - 32];
		uint8_t saved_mod = g_kb_modifier;
		if (entry->shift) g_kb_modifier |= (1u << 1);
		makd_action_kb_press(entry->hid, 35 + (millis() % 41));
		g_kb_modifier = saved_mod;
	}
	reply_set(tx);
}

// ---- Keyboard init (Ferrum: clears all keyboard locks/masks) ----

static void handle_kb_init(macku_tx_fn tx)
{
	makd_action_kb_init();
	g_masked_count = 0;   // clear keyboard masks (Ferrum compat)
	g_disabled_count = 0; // clear disabled keys
	reply_set(tx);
}

// ---- isdown ----

static void handle_kb_isdown(const char *args, uint16_t args_len,
                             macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool needs_shift;
	int16_t key = parse_key_arg(args, args_len, &pos, &needs_shift);
	if (key < 0) { reply_get("0", tx); return; }
	char buf[4] = { (char)('0' + makd_action_kb_isdown((uint8_t)key)), '\0' };
	reply_get(buf, tx);
}

// ---- disable ----

static void handle_kb_disable(const char *args, uint16_t args_len,
                              macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[128];
		char *p = buf;
		for (uint8_t i = 0; i < g_disabled_count; i++) {
			if (i > 0) *p++ = ',';
			p = u32_to_str(p, g_disabled_keys[i]);
		}
		*p = '\0';
		reply_get(buf, tx);
		return;
	}

	int32_t vals[16];
	uint8_t n = parse_int_args(args, args_len, vals, 16);

	uint16_t pos = 0;
	bool has_quotes = false;
	for (uint16_t i = 0; i < args_len; i++) {
		if (args[i] == '\'' || args[i] == '"') { has_quotes = true; break; }
	}

	if (has_quotes) {
		while (pos < args_len) {
			bool ns;
			int16_t key = parse_key_arg(args, args_len, &pos, &ns);
			if (key < 0) break;
			uint16_t peek = pos;
			while (peek < args_len && (args[peek] == ' ' || args[peek] == ',')) peek++;
			if (peek < args_len && (args[peek] == '0' || args[peek] == '1') &&
			    (peek + 1 >= args_len || args[peek + 1] == ',' || args[peek + 1] == ' ')) {
				pos = peek + 1;
				makd_action_kb_disable_set((uint8_t)key, args[peek] - '0');
				break;
			}
			makd_action_kb_disable_set((uint8_t)key, 1);
		}
	} else if (n == 2 && (vals[1] == 0 || vals[1] == 1)) {
		makd_action_kb_disable_set((uint8_t)vals[0], (uint8_t)vals[1]);
	} else {
		for (uint8_t i = 0; i < n; i++)
			makd_action_kb_disable_set((uint8_t)vals[i], 1);
	}
	reply_set(tx);
}

// ---- mask (Ferrum: km.mask(key[, state])) ----
// One arg = GET state, two args = SET state

static void handle_kb_mask(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool ns;
	int16_t key = parse_key_arg(args, args_len, &pos, &ns);
	if (key < 0) { reply_set(tx); return; }

	uint8_t hid = (uint8_t)key;

	// Check if there's a second argument
	uint16_t peek = pos;
	while (peek < args_len && (args[peek] == ' ' || args[peek] == ',')) peek++;

	if (peek >= args_len) {
		// GET: return mask state for this key (Ferrum compat)
		uint8_t state = 0;
		for (uint8_t i = 0; i < g_masked_count; i++) {
			if (g_masked_keys[i] == hid) { state = 1; break; }
		}
		char buf[4] = { '0' + state, '\0' };
		reply_get(buf, tx);
		return;
	}

	int32_t mode_val[1];
	parse_int_args(&args[pos], args_len - pos, mode_val, 1);
	makd_action_kb_mask(hid, (uint8_t)mode_val[0]);
	reply_set(tx);
}

// ---- remap ----

static void handle_kb_remap(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	uint16_t pos = 0;
	bool ns1, ns2;
	int16_t src = parse_key_arg(args, args_len, &pos, &ns1);
	int16_t dst = parse_key_arg(args, args_len, &pos, &ns2);
	if (src < 0) { reply_set(tx); return; }
	makd_action_kb_remap((uint8_t)src, (dst < 0) ? 0 : (uint8_t)dst);
	reply_set(tx);
}

// ---- Streaming ----

static void handle_stream(uint8_t *mode, uint8_t *period,
                          const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[16];
		char *p = u32_to_str(buf, *mode);
		if (*period) {
			*p++ = ',';
			p = u32_to_str(p, *period);
		}
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[2];
		uint8_t n = parse_int_args(args, args_len, vals, 2);
		if (n >= 1) *mode = (uint8_t)vals[0];
		if (n >= 2) *period = (uint8_t)vals[1];
		reply_set(tx);
	}
}

// ---- System / Misc ----

static void handle_help(macku_tx_fn tx)
{
	reply_get("left,right,middle,side1,side2,click,turbo,"
	      "move,moveto,wheel,pan,tilt,getpos,silent,mo,"
	      "lock_*,catch_*,catch_xy,remap_button,remap_axis,invert_x,invert_y,swap_xy,"
	      "down,up,press,multidown,multiup,multipress,string,init,isdown,disable,mask,remap,"
	      "keyboard,keys,buttons,axis,axes,mouse,"
	      "help,info,version,device,fault,reboot,serial,log,echo,baud,"
	      "bypass,hs,led,release,screen", tx);
}

static void handle_info(macku_tx_fn tx)
{
	char buf[80];
	char *p = buf;
	const char *s = "fw=";
	while (*s) *p++ = *s++;
	s = g_identity_mode ? FW_VERSION_FERRUM : FW_VERSION_MACKU;
	while (*s) *p++ = *s++;
	*p++ = ',';
	s = "uptime=";
	while (*s) *p++ = *s++;
	p = u32_to_str(p, millis());
	*p++ = 'm'; *p++ = 's';
	*p = '\0';
	reply_get(buf, tx);
}

static void handle_version(macku_tx_fn tx)
{
	if (g_identity_mode == 0) {
		// MACKU: no echo line, just "km.MAKCU\r\n>>> "
		tx((const uint8_t *)FW_VERSION_MACKU "\r\n>>> ", 14);
	} else {
		reply_get(FW_VERSION_FERRUM, tx);
	}
}

static void handle_device(macku_tx_fn tx)
{
	reply_get("none", tx);
}

static void handle_fault(macku_tx_fn tx)
{
	char buf[64];
	char *p = buf;
	const char *s = "or=";
	while (*s) *p++ = *s++;
	p = u32_to_str(p, kmbox_uart_overrun());
	*p++ = ',';
	s = "fe=";
	while (*s) *p++ = *s++;
	p = u32_to_str(p, kmbox_uart_framing());
	*p++ = ',';
	s = "nf=";
	while (*s) *p++ = *s++;
	p = u32_to_str(p, kmbox_uart_noise());
	*p = '\0';
	reply_get(buf, tx);
}

static void handle_reboot(macku_tx_fn tx)
{
	reply_set(tx);
	for (volatile int i = 0; i < 100000; i++) {}
	SCB_AIRCR = 0x05FA0004;
}

static void handle_serial_cmd(const char *args, uint16_t args_len,
                               macku_tx_fn tx)
{
	if (args_len == 0) {
		reply_get("", tx);
	} else {
		reply_set(tx);
	}
}

static void handle_log(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[4] = { '0' + g_log_level, '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1) {
			g_log_level = (uint8_t)vals[0];
			if (g_log_level > 5) g_log_level = 5;
		}
		reply_set(tx);
	}
}

static void handle_echo_cmd(const char *args, uint16_t args_len,
                            macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[4] = { (char)('0' + g_echo), '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_echo = vals[0] ? 1 : 0;
		reply_set(tx);
	}
}

// km.baud() — return current baud
// km.baud(rate) — change baud at runtime (no output, baud switches immediately)
static void handle_baud(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[16];
		char *p = u32_to_str(buf, kmbox_current_baud());
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1 && vals[0] >= 9600) {
			// No output — baud changes immediately, response would be garbled
			kmbox_set_baud((uint32_t)vals[0]);
		}
	}
}

static void handle_bypass(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[4] = { (char)('0' + g_bypass_mode), '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_bypass_mode = (uint8_t)vals[0];
		reply_set(tx);
	}
}

static void handle_hs(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[4] = { (char)('0' + g_hs_mode), '\0' };
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_hs_mode = vals[0] ? 1 : 0;
		reply_set(tx);
	}
}

static void handle_led(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		reply_get("device,off", tx);
	} else {
		reply_set(tx);
	}
}

static void handle_release(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[16];
		char *p = u32_to_str(buf, g_release_timer_ms);
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[1];
		if (parse_int_args(args, args_len, vals, 1) >= 1)
			g_release_timer_ms = (uint32_t)vals[0];
		reply_set(tx);
	}
}

static void handle_screen(const char *args, uint16_t args_len, macku_tx_fn tx)
{
	if (args_len == 0) {
		char buf[24];
		char *p = i32_to_str(buf, g_screen_w);
		*p++ = ',';
		p = i32_to_str(p, g_screen_h);
		*p = '\0';
		reply_get(buf, tx);
	} else {
		int32_t vals[2];
		if (parse_int_args(args, args_len, vals, 2) >= 2) {
			g_screen_w = (int16_t)vals[0];
			g_screen_h = (int16_t)vals[1];
		}
		reply_set(tx);
	}
}

// ======== COMMAND DISPATCH TABLE ========

typedef void (*cmd_handler_fn)(const char *args, uint16_t args_len,
                               macku_tx_fn tx);

typedef struct {
	const char     *name;
	cmd_handler_fn  handler;
} cmd_entry_t;

// Forward declarations for dispatch wrappers
static void wrap_axis(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_axis_mode, &g_stream_axis_period, a, l, tx); }
static void wrap_axes(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_axis_mode, &g_stream_axis_period, a, l, tx); }
static void wrap_baud(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_baud(a, l, tx); }
static void wrap_buttons(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_buttons_mode, &g_stream_buttons_period, a, l, tx); }
static void wrap_bypass(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_bypass(a, l, tx); }
static void wrap_catch_xy(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_catch_xy(a, l, tx); }
static void wrap_click(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_click(a, l, tx); }
static void wrap_device(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_device(tx); }
static void wrap_disable(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_disable(a, l, tx); }
static void wrap_down(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_down(a, l, tx); }
static void wrap_echo(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_echo_cmd(a, l, tx); }
static void wrap_fault(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_fault(tx); }
static void wrap_getpos(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_getpos(tx); }
static void wrap_help(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_help(tx); }
static void wrap_hs(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_hs(a, l, tx); }
static void wrap_info(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_info(tx); }
static void wrap_init(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_kb_init(tx); }
static void wrap_invert_x(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_flag(&g_invert_x, a, l, tx); }
static void wrap_invert_y(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_flag(&g_invert_y, a, l, tx); }
static void wrap_isdown(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_isdown(a, l, tx); }
static void wrap_keyboard(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_kb_mode, &g_stream_kb_period, a, l, tx); }
static void wrap_keys(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_kb_mode, &g_stream_kb_period, a, l, tx); }
static void wrap_led(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_led(a, l, tx); }
static void wrap_left(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_button(a, l, tx); }
static void wrap_log(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_log(a, l, tx); }
static void wrap_mask(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_mask(a, l, tx); }
static void wrap_middle(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_button(a, l, tx); }
static void wrap_mo(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_mo(a, l, tx); }
static void wrap_mouse(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_stream(&g_stream_mouse_mode, &g_stream_mouse_period, a, l, tx); }
static void wrap_move(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_move(a, l, tx); }
static void wrap_moveto(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_moveto(a, l, tx); }
static void wrap_multidown(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_multidown(a, l, tx); }
static void wrap_multipress(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_multipress(a, l, tx); }
static void wrap_multiup(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_multiup(a, l, tx); }
static void wrap_pan(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_pan(a, l, tx); }
static void wrap_press(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_press(a, l, tx); }
static void wrap_reboot(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_reboot(tx); }
static void wrap_release(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_release(a, l, tx); }
static void wrap_remap(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_remap(a, l, tx); }
static void wrap_remap_axis(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_remap_axis(a, l, tx); }
static void wrap_remap_button(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_remap_button(a, l, tx); }
static void wrap_right(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_button(a, l, tx); }
static void wrap_screen(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_screen(a, l, tx); }
static void wrap_serial(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_serial_cmd(a, l, tx); }
static void wrap_side1(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_button(a, l, tx); }
static void wrap_side2(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_button(a, l, tx); }
static void wrap_silent(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_silent(a, l, tx); }
static void wrap_string(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_string(a, l, tx); }
static void wrap_swap_xy(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_flag(&g_swap_xy, a, l, tx); }
static void wrap_tilt(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_tilt(a, l, tx); }
static void wrap_turbo(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_turbo(a, l, tx); }
static void wrap_up(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_kb_up(a, l, tx); }
static void wrap_version(const char *a, uint16_t l, macku_tx_fn tx)
{  (void)a; (void)l; handle_version(tx); }
static void wrap_wheel(const char *a, uint16_t l, macku_tx_fn tx)
{  handle_wheel(a, l, tx); }

// Sorted alphabetically for binary search
static const cmd_entry_t cmd_table[] = {
	{"axes",          wrap_axes},
	{"axis",          wrap_axis},
	{"baud",          wrap_baud},
	{"buttons",       wrap_buttons},
	{"bypass",        wrap_bypass},
	{"catch_xy",      wrap_catch_xy},
	{"click",         wrap_click},
	{"device",        wrap_device},
	{"disable",       wrap_disable},
	{"down",          wrap_down},
	{"echo",          wrap_echo},
	{"fault",         wrap_fault},
	{"getpos",        wrap_getpos},
	{"help",          wrap_help},
	{"hs",            wrap_hs},
	{"info",          wrap_info},
	{"init",          wrap_init},
	{"invert_x",      wrap_invert_x},
	{"invert_y",      wrap_invert_y},
	{"isdown",        wrap_isdown},
	{"keyboard",      wrap_keyboard},
	{"keys",          wrap_keys},
	{"led",           wrap_led},
	{"left",          wrap_left},
	{"log",           wrap_log},
	{"m",             wrap_move},
	{"mask",          wrap_mask},
	{"middle",        wrap_middle},
	{"mo",            wrap_mo},
	{"mouse",         wrap_mouse},
	{"move",          wrap_move},
	{"moveto",        wrap_moveto},
	{"multidown",     wrap_multidown},
	{"multipress",    wrap_multipress},
	{"multiup",       wrap_multiup},
	{"pan",           wrap_pan},
	{"press",         wrap_press},
	{"reboot",        wrap_reboot},
	{"release",       wrap_release},
	{"remap",         wrap_remap},
	{"remap_axis",    wrap_remap_axis},
	{"remap_button",  wrap_remap_button},
	{"right",         wrap_right},
	{"screen",        wrap_screen},
	{"serial",        wrap_serial},
	{"side1",         wrap_side1},
	{"side2",         wrap_side2},
	{"silent",        wrap_silent},
	{"string",        wrap_string},
	{"swap_xy",       wrap_swap_xy},
	{"tilt",          wrap_tilt},
	{"turbo",         wrap_turbo},
	{"up",            wrap_up},
	{"version",       wrap_version},
	{"wheel",         wrap_wheel},
};
#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))

void macku_init(void)
{
	// State is shared with makd.c — initialized by makd_init()
}

void macku_dispatch(const char *cmd, uint8_t cmd_len,
                    const char *args, uint16_t args_len, macku_tx_fn tx)
{
	(void)cmd_len;

	// Set dispatch context for reply_get/reply_set auto-echoing
	cur_cmd = cmd;
	cur_args = args;

	// Check lock_ prefix
	if (cmd[0] == 'l' && cmd[1] == 'o' && cmd[2] == 'c' && cmd[3] == 'k' && cmd[4] == '_') {
		handle_lock(&cmd[5], args, args_len, tx);
		return;
	}

	// Check catch_ prefix (but not catch_xy which is in the table)
	if (cmd[0] == 'c' && cmd[1] == 'a' && cmd[2] == 't' && cmd[3] == 'c' &&
	    cmd[4] == 'h' && cmd[5] == '_') {
		// catch_xy goes through the table
		if (cmd[6] != 'x') {
			handle_catch(&cmd[6], args, args_len, tx);
			return;
		}
	}

	// Binary search the command table
	int lo = 0, hi = (int)CMD_TABLE_SIZE - 1;
	while (lo <= hi) {
		int mid = (lo + hi) / 2;
		int cmp = key_cmp(cmd, cmd_table[mid].name);
		if (cmp == 0) {
			cmd_table[mid].handler(args, args_len, tx);
			return;
		}
		if (cmp < 0) hi = mid - 1;
		else lo = mid + 1;
	}

	// Unknown command
	reply_get("ERR:unknown", tx);
}

void macku_dispatch_bin(const uint8_t *payload, uint16_t len, macku_tx_fn tx)
{
	// DE AD binary frames — set context for responses
	cur_cmd = "baud";
	cur_args = "";

	if (len < 1) return;
	uint8_t cmd = payload[0];

	if (cmd == 0xA5 && len >= 5) {
		// Baud rate change: DE AD [len_lo len_hi] [A5] [baud LE u32]
		uint32_t rate = (uint32_t)payload[1]
		              | ((uint32_t)payload[2] << 8)
		              | ((uint32_t)payload[3] << 16)
		              | ((uint32_t)payload[4] << 24);
		if (rate >= 9600)
			kmbox_set_baud(rate);
	}
}
