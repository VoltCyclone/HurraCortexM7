#pragma once
#include <stdint.h>
#include <stdbool.h>

// MAKCU text protocol parser (matches makcu-rs wire format)
// Commands: km.move(dx,dy)\r\n  km.left(1)\r\n  km.wheel(d)\r\n  etc.
// Tracked: km.move(dx,dy)#42\r\n  →  response includes #42
// Button state broadcasts: single byte < 32 with bitmask
// Response: >>>\r\n  or  >>> OK#id\r\n  for tracked commands

// Result of parsing a MAKCU text line
typedef struct {
	int16_t  mouse_dx;
	int16_t  mouse_dy;
	uint8_t  mouse_buttons;  // full button mask after applying command
	int8_t   mouse_wheel;
	bool     has_mouse;
	bool     click_release;  // true = schedule button release after click

	uint8_t  kb_modifier;
	uint8_t  kb_keys[6];
	bool     has_keyboard;
	bool     kb_click_release;
	uint8_t  kb_release_key;

	bool     needs_response;
	// Optional text payload before >>> (for version, info, etc.)
	const char *text_response; // NULL = no extra text before >>>

	// Tracked command ID (0 = untracked)
	uint32_t track_id;
} makcu_result_t;

void makcu_init(void);

// Parse a complete text line (without trailing \r\n).
// Returns true if the line was a recognized MAKCU command.
bool makcu_parse_line(const char *line, uint8_t len, makcu_result_t *out);
