#pragma once
#include <stdint.h>
#include <stdbool.h>

// Ferrum KM API text protocol parser (matches FerrumAPI.py wire format)
// Wire: km.mouse_move(dx,dy)\r  km.mouse_button_press(code)\r  etc.
// Button codes are bitmasks: 1=left, 2=right, 4=middle, 8=side1, 16=side2
// Response: >>>\r\n
// Version: responds with "Ferrum" identifier for device detection

#define FERRUM_MAX_LINE 128

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
	const char *text_response; // NULL = no extra text before >>>
} ferrum_result_t;

void ferrum_init(void);

// Parse a complete text line (without trailing \r or \r\n).
// Returns true if the line was a recognized Ferrum command.
bool ferrum_parse_line(const char *line, uint8_t len, ferrum_result_t *out);
