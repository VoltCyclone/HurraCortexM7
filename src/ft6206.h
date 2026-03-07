// ft6206.h — FT6206 capacitive touch controller driver
// Bare-metal LPI2C1 on i.MX RT1062 (Teensy 4.1 pins 18/19)
// Only compiled when TOUCH_ENABLED == 1

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint16_t x;
	uint16_t y;
	bool     valid;
} touch_point_t;

// Initialize LPI2C1 and FT6206 (pin mux, clock, threshold).
// Returns true if FT6206 responds on the bus.
bool ft6206_init(void);

// Poll for touch events. Call from main loop.
// Returns true if a touch event occurred (press or release).
bool ft6206_poll(touch_point_t *pt);

// Check if screen is currently being touched.
bool ft6206_is_touched(void);

// Read raw touch point (no debounce). Returns valid=false if not touched.
touch_point_t ft6206_read(void);
