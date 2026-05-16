// src/actions.c — transport-agnostic injection actions
//
// These functions drive kmbox_inject_mouse / kmbox_inject_keyboard and own
// the mouse/keyboard state.  They have no knowledge of any wire protocol;
// Protocol handlers (ferrum.c) call them by name. These are transport-agnostic —
// they own the injection state and call kmbox_inject_* / smooth_inject.

#include "actions.h"
#include "kmbox.h"
#include "smooth.h"
#include <string.h>

extern uint32_t millis(void);

// Mouse + keyboard state
uint8_t  g_buttons;
uint8_t  g_kb_modifier;
uint8_t  g_kb_keys[6];
int32_t  g_pos_x, g_pos_y;
uint16_t g_lock_mask;

typedef struct {
	uint8_t  button;
	uint8_t  remaining;
	uint32_t delay_ms;
	uint32_t next_at;
	bool     pressed;
} act_click_sched_t;

static act_click_sched_t g_click_sched;
static uint8_t  g_masked_keys[ACT_MAX_DISABLED_KEYS];
static uint8_t  g_masked_modes[ACT_MAX_DISABLED_KEYS];
static uint8_t  g_masked_count;

static uint8_t btn_idx_to_mask(uint8_t idx)
{
	if (idx >= 1 && idx <= 5)
		return 1u << (idx - 1);
	return 0;
}

void act_init(void)
{
	g_buttons = 0;
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	g_pos_x = 0;
	g_pos_y = 0;
	g_lock_mask = 0;
	memset(&g_click_sched, 0, sizeof(g_click_sched));
	g_masked_count = 0;
}

int8_t act_button_set(uint8_t mask, uint8_t action)
{
	if (action == 0) {
		g_buttons &= ~mask;
		kmbox_inject_mouse(0, 0, g_buttons, 0, false);
	} else if (action == 1) {
		g_buttons |= mask;
		kmbox_inject_mouse(0, 0, g_buttons, 0, false);
	} else if (action == 2) {
		g_buttons &= ~mask;
	} else {
		return -1;
	}
	return 0;
}

void act_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms)
{
	uint8_t mask = btn_idx_to_mask(button_1based);
	g_buttons |= mask;
	kmbox_inject_mouse(0, 0, g_buttons, 0, false);

	if (count == 1) {
		kmbox_schedule_click_release(mask, delay_ms);
	} else {
		g_click_sched.button = button_1based;
		g_click_sched.remaining = count - 1;
		g_click_sched.delay_ms = delay_ms;
		g_click_sched.next_at = millis() + delay_ms;
		g_click_sched.pressed = true;
	}
}

void act_move(int16_t dx, int16_t dy, bool smooth)
{
	g_pos_x += dx;
	g_pos_y += dy;
	if (smooth) smooth_inject(dx, dy);
	else        kmbox_inject_mouse(dx, dy, g_buttons, 0, false);
}

int8_t act_kb_down(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier |= (1u << (key - 0xE0));
	} else {
		bool added = false;
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) { added = true; break; }
			if (g_kb_keys[i] == 0) { g_kb_keys[i] = key; added = true; break; }
		}
		if (!added) return -1;
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
	return 0;
}

void act_kb_up(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier &= ~(1u << (key - 0xE0));
	} else {
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) { g_kb_keys[i] = 0; break; }
		}
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
}

void act_kb_press(uint8_t key, uint32_t delay_ms)
{
	if (key >= 0xE0 && key <= 0xE7) {
		g_kb_modifier |= (1u << (key - 0xE0));
	} else {
		for (int i = 0; i < 6; i++) {
			if (g_kb_keys[i] == key) break;
			if (g_kb_keys[i] == 0) { g_kb_keys[i] = key; break; }
		}
	}
	kmbox_inject_keyboard(g_kb_modifier, g_kb_keys);
	kmbox_schedule_kb_release(key, delay_ms);
}

uint8_t act_kb_isdown(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7)
		return (g_kb_modifier & (1u << (key - 0xE0))) ? 1 : 0;
	for (int i = 0; i < 6; i++)
		if (g_kb_keys[i] == key) return 1;
	return 0;
}

void act_kb_init(void)
{
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	kmbox_inject_keyboard(0, g_kb_keys);
}

void act_kb_mask(uint8_t key, uint8_t mode)
{
	if (mode == 0) {
		for (uint8_t i = 0; i < g_masked_count; i++) {
			if (g_masked_keys[i] == key) {
				g_masked_keys[i] = g_masked_keys[--g_masked_count];
				g_masked_modes[i] = g_masked_modes[g_masked_count];
				break;
			}
		}
	} else {
		for (uint8_t i = 0; i < g_masked_count; i++) {
			if (g_masked_keys[i] == key) {
				g_masked_modes[i] = mode;
				return;
			}
		}
		if (g_masked_count < ACT_MAX_DISABLED_KEYS) {
			g_masked_keys[g_masked_count] = key;
			g_masked_modes[g_masked_count] = mode;
			g_masked_count++;
		}
	}
}
