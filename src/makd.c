// MAKD protocol dispatch

#include "makd.h"
#include "kmbox.h"
#include "smooth.h"
#include "imxrt.h"
#include <string.h>

extern uint32_t millis(void);

#define MAKD_FW_VERSION "hurra-cortex-m7-1.0"

uint8_t  g_buttons;
uint8_t  g_kb_modifier;
uint8_t  g_kb_keys[6];
int32_t  g_pos_x, g_pos_y;
uint8_t  g_invert_x, g_invert_y, g_swap_xy;
uint8_t  g_catch_mode;
uint16_t g_lock_mask;
uint16_t g_turbo_delay[5];
click_sched_t g_click_sched;
uint8_t  g_button_remap[5];
uint8_t  g_disabled_keys[MAKD_MAX_DISABLED_KEYS];
uint8_t  g_disabled_count;
uint8_t  g_masked_keys[MAKD_MAX_DISABLED_KEYS];
uint8_t  g_masked_modes[MAKD_MAX_DISABLED_KEYS];
uint8_t  g_masked_count;
uint8_t  g_key_remap_src[MAKD_MAX_KEY_REMAPS];
uint8_t  g_key_remap_dst[MAKD_MAX_KEY_REMAPS];
uint8_t  g_key_remap_count;
uint8_t  g_stream_axis_mode, g_stream_axis_period;
uint8_t  g_stream_buttons_mode, g_stream_buttons_period;
uint8_t  g_stream_mouse_mode, g_stream_mouse_period;
uint8_t  g_stream_kb_mode, g_stream_kb_period;
int16_t  g_screen_w, g_screen_h;
uint8_t  g_echo;
uint8_t  g_identity_mode;   // 0=MACKU, 1=Ferrum
uint8_t  g_log_level;
uint32_t g_release_timer_ms;
uint8_t  g_bypass_mode;
uint8_t  g_hs_mode;
int8_t   g_pending_pan, g_pending_tilt;

static inline uint16_t rd_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static inline int16_t rd_i16(const uint8_t *p)
{
	return (int16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void wr_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

static inline void wr_i16(uint8_t *p, int16_t v)
{
	wr_u16(p, (uint16_t)v);
}

static inline void wr_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v);
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static void send_response(makd_cmd_t cmd, const uint8_t *payload, uint16_t len,
                          makd_tx_fn tx)
{
	uint8_t hdr[MAKD_HEADER_SIZE];
	hdr[0] = MAKD_SYNC;
	hdr[1] = cmd;
	hdr[2] = (uint8_t)(len & 0xFF);
	hdr[3] = (uint8_t)(len >> 8);
	tx(hdr, MAKD_HEADER_SIZE);
	if (len > 0 && payload)
		tx(payload, len);
}

static void send_ok(makd_cmd_t cmd, makd_tx_fn tx)
{
	uint8_t status = MAKD_OK;
	send_response(cmd, &status, 1, tx);
}

static void send_err(makd_cmd_t cmd, makd_tx_fn tx)
{
	uint8_t status = MAKD_ERR;
	send_response(cmd, &status, 1, tx);
}

static uint8_t btn_idx_to_mask(uint8_t idx)
{
	if (idx >= 1 && idx <= 5)
		return 1u << (idx - 1);
	return 0;
}

static uint8_t cmd_to_btn_mask(makd_cmd_t cmd)
{
	switch (cmd) {
	case MAKD_CMD_MOUSE_LEFT:   return 0x01;
	case MAKD_CMD_MOUSE_RIGHT:  return 0x02;
	case MAKD_CMD_MOUSE_MIDDLE: return 0x04;
	case MAKD_CMD_MOUSE_SIDE1:  return 0x08;
	case MAKD_CMD_MOUSE_SIDE2:  return 0x10;
	default: return 0;
	}
}

static void handle_button(makd_cmd_t cmd, const uint8_t *payload, uint16_t len,
                          makd_tx_fn tx)
{
	uint8_t mask = cmd_to_btn_mask(cmd);
	if (!mask) { send_err(cmd, tx); return; }

	if (len == 0) {
		uint8_t state = (g_buttons & mask) ? 2 : 0;
		send_response(cmd, &state, 1, tx);
	} else {
		if (makd_action_button_set(mask, payload[0]) < 0) {
			send_err(cmd, tx); return;
		}
		send_ok(cmd, tx);
	}
}

static void handle_click(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 3) { send_err(MAKD_CMD_CLICK, tx); return; }
	uint8_t button = payload[0];
	uint8_t count = payload[1];
	uint8_t delay = payload[2];

	if (button < 1 || button > 5 || count == 0) {
		send_err(MAKD_CMD_CLICK, tx); return;
	}

	uint32_t d = delay ? delay : (35 + (millis() % 41));
	makd_action_click(button, count, d);
	send_ok(MAKD_CMD_CLICK, tx);
}

static void handle_turbo(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[10];
		for (int i = 0; i < 5; i++)
			wr_u16(&resp[i * 2], g_turbo_delay[i]);
		send_response(MAKD_CMD_TURBO, resp, 10, tx);
	} else if (len >= 1) {
		uint8_t button = payload[0];
		if (button == 0) {
			memset(g_turbo_delay, 0, sizeof(g_turbo_delay));
			send_ok(MAKD_CMD_TURBO, tx);
		} else if (button >= 1 && button <= 5) {
			uint16_t delay = 0;
			if (len >= 3)
				delay = rd_u16(&payload[1]);
			else
				delay = 0;
			g_turbo_delay[button - 1] = delay ? delay : 1;
			send_ok(MAKD_CMD_TURBO, tx);
		} else {
			send_err(MAKD_CMD_TURBO, tx);
		}
	}
}

static void handle_move(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 4) { send_err(MAKD_CMD_MOVE, tx); return; }
	makd_action_move(rd_i16(&payload[0]), rd_i16(&payload[2]),
	                 len >= 5 && payload[4] > 0);
	send_ok(MAKD_CMD_MOVE, tx);
}

static void handle_moveto(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 4) { send_err(MAKD_CMD_MOVETO, tx); return; }
	makd_action_moveto(rd_i16(&payload[0]), rd_i16(&payload[2]),
	                   len >= 5 && payload[4] > 0);
	send_ok(MAKD_CMD_MOVETO, tx);
}

static void handle_wheel(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 1) { send_err(MAKD_CMD_WHEEL, tx); return; }
	int8_t delta = (int8_t)payload[0];
	kmbox_inject_mouse(0, 0, g_buttons, delta, false);
	send_ok(MAKD_CMD_WHEEL, tx);
}

static void handle_pan(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_PAN, (const uint8_t *)&g_pending_pan, 1, tx);
	} else {
		g_pending_pan = (int8_t)payload[0];
		send_ok(MAKD_CMD_PAN, tx);
	}
}

static void handle_tilt(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_TILT, (const uint8_t *)&g_pending_tilt, 1, tx);
	} else {
		g_pending_tilt = (int8_t)payload[0];
		send_ok(MAKD_CMD_TILT, tx);
	}
}

static void handle_silent(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 4) { send_err(MAKD_CMD_SILENT, tx); return; }
	makd_action_silent(rd_i16(&payload[0]), rd_i16(&payload[2]));
	send_ok(MAKD_CMD_SILENT, tx);
}

static void handle_mo(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 8) { send_err(MAKD_CMD_MO, tx); return; }
	makd_action_mo(payload[0], rd_i16(&payload[1]), rd_i16(&payload[3]),
	               (int8_t)payload[5], (int8_t)payload[6], (int8_t)payload[7]);
	send_ok(MAKD_CMD_MO, tx);
}

static void handle_catch(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_CATCH, &g_catch_mode, 1, tx);
	} else {
		g_catch_mode = payload[0];
		send_ok(MAKD_CMD_CATCH, tx);
	}
}

static void handle_getpos(makd_tx_fn tx)
{
	uint8_t resp[4];
	wr_i16(&resp[0], (int16_t)g_pos_x);
	wr_i16(&resp[2], (int16_t)g_pos_y);
	send_response(MAKD_CMD_GETPOS, resp, 4, tx);
}

static void handle_invert(makd_cmd_t cmd, uint8_t *flag,
                          const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(cmd, flag, 1, tx);
	} else {
		*flag = payload[0] ? 1 : 0;
		send_ok(cmd, tx);
	}
}

static void handle_swap_xy(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_SWAP_XY, &g_swap_xy, 1, tx);
	} else {
		g_swap_xy = payload[0] ? 1 : 0;
		send_ok(MAKD_CMD_SWAP_XY, tx);
	}
}

static void handle_lock(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[2];
		wr_u16(resp, g_lock_mask);
		send_response(MAKD_CMD_LOCK, resp, 2, tx);
	} else if (len >= 2) {
		uint8_t target = payload[0];
		uint8_t state = payload[1];
		if (target >= MAKD_LOCK_TARGET_COUNT) {
			send_err(MAKD_CMD_LOCK, tx); return;
		}
		if (state)
			g_lock_mask |= (1u << target);
		else
			g_lock_mask &= ~(1u << target);
		send_ok(MAKD_CMD_LOCK, tx);
	} else {
		uint8_t target = payload[0];
		if (target >= MAKD_LOCK_TARGET_COUNT) {
			send_err(MAKD_CMD_LOCK, tx); return;
		}
		uint8_t locked = (g_lock_mask & (1u << target)) ? 1 : 0;
		send_response(MAKD_CMD_LOCK, &locked, 1, tx);
	}
}

static void handle_remap_button(const uint8_t *payload, uint16_t len,
                                makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[10];
		for (int i = 0; i < 5; i++) {
			resp[i * 2] = (uint8_t)(i + 1);
			resp[i * 2 + 1] = g_button_remap[i] ? g_button_remap[i] : (uint8_t)(i + 1);
		}
		send_response(MAKD_CMD_REMAP_BUTTON, resp, 10, tx);
	} else if (len >= 1 && payload[0] == 0) {
		memset(g_button_remap, 0, sizeof(g_button_remap));
		send_ok(MAKD_CMD_REMAP_BUTTON, tx);
	} else if (len >= 2) {
		uint8_t src = payload[0];
		uint8_t dst = payload[1];
		if (src < 1 || src > 5) { send_err(MAKD_CMD_REMAP_BUTTON, tx); return; }
		g_button_remap[src - 1] = dst;
		send_ok(MAKD_CMD_REMAP_BUTTON, tx);
	} else {
		send_err(MAKD_CMD_REMAP_BUTTON, tx);
	}
}

static void handle_remap_axis(const uint8_t *payload, uint16_t len,
                              makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[3] = { g_invert_x, g_invert_y, g_swap_xy };
		send_response(MAKD_CMD_REMAP_AXIS, resp, 3, tx);
	} else if (len == 1 && payload[0] == 0) {
		g_invert_x = 0;
		g_invert_y = 0;
		g_swap_xy = 0;
		send_ok(MAKD_CMD_REMAP_AXIS, tx);
	} else if (len >= 3) {
		g_invert_x = payload[0] ? 1 : 0;
		g_invert_y = payload[1] ? 1 : 0;
		g_swap_xy = payload[2] ? 1 : 0;
		send_ok(MAKD_CMD_REMAP_AXIS, tx);
	} else {
		send_err(MAKD_CMD_REMAP_AXIS, tx);
	}
}

static void handle_kb_down(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 1) { send_err(MAKD_CMD_KB_DOWN, tx); return; }
	if (makd_action_kb_down(payload[0]) < 0) {
		send_err(MAKD_CMD_KB_DOWN, tx); return;
	}
	send_ok(MAKD_CMD_KB_DOWN, tx);
}

static void handle_kb_up(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 1) { send_err(MAKD_CMD_KB_UP, tx); return; }
	makd_action_kb_up(payload[0]);
	send_ok(MAKD_CMD_KB_UP, tx);
}

static void handle_kb_press(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 1) { send_err(MAKD_CMD_KB_PRESS, tx); return; }
	uint32_t delay = (len >= 2) ? payload[1] : 0;
	if (delay == 0) delay = 35 + (millis() % 51);
	makd_action_kb_press(payload[0], delay);
	send_ok(MAKD_CMD_KB_PRESS, tx);
}

static void handle_kb_init(makd_tx_fn tx)
{
	makd_action_kb_init();
	send_ok(MAKD_CMD_KB_INIT, tx);
}

static void handle_kb_isdown(const uint8_t *payload, uint16_t len,
                             makd_tx_fn tx)
{
	if (len < 1) { send_err(MAKD_CMD_KB_ISDOWN, tx); return; }
	uint8_t is_down = makd_action_kb_isdown(payload[0]);
	send_response(MAKD_CMD_KB_ISDOWN, &is_down, 1, tx);
}

static void handle_kb_disable(const uint8_t *payload, uint16_t len,
                              makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_KB_DISABLE, g_disabled_keys, g_disabled_count, tx);
	} else if (len == 2) {
		makd_action_kb_disable_set(payload[0], payload[1]);
		send_ok(MAKD_CMD_KB_DISABLE, tx);
	} else {
		for (uint16_t i = 0; i < len; i++)
			makd_action_kb_disable_set(payload[i], 1);
		send_ok(MAKD_CMD_KB_DISABLE, tx);
	}
}

static void handle_kb_mask(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 2) { send_err(MAKD_CMD_KB_MASK, tx); return; }
	makd_action_kb_mask(payload[0], payload[1]);
	send_ok(MAKD_CMD_KB_MASK, tx);
}

static void handle_kb_remap(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len < 2) { send_err(MAKD_CMD_KB_REMAP, tx); return; }
	makd_action_kb_remap(payload[0], payload[1]);
	send_ok(MAKD_CMD_KB_REMAP, tx);
}

static void handle_kb_string(const uint8_t *payload, uint16_t len,
                             makd_tx_fn tx)
{
	// TODO: ASCII->HID scancode table
	(void)payload;
	(void)len;
	send_ok(MAKD_CMD_KB_STRING, tx);
}

static void handle_stream(makd_cmd_t cmd, uint8_t *mode, uint8_t *period,
                          const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[2] = { *mode, *period };
		send_response(cmd, resp, 2, tx);
	} else {
		*mode = payload[0];
		if (len >= 2)
			*period = payload[1];
		send_ok(cmd, tx);
	}
}

static void handle_version(makd_tx_fn tx)
{
	const char *ver = MAKD_FW_VERSION;
	uint16_t vlen = 0;
	while (ver[vlen]) vlen++;
	send_response(MAKD_CMD_VERSION, (const uint8_t *)ver, vlen, tx);
}

static void handle_echo(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_ECHO, &g_echo, 1, tx);
	} else {
		g_echo = payload[0] ? 1 : 0;
		send_ok(MAKD_CMD_ECHO, tx);
	}
}

static void handle_reboot(makd_tx_fn tx)
{
	send_ok(MAKD_CMD_REBOOT, tx);
	for (volatile int i = 0; i < 100000; i++) {}
	SCB_AIRCR = 0x05FA0004;
}

static void handle_info(makd_tx_fn tx)
{
	uint8_t resp[64];
	uint16_t pos = 0;

	uint8_t field_count = 0;
	resp[pos++] = 0;

	const char *ver = MAKD_FW_VERSION;
	uint8_t vlen = 0;
	while (ver[vlen]) vlen++;
	if (pos + 2 + vlen <= sizeof(resp)) {
		resp[pos++] = 0x01;
		resp[pos++] = vlen;
		memcpy(&resp[pos], ver, vlen);
		pos += vlen;
		field_count++;
	}

	if (pos + 6 <= sizeof(resp)) {
		resp[pos++] = 0x02;
		resp[pos++] = 4;
		wr_u32(&resp[pos], millis());
		pos += 4;
		field_count++;
	}

	resp[0] = field_count;
	send_response(MAKD_CMD_INFO, resp, pos, tx);
}

static void handle_baud(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[4];
		wr_u32(resp, kmbox_current_baud());
		send_response(MAKD_CMD_BAUD, resp, 4, tx);
	} else if (len >= 4) {
		uint32_t rate = rd_u32(payload);
		if (rate >= 9600 && rate <= 6000000) {
			send_ok(MAKD_CMD_BAUD, tx);
			kmbox_set_baud(rate);
		} else {
			send_err(MAKD_CMD_BAUD, tx);
		}
	} else {
		send_err(MAKD_CMD_BAUD, tx);
	}
}

static void handle_device(makd_tx_fn tx)
{
	// TODO: query from desc_capture
	uint8_t type = 0;
	send_response(MAKD_CMD_DEVICE, &type, 1, tx);
}

static void handle_bypass(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_BYPASS, &g_bypass_mode, 1, tx);
	} else {
		g_bypass_mode = payload[0];
		send_ok(MAKD_CMD_BYPASS, tx);
	}
}

static void handle_fault(makd_tx_fn tx)
{
	uint8_t resp[12];
	wr_u32(&resp[0], kmbox_uart_overrun());
	wr_u32(&resp[4], kmbox_uart_framing());
	wr_u32(&resp[8], kmbox_uart_noise());
	send_response(MAKD_CMD_FAULT, resp, 12, tx);
}

static void handle_hs(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_HS, &g_hs_mode, 1, tx);
	} else {
		g_hs_mode = payload[0] ? 1 : 0;
		send_ok(MAKD_CMD_HS, tx);
	}
}

static void handle_led(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[2] = { 1, 0 };
		send_response(MAKD_CMD_LED, resp, 2, tx);
	} else if (len >= 2) {
		send_ok(MAKD_CMD_LED, tx);
	} else {
		send_err(MAKD_CMD_LED, tx);
	}
}

static void handle_log(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_LOG, &g_log_level, 1, tx);
	} else {
		g_log_level = payload[0];
		if (g_log_level > 5) g_log_level = 5;
		send_ok(MAKD_CMD_LOG, tx);
	}
}

static void handle_release(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[4];
		wr_u32(resp, g_release_timer_ms);
		send_response(MAKD_CMD_RELEASE, resp, 4, tx);
	} else if (len >= 4) {
		g_release_timer_ms = rd_u32(payload);
		send_ok(MAKD_CMD_RELEASE, tx);
	} else {
		send_err(MAKD_CMD_RELEASE, tx);
	}
}

static void handle_screen(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		uint8_t resp[4];
		wr_i16(&resp[0], g_screen_w);
		wr_i16(&resp[2], g_screen_h);
		send_response(MAKD_CMD_SCREEN, resp, 4, tx);
	} else if (len >= 4) {
		g_screen_w = rd_i16(&payload[0]);
		g_screen_h = rd_i16(&payload[2]);
		send_ok(MAKD_CMD_SCREEN, tx);
	} else {
		send_err(MAKD_CMD_SCREEN, tx);
	}
}

static void handle_serial(const uint8_t *payload, uint16_t len, makd_tx_fn tx)
{
	if (len == 0) {
		send_response(MAKD_CMD_SERIAL, NULL, 0, tx);
	} else if (len == 1 && payload[0] == 0) {
		send_ok(MAKD_CMD_SERIAL, tx);
	} else {
		send_ok(MAKD_CMD_SERIAL, tx);
	}
}

// ---- Shared action functions (transport-agnostic) ----

int8_t makd_action_button_set(uint8_t mask, uint8_t action)
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

void makd_action_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms)
{
	uint8_t mask = btn_idx_to_mask(button_1based);
	g_buttons |= mask;
	kmbox_inject_mouse(0, 0, g_buttons, 0, false);

	if (count == 1) {
		kmbox_schedule_click_release(mask, delay_ms);
	} else {
		g_click_sched.button = button_1based;
		g_click_sched.remaining = count - 1;
		g_click_sched.delay_ms = (uint8_t)delay_ms;
		g_click_sched.next_at = millis() + delay_ms;
		g_click_sched.pressed = true;
	}
}

void makd_action_move(int16_t dx, int16_t dy, bool smooth)
{
	if (g_invert_x) dx = -dx;
	if (g_invert_y) dy = -dy;
	if (g_swap_xy) { int16_t t = dx; dx = dy; dy = t; }
	g_pos_x += dx;
	g_pos_y += dy;
	if (smooth) smooth_inject(dx, dy);
	else kmbox_inject_mouse(dx, dy, g_buttons, 0, false);
}

void makd_action_moveto(int16_t x, int16_t y, bool smooth)
{
	int16_t dx = x - (int16_t)g_pos_x;
	int16_t dy = y - (int16_t)g_pos_y;
	if (g_invert_x) dx = -dx;
	if (g_invert_y) dy = -dy;
	if (g_swap_xy) { int16_t t = dx; dx = dy; dy = t; }
	g_pos_x = x;
	g_pos_y = y;
	if (smooth) smooth_inject(dx, dy);
	else kmbox_inject_mouse(dx, dy, g_buttons, 0, false);
}

void makd_action_silent(int16_t x, int16_t y)
{
	kmbox_inject_mouse(x, y, g_buttons | 0x01, 0, false);
	g_buttons &= ~0x01;
}

void makd_action_mo(uint8_t buttons, int16_t x, int16_t y,
                    int8_t wheel, int8_t pan, int8_t tilt)
{
	g_buttons = buttons;
	g_pending_pan = pan;
	g_pending_tilt = tilt;
	kmbox_inject_mouse(x, y, buttons, wheel, false);
}

int8_t makd_action_kb_down(uint8_t key)
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

void makd_action_kb_up(uint8_t key)
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

void makd_action_kb_press(uint8_t key, uint32_t delay_ms)
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

uint8_t makd_action_kb_isdown(uint8_t key)
{
	if (key >= 0xE0 && key <= 0xE7)
		return (g_kb_modifier & (1u << (key - 0xE0))) ? 1 : 0;
	for (int i = 0; i < 6; i++)
		if (g_kb_keys[i] == key) return 1;
	return 0;
}

void makd_action_kb_init(void)
{
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	kmbox_inject_keyboard(0, g_kb_keys);
}

void makd_action_kb_disable_set(uint8_t key, uint8_t enable)
{
	if (enable) {
		if (g_disabled_count < MAKD_MAX_DISABLED_KEYS) {
			bool found = false;
			for (uint8_t i = 0; i < g_disabled_count; i++)
				if (g_disabled_keys[i] == key) { found = true; break; }
			if (!found)
				g_disabled_keys[g_disabled_count++] = key;
		}
	} else {
		for (uint8_t i = 0; i < g_disabled_count; i++) {
			if (g_disabled_keys[i] == key) {
				g_disabled_keys[i] = g_disabled_keys[--g_disabled_count];
				break;
			}
		}
	}
}

void makd_action_kb_mask(uint8_t key, uint8_t mode)
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
		if (g_masked_count < MAKD_MAX_DISABLED_KEYS) {
			g_masked_keys[g_masked_count] = key;
			g_masked_modes[g_masked_count] = mode;
			g_masked_count++;
		}
	}
}

void makd_action_kb_remap(uint8_t src, uint8_t dst)
{
	if (dst == 0) {
		for (uint8_t i = 0; i < g_key_remap_count; i++) {
			if (g_key_remap_src[i] == src) {
				g_key_remap_src[i] = g_key_remap_src[--g_key_remap_count];
				g_key_remap_dst[i] = g_key_remap_dst[g_key_remap_count];
				break;
			}
		}
	} else {
		for (uint8_t i = 0; i < g_key_remap_count; i++) {
			if (g_key_remap_src[i] == src) {
				g_key_remap_dst[i] = dst;
				return;
			}
		}
		if (g_key_remap_count < MAKD_MAX_KEY_REMAPS) {
			g_key_remap_src[g_key_remap_count] = src;
			g_key_remap_dst[g_key_remap_count] = dst;
			g_key_remap_count++;
		}
	}
}

void makd_init(void)
{
	g_buttons = 0;
	g_kb_modifier = 0;
	memset(g_kb_keys, 0, sizeof(g_kb_keys));
	g_pos_x = 0;
	g_pos_y = 0;
	g_invert_x = 0;
	g_invert_y = 0;
	g_swap_xy = 0;
	g_catch_mode = 0;
	g_lock_mask = 0;
	memset(g_turbo_delay, 0, sizeof(g_turbo_delay));
	memset(&g_click_sched, 0, sizeof(g_click_sched));
	memset(g_button_remap, 0, sizeof(g_button_remap));
	g_disabled_count = 0;
	g_masked_count = 0;
	g_key_remap_count = 0;
	g_stream_axis_mode = 0;
	g_stream_buttons_mode = 0;
	g_stream_mouse_mode = 0;
	g_stream_kb_mode = 0;
	g_screen_w = 1920;
	g_screen_h = 1080;
	// Identity: Ferrum at >= 2Mbaud, MACKU otherwise (overridable via TFT)
	g_identity_mode = (CMD_BAUD >= 2000000) ? 1 : 0;
	// Echo: both MACKU (4Mbaud) and Ferrum software mode echo commands
	g_echo = (CMD_BAUD > 1000000) ? 1 : 0;
	g_log_level = 0;
	g_release_timer_ms = 0;
	g_bypass_mode = 0;
	g_hs_mode = 0;
	g_pending_pan = 0;
	g_pending_tilt = 0;
}

void makd_dispatch(makd_cmd_t cmd, const uint8_t *payload, uint16_t len,
                   makd_tx_fn tx)
{
	switch (cmd) {
	case MAKD_CMD_MOUSE_LEFT:
	case MAKD_CMD_MOUSE_RIGHT:
	case MAKD_CMD_MOUSE_MIDDLE:
	case MAKD_CMD_MOUSE_SIDE1:
	case MAKD_CMD_MOUSE_SIDE2:
		handle_button(cmd, payload, len, tx);
		break;

	case MAKD_CMD_CLICK:
		handle_click(payload, len, tx);
		break;

	case MAKD_CMD_TURBO:
		handle_turbo(payload, len, tx);
		break;

	case MAKD_CMD_MOVE:
		handle_move(payload, len, tx);
		break;

	case MAKD_CMD_MOVETO:
		handle_moveto(payload, len, tx);
		break;

	case MAKD_CMD_WHEEL:
		handle_wheel(payload, len, tx);
		break;

	case MAKD_CMD_PAN:
		handle_pan(payload, len, tx);
		break;

	case MAKD_CMD_TILT:
		handle_tilt(payload, len, tx);
		break;

	case MAKD_CMD_SILENT:
		handle_silent(payload, len, tx);
		break;

	case MAKD_CMD_MO:
		handle_mo(payload, len, tx);
		break;

	case MAKD_CMD_CATCH:
		handle_catch(payload, len, tx);
		break;

	case MAKD_CMD_GETPOS:
		handle_getpos(tx);
		break;

	case MAKD_CMD_INVERT_X:
		handle_invert(cmd, &g_invert_x, payload, len, tx);
		break;

	case MAKD_CMD_INVERT_Y:
		handle_invert(cmd, &g_invert_y, payload, len, tx);
		break;

	case MAKD_CMD_LOCK:
		handle_lock(payload, len, tx);
		break;

	case MAKD_CMD_REMAP_BUTTON:
		handle_remap_button(payload, len, tx);
		break;

	case MAKD_CMD_SWAP_XY:
		handle_swap_xy(payload, len, tx);
		break;

	case MAKD_CMD_REMAP_AXIS:
		handle_remap_axis(payload, len, tx);
		break;

	case MAKD_CMD_KB_DISABLE:
		handle_kb_disable(payload, len, tx);
		break;

	case MAKD_CMD_KB_DOWN:
		handle_kb_down(payload, len, tx);
		break;

	case MAKD_CMD_KB_INIT:
		handle_kb_init(tx);
		break;

	case MAKD_CMD_KB_ISDOWN:
		handle_kb_isdown(payload, len, tx);
		break;

	case MAKD_CMD_KB_MASK:
		handle_kb_mask(payload, len, tx);
		break;

	case MAKD_CMD_KB_PRESS:
		handle_kb_press(payload, len, tx);
		break;

	case MAKD_CMD_KB_REMAP:
		handle_kb_remap(payload, len, tx);
		break;

	case MAKD_CMD_KB_STRING:
		handle_kb_string(payload, len, tx);
		break;

	case MAKD_CMD_KB_UP:
		handle_kb_up(payload, len, tx);
		break;

	case MAKD_CMD_STREAM_AXIS:
		handle_stream(cmd, &g_stream_axis_mode, &g_stream_axis_period,
		              payload, len, tx);
		break;

	case MAKD_CMD_STREAM_BUTTONS:
		handle_stream(cmd, &g_stream_buttons_mode, &g_stream_buttons_period,
		              payload, len, tx);
		break;

	case MAKD_CMD_STREAM_MOUSE:
		handle_stream(cmd, &g_stream_mouse_mode, &g_stream_mouse_period,
		              payload, len, tx);
		break;

	case MAKD_CMD_KB_STREAM:
		handle_stream(cmd, &g_stream_kb_mode, &g_stream_kb_period,
		              payload, len, tx);
		break;

	case MAKD_CMD_BAUD:
		handle_baud(payload, len, tx);
		break;

	case MAKD_CMD_BYPASS:
		handle_bypass(payload, len, tx);
		break;

	case MAKD_CMD_DEVICE:
		handle_device(tx);
		break;

	case MAKD_CMD_ECHO:
		handle_echo(payload, len, tx);
		break;

	case MAKD_CMD_FAULT:
		handle_fault(tx);
		break;

	case MAKD_CMD_HS:
		handle_hs(payload, len, tx);
		break;

	case MAKD_CMD_INFO:
		handle_info(tx);
		break;

	case MAKD_CMD_LED:
		handle_led(payload, len, tx);
		break;

	case MAKD_CMD_LOG:
		handle_log(payload, len, tx);
		break;

	case MAKD_CMD_REBOOT:
		handle_reboot(tx);
		break;

	case MAKD_CMD_RELEASE:
		handle_release(payload, len, tx);
		break;

	case MAKD_CMD_SCREEN:
		handle_screen(payload, len, tx);
		break;

	case MAKD_CMD_SERIAL:
		handle_serial(payload, len, tx);
		break;

	case MAKD_CMD_VERSION:
		handle_version(tx);
		break;

	default:
		send_err(cmd, tx);
		break;
	}
}
