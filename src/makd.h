// MAKD binary protocol
//
// Frame: [0x50] [CMD:u8] [LEN_LO:u8] [LEN_HI:u8] [PAYLOAD...]
// All multi-byte values little-endian.

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MAKD_SYNC          0x50
#define MAKD_HEADER_SIZE   4
#define MAKD_MAX_PAYLOAD   280  // 256 for string cmd + headroom
#define MAKD_OK            0x00
#define MAKD_ERR           0x01
typedef enum __attribute__((packed)) {
	// Streaming
	MAKD_CMD_STREAM_AXIS    = 0x01,
	MAKD_CMD_STREAM_BUTTONS = 0x02,
	MAKD_CMD_STREAM_MOUSE   = 0x0C,
	// Mouse
	MAKD_CMD_CATCH          = 0x03,
	MAKD_CMD_CLICK          = 0x04,
	MAKD_CMD_GETPOS         = 0x05,
	MAKD_CMD_INVERT_X       = 0x06,
	MAKD_CMD_INVERT_Y       = 0x07,
	MAKD_CMD_MOUSE_LEFT     = 0x08,
	MAKD_CMD_LOCK           = 0x09,
	MAKD_CMD_MOUSE_MIDDLE   = 0x0A,
	MAKD_CMD_MO             = 0x0B,  // raw mouse frame
	MAKD_CMD_MOVE           = 0x0D,
	MAKD_CMD_MOVETO         = 0x0E,
	MAKD_CMD_PAN            = 0x0F,
	MAKD_CMD_REMAP_BUTTON   = 0x10,
	MAKD_CMD_MOUSE_RIGHT    = 0x11,
	MAKD_CMD_MOUSE_SIDE1    = 0x12,
	MAKD_CMD_MOUSE_SIDE2    = 0x13,
	MAKD_CMD_SILENT         = 0x14,
	MAKD_CMD_SWAP_XY        = 0x15,
	MAKD_CMD_TILT           = 0x16,
	MAKD_CMD_TURBO          = 0x17,
	MAKD_CMD_WHEEL          = 0x18,
	MAKD_CMD_REMAP_AXIS     = 0x19,
	// Keyboard
	MAKD_CMD_KB_DISABLE     = 0xA1,
	MAKD_CMD_KB_DOWN        = 0xA2,
	MAKD_CMD_KB_INIT        = 0xA3,
	MAKD_CMD_KB_ISDOWN      = 0xA4,
	MAKD_CMD_KB_STREAM      = 0xA5,  // keyboard streaming
	MAKD_CMD_KB_MASK        = 0xA6,
	MAKD_CMD_KB_PRESS       = 0xA7,
	MAKD_CMD_KB_REMAP       = 0xA8,
	MAKD_CMD_KB_STRING      = 0xA9,
	MAKD_CMD_KB_UP          = 0xAA,
	// System
	MAKD_CMD_BAUD           = 0xB1,
	MAKD_CMD_BYPASS         = 0xB2,
	MAKD_CMD_DEVICE         = 0xB3,
	MAKD_CMD_ECHO           = 0xB4,
	MAKD_CMD_FAULT          = 0xB5,
	MAKD_CMD_HS             = 0xB7,
	MAKD_CMD_INFO           = 0xB8,
	MAKD_CMD_LED            = 0xB9,
	MAKD_CMD_LOG            = 0xBA,
	MAKD_CMD_REBOOT         = 0xBB,
	MAKD_CMD_RELEASE        = 0xBC,
	MAKD_CMD_SCREEN         = 0xBD,
	MAKD_CMD_SERIAL         = 0xBE,
	MAKD_CMD_VERSION        = 0xBF,
} makd_cmd_t;

typedef enum __attribute__((packed)) {
	MAKD_LOCK_MX            = 0x00,
	MAKD_LOCK_MY            = 0x01,
	MAKD_LOCK_MW            = 0x02,
	MAKD_LOCK_MX_POS        = 0x03,
	MAKD_LOCK_MX_NEG        = 0x04,
	MAKD_LOCK_MY_POS        = 0x05,
	MAKD_LOCK_MY_NEG        = 0x06,
	MAKD_LOCK_MW_POS        = 0x07,
	MAKD_LOCK_MW_NEG        = 0x08,
	MAKD_LOCK_ML            = 0x09,
	MAKD_LOCK_MM            = 0x0A,
	MAKD_LOCK_MR            = 0x0B,
	MAKD_LOCK_MS1           = 0x0C,
	MAKD_LOCK_MS2           = 0x0D,
	MAKD_LOCK_TARGET_COUNT  = 0x0E,
} makd_lock_t;

typedef void (*makd_tx_fn)(const uint8_t *data, uint16_t len);

typedef struct {
	uint8_t    state;     // 0=sync, 1=cmd, 2=len_lo, 3=len_hi, 4=payload
	makd_cmd_t cmd;
	uint16_t len;
	uint16_t pos;
	uint8_t  buf[MAKD_MAX_PAYLOAD];
} makd_parser_t;

static inline void makd_parser_reset(makd_parser_t *p)
{
	p->state = 0;
	p->pos = 0;
}

// Feed one byte.  Returns true when a complete frame is ready in p->cmd/buf/len.
static inline bool makd_parser_feed(makd_parser_t *p, uint8_t b)
{
	switch (p->state) {
	case 0: // WAIT_SYNC
		if (b == MAKD_SYNC)
			p->state = 1;
		return false;
	case 1: // WAIT_CMD
		p->cmd = b;
		p->state = 2;
		return false;
	case 2: // WAIT_LEN_LO
		p->len = b;
		p->state = 3;
		return false;
	case 3: // WAIT_LEN_HI
		p->len |= (uint16_t)b << 8;
		if (p->len > MAKD_MAX_PAYLOAD) {
			p->state = 0;
			return false;
		}
		if (p->len == 0) {
			p->state = 0;
			return true; // complete: zero-length payload
		}
		p->pos = 0;
		p->state = 4;
		return false;
	case 4: // PAYLOAD
		p->buf[p->pos++] = b;
		if (p->pos >= p->len) {
			p->state = 0;
			return true; // complete
		}
		return false;
	default:
		p->state = 0;
		return false;
	}
}

#define MAKD_MAX_DISABLED_KEYS 32
#define MAKD_MAX_KEY_REMAPS    16

typedef struct {
	uint8_t  button;     // 1-5
	uint8_t  remaining;  // clicks left
	uint8_t  delay_ms;   // inter-click delay
	uint32_t next_at;    // millis() timestamp
	bool     pressed;    // current phase: true=held, false=released
} click_sched_t;

extern uint8_t  g_buttons;
extern uint8_t  g_kb_modifier;
extern uint8_t  g_kb_keys[6];
extern int32_t  g_pos_x, g_pos_y;
extern uint8_t  g_invert_x, g_invert_y, g_swap_xy;
extern uint8_t  g_catch_mode;
extern uint16_t g_lock_mask;
extern uint16_t g_turbo_delay[5];
extern click_sched_t g_click_sched;
extern uint8_t  g_button_remap[5];
extern uint8_t  g_disabled_keys[MAKD_MAX_DISABLED_KEYS];
extern uint8_t  g_disabled_count;
extern uint8_t  g_masked_keys[MAKD_MAX_DISABLED_KEYS];
extern uint8_t  g_masked_modes[MAKD_MAX_DISABLED_KEYS];
extern uint8_t  g_masked_count;
extern uint8_t  g_key_remap_src[MAKD_MAX_KEY_REMAPS];
extern uint8_t  g_key_remap_dst[MAKD_MAX_KEY_REMAPS];
extern uint8_t  g_key_remap_count;
extern uint8_t  g_stream_axis_mode, g_stream_axis_period;
extern uint8_t  g_stream_buttons_mode, g_stream_buttons_period;
extern uint8_t  g_stream_mouse_mode, g_stream_mouse_period;
extern uint8_t  g_stream_kb_mode, g_stream_kb_period;
extern int16_t  g_screen_w, g_screen_h;
extern uint8_t  g_echo;
extern uint8_t  g_identity_mode;   // 0=MACKU, 1=Ferrum (mirrors identity_mode_t)
extern uint8_t  g_log_level;
extern uint32_t g_release_timer_ms;
extern uint8_t  g_bypass_mode;
extern uint8_t  g_hs_mode;
extern int8_t   g_pending_pan, g_pending_tilt;

void makd_init(void);
void makd_dispatch(makd_cmd_t cmd, const uint8_t *payload, uint16_t len,
                   makd_tx_fn tx);

// Shared action functions (transport-agnostic)
// Used by both MAKD (binary) and MACKU (text) protocol handlers.
int8_t  makd_action_button_set(uint8_t mask, uint8_t action);
void    makd_action_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms);
void    makd_action_move(int16_t dx, int16_t dy, bool smooth);
void    makd_action_moveto(int16_t x, int16_t y, bool smooth);
void    makd_action_silent(int16_t x, int16_t y);
void    makd_action_mo(uint8_t buttons, int16_t x, int16_t y,
                       int8_t wheel, int8_t pan, int8_t tilt);
int8_t  makd_action_kb_down(uint8_t key);
void    makd_action_kb_up(uint8_t key);
void    makd_action_kb_press(uint8_t key, uint32_t delay_ms);
uint8_t makd_action_kb_isdown(uint8_t key);
void    makd_action_kb_disable_set(uint8_t key, uint8_t enable);
void    makd_action_kb_mask(uint8_t key, uint8_t mode);
void    makd_action_kb_remap(uint8_t src, uint8_t dst);
void    makd_action_kb_init(void);
