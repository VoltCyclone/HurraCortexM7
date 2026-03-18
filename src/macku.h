// Macku Legacy text protocol — coexists with MAKD binary
//
// RX: .command(args) — start with '.', end with ')'
//     Optional binary frame: DE AD <lenLE:2> <payload>
//     The km. prefix is ignored.  Other bytes (\r, \n, etc.) discarded.
//
// TX: km.command(result)\r\n>>>
//     Setters echo input as ACK unless echo(0).

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MACKU_CMD_MAX    32
#define MACKU_ARG_MAX    280
#define MACKU_BIN_MAX    280

// ---- TX callback type (same shape as makd_tx_fn) ----
typedef void (*macku_tx_fn)(const uint8_t *data, uint16_t len);

// ---- Text + DE AD binary parser ----
typedef struct {
	uint8_t  state;           // 0=IDLE, 1=CMD_NAME, 2=ARGS
	char     cmd[MACKU_CMD_MAX];
	uint8_t  cmd_len;
	char     arg[MACKU_ARG_MAX];
	uint16_t arg_len;
	// DE AD binary sub-parser
	uint8_t  bin_state;       // 0=IDLE, 1=GOT_DE, 2=LEN_LO, 3=LEN_HI, 4=PAYLOAD
	uint8_t  bin_buf[MACKU_BIN_MAX];
	uint16_t bin_len;
	uint16_t bin_pos;
} macku_parser_t;

static inline void macku_parser_reset(macku_parser_t *p)
{
	p->state = 0;
	p->cmd_len = 0;
	p->arg_len = 0;
	p->bin_state = 0;
	p->bin_pos = 0;
}

// Feed one byte.  Returns: 0=nothing, 1=text command ready, 2=binary frame ready.
// On return 1: p->cmd[0..cmd_len-1] = command name, p->arg[0..arg_len-1] = args
// On return 2: p->bin_buf[0..bin_len-1] = binary payload
static inline uint8_t macku_parser_feed(macku_parser_t *p, uint8_t b)
{
	// ---- DE AD binary sub-parser (independent, checked first) ----
	if (p->state == 0) {
		switch (p->bin_state) {
		case 0:
			if (b == 0xDE) {
				p->bin_state = 1;
				return 0;
			}
			break;
		case 1:
			if (b == 0xAD) {
				p->bin_state = 2;
			} else {
				p->bin_state = 0;
				// fall through to text parser below
			}
			return 0;
		case 2:
			p->bin_len = b;
			p->bin_state = 3;
			return 0;
		case 3:
			p->bin_len |= (uint16_t)b << 8;
			if (p->bin_len > MACKU_BIN_MAX) {
				p->bin_state = 0;
				return 0;
			}
			if (p->bin_len == 0) {
				p->bin_state = 0;
				return 2;
			}
			p->bin_pos = 0;
			p->bin_state = 4;
			return 0;
		case 4:
			p->bin_buf[p->bin_pos++] = b;
			if (p->bin_pos >= p->bin_len) {
				p->bin_state = 0;
				return 2;
			}
			return 0;
		}
	}

	// ---- Text command parser ----
	switch (p->state) {
	case 0: // IDLE — wait for '.'
		if (b == '.') {
			p->cmd_len = 0;
			p->arg_len = 0;
			p->state = 1;
		}
		// else discard (km prefix, \r, \n, spaces, etc.)
		return 0;

	case 1: // CMD_NAME — accumulate until '('
		if (b == '(') {
			p->cmd[p->cmd_len] = '\0';
			p->state = 2;
			return 0;
		}
		if (b == '.') {
			// Check if cmd_buf is "km" — strip prefix
			if (p->cmd_len == 2 && p->cmd[0] == 'k' && p->cmd[1] == 'm') {
				p->cmd_len = 0;
				return 0;
			}
			// Otherwise malformed — reset
			p->state = 0;
			return 0;
		}
		if ((b >= 'a' && b <= 'z') || (b >= 'A' && b <= 'Z') ||
		    (b >= '0' && b <= '9') || b == '_' || b == '+' || b == '-') {
			if (p->cmd_len < MACKU_CMD_MAX - 1)
				p->cmd[p->cmd_len++] = (char)b;
			return 0;
		}
		// Invalid char in command name — reset
		p->state = 0;
		return 0;

	case 2: // ARGS — accumulate until ')'
		if (b == ')') {
			p->arg[p->arg_len] = '\0';
			p->state = 0;
			return 1; // text command complete
		}
		if (p->arg_len < MACKU_ARG_MAX - 1)
			p->arg[p->arg_len++] = (char)b;
		else
			p->state = 0; // overflow — discard
		return 0;
	}

	p->state = 0;
	return 0;
}

// ---- Protocol API ----
void macku_init(void);
void macku_dispatch(const char *cmd, uint8_t cmd_len,
                    const char *args, uint16_t args_len, macku_tx_fn tx);
void macku_dispatch_bin(const uint8_t *payload, uint16_t len, macku_tx_fn tx);
