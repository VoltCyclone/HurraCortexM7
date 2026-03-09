// kmnet.c — KMBox Net UDP protocol handler
// Implements the device side of the KMBox Net protocol.
// Parses 16-byte headers, dispatches commands to the shared injection API.

#if !NET_ENABLED
typedef int _kmnet_unused;
#else

#include "kmnet.h"
#include "udp.h"
#include "enet.h"
#include "kmbox.h"
#include "smooth.h"
#include <string.h>

extern uint32_t millis(void);

// ---- KMBox Net protocol constants ----
#define CMD_CONNECT       0xAF3C2828
#define CMD_MOUSE_MOVE    0xAEDE7345
#define CMD_MOUSE_LEFT    0x9823AE8D
#define CMD_MOUSE_MIDDLE  0x97A3AE8D
#define CMD_MOUSE_RIGHT   0x238D8212
#define CMD_MOUSE_WHEEL   0xFFEEAD38
#define CMD_MOUSE_AUTO    0xAEDE7346
#define CMD_KEYBOARD_ALL  0x123C2C2F
#define CMD_REBOOT        0xAA8855AA
#define CMD_DEBUG         0x27382021
#define CMD_BAZER_MOVE    0xA238455A
#define CMD_MONITOR       0x27388020
#define CMD_MASK_MOUSE    0x23234343
#define CMD_UNMASK_ALL    0x23344343
#define CMD_SETCONFIG     0x1D3D3323
#define CMD_SETVIDPID     0xFFED3232
#define CMD_SHOWPIC       0x12334883
#define CMD_TRACE_ENABLE  0xBBCDDDAC

// ---- Packet structures ----
typedef struct {
	uint32_t mac;       // Device UUID
	uint32_t rand;      // Random or overloaded parameter
	uint32_t indexpts;  // Sequence counter
	uint32_t cmd;       // Command code
} cmd_head_t;

typedef struct {
	int32_t button;
	int32_t x;
	int32_t y;
	int32_t wheel;
	int32_t point[10];  // Bezier control points
} soft_mouse_t;

typedef struct {
	int8_t ctrl;
	int8_t reserved;
	int8_t button[10];
} soft_keyboard_t;

// ---- Configuration ----
#ifndef KMNET_IP
#define KMNET_IP       0xC0A802BC  // 192.168.2.188
#endif
#ifndef KMNET_NETMASK
#define KMNET_NETMASK  0xFFFFFF00  // 255.255.255.0
#endif
#ifndef KMNET_GATEWAY
#define KMNET_GATEWAY  0xC0A80201  // 192.168.2.1
#endif
#ifndef KMNET_PORT
#define KMNET_PORT     12888
#endif

// ---- State ----
static uint32_t device_uuid;
static uint16_t listen_port = KMNET_PORT;

// Connected client
static uint32_t client_ip;
static uint16_t client_port;
static bool     client_connected;

// Stats
static uint32_t rx_count;
static uint32_t tx_count;

// Link polling
static uint32_t last_link_check;
static bool     link_status;

// ---- Helper: read little-endian uint32 from buffer ----
static inline uint32_t rd32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int32_t rd32le_s(const uint8_t *p)
{
	return (int32_t)rd32le(p);
}

static inline void wr32le(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

// ---- Parse header from raw bytes ----
static bool parse_header(const uint8_t *data, uint16_t len, cmd_head_t *hdr)
{
	if (len < 16) return false;
	hdr->mac      = rd32le(data);
	hdr->rand     = rd32le(data + 4);
	hdr->indexpts = rd32le(data + 8);
	hdr->cmd      = rd32le(data + 12);
	return true;
}

// ---- Send response (echo header back to client) ----
static void send_response(const cmd_head_t *hdr, const void *payload, uint16_t payload_len)
{
	uint8_t buf[128];
	wr32le(buf,      hdr->mac);
	wr32le(buf + 4,  hdr->rand);
	wr32le(buf + 8,  hdr->indexpts);
	wr32le(buf + 12, hdr->cmd);
	if (payload && payload_len > 0 && payload_len <= sizeof(buf) - 16) {
		memcpy(buf + 16, payload, payload_len);
	}
	uint16_t total = 16 + payload_len;
	if (udp_send(client_ip, client_port, listen_port, buf, total))
		tx_count++;
}

// ---- Generate UUID from factory MAC ----
static uint32_t generate_uuid(void)
{
	uint8_t mac[6];
	enet_get_mac(mac);
	// Simple hash: FNV-1a 32-bit
	uint32_t h = 0x811C9DC5;
	for (int i = 0; i < 6; i++) {
		h ^= mac[i];
		h *= 0x01000193;
	}
	return h;
}

// ---- Command dispatch ----

static void handle_connect(const cmd_head_t *hdr, uint32_t src_ip, uint16_t src_port)
{
	// Accept connection — client must have our UUID in the mac field
	if (hdr->mac != device_uuid) return;

	client_ip = src_ip;
	client_port = src_port;
	client_connected = true;
	send_response(hdr, NULL, 0);
}

static void handle_mouse_move(const cmd_head_t *hdr, const uint8_t *data, uint16_t len)
{
	if (len < 16 + 12) { send_response(hdr, NULL, 0); return; }
	const uint8_t *m = data + 16;
	int32_t button = rd32le_s(m);
	int32_t x      = rd32le_s(m + 4);
	int32_t y      = rd32le_s(m + 8);
	int32_t wheel  = (len >= 16 + 16) ? rd32le_s(m + 12) : 0;

	kmbox_inject_mouse((int16_t)x, (int16_t)y, (uint8_t)button,
	                   (int8_t)wheel, false);
	send_response(hdr, NULL, 0);
}

static void handle_mouse_button(const cmd_head_t *hdr, const uint8_t *data, uint16_t len,
                                uint8_t button_bit)
{
	if (len < 16 + 4) { send_response(hdr, NULL, 0); return; }
	int32_t state = rd32le_s(data + 16);  // 1=press, 0=release
	// Get current buttons, set/clear this bit
	uint8_t buttons = state ? button_bit : 0;
	kmbox_inject_mouse(0, 0, buttons, 0, false);
	send_response(hdr, NULL, 0);
}

static void handle_mouse_wheel(const cmd_head_t *hdr, const uint8_t *data, uint16_t len)
{
	if (len < 16 + 16) { send_response(hdr, NULL, 0); return; }
	int32_t wheel = rd32le_s(data + 16 + 12);
	kmbox_inject_mouse(0, 0, 0, (int8_t)wheel, false);
	send_response(hdr, NULL, 0);
}

static void handle_mouse_automove(const cmd_head_t *hdr, const uint8_t *data, uint16_t len)
{
	if (len < 16 + 12) { send_response(hdr, NULL, 0); return; }
	const uint8_t *m = data + 16;
	int32_t x = rd32le_s(m + 4);
	int32_t y = rd32le_s(m + 8);
	// Automove uses the smooth injection path
	kmbox_inject_mouse((int16_t)x, (int16_t)y, 0, 0, true);
	send_response(hdr, NULL, 0);
}

static void handle_keyboard(const cmd_head_t *hdr, const uint8_t *data, uint16_t len)
{
	if (len < 16 + 12) { send_response(hdr, NULL, 0); return; }
	const uint8_t *k = data + 16;
	uint8_t modifier = (uint8_t)k[0];
	uint8_t keys[6];
	// KMBox Net keyboard has 10-byte key array; we use first 6
	for (int i = 0; i < 6; i++)
		keys[i] = (i + 2 < 12) ? (uint8_t)k[i + 2] : 0;
	kmbox_inject_keyboard(modifier, keys);
	send_response(hdr, NULL, 0);
}

static void handle_reboot(const cmd_head_t *hdr)
{
	send_response(hdr, NULL, 0);
	// Trigger software reset via SCB_AIRCR
	__asm volatile("dsb");
	*((volatile uint32_t *)0xE000ED0C) = 0x05FA0004;
	while (1) {}
}

static void dispatch_command(const cmd_head_t *hdr, const uint8_t *data, uint16_t len,
                             uint32_t src_ip, uint16_t src_port)
{
	// Connect command can come from anyone
	if (hdr->cmd == CMD_CONNECT) {
		handle_connect(hdr, src_ip, src_port);
		return;
	}

	// All other commands require an established connection with matching UUID
	if (!client_connected) return;
	if (hdr->mac != device_uuid) return;

	switch (hdr->cmd) {
	case CMD_MOUSE_MOVE:
		handle_mouse_move(hdr, data, len);
		break;
	case CMD_MOUSE_LEFT:
		handle_mouse_button(hdr, data, len, 0x01);
		break;
	case CMD_MOUSE_RIGHT:
		handle_mouse_button(hdr, data, len, 0x02);
		break;
	case CMD_MOUSE_MIDDLE:
		handle_mouse_button(hdr, data, len, 0x04);
		break;
	case CMD_MOUSE_WHEEL:
		handle_mouse_wheel(hdr, data, len);
		break;
	case CMD_MOUSE_AUTO:
	case CMD_BAZER_MOVE:
		handle_mouse_automove(hdr, data, len);
		break;
	case CMD_KEYBOARD_ALL:
		handle_keyboard(hdr, data, len);
		break;
	case CMD_REBOOT:
		handle_reboot(hdr);
		break;
	case CMD_DEBUG:
		send_response(hdr, NULL, 0);
		break;
	case CMD_MONITOR:
	case CMD_MASK_MOUSE:
	case CMD_UNMASK_ALL:
	case CMD_SETCONFIG:
	case CMD_SETVIDPID:
	case CMD_SHOWPIC:
	case CMD_TRACE_ENABLE:
		// Acknowledge but don't implement these yet
		send_response(hdr, NULL, 0);
		break;
	default:
		break;
	}
}

// ---- Public API ----

void kmnet_init(void)
{
	// Initialize Ethernet hardware
	if (!enet_init()) {
		// PHY not detected — this is fatal in NET mode
		// Main will show error on TFT
		return;
	}

	device_uuid = generate_uuid();
	client_connected = false;
	rx_count = 0;
	tx_count = 0;
	link_status = false;
	last_link_check = 0;

	udp_init(KMNET_IP, KMNET_NETMASK, KMNET_GATEWAY);
}

void kmnet_poll(void)
{
	// Poll link status periodically
	uint32_t now = millis();
	if (now - last_link_check >= 100) {
		link_status = enet_link_up();
		last_link_check = now;
	}

	// Process incoming packets (drain all available)
	udp_packet_t pkt;
	while (udp_poll(&pkt)) {
		rx_count++;

		// Must be on our listen port
		if (pkt.dst_port != listen_port) continue;

		cmd_head_t hdr;
		if (!parse_header(pkt.data, pkt.len, &hdr)) continue;

		dispatch_command(&hdr, pkt.data, pkt.len, pkt.src_ip, pkt.src_port);
	}
}

uint32_t kmnet_rx_count(void)        { return rx_count; }
uint32_t kmnet_tx_count(void)        { return tx_count; }
uint32_t kmnet_get_uuid(void)        { return device_uuid; }
uint16_t kmnet_get_port(void)        { return listen_port; }
bool     kmnet_client_connected(void){ return client_connected; }
bool     kmnet_link_up(void)         { return link_status; }

#endif // NET_ENABLED
