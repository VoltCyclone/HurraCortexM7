// tft_display.h — TFT stats + settings rendering
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool     host_connected;
	bool     device_configured;
	bool     kmbox_active;
	uint8_t  protocol_mode;     // 0=none, 1=KMBox, 2=Makcu, 3=Ferrum
	uint8_t  num_endpoints;
	uint8_t  device_speed;      // 0=full, 1=low, 2=high

	uint32_t report_count;
	uint32_t drop_count;
	uint32_t reports_per_sec;

	bool     smooth_active;
	uint8_t  smooth_queue_depth;
	uint8_t  smooth_queue_max;
	uint32_t inject_count;

	uint32_t kmbox_frames_ok;
	uint32_t kmbox_frames_err;

	uint32_t uart_overrun;     // OR: FIFO overrun (DMA too slow)
	uint32_t uart_framing;     // FE: baud mismatch or signal integrity
	uint32_t uart_noise;       // NF: electrical noise on line
	uint32_t uart_rx_bytes;
	uint32_t uart_tx_bytes;

#if NET_ENABLED
	bool     net_connected;
	bool     net_link_up;
	uint32_t net_ip;
	uint16_t net_port;
	uint32_t net_uuid;
	uint32_t net_rx_count;
	uint32_t net_tx_count;
#endif

#if BT_ENABLED
	bool     bt_connected;
	uint32_t bt_baud;
	uint32_t bt_frames_ok;
	uint32_t bt_frames_err;
	uint32_t bt_rx_bytes;
#endif

	uint32_t uptime_sec;
	int8_t   cpu_temp_c;
	uint16_t usb_vid;
	uint16_t usb_pid;
	char     usb_product[22];
} tft_proxy_stats_t;

typedef enum {
	TFT_VIEW_STATS,
	TFT_VIEW_SETTINGS,
} tft_view_t;

typedef enum {
	IDENTITY_MACKU,
	IDENTITY_FERRUM,
	IDENTITY_COUNT
} identity_mode_t;

typedef enum {
	SETTING_IDENTITY,
	SETTING_BAUD,
	SETTING_SMOOTH_ENABLED,
	SETTING_SMOOTH_MAX,
	SETTING_HUMANIZE_ENABLED,
	SETTING_BACKLIGHT,
	SETTING_COUNT
} setting_id_t;

typedef struct {
	identity_mode_t identity;
	uint32_t baud;
	bool     smooth_enabled;
	int16_t  smooth_max;
	bool     humanize_enabled;
	bool     backlight;
} tft_settings_t;

void tft_display_init(void);
void tft_display_update(const tft_proxy_stats_t *stats);
void tft_display_error(const char *msg);

bool tft_display_touch(uint16_t x, uint16_t y);
tft_view_t tft_display_get_view(void);
const tft_settings_t *tft_display_get_settings(void);
