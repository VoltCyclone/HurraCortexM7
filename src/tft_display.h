// tft_display.h — Stats + settings rendering for TFT display
// Supports ST7735 (128x160) and ILI9341 (240x320) via TFT_DRIVER

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	// Connection status
	bool     host_connected;
	bool     device_configured;
	bool     kmbox_active;
	uint8_t  protocol_mode;     // 0=none, 1=KMBox, 2=Makcu, 3=Ferrum

	// Device info
	uint8_t  num_endpoints;
	uint8_t  device_speed;      // 0=full, 1=low, 2=high

	// Throughput
	uint32_t report_count;
	uint32_t drop_count;
	uint32_t reports_per_sec;

	// Smooth injection
	bool     smooth_active;
	uint8_t  smooth_queue_depth;
	uint8_t  smooth_queue_max;
	uint32_t inject_count;

	// KMBox frame stats
	uint32_t kmbox_frames_ok;
	uint32_t kmbox_frames_err;

	// UART hardware error breakdown
	uint32_t uart_overrun;     // OR: FIFO overrun (DMA too slow)
	uint32_t uart_framing;     // FE: baud mismatch or signal integrity
	uint32_t uart_noise;       // NF: electrical noise on line

	// UART / KMBox RX/TX
	uint32_t uart_rx_bytes;    // total bytes received on LPUART6 (DMA ring)
	uint32_t uart_tx_bytes;    // total bytes sent via LPUART6 TX DMA

#if NET_ENABLED
	// Network (KMBox Net over Ethernet)
	bool     net_connected;    // Client connected via UDP
	bool     net_link_up;      // Ethernet PHY link status
	uint32_t net_ip;           // Our IP address (host byte order)
	uint16_t net_port;         // Listening UDP port
	uint32_t net_uuid;         // Device UUID (8 hex chars)
	uint32_t net_rx_count;     // UDP packets received
	uint32_t net_tx_count;     // UDP packets sent
#endif

#if BT_ENABLED
	// Bluetooth (HC-05 SPP over LPUART7)
	bool     bt_connected;     // STA pin high = paired
	uint32_t bt_baud;          // Active baud rate after AT config
	uint32_t bt_frames_ok;     // Parsed command frames
	uint32_t bt_frames_err;    // Checksum/length errors
	uint32_t bt_rx_bytes;      // Total bytes received
#endif

	// System
	uint32_t uptime_sec;
	int8_t   cpu_temp_c;        // CPU die temperature in °C

	// USB device descriptor
	uint16_t usb_vid;
	uint16_t usb_pid;
	char     usb_product[22];   // truncated product name (21 chars + null)
} tft_proxy_stats_t;

// ---- View modes ----
typedef enum {
	TFT_VIEW_STATS,     // Default: proxy stats dashboard
	TFT_VIEW_SETTINGS,  // Touch-driven settings menu
} tft_view_t;

// ---- Settings ----
typedef enum {
	SETTING_SMOOTH_ENABLED,
	SETTING_SMOOTH_MAX,        // max displacement per frame
	SETTING_HUMANIZE_ENABLED,
	SETTING_BACKLIGHT,         // on/off
	SETTING_COUNT
} setting_id_t;

typedef struct {
	bool     smooth_enabled;
	int16_t  smooth_max;       // max per frame (1-127)
	bool     humanize_enabled;
	bool     backlight;
} tft_settings_t;

// Initialize display hardware + draw splash screen
void tft_display_init(void);

// Render stats into framebuffer and flush to display.
// Call from main loop at ~30 Hz.
void tft_display_update(const tft_proxy_stats_t *stats);

// Show an error message (replaces normal display)
void tft_display_error(const char *msg);

// ---- Touch/settings (only active when TOUCH_ENABLED) ----

// Process a touch event (coordinates in display space).
// Returns true if a setting was changed.
bool tft_display_touch(uint16_t x, uint16_t y);

// Get current view mode
tft_view_t tft_display_get_view(void);

// Get current settings
const tft_settings_t *tft_display_get_settings(void);
