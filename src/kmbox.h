#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

// KMBox B command codes
#define KMBOX_CMD_MOUSE_MOVE     0x01
#define KMBOX_CMD_MOUSE_BUTTON   0x02
#define KMBOX_CMD_MOUSE_WHEEL    0x03
#define KMBOX_CMD_MOUSE_ALL      0x04
#define KMBOX_CMD_KEYBOARD       0x05
#define KMBOX_CMD_KEYBOARD_REL   0x06
#define KMBOX_CMD_SMOOTH_MOVE    0x07  // smooth injection: int16 X, int16 Y
#define KMBOX_CMD_SMOOTH_CONFIG  0x08  // config: uint8 max_per_frame
#define KMBOX_CMD_SMOOTH_CLEAR   0x09  // flush smooth queue
#define KMBOX_CMD_PING           0xFE

// Frame constants
#define KMBOX_SYNC1              0x57
#define KMBOX_SYNC2              0xAB
#define KMBOX_MAX_PAYLOAD        56

// Makcu sync byte (auto-detected alongside KMBox B)
#define MAKCU_SYNC_BYTE          0x50

// Initialize LPUART6 for multi-protocol command reception.
// Auto-detects KMBox B (0x57), Makcu (0x50), and Ferrum (text) protocols.
// Always active regardless of UART_ENABLED.
void kmbox_init(void);

// Poll UART RX, assemble frames, update injection state.
// Also processes smooth injection queue each call.
// Call once per main loop iteration. Non-blocking.
void kmbox_poll(void);

// Merge injection state into a real HID report in-place.
// iface_protocol: 1=keyboard, 2=mouse (from captured descriptor)
void kmbox_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len);

// Cache mouse/keyboard EP numbers from descriptors.
// Call once after enumeration — eliminates per-poll iface scanning.
void kmbox_cache_endpoints(const captured_descriptors_t *desc);

// If injection state is dirty and no real report came this cycle,
// synthesize and send injected-only reports.
void kmbox_send_pending(void);

// Apply smooth injection output to inject state.
// Called from PIT ISR-gated loop in main, not from kmbox_poll.
void kmbox_inject_smooth(int16_t dx, int16_t dy);

// Diagnostic counters
uint32_t kmbox_frame_count(void);
uint32_t kmbox_error_count(void);
uint32_t kmbox_rx_byte_count(void);
