#pragma once
#include <stdint.h>
#include <stdbool.h>

// HC-05 Bluetooth SPP input over LPUART7 (Teensy pins D28 TX / D29 RX)
// D30 = STA pin (input, high when paired/connected)
// Same multi-protocol auto-detect as UART: KMBox B, Makcu, Ferrum

void bt_init(void);

// Poll LPUART7 DMA ring for incoming bytes, parse and inject.
// Call from main loop alongside kmbox_poll().
void bt_poll(void);

// True if HC-05 STA pin reads high (paired and connected)
bool bt_connected(void);

// Stats
uint32_t bt_frame_count(void);
uint32_t bt_error_count(void);
uint32_t bt_rx_byte_count(void);

// Active baud rate (may differ from BT_BAUD if AT config failed)
uint32_t bt_get_baud(void);

// Detected protocol: 0=none, 1=KMBox, 2=Makcu, 3=Ferrum
uint8_t bt_protocol_mode(void);
