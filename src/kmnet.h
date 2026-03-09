// kmnet.h — KMBox Net UDP protocol handler
// Implements the device side of the KMBox Net protocol over Ethernet.

#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialize ENET + UDP stack + generate device UUID.
// Must be called before kmnet_poll(). Blocks until PHY is ready.
void kmnet_init(void);

// Poll for incoming KMBox Net commands and dispatch them.
// Call from main loop (replaces kmbox_poll() when NET_ENABLED).
void kmnet_poll(void);

// Statistics
uint32_t kmnet_rx_count(void);
uint32_t kmnet_tx_count(void);

// Device identity (displayed on TFT)
uint32_t kmnet_get_uuid(void);
uint16_t kmnet_get_port(void);
bool     kmnet_client_connected(void);
bool     kmnet_link_up(void);
