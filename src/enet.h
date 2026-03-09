// enet.h — Bare-metal ENET MAC + DP83825I PHY driver for Teensy 4.1
// Polled RX/TX with DMA descriptor rings, no interrupts needed.

#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialize ENET clocks, PHY, pin mux, DMA rings, and enable MAC.
// Returns true on success, false if PHY not detected.
bool enet_init(void);

// Poll PHY link status (non-blocking MDIO read).
// Call periodically (~every 100ms) from main loop.
bool enet_link_up(void);

// Read the factory-programmed MAC address from OTP fuses.
void enet_get_mac(uint8_t mac[6]);

// Poll for a received Ethernet frame (zero-copy).
// Returns frame length (>0) and sets *frame_out to point into DMA buffer.
// Returns 0 if no frame available. Caller must call enet_rx_release()
// before the next enet_rx() call.
int enet_rx(const uint8_t **frame_out);

// Release the current RX descriptor back to hardware.
void enet_rx_release(void);

// Transmit an Ethernet frame. Blocks until a TX descriptor is available.
// Returns true on success.
bool enet_tx(const uint8_t *frame, uint16_t len);
