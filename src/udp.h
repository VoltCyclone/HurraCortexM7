// udp.h — Minimal UDP/IP/ARP stack for bare-metal Teensy 4.1
// No TCP, no DHCP, no fragmentation. Static IP only.

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t       src_ip;
	uint16_t       src_port;
	uint16_t       dst_port;
	const uint8_t *data;
	uint16_t       len;
} udp_packet_t;

// Initialize the UDP stack with a static IP configuration.
void udp_init(uint32_t ip, uint32_t netmask, uint32_t gateway);

// Poll for an incoming UDP packet. Returns true if a packet was received.
// The packet's data pointer is valid until the next udp_poll() call.
bool udp_poll(udp_packet_t *pkt);

// Send a UDP packet to the specified destination.
bool udp_send(uint32_t dst_ip, uint16_t dst_port, uint16_t src_port,
              const void *data, uint16_t len);

// Get our configured IP address.
uint32_t udp_get_ip(void);
