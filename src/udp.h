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

void udp_init(uint32_t ip, uint32_t netmask, uint32_t gateway);

bool udp_poll(udp_packet_t *pkt);

bool udp_send(uint32_t dst_ip, uint16_t dst_port, uint16_t src_port,
              const void *data, uint16_t len);

uint32_t udp_get_ip(void);
