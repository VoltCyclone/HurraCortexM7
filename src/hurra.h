// src/hurra.h — public API mirrors ferrum.h / makcu.h so proto.h can alias.
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void (*hurra_tx_fn)(const uint8_t *buf, uint16_t len);

void hurra_init(void);
void hurra_reset(void);
void hurra_set_tx(hurra_tx_fn tx);

void hurra_feed_byte(uint8_t b);
void hurra_tick(void);

void hurra_notify_buttons(uint8_t buttons_bitmap);
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll);
void hurra_notify_keys(const uint8_t keys[6]);
