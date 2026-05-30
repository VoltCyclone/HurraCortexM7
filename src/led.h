// src/led.h — on-board LED (pin 13) driver.
// Two modes: GPIO7[3] for boot/enumeration/fatal codes, and an autonomous
// QuadTimer2 heartbeat (running) whose blink rate encodes UART status.
#pragma once
#include <stdint.h>

void led_init(void);     // assert pin 13 as GPIO7 output, off
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms); // blocking fatal

void led_heartbeat_start(void);                 // start autonomous blink at idle rate
void led_heartbeat_set_rate(uint16_t centihz);  // blink rate in 0.01 Hz units; glitch-free
