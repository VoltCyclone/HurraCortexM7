// src/actions.h — transport-agnostic injection actions
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ACT_MAX_DISABLED_KEYS 32

// Mouse + keyboard state — transport-agnostic, accessed by ferrum.c.
extern uint8_t  g_buttons;
extern uint8_t  g_kb_modifier;
extern uint8_t  g_kb_keys[6];
extern int32_t  g_pos_x, g_pos_y;
extern uint16_t g_lock_mask;

void    act_init(void);
int8_t  act_button_set(uint8_t mask, uint8_t action);    // action: 0=up, 1=down
void    act_click(uint8_t button_1based, uint8_t count, uint32_t delay_ms);
void    act_move(int16_t dx, int16_t dy, bool smooth);
int8_t  act_kb_down(uint8_t key);
void    act_kb_up(uint8_t key);
void    act_kb_press(uint8_t key, uint32_t delay_ms);
uint8_t act_kb_isdown(uint8_t key);
void    act_kb_init(void);
void    act_kb_mask(uint8_t key, uint8_t mode);
