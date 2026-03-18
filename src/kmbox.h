#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"
#include "makd.h"

void kmbox_init(void);

void kmbox_poll(void);

void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len);

void kmbox_cache_endpoints(const captured_descriptors_t *desc);

void kmbox_send_pending(void);

void kmbox_inject_smooth(int16_t dx, int16_t dy);

void kmbox_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                        int8_t wheel, bool use_smooth);
void kmbox_inject_keyboard(uint8_t modifier, const uint8_t keys[6]);

void kmbox_schedule_click_release(uint8_t button_mask, uint32_t delay_ms);
void kmbox_schedule_kb_release(uint8_t key, uint32_t delay_ms);

uint32_t kmbox_frame_count(void);
uint32_t kmbox_error_count(void);
uint32_t kmbox_rx_byte_count(void);
uint32_t kmbox_tx_byte_count(void);
uint32_t kmbox_uart_overrun(void);  // OR: FIFO overrun
uint32_t kmbox_uart_framing(void);  // FE: baud mismatch / signal
uint32_t kmbox_uart_noise(void);    // NF: electrical noise
uint8_t  kmbox_protocol_mode(void); // 0=none, 1=MAKD, 2=MACKU, 3=Ferrum
void     kmbox_set_baud(uint32_t baud);
uint32_t kmbox_current_baud(void);
