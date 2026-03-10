#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"
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
#define KMBOX_SYNC1              0x57
#define KMBOX_SYNC2              0xAB
#define KMBOX_MAX_PAYLOAD        56

#define MAKCU_SYNC_BYTE          0x50

void kmbox_init(void);

void kmbox_poll(void);

void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len);

void kmbox_cache_endpoints(const captured_descriptors_t *desc);

void kmbox_send_pending(void);

void kmbox_inject_smooth(int16_t dx, int16_t dy);

// Public injection API — called by transport layers (UART or NET)
void kmbox_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                        int8_t wheel, bool use_smooth);
void kmbox_inject_keyboard(uint8_t modifier, const uint8_t keys[6]);

uint32_t kmbox_frame_count(void);
uint32_t kmbox_error_count(void);
uint32_t kmbox_rx_byte_count(void);
