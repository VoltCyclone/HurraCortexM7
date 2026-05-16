#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

// Maximum endpoints: EP0 (control) + up to 7 more
// dQH array indexed: [ep*2] = RX (OUT), [ep*2+1] = TX (IN)
#define USB_DEV_NUM_ENDPOINTS  8

// Device Queue Head (dQH) — 64 bytes, must be 4KB-aligned as an array
typedef struct __attribute__((aligned(64))) {
	volatile uint32_t config;      // max_pkt[26:16], IOS[15], ZLT[29]
	volatile uint32_t current;     // Current dTD pointer (hardware)
	volatile uint32_t next;        // Next dTD pointer
	volatile uint32_t token;       // Status/token
	volatile uint32_t buffer[5];   // Buffer page pointers
	volatile uint32_t reserved;
	volatile uint32_t setup0;      // SETUP packet bytes 0-3 (EP0 RX only)
	volatile uint32_t setup1;      // SETUP packet bytes 4-7 (EP0 RX only)
	uint32_t _pad[4];             // Pad to 64 bytes
} usb_dev_dqh_t;

// Device Transfer Descriptor (dTD) — 32 bytes
typedef struct __attribute__((aligned(32))) {
	volatile uint32_t next;        // Next dTD (bit 0 = terminate)
	volatile uint32_t token;       // total_bytes[30:16], IOC[15], active[7]
	volatile uint32_t buffer[5];   // Buffer page pointers
	uint32_t _pad;
} usb_dev_dtd_t;

// dQH config field
#define DQH_MAX_PACKET(n)   ((uint32_t)((n) & 0x7FF) << 16)
#define DQH_IOS             (1 << 15)  // Interrupt on SETUP (EP0 RX)
#define DQH_ZLT_DISABLE     (1 << 29)  // Disable zero-length termination

// dTD token field
#define DTD_ACTIVE           (1 << 7)
#define DTD_HALTED           (1 << 6)
#define DTD_BUFFER_ERR       (1 << 5)
#define DTD_XACT_ERR         (1 << 3)
#define DTD_IOC              (1 << 15)
#define DTD_TOTAL_BYTES(n)   ((uint32_t)((n) & 0x7FFF) << 16)
#define DTD_TERMINATE        1

// Device state
typedef enum {
	USB_DEV_STATE_DEFAULT,
	USB_DEV_STATE_ADDRESS,
	USB_DEV_STATE_CONFIGURED
} usb_dev_state_t;

// Maximum interrupt IN endpoints for composite device support
#define MAX_INT_EPS 4

// Public API
bool usb_device_init(const captured_descriptors_t *desc);
void usb_device_poll(void);
bool usb_device_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len);
bool usb_device_is_configured(void);
