#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct __attribute__((aligned(64))) {
	volatile uint32_t horizontal_link;
	volatile uint32_t capabilities[2];
	volatile uint32_t current;
	volatile uint32_t next;
	volatile uint32_t alt_next;
	volatile uint32_t token;
	volatile uint32_t buffer[5];
	uint32_t _pad[5];
} ehci_qh_t;
typedef struct __attribute__((aligned(32))) {
	volatile uint32_t next;
	volatile uint32_t alt_next;
	volatile uint32_t token;
	volatile uint32_t buffer[5];
} ehci_qtd_t;
typedef struct __attribute__((packed)) {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} usb_setup_t;
#define USB_SPEED_FULL  0  // 12 Mbps
#define USB_SPEED_LOW   1  // 1.5 Mbps
#define USB_SPEED_HIGH  2  // 480 Mbps
#define USB_REQ_GET_DESCRIPTOR  6
#define USB_REQ_SET_ADDRESS     5
#define USB_REQ_SET_CONFIG      9
#define USB_DESC_DEVICE         1
#define USB_DESC_CONFIGURATION  2
#define USB_DESC_STRING         3
#define USB_DESC_INTERFACE      4
#define USB_DESC_ENDPOINT       5
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22
#define QTD_TOKEN_ACTIVE     (1 << 7)
#define QTD_TOKEN_HALTED     (1 << 6)
#define QTD_TOKEN_BUFERR     (1 << 5)
#define QTD_TOKEN_BABBLE     (1 << 4)
#define QTD_TOKEN_XACTERR    (1 << 3)
#define QTD_TOKEN_MISSED     (1 << 2)
#define QTD_TOKEN_SPLITXSTATE (1 << 1)
#define QTD_TOKEN_PING       (1 << 0)
#define QTD_TOKEN_STATUS_MASK 0xFF
#define QTD_TOKEN_PID_SETUP  (2 << 8)
#define QTD_TOKEN_PID_IN     (1 << 8)
#define QTD_TOKEN_PID_OUT    (0 << 8)
#define QTD_TOKEN_TOGGLE     (1 << 31)
#define QTD_TOKEN_IOC        (1 << 15)
#define QTD_TOKEN_NBYTES(n)  ((uint32_t)(n) << 16)
#define QTD_TOKEN_CPAGE(n)   ((uint32_t)(n) << 12)
#define QTD_TOKEN_CERR(n)    ((uint32_t)(n) << 10)
#define QTD_TERMINATE        1
bool usb_host_init(void);
bool usb_host_device_connected(void);
void usb_host_port_reset(void);
uint8_t usb_host_device_speed(void);
void usb_host_power_on(void);
int usb_host_control_transfer(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data, uint32_t timeout_ms);

// Fire-and-forget control OUT: sets up DMA, kicks hardware, returns immediately.
// Caller must check usb_host_control_async_busy() before calling again.
void usb_host_control_transfer_fire(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data);
bool usb_host_control_async_busy(void);

#define MAX_INTR_EPS 4

void usb_host_interrupt_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt);
int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len);
// Zero-copy poll: returns pointer directly into DMA buffer (valid until next poll).
// Caller must finish reading/modifying before calling poll again on this index.
int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len);
void usb_host_interrupt_dump_state(void);
