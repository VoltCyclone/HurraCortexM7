// USB Device controller driver for Teensy 4.1 USB1 port
// Replays captured descriptors and forwards HID reports.
// Supports composite devices with multiple interrupt IN endpoints.
// Bare-metal, polled (no interrupts).
//
// DMA buffers are in a non-cacheable MPU region (.dmabuffers at 0x20200000).
// No manual cache maintenance needed — bare DSBs drain the write buffer
// before the USB controller is told to read DMA structures.

#include <string.h>
#include "imxrt.h"
#include "usb_device.h"
#include "usb_host.h"
#include "uart.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// ---- DMA-safe static allocations ----

// 16 dQH entries (8 EP × 2 dir), 4KB-aligned
static usb_dev_dqh_t dqh_list[USB_DEV_NUM_ENDPOINTS * 2]
	__attribute__((section(".dmabuffers"), aligned(4096)));

// dTDs — one for EP0 TX, one for EP0 RX (status), one per interrupt IN endpoint
static usb_dev_dtd_t dtd_ep0_tx  __attribute__((section(".dmabuffers"), aligned(32)));
static usb_dev_dtd_t dtd_ep0_rx  __attribute__((section(".dmabuffers"), aligned(32)));
static usb_dev_dtd_t dtd_int_tx[MAX_INT_EPS]
	__attribute__((section(".dmabuffers"), aligned(32)));

// Data buffers — double-banked: USB DMA reads bank[active], CPU writes bank[!active]
static uint8_t ep0_tx_buf[512] __attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t int_tx_buf[MAX_INT_EPS][2][64]
	__attribute__((section(".dmabuffers"), aligned(32)));

// ---- State ----

static const captured_descriptors_t *cap_desc;
static usb_dev_state_t dev_state;
static uint8_t ep_to_slot[USB_DEV_NUM_ENDPOINTS]; // EP num -> dtd/buf slot
static uint8_t num_int_eps;

// Bitmask state — bit N corresponds to EP N (bits 1-7 used, bit 0 unused)
static uint8_t ep_busy_mask;     // bit set = EP has active DMA transfer in flight
static uint8_t active_bank_mask; // bit set = EP using bank 1, clear = bank 0
static uint8_t pending_len[USB_DEV_NUM_ENDPOINTS];  // 0 = no pending report staged

// ---- Endpoint control register access ----

static volatile uint32_t *endptctrl_reg(uint8_t ep)
{
	// ENDPTCTRL0-7 at offset 0x1C0 + ep*4
	return (volatile uint32_t *)(0x402E0000 + 0x1C0 + ep * 4);
}

// Prime an interrupt IN endpoint from a specific bank buffer.
// Caller must ensure EP is not busy.
static void prime_int_ep(uint8_t ep_num, uint8_t slot, uint8_t bank, uint16_t len)
{
	uint8_t qh_idx = ep_num * 2 + 1;

	dtd_int_tx[slot].next = DTD_TERMINATE;
	dtd_int_tx[slot].token = DTD_ACTIVE | DTD_IOC | DTD_TOTAL_BYTES(len);
	dtd_int_tx[slot].buffer[0] = (uint32_t)int_tx_buf[slot][bank];

	dqh_list[qh_idx].next = (uint32_t)&dtd_int_tx[slot];
	dqh_list[qh_idx].token = 0;
	asm volatile("dsb" ::: "memory");

	USB1_ENDPTPRIME = (1 << (16 + ep_num));
	if (bank)
		active_bank_mask |= (1 << ep_num);
	else
		active_bank_mask &= ~(1 << ep_num);
	ep_busy_mask |= (1 << ep_num);
}

// ---- EP0 primitives ----

static void ep0_tx_data(const uint8_t *data, uint16_t len)
{
	if (data && len > 0) {
		if (len > sizeof(ep0_tx_buf)) len = sizeof(ep0_tx_buf);
		memcpy(ep0_tx_buf, data, len);
	}

	memset(&dtd_ep0_tx, 0, sizeof(dtd_ep0_tx));
	dtd_ep0_tx.next = DTD_TERMINATE;
	dtd_ep0_tx.token = DTD_ACTIVE | DTD_IOC | DTD_TOTAL_BYTES(len);
	if (len > 0) {
		dtd_ep0_tx.buffer[0] = (uint32_t)ep0_tx_buf;
		dtd_ep0_tx.buffer[1] = ((uint32_t)ep0_tx_buf + 4096) & ~0xFFF;
	}

	// Link dTD to EP0 TX dQH (index 1) and prime
	dqh_list[1].next = (uint32_t)&dtd_ep0_tx;
	dqh_list[1].token = 0;
	asm volatile("dsb" ::: "memory");

	USB1_ENDPTPRIME = (1 << 16); // Prime EP0 TX

	// For IN data transfers (len > 0), the host follows up with a
	// STATUS OUT ZLP.  We must prime EP0 RX to ACK it; otherwise the
	// controller NAKs the status phase and the host retries for
	// seconds before giving up — causing the slow string-descriptor
	// appearance.
	if (len > 0) {
		memset(&dtd_ep0_rx, 0, sizeof(dtd_ep0_rx));
		dtd_ep0_rx.next  = DTD_TERMINATE;
		dtd_ep0_rx.token = DTD_ACTIVE | DTD_IOC | DTD_TOTAL_BYTES(0);

		dqh_list[0].next  = (uint32_t)&dtd_ep0_rx;
		dqh_list[0].token = 0;
		asm volatile("dsb" ::: "memory");

		USB1_ENDPTPRIME |= (1 << 0); // Prime EP0 RX
	}
}

static void ep0_stall(void)
{
	USB1_ENDPTCTRL0 |= USB_ENDPTCTRL_TXS | USB_ENDPTCTRL_RXS;
}

// ---- Request handlers ----

static void handle_get_descriptor(const usb_setup_t *setup)
{
	uint8_t desc_type = setup->wValue >> 8;
	uint8_t desc_index = setup->wValue & 0xFF;
	uint16_t max_len = setup->wLength;
	const uint8_t *data = NULL;
	uint16_t len = 0;

	switch (desc_type) {
	case USB_DESC_DEVICE:
		data = cap_desc->device_desc;
		len = cap_desc->device_desc_len;
		break;

	case USB_DESC_CONFIGURATION:
		data = cap_desc->config_desc;
		len = cap_desc->config_desc_len;
		break;

	case USB_DESC_STRING:
		if (desc_index == 0) {
			// Language ID descriptor
			static const uint8_t lang_desc[] = {0x04, 0x03, 0x09, 0x04};
			data = lang_desc;
			len = 4;
		} else {
			// Look up by original USB string index
			for (uint8_t i = 0; i < cap_desc->num_strings; i++) {
				if (cap_desc->string_index[i] == desc_index) {
					data = cap_desc->string_desc[i];
					len = cap_desc->string_desc_len[i];
					break;
				}
			}
			if (data == NULL) {
				uart_puts("  DEV: string idx ");
				uart_putdec(desc_index);
				uart_puts(" NOT FOUND (have");
				for (uint8_t i = 0; i < cap_desc->num_strings; i++) {
					uart_putc(' ');
					uart_putdec(cap_desc->string_index[i]);
				}
				uart_puts(")\r\n");
			}
		}
		break;

	case USB_DESC_HID_REPORT:
		// Route by interface number (wIndex)
		{
			uint16_t iface_num = setup->wIndex;
			for (uint8_t i = 0; i < cap_desc->num_ifaces; i++) {
				if (cap_desc->ifaces[i].iface_num == iface_num &&
				    cap_desc->ifaces[i].iface_class == 3) {
					data = cap_desc->ifaces[i].hid_report_desc;
					len = cap_desc->ifaces[i].hid_report_desc_len;
					break;
				}
			}
		}
		break;

	case USB_DESC_HID:
		// HID class descriptor — find the one matching wIndex (interface number)
		{
			uint16_t target_iface = setup->wIndex;
			const uint8_t *p = cap_desc->config_desc;
			const uint8_t *end_ptr = p + cap_desc->config_desc_len;
			uint8_t current_iface_num = 0xFF;

			while (p < end_ptr) {
				uint8_t dlen = p[0];
				uint8_t dtype = p[1];
				if (dlen < 2 || p + dlen > end_ptr) break;

				if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
					current_iface_num = p[2];
				} else if (dtype == USB_DESC_HID &&
				           current_iface_num == target_iface) {
					data = p;
					len = dlen;
					break;
				}
				p += dlen;
			}
		}
		break;

	default:
		break;
	}

	if (data == NULL || len == 0) {
		ep0_stall();
		return;
	}

	if (len > max_len) len = max_len;
	ep0_tx_data(data, len);
}

static void handle_set_address(const usb_setup_t *setup)
{
	uint8_t addr = setup->wValue & 0x7F;

	// USBADRA: hardware applies address after status stage completes
	USB1_DEVICEADDR = USB_DEVICEADDR_USBADR(addr) | USB_DEVICEADDR_USBADRA;

	ep0_tx_data(NULL, 0);
	dev_state = USB_DEV_STATE_ADDRESS;
}

static void configure_all_interrupt_endpoints(void)
{
	num_int_eps = 0;
	memset(ep_to_slot, 0xFF, sizeof(ep_to_slot));
	ep_busy_mask = 0;
	active_bank_mask = 0;
	memset(pending_len, 0, sizeof(pending_len));

	for (uint8_t i = 0; i < cap_desc->num_ifaces; i++) {
		const captured_iface_t *iface = &cap_desc->ifaces[i];
		if (iface->interrupt_ep == 0) continue;

		uint8_t ep_num = iface->interrupt_ep & 0x0F;
		if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) continue;
		if (num_int_eps >= MAX_INT_EPS) break;

		// Check for EP collision
		if (ep_to_slot[ep_num] != 0xFF) {
			uart_puts("  DEV: EP collision on EP");
			uart_putdec(ep_num);
			uart_puts(", skipping\r\n");
			continue;
		}

		uint8_t slot = num_int_eps++;
		ep_to_slot[ep_num] = slot;

		uint16_t maxpkt = iface->interrupt_maxpkt;
		uint8_t qh_idx = ep_num * 2 + 1;

		memset(&dqh_list[qh_idx], 0, sizeof(usb_dev_dqh_t));
		dqh_list[qh_idx].config = DQH_MAX_PACKET(maxpkt) | DQH_ZLT_DISABLE;
		dqh_list[qh_idx].next = DTD_TERMINATE;
		asm volatile("dsb" ::: "memory");

		volatile uint32_t *epctrl = endptctrl_reg(ep_num);
		*epctrl = USB_ENDPTCTRL_TXE | USB_ENDPTCTRL_TXT(3) | USB_ENDPTCTRL_TXR;
	}
}

static void handle_set_configuration(const usb_setup_t *setup)
{
	uint8_t config_val = setup->wValue & 0xFF;

	if (config_val == 0) {
		dev_state = USB_DEV_STATE_ADDRESS;
		// Flush all interrupt EPs
		for (uint8_t i = 0; i < cap_desc->num_ifaces; i++) {
			uint8_t ep_num = cap_desc->ifaces[i].interrupt_ep & 0x0F;
			if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) continue;
			USB1_ENDPTFLUSH = (1 << (16 + ep_num));
			while (USB1_ENDPTFLUSH) ;
			volatile uint32_t *epctrl = endptctrl_reg(ep_num);
			*epctrl = 0;
		}
		num_int_eps = 0;
		memset(ep_to_slot, 0xFF, sizeof(ep_to_slot));
		ep_busy_mask = 0;
	} else {
		configure_all_interrupt_endpoints();
		dev_state = USB_DEV_STATE_CONFIGURED;
	}

	ep0_tx_data(NULL, 0);
}

static void handle_get_status(const usb_setup_t *setup)
{
	(void)setup;
	static const uint8_t status_buf[2] = {0, 0};
	ep0_tx_data(status_buf, 2);
}

static void handle_standard_request(const usb_setup_t *setup)
{
	switch (setup->bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		handle_get_descriptor(setup);
		break;
	case USB_REQ_SET_ADDRESS:
		handle_set_address(setup);
		break;
	case USB_REQ_SET_CONFIG:
		handle_set_configuration(setup);
		break;
	case 0x00: // GET_STATUS
		handle_get_status(setup);
		break;
	case 0x01: // CLEAR_FEATURE
	case 0x03: // SET_FEATURE
		ep0_tx_data(NULL, 0); // Accept silently
		break;
	case 0x08: { // GET_CONFIGURATION
		uint8_t cfg = (dev_state == USB_DEV_STATE_CONFIGURED)
			? cap_desc->config_desc[5] : 0; // bConfigurationValue
		ep0_tx_data(&cfg, 1);
		break;
	}
	case 0x0A: // GET_INTERFACE
		{
			// Return alternate setting 0 for any interface
			static const uint8_t alt = 0;
			ep0_tx_data(&alt, 1);
		}
		break;
	case 0x0B: // SET_INTERFACE
		// Accept alt setting 0 silently (composite HID needs this)
		ep0_tx_data(NULL, 0);
		break;
	default:
		ep0_stall();
		break;
	}
}

static void handle_class_request(const usb_setup_t *setup)
{
	switch (setup->bRequest) {
	case 0x0A: // SET_IDLE
		ep0_tx_data(NULL, 0);
		break;
	case 0x0B: // SET_PROTOCOL
		ep0_tx_data(NULL, 0);
		break;
	case 0x01: // GET_REPORT
		{
			uint16_t len = setup->wLength;
			if (len > sizeof(ep0_tx_buf)) len = sizeof(ep0_tx_buf);
			memset(ep0_tx_buf, 0, len);
			ep0_tx_data(ep0_tx_buf, len);
		}
		break;
	case 0x09: // SET_REPORT
		// Accept and discard — send ZLP status
		ep0_tx_data(NULL, 0);
		break;
	case 0x03: // GET_PROTOCOL
		{
			static const uint8_t proto = 1; // Report protocol
			ep0_tx_data(&proto, 1);
		}
		break;
	default:
		ep0_stall();
		break;
	}
}

// ---- SETUP packet handling ----

static void handle_setup_packet(void)
{
	usb_setup_t setup;

	// Read SETUP data using SUTW semaphore
	do {
		USB1_USBCMD |= USB_USBCMD_SUTW;
		uint32_t s0 = dqh_list[0].setup0;
		uint32_t s1 = dqh_list[0].setup1;
		setup.bmRequestType = s0 & 0xFF;
		setup.bRequest      = (s0 >> 8) & 0xFF;
		setup.wValue        = (s0 >> 16) & 0xFFFF;
		setup.wIndex        = s1 & 0xFFFF;
		setup.wLength       = (s1 >> 16) & 0xFFFF;
	} while (!(USB1_USBCMD & USB_USBCMD_SUTW));

	USB1_USBCMD &= ~USB_USBCMD_SUTW;

	// Acknowledge SETUP
	USB1_ENDPTSETUPSTAT = 1;
	while (USB1_ENDPTSETUPSTAT & 1) ;

	// Flush stale EP0 transfers
	USB1_ENDPTFLUSH = (1 << 16) | (1 << 0); // Flush EP0 TX and RX
	while (USB1_ENDPTFLUSH) ;

	// Dispatch based on request type (bits 6:5 of bmRequestType)
	uint8_t req_type = setup.bmRequestType & 0x60;

	if (req_type == 0x00) {
		handle_standard_request(&setup);
	} else if (req_type == 0x20) {
		handle_class_request(&setup);
	} else {
		ep0_stall();
	}
}

// ---- Bus reset ----

static void handle_bus_reset(void)
{
	USB1_ENDPTSETUPSTAT = USB1_ENDPTSETUPSTAT;
	USB1_ENDPTCOMPLETE = USB1_ENDPTCOMPLETE;

	while (USB1_ENDPTPRIME) ;
	USB1_ENDPTFLUSH = 0xFFFFFFFF;
	while (USB1_ENDPTFLUSH) ;

	// Reset address
	USB1_DEVICEADDR = 0;
	dev_state = USB_DEV_STATE_DEFAULT;

	// Re-init EP0 dQH
	dqh_list[0].next = DTD_TERMINATE;
	dqh_list[0].token = 0;
	dqh_list[1].next = DTD_TERMINATE;
	dqh_list[1].token = 0;
	asm volatile("dsb" ::: "memory");

	ep_busy_mask = 0;
	active_bank_mask = 0;
	memset(pending_len, 0, sizeof(pending_len));
	num_int_eps = 0;
	memset(ep_to_slot, 0xFF, sizeof(ep_to_slot));
}

// ---- Public API ----

bool usb_device_init(const captured_descriptors_t *desc)
{
	cap_desc = desc;
	dev_state = USB_DEV_STATE_DEFAULT;
	ep_busy_mask = 0;
	active_bank_mask = 0;
	memset(ep_to_slot, 0xFF, sizeof(ep_to_slot));
	memset(pending_len, 0, sizeof(pending_len));
	num_int_eps = 0;

	// Match PJRC core USB bring-up power rail settings.
	PMU_REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) |
		PMU_REG_3P0_BO_OFFSET(6) |
		PMU_REG_3P0_ENABLE_LINREG;

	// Gate clock for USB1/USB2 controller block (USBOH3).
	CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);

	// Recommended bus burst settings used by Teensy core.
	// Set before conditional reset since the unconditional path doesn't
	// reset the controller again (matching PJRC usb.c sequence).
	USB1_BURSTSIZE = 0x0404;

	// If PHY was previously in use, perform a clean-slate reset
	if ((USBPHY1_PWD & (USBPHY_PWD_RXPWDRX | USBPHY_PWD_RXPWDDIFF |
		USBPHY_PWD_RXPWD1PT1 | USBPHY_PWD_RXPWDENV |
		USBPHY_PWD_TXPWDV2I | USBPHY_PWD_TXPWDIBIAS |
		USBPHY_PWD_TXPWDFS)) || (USB1_USBMODE & USB_USBMODE_CM_MASK)) {
		USBPHY1_CTRL_SET = USBPHY_CTRL_SFTRST;
		USB1_USBCMD |= USB_USBCMD_RST;
		uint32_t rto = 4000000u;
		while ((USB1_USBCMD & USB_USBCMD_RST) && --rto) ;
		USBPHY1_CTRL_CLR = USBPHY_CTRL_SFTRST;
		delay(25);
	}

	// Power up USBPHY1 (match PJRC usb.c — only clkgate + pwd)
	USBPHY1_CTRL_CLR = USBPHY_CTRL_CLKGATE;
	USBPHY1_PWD = 0;

	// Set device mode with setup lockout (must be done right after reset)
	USB1_USBMODE = USB_USBMODE_CM(2) | USB_USBMODE_SLOM;

	// 4. Initialize dQH array
	memset(dqh_list, 0, sizeof(dqh_list));

	// EP0 RX (OUT) — index 0: maxpkt=64, IOS for setup interrupt
	dqh_list[0].config = DQH_MAX_PACKET(64) | DQH_IOS;
	dqh_list[0].next = DTD_TERMINATE;

	// EP0 TX (IN) — index 1: maxpkt=64
	dqh_list[1].config = DQH_MAX_PACKET(64);
	dqh_list[1].next = DTD_TERMINATE;

	asm volatile("dsb" ::: "memory");

	// 5. Set endpoint list address
	USB1_ENDPOINTLISTADDR = (uint32_t)dqh_list;

	// 6. Enable core USB status sources (we still poll USBSTS).
	USB1_USBINTR = USB_USBINTR_UE | USB_USBINTR_UEE |
		USB_USBINTR_URE | USB_USBINTR_SLE;

	// 7. Clear all pending status
	USB1_USBSTS = USB1_USBSTS;

	// 8. Start controller (direct assignment, not |=, per PJRC usb.c)
	USB1_USBCMD = USB_USBCMD_RS;

	return true;
}

void usb_device_poll(void)
{
	uint32_t status = USB1_USBSTS;
	USB1_USBSTS = status; // Write-to-clear

	// Bus reset
	if (status & USB_USBSTS_URI) {
		handle_bus_reset();
	}

	// Transfer complete / SETUP
	if (status & USB_USBSTS_UI) {
		// Check for SETUP packet on EP0
		if (USB1_ENDPTSETUPSTAT & 1) {
			handle_setup_packet();
		}

		// Check for endpoint completions
		uint32_t complete = USB1_ENDPTCOMPLETE;
		if (complete) {
			USB1_ENDPTCOMPLETE = complete;

			// Mask completed TX EPs against busy set — only process real completions
			uint8_t done = (uint8_t)(complete >> 16) & ep_busy_mask;
			ep_busy_mask &= ~done; // clear all done EPs in one shot

			// Iterate only the completed EPs via CTZ (no scanning idle EPs)
			while (done) {
				uint8_t ep = (uint8_t)__builtin_ctz(done);
				done &= done - 1; // clear lowest set bit

				// If a report was staged in the other bank, send it now
				if (pending_len[ep] > 0) {
					uint8_t slot = ep_to_slot[ep];
					uint8_t bank = ((active_bank_mask >> ep) ^ 1) & 1;
					prime_int_ep(ep, slot, bank, pending_len[ep]);
					pending_len[ep] = 0;
				}
			}
		}
	}
}

bool usb_device_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
	if (dev_state != USB_DEV_STATE_CONFIGURED) return false;
	if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) return false;

	uint8_t slot = ep_to_slot[ep_num];
	if (slot >= MAX_INT_EPS) return false;
	if (len > 64) len = 64;

	uint8_t ep_bit = (1 << ep_num);
	if (ep_busy_mask & ep_bit) {
		// EP is in-flight — stage report in the other bank (overwrites stale pending)
		uint8_t bank = ((active_bank_mask >> ep_num) ^ 1) & 1;
		memcpy(int_tx_buf[slot][bank], data, len);
		pending_len[ep_num] = (uint8_t)len;
		return true; // staged, not dropped
	}

	// EP is free — send immediately from current bank
	uint8_t bank = (active_bank_mask >> ep_num) & 1;
	memcpy(int_tx_buf[slot][bank], data, len);
	prime_int_ep(ep_num, slot, bank, len);
	return true;
}

bool usb_device_is_configured(void)
{
	return dev_state == USB_DEV_STATE_CONFIGURED;
}
