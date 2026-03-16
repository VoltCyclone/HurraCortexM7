// USB Device controller driver for Teensy 4.1 USB1 port
// Replays captured descriptors and forwards HID reports.
// Supports composite devices with multiple interrupt IN endpoints.
//
// USB1_USBINTR enables (UE|UEE|URE|SLE) cause the controller to assert
// its IRQ line on bus events.  No ISR is installed — the NVIC pending
// bit alone wakes WFE via SEVONPEND (set in main).  usb_device_poll()
// reads and clears USB1_USBSTS, which deasserts the IRQ line and
// clears the NVIC pending bit (level-sensitive).
//
// DMA buffers are in a non-cacheable MPU region (.dmabuffers at 0x20200000).
// No manual cache maintenance needed — bare DSBs drain the write buffer
// before the USB controller is told to read DMA structures.

#include <string.h>
#include "imxrt.h"
#include "usb_device.h"
#include "usb_host.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);
static usb_dev_dqh_t dqh_list[USB_DEV_NUM_ENDPOINTS * 2]
	__attribute__((section(".dmabuffers"), aligned(4096)));

// dTDs — one for EP0 TX, one for EP0 RX (status), one per interrupt IN endpoint
static usb_dev_dtd_t dtd_ep0_tx  __attribute__((section(".dmabuffers"), aligned(32)));
static usb_dev_dtd_t dtd_ep0_rx  __attribute__((section(".dmabuffers"), aligned(32)));
static usb_dev_dtd_t dtd_int_tx[MAX_INT_EPS]
	__attribute__((section(".dmabuffers"), aligned(32)));

// Data buffers — double-banked: USB DMA reads bank[active], CPU writes bank[!active]
static uint8_t ep0_tx_buf[512] __attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t ep0_rx_buf[512] __attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t int_tx_buf[MAX_INT_EPS][2][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
static const captured_descriptors_t *cap_desc;
static usb_dev_state_t dev_state;
static uint8_t ep_to_slot[USB_DEV_NUM_ENDPOINTS]; // EP num -> dtd/buf slot
static uint8_t num_int_eps;
static uint8_t ep_busy_mask;     // bit set = EP has active DMA transfer in flight
static uint8_t active_bank_mask; // bit set = EP using bank 1, clear = bank 0
static uint8_t pending_len[USB_DEV_NUM_ENDPOINTS];  // 0 = no pending report staged

// Deferred passthrough: SET_REPORT data queued for async forwarding to real device.
// Avoids blocking the setup handler (and stalling interrupt EP polling).
static struct {
	usb_setup_t setup;
	uint8_t     data[64];
	uint16_t    data_len;
	bool        pending;
} deferred_out;

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

static void handle_passthrough(const usb_setup_t *setup);

static void ep0_stall(void)
{
	USB1_ENDPTCTRL0 |= USB_ENDPTCTRL_TXS | USB_ENDPTCTRL_RXS;
}
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
			static const uint8_t lang_desc[] = {0x04, 0x03, 0x09, 0x04};
			data = lang_desc;
			len = 4;
		} else {
			for (uint8_t i = 0; i < cap_desc->num_strings; i++) {
				if (cap_desc->string_index[i] == desc_index) {
					data = cap_desc->string_desc[i];
					len = cap_desc->string_desc_len[i];
					break;
				}
			}
			if (data == NULL) { /* string not found */ }
		}
		break;

	case USB_DESC_HID_REPORT:
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
		if (ep_to_slot[ep_num] != 0xFF) continue;

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
	case 0x01: // GET_REPORT — return zeros (real data flows via interrupt EPs)
		{
			uint16_t len = setup->wLength;
			if (len > sizeof(ep0_tx_buf)) len = sizeof(ep0_tx_buf);
			memset(ep0_tx_buf, 0, len);
			ep0_tx_data(ep0_tx_buf, len);
		}
		break;
	case 0x09: // SET_REPORT — forward to real device (LED control, etc.)
		handle_passthrough(setup);
		return;
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
// Blocking receive of EP0 OUT data phase (for control transfers with host-to-device data).
// Returns bytes received, or -1 on error/timeout.
static int ep0_rx_data(uint16_t max_len)
{
	if (max_len > sizeof(ep0_rx_buf)) max_len = sizeof(ep0_rx_buf);

	memset(&dtd_ep0_rx, 0, sizeof(dtd_ep0_rx));
	dtd_ep0_rx.next  = DTD_TERMINATE;
	dtd_ep0_rx.token = DTD_ACTIVE | DTD_IOC | DTD_TOTAL_BYTES(max_len);
	dtd_ep0_rx.buffer[0] = (uint32_t)ep0_rx_buf;
	dtd_ep0_rx.buffer[1] = ((uint32_t)ep0_rx_buf + 4096) & ~0xFFFu;

	dqh_list[0].next  = (uint32_t)&dtd_ep0_rx;
	dqh_list[0].token = 0;
	asm volatile("dsb" ::: "memory");

	USB1_ENDPTPRIME = (1 << 0); // Prime EP0 RX

	uint32_t start = millis();
	while (1) {
		uint32_t token = dtd_ep0_rx.token;
		if (!(token & DTD_ACTIVE)) {
			if (token & (DTD_HALTED | DTD_BUFFER_ERR | DTD_XACT_ERR))
				return -1;
			uint32_t remaining = (token >> 16) & 0x7FFF;
			return (int)(max_len - remaining);
		}
		if ((millis() - start) > 50) return -1;
	}
}

// Forward a control request to the real USB device and relay the response.
// IN requests block briefly (rare — vendor IN only, GET_REPORT handled locally).
// OUT requests are deferred: ACK the computer immediately, forward next poll cycle.
static void handle_passthrough(const usb_setup_t *setup)
{
	bool is_in = (setup->bmRequestType & 0x80) != 0;
	uint16_t wLength = setup->wLength;

	if (is_in) {
		// Device-to-host: forward to real device, return response.
		// Wait for any in-flight fire-and-forget to finish first, but bound wait time.
		uint32_t start = millis();
		while (usb_host_control_async_busy()) {
			if ((millis() - start) > 200) {
				ep0_stall();
				return;
			}
		}
		int ret = usb_host_control_transfer(cap_desc->dev_addr,
			cap_desc->ep0_maxpkt, setup, ep0_rx_buf, 200);
		if (ret < 0) {
			ep0_stall();
			return;
		}
		ep0_tx_data(ep0_rx_buf, (uint16_t)ret);
	} else if (wLength > 0) {
		// Host-to-device with data: receive data, ACK immediately, defer forward
		int rxd = ep0_rx_data(wLength);
		if (rxd < 0) {
			ep0_stall();
			return;
		}
		ep0_tx_data(NULL, 0); // ACK computer now — don't block
		// Queue for forwarding in next poll cycle
		if (rxd <= (int)sizeof(deferred_out.data)) {
			memcpy(&deferred_out.setup, setup, sizeof(*setup));
			memcpy(deferred_out.data, ep0_rx_buf, rxd);
			deferred_out.data_len = (uint16_t)rxd;
			// Ensure forwarded setup uses the actual received length
			deferred_out.setup.wLength = deferred_out.data_len;
			deferred_out.pending = true;
		}
	} else {
		// No data phase: ACK immediately, defer forward
		ep0_tx_data(NULL, 0);
		memcpy(&deferred_out.setup, setup, sizeof(*setup));
		deferred_out.data_len = 0;
		deferred_out.pending = true;
	}
}

static void handle_setup_packet(void)
{
	usb_setup_t setup;
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
	USB1_ENDPTSETUPSTAT = 1;
	while (USB1_ENDPTSETUPSTAT & 1) ;
	USB1_ENDPTFLUSH = (1 << 16) | (1 << 0); // Flush EP0 TX and RX
	while (USB1_ENDPTFLUSH) ;
	uint8_t req_type = setup.bmRequestType & 0x60;

	if (req_type == 0x00) {
		handle_standard_request(&setup);
	} else if (req_type == 0x20) {
		handle_class_request(&setup);
	} else {
		// Vendor and other request types: pass through to real device
		handle_passthrough(&setup);
	}
}
static void handle_bus_reset(void)
{
	USB1_ENDPTSETUPSTAT = USB1_ENDPTSETUPSTAT;
	USB1_ENDPTCOMPLETE = USB1_ENDPTCOMPLETE;

	while (USB1_ENDPTPRIME) ;
	USB1_ENDPTFLUSH = 0xFFFFFFFF;
	while (USB1_ENDPTFLUSH) ;
	USB1_DEVICEADDR = 0;
	dev_state = USB_DEV_STATE_DEFAULT;
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
	CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);
	USB1_BURSTSIZE = 0x0404;
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
	USBPHY1_CTRL_CLR = USBPHY_CTRL_CLKGATE;
	USBPHY1_PWD = 0;
	USB1_USBMODE = USB_USBMODE_CM(2) | USB_USBMODE_SLOM;
	memset(dqh_list, 0, sizeof(dqh_list));
	dqh_list[0].config = DQH_MAX_PACKET(64) | DQH_IOS;
	dqh_list[0].next = DTD_TERMINATE;
	dqh_list[1].config = DQH_MAX_PACKET(64);
	dqh_list[1].next = DTD_TERMINATE;
	asm volatile("dsb" ::: "memory");
	USB1_ENDPOINTLISTADDR = (uint32_t)dqh_list;
	USB1_USBINTR = USB_USBINTR_UE | USB_USBINTR_UEE |
		USB_USBINTR_URE | USB_USBINTR_SLE;
	USB1_USBSTS = USB1_USBSTS;
	USB1_USBCMD = USB_USBCMD_RS;
	return true;
}

void usb_device_poll(void)
{
	uint32_t status = USB1_USBSTS;
	USB1_USBSTS = status; // Write-to-clear
	if (__builtin_expect(status & USB_USBSTS_URI, 0)) {
		handle_bus_reset();
		deferred_out.pending = false; // discard on reset
	}
	if (status & USB_USBSTS_UI) {
		if (USB1_ENDPTSETUPSTAT & 1) {
			handle_setup_packet();
		}
		uint32_t complete = USB1_ENDPTCOMPLETE;
		if (complete) {
			USB1_ENDPTCOMPLETE = complete;
			uint8_t done = (uint8_t)(complete >> 16) & ep_busy_mask;
			ep_busy_mask &= ~done; // clear all done EPs in one shot
			while (done) {
				uint8_t ep = (uint8_t)__builtin_ctz(done);
				done &= done - 1; // clear lowest set bit
				if (pending_len[ep] > 0) {
					uint8_t slot = ep_to_slot[ep];
					uint8_t bank = ((active_bank_mask >> ep) ^ 1) & 1;
					prime_int_ep(ep, slot, bank, pending_len[ep]);
					pending_len[ep] = 0;
				}
			}
		}
	}

	// Forward deferred passthrough requests to real device (fire-and-forget).
	// Skip if a previous async transfer is still in flight — try again next poll.
	if (__builtin_expect(deferred_out.pending, 0) &&
	    !usb_host_control_async_busy()) {
		deferred_out.pending = false;
		usb_host_control_transfer_fire(cap_desc->dev_addr,
			cap_desc->ep0_maxpkt, &deferred_out.setup,
			deferred_out.data_len > 0 ? deferred_out.data : NULL);
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
		uint8_t bank = ((active_bank_mask >> ep_num) ^ 1) & 1;
		memcpy(int_tx_buf[slot][bank], data, len);
		pending_len[ep_num] = (uint8_t)len;
		return true; // staged, not dropped
	}
	uint8_t bank = (active_bank_mask >> ep_num) & 1;
	memcpy(int_tx_buf[slot][bank], data, len);
	prime_int_ep(ep_num, slot, bank, len);
	return true;
}

bool usb_device_is_configured(void)
{
	return dev_state == USB_DEV_STATE_CONFIGURED;
}
