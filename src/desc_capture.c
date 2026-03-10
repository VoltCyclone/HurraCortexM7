// USB descriptor capture — sequential blocking enumeration
#include <string.h>
#include "imxrt.h"
#include "usb_host.h"
#include "desc_capture.h"
#include "uart.h"

extern void delay(uint32_t msec);

static usb_setup_t make_get_descriptor(uint8_t type, uint8_t index,
	uint16_t langid, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x80; // Device-to-Host, Standard, Device
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = langid;
	s.wLength = length;
	return s;
}

static usb_setup_t make_get_iface_descriptor(uint8_t type, uint8_t index,
	uint16_t iface, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x81; // Device-to-Host, Standard, Interface
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = iface;
	s.wLength = length;
	return s;
}

static bool parse_config_descriptor(captured_descriptors_t *desc)
{
	const uint8_t *p = desc->config_desc;
	const uint8_t *end = p + desc->config_desc_len;
	captured_iface_t *cur_iface = NULL;
	desc->num_ifaces = 0;

	while (p < end) {
		uint8_t dlen = p[0];
		uint8_t dtype = p[1];

		if (dlen < 2 || p + dlen > end) break;

		if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
			uint8_t alt_setting = p[3];
			if (alt_setting != 0) {
				cur_iface = NULL;
				p += dlen;
				continue;
			}

			if (desc->num_ifaces < MAX_INTERFACES) {
				cur_iface = &desc->ifaces[desc->num_ifaces++];
				memset(cur_iface, 0, sizeof(*cur_iface));
				cur_iface->iface_num      = p[2];
				cur_iface->iface_class    = p[5];
				cur_iface->iface_subclass = p[6];
				cur_iface->iface_protocol = p[7];
			} else {
				cur_iface = NULL;
				uart_puts("  WARNING: too many interfaces, skipping\r\n");
			}
		} else if (dtype == USB_DESC_HID && dlen >= 9 && cur_iface != NULL) {
			if (cur_iface->iface_class == 3) {
				uint8_t num_descs = p[5];
				for (uint8_t i = 0; i < num_descs; i++) {
					if (6 + i * 3 + 2 < dlen) {
						uint8_t rtype = p[6 + i * 3];
						uint16_t rlen = p[7 + i * 3] | (p[8 + i * 3] << 8);
						if (rtype == USB_DESC_HID_REPORT) {
							cur_iface->hid_report_desc_len = rlen;
						}
					}
				}
			}
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && cur_iface != NULL) {
			uint8_t ep_addr = p[2];
			uint8_t ep_attr = p[3];
			uint16_t ep_maxpkt = p[4] | (p[5] << 8);
			uint8_t ep_interval = p[6];
			if ((ep_attr & 3) == 3 && (ep_addr & 0x80)) {
				cur_iface->interrupt_ep       = ep_addr;
				cur_iface->interrupt_maxpkt   = ep_maxpkt;
				cur_iface->interrupt_interval = ep_interval;
			}
		}

		p += dlen;
	}

	return desc->num_ifaces > 0;
}

bool capture_descriptors(captured_descriptors_t *desc)
{
	memset(desc, 0, sizeof(*desc));
	int ret;
	usb_setup_t setup;
	setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 8);
	ret = usb_host_control_transfer(0, 8, &setup, desc->device_desc, 2000);
	if (ret < 0 || ret < 8 || desc->device_desc[0] != 18 ||
	    desc->device_desc[1] != USB_DESC_DEVICE) {
		uart_puts("FAIL: could not read device descriptor (8 bytes)\r\n");
		return false;
	}
	desc->ep0_maxpkt = desc->device_desc[7]; // bMaxPacketSize0
	desc->dev_addr = 1;
	setup.bmRequestType = 0x00; // Host-to-Device, Standard, Device
	setup.bRequest = USB_REQ_SET_ADDRESS;
	setup.wValue = desc->dev_addr;
	setup.wIndex = 0;
	setup.wLength = 0;
	ret = usb_host_control_transfer(0, desc->ep0_maxpkt, &setup, NULL, 2000);
	if (ret < 0) {
		uart_puts("FAIL: SET_ADDRESS\r\n");
		return false;
	}
	delay(10); // Device needs time to process new address
	setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 18);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->device_desc, 2000);
	if (ret < 0 || ret < 18 || desc->device_desc[0] != 18 ||
	    desc->device_desc[1] != USB_DESC_DEVICE) {
		uart_puts("FAIL: full device descriptor\r\n");
		return false;
	}
	desc->device_desc_len = 18;
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, 9);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc, 2000);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
		uart_puts("FAIL: config descriptor header\r\n");
		return false;
	}
	uint16_t total_len = desc->config_desc[2] | (desc->config_desc[3] << 8);

	if (total_len > MAX_CONFIG_DESC_SIZE) {
		uart_puts("  WARNING: truncating to ");
		uart_putdec(MAX_CONFIG_DESC_SIZE);
		uart_puts("\r\n");
		total_len = MAX_CONFIG_DESC_SIZE;
	}
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, total_len);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc, 2000);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
		uart_puts("FAIL: full config descriptor\r\n");
		return false;
	}
	desc->config_desc_len = (uint16_t)ret;
	parse_config_descriptor(desc);
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		captured_iface_t *iface = &desc->ifaces[i];
		if (iface->iface_class != 3) continue;
		if (iface->hid_report_desc_len == 0) continue;

		uint16_t rdlen = iface->hid_report_desc_len;
		if (rdlen > MAX_HID_REPORT_DESC_SIZE)
			rdlen = MAX_HID_REPORT_DESC_SIZE;

		setup = make_get_iface_descriptor(USB_DESC_HID_REPORT, 0,
			iface->iface_num, rdlen);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, iface->hid_report_desc, 2000);
		if (ret < 0) {
			uart_puts("    FAIL\r\n");
			iface->hid_report_desc_len = 0;
		} else {
			iface->hid_report_desc_len = ret;
		}
	}

	uint8_t str_buf[MAX_STRING_DESC_SIZE];
	setup = make_get_descriptor(USB_DESC_STRING, 0, 0, 4);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, str_buf, 2000);
	uint16_t langid = 0x0409; // Default to English
	if (ret >= 4 && str_buf[1] == USB_DESC_STRING) {
		langid = str_buf[2] | (str_buf[3] << 8);
	}
	uint8_t string_indices[3] = {
		desc->device_desc[14], // iManufacturer
		desc->device_desc[15], // iProduct
		desc->device_desc[16], // iSerialNumber
	};

	desc->num_strings = 0;
	for (int i = 0; i < 3; i++) {
		if (string_indices[i] == 0) continue;
		if (desc->num_strings >= MAX_STRINGS) break;

		setup = make_get_descriptor(USB_DESC_STRING, string_indices[i],
			langid, MAX_STRING_DESC_SIZE);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->string_desc[desc->num_strings], 2000);
		if (ret > 0) {
			desc->string_desc_len[desc->num_strings] = ret;
			desc->string_index[desc->num_strings] = string_indices[i];
			desc->num_strings++;
		}
	}

	desc->valid = true;
	return true;
}

void dump_descriptors(const captured_descriptors_t *desc)
{
	uart_puts("\r\n========== DEVICE DESCRIPTOR (");
	uart_putdec(desc->device_desc_len);
	uart_puts(" bytes) ==========\r\n");
	uart_hexdump(desc->device_desc, desc->device_desc_len);

	uart_puts("\r\n========== CONFIGURATION DESCRIPTOR (");
	uart_putdec(desc->config_desc_len);
	uart_puts(" bytes) ==========\r\n");
	uart_hexdump(desc->config_desc, desc->config_desc_len);

	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		const captured_iface_t *iface = &desc->ifaces[i];
		uart_puts("\r\n========== INTERFACE ");
		uart_putdec(iface->iface_num);
		uart_puts(" (class=");
		uart_putdec(iface->iface_class);
		uart_puts(") ==========\r\n");

		if (iface->interrupt_ep) {
			uart_puts("  Interrupt IN EP: 0x");
			uart_puthex8(iface->interrupt_ep);
			uart_puts(", maxpkt=");
			uart_putdec(iface->interrupt_maxpkt);
			uart_puts(", interval=");
			uart_putdec(iface->interrupt_interval);
			uart_puts("ms\r\n");
		}

		if (iface->hid_report_desc_len > 0) {
			uart_puts("  HID Report Descriptor (");
			uart_putdec(iface->hid_report_desc_len);
			uart_puts(" bytes):\r\n");
			uart_hexdump(iface->hid_report_desc, iface->hid_report_desc_len);
		}
	}

	for (uint8_t i = 0; i < desc->num_strings; i++) {
		uart_puts("\r\n========== STRING DESCRIPTOR ");
		uart_putdec(i);
		uart_puts(" (");
		uart_putdec(desc->string_desc_len[i]);
		uart_puts(" bytes) ==========\r\n");
		uart_hexdump(desc->string_desc[i], desc->string_desc_len[i]);
	}

	uart_puts("\r\n========== SUMMARY ==========\r\n");
	uart_puts("EP0 max packet: ");
	uart_putdec(desc->ep0_maxpkt);
	uart_puts("\r\nInterfaces: ");
	uart_putdec(desc->num_ifaces);
	uart_puts("\r\n");
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		uart_puts("  [");
		uart_putdec(i);
		uart_puts("] iface=");
		uart_putdec(desc->ifaces[i].iface_num);
		uart_puts(" class=");
		uart_putdec(desc->ifaces[i].iface_class);
		if (desc->ifaces[i].interrupt_ep) {
			uart_puts(" EP=0x");
			uart_puthex8(desc->ifaces[i].interrupt_ep);
		}
		uart_puts("\r\n");
	}
}
