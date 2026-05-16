// USB descriptor capture
#include <string.h>
#include "imxrt.h"
#include "usb_host.h"
#include "desc_capture.h"

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
			// Skip alternate settings — assumes alt 0 precedes higher alts
			// (true for all compliant devices per USB spec ordering)
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
		return false;
	}
	desc->ep0_maxpkt = desc->device_desc[7]; // bMaxPacketSize0
	if (desc->ep0_maxpkt != 8  && desc->ep0_maxpkt != 16 &&
	    desc->ep0_maxpkt != 32 && desc->ep0_maxpkt != 64) {
		return false;
	}
	desc->dev_addr = 1;
	setup.bmRequestType = 0x00; // Host-to-Device, Standard, Device
	setup.bRequest = USB_REQ_SET_ADDRESS;
	setup.wValue = desc->dev_addr;
	setup.wIndex = 0;
	setup.wLength = 0;
	ret = usb_host_control_transfer(0, desc->ep0_maxpkt, &setup, NULL, 2000);
	if (ret < 0) return false;
	delay(2);  // Let status phase complete at address 0
	delay(10); // Device needs time to process new address
	setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 18);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->device_desc, 2000);
	if (ret < 0 || ret < 18 || desc->device_desc[0] != 18 ||
	    desc->device_desc[1] != USB_DESC_DEVICE) {
		return false;
	}
	desc->device_desc_len = 18;
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, 9);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc, 2000);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
		return false;
	}
	uint16_t total_len = desc->config_desc[2] | (desc->config_desc[3] << 8);
	if (total_len < 9) return false;
	if (total_len > MAX_CONFIG_DESC_SIZE)
		total_len = MAX_CONFIG_DESC_SIZE;
	setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, total_len);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->config_desc, 2000);
	if (ret < 0 || ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
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
			iface->hid_report_desc_len = 0;
		} else {
			iface->hid_report_desc_len = (uint16_t)ret;
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

	setup.bmRequestType = 0x00;
	setup.bRequest = USB_REQ_SET_CONFIG;
	setup.wValue = desc->config_desc[5]; // bConfigurationValue
	setup.wIndex = 0;
	setup.wLength = 0;
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, NULL, 2000);
	if (ret < 0) return false;

	// SET_IDLE for each HID interface — report only on change
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (desc->ifaces[i].iface_class != 3) continue;
		setup.bmRequestType = 0x21; // Host-to-Device, Class, Interface
		setup.bRequest = 0x0A;      // HID SET_IDLE
		setup.wValue = 0;           // duration=0, report_id=0
		setup.wIndex = desc->ifaces[i].iface_num;
		setup.wLength = 0;
		usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, NULL, 2000);
		// Don't fail on error, some devices STALL SET_IDLE legally
	}

	desc->valid = true;
	return true;
}

