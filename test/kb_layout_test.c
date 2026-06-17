// Host-native test for src/kb_layout.c

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "kb_layout.h"

int main(void)
{
	int failures = 0;
	kb_layout_t layout;

	// Standard 8-byte boot keyboard report descriptor (no Report ID).
	const uint8_t boot_desc[] = {
		0x05, 0x01,       // Usage Page (Generic Desktop)
		0x09, 0x06,       // Usage (Keyboard)
		0xA1, 0x01,       // Collection (Application)
		0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
		0x19, 0xE0,       //   Usage Minimum (224)
		0x29, 0xE7,       //   Usage Maximum (231)
		0x15, 0x00,       //   Logical Minimum (0)
		0x25, 0x01,       //   Logical Maximum (1)
		0x75, 0x01,       //   Report Size (1)
		0x95, 0x08,       //   Report Count (8)
		0x81, 0x02,       //   Input (Data,Var,Abs)
		0x95, 0x01,       //   Report Count (1)
		0x75, 0x08,       //   Report Size (8)
		0x81, 0x01,       //   Input (Const,Array,Abs)
		0x95, 0x06,       //   Report Count (6)
		0x75, 0x08,       //   Report Size (8)
		0x15, 0x00,       //   Logical Minimum (0)
		0x25, 0x65,       //   Logical Maximum (101)
		0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
		0x19, 0x00,       //   Usage Minimum (0)
		0x29, 0x65,       //   Usage Maximum (101)
		0x81, 0x00,       //   Input (Data,Ary,Abs)
		0xC0,             // End Collection
	};

	kb_layout_parse(&layout, boot_desc, sizeof(boot_desc), 8);
	if (layout.report_id != 0 || layout.report_len != 8 ||
	    layout.modifier_off != 0 || layout.keys_off != 2 ||
	    layout.keys_count != 6 || !layout.valid) {
		printf("FAIL: boot descriptor layout\n");
		failures++;
	}

	// Same descriptor with a leading Report ID byte (9-byte reports).
	const uint8_t rid_desc[] = {
		0x05, 0x01,
		0x09, 0x06,
		0xA1, 0x01,
		0x85, 0x01,       //   Report ID (1)
		0x05, 0x07,
		0x19, 0xE0,
		0x29, 0xE7,
		0x15, 0x00,
		0x25, 0x01,
		0x75, 0x01,
		0x95, 0x08,
		0x81, 0x02,
		0x95, 0x01,
		0x75, 0x08,
		0x81, 0x01,
		0x95, 0x06,
		0x75, 0x08,
		0x15, 0x00,
		0x25, 0x65,
		0x05, 0x07,
		0x19, 0x00,
		0x29, 0x65,
		0x81, 0x00,
		0xC0,
	};

	kb_layout_parse(&layout, rid_desc, sizeof(rid_desc), 9);
	if (layout.report_id != 1 || layout.report_len != 9 ||
	    layout.modifier_off != 1 || layout.keys_off != 3 ||
	    layout.keys_count != 6 || !layout.valid) {
		printf("FAIL: report-id descriptor layout\n");
		failures++;
	}

	// Missing descriptor should fall back to legacy 8-byte layout.
	kb_layout_parse(&layout, NULL, 0, 0);
	if (layout.report_id != 0 || layout.report_len != 8 ||
	    layout.modifier_off != 0 || layout.keys_off != 2 ||
	    !layout.valid) {
		printf("FAIL: empty descriptor fallback\n");
		failures++;
	}

	if (failures == 0) {
		printf("ALL PASSED\n");
		return 0;
	}
	return 1;
}
