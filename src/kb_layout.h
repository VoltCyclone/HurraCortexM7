// src/kb_layout.h — minimal HID keyboard report-descriptor layout extractor
//
// Parses a keyboard interface's HID report descriptor to discover whether the
// input report carries a leading Report ID byte.  The data layout is assumed to
// be the standard boot-protocol byte order (modifier, reserved, 6 keycodes);
// only the presence/absence of a Report ID and the total report length are
// extracted.  Non-boot or exotic keyboard layouts fall back to the legacy 8-byte
// path.

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint8_t report_id;      // 0 if descriptor contains no Report ID
	uint8_t report_len;     // total input report bytes (including report_id)
	uint8_t modifier_off;   // byte offset of the modifier bitmap
	uint8_t keys_off;       // byte offset of the first keycode
	uint8_t keys_count;     // number of simultaneous keycodes (always 6 here)
	bool    valid;          // true when the parsed layout fits in report_len
} kb_layout_t;

// Parse `report_desc` (rdlen bytes) and fill `out`.  `maxpkt` is the interrupt
// IN endpoint's wMaxPacketSize; it is used as the report length when a Report
// ID is present.  If rdlen==0 or maxpkt==0 the parser falls back to the
// no-report-id 8-byte boot layout.
void kb_layout_parse(kb_layout_t *out, const uint8_t *report_desc,
                     uint16_t rdlen, uint16_t maxpkt);
