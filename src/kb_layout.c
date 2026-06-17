// src/kb_layout.c — see kb_layout.h

#include "kb_layout.h"
#include <string.h>

void kb_layout_parse(kb_layout_t *out, const uint8_t *report_desc,
                     uint16_t rdlen, uint16_t maxpkt)
{
	memset(out, 0, sizeof(*out));
	out->keys_count = 6;

	uint8_t report_id = 0;
	uint16_t i = 0;

	while (i < rdlen) {
		uint8_t b = report_desc[i];

		// Long item: skip
		if (b == 0xFE) {
			if (i + 2 < rdlen)
				i += 3u + report_desc[i + 1];
			else
				break;
			continue;
		}

		uint8_t sz = b & 0x03u;
		if (sz == 3) sz = 4;
		if (i + 1 + sz > rdlen) break;

		uint32_t val = 0;
		if (sz >= 1) val = report_desc[i + 1];
		if (sz >= 2) val |= (uint32_t)report_desc[i + 2] << 8;
		if (sz >= 4) val |= (uint32_t)report_desc[i + 3] << 16
			            | (uint32_t)report_desc[i + 4] << 24;

		// Report ID (0x84) — capture the first one in the descriptor.
		if ((b & 0xFCu) == 0x84u) {
			if (report_id == 0)
				report_id = (uint8_t)val;
		}

		i += 1u + sz;
	}

	out->report_id = report_id;
	if (report_id != 0) {
		// Standard boot layout with a leading Report ID byte.
		out->modifier_off = 1;
		out->keys_off = 3;
		if (maxpkt > 0 && maxpkt <= 64)
			out->report_len = (uint8_t)maxpkt;
		else
			out->report_len = 9; // common full-speed Report-ID keyboard size
	} else {
		// Legacy 8-byte boot report.
		out->modifier_off = 0;
		out->keys_off = 2;
		out->report_len = 8;
	}

	out->valid = (out->report_len >= out->keys_off + out->keys_count);
}
