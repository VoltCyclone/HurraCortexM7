#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Always-on humanization filter. Operates on the INJECTED mouse delta only;
 * real-mouse passthrough is never routed through it. */
void     humanize_init(uint32_t interval_us);   /* seed + level default */
void     humanize_filter(int16_t *dx, int16_t *dy); /* in-place, per frame */
uint32_t humanize_timing_next(uint32_t base_ldval, bool *out_skip);
void     humanize_set_level(uint8_t level);      /* 0=off..3=strong */
bool     humanize_pending(void);   /* true while owed motion remains to emit */
