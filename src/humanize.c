#include "humanize.h"
void humanize_init(uint32_t interval_us) { (void)interval_us; }
void humanize_filter(int16_t *dx, int16_t *dy) { (void)dx; (void)dy; }
uint32_t humanize_timing_next(uint32_t b, bool *s) { *s = false; return b; }
void humanize_set_level(uint8_t level) { (void)level; }
