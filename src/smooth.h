#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SMOOTH_FP_SHIFT  16
#define SMOOTH_FP_ONE    (1 << SMOOTH_FP_SHIFT)
#define SMOOTH_FP_HALF   (1 << (SMOOTH_FP_SHIFT - 1))

#define SMOOTH_QUEUE_SIZE 32

void smooth_init(uint32_t interval_us);
void smooth_inject(int16_t x, int16_t y);
void smooth_process_frame(int16_t *out_x, int16_t *out_y);
bool smooth_has_pending(void);
void smooth_clear(void);
void smooth_set_max_per_frame(int16_t max);
void smooth_set_humanize(bool enabled);
uint32_t smooth_timing_next(uint32_t base_ldval, bool *out_skip);
