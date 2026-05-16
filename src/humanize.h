#pragma once
#include <stdint.h>

// Humanize stub — enables DWT cycle counter for LFSR seeding.
// Tremor generation is now handled by EWMA noise in smooth.c.
void humanize_init(void);
