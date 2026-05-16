// Humanize stub — DWT cycle counter init only.
// Tremor generation moved to EWMA noise model in smooth.c.

#include "humanize.h"

void humanize_init(void)
{
	// Enable DWT cycle counter (used for LFSR seeding)
	volatile uint32_t *dwt_ctrl = (volatile uint32_t *)0xE0001000;
	*dwt_ctrl |= 1;
}
