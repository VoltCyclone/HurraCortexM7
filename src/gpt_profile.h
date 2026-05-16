#pragma once
// GPT2 free-running microsecond counter for profiling.
// Uses IPG clock (204 MHz) with /204 prescaler -> 1 MHz = 1µs resolution.
// 32-bit counter wraps every ~71.6 minutes.
// Zero CPU overhead — reads are a single register load.

#include <stdint.h>
#include "imxrt.h"

static inline void gpt_profile_init(void)
{
	// Enable GPT2 clocks
	CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) |
	              CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

	// Reset and configure GPT2
	GPT2_CR = 0;                // disable
	GPT2_PR = 203;              // prescaler: IPG/204 = ~1MHz
	GPT2_CR = (1 << 9) |        // FRR: free-run mode
	          (1 << 6) |         // CLKSRC = 001 (IPG clock)
	          (1 << 0);          // EN: enable timer
}

// Read current microsecond timestamp (wraps at ~71.6 minutes)
static inline uint32_t gpt_profile_us(void)
{
	return GPT2_CNT;
}

// Measure elapsed microseconds between two timestamps (handles single wrap)
static inline uint32_t gpt_profile_elapsed(uint32_t start, uint32_t end)
{
	return end - start; // unsigned subtraction handles wrap
}

