#pragma once
// GPT2 free-running microsecond counter for profiling AND poll-interval
// measurement. Clocked from ipg_clk (CLKSRC=001). On this board the CCM
// sets IPG = ARM/4 = F_CPU/4 (core/startup.c IPG_PODF=/4), so the /N prescaler
// for a true 1 MHz (1 µs) tick is N = (F_CPU/4)/1e6. Deriving it from F_CPU
// keeps the tick at 1 µs even if F_CPU changes (e.g. 816→600 MHz), instead of
// the old hard-coded /204 that was only correct at 816 MHz.
// 32-bit counter wraps every ~71.6 minutes; unsigned subtraction is single-wrap safe.
// Zero CPU overhead — reads are a single register load.

#include <stdint.h>
#include "imxrt.h"

// IPG clock in Hz on this board = ARM (F_CPU) / 4. If the CCM IPG_PODF in
// core/startup.c ever changes, update this divisor to match.
#define GPT_IPG_HZ        (F_CPU / 4u)
// Prescaler register value = divisor-1; gives a 1 MHz counter (1 tick = 1 µs).
#define GPT_PRESCALER_1MHZ ((GPT_IPG_HZ / 1000000u) - 1u)

static inline void gpt_profile_init(void)
{
	// Enable GPT2 clocks
	CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) |
	              CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

	// Reset and configure GPT2
	GPT2_CR = 0;                       // disable
	GPT2_PR = GPT_PRESCALER_1MHZ;      // ipg_clk / (F_CPU/4/1e6) -> 1 MHz tick
	GPT2_CR = (1 << 9) |               // FRR: free-run mode
	          (1 << 6) |               // CLKSRC = 001 (ipg_clk)
	          (1 << 0);                // EN: enable timer
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

