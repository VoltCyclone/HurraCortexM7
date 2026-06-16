#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Pure synth-emission cadence decisions. No hardware deps — host-testable.
 *
 * All timestamps are free-running microseconds (GPT2 on target, plain uint32
 * in tests) and may wrap; every comparison uses wrap-safe unsigned subtraction.
 *
 * "silence" = the real mouse has not delivered a report for long enough that we
 * must fabricate a carrier.  "due" = enough time has passed since our last synth
 * frame to emit another, at the measured device rate. */

/* When no confident measurement exists yet, fall back to 1 kHz (1000 us). */
#define SYNTH_FALLBACK_US 1000u
/* Declare the mouse idle after this many measured intervals with no report. */
#define SYNTH_SILENCE_PERIODS 2u
/* Never let a burst-fooled measurement drive synth faster than this floor.
 * Mirrors humanize.c's HZ_LDVAL_US_MIN (125 us = 8 kHz) so synth can never
 * exceed the fastest cadence the rest of the system is clamped to. */
#define SYNTH_PERIOD_FLOOR_US 125u

/* Effective cadence period: the measured interval, clamped to the floor, or the
 * 1 kHz fallback when measured == 0 (EWMA not yet confident). */
static inline uint32_t synth_period_us(uint32_t measured_us) {
	uint32_t p = measured_us ? measured_us : SYNTH_FALLBACK_US;
	if (p < SYNTH_PERIOD_FLOOR_US) p = SYNTH_PERIOD_FLOOR_US;
	return p;
}

/* True when the real mouse is silent: no merge for SILENCE_PERIODS * period. */
static inline bool synth_mouse_silent(uint32_t now_us, uint32_t last_merge_us,
	                                  uint32_t measured_us) {
	uint32_t period = synth_period_us(measured_us);
	return (uint32_t)(now_us - last_merge_us) >= period * SYNTH_SILENCE_PERIODS;
}

/* True when a synth frame is due: >= one period since the last synth emission. */
static inline bool synth_due(uint32_t now_us, uint32_t last_synth_us,
	                         uint32_t measured_us) {
	uint32_t period = synth_period_us(measured_us);
	return (uint32_t)(now_us - last_synth_us) >= period;
}
