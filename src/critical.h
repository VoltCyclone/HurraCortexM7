#pragma once
#include <stdint.h>

/* Reentrant critical section by save/restore of PRIMASK.
 *
 * Why PRIMASK (mask ALL IRQs) and not BASEPRI (mask only PIT and below): the
 * sections we protect are a handful of instructions (a mask RMW), and this
 * codebase ships no BASEPRI helper. Globally masking for ~tens of nanoseconds
 * at 912 MHz is simpler and the latency it adds to the PIT tick is far below
 * the timing jitter humanize_timing_next already injects deliberately.
 *
 * Reentrant: an inner enter/exit pair will not prematurely re-enable IRQs that
 * an outer pair had disabled, because exit restores the *saved* PRIMASK.
 *
 * On the host test build (no ARM), these compile to no-ops so logic that calls
 * them stays testable. */

#if defined(HOSTTEST) || defined(HUMANIZE_HOSTTEST)
static inline uint32_t crit_enter(void) { return 0; }
static inline void     crit_exit(uint32_t s) { (void)s; }
#else
static inline uint32_t crit_enter(void) {
    uint32_t primask;
    __asm__ volatile("MRS %0, primask" : "=r"(primask));
    __asm__ volatile("CPSID i" ::: "memory");
    return primask;
}
static inline void crit_exit(uint32_t saved) {
    /* Only re-enable if the saved state had IRQs enabled (primask bit 0 == 0). */
    if ((saved & 1u) == 0u)
        __asm__ volatile("CPSIE i" ::: "memory");
}
#endif
