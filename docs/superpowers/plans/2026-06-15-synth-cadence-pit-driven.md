# Synth Injection Cadence — Measured-Rate (A) + PIT-Driven Emission (C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hardcoded 1 kHz synth-emission cadence in the mouse injection path with the firmware's already-measured real poll interval, then (optionally) move synth emission from the main loop into the PIT ISR so it fires jitter-free at exactly the device's poll rate.

**Architecture:** The firmware proxies a USB mouse and injects humanized motion. Real reports carry injection via the *merge path* (`kmbox_merge_report`); when the mouse is silent the *synth path* (`kmbox_send_pending`) fabricates a carrier. Both must emit at most one report per device poll window, never both in the same window. Today the synth path re-derives cadence from `millis()` with a hardcoded `SYNTH_SILENCE_MS=2` and `ms != last_synth_ms`, which assumes a 1 kHz device. Part A extracts the cadence decision into a pure, host-testable module driven by GPT2 microseconds and the measured EWMA interval. Part C reuses the existing "precompute-in-main, fire-in-ISR" pattern (already used for `pit_next_ldval`) to emit a pre-built report from `pit0_isr`, guarding the one shared RMW (`ep_busy_mask`/`active_bank_mask`) with a reentrant critical section.

**Tech Stack:** C11, bare-metal Cortex-M7 (i.MX RT1062 @ 912 MHz), GNU arm-none-eabi-gcc. Host-native unit tests compiled with system `cc` via `make test` (no hardware headers). PIT0 timer (24 MHz PERCLK), GPT2 free-running 1 MHz counter.

---

## Background: exact current state (verified against source)

- **Cadence tracking** lives in `src/kmbox.c:760-767`:
  - `static uint32_t last_merge_ms;` — set to `millis()` at `kmbox.c:787` when a real report rides through.
  - `static uint32_t last_synth_ms;` — set at `kmbox.c:1096`.
  - `#define SYNTH_SILENCE_MS 2`
- **Synth gate** is `src/kmbox.c:1092-1096`:
  ```c
  uint32_t ms = millis();
  bool mouse_silent = (uint32_t)(ms - last_merge_ms) >= SYNTH_SILENCE_MS;
  if (inject.mouse_dirty && mouse_silent && ms != last_synth_ms &&
      cached_mouse_ep && mouse_layout.valid) {
      last_synth_ms = ms;
      ...
  ```
- **`merged_this_cycle`** (`kmbox.c:177`, reset at `616`, set at `895`/`905`) already prevents the merge and synth paths from both running in one main-loop iteration: `kmbox_send_pending` early-returns at `kmbox.c:1087` if a merge happened this cycle.
- **Measured interval** is already available: `humanize_measured_interval_us()` (`humanize.c:159`) returns the EWMA-smoothed real poll interval in µs, fed by `humanize_record_arrival(gpt_profile_us())` at `main.c:309`. Note: `humanize_init(interval_us)` seeds `S.meas_interval_us` to the nominal bInterval at `humanize.c:128`, so in normal operation it returns non-zero immediately — the `synth_period_us(0)` fallback is defensive (covers a hypothetical pre-init call), not a path hit at runtime. Harmless to keep; do not rely on it firing.
- **GPT2 µs counter**: `gpt_profile_us()` (`src/gpt_profile.h:42`), 1 MHz, wraps ~71.6 min, single-load atomic. `gpt_profile_init()` called at `main.c:122`.
- **PIT0**: enumerated to the mouse poll rate at `main.c:198-225`, ISR at `main.c:62-71` (W1C flag, load `pit_next_ldval`, set `pit_tick_pending`, DSB). NVIC priority 64 (`main.c:118`) — the **highest-priority interrupt** in the system (DMA=96/160, USB host=144).
- **USB device is polled, not interrupt-driven**: no `IRQ_USB1` vector is attached. Only `usb_device_poll` (`usb_device.c:610`, main-loop) and `usb_device_send_report` (`usb_device.c:651`, main-loop) touch device-TX state today.
- **The one shared RMW hazard** for Part C: `ep_busy_mask` is read-cleared in `usb_device_poll` (`usb_device.c:626-627`) and read-set in `prime_int_ep` (`usb_device.c:74`); `active_bank_mask` likewise (`usb_device.c:71-73`, read at `633`/`662`/`667`). Non-atomic, currently safe only because single-threaded.
- **Available IRQ primitives** (`include/imxrt.h:10073-10074`): only `__disable_irq()` / `__enable_irq()`. There is **no** `__set_BASEPRI` helper and **no** reentrant critical-section helper. Part C must add one.
- **`NVIC_SET_PRIORITY`** (`imxrt.h:10069`) writes the raw 8-bit priority value left-justified directly to the register, so a BASEPRI mask value uses the **same raw scale** (e.g. `64`), NOT a `<< (8 - PRIO_BITS)` shift.
- **Test harness** (`Makefile:92-99`): `make test` compiles `test/humanize_test.c` + `src/humanize.c` and `test/motion_test.c` + `src/actions.c` with host `cc`, `-DHUMANIZE_HOSTTEST`. Tests are deterministic, time-stepped, use a local `CHECK(cond, msg)` macro and return non-zero `failures`. New host-testable C must not pull hardware headers in the host build.

---

## File Structure

**New files:**
- `src/synth_cadence.h` — pure cadence-decision interface. Plain C, no hardware headers. One responsibility: given "now" (µs), last-merge timestamp (µs), measured interval (µs), and last-synth timestamp (µs), decide (a) is the mouse silent, (b) is a synth frame due now. Shared by Part A (main-loop) and Part C (ISR).
- `src/synth_cadence.c` — implementation (pure arithmetic, wrap-safe unsigned subtraction).
- `test/synth_cadence_test.c` — host-native tests, same style as `motion_test.c`.
- `src/critical.h` (Part C only) — reentrant critical section over the PIT IRQ using `__disable_irq`/`__enable_irq` save/restore via PRIMASK. Header-only `static inline`.

**Modified files:**
- `Makefile` — add `synth_cadence.c` to `SRC`/`HOT_SRC`; add `synth_cadence_test` to the `test` target.
- `src/kmbox.c` — Part A: replace the `millis()` synth gate with `synth_cadence` calls on `gpt_profile_us()`; retire `last_merge_ms`/`last_synth_ms`/`SYNTH_SILENCE_MS`. Part C: add a publish-the-next-synth-frame builder and the lock-free double buffer; guard the wheel/dirty bookkeeping.
- `src/usb_device.c` — Part C: wrap the `ep_busy_mask`/`active_bank_mask`/`pending_len` RMWs in `usb_device_poll` and `usb_device_send_report` with the reentrant critical section.
- `src/main.c` — Part A: nothing required (synth still emitted from `kmbox_send_pending`). Part C: arm the published synth frame on the `pit_tick_pending` path; add ISR emission in `pit0_isr`.

**Why a separate `synth_cadence` module:** the cadence math is the part with real logic and real bugs (wrap, fallback-before-measurement, silence-multiple). Isolating it as pure functions makes it host-testable with `make test` and lets Part A and Part C share *identical* decision logic — Part C must not silently diverge from Part A's silence rule.

---

# PART A — Measured-rate synth cadence (main-loop emission)

Self-contained and shippable on its own. No concurrency changes. Fixes the 1 kHz hardcode.

### Task A1: Create the pure cadence module with a failing test

**Files:**
- Create: `src/synth_cadence.h`
- Create: `src/synth_cadence.c`
- Create: `test/synth_cadence_test.c`
- Modify: `Makefile:92-99` (test target)

- [ ] **Step 1: Write the header**

Create `src/synth_cadence.h`:

```c
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
```

> Note: these are `static inline` in the header so both the ISR (Part C, must avoid a call into a non-ITCM `.o`) and the host test get them with zero link friction. `synth_cadence.c` exists only to give the module a translation unit for any future non-inline helpers and to keep the Makefile pattern uniform; it may be a near-empty file.

- [ ] **Step 2: Write the (near-empty) implementation file**

Create `src/synth_cadence.c`:

```c
#include "synth_cadence.h"
/* All current logic is static inline in the header. This TU exists so the
 * module has a stable object-file slot in SRC/HOT_SRC for future growth. */
```

- [ ] **Step 3: Write the failing test**

Create `test/synth_cadence_test.c`:

```c
#include <stdio.h>
#include <stdint.h>
#include "synth_cadence.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    /* period selection */
    CHECK(synth_period_us(0)    == 1000u, "no measurement -> 1 kHz fallback");
    CHECK(synth_period_us(1000) == 1000u, "1 kHz measured -> 1000us");
    CHECK(synth_period_us(125)  == 125u,  "8 kHz measured -> 125us");
    CHECK(synth_period_us(50)   == 125u,  "below floor -> clamped to 125us");

    /* silence: 1 kHz device, silence after 2 ms */
    CHECK(!synth_mouse_silent(1999, 0, 1000), "1kHz: 1999us since merge -> active");
    CHECK( synth_mouse_silent(2000, 0, 1000), "1kHz: 2000us since merge -> silent");

    /* silence: 8 kHz device, silence after 250us (2 * 125us) */
    CHECK(!synth_mouse_silent(249, 0, 125), "8kHz: 249us since merge -> active");
    CHECK( synth_mouse_silent(250, 0, 125), "8kHz: 250us since merge -> silent");

    /* due: fires once per period, not per loop iteration */
    CHECK(!synth_due(999,  0, 1000), "1kHz: 999us since synth -> not due");
    CHECK( synth_due(1000, 0, 1000), "1kHz: 1000us since synth -> due");
    CHECK( synth_due(125,  0, 125),  "8kHz: 125us since synth -> due");

    /* wrap safety: now just past the uint32 wrap, last just before it */
    CHECK( synth_due(50u, 0xFFFFFFFFu - 950u, 1000),
           "wrap: 1000us elapsed across the uint32 boundary -> due");
    CHECK(!synth_due(50u, 0xFFFFFFFFu - 800u, 1000),
           "wrap: ~850us elapsed across the boundary -> not due");

    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("synth_cadence: all passed\n");
    return 0;
}
```

- [ ] **Step 4: Add the test to the Makefile**

Modify `Makefile` `test:` target (currently lines 92-99) to append, after the `motion_test` block:

```make
	cc -std=c11 -O2 -Isrc -o /tmp/synth_cadence_test \
	   test/synth_cadence_test.c src/synth_cadence.c
	/tmp/synth_cadence_test
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `make test`
Expected: existing humanize/motion tests pass, then `synth_cadence: all passed`. (These are pure arithmetic with hand-computed expectations, so they pass immediately — the "failing" stage here is a compile-error gate: if the header math is wrong the hand-computed asserts fail. Confirm by temporarily breaking `SYNTH_SILENCE_PERIODS` to 1 and seeing the 2000us silence assert fail, then revert.)

- [ ] **Step 6: Commit**

```bash
git add src/synth_cadence.h src/synth_cadence.c test/synth_cadence_test.c Makefile
git commit -m "feat: pure synth-cadence module (measured-rate, wrap-safe) + host tests"
```

---

### Task A2: Switch the kmbox synth gate to GPT2 µs + measured cadence

**Files:**
- Modify: `src/kmbox.c:760-767` (tracking vars), `:787` (merge timestamp), `:1092-1096` (gate), `:1096` (synth-stamp)

- [ ] **Step 1: Add the include and replace the tracking variables**

In `src/kmbox.c`, add near the other includes (top of file, with the existing `extern uint32_t millis(void);` block around line 13):

```c
#include "gpt_profile.h"
#include "synth_cadence.h"
```

Replace the block at `kmbox.c:760-767`:

```c
// Output cadence tracking. last_merge_ms = when a real mouse report last rode
// through (injection rides those). last_synth_ms = last standalone synth frame.
// Used to keep exactly one mouse report per ~1 ms: injection rides merge
// reports while the mouse is active, and the synth path only fills in when the
// mouse has gone silent (so the two paths never both emit in the same frame).
static uint32_t last_merge_ms;
static uint32_t last_synth_ms;
#define SYNTH_SILENCE_MS 2   // mouse considered idle after this many ms of no report
```

with:

```c
// Output cadence tracking (GPT2 microseconds). last_merge_us = when a real
// mouse report last rode through (injection rides those). last_synth_us = last
// standalone synth frame. The cadence rule (see synth_cadence.h) keeps exactly
// one mouse report per *measured device poll interval*: injection rides merge
// reports while the mouse is active, and the synth path only fills in when the
// mouse has gone silent, at the same rate the merge path was running — not a
// fixed 1 kHz. The two paths never both emit in the same window.
static uint32_t last_merge_us;
static uint32_t last_synth_us;
```

- [ ] **Step 2: Update the merge timestamp**

At `kmbox.c:787`, replace:

```c
		last_merge_ms = millis();   // a real mouse report is riding through now
```

with:

```c
		last_merge_us = gpt_profile_us();   // a real mouse report is riding through now
```

> Part-C note: this in-merge stamp is correct for Part A (main-loop-only). Part C Task C4 Step 2c relocates the stamp to the poll site (before `kmbox_merge_report`) to close an ISR race window, and deletes this line. If you are executing Part A only, keep it here.

- [ ] **Step 3: Replace the synth gate**

At `kmbox.c:1092-1096`, replace:

```c
	uint32_t ms = millis();
	bool mouse_silent = (uint32_t)(ms - last_merge_ms) >= SYNTH_SILENCE_MS;
	if (inject.mouse_dirty && mouse_silent && ms != last_synth_ms &&
	    cached_mouse_ep && mouse_layout.valid) {
		last_synth_ms = ms;
```

with:

```c
	uint32_t now_us = gpt_profile_us();
	uint32_t measured_us = humanize_measured_interval_us();
	bool mouse_silent = synth_mouse_silent(now_us, last_merge_us, measured_us);
	bool due = synth_due(now_us, last_synth_us, measured_us);
	if (inject.mouse_dirty && mouse_silent && due &&
	    cached_mouse_ep && mouse_layout.valid) {
		last_synth_us = now_us;
```

- [ ] **Step 4: Verify `humanize.h` is already included in kmbox.c**

Run: `grep -n 'humanize.h\|humanize_measured_interval_us' src/kmbox.c`
Expected: `#include "humanize.h"` already present (it is — `humanize_filter`/`humanize_return`/`humanize_pending` are used in the merge path). If absent, add it next to the new includes.

- [ ] **Step 5: Build the firmware (both protocol variants)**

Run: `make clean && make && make PROTOCOL=ferrum`
Expected: zero warnings, `firmware.hex` produced for both. Confirm no remaining references to the retired names:

Run: `grep -n 'last_merge_ms\|last_synth_ms\|SYNTH_SILENCE_MS' src/kmbox.c`
Expected: no matches.

- [ ] **Step 6: Run host tests (regression guard)**

Run: `make test`
Expected: all pass (humanize, motion, synth_cadence).

- [ ] **Step 7: Commit**

```bash
git add src/kmbox.c
git commit -m "fix: drive synth cadence from measured poll interval, not hardcoded 1 kHz"
```

---

### Task A3: Document the cadence invariant at the synth call site

**Files:**
- Modify: `src/kmbox.c:1087-1091` (the comment above the synth block)

- [ ] **Step 1: Update the explanatory comment**

At `kmbox.c:1088-1091`, replace the existing comment:

```c
	// Only synthesize a standalone mouse report when the physical mouse has
	// gone silent — otherwise injection rides the next real report (merge),
	// so the two paths never both emit in the same frame (which would flood /
	// overwrite at the 1 kHz endpoint). Capped to one synth per ms.
```

with:

```c
	// Only synthesize a standalone mouse report when the physical mouse has
	// gone silent — otherwise injection rides the next real report (merge),
	// so the two paths never both emit in the same poll window (a double-emit
	// would overwrite at the endpoint, since the host reads <=1 report per
	// bInterval). Silence and cadence both derive from the *measured* device
	// poll interval (synth_cadence.h), so on an 8 kHz mouse the synth path
	// fills in at 8 kHz too, matching the rate the merge path was running.
```

- [ ] **Step 2: Build and commit**

Run: `make` (expect zero warnings)

```bash
git add src/kmbox.c
git commit -m "docs: explain measured-rate synth cadence invariant"
```

---

**PART A COMPLETE.** Shippable. The synth path now tracks the real device rate with no concurrency changes. Residual: synth emit instant still has main-loop-iteration granularity (negligible at 1 kHz, ~one loop pass at 8 kHz). Part C removes that.

---

# PART C — PIT-driven ISR emission (jitter-free)

Builds on Part A. Moves synth emission into `pit0_isr` using the existing precompute-in-main / fire-in-ISR pattern. **Higher risk** — touches the USB device controller prime path. Do not squash into Part A's commits; Part C's safety depends on hardware validation (Tasks C5–C6) that Part A does not.

### Task C1: Add a reentrant critical section with a failing test

The only available primitives are `__disable_irq`/`__enable_irq` (global PRIMASK). A naive `__enable_irq()` at the end of a section would wrongly re-enable interrupts even if they were already disabled by an outer section. We need save/restore of PRIMASK.

**Files:**
- Create: `src/critical.h`
- Create: `test/critical_test.c`
- Modify: `Makefile` test target

- [ ] **Step 1: Write the header**

Create `src/critical.h`:

```c
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
```

- [ ] **Step 2: Write the failing test**

Create `test/critical_test.c`:

```c
#include <stdio.h>
#include <stdint.h>
#define HOSTTEST
#include "critical.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

/* On host these are no-ops; the test asserts the *shape* of the API (nesting
 * compiles and returns a token) so the target build can't drift the signature.
 * Hardware reentrancy is validated on-target in Task C6, not here. */
int main(void) {
    uint32_t outer = crit_enter();
    uint32_t inner = crit_enter();
    crit_exit(inner);
    crit_exit(outer);
    CHECK(1, "nested enter/exit compiles and links");
    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("critical: all passed\n");
    return 0;
}
```

- [ ] **Step 3: Add to Makefile test target**

Append to the `test:` target:

```make
	cc -std=c11 -O2 -Isrc -o /tmp/critical_test test/critical_test.c
	/tmp/critical_test
```

- [ ] **Step 4: Run**

Run: `make test`
Expected: `critical: all passed`.

- [ ] **Step 5: Commit**

```bash
git add src/critical.h test/critical_test.c Makefile
git commit -m "feat: reentrant PRIMASK critical section (host no-op shim) + test"
```

---

### Task C2: Guard the USB device-TX RMWs

**Files:**
- Modify: `src/usb_device.c` — include `critical.h`; wrap the mask RMWs in `usb_device_poll` (`:623-638`) and `usb_device_send_report` (`:660-670`). `prime_int_ep` (`:57-75`) is called from inside both guarded regions, so it stays unguarded itself.

- [ ] **Step 1: Add the include**

Near the top of `src/usb_device.c` (with the other includes):

```c
#include "critical.h"
```

- [ ] **Step 2: Guard the completion handler in `usb_device_poll`**

At `usb_device.c:623-638`, wrap the ENDPTCOMPLETE block. Replace:

```c
		uint32_t complete = USB1_ENDPTCOMPLETE;
		if (complete) {
			USB1_ENDPTCOMPLETE = complete;
			uint8_t done = (uint8_t)(complete >> 16) & ep_busy_mask;
			ep_busy_mask &= ~done; // clear all done EPs in one shot
			while (done) {
				uint8_t ep = (uint8_t)__builtin_ctz(done);
				done &= done - 1; // clear lowest set bit
				if (pending_len[ep] > 0) {
					uint8_t slot = ep_to_slot[ep];
					uint8_t bank = ((active_bank_mask >> ep) ^ 1) & 1;
					prime_int_ep(ep, slot, bank, pending_len[ep]);
					pending_len[ep] = 0;
				}
			}
		}
```

with:

```c
		uint32_t complete = USB1_ENDPTCOMPLETE;
		if (complete) {
			USB1_ENDPTCOMPLETE = complete;
			// ep_busy_mask / active_bank_mask / pending_len are also touched by
			// the PIT ISR's synth emit (Part C). Guard the RMW so a tick landing
			// mid-update can't clobber a bit. prime_int_ep runs inside the guard.
			uint32_t cs = crit_enter();
			uint8_t done = (uint8_t)(complete >> 16) & ep_busy_mask;
			ep_busy_mask &= ~done; // clear all done EPs in one shot
			while (done) {
				uint8_t ep = (uint8_t)__builtin_ctz(done);
				done &= done - 1; // clear lowest set bit
				if (pending_len[ep] > 0) {
					uint8_t slot = ep_to_slot[ep];
					uint8_t bank = ((active_bank_mask >> ep) ^ 1) & 1;
					prime_int_ep(ep, slot, bank, pending_len[ep]);
					pending_len[ep] = 0;
				}
			}
			crit_exit(cs);
		}
```

- [ ] **Step 3: Guard `usb_device_send_report`**

Hold ONE critical section across the whole body. Do **not** release around the `memcpy` — an earlier draft of this plan split the lock to shorten the PIT-blocking window, but that split is a statically-provable data-corruption race, not just a timing risk:

> **Why the split is unsafe (do not do it):** if main reads `bank` under the lock, releases, and the PIT ISR then primes the *same* EP on that same bank before main re-takes the lock, main's subsequent `memcpy` writes into the bank buffer the controller is actively DMA-reading (torn data delivered to host) and then double-primes a live EP. A logic analyzer cannot see the silent buffer corruption. The single-lock version below cannot be interrupted by the ISR anywhere inside the function, so it has no such window.

The 64-byte `memcpy` + `prime_int_ep` under lock is ~tens of cycles at 912 MHz — far below the deliberate `humanize_timing_next` jitter band, so the longer hold is harmless.

At `usb_device.c:660-670`, replace:

```c
	uint8_t ep_bit = (1 << ep_num);
	if (ep_busy_mask & ep_bit) {
		uint8_t bank = ((active_bank_mask >> ep_num) ^ 1) & 1;
		memcpy(int_tx_buf[slot][bank], data, len);
		pending_len[ep_num] = (uint8_t)len;
		return true; // staged, not dropped
	}
	uint8_t bank = (active_bank_mask >> ep_num) & 1;
	memcpy(int_tx_buf[slot][bank], data, len);
	prime_int_ep(ep_num, slot, bank, len);
	return true;
```

with:

```c
	uint8_t ep_bit = (1 << ep_num);
	// One critical section across the whole RMW+copy+prime. The PIT ISR's synth
	// emit also calls this function; releasing mid-body would let the ISR prime
	// the same EP between our bank-read and our memcpy, corrupting a live DMA
	// bank. Keep it atomic — the hold is only a few dozen cycles.
	uint32_t cs = crit_enter();
	bool ok;
	if (ep_busy_mask & ep_bit) {
		uint8_t bank = ((active_bank_mask >> ep_num) ^ 1) & 1;
		memcpy(int_tx_buf[slot][bank], data, len);
		pending_len[ep_num] = (uint8_t)len; // staged, not dropped
		ok = true;
	} else {
		uint8_t bank = (active_bank_mask >> ep_num) & 1;
		memcpy(int_tx_buf[slot][bank], data, len);
		prime_int_ep(ep_num, slot, bank, len);
		ok = true;
	}
	crit_exit(cs);
	return ok;
```

> Note: `usb_device_send_report` is now called from ISR context (Part C). Because the whole body runs under `crit_enter`, and the PIT ISR is the only interrupt that calls it, there is no re-entrancy: when the ISR runs this function, it was either not already inside it (main was elsewhere) or main was inside it with IRQs masked (so the ISR could not have fired). `crit_enter`'s PRIMASK save/restore makes the ISR's own call self-consistent (IRQs already masked → its `crit_exit` leaves them masked until the outer exit).

- [ ] **Step 4: Build both variants**

Run: `make clean && make && make PROTOCOL=ferrum`
Expected: zero warnings.

- [ ] **Step 5: Commit**

```bash
git add src/usb_device.c
git commit -m "feat: guard USB device-TX mask RMWs for ISR-context emission"
```

---

### Task C3: Build + publish the next synth frame from the main loop (precompute)

Move the report-construction (including the FPU `humanize_filter`) into a function the main loop calls on the `pit_tick_pending` path, writing into a lock-free double buffer. Keeps the FPU out of the ISR.

**Files:**
- Modify: `src/kmbox.c` — add the double buffer + a `kmbox_publish_synth(void)` that builds the next frame; expose it and the ISR-side `kmbox_emit_synth_isr(uint32_t now_us)` in `src/kmbox.h`.
- Modify: `src/kmbox.h` — declare the two new functions.

- [ ] **Step 1: Add the double-buffer state and publish builder in kmbox.c**

After the `inject` definition (around `kmbox.c:131`), add:

```c
// --- Part C: lock-free synth frame handoff (main builds, PIT ISR emits) ----
// Main loop builds the next synth report into the inactive buffer, then flips
// synth_pub_idx with a single aligned store (atomic publish). The ISR reads the
// published buffer and emits it — no FPU and no humanize filter in interrupt
// context (the only inject write is a single-store wheel clear on confirmed emit).
// All cross-context scalars are volatile: written in one context, read in the
// other, and the compiler must not cache or reorder them.
static uint8_t           synth_buf[2][16];
static volatile uint8_t  synth_buf_len[2];
static volatile uint8_t  synth_pub_idx;   // buffer the ISR should read
static volatile bool     synth_armed;     // a fresh frame is waiting to emit
// last_synth_us is now WRITTEN by the ISR (kmbox_emit_synth_isr) and READ by the
// main builder, so it must be volatile (uint32 access is atomic on M7, but the
// compiler would otherwise cache the main-loop read across iterations).
static volatile uint32_t last_synth_us;
```

- [ ] **Step 1a: Delete the `last_synth_us` definition that Task A2 added, to avoid a duplicate**

Task A2 Step 1 added `static uint32_t last_synth_us;` to the tracking block near `kmbox.c:760`. Two definitions of the same static in one translation unit is a constraint violation. **Find and delete that A2 line now**; the volatile definition in this Part-C block (Step 1 above) replaces it. Leave `last_merge_us` where A2 put it.

Run: `grep -cn 'last_synth_us *;' src/kmbox.c` after editing.
Expected: the definition appears exactly once (the volatile one here). If the grep shows two definition lines, delete the non-volatile one.

- [ ] **Step 2: Write the publish builder**

Add this function near `kmbox_send_pending` (it factors out the synth-construction currently inline at `kmbox.c:1097-1126`):

```c
// Build the next standalone synth report into the inactive buffer and publish
// it for the PIT ISR. Runs in the main loop (FPU + inject consumption here).
// Does NOT decide silence/cadence — that is the ISR's gate at emit time.
__attribute__((section(".fastrun")))
void kmbox_publish_synth(void)
{
	if (!(inject.mouse_dirty && cached_mouse_ep && mouse_layout.valid)) {
		synth_armed = false;
		return;
	}
	uint8_t w = synth_pub_idx ^ 1u;          // inactive buffer
	uint8_t *synth = synth_buf[w];
	memset(synth, 0, 16);
	uint8_t doff = mouse_layout.data_off;
	if (doff) synth[0] = mouse_layout.report_id;
	synth[doff] = inject.mouse_buttons;

	int16_t inj_dx, inj_dy;
	kmbox_take_injection(&inj_dx, &inj_dy);  // humanize_filter runs here (FPU)
	int32_t dx = inj_dx, dy = inj_dy;
	if (dx >  mouse_layout.x_max) dx =  mouse_layout.x_max;
	if (dx < -mouse_layout.x_max) dx = -mouse_layout.x_max;
	if (dy >  mouse_layout.y_max) dy =  mouse_layout.y_max;
	if (dy < -mouse_layout.y_max) dy = -mouse_layout.y_max;
	write_report_field(synth, 16, mouse_layout.x_bit, mouse_layout.x_size, doff, dx);
	write_report_field(synth, 16, mouse_layout.y_bit, mouse_layout.y_size, doff, dy);

	// Snapshot the wheel into the buffer but DO NOT consume inject.mouse_wheel
	// here — it must only be cleared when the report is actually emitted, else a
	// frame that gets superseded (synth_armed re-published, or disarmed) would
	// silently drop the wheel delta. The ISR clears it on confirmed emit.
	synth_wheel_pending[w] = 0;
	if (mouse_layout.wheel_bit != 0xFFFF && inject.mouse_wheel != 0 &&
	    mouse_layout.wheel_report_id == mouse_layout.report_id) {
		int32_t wv = inject.mouse_wheel;
		if (wv >  mouse_layout.w_max) wv =  mouse_layout.w_max;
		if (wv < -mouse_layout.w_max) wv = -mouse_layout.w_max;
		write_report_field(synth, 16, mouse_layout.wheel_bit,
		                   mouse_layout.wheel_size, doff, wv);
		synth_wheel_pending[w] = 1;   // this published frame carries the wheel
	}
	uint8_t rlen = cached_mouse_report_len;
	if (rlen == 0) rlen = (cached_mouse_maxpkt < 16) ? (uint8_t)cached_mouse_maxpkt : 16;
	synth_buf_len[w] = rlen;

	// dirty recompute mirrors the original synth block (kmbox.c:1128-1129), but
	// keep mouse_wheel in the predicate since we have NOT consumed it yet — a
	// pending wheel must keep the path dirty so a later emit still flushes it.
	inject.mouse_dirty = (inject.mouse_buttons != 0 ||
	                      inject.mouse_wheel != 0 ||
	                      humanize_pending());

	synth_pub_idx = w;        // publish the index for this buffer
	__asm volatile("dsb" ::: "memory"); // commit buffer + index before arming
	synth_armed = true;       // arm last: ISR keys off this (see emit barrier)
}
```

> Add the wheel-pending flag array next to the other Part-C state in Step 1:
> ```c
> static volatile uint8_t  synth_wheel_pending[2]; // 1 = published frame carries the wheel
> ```

- [ ] **Step 3: Write the ISR-side emitter**

Add (also `.fastrun` so it is reachable from the ITCM ISR without a slow flash call):

```c
// Emit the published synth frame from the PIT ISR if the mouse is silent and a
// frame is due. Returns true if it emitted. Pure consumer: no FPU, no humanize
// filter. last_merge_us / measured interval are owned by the main loop; reads
// here are single-load atomic on 32-bit.
__attribute__((section(".fastrun")))
bool kmbox_emit_synth_isr(uint32_t now_us)
{
	if (!synth_armed) return false;
	// Consumer-side barrier: pair with the publisher's DSB. Without it the M7's
	// out-of-order load unit may hoist the synth_pub_idx / synth_buf reads above
	// the synth_armed check and observe a stale index or half-built buffer.
	__asm volatile("dmb" ::: "memory");
	uint32_t measured_us = humanize_measured_interval_us();
	if (!synth_mouse_silent(now_us, last_merge_us, measured_us)) return false;
	if (!synth_due(now_us, last_synth_us, measured_us)) return false;
	uint8_t idx = synth_pub_idx;
	usb_device_send_report(cached_mouse_ep, synth_buf[idx], synth_buf_len[idx]);
	last_synth_us = now_us;
	// Consume the wheel ONLY now that the frame is actually on the wire (the
	// buffer for `idx` carried it). Deferring the clear to here is what prevents
	// a superseded/disarmed publish from dropping a wheel delta.
	if (synth_wheel_pending[idx]) inject.mouse_wheel = 0;
	synth_armed = false;
	return true;
}
```

> **Wheel-clear race note:** `inject.mouse_wheel` is otherwise written only in the main loop (`kmbox_inject_move` / wheel inject and the merge path). Clearing it from the ISR is a write that races with those main-loop writes. It is a single `int8_t` store, atomic on M7, but a main-loop read-modify-write on `inject.mouse_wheel` (e.g. `+=`) could lose the ISR's clear. This is acceptable: the worst case is one extra wheel unit emitted, never motion corruption. If the wheel must be exactly conserved, gate the clear behind the same `crit_enter`/`crit_exit` as the send. Document the choice; do not leave it implicit.

- [ ] **Step 4: Declare both in kmbox.h**

Add to `src/kmbox.h` (near `kmbox_send_pending`'s declaration):

```c
// Part C: PIT-driven synth emission. publish builds the next frame (main loop,
// FPU); emit_isr fires it from the PIT ISR. now_us is gpt_profile_us().
void kmbox_publish_synth(void);
bool kmbox_emit_synth_isr(uint32_t now_us);
```

- [ ] **Step 5: Build (will fail to link until C4 wires callers — that's expected)**

Run: `make 2>&1 | tail -5`
Expected: compiles; may warn about unused functions until C4. If `-Werror` flags unused-static, note these are non-static (declared in header) so they are not unused-static — the build should pass. If it fails, proceed to C4 before judging.

- [ ] **Step 6: Verify single definition of last_synth_us**

Run: `grep -n 'last_synth_us' src/kmbox.c`
Expected: exactly one definition line plus its uses in `kmbox_publish_synth`/`kmbox_emit_synth_isr` (and none left in the A2 gate, which C4 removes).

- [ ] **Step 7: Commit**

```bash
git add src/kmbox.c src/kmbox.h
git commit -m "feat: lock-free synth frame publish/emit split for PIT-driven emission"
```

---

### Task C4: Wire publish into main loop and emit into the PIT ISR

**Files:**
- Modify: `src/main.c:62-71` (ISR), `:286` (publish on tick), `:331` (replace `kmbox_send_pending` synth duties)
- Modify: `src/kmbox.c:1087-1130` (gut the inline synth construction now living in `kmbox_publish_synth`)

- [ ] **Step 1: Replace the inline synth block in kmbox_send_pending**

`kmbox_send_pending` keeps the wheel-on-separate-report-ID flush (`kmbox.c:1079-1085`) and the keyboard flush (`:1131-1133`), but its inline mouse-synth construction (`:1087-1130`) is now done by `kmbox_publish_synth`. Replace `kmbox.c:1087-1130` (from `if (merged_this_cycle) return;` through the close of the synth `if` block, up to but not including the keyboard `if`) with:

```c
	if (merged_this_cycle) return;
	// Mouse synth is now built by kmbox_publish_synth (called on the PIT tick)
	// and emitted by kmbox_emit_synth_isr from pit0_isr. Nothing to do here for
	// the mouse path; the keyboard flush below still runs from the main loop.
```

Keep the `if (__builtin_expect(inject.kb_dirty && cached_kb_ep, 0))` block that follows.

- [ ] **Step 2: Call publish on the PIT-tick path in main.c**

At `main.c:286`, after `pit_next_ldval = humanize_timing_next(pit_base_ldval);`, add:

```c
			// Build the next synth frame for the ISR to fire (FPU work here,
			// not in the ISR). Cheap no-op when nothing is injected/dirty.
			kmbox_publish_synth();
```

- [ ] **Step 2b: Add the kmbox.h include guard check in main.c**

Run: `grep -n '#include "kmbox.h"' src/main.c`
Expected: present. (It is — `kmbox_poll_fast`/`kmbox_send_pending` are already called.)

- [ ] **Step 2c: Close the never-both-emit window — stamp `last_merge_us` at the poll site**

In Part A, `last_merge_us` is stamped *inside* `kmbox_merge_report` (`kmbox.c:787`). Under Part C that leaves a narrow window: between `usb_host_interrupt_poll_zerocopy` returning a live report (`main.c:301-303`) and `kmbox_merge_report` stamping `last_merge_us`, a PIT tick can fire, see the *stale* `last_merge_us`, judge the mouse silent, and emit a synth — then main emits the passthrough for the same window. Two reports, one poll window. The window is a few instructions (~1-in-90k at 1 kHz) but non-zero and real.

Close it by stamping `last_merge_us` at the poll site, *before* `kmbox_merge_report`, co-located with the existing `humanize_record_arrival` call. Add a setter to kmbox:

In `src/kmbox.h`:
```c
void kmbox_mark_merge_arrival(uint32_t now_us); // stamp before merge (Part C)
```
In `src/kmbox.c` (near the cadence state):
```c
__attribute__((section(".fastrun")))
void kmbox_mark_merge_arrival(uint32_t now_us) { last_merge_us = now_us; }
```
Then in `src/main.c`, change the arrival block at `main.c:308-309`:
```c
				if (ep_map[m].iface_protocol == 2) {
					uint32_t arr_us = gpt_profile_us();
					humanize_record_arrival(arr_us);
					kmbox_mark_merge_arrival(arr_us); // gate synth BEFORE merge runs
				}
```
And **remove** the now-redundant `last_merge_us = gpt_profile_us();` line that Task A2 Step 2 placed at the top of `kmbox_merge_report` (`kmbox.c:787`) — stamping in both places is harmless but the poll-site stamp is the one that closes the window. Leaving the in-merge stamp would re-stamp a few instructions later with a near-identical value; delete it to keep one source of truth.

Run after editing: `grep -n 'last_merge_us *=' src/kmbox.c`
Expected: assignments only inside `kmbox_mark_merge_arrival` (the setter), none inside `kmbox_merge_report`.

> Residual (document, do not chase): a real report can still arrive at the controller between the ISR's silence check and main's next poll iteration; the ISR cannot know about a report main hasn't dequeued yet. The 2-period silence threshold plus stamping-before-merge reduces this to the USB poll granularity itself, which is the floor for any single-core design here. Accept it.

- [ ] **Step 3: Emit from the PIT ISR**

Replace `pit0_isr` at `main.c:62-71`:

```c
static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF;
	PIT_LDVAL0 = pit_next_ldval; // precomputed, no FPU in ISR
	pit_tick_pending = true;
	// Fire the pre-built synth frame at the humanized poll cadence. No FPU /
	// no humanize filter here; gates internally on silence + due so a real
	// report arriving since publish suppresses the synth (last_merge_us will
	// have advanced past the silence window).
	kmbox_emit_synth_isr(gpt_profile_us());
	// The TFLG W1C is a posted write; without a DSB the NVIC can still see
	// the IRQ asserted at exception return and immediately re-enter — a
	// spurious double tick (timing jitter on the injection cadence).
	__asm volatile("dsb" ::: "memory");
}
```

- [ ] **Step 4: Confirm gpt_profile.h is included in main.c**

Run: `grep -n 'gpt_profile' src/main.c`
Expected: `#include "gpt_profile.h"` present and `gpt_profile_init()` at line ~122. (It is.)

- [ ] **Step 5: Build both variants, zero warnings**

Run: `make clean && make && make PROTOCOL=ferrum`
Expected: zero warnings, both `firmware.hex` produced.

- [ ] **Step 6: Confirm the ISR pulls no soft-float / no slow libcalls**

Run: `make` then
`arm-none-eabi-objdump -d firmware.elf | awk '/<pit0_isr>:/{f=1} f{print} /^$/{if(f)exit}'`
Expected: no `bl __aeabi_*`, no `bl` into `humanize_filter`/`expf`/`sqrtf`. Only the call to `kmbox_emit_synth_isr` (and from there `usb_device_send_report`/`memcpy`), all `.fastrun`. If any float libcall appears, the FPU leaked into the ISR — stop and re-check that `humanize_filter` is only reached via `kmbox_publish_synth` (main loop), not `kmbox_emit_synth_isr`.

- [ ] **Step 7: Confirm emitter and its callees are in ITCM (.fastrun)**

Run: `arm-none-eabi-nm firmware.elf | grep -E 'kmbox_emit_synth_isr|kmbox_publish_synth|usb_device_send_report'` and cross-check the addresses fall in the ITCM range per `core/imxrt1062_mm.ld`.
Expected: all in `.fastrun`/ITCM. A flash-resident ISR callee would add cache-miss jitter — the thing this whole part exists to remove.

- [ ] **Step 8: Run host tests (regression)**

Run: `make test`
Expected: all pass.

- [ ] **Step 9: Commit**

```bash
git add src/main.c src/kmbox.c src/kmbox.h
git commit -m "feat: emit synth report from PIT ISR at humanized poll cadence"
```

---

### Task C5: On-target smoke test — basic injection still works

**Files:** none (hardware validation)

- [ ] **Step 1: Flash**

Run: `make flash`
Expected: teensy_loader_cli reports success.

- [ ] **Step 2: Real-mouse passthrough sanity**

Move the physical mouse. Cursor tracks normally (merge path unaffected). No stutter, no stuck cursor.

- [ ] **Step 3: Silent injection sanity**

With the physical mouse still, issue an injected move (e.g. `km.move(100,0)` over the link). Cursor moves smoothly via the synth path (now ISR-driven). Confirm it actually moves — a stuck synth means the ISR gate never passes (check `synth_armed` is set by publish; check `last_merge_us` isn't being stamped without real reports).

- [ ] **Step 4: Mixed sanity**

Inject while slowly moving the physical mouse. Motion composes; no doubled or dropped cursor jumps at the moment you lift off / touch down (the merge↔synth handoff).

- [ ] **Step 5: Commit a note (no code)**

```bash
git commit --allow-empty -m "test: on-target smoke — passthrough + silent + mixed injection OK"
```

---

### Task C6: On-target timing validation — logic analyzer

This is the checkpoint that licenses Part C. Until it passes, Part A is the shippable artifact.

**Files:** none (hardware validation)

- [ ] **Step 1: Capture ENDPTPRIME / ENDPTSTAT during sustained silent injection**

With a logic analyzer (or the on-chip trace if available) on the USB1 device-controller activity, drive continuous injected motion with the physical mouse idle, on a 1 kHz device. Confirm: exactly one report per poll window, no double-prime of the mouse EP, no missed prime (no stuck `ENDPTPRIME` bit), endpoint never halts.
Expected: clean 1-per-window cadence with no double-prime (the C2 single-lock guard should already prevent it). If a double-prime still appears, the prime path needs the ATDTW handshake in `prime_int_ep` — stop and investigate before relying on Part C.

- [ ] **Step 2: Repeat on the fastest device you have (ideally 8 kHz, else 1 kHz)**

Confirm the silent-injection rate now matches the device poll rate (the bug Part A fixed, now ISR-timed), and the merge↔synth transition shows no gap or burst at handoff.

- [ ] **Step 3: Measure PIT ISR duration**

Toggle a spare GPIO at ISR entry/exit (or read DWT cycle counter around the body) and capture the worst-case ISR length under sustained injection. Confirm it does not starve DMA (prio 96/160) or USB-host (144) servicing — i.e. no RX overruns (`rx_drv_overrun_count`), no host transfer errors, during a stress run.
Expected: ISR well under one poll period; counters stable.

- [ ] **Step 4: Critical-section latency sanity**

Confirm the `crit_enter`/`crit_exit` windows in `usb_device_poll`/`send_report` don't delay the PIT tick enough to matter: capture PIT period jitter with and without sustained device traffic. It should stay within the deliberate `humanize_timing_next` band, not exceed it.

- [ ] **Step 5: Record results**

```bash
git commit --allow-empty -m "test: on-target LA validation — 1-per-window, ISR budget, no double-prime"
```

---

**PART C COMPLETE** once C6 passes. If C6 reveals a hardware prime race even with the single-lock guard (e.g. a missed prime that needs the ATDTW handshake), revert Part C commits (Part A stands alone) and reassess `prime_int_ep`.

---

## Risks & rollback

- **Part A** is low risk, no concurrency change; rollback is a single `git revert` of the A2 commit.
- **Part C** risk concentrates in C2 (USB controller prime path under concurrency) and is gated by C6. Because C is committed separately, `git revert` of the C-range restores the working Part-A firmware. C2 holds **one** critical section across the whole `usb_device_send_report` body — the lock-splitting optimization was rejected during review as a statically-provable DMA-buffer corruption race (see C2 Step 3). The ATDTW (Add-dTD-TripWire) handshake is NOT added by this plan; if C6 still shows missed primes, that is the next investigation, out of scope here.
- **Known residual (accepted):** the never-both-emit invariant is closed to the USB-poll granularity by stamping `last_merge_us` before the merge (C4 Step 2c), but a single-core design cannot make it provably zero — the ISR cannot know about a report the main loop has not yet dequeued. Documented at C4 Step 2c; surfaced here so it is not mistaken for an oversight.
- **Wheel conservation under Part C** is best-effort, not exact: the ISR clears `inject.mouse_wheel` after a confirmed emit, which races a main-loop wheel RMW (worst case: one extra wheel unit, never motion corruption). C3 documents the exact-conservation alternative (clear under the critical section) if a future requirement needs it.
- **F_CPU dependency:** both parts rely on GPT2 being a true 1 MHz tick, which `gpt_profile.h:24` already enforces with a `_Static_assert(F_CPU > 528 MHz)`. No new clock assumptions introduced.

## Spec-to-task traceability

- "Drive synth cadence from measured interval, not 1 kHz" → A1 (module), A2 (wiring), A3 (docs).
- "Wrap-safe across GPT2's 71.6-min rollover" → A1 Step 3 wrap tests; all comparisons unsigned-subtraction.
- "Fall back to 1 kHz if no measurement" → A1 `synth_period_us(0)` + test (defensive; see background note — seeded non-zero at init).
- "Never exceed device service rate / floor at 8 kHz" → A1 `SYNTH_PERIOD_FLOOR_US` + clamp test.
- "Move emission to PIT ISR, FPU out of ISR" → C3 (publish builds w/ FPU in main; emit is pure consumer), C4 Step 6 (objdump proves no float libcall in ISR).
- "Guard shared USB-TX state" → C1 (critical section), C2 (single lock across `usb_device_send_report` + `usb_device_poll` RMW), C6 Step 1/4 (validation).
- "Lock-free publish/consume memory ordering" → C3 publisher DSB before `synth_armed=true`; C3 consumer DMB after the `synth_armed` check in `kmbox_emit_synth_isr`.
- "Cross-context scalars are race-safe" → C3 `volatile` on `synth_armed`/`synth_pub_idx`/`synth_buf_len`/`last_synth_us`/`synth_wheel_pending`; uint32/byte stores atomic on M7.
- "Preserve the never-both-emit invariant" → C4 Step 2c (stamp `last_merge_us` before the merge, closing the ISR window) + C3 emit silence/due gate + retained `merged_this_cycle` early-return in `kmbox_send_pending`.
- "Wheel delta not dropped on superseded publish" → C3 (`synth_wheel_pending[]`, clear only on confirmed ISR emit).
- "Keep ISR callees in ITCM" → C4 Step 7.
