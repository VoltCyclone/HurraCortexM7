# Built-in Humanization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the opt-in `smooth.c` trajectory generator with a single always-on, default-on humanization *filter* that perturbs every injected mouse delta (jerk, micro-noise, dither, human caps) while leaving real-mouse passthrough untouched.

**Architecture:** One module, `src/humanize.c`, exposes a pure per-frame filter `humanize_filter(&dx,&dy)` that transforms the *injected* delta only. It is the single consumption point for pending injection (`kmbox_take_injection`), called by both the merge and synth send paths in `src/kmbox.c`. Displacement is conserved by a 1st-order exponential "owed" drain; correlated noise + sub-pixel dither break Tier-1 signatures; a human per-frame cap prevents teleport/saturation. The `smooth.c` generator (queue/easing/Fitts) is retired — it was already off the active integration path (`bridge → hurra_move`, never `move_smooth`).

**Tech Stack:** Bare-metal C (i.MXRT1062, ARM GCC `-O2 -ffast-math`), fixed-point + single-precision float, host-compiled unit tests via a native `gcc` target.

---

## File Structure

- `src/humanize.h` — public API: `humanize_init`, `humanize_filter`, `humanize_timing_next`, `humanize_set_level`, `humanize_reseed`. (Replaces the stub.)
- `src/humanize.c` — the filter: RNG + fast-math primitives (moved from `smooth.c`), per-session state, per-frame filter, timing jitter, level presets. (Replaces the stub.)
- `src/kmbox.c` — add `kmbox_take_injection`; route merge fast/slow + `kmbox_send_pending` injection consumption through it; call `humanize_init` is done from main.
- `src/main.c` — call `humanize_init(interval_us)`; replace the PIT-tick `smooth_*` generator block with `humanize_timing_next`.
- `src/ferrum.c` — add `km.human(level)` command.
- `src/hurra.c` — `MOUSE_MOVE_SMOOTH` (0x11) routes to the same raw path as `MOUSE_MOVE`.
- `src/smooth.c`, `src/smooth.h`, `src/smooth_config.h` — deleted.
- `Makefile`, `CLAUDE.md` — drop `smooth`, add the host-test target.
- `test/humanize_test.c` — host unit tests.
- `tools/humanization_analyze.py` — statistical Tier-1 analyzer.

---

## Task 1: Host test harness

**Files:**
- Create: `test/humanize_test.c`
- Modify: `Makefile` (add `test` target)

- [ ] **Step 1: Add a native test target to the Makefile**

Append to `Makefile`:

```makefile
# Host-native unit tests (no cross-compile). humanize.c must stay free of
# hardware headers behind HUMANIZE_HOSTTEST so it builds with system gcc.
.PHONY: test
test:
	cc -std=c11 -O2 -DHUMANIZE_HOSTTEST -Isrc -o /tmp/humanize_test \
	   test/humanize_test.c src/humanize.c -lm
	/tmp/humanize_test
```

- [ ] **Step 2: Write the test scaffold (one passing assert)**

Create `test/humanize_test.c`:

```c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "humanize.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    humanize_init(1000);            /* 1 ms frame */
    CHECK(1, "scaffold");
    printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
```

- [ ] **Step 3: Stub the header so the scaffold links**

Create `src/humanize.h` (final API; bodies arrive in Task 2):

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Always-on humanization filter. Operates on the INJECTED mouse delta only;
 * real-mouse passthrough is never routed through it. */
void     humanize_init(uint32_t interval_us);   /* seed + level default */
void     humanize_filter(int16_t *dx, int16_t *dy); /* in-place, per frame */
uint32_t humanize_timing_next(uint32_t base_ldval, bool *out_skip);
void     humanize_set_level(uint8_t level);      /* 0=off..3=strong */
```

- [ ] **Step 4: Create a minimal `src/humanize.c` so the target builds**

Replace `src/humanize.c` with:

```c
#include "humanize.h"
void humanize_init(uint32_t interval_us) { (void)interval_us; }
void humanize_filter(int16_t *dx, int16_t *dy) { (void)dx; (void)dy; }
uint32_t humanize_timing_next(uint32_t b, bool *s) { *s = false; return b; }
void humanize_set_level(uint8_t level) { (void)level; }
```

- [ ] **Step 5: Run the harness**

Run: `make test`
Expected: `ALL PASSED`

- [ ] **Step 6: Commit**

```bash
git add Makefile test/humanize_test.c src/humanize.h src/humanize.c
git commit -m "test: host harness + humanize.c filter API skeleton"
```

---

## Task 2: Filter core — conservation, idle gate, clamp

**Files:**
- Modify: `src/humanize.c`
- Test: `test/humanize_test.c`

- [ ] **Step 1: Write failing tests for the three invariants**

Add to `test/humanize_test.c` `main()` before the summary:

```c
    /* (A) Conservation: summed output == summed injected, within rounding. */
    humanize_init(1000);
    humanize_set_level(2);
    long sx = 0;
    for (int i = 0; i < 5000; i++) {           /* steady 3 px/frame stream */
        int16_t dx = 3, dy = 0;
        humanize_filter(&dx, &dy);
        sx += dx;
    }
    /* feed zeros to flush the owed accumulator */
    for (int i = 0; i < 200; i++) { int16_t dx = 0, dy = 0; humanize_filter(&dx,&dy); sx += dx; }
    CHECK(labs(sx - 5000L*3) <= 2, "conservation: output sum tracks input sum");

    /* (B) Idle gate: zero in, settled -> zero out (no tremor on still cursor). */
    humanize_init(1000);
    for (int i = 0; i < 50; i++) { int16_t dx=0, dy=0; humanize_filter(&dx,&dy); }
    int moved = 0;
    for (int i = 0; i < 500; i++) { int16_t dx=0, dy=0; humanize_filter(&dx,&dy); if (dx||dy) moved=1; }
    CHECK(!moved, "idle gate: still cursor stays still");

    /* (C) Human cap: a huge single injection never emits a teleport frame. */
    humanize_init(1000);
    humanize_set_level(2);
    int16_t bx = 30000, by = 0; long total = 0; int maxframe = 0;
    humanize_filter(&bx, &by); total += bx; if (abs(bx) > maxframe) maxframe = abs(bx);
    for (int i = 0; i < 4000; i++) { int16_t dx=0,dy=0; humanize_filter(&dx,&dy); total += dx; if (abs(dx)>maxframe) maxframe=abs(dx); }
    CHECK(maxframe <= 127, "cap: no single frame exceeds human ceiling");
    CHECK(labs(total - 30000) <= 4, "cap: clamped motion is carried, not dropped");
```

- [ ] **Step 2: Run to verify failure**

Run: `make test`
Expected: FAIL (conservation/idle/cap assertions fail against the stub).

- [ ] **Step 3: Implement the filter core**

Replace `src/humanize.c` with (RNG/fast-math primitives are copied verbatim from `smooth.c`; the filter is new):

```c
#include "humanize.h"
#include <string.h>
#include <math.h>

/* ── tunables ───────────────────────────────────────────────────────── */
#define HZ_DEFAULT_LEVEL   2        /* boot default: on, "normal" */
#define HZ_MAX_PER_FRAME   127      /* human per-frame ceiling (counts) */
#define HZ_IDLE_EPS        0.01f    /* |owed| below this = settled */

/* Per-level presets: drain rate k (fraction of owed emitted per frame),
 * and perpendicular-noise amplitude (counts RMS at speed). Level 0 = off. */
static const float HZ_DRAIN[4] = { 1.0f, 0.55f, 0.40f, 0.30f };
static const float HZ_NOISE[4] = { 0.0f, 0.15f, 0.35f, 0.60f };

static struct {
    uint8_t  level;
    float    drain, noise_amp;
    float    owed_x, owed_y;        /* undelivered injected motion */
    float    res_x, res_y;          /* sub-pixel residual */
    float    ewma;                  /* noise correlation alpha */
    float    n_perp;                /* correlated perpendicular noise state */
    uint32_t a, b, c, ctr;          /* SFC32 */
    uint32_t timing_lfsr;
    int      idle;
} S;

/* ── RNG (verbatim from smooth.c) ───────────────────────────────────── */
static inline uint32_t sfc32(void) {
    uint32_t t = S.a + S.b + S.ctr++;
    S.a = S.b ^ (S.b >> 9);
    S.b = S.c + (S.c << 3);
    S.c = ((S.c << 21) | (S.c >> 11)) + t;
    return t;
}
static inline float sfc32_uniform(void) {       /* [-1, 1) */
    int32_t bal = (int32_t)(sfc32() >> 8) - 0x800000;
    return (float)bal * (1.0f / 8388608.0f);
}

/* ── seeding ────────────────────────────────────────────────────────── */
#ifdef HUMANIZE_HOSTTEST
static uint32_t hw_entropy(void) { return 0x12345678u; }   /* deterministic */
#else
#include "imxrt.h"
static uint32_t hw_entropy(void) {
    volatile uint32_t *dwt_ctrl = (volatile uint32_t *)0xE0001000;
    volatile uint32_t *dwt_cyc  = (volatile uint32_t *)0xE0001004;
    *dwt_ctrl |= 1;
    volatile uint32_t *uid0 = (volatile uint32_t *)0x401F4410;
    return *dwt_cyc ^ *uid0;
}
#endif

void humanize_set_level(uint8_t level) {
    if (level > 3) level = 3;
    S.level     = level;
    S.drain     = HZ_DRAIN[level];
    S.noise_amp = HZ_NOISE[level];
}

void humanize_init(uint32_t interval_us) {
    memset(&S, 0, sizeof(S));
    uint32_t seed = hw_entropy();
    S.a = seed ^ 0xCAFEBABEu; S.b = seed ^ 0xDEADBEEFu;
    S.c = seed ^ 0x8BADF00Du; S.ctr = 1;
    if (!S.a) S.a = 0xCAFEBABEu;
    for (int i = 0; i < 16; i++) sfc32();
    S.timing_lfsr = sfc32() | 1u;
    /* noise correlation: ~per-ms alpha, mild */
    S.ewma = 0.85f;
    /* per-session personality: small noise-amp jitter from hardware seed */
    humanize_set_level(HZ_DEFAULT_LEVEL);
    S.noise_amp *= 1.0f + 0.15f * sfc32_uniform();
    (void)interval_us;
}

/* Drain `owed` by a fraction this frame (1st-order response -> rise/settle,
 * nonzero varying jerk), add correlated perpendicular noise, dither via
 * sub-pixel residual, clamp to the human ceiling and carry the overflow.
 * Conserves total displacement: everything not emitted stays in owed/res. */
static int16_t drain_axis(float *owed, float *res, float emit_v, float noise) {
    float want = emit_v + noise + *res;
    /* clamp emit to human ceiling; overflow returns to owed via res path */
    if (want >  (float)HZ_MAX_PER_FRAME) want =  (float)HZ_MAX_PER_FRAME;
    if (want < -(float)HZ_MAX_PER_FRAME) want = -(float)HZ_MAX_PER_FRAME;
    int16_t out = (int16_t)(want >= 0 ? (want + 0.5f) : (want - 0.5f));
    *res = want - (float)out;                 /* sub-pixel + clamp remainder */
    *owed -= emit_v;                          /* consume the drained portion */
    return out;
}

void humanize_filter(int16_t *dx, int16_t *dy) {
    if (S.level == 0) return;                 /* off: passthrough */

    S.owed_x += (float)*dx;
    S.owed_y += (float)*dy;

    /* idle gate: nothing owed and residual settled -> emit nothing */
    if (fabsf(S.owed_x) < HZ_IDLE_EPS && fabsf(S.owed_y) < HZ_IDLE_EPS &&
        fabsf(S.res_x) < 0.5f && fabsf(S.res_y) < 0.5f) {
        if (S.idle < 1000) S.idle++;
        *dx = 0; *dy = 0;
        return;
    }
    S.idle = 0;

    float ex = S.owed_x * S.drain;            /* this frame's emitted velocity */
    float ey = S.owed_y * S.drain;

    /* correlated perpendicular noise, scaled by speed */
    float speed = sqrtf(ex*ex + ey*ey);
    S.n_perp = S.ewma * S.n_perp + (1.0f - S.ewma) * sfc32_uniform();
    float nmag = S.n_perp * S.noise_amp * speed;
    float nx = 0.0f, ny = 0.0f;
    if (speed > 1e-3f) { nx = -ey / speed * nmag; ny = ex / speed * nmag; }

    *dx = drain_axis(&S.owed_x, &S.res_x, ex, nx);
    *dy = drain_axis(&S.owed_y, &S.res_y, ey, ny);
}

uint32_t humanize_timing_next(uint32_t base_ldval, bool *out_skip) {
    *out_skip = false;
    if (S.level == 0) return base_ldval;
    /* xorshift32 jitter, ±~12%, never a skipped poll (bimodal = detectable) */
    S.timing_lfsr ^= S.timing_lfsr << 13;
    S.timing_lfsr ^= S.timing_lfsr >> 17;
    S.timing_lfsr ^= S.timing_lfsr << 5;
    float u = (float)(S.timing_lfsr >> 8) * (1.0f / 16777216.0f) - 0.5f;
    float r = (float)base_ldval * (1.0f + 0.12f * u);
    float lo = (float)base_ldval * 0.80f, hi = (float)base_ldval * 1.20f;
    if (r < lo) r = lo; if (r > hi) r = hi;
    return (uint32_t)r;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `make test`
Expected: `ALL PASSED`

- [ ] **Step 5: Add a jerk/quantization regression test**

Add before the summary in `test/humanize_test.c`:

```c
    /* (D) Tier-1: a constant-velocity stream must not emit a long run of
     *     identical values (anti-quantization) and must vary frame-to-frame. */
    humanize_init(1000);
    humanize_set_level(2);
    int max_run = 0, run = 0; int16_t prev = -999;
    for (int i = 0; i < 3000; i++) {
        int16_t dx = 5, dy = 5; humanize_filter(&dx, &dy);
        if (dx == prev) { run++; if (run > max_run) max_run = run; } else run = 0;
        prev = dx;
    }
    CHECK(max_run < 200, "anti-quantization: no long identical-value run");
```

- [ ] **Step 6: Run and commit**

Run: `make test` → Expected: `ALL PASSED`

```bash
git add src/humanize.c src/humanize.h test/humanize_test.c
git commit -m "feat: humanization filter core (conserving 1st-order drain + noise + dither + cap)"
```

---

## Task 3: Single injection-consumption point in kmbox

**Files:**
- Modify: `src/kmbox.c` (add helper, route fast/slow merge + send_pending through it)
- Modify: `src/kmbox.h` (none required; helper is static)

- [ ] **Step 1: Add the consumption helper near the merge code**

In `src/kmbox.c`, add `#include "humanize.h"` if not present, and add above `kmbox_merge_report`:

```c
/* Pull this frame's injected delta out of the pending accumulators, run it
 * through the humanization filter, and return the humanized amount to apply.
 * Real-mouse passthrough is NOT routed here — only injected motion. */
static void kmbox_take_injection(int16_t *out_dx, int16_t *out_dy)
{
    int16_t dx = inject.mouse_dx;
    int16_t dy = inject.mouse_dy;
    inject.mouse_dx = 0;          /* filter now owns the remainder via owed */
    inject.mouse_dy = 0;
    humanize_filter(&dx, &dy);
    *out_dx = dx;
    *out_dy = dy;
}
```

- [ ] **Step 2: Route the fast merge path through it**

In `kmbox_merge_report` fast path, replace the block that reads `inject.mouse_dx`/`inject.mouse_dy` into the report (the `done_dx`/`done_dy` carry block from the current code) so the injected component comes from `kmbox_take_injection`. Concretely, at the top of the fast-path body (after the buttons line) insert:

```c
                int16_t inj_dx, inj_dy;
                kmbox_take_injection(&inj_dx, &inj_dy);
```

and change each axis to add `inj_dx`/`inj_dy` (instead of `inject.mouse_dx`/`inject.mouse_dy`), e.g. for the 8-bit X axis:

```c
                    int32_t rx = (int8_t)report[mouse_layout.x_byte];
                    int32_t mx = rx + inj_dx;
                    if (mx >  mouse_layout.x_max) mx =  mouse_layout.x_max;
                    if (mx < -mouse_layout.x_max) mx = -mouse_layout.x_max;
                    report[mouse_layout.x_byte] = (uint8_t)(int8_t)mx;
```

(Apply the same `inj_dx`/`inj_dy` substitution to the 16-bit X, both Y branches.) The field clamp stays as a hard safety bound; the filter's own cap means it rarely triggers. Update the telemetry call to `proto_notify_axes(inj_dx, inj_dy, w_tlm);` and the dirty recompute to drop the now-zeroed dx/dy:

```c
                inject.mouse_dirty = (inject.mouse_buttons != 0 ||
                                      inject.mouse_wheel != 0);
```

- [ ] **Step 3: Route the slow path and synth path the same way**

In `kmbox_merge_report_slow`, replace `inject.mouse_dx`/`inject.mouse_dy` usage with a leading `kmbox_take_injection(&inj_dx,&inj_dy);` and use `inj_dx/inj_dy`; drop the old `inject.mouse_dx = 0` lines. In `kmbox_send_pending`, replace the `int32_t dx = inject.mouse_dx; int32_t dy = inject.mouse_dy;` lines with:

```c
        int16_t inj_dx, inj_dy;
        kmbox_take_injection(&inj_dx, &inj_dy);
        int32_t dx = inj_dx;
        int32_t dy = inj_dy;
```

and remove the subsequent `inject.mouse_dx = 0; inject.mouse_dy = 0;` (the filter owns the remainder now). Keep wheel handling unchanged.

- [ ] **Step 4: Make `inject.mouse_dirty` stay true while the filter still owes motion**

The filter holds undelivered motion in `owed` after `inject.mouse_dx` is zeroed, so the send paths must keep firing. Add to `src/humanize.h`:

```c
bool humanize_pending(void);   /* true while owed motion remains to emit */
```

and to `src/humanize.c`:

```c
bool humanize_pending(void) {
    return fabsf(S.owed_x) >= HZ_IDLE_EPS || fabsf(S.owed_y) >= HZ_IDLE_EPS;
}
```

Then in `kmbox.c`, anywhere `inject.mouse_dirty` is recomputed, OR in `humanize_pending()`:

```c
                inject.mouse_dirty = (inject.mouse_buttons != 0 ||
                                      inject.mouse_wheel != 0 ||
                                      humanize_pending());
```

- [ ] **Step 5: Build firmware (both protocols)**

Run: `make clean && make && make clean && make PROTOCOL=ferrum`
Expected: both link, no warnings.

- [ ] **Step 6: Commit**

```bash
git add src/kmbox.c src/humanize.c src/humanize.h
git commit -m "feat: route all injection through the humanization filter (single point)"
```

---

## Task 4: Wire init + timing into main loop; retire the generator's PIT block

**Files:**
- Modify: `src/main.c`

- [ ] **Step 1: Initialise the filter where `smooth_init` was**

In `src/main.c`, replace line 192 `smooth_init(interval_us);` with:

```c
		humanize_init(interval_us);
```

and replace `#include "smooth.h"` with `#include "humanize.h"` (and remove any `humanize_init()` stub call if a separate one exists).

- [ ] **Step 2: Replace the PIT-tick generator block with timing jitter only**

In the main loop (current lines ~231-239), replace:

```c
		if (pit_tick_pending) {
			pit_tick_pending = false;
			did_work = true;
			bool skip = false;
			uint32_t next_ldval = smooth_timing_next(pit_base_ldval, &skip);
			pit_next_ldval = next_ldval;
			if (!skip) {
				int16_t sx, sy;
				smooth_process_frame(&sx, &sy);
				if (sx || sy) kmbox_inject_smooth(sx, sy);
			}
		}
```

with:

```c
		if (pit_tick_pending) {
			pit_tick_pending = false;
			did_work = true;
			bool skip = false;
			pit_next_ldval = humanize_timing_next(pit_base_ldval, &skip);
		}
```

- [ ] **Step 3: Build**

Run: `make`
Expected: links (will still reference `kmbox_inject_smooth`? no — removed). Any remaining `smooth_*` references are removed in Task 6; if the build breaks on `smooth.h` here, that's expected until Task 6. To keep this task self-contained, temporarily keep `src/smooth.c` compiled — it just no longer runs.

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add src/main.c
git commit -m "feat: init humanization filter + drive PIT jitter from it"
```

---

## Task 5: `km.human(level)` command — default-on, minimal

**Files:**
- Modify: `src/ferrum.c`
- Modify: `src/hurra.c` (optional binary type; ASCII path is primary)
- Modify: `hurra-app/src/ferrum_parser.c`, `hurra-app/src/bridge.c` (bridge passthrough)

- [ ] **Step 1: Add the firmware command**

In `src/ferrum.c`, add `#include "humanize.h"` and a handler near `cmd_baud`:

```c
static void cmd_human(arg_t *args, uint8_t nargs)
{
	if (nargs != 1) return;
	int32_t n;
	if (!parse_int(args[0].p, args[0].len, &n)) return;
	if (n < 0) n = 0;
	if (n > 3) n = 3;
	humanize_set_level((uint8_t)n);
}
```

and register it in `dispatch` next to `"baud"`:

```c
	if (name_is(name, name_len, "human"))        { cmd_human(args, nargs); return; }
```

- [ ] **Step 2: Confirm default-on requires no command**

`humanize_init` already calls `humanize_set_level(HZ_DEFAULT_LEVEL)` (= 2). Verify no code path resets it to 0 on connect. Grep:

Run: `grep -rn "humanize_set_level\|HZ_DEFAULT_LEVEL" src/`
Expected: only `humanize_init` (default) and `cmd_human` (explicit) set it.

- [ ] **Step 3: Bridge passthrough (so an external sender can issue it)**

In `hurra-app/src/ferrum_parser.c`, add an `on_human` callback to `ferrum_callbacks_t`, parse `human` like other single-int setters, and in `hurra-app/src/bridge.c` wire `cbs.on_human = cb_human;` where `cb_human` sends the command on to the device. If the device firmware is reached via the Ferrum-ASCII bridge, the bridge must forward `km.human(n)`; mirror how `cb_baud`/`cmd_baud` is forwarded. (If no binary Hurra type exists for it, forward as a passthrough text command.)

- [ ] **Step 4: Build firmware + bridge**

Run: `make PROTOCOL=ferrum && make` then `cd ../hurra-app && make`
Expected: all build.

- [ ] **Step 5: Commit**

```bash
git add src/ferrum.c ../hurra-app/src/ferrum_parser.c ../hurra-app/src/bridge.c
git commit -m "feat: km.human(level) control, default-on (level 2)"
```

---

## Task 6: Retire `smooth.c` and consolidate

**Files:**
- Delete: `src/smooth.c`, `src/smooth.h`, `src/smooth_config.h`
- Modify: `src/hurra.c`, `src/kmbox.c`, `src/kmbox.h`, `Makefile`, `CLAUDE.md`

- [ ] **Step 1: Repoint `MOUSE_MOVE_SMOOTH` to the raw path**

In `src/hurra.c`, change the `move_smooth` handler (line ~209) from `act_move(..., true)` to `act_move(..., false)` so smoothed frames become normal injection (which the filter humanizes):

```c
    act_move(rd_i16le(&msg->data[0]), rd_i16le(&msg->data[2]), false);
```

- [ ] **Step 2: Remove `act_move`'s smooth branch and `kmbox_inject_smooth`**

In `src/actions.c`, drop the `if (smooth) smooth_inject(...)` branch so `act_move` always calls `kmbox_inject_mouse`. In `src/kmbox.c`, delete `kmbox_inject_smooth` and the field-cap call `smooth_set_max_per_frame(...)` in `kmbox_cache_endpoints` (the filter's `HZ_MAX_PER_FRAME` replaces it). Remove `kmbox_inject_smooth` from `src/kmbox.h`. Remove `#include "smooth.h"`.

- [ ] **Step 3: Delete the files and drop from the build**

```bash
git rm src/smooth.c src/smooth.h src/smooth_config.h
```

In `Makefile`, remove `src/smooth.c` from `CORE_SRC` (line 46) and `src/smooth.o` from `HOT_SRC` (line 61).

- [ ] **Step 4: Build both protocols, run host tests**

Run: `make clean && make && make clean && make PROTOCOL=ferrum && make test`
Expected: both firmwares link with no `smooth_*`/`kmbox_inject_smooth` undefined-reference errors; `ALL PASSED`.

- [ ] **Step 5: Update CLAUDE.md**

In `CLAUDE.md`, change the `src/smooth.c` line under Key Files to describe `src/humanize.c` (always-on humanization filter) and note the generator was retired.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor: retire smooth.c trajectory generator; humanize.c is the single path"
```

---

## Task 7: Statistical Tier-1 analyzer

**Files:**
- Create: `tools/humanization_analyze.py`

- [ ] **Step 1: Write the analyzer**

Create `tools/humanization_analyze.py`:

```python
#!/usr/bin/env python3
"""Tier-1 humanization analyzer.

Reads a motion trace (one "dx dy" pair per line, whitespace-separated; lines
that fail to parse are skipped) and reports the kinematic signatures anti-cheat
Tier-1 detectors look for. Compare a captured device-output trace against a
real human baseline (a passthrough-mouse capture).

Usage:  tools/humanization_analyze.py trace.txt [--baseline human.txt]
"""
import sys, argparse, math, statistics

def load(path):
    xs = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 2: continue
            try: xs.append((float(p[0]), float(p[1])))
            except ValueError: continue
    return xs

def metrics(tr):
    # velocity per frame = the delta itself (1 frame dt)
    v = [math.hypot(dx, dy) for dx, dy in tr]
    a = [v[i] - v[i-1] for i in range(1, len(v))]          # acceleration
    j = [a[i] - a[i-1] for i in range(1, len(a))]          # jerk
    zero_jerk = sum(1 for x in j if abs(x) < 1e-9) / max(len(j), 1)
    # longest identical-value run on dx (quantization tell)
    run = mx = 1
    for i in range(1, len(tr)):
        run = run + 1 if tr[i][0] == tr[i-1][0] else 1
        mx = max(mx, run)
    return {
        "frames": len(tr),
        "zero_jerk_frac": zero_jerk,
        "jerk_rms": (statistics.pstdev(j) if len(j) > 1 else 0.0),
        "max_identical_run": mx,
        "vel_mean": (statistics.mean(v) if v else 0.0),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--baseline")
    a = ap.parse_args()
    m = metrics(load(a.trace))
    print(f"trace: {a.trace}")
    for k, val in m.items(): print(f"  {k}: {val:.4f}" if isinstance(val,float) else f"  {k}: {val}")
    # Heuristic pass/fail (tune against the human baseline)
    flags = []
    if m["zero_jerk_frac"] > 0.20: flags.append("HIGH zero-jerk fraction (robotic)")
    if m["max_identical_run"] > 200: flags.append("long identical-value run (quantized)")
    if m["jerk_rms"] < 0.05 and m["vel_mean"] > 1.0: flags.append("near-zero jerk variance (too smooth)")
    if a.baseline:
        b = metrics(load(a.baseline))
        print(f"baseline: {a.baseline}")
        for k, val in b.items(): print(f"  {k}: {val:.4f}" if isinstance(val,float) else f"  {k}: {val}")
    print("RESULT:", "FLAGS: " + "; ".join(flags) if flags else "looks human (Tier-1)")
    return 1 if flags else 0

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Smoke-test against a synthetic robotic trace**

Run:

```bash
python3 - <<'PY' > /tmp/robotic.txt
for _ in range(2000): print("5 0")
PY
python3 tools/humanization_analyze.py /tmp/robotic.txt; echo "exit=$?"
```

Expected: flags the constant-velocity/identical-run trace (`exit=1`).

- [ ] **Step 3: Commit**

```bash
git add tools/humanization_analyze.py
git commit -m "test: Tier-1 humanization statistical analyzer"
```

---

## Self-Review

**Spec coverage:**
- §3 always-on signal filter, 1st-order light dynamics → Tasks 1–2. ✓
- §4 integration (injected-only, passthrough untouched, kmbox-compatible) → Task 3 (`kmbox_take_injection` on injection only). ✓
- §5 placement / single chokepoint → Task 3 (merge fast/slow + synth all consume via one helper). ✓
- §6 filter algorithm (idle gate, conserving drain, noise, dither, cap, timing) → Tasks 2 & 4. ✓
- §7 consolidation / retire generator → Task 6. ✓
- §8 default-on + minimal `km.human` → Tasks 2 (init default) & 5. ✓
- §9 verification (unit core + analyzer) → Tasks 1–2, 7. ✓
- §10 limitation (Tier-2 not covered) — documented in spec; no task needed.

**Placeholder scan:** no TBD/TODO; every code step has full code. Task 3 substitutions reference the existing carry-block by name and show the per-axis replacement pattern explicitly.

**Type consistency:** `humanize_init/filter/timing_next/set_level/pending` signatures match between `humanize.h`, `humanize.c`, tests, and callers. `kmbox_take_injection(int16_t*, int16_t*)` used consistently. `HZ_MAX_PER_FRAME` (127) matches the field-cap it replaces.

**Open risk to verify during execution:** the acceptance test "feed a recorded human trace, filter must not degrade human-ness" (§9) needs a captured baseline; Task 7 provides the analyzer but the capture path (HID logger on target, or firmware delta telemetry) is hardware-dependent and should be set up when running on-device.
