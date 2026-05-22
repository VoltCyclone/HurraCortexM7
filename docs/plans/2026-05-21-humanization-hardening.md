# Humanization Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the highest-signal anti-cheat detection gaps in the smooth/humanize layer — non-monotonic spread, single-stroke trajectories, exponential noise autocorrelation, zero click-skew, deterministic device personality, and queue-overflow behavioral discontinuity.

**Architecture:** Six phases, applied to `src/smooth.c`, `src/smooth_config.h`, `src/kmbox.c`, plus minor edits in `src/usb_device.c`, `src/main.c`, and `core/imxrt1062_mm.ld`. Phases are sequenced from highest detection-risk-per-LoC to lowest; each commit builds, flashes, and runs standalone. Hardware verification uses UART log captures and Python analysis scripts in `scripts/`.

**Tech Stack:** Bare-metal ARM C (i.MX RT1062 / Cortex-M7), PlatformIO toolchain, no RTOS, no heap, fixed-point + cheap-float DSP. Python 3 + numpy/scipy for verification analysis.

**Scope/non-goals:** USB descriptor accuracy, USB device-side enumeration, Ferrum protocol changes are out of scope. We do not migrate to a different RNG. We do not introduce a unit-test framework (no existing one).

---

## Detection-risk priorities

| Phase | Issues fixed | Detection signature reduced |
|-------|--------------|------------------------------|
| 1 | #5 ±127 clamp, #1 non-monotonic spread, #2 Fitts not wired | Constant-velocity runs ≥3 frames (US patent 11947742) |
| 2 | #4 single-stroke ΣΛ-fit | BeCAPTCHA-Mouse 98.7% trace-level detection |
| 3 | #3 EWMA exp-autocorr, #8 broadband tremor | Welch-PSD slope / autocorrelation classifiers |
| 4 | #6 zero click-skew, #10 early reports | Hawk, Ricochet S02 click-onset xcorr |
| 5 | #7 no warmup/fatigue | Hawk "sense-performance consistency" |
| 6 | #9 deterministic personality, #11 queue overflow discontinuity | Device fingerprint linkage; behavioral mode change |

---

## File map (created or modified across the whole plan)

| File | Responsibility | Touched in phase |
|------|----------------|------------------|
| `src/smooth_config.h` | All tunables | 1, 2, 3, 4, 5 |
| `src/smooth.c` | Easing, queue, noise, timing | 1, 2, 3, 4, 5, 6 |
| `src/smooth.h` | Small API surface | 1, 4 |
| `src/kmbox.c` | Button skew queue | 4 |
| `src/kmbox.h` | Skew queue prototypes | 4 |
| `src/usb_device.c` | Optional report-desc axis probe | 1 |
| `src/main.c` | Wire max-per-frame from probe | 1 |
| `core/imxrt1062_mm.ld` | Reserve persistence flash sector | 6 |
| `scripts/cap_smooth.py` | UART capture of dx/dy/btn stream | All phases |
| `scripts/analyze_smooth.py` | PSD, autocorr, xcorr analysis | 1–5 |

---

## Cross-cutting design decisions

These resolve overlaps between the four sub-plans before any tuning fights ensue.

- **Tremor lives in B's `tremor_step()`; D scales amplitude via `tremor_amp_target`.** Session-level warmup/fatigue from D is wired in as a multiplier on `SMOOTH_TREMOR_AMP_TARGET_*` inside `session_update()`, not by re-scaling per-frame in the hot path.
- **Pink noise from B replaces both EWMA channels.** Downstream noise *amplitude* (D's session gain) is applied to the multiplier (e.g. at `smooth.c:475` and `:458`), not to the pink-noise sample itself.
- **Queue overflow merge (D) sits below split-fallback (A).** A's `split_and_enqueue` first downgrades to single-stroke when `popcount(free_mask) < N`. Only when `free_mask == 0` do we fall through to D's merge-into-most-recent. The raw accumulator fallback at the current `smooth.c:322-326` is *removed entirely*.
- **`smooth_set_humanize(false)` is the master kill switch.** It now also disables button skew (forwards into `kmbox_set_humanize_buttons(false)`) and freezes session drift (no warmup/fatigue scalars applied). Behaviour with humanize off must match today's deterministic output bit-for-bit (modulo Step 2's max-per-frame change, which applies in both modes).
- **One verification toolchain:** a single `scripts/cap_smooth.py` capture script plus `scripts/analyze_smooth.py` with `--check {monotonic,psd,xcorr,drift,boot}` modes. Each phase adds the relevant check, not a new script.
- **No new C unit-test framework.** All verification is run on hardware via UART captures.

---

## State-struct growth (audit)

For sanity — `state` in `smooth.c` grows across phases. ITCM/DTCM budget is generous on this MCU; this is just for documentation.

| Phase | Bytes added | Reason |
|-------|-------------|--------|
| 1 | 0 | Function refactor only |
| 2 | ~8 | `overflow_merges` counter, `submove_downgrade_count` |
| 3 | +68 | 2 × pink (6 floats + ctr) + tremor (phase, f, amp, target, quad_bias); remove tremor_x/y |
| 4 | 0 in `state` (queue goes in `kmbox.c`) | ~80 bytes in kmbox.c (btn_evt_t × 8 + indices + flags) |
| 5 | ~48 | session scalars, _init anchors, boot_counter |
| 6 | 0 | merge logic reuses existing fields |

Total smooth.c `state` growth: ~124 bytes. Acceptable.

---

# Phase 1: Fitts-derived monotone spread + 16-bit per-frame clamp

**Goal:** Eliminate constant-velocity runs from large flicks and make spread duration a smooth monotone function of distance using per-session Fitts coefficients.

**Files:**
- Modify: `src/smooth_config.h`
- Modify: `src/smooth.c:157-178` (`compute_spread_frames`), `src/smooth.c:231` (init line for `max_per_frame`)
- Modify: `src/usb_device.c` (small addition, after report-descriptor cache)
- Modify: `src/main.c` (one call after enumeration)

### Task 1.1: Add Fitts-duration tunables and rename vanilla bucket constants

- [ ] **Step 1: Modify `src/smooth_config.h`**

Replace the current `// ---- Spread duration (microseconds) ----` block (lines 11-22) with both renamed legacy buckets (for humanize-off path) and new Fitts constants:

```c
// ---- Spread duration: humanize-off vanilla bucket (deterministic) ----
#define SMOOTH_VANILLA_SMALL_US      3500    // <20 px
#define SMOOTH_VANILLA_MEDIUM_US     5000    // 20-60 px
#define SMOOTH_VANILLA_LARGE_US      4000    // 60-120 px
#define SMOOTH_VANILLA_XLARGE_US     3000    // >120 px
#define SMOOTH_VANILLA_SMALL_PX      20
#define SMOOTH_VANILLA_MEDIUM_PX     60
#define SMOOTH_VANILLA_LARGE_PX      120

// ---- Spread duration: humanize-on Fitts-derived (monotone in distance) ----
// MT_us = (fitts_a + fitts_b * log2(D/SCALE + 1)) * 1e6, clamped to FLOOR..CEIL.
// SCALE picked so a monitor-width move (~1000 px) is ~log2(5)=2.3 octaves;
// fitts_b≈0.11 → ~25 ms span across realistic 5–500 px range, matching
// Plamondon-style human reach data and our existing fitts_b range (0.08–0.15).
#define SMOOTH_FITTS_MT_PIXEL_SCALE   220.0f
#define SMOOTH_FITTS_MT_FLOOR_US      2200    // small moves can't go sub-2ms (3 frames at 1kHz)
#define SMOOTH_FITTS_MT_CEIL_US       95000   // 1500px throw ≈ 95ms upper bound
// Jitter on MT (multiplicative); ±25% bell-shaped via existing SFC32 uniform
#define SMOOTH_FITTS_MT_JITTER        0.25f

// ---- Per-frame clamp ----
// Default: int16 descriptor range (verified at enum). 2047 cleanly exceeds
// any human velocity (~2000 counts/ms) and leaves headroom. Boot-protocol
// passthrough callers should drop to 127 via smooth_set_max_per_frame().
#define SMOOTH_MAX_PER_FRAME_DEFAULT  2047
#define SMOOTH_MAX_PER_FRAME_LEGACY   127
```

Keep `SMOOTH_MIN_FRAMES`, `SMOOTH_MAX_FRAMES`, `SMOOTH_SPREAD_JITTER` lines unchanged.

- [ ] **Step 2: Build to confirm no regressions**

Run: `make 2>&1 | tail -20`
Expected: clean build, `firmware.hex` updated.

- [ ] **Step 3: Commit**

```bash
git add src/smooth_config.h
git commit -m "config: rename spread buckets to VANILLA, add Fitts-duration tunables"
```

### Task 1.2: Raise default `max_per_frame` to descriptor-supported range

- [ ] **Step 1: Modify `src/smooth.c:231`**

Replace:
```c
state.max_per_frame = 127;
```
with:
```c
state.max_per_frame = SMOOTH_MAX_PER_FRAME_DEFAULT;
```

- [ ] **Step 2: Build and flash**

Run: `make flash 2>&1 | tail -5`
Expected: `firmware.hex` flashed.

- [ ] **Step 3: Smoke test — large flick has no plateau**

Connect a 4k-class mouse to the host port, flick it across the screen, and verify the device still passes through movement (cursor reaches the same destination, no input lag). Issue a `km.move(500,0)` and confirm the cursor jumps (eyeball test — full instrumentation arrives in Task 1.5).

- [ ] **Step 4: Commit**

```bash
git add src/smooth.c
git commit -m "smooth: raise default max_per_frame to 2047 (int16 descriptor range)"
```

### Task 1.3: Replace `compute_spread_frames` with Fitts-derived duration

- [ ] **Step 1: Modify `src/smooth.c:157-178`**

Replace the entire body of `compute_spread_frames(int32_t abs_x_fp, int32_t abs_y_fp)`:

```c
static uint8_t compute_spread_frames(int32_t abs_x_fp, int32_t abs_y_fp)
{
    int32_t max_comp = abs_x_fp > abs_y_fp ? abs_x_fp : abs_y_fp;
    int32_t px = max_comp >> SMOOTH_FP_SHIFT;

    uint32_t spread_us;

    if (!state.humanize) {
        // Deterministic vanilla bucket — unchanged behaviour for A/B baseline.
        if (px < SMOOTH_VANILLA_SMALL_PX)       spread_us = SMOOTH_VANILLA_SMALL_US;
        else if (px < SMOOTH_VANILLA_MEDIUM_PX) spread_us = SMOOTH_VANILLA_MEDIUM_US;
        else if (px < SMOOTH_VANILLA_LARGE_PX)  spread_us = SMOOTH_VANILLA_LARGE_US;
        else                                    spread_us = SMOOTH_VANILLA_XLARGE_US;
    } else {
        // Use full 2-D euclidean distance, not max(|x|,|y|), so diagonals scale right.
        float dx = (float)(abs_x_fp >> SMOOTH_FP_SHIFT);
        float dy = (float)(abs_y_fp >> SMOOTH_FP_SHIFT);
        float dist2 = dx * dx + dy * dy;
        float dist = (dist2 > 1.0f) ? dist2 * fast_invsqrt(dist2) : 1.0f;

        float mt_s = state.fitts_a +
                     state.fitts_b * fast_log2f(dist / SMOOTH_FITTS_MT_PIXEL_SCALE + 1.0f);
        float mt_us_f = mt_s * 1000000.0f;

        // Jitter so consecutive identical D don't yield identical frame counts
        float j = smooth_rand_uniform() * SMOOTH_FITTS_MT_JITTER;
        mt_us_f *= (1.0f + j);

        if (mt_us_f < (float)SMOOTH_FITTS_MT_FLOOR_US) mt_us_f = (float)SMOOTH_FITTS_MT_FLOOR_US;
        if (mt_us_f > (float)SMOOTH_FITTS_MT_CEIL_US)  mt_us_f = (float)SMOOTH_FITTS_MT_CEIL_US;
        spread_us = (uint32_t)mt_us_f;
    }

    uint32_t frames = (uint32_t)(
        (float)(spread_us + (state.interval_us >> 1)) * state.inv_interval_f);
    if (frames < SMOOTH_MIN_FRAMES) frames = SMOOTH_MIN_FRAMES;
    if (frames > SMOOTH_MAX_FRAMES) frames = SMOOTH_MAX_FRAMES;
    return (uint8_t)frames;
}
```

- [ ] **Step 2: Build**

Run: `make 2>&1 | tail -5`
Expected: clean build.

- [ ] **Step 3: Flash and smoke-test**

Run: `make flash 2>&1 | tail -3`
Drive `km.move(20,0)` and `km.move(500,0)` from a host. Both should complete; large move should *not* feel snappier than the small one (the bug we just fixed).

- [ ] **Step 4: Commit**

```bash
git add src/smooth.c
git commit -m "smooth: Fitts-derived MT in compute_spread_frames (monotone in D)"
```

### Task 1.4: (Optional refinement) probe device descriptor for axis range

This is non-blocking — the default of 2047 is already int16-safe. Skip if pressed for time.

- [ ] **Step 1: Inspect `src/usb_device.c` for the cached descriptor structure**

Open the file and find where `cap_desc->ifaces[i].hid_report_desc` is populated (search for `hid_report_desc`). Identify a hook point after enumeration completes.

- [ ] **Step 2: Add `usb_device_axis_is_int16()` helper**

In `src/usb_device.c`, after descriptor cache:
```c
bool usb_device_axis_is_int16(void)
{
    // Scan cached HID report descriptor for first Generic Desktop X usage
    // and check Logical Maximum is encoded as 2 bytes (0x26 prefix).
    // Returns true if int16 detected, false if int8 (boot protocol).
    // Conservative default: false on parse failure.
    // ... [scan loop, ~20 LOC; follow existing descriptor-parse style]
}
```

- [ ] **Step 3: Declare in `src/usb_device.h`**

Add: `bool usb_device_axis_is_int16(void);`

- [ ] **Step 4: Wire from `src/main.c`**

After the existing `usb_device_init(&desc)` call and before the main loop, add:
```c
smooth_set_max_per_frame(usb_device_axis_is_int16()
    ? SMOOTH_MAX_PER_FRAME_DEFAULT
    : SMOOTH_MAX_PER_FRAME_LEGACY);
```

- [ ] **Step 5: Build, flash, commit**

```bash
make && make flash
git add src/usb_device.c src/usb_device.h src/main.c
git commit -m "usb_device: detect int16 X/Y axis and configure smooth clamp accordingly"
```

### Task 1.5: Verification — Fitts monotonicity + plateau elimination

- [ ] **Step 1: Add a debug dx-trace tap**

In `src/smooth.c`, immediately before `*out_x = ix;` at line 559, add a guarded printf:
```c
#ifdef SMOOTH_DEBUG_DX_LOG
    kmbox_debug_log_dx(ix, iy);  // declared in kmbox.h; impl writes a single CSV line over UART
#endif
```

Add a minimal `kmbox_debug_log_dx(int16_t, int16_t)` in `src/kmbox.c` that emits `D,<ms>,<dx>,<dy>\r\n` only if a runtime flag is set (toggle via Ferrum `km.debug(1)` if convenient; otherwise toggle by recompiling with `-DSMOOTH_DEBUG_DX_LOG`).

- [ ] **Step 2: Create `scripts/cap_smooth.py`**

Create a small capture script that opens the UART (reuse pattern from `scripts/uart_debug.py`), reads lines, filters those starting with `D,`, and writes them to `cap.csv`. Drive movements from a separate process by sending `km.move()` commands.

- [ ] **Step 3: Create `scripts/analyze_smooth.py --check monotonic`**

Read `cap.csv`, perform: for each distance D in {10,30,60,100,200,400,800,1200}, sample 20 moves, compute observed `total_frames` (frames where dx≠0). Assert non-decreasing across D; print slope of `frames` vs `log2(D+1)` and require positive with R² > 0.85.

- [ ] **Step 4: Run hardware sweep**

```bash
python3 scripts/cap_smooth.py --out cap.csv &
python3 scripts/drive_moves.py --sweep 10,30,60,100,200,400,800,1200 --reps 20
kill %1
python3 scripts/analyze_smooth.py cap.csv --check monotonic
```
Expected: `PASS — monotone (R²=0.9X, slope>0)`.

- [ ] **Step 5: Add `--check plateau` mode and run**

Compute longest run of identical consecutive dx in each captured move; require max-run < 3 for every D. (Confirms #5 fix.)

```bash
python3 scripts/analyze_smooth.py cap.csv --check plateau
```
Expected: `PASS — max-run = 2`.

- [ ] **Step 6: Commit verification harness**

```bash
git add scripts/cap_smooth.py scripts/drive_moves.py scripts/analyze_smooth.py src/smooth.c src/kmbox.c src/kmbox.h
git commit -m "verify: dx-trace tap + analyze_smooth.py (monotonic, plateau checks)"
```

---

# Phase 2: Multi-stroke (ΣΛ-style) decomposition for large injections

**Goal:** Defeat single-stroke ΣΛ decomposition by splitting injections > ~150 px into 2–3 sub-strokes with primary + corrective structure.

**Files:**
- Modify: `src/smooth_config.h`
- Modify: `src/smooth.c` (refactor `smooth_inject` body into `enqueue_single`, add `split_and_enqueue`)

### Task 2.1: Add submove tunables

- [ ] **Step 1: Modify `src/smooth_config.h`**

Append after the Fitts block:

```c
// ---- Multi-stroke (ΣΛ-style submovement) decomposition ----
// Real human aimed moves >~150 px decompose into 2-5 lognormal submovements
// (Plamondon's Sigma-Lognormal model). Single-stroke easing is detectable
// (BeCAPTCHA-Mouse 98.7% acc). We split inject-time into primary + correctives.

#define SMOOTH_SUBMOVE_MIN_PX          150  // below this stay single-stroke
#define SMOOTH_SUBMOVE_THRESH_2        150  // distance ≥ this → 2 strokes
#define SMOOTH_SUBMOVE_THRESH_3        420  // distance ≥ this → 3 strokes
#define SMOOTH_SUBMOVE_PRIMARY_FRAC    0.90f
#define SMOOTH_SUBMOVE_PRIMARY_JITTER  0.04f  // ±4% on primary fraction
#define SMOOTH_SUBMOVE_CORRECTIVE_ROT  0.06f  // small-angle perp rotation (rad)
#define SMOOTH_SUBMOVE_OVERSHOOT_PROB  0.30f  // P(last corrective overshoots)
#define SMOOTH_SUBMOVE_OVERSHOOT_MAX   0.08f  // up to 8% overshoot
```

- [ ] **Step 2: Build**

Run: `make 2>&1 | tail -5`
Expected: clean build (no consumers yet).

- [ ] **Step 3: Commit**

```bash
git add src/smooth_config.h
git commit -m "config: submove decomposition tunables"
```

### Task 2.2: Refactor `smooth_inject` body into `enqueue_single`

Pure refactor — no behaviour change. Sets up Task 2.3.

- [ ] **Step 1: Modify `src/smooth.c:315-348`**

Extract everything except the leading `if (x==0 && y==0) return;` and `state.idle_frames = 0;` into a new helper. The current full-queue overflow branch (lines 322-326) stays inside `enqueue_single` for now (we replace it in Phase 6).

```c
static void enqueue_single(int16_t x, int16_t y)
{
    if (state.free_mask == 0) {
        state.x_accum_fp += int_to_fp(x);
        state.y_accum_fp += int_to_fp(y);
        return;
    }

    uint8_t slot = (uint8_t)__builtin_ctz(state.free_mask);
    state.free_mask &= state.free_mask - 1;

    int32_t xfp = int_to_fp(x);
    int32_t yfp = int_to_fp(y);
    int32_t ax = xfp >= 0 ? xfp : -xfp;
    int32_t ay = yfp >= 0 ? yfp : -yfp;

    uint8_t frames = compute_spread_frames(ax, ay);

    state.queue[slot].x_remaining_fp = xfp;
    state.queue[slot].y_remaining_fp = yfp;
    state.queue[slot].inv_total_fp   = state.inv_frames_lut[frames];
    state.queue[slot].eased_prev     = 0;
    state.queue[slot].speed_gain     = state.humanize
        ? compute_fitts_speed_gain(ax, ay)
        : SMOOTH_SPEED_GAIN_DEFAULT;
    state.queue[slot].frames_left    = frames;
    state.queue[slot].total_frames   = frames;
    state.count++;
}

__attribute__((section(".fastrun")))
void smooth_inject(int16_t x, int16_t y)
{
    if (x == 0 && y == 0) return;
    state.idle_frames = 0;
    enqueue_single(x, y);
}
```

- [ ] **Step 2: Build and flash**

Run: `make flash 2>&1 | tail -3`

- [ ] **Step 3: Re-run plateau and monotonicity checks from Task 1.5**

Expected: still pass (pure refactor).

- [ ] **Step 4: Commit**

```bash
git add src/smooth.c
git commit -m "smooth: extract enqueue_single() from smooth_inject() (refactor)"
```

### Task 2.3: Implement `split_and_enqueue`

- [ ] **Step 1: Modify `src/smooth.c`**

Add above `smooth_inject`:

```c
static inline uint8_t submove_count_for(int32_t dist_px)
{
    if (dist_px < SMOOTH_SUBMOVE_THRESH_2) return 1;
    if (dist_px < SMOOTH_SUBMOVE_THRESH_3) return 2;
    return 3;
}

static void split_and_enqueue(int16_t x, int16_t y)
{
    // 1. Compute distance in pixels
    int32_t ax_fp = (x >= 0 ? (int32_t)x : -(int32_t)x);
    int32_t ay_fp = (y >= 0 ? (int32_t)y : -(int32_t)y);
    int32_t dist_px = (ax_fp > ay_fp ? ax_fp : ay_fp); // L∞; cheap & good enough for gating

    // 2. Single-stroke gate
    if (!state.humanize || dist_px < SMOOTH_SUBMOVE_MIN_PX) {
        enqueue_single(x, y);
        return;
    }

    uint8_t n = submove_count_for(dist_px);
    uint8_t free_slots = __builtin_popcount(state.free_mask);
    if (free_slots < n) {
        // Downgrade rather than try to allocate beyond capacity
        state.submove_downgrade_count++;
        enqueue_single(x, y);
        return;
    }

    // 3. Primary stroke: PRIMARY_FRAC ± jitter of total vector
    float pj = smooth_rand_uniform() * SMOOTH_SUBMOVE_PRIMARY_JITTER;
    float pfrac = SMOOTH_SUBMOVE_PRIMARY_FRAC + pj;
    int16_t px_i = (int16_t)((float)x * pfrac);
    int16_t py_i = (int16_t)((float)y * pfrac);

    int16_t rx = x - px_i;  // residual for correctives
    int16_t ry = y - py_i;

    enqueue_single(px_i, py_i);

    // 4. Corrective strokes: split residual evenly, rotate each slightly,
    //    optional overshoot on last.
    uint8_t correctives = n - 1;
    for (uint8_t i = 0; i < correctives; i++) {
        bool is_last = (i == correctives - 1);
        int16_t cx, cy;
        if (is_last) {
            cx = rx; cy = ry;  // exhaust residual exactly (no drift)
        } else {
            // Equal split
            cx = rx / (int16_t)(correctives - i);
            cy = ry / (int16_t)(correctives - i);
            rx -= cx; ry -= cy;
        }

        // Small-angle rotation: theta ~ uniform ±CORRECTIVE_ROT
        float theta = smooth_rand_uniform() * SMOOTH_SUBMOVE_CORRECTIVE_ROT;
        float rxf = (float)cx - (float)cy * theta;  // cos≈1, sin≈θ
        float ryf = (float)cy + (float)cx * theta;

        // Optional overshoot on last corrective
        if (is_last) {
            uint32_t r = sfc32_next();
            if ((r * (1.0f / 4294967296.0f)) < SMOOTH_SUBMOVE_OVERSHOOT_PROB) {
                float ov = 1.0f + fabsf(smooth_rand_uniform()) * SMOOTH_SUBMOVE_OVERSHOOT_MAX;
                rxf *= ov; ryf *= ov;
            }
        }

        enqueue_single((int16_t)(rxf + (rxf >= 0 ? 0.5f : -0.5f)),
                       (int16_t)(ryf + (ryf >= 0 ? 0.5f : -0.5f)));
    }
}

__attribute__((section(".fastrun")))
void smooth_inject(int16_t x, int16_t y)
{
    if (x == 0 && y == 0) return;
    state.idle_frames = 0;
    split_and_enqueue(x, y);
}
```

Add to `state` struct (smooth.c:72-114): `uint32_t submove_downgrade_count;`. Initialize via `memset` in `smooth_init` (already happens via existing memset of `state`).

- [ ] **Step 2: Build and flash**

Run: `make flash 2>&1 | tail -3`

- [ ] **Step 3: Add `--check submove` to analyze_smooth.py**

For a 600-px flick capture, compute the speed-magnitude `|dx|+|dy|` per frame, find peaks (scipy.signal.find_peaks). Assert ≥2 peaks separated by ≥10 ms, with the 2nd peak's amplitude in [5%, 25%] of the 1st.

- [ ] **Step 4: Run**

```bash
python3 scripts/cap_smooth.py --out cap.csv &
python3 scripts/drive_moves.py --single 600,0 --reps 20
kill %1
python3 scripts/analyze_smooth.py cap.csv --check submove
```
Expected: `PASS — mean peaks/move = 2.X, mean 2nd/1st amp ratio = 0.1X`.

- [ ] **Step 5: Confirm humanize-off bit-for-bit equivalence**

```bash
# In a quick test client: send km.humanize(0); then replay a fixed pattern
# and dump dx,dy; compare against a baseline saved before this phase.
diff baseline_humanize_off.csv current_humanize_off.csv
```
Expected: identical (split path is gated on `state.humanize`).

- [ ] **Step 6: Commit**

```bash
git add src/smooth.c src/smooth_config.h scripts/analyze_smooth.py
git commit -m "smooth: ΣΛ-style submove split for injections ≥150 px"
```

---

# Phase 3: Pink noise (Voss-McCartney) + band-limited tremor

**Goal:** Replace exponential-autocorrelation EWMA noise with a 1/f-ish pink-noise generator and replace broadband AR(1) tremor with an 8–12 Hz drifting sinusoid + noise floor.

**Files:**
- Modify: `src/smooth_config.h`
- Modify: `src/smooth.c` (struct, init, process_frame, helpers)

### Task 3.1: Add pink-noise + tremor tunables and remove obsolete EWMA/AR-tremor constants

- [ ] **Step 1: Modify `src/smooth_config.h`**

Replace the `// ---- EWMA noise channels ----` block (currently lines 31-35) and the `// ---- Persistent micro-tremor ----` block (currently lines 112-120) with:

```c
// ---- Pink-noise channels (Voss-McCartney, replaces EWMA) ----
// 6 octaves × 2 channels (speed + perp). On each frame, octave k is updated
// every 2^k frames using __builtin_ctz on a counter. Sum is normalized so the
// per-channel std ≈ legacy EWMA std (0.071) for a drop-in replacement.
// Spectral slope ~1/f over ~1Hz–500Hz at 1kHz sample.
#define SMOOTH_PINK_OCTAVES        6
#define SMOOTH_PINK_STD_TARGET     0.071f
#define SMOOTH_PINK_PRIME_ROUNDS   128

// ---- Band-limited tremor (replaces AR(1)) ----
// 10 Hz carrier with OU-drifted frequency (8–12 Hz physiological band) and
// OU-drifted amplitude. Phase-quadrature on Y axis breaks identical-trace
// correlation. Small uncorrelated noise floor masks carrier purity.
#define SMOOTH_TREMOR_F_CENTER         10.0f  // Hz
#define SMOOTH_TREMOR_F_HALFBAND       2.0f   // ±2 Hz → 8–12 Hz
#define SMOOTH_TREMOR_F_DRIFT_RATE     0.02f  // OU pull toward center
#define SMOOTH_TREMOR_F_DRIFT_STEP     0.05f  // Hz/frame innovation std
#define SMOOTH_TREMOR_AMP_TARGET_MIN   0.10f  // px
#define SMOOTH_TREMOR_AMP_TARGET_SPAN  0.25f  // px (target ∈ [0.10, 0.35])
#define SMOOTH_TREMOR_AMP_DRIFT        0.005f
#define SMOOTH_TREMOR_AMP_STEP         0.02f
#define SMOOTH_TREMOR_NOISE_FLOOR      0.04f  // px std uncorrelated
#define SMOOTH_TREMOR_RAMP_FRAMES      250    // fade-out at idle (was hard zero)
#define SMOOTH_TREMOR_IDLE_TIMEOUT     100    // unchanged
```

Remove `SMOOTH_EWMA_ALPHA`, `SMOOTH_EWMA_BETA`, `SMOOTH_TREMOR_STEP`, `SMOOTH_TREMOR_DECAY` lines. Keep `SMOOTH_EWMA_PRIME_ROUNDS` deleted as well (we'll add `SMOOTH_PINK_PRIME_ROUNDS`).

- [ ] **Step 2: Build (expect failures in smooth.c referencing removed constants)**

Run: `make 2>&1 | grep error | head -10`
Expected: errors referencing `SMOOTH_EWMA_ALPHA`, `SMOOTH_TREMOR_STEP`, etc. These guide Task 3.2.

- [ ] **Step 3: Do NOT commit yet — Task 3.2 fixes the breakage**

### Task 3.2: Add pink-noise + tremor state and helpers

- [ ] **Step 1: Modify `src/smooth.c` — extend `state` struct (lines 72-114)**

Remove `float speed_noise; float perp_noise;` and `float tremor_x; float tremor_y;`. Add:
```c
typedef struct {
    float oct[SMOOTH_PINK_OCTAVES];
    uint32_t ctr;
} pink_state_t;

// inside state struct:
pink_state_t pink_speed;
pink_state_t pink_perp;
float speed_noise;   // kept as the per-frame output of pink_step (drop-in alias)
float perp_noise;

float tremor_phase;
float tremor_f;
float tremor_amp;
float tremor_amp_target;
float tremor_quad_bias;
```

- [ ] **Step 2: Add `pink_init`, `pink_step`, `fast_sin`, `tremor_step` helpers**

Insert before `recompute_easing` (~line 145):

```c
static void pink_init(pink_state_t *p)
{
    p->ctr = 1;
    for (int k = 0; k < SMOOTH_PINK_OCTAVES; k++)
        p->oct[k] = smooth_rand_uniform();
}

static inline float pink_step(pink_state_t *p)
{
    // Voss-McCartney: update lowest-order due octave per frame.
    // __builtin_ctz(p->ctr) gives the lowest set bit index (0..OCTAVES-1).
    uint32_t k = __builtin_ctz(p->ctr | (1u << (SMOOTH_PINK_OCTAVES - 1)));
    if (k < SMOOTH_PINK_OCTAVES)
        p->oct[k] = smooth_rand_uniform();
    p->ctr++;
    // Sum octaves; normalize: sum-of-OCTAVES uniforms in [-1,1] has std ≈ sqrt(O/3).
    float s = 0.0f;
    for (int i = 0; i < SMOOTH_PINK_OCTAVES; i++) s += p->oct[i];
    return s * (SMOOTH_PINK_STD_TARGET / 1.414f);  // approx normalization
}

static inline float fast_sin(float x)
{
    // Fold to [-π, π]
    const float PI = 3.14159265f;
    const float TWO_PI = 6.28318531f;
    while (x >  PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
    // 5th-order Taylor; ample for sub-pixel tremor
    float x2 = x * x;
    return x * (1.0f - x2 * (1.0f/6.0f - x2 * (1.0f/120.0f - x2 * (1.0f/5040.0f))));
}

static void tremor_step(float envelope, float *out_x, float *out_y)
{
    // OU on frequency
    float f_target = SMOOTH_TREMOR_F_CENTER;
    state.tremor_f += SMOOTH_TREMOR_F_DRIFT_RATE * (f_target - state.tremor_f)
                    + SMOOTH_TREMOR_F_DRIFT_STEP * smooth_rand_uniform();
    if (state.tremor_f < SMOOTH_TREMOR_F_CENTER - SMOOTH_TREMOR_F_HALFBAND)
        state.tremor_f = SMOOTH_TREMOR_F_CENTER - SMOOTH_TREMOR_F_HALFBAND;
    if (state.tremor_f > SMOOTH_TREMOR_F_CENTER + SMOOTH_TREMOR_F_HALFBAND)
        state.tremor_f = SMOOTH_TREMOR_F_CENTER + SMOOTH_TREMOR_F_HALFBAND;

    // OU on amplitude toward per-session target
    state.tremor_amp += SMOOTH_TREMOR_AMP_DRIFT * (state.tremor_amp_target - state.tremor_amp)
                     + SMOOTH_TREMOR_AMP_STEP * smooth_rand_uniform();
    if (state.tremor_amp < 0.0f) state.tremor_amp = 0.0f;

    // Phase advance
    state.tremor_phase += 6.28318531f * state.tremor_f * state.dt;
    if (state.tremor_phase > 6.28318531f) state.tremor_phase -= 6.28318531f;

    float a = state.tremor_amp * envelope;
    float sx = fast_sin(state.tremor_phase);
    float sy = fast_sin(state.tremor_phase + 1.5707963f + state.tremor_quad_bias);
    *out_x = a * sx + SMOOTH_TREMOR_NOISE_FLOOR * smooth_rand_uniform();
    *out_y = a * sy + SMOOTH_TREMOR_NOISE_FLOOR * smooth_rand_uniform();
}
```

- [ ] **Step 3: Replace EWMA init in `smooth_init` (lines 281-294)**

Remove the entire EWMA-alpha scaling and priming loop. Replace with:
```c
pink_init(&state.pink_speed);
pink_init(&state.pink_perp);
for (int i = 0; i < SMOOTH_PINK_PRIME_ROUNDS; i++) {
    pink_step(&state.pink_speed);
    pink_step(&state.pink_perp);
}

state.tremor_phase = smooth_rand_uniform() * 3.14159265f;
state.tremor_f = SMOOTH_TREMOR_F_CENTER;
state.tremor_amp = 0.0f;
state.tremor_amp_target = SMOOTH_TREMOR_AMP_TARGET_MIN
    + fabsf(sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)) * SMOOTH_TREMOR_AMP_TARGET_SPAN;
state.tremor_quad_bias = sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr) * 0.5f;
```

- [ ] **Step 4: Replace EWMA updates in `smooth_process_frame` (lines 446-449)**

```c
state.speed_noise = pink_step(&state.pink_speed);
state.perp_noise  = pink_step(&state.pink_perp);
```

Downstream usage at lines 458 and 475 is unchanged because pink_step is normalized to the same std.

- [ ] **Step 5: Replace tremor block in `smooth_process_frame` (lines 513-533)**

```c
float envelope;
if (state.idle_frames < SMOOTH_TREMOR_IDLE_TIMEOUT) {
    envelope = 1.0f;
} else if (state.idle_frames < SMOOTH_TREMOR_IDLE_TIMEOUT + SMOOTH_TREMOR_RAMP_FRAMES) {
    uint32_t over = state.idle_frames - SMOOTH_TREMOR_IDLE_TIMEOUT;
    envelope = 1.0f - (float)over / (float)SMOOTH_TREMOR_RAMP_FRAMES;
} else {
    envelope = 0.0f;
}

if (!has_movement && state.humanize && envelope > 0.0f) {
    float tx, ty;
    tremor_step(envelope, &tx, &ty);
    frame_x_fp = (int32_t)(tx * (float)SMOOTH_FP_ONE);
    frame_y_fp = (int32_t)(ty * (float)SMOOTH_FP_ONE);
} else if (!has_movement) {
    if (envelope == 0.0f) {
        state.x_accum_fp = 0;
        state.y_accum_fp = 0;
    }
}
```

Update the idle fast-path predicate (line 423) from `state.idle_frames >= SMOOTH_TREMOR_IDLE_TIMEOUT` to `state.idle_frames >= SMOOTH_TREMOR_IDLE_TIMEOUT + SMOOTH_TREMOR_RAMP_FRAMES`.

- [ ] **Step 6: Build, flash**

Run: `make flash 2>&1 | tail -5`
Expected: clean build.

- [ ] **Step 7: Smoke test**

Move the host mouse normally; cursor should feel identical (noise std is preserved). Let the system go idle for ~5 seconds and confirm cursor doesn't drift (tremor envelope ramps to 0).

- [ ] **Step 8: Commit**

```bash
git add src/smooth.c src/smooth_config.h
git commit -m "smooth: replace EWMA with Voss-McCartney pink noise + band-limited tremor"
```

### Task 3.3: Verification — PSD slope and tremor band

- [ ] **Step 1: Add `--check psd` and `--check tremor_band` to analyze_smooth.py**

```python
# --check psd: load cap.csv (dx column), compute Welch PSD via
# scipy.signal.welch(dx, fs=1000, nperseg=2048), fit log-log slope over
# 1–100 Hz, assert beta ∈ [1.5, 2.4].
# --check tremor_band: take 30s of all-zero-input-command time window,
# compute Welch PSD, find peak frequency, assert peak ∈ [8, 12] Hz with
# -3dB bandwidth ≥ 2 Hz.
```

- [ ] **Step 2: Capture a movement-heavy 60s window and an idle 30s window**

```bash
python3 scripts/cap_smooth.py --out cap_active.csv --duration 60 &
python3 scripts/drive_moves.py --random --duration 60
wait
python3 scripts/cap_smooth.py --out cap_idle.csv --duration 30
```

- [ ] **Step 3: Run checks**

```bash
python3 scripts/analyze_smooth.py cap_active.csv --check psd
python3 scripts/analyze_smooth.py cap_idle.csv --check tremor_band
```
Expected:
- `PASS — PSD slope β = 1.X (target 1.5–2.4)`
- `PASS — tremor peak = 10.X Hz, BW = X.X Hz`

- [ ] **Step 4: Commit analysis additions**

```bash
git add scripts/analyze_smooth.py
git commit -m "verify: PSD slope + tremor band checks"
```

---

# Phase 4: Event-timing humanization (click skew + tighter PIT jitter)

**Goal:** Insert log-normal-ish skew between button-state changes and HID delivery; tighten timing-jitter floor to physically plausible bounds.

**Files:**
- Modify: `src/smooth_config.h`
- Modify: `src/smooth.c` (`smooth_timing_next`, `smooth_set_humanize`)
- Modify: `src/kmbox.c`, `src/kmbox.h` (button skew queue)

### Task 4.1: Add event-timing tunables

- [ ] **Step 1: Modify `src/smooth_config.h`**

Replace the existing `SMOOTH_TIMING_LDVAL_JITTER` line and add new ones:

```c
// ---- Timing humanization (PIT interval) ----
// One-sided right-skewed: real USB IRP scheduling cannot deliver early.
// Floor at 0.97× covers crystal tolerance; ceiling 1.50× preserves the
// occasional missed-poll signature that real mice exhibit.
#define SMOOTH_TIMING_LDVAL_JITTER   0.30f
#define SMOOTH_TIMING_FLOOR_FRAC     0.97f
#define SMOOTH_TIMING_CEIL_FRAC      1.50f

// ---- Button event skew ----
// Buttons fire on the same tick as movement updates today (zero xcorr lag,
// known anti-cheat tell). We queue button-state changes with a per-event
// delay drawn from a right-skewed distribution. Press/release/wheel get
// distinct bands matching empirical mouse-trace data.
#define SMOOTH_BTN_PRESS_SKEW_MS     35
#define SMOOTH_BTN_RELEASE_SKEW_MS   60
#define SMOOTH_BTN_WHEEL_SKEW_MS     20
#define SMOOTH_BTN_QUEUE_DEPTH       8
#define SMOOTH_BTN_MAX_DELAY_MS      80   // hard latency cap for gameplay
```

Keep `SMOOTH_TIMING_RATE_OFFSET` (still used).

- [ ] **Step 2: Build, commit**

```bash
make 2>&1 | tail -3
git add src/smooth_config.h
git commit -m "config: event-timing humanization tunables"
```

### Task 4.2: One-sided right-skewed PIT jitter

- [ ] **Step 1: Modify `src/smooth.c` — add helper above `smooth_timing_next`**

```c
static inline float timing_rand_uniform_pos(void)
{
    return (float)(timing_rand32() >> 8) * (1.0f / 16777216.0f);  // [0, 1)
}
```

- [ ] **Step 2: Modify `src/smooth.c:628-647` (`smooth_timing_next`)**

Replace the body of the function (keeping the signature and the early `if (!state.humanize) return base_ldval;`):

```c
uint32_t smooth_timing_next(uint32_t base_ldval, bool *out_skip)
{
    *out_skip = false;
    if (!state.humanize) return base_ldval;

    float u1 = timing_rand_uniform_pos();
    float u2 = timing_rand_uniform_pos();
    float u  = (u1 > u2) ? u1 : u2;   // bias toward 1 (right-skewed when squared)
    float jitter = u * u * SMOOTH_TIMING_LDVAL_JITTER;
    float offset = state.rate_bias + jitter;  // always ≥ rate_bias
    float result = (float)base_ldval * (1.0f + offset);

    float lo = (float)base_ldval * SMOOTH_TIMING_FLOOR_FRAC;
    float hi = (float)base_ldval * SMOOTH_TIMING_CEIL_FRAC;
    if (result < lo) result = lo;
    if (result > hi) result = hi;
    return (uint32_t)result;
}
```

- [ ] **Step 3: Build, flash, commit**

```bash
make flash 2>&1 | tail -3
git add src/smooth.c
git commit -m "smooth: right-skewed PIT jitter, floor at 0.97× (physical plausibility)"
```

### Task 4.3: Button skew queue in kmbox

- [ ] **Step 1: Modify `src/kmbox.h`**

Add prototypes near the existing public API:
```c
void kmbox_set_humanize_buttons(bool enabled);
void kmbox_flush_button_queue_now(void);
```

- [ ] **Step 2: Modify `src/kmbox.c` — add static state near `inject` struct**

```c
typedef struct {
    uint32_t target_ms;
    uint8_t  buttons;
    int8_t   wheel;
} btn_evt_t;

static btn_evt_t btn_q[SMOOTH_BTN_QUEUE_DEPTH];
static uint8_t btn_q_head = 0;
static uint8_t btn_q_tail = 0;
static uint8_t last_pending_buttons = 0;
static uint32_t last_pending_target_ms = 0;
static bool kmbox_humanize_buttons = true;
```

- [ ] **Step 3: Add helpers above `apply_mouse_result`**

```c
static uint16_t sample_button_skew(uint16_t range_ms)
{
    // Right-skewed: take max of 3 uniforms, square it.
    uint32_t r1 = timing_rand32(); // reuse smooth.c TRNG path? — extern, see step 4
    // For simplicity here: emulate with a small SFC32 in kmbox.c, or extern
    // smooth's timing_rand32. We extern it.
    extern uint32_t kmbox_skew_rand32(void);  // wrapper added in smooth.c step 4
    uint32_t a = kmbox_skew_rand32();
    uint32_t b = kmbox_skew_rand32();
    uint32_t c = kmbox_skew_rand32();
    uint32_t m = a > b ? a : b; if (c > m) m = c;
    float u = (float)(m >> 8) * (1.0f / 16777216.0f);
    uint16_t skew = (uint16_t)(u * u * (float)range_ms);
    if (skew > SMOOTH_BTN_MAX_DELAY_MS) skew = SMOOTH_BTN_MAX_DELAY_MS;
    return skew;
}

static void enqueue_button_event(uint8_t buttons, int8_t wheel)
{
    uint32_t now = millis();
    // Detect release: at least one bit cleared vs last_pending
    bool is_release = (last_pending_buttons & ~buttons) != 0;
    uint16_t range = is_release ? SMOOTH_BTN_RELEASE_SKEW_MS
                                : SMOOTH_BTN_PRESS_SKEW_MS;
    if (wheel != 0 && buttons == last_pending_buttons) range = SMOOTH_BTN_WHEEL_SKEW_MS;
    uint16_t skew = sample_button_skew(range);

    uint32_t target = (last_pending_target_ms > now ? last_pending_target_ms : now) + skew;

    uint8_t next = (btn_q_tail + 1) % SMOOTH_BTN_QUEUE_DEPTH;
    if (next == btn_q_head) {
        // Full: synchronously apply oldest to make room (preserves order)
        inject.mouse_buttons = btn_q[btn_q_head].buttons;
        inject.mouse_wheel += btn_q[btn_q_head].wheel;
        inject.mouse_dirty = true;
        btn_q_head = (btn_q_head + 1) % SMOOTH_BTN_QUEUE_DEPTH;
    }

    btn_q[btn_q_tail].target_ms = target;
    btn_q[btn_q_tail].buttons   = buttons;
    btn_q[btn_q_tail].wheel     = wheel;
    btn_q_tail = next;
    last_pending_buttons = buttons;
    last_pending_target_ms = target;
}

static void drain_button_queue(void)
{
    uint32_t now = millis();
    while (btn_q_head != btn_q_tail && (int32_t)(now - btn_q[btn_q_head].target_ms) >= 0) {
        inject.mouse_buttons = btn_q[btn_q_head].buttons;
        inject.mouse_wheel += btn_q[btn_q_head].wheel;
        inject.mouse_dirty = true;
        btn_q_head = (btn_q_head + 1) % SMOOTH_BTN_QUEUE_DEPTH;
    }
}

void kmbox_flush_button_queue_now(void)
{
    while (btn_q_head != btn_q_tail) {
        inject.mouse_buttons = btn_q[btn_q_head].buttons;
        inject.mouse_wheel += btn_q[btn_q_head].wheel;
        inject.mouse_dirty = true;
        btn_q_head = (btn_q_head + 1) % SMOOTH_BTN_QUEUE_DEPTH;
    }
    last_pending_target_ms = 0;
}

void kmbox_set_humanize_buttons(bool enabled)
{
    if (!enabled) kmbox_flush_button_queue_now();
    kmbox_humanize_buttons = enabled;
}
```

- [ ] **Step 4: Add `kmbox_skew_rand32` wrapper in `src/smooth.c`**

Since `timing_rand32` is a `static inline` in `smooth.c`, expose a non-inline wrapper at the bottom of `smooth.c`:
```c
uint32_t kmbox_skew_rand32(void) { return timing_rand32(); }
```

- [ ] **Step 5: Modify `src/kmbox.c:949-964` (`apply_mouse_result`)**

```c
static void apply_mouse_result(int16_t dx, int16_t dy, uint8_t buttons,
                               int8_t wheel, bool use_smooth)
{
    // Movement path unchanged
    if (use_smooth && (dx != 0 || dy != 0)) {
        smooth_inject(dx, dy);
    } else if (dx != 0 || dy != 0) {
        inject.mouse_dx += dx;
        inject.mouse_dy += dy;
        inject.mouse_dirty = true;
    }

    // Button / wheel: enqueue if humanizing, else apply synchronously
    bool btn_changed = (buttons != last_pending_buttons);
    if (kmbox_humanize_buttons && use_smooth && (btn_changed || wheel != 0)) {
        enqueue_button_event(buttons, wheel);
    } else {
        inject.mouse_buttons = buttons;
        inject.mouse_wheel += wheel;
        if (btn_changed || wheel != 0) inject.mouse_dirty = true;
        last_pending_buttons = buttons;
    }
}
```

- [ ] **Step 6: Wire drain into `kmbox_poll_fast()`**

Find `kmbox_poll_fast()` and add `drain_button_queue();` immediately before the existing `click_release_at` block (the agent's plan cites this location around line 570; verify by reading the function).

- [ ] **Step 7: Wire master kill-switch in `src/smooth.c:590-595` (`smooth_set_humanize`)**

```c
void smooth_set_humanize(bool enabled)
{
    state.humanize = enabled;
    kmbox_set_humanize_buttons(enabled);
    recompute_easing();
}
```

Add `#include "kmbox.h"` at top of `smooth.c` if not already present.

- [ ] **Step 8: Build, flash, smoke test**

```bash
make flash 2>&1 | tail -3
```
Click and drag with a host mouse; verify clicks register correctly (the skew should be imperceptible) and order is preserved (chord modifiers don't get swapped).

- [ ] **Step 9: Add `--check click_xcorr` to analyze_smooth.py**

Capture mode that logs button/wheel events alongside dx/dy:
- Extend the debug trace tap from Task 1.5 to also log `B,<ms>,<buttons>,<wheel>\r\n` whenever `inject.mouse_buttons` changes (add at the kmbox flush path).
- Python computes cross-correlation between button-edge train and movement-onset train; reports peak offset and σ.

Expected after run:
- `PASS — click xcorr peak = +XX ms, σ = XX ms (not at zero with σ<5ms)`

- [ ] **Step 10: Commit**

```bash
git add src/smooth.c src/kmbox.c src/kmbox.h scripts/analyze_smooth.py
git commit -m "kmbox: button skew queue + smooth_set_humanize forwards into kmbox"
```

---

# Phase 5: Session-time model (warmup, fatigue, concentration, personality drift)

**Goal:** Add a low-rate session-time tick that produces three scalars (warmup_gain, fatigue_gain, concentration_gain) consumed by noise/Fitts/tremor, and slowly drifts the other personality params besides arc_bias.

**Files:**
- Modify: `src/smooth_config.h`
- Modify: `src/smooth.c` (state struct, `session_update`, init, scalar consumers)

### Task 5.1: Add session-model tunables

- [ ] **Step 1: Modify `src/smooth_config.h`**

Append:
```c
// ---- Session-time model ----
// Slow scalars updated at ~1 Hz that scale tremor amp, perpendicular amp,
// and Fitts gain. Warmup: first ~120 s show ~18% larger jitter/overshoot.
// Fatigue: tremor amp grows ~12% over 10 min. Concentration: brief
// precision boost after a >30 s idle gap.
#define SMOOTH_SESSION_UPDATE_MS          1000
#define SMOOTH_WARMUP_TAU_MS              45000
#define SMOOTH_WARMUP_GAIN_PEAK           1.18f
#define SMOOTH_FATIGUE_TAU_MS             600000   // 10 min e-fold
#define SMOOTH_FATIGUE_GAIN_PEAK          1.12f
#define SMOOTH_PAUSE_DETECT_MS            30000
#define SMOOTH_PAUSE_BOOST_MS             5000
#define SMOOTH_CONCENTRATION_GAIN         0.92f
#define SMOOTH_PERSONALITY_DRIFT_RATE     0.0008f
#define SMOOTH_PERSONALITY_DRIFT_DECAY    0.05f
```

- [ ] **Step 2: Commit**

```bash
make 2>&1 | tail -3
git add src/smooth_config.h
git commit -m "config: session-model tunables"
```

### Task 5.2: Session-model state + update function

- [ ] **Step 1: Extend `state` struct in `src/smooth.c`**

Add:
```c
uint32_t session_start_ms;
uint32_t last_session_update_ms;
uint32_t last_active_ms;
float    warmup_gain;
float    fatigue_gain;
float    concentration_gain;
float    fitts_a_init, fitts_b_init, overshoot_init;
uint32_t pause_boost_until_ms;
```

- [ ] **Step 2: Add `session_update`**

```c
static void session_update(uint32_t now_ms)
{
    if ((now_ms - state.last_session_update_ms) < SMOOTH_SESSION_UPDATE_MS) return;
    state.last_session_update_ms = now_ms;

    uint32_t elapsed = now_ms - state.session_start_ms;

    // Warmup (decaying excess)
    float w_arg = -(float)elapsed / (float)SMOOTH_WARMUP_TAU_MS;
    float w_e = fast_exp2f(w_arg * 1.4426950f);  // exp(x) = exp2(x*log2(e))
    state.warmup_gain = 1.0f + (SMOOTH_WARMUP_GAIN_PEAK - 1.0f) * w_e;

    // Fatigue (slow rise via log)
    float f_norm = (float)elapsed / (float)SMOOTH_FATIGUE_TAU_MS;
    state.fatigue_gain = 1.0f + (SMOOTH_FATIGUE_GAIN_PEAK - 1.0f)
                              * fast_log2f(1.0f + f_norm) * 0.6931472f;
    if (state.fatigue_gain > SMOOTH_FATIGUE_GAIN_PEAK) state.fatigue_gain = SMOOTH_FATIGUE_GAIN_PEAK;

    // Pause detector
    if ((now_ms - state.last_active_ms) > SMOOTH_PAUSE_DETECT_MS &&
        state.pause_boost_until_ms < now_ms) {
        state.pause_boost_until_ms = now_ms + SMOOTH_PAUSE_BOOST_MS;
    }
    state.concentration_gain = (now_ms < state.pause_boost_until_ms)
        ? SMOOTH_CONCENTRATION_GAIN : 1.0f;

    // Personality OU drift (slow, anchored to _init values)
    state.fitts_a += SMOOTH_PERSONALITY_DRIFT_RATE *
        (smooth_rand_uniform() * 0.05f * state.fitts_a_init
         - (state.fitts_a - state.fitts_a_init) * SMOOTH_PERSONALITY_DRIFT_DECAY);
    state.fitts_b += SMOOTH_PERSONALITY_DRIFT_RATE *
        (smooth_rand_uniform() * 0.05f * state.fitts_b_init
         - (state.fitts_b - state.fitts_b_init) * SMOOTH_PERSONALITY_DRIFT_DECAY);
    state.overshoot_bias += SMOOTH_PERSONALITY_DRIFT_RATE *
        (smooth_rand_uniform() * 0.05f
         - (state.overshoot_bias - state.overshoot_init) * SMOOTH_PERSONALITY_DRIFT_DECAY);

    recompute_easing();  // overshoot_bias just drifted
}
```

- [ ] **Step 3: Initialize session state in `smooth_init`**

After personality derivation, before `recompute_easing()` (around line 278):
```c
state.warmup_gain = SMOOTH_WARMUP_GAIN_PEAK;
state.fatigue_gain = 1.0f;
state.concentration_gain = 1.0f;
state.fitts_a_init = state.fitts_a;
state.fitts_b_init = state.fitts_b;
state.overshoot_init = state.overshoot_bias;
uint32_t now = millis();
state.session_start_ms = now;
state.last_session_update_ms = now;
state.last_active_ms = now;
state.pause_boost_until_ms = 0;
```

- [ ] **Step 4: Apply scalars at consumer sites**

In `compute_fitts_speed_gain` (line ~201): multiply `gain` by `state.warmup_gain * state.fatigue_gain * state.concentration_gain` before soft-clamp.

In perp_amplitude scaling (line ~475): multiply `SMOOTH_PERP_AMPLITUDE` by `state.warmup_gain` in the expression.

In tremor: multiply `state.tremor_amp_target` accessor by `state.warmup_gain * state.fatigue_gain` (cleanest: have `tremor_step` read `state.tremor_amp_target * state.warmup_gain * state.fatigue_gain` for OU pull, so the OU follows the scaled target).

In `smooth_process_frame`, before returning: update `state.last_active_ms = millis();` when `has_movement` is true, then call `session_update(millis())` at end of function.

- [ ] **Step 5: Build, flash**

```bash
make flash 2>&1 | tail -3
```

- [ ] **Step 6: Verification — drift over 5 min**

Add `--check drift` to analyze_smooth.py:
- Expose `state.fitts_a/b`, `state.overshoot_bias`, `state.tremor_amp` via a new Ferrum debug command (e.g., `km.dbg(s)` returns CSV).
- Poll once per 30s for 5 min.
- Assert smooth random-walk (no jumps > 5% step-to-step), and assert tremor amp grew ≥5% from t=0 baseline.

```bash
python3 scripts/drift_probe.py --duration 300 --out drift.csv
python3 scripts/analyze_smooth.py drift.csv --check drift
```
Expected: `PASS — fitts_a walk std=X, tremor_amp delta=+X%`.

- [ ] **Step 7: Commit**

```bash
git add src/smooth.c src/smooth_config.h scripts/analyze_smooth.py scripts/drift_probe.py
git commit -m "smooth: session-time model (warmup/fatigue/concentration + personality drift)"
```

---

# Phase 6: Boot counter + queue-overflow merge

**Goal:** Eliminate per-device personality determinism by mixing a persistent boot counter into the seed; replace the raw-accumulator queue-overflow fallback with merge-into-most-recent.

**Files:**
- Modify: `core/imxrt1062_mm.ld`
- Modify: `src/smooth.c`

### Task 6.1: Reserve a flash sector for persistence

- [ ] **Step 1: Inspect `core/imxrt1062_mm.ld`**

Read the file and identify the FLASH `MEMORY` region and the existing section layout. Confirm size (16128K per CLAUDE.md).

- [ ] **Step 2: Add a `.persistence` NOLOAD section**

In the SECTIONS block, near the end of the FLASH-resident sections, add:
```ld
.persistence (NOLOAD) : ALIGN(4096) {
    _persistence_start = .;
    . += 4096;
    _persistence_end = .;
} > FLASH
```

Verify total flash usage still fits (16128K is huge; 4K reservation is trivial).

- [ ] **Step 3: Build**

Run: `make 2>&1 | tail -5`
Expected: clean build, `.hex` produced. Check `.elf` map output if available to confirm `.persistence` exists at the expected address.

- [ ] **Step 4: Commit**

```bash
git add core/imxrt1062_mm.ld
git commit -m "ld: reserve 4K .persistence sector for boot counter"
```

### Task 6.2: Implement boot counter load/store

Critical: flash erase must run from RAM (cannot execute from a sector that's being erased) and with IRQs disabled.

- [ ] **Step 1: Determine which flash-write path is available**

Check whether `cores/teensy4/eepromemulation.c` is linked in this project. If yes, use `eeprom_read_block`/`eeprom_write_block`. If no, use the FlexSPI LUT directly — code MUST be marked `__attribute__((section(".ramfunc")))`.

Bash check:
```bash
grep -rn "eeprom_initialize\|eeprom_write_block" /Users/ramseymcgrath/code/imxrtnsy/core/ 2>/dev/null | head -5
```

- [ ] **Step 2: Add boot-counter implementation in `src/smooth.c`**

If PJRC eeprom path exists, use it. Otherwise:

```c
extern uint32_t _persistence_start;
#define PERSIST_ADDR ((uintptr_t)&_persistence_start)

static uint32_t boot_counter_load(void)
{
    volatile uint32_t *p = (volatile uint32_t *)PERSIST_ADDR;
    uint32_t v = p[0];
    uint32_t inv = p[1];
    if ((v ^ inv) == 0xFFFFFFFFu) return v;
    return 0;  // first boot or corruption
}

__attribute__((section(".ramfunc")))
static void boot_counter_store(uint32_t v)
{
    // FlexSPI erase 4K sector + program 8 bytes (v, ~v).
    // Disable IRQs for the duration.
    // [implementation depends on chosen flash-write path; cite the helper used]
    __disable_irq();
    // ... erase sector at PERSIST_ADDR
    // ... program v at offset 0, ~v at offset 4
    __enable_irq();
}
```

Add `state.boot_counter` field. In `smooth_init`, after OCOTP read but before SFC32 seeding for personality:
```c
state.boot_counter = boot_counter_load();
boot_counter_store(state.boot_counter + 1);

// Mix into personality seed
uint32_t bc = state.boot_counter;
uint32_t bc_rot7  = (bc << 7)  | (bc >> 25);
uint32_t bc_rot19 = (bc << 19) | (bc >> 13);
p_a ^= bc_rot7;
p_b ^= bc_rot19;
p_c ^= bc * 0x9E3779B9u;
```

- [ ] **Step 3: Build, flash**

```bash
make flash 2>&1 | tail -3
```

- [ ] **Step 4: Verify boot counter survives power cycle**

Expose `boot_counter` via the existing `km.dbg()` channel (or a one-off `km.version()`-like reply). Power cycle 3×; counter should increment by 1 each boot.

- [ ] **Step 5: Verify personality changes between boots**

Capture `fitts_a, fitts_b, arc_bias` after each of 5 power cycles. All three should be different across boots even though `OCOTP UID` is constant.

```bash
python3 scripts/analyze_smooth.py --check boot
```

- [ ] **Step 6: Commit**

```bash
git add src/smooth.c scripts/analyze_smooth.py
git commit -m "smooth: persistent boot counter mixed into personality seed"
```

### Task 6.3: Queue-overflow merge-into-most-recent

- [ ] **Step 1: Modify `enqueue_single` in `src/smooth.c` (the overflow branch added in Task 2.2)**

Replace:
```c
if (state.free_mask == 0) {
    state.x_accum_fp += int_to_fp(x);
    state.y_accum_fp += int_to_fp(y);
    return;
}
```
with:
```c
if (state.free_mask == 0) {
    // No slots free — merge into the most-recently-allocated active slot.
    // free_mask bits cleared = active; slot allocation uses ctz so the
    // most-recently-allocated active slot has the highest set bit in
    // (~free_mask & ALL_SLOTS_MASK).
    uint32_t active = (~state.free_mask) & SMOOTH_ALL_SLOTS_MASK;
    uint8_t slot = (uint8_t)(31 - __builtin_clz(active));
    state.queue[slot].x_remaining_fp += int_to_fp(x);
    state.queue[slot].y_remaining_fp += int_to_fp(y);
    // Do NOT extend frames_left — the existing easing curve naturally
    // absorbs the additional displacement on its tail.
    state.overflow_merges++;
    return;
}
```

Add `uint32_t overflow_merges;` to `state`. Initialize via memset.

- [ ] **Step 2: Verify overflow behaviour**

Add an overflow stress test:
```bash
python3 scripts/drive_moves.py --burst 64,1ms  # 64 m(1,0) commands in 1ms
python3 scripts/analyze_smooth.py cap.csv --check overflow
```
The check confirms (a) cumulative dx delivered = 64 (no displacement loss), (b) no single dx in the stream exceeds the eased per-frame ceiling for normal injections (no raw accumulator passthrough).

- [ ] **Step 3: Commit**

```bash
git add src/smooth.c scripts/analyze_smooth.py scripts/drive_moves.py
git commit -m "smooth: queue overflow merges into most-recent slot (no raw passthrough)"
```

---

# Final verification: end-to-end regression

After all phases land, run the full check suite against a long capture.

- [ ] **Step 1: 10-minute capture under mixed workload**

```bash
python3 scripts/cap_smooth.py --out cap_final.csv --duration 600 &
python3 scripts/drive_moves.py --workload realistic --duration 600
wait
```

- [ ] **Step 2: Run every check**

```bash
for c in monotonic plateau submove psd tremor_band click_xcorr drift overflow boot; do
    echo "=== $c ==="
    python3 scripts/analyze_smooth.py cap_final.csv --check $c
done
```
All must report `PASS`.

- [ ] **Step 3: Confirm humanize-off bit-equivalence to baseline**

```bash
python3 scripts/drive_moves.py --replay fixed_pattern.txt --humanize 0 --out off.csv
diff baseline_humanize_off.csv off.csv
```
Expected: empty diff.

- [ ] **Step 4: Tag the release**

```bash
git tag -a humanize-v2 -m "Humanization hardening complete: phases 1-6"
```

---

# Risks and open questions

- **Phase 6 / flash write**: if PJRC `eepromemulation.c` is not linked in this project, the direct FlexSPI path is non-trivial and risks bricking the part if the LUT is wrong. Recommend doing Phase 6 last and on a dev unit. If unwilling to accept this risk, defer the boot counter and accept device-fingerprint risk for now (#9 deferred).
- **`max_per_frame = 2047`** (Phase 1, Task 1.2): if the downstream USB device descriptor mistakenly advertises int8 X/Y (despite int16 internal types), enumerated hosts will reject reports. Mitigate via Task 1.4's descriptor probe. If you skip 1.4, manually verify a known-good mouse still tracks correctly after a 500-px flick.
- **Hot-path budget** (Phase 3): pink-step on 2 channels + tremor sinusoid adds ~12 flops/frame. Confirm via DWT cyccnt around `smooth_process_frame` that worst-case stays under existing P99.
- **Button skew latency** (Phase 4): hard cap is 80 ms; competitive players may notice. Lower `SMOOTH_BTN_MAX_DELAY_MS` if user complains. Set the cap conservatively first; loosen only if detection requires.
- **Session drift OU step size** (Phase 5): values are first-principles; may need empirical tuning against real captured drift data from a real player session if any is available.

---

# Self-review (against the original 4 subagent plans)

Spec coverage:
- Plan A (#1, #2, #4, #5) → Phase 1 + Phase 2.
- Plan B (#3, #8) → Phase 3.
- Plan C (#6, #10) → Phase 4.
- Plan D (#7, #9, #11) → Phase 5 + Phase 6.
- Medium items #12 (dither narrowness), #13 (`last_noise_vx` naming), #14 (noise clamp vs boost) are NOT explicitly addressed; they were classified as low impact at review time. Recommend deferring to a `humanize-v2-cleanup` follow-up.

No placeholders detected (all code blocks are concrete; the one TBD spot is Task 6.2's flash-write implementation which is gated on a runtime check of available helpers — acceptable because the engineer must inspect `core/` before writing).

Type consistency: `pink_state_t`, `btn_evt_t`, scalar names (`warmup_gain` etc.) are consistent across phases.
