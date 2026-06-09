# Adaptive Feed-Rate + Normalized-Speed Humanization — Implementation Plan

Status: proposed. Grounded against i.MXRT1060 RM, NXP MCUXpresso SDK (fsl_pit.c / fsl_gpt.c),
PJRC Teensy 4 core (startup.c, imxrt1062.ld), ARM Cortex-M7 TRM, and USB 2.0 / EHCI / HID specs.
Target: Teensy MicroMod (i.MXRT1062, Cortex-M7 @ F_CPU=816 MHz), USB-host mouse passthrough.

The goal MAKD described: make injection cadence follow the *real* measured mouse poll
interval instead of nominal bInterval, and scale humanization noise by a DPI-independent
speed estimate. Research confirmed the idea is sound — but surfaced two pre-existing clock
bugs that must be fixed first, and several corrections to the mental model.

---

## PHASE 0 — Fix pre-existing clock bugs (BLOCKER, do first, independent of feature)

These are wrong *today*; the adaptive feature is meaningless until they're fixed because it
both sets timer periods and measures intervals using these miscalibrated clocks.

### 0.1 PIT clock constant (8.5× error)
- File: `src/main.c:185`
- Current: `uint32_t ipg_mhz = (F_CPU / 4u) / 1000000u;`  → 816/4 = 204 MHz (WRONG)
- Truth: PIT is clocked by **PERCLK = 24 MHz**, fixed in `core/startup.c:406`
  (`PERCLK_CLK_SEL=1` = 24 MHz XTAL, `PERCLK_PODF` cleared = /1).
  Confirmed by NXP fsl_pit.c, RM PIT chapter, and Teensy core (reboot uses LDVAL=2400000 for 100ms).
- Fix:
  ```c
  #define PIT_CLK_HZ 24000000u   /* PERCLK = 24 MHz OSC, per core/startup.c:406 */
  uint32_t ldval = (uint32_t)(((uint64_t)PIT_CLK_HZ * interval_us) / 1000000u) - 1u;
  ```
- Effect: injection cadence currently runs ~8.5× too slow; this restores true bInterval timing.
- Resolution at 24 MHz = 41.667 ns/tick; max 32-bit interval ≈ 179 s. Working range
  [125 µs, 10 ms] → LDVAL ≈ 2,999 … 239,999. Ample headroom.

### 0.2 GPT2 timestamp tick rate (used by the new measurement path)
- File: `src/gpt_profile.h:18` — `GPT2_PR = 203` assumes ipg_clk = 204 MHz → 1 MHz tick.
- Suspect: on this part ipg_clk is NOT reliably 204 MHz (IPG capped ~150 MHz). GPT CLKSRC=001
  selects ipg_clk; the true rate must be derived, not assumed.
- Action (pick one):
  - (a) Re-derive the real ipg_clk from the CCM divider chain and recompute GPT2_PR so the
        tick is a genuine 1 MHz, OR
  - (b) Keep GPT2_PR as-is but store a measured `gpt_ticks_per_us` and convert at use sites.
  - Empirical cross-check: compare GPT2 deltas against a known cadence (e.g. a 1 kHz FS mouse
        or the PIT once 0.1 is fixed) and confirm 1 tick ≈ 1 µs.
- Until this is verified, treat `gpt_profile_us()` as "ticks", not microseconds.

### 0.3 Verify register/bit facts already confirmed (no action, reference)
- PIT regs/bits: imxrt.h:7627-7658 (TEN=1<<0, TIE=1<<1, CHN=1<<2, TIF=1<<0). All real.
- GPT regs: CR/PR/SR/CNT correct; note bare `GPT2_OCR`/`GPT2_ICR` don't exist — they're
  OCR1/2/3 and ICR1/2 (not used here).
- GPT2 FRR free-run + CLKSRC=001 in gpt_profile.h are correctly encoded.

---

## PHASE 1 — Timestamp real report arrival (offload timing to hardware)

### 1.1 Capture point
- File: `src/main.c:248`, immediately after `if (ret > 0 && rpt_ptr) {`, BEFORE `kmbox_merge_report`.
  ```c
  if (ret > 0 && rpt_ptr) {
      did_work = true;
      uint32_t report_ts = gpt_profile_us();      /* GPT2_CNT single-load, atomic */
      humanize_record_arrival(ep_map[m].iface_protocol, report_ts);
      kmbox_merge_report(ep_map[m].iface_protocol, rpt_ptr, ret);
  ```
- Rationale: this is the single precise "a new report just arrived" point. The completion is
  *detected* in `usb_host.c:676-679` (qTD token ACTIVE 1→0), but the timestamp belongs at the
  main-loop consumption site. Only timestamp the MOUSE interface (protocol==2); keyboard
  reports must not perturb mouse cadence.

### 1.2 Hard limits to design around (from USB/EHCI/HID fact-check)
- There is NO sensor-sample timestamp in USB. We measure DELIVERY time only, with up to one
  full poll interval of irreducible aliasing. Never assume "real" motion timing.
- Measured jitter is dominated by host/EHCI quantization (125 µs microframe HS / 1 ms frame FS)
  + our ISR/poll latency — the same signal a poll-rate tester averages.
- Confirm bus speed: FS mouse → bInterval in ms; HS mouse → 2^(bInterval-1) microframes.
  This is the #1 timing-math bug; `main.c:171-178` already branches on speed — keep that.

---

## PHASE 2 — Derive measured interval + adaptive PIT period (the "dynamic feed rate")

### 2.1 New state in `humanize.c` struct S (main-loop only, NO volatile needed)
```c
uint32_t last_report_ts;      /* GPT ticks of previous mouse report (0 = none yet) */
uint32_t meas_interval;       /* EWMA-smoothed delivery interval, GPT ticks */
uint32_t arrival_count;       /* reports seen since init (skip first few) */
```

### 2.2 `humanize_record_arrival(uint8_t proto, uint32_t ts)`  — new fn, main loop
- If proto != mouse: return.
- First report (count==0): store ts, count=1, return (no interval yet).
- dt = ts - last_report_ts  (unsigned, single-wrap safe; intervals are sub-ms).
- Reject outliers BEFORE smoothing:
  - dropout: dt > ~4× current target → reset baseline (store ts, don't update meas), treat as idle.
  - burst/double-report: dt < ~0.5× current target → ignore (USB retry / NAK artifact).
- EWMA (integer, ISR-readable): `meas_interval += ((int32_t)dt - (int32_t)meas_interval) >> 4;`
- Store last_report_ts = ts; count++.

### 2.3 Adaptive base with SLEW (do NOT jump the period)
- File: `src/main.c:228-231` (main-loop pit_tick handler — FPU/64-bit allowed here, NOT in ISR).
- Convert smoothed interval → target LDVAL using the SAME 24 MHz math as Phase 0.1.
- Clamp target to [125 µs, 10 ms] LDVAL window FIRST (a glitch must never reach the timer).
- Slew `pit_base_ldval` toward target (~3% per tick), don't assign directly:
  ```c
  if (pit_tick_pending) {
      pit_tick_pending = false; did_work = true;
      uint32_t tgt = humanize_target_ldval(PIT_CLK_HZ);   /* 0 if <5 reports or invalid */
      if (tgt) {
          int32_t err = (int32_t)tgt - (int32_t)pit_base_ldval;
          /* optional: clamp |err| to a max step for extra safety */
          pit_base_ldval += err >> 5;                      /* ~3%/tick critically-damped slew */
      }
      pit_next_ldval = humanize_timing_next(pit_base_ldval);  /* existing ±12% LFSR jitter rides on top */
  }
  ```
- Why this is RM-correct: a new LDVAL latches at the next reload (NXP fsl_pit.c:
  "value is loaded after the timer expires"). Writing pit_next_ldval in the ISR at the reload
  boundary applies it to the cycle just starting. No TEN disable/re-enable (that would drop the
  in-flight countdown = one short/long boundary tick). Keep the existing precompute-in-ISR pattern.
- The existing `humanize_timing_next` LFSR jitter layer is UNCHANGED; it now jitters a moving base.

### 2.4 Concurrency (verified safe)
- ISR↔main shared: `pit_tick_pending` (volatile bool), `pit_next_ldval` (volatile u32). Both atomic. OK.
- `pit_base_ldval`, all `S.*` fields: main-loop only. No volatile. OK.
- FPU stays out of the ISR (ISR only does the single u32 LDVAL store). Confirmed.

---

## PHASE 3 — Normalized-speed envelope (DPI-independent noise scaling)

### 3.1 Corrections baked in (from HID/DSP fact-check)
- Per-report delta conflates speed, DPI, AND poll rate. The DPI-independent speed proxy is
  **delta_magnitude ÷ measured_interval** (counts/sec), not raw per-report delta.
- Normalize against an OBSERVED running peak (envelope), NOT the field logical max (16-bit
  mice almost never saturate, so logical max is a useless normalizer).
- Parse the report descriptor's Logical Max at enumeration; don't assume 8-bit −127..127.
  (If descriptor parsing isn't already available, the running-peak envelope makes this
  non-blocking — it self-calibrates regardless of field width. Note as a refinement.)
- Decay MUST be time-based, not per-frame: a fixed per-frame decay leaks 8× too fast on an
  8 kHz mouse. Use dt from Phase 2.

### 3.2 State in S
```c
float peak_speed;   /* envelope of counts/sec, running peak with time-based decay */
```

### 3.3 In `humanize_filter` (humanize.c:104-131), where `speed` is already computed
```c
float speed = sqrtf(ex*ex + ey*ey);                 /* existing: counts this frame */
/* counts/sec using measured interval; guard div-by-zero */
float rate = (S.meas_interval > 0)
           ? speed * (float)GPT_TICKS_PER_SEC / (float)S.meas_interval
           : speed;
/* envelope: instant attack, time-based decay */
if (rate > S.peak_speed) S.peak_speed = rate;       /* attack */
else S.peak_speed *= expf(-dt_sec / HZ_PEAK_TAU);   /* decay; or precomputed per-frame k */
if (S.peak_speed < 1.0f) S.peak_speed = 1.0f;       /* floor: no div-by-zero / blowup at rest */
float norm = rate / S.peak_speed;                   /* 0..1 normalized speed */
/* swap the noise driver from raw speed → normalized speed */
float nmag = S.n_perp * S.noise_amp * norm;         /* was: * speed */
```
- `expf` is fine (M7 hardware FPU, main loop). If you want zero transcendental cost, precompute
  a per-frame decay constant `k` from the nominal period and use `S.peak_speed *= k;`.
- Open tuning question: scaling noise by `norm` (0..1) vs by `speed` changes feel — keep behind
  a level/flag and A/B on hardware. This is a behavior change, not a correctness fix.

### 3.4 Decision still open (ask before coding 3.x)
- Combined-magnitude envelope (one peak) vs per-axis (peak_x, peak_y). Combined is simpler and
  matches the perpendicular-noise model; per-axis is what MAKD's "max X_Y" literally said.
  Default to combined unless you want per-axis.

---

## PHASE 4 — Determinism hardening (cheap, lock in what's already good)

- Research confirmed `S` is already in DTCM and hot code in ITCM *by default* (no annotations),
  and DTCM is single-cycle / non-cacheable → already deterministic. Two no-risk hardening steps:

### 4.1 Make the hot-path placement explicit (LTO/flag-proof)
- File: `src/humanize.c` — add `__attribute__((section(".fastrun")))` to the per-tick path:
  `humanize_filter`, `drain_axis`, `humanize_timing_next`, `humanize_return`, `humanize_pending`.
  (`humanize_init`, `humanize_set_level` can stay default.)
- Do NOT move `S` to `.dmabuffers` — that would push it to OCRAM (cacheable-ish/slower). Leave
  `S` in .bss (→ DTCM). Optional `aligned(32)` is cosmetic here.

### 4.2 Guard the non-cacheable DMA window
- `core/startup.c:215-219` marks the first 64 KB of OCRAM non-cacheable (MPU region 10) — THIS is
  why the `asm volatile("dsb")`-only pattern in usb_host.c is correct and no cache clean/invalidate
  is needed. Don't "fix" the dsb pattern away.
- Add a linker `ASSERT(SIZEOF(.bss.dma) <= 64K, ...)` (or widen region 10) so a future growth of
  `.dmabuffers` past 64 KB can't silently land in cacheable memory and break DMA coherency.
- Add a one-line comment at the qTD-arm sites noting "dsb suffices BECAUSE DMA mem is MPU-noncacheable".

---

## New / changed public API (humanize.h)
- `void     humanize_record_arrival(uint8_t iface_protocol, uint32_t ts_ticks);`
- `uint32_t humanize_target_ldval(uint32_t pit_clk_hz);`   /* 0 = no valid measurement yet */
- (internal) extend struct S; add HZ_PEAK_TAU, GPT_TICKS_PER_SEC constants.

## Build / sequencing
1. Phase 0 (clock fixes) — land + verify on hardware FIRST. Measure cadence before/after.
2. Phase 1 + 2 (timestamp → adaptive PIT) — verify PIT period tracks a real mouse; scope/log it.
3. Phase 3 (normalized speed) — behind a flag; A/B feel.
4. Phase 4 (hardening) — anytime; zero behavior change.

## Risks / watch-items
- GPT2 tick calibration (0.2) gates ALL measurement accuracy — verify empirically.
- Closed-loop stability: target-LDVAL math MUST match the PIT-set math exactly, or the loop drifts.
- Slew + outlier rejection prevent a measurement glitch from jerking the period (detectable).
- Report-descriptor field width (8 vs 16-bit) — running-peak envelope self-calibrates, but a true
  counts/sec needs the descriptor's logical max only if you ever want absolute (not relative) speed.
