# Built-in Humanization — Design & Findings

Status: draft for review
Date: 2026-06-02
Component: Hurra-v2 firmware (`src/humanize.c`, `src/smooth.c`, `src/kmbox.c`, `src/main.c`)

## 1. Problem

Humanization today lives in `src/smooth.c` as a **trajectory generator**, reachable only
through the Hurra `MOUSE_MOVE_SMOOTH` (0x11) frame / `act_move(..., smooth=true)`. Every
other path — raw `km.move`, real-mouse passthrough, synth-injected reports — bypasses it and
emits unhumanized motion. `src/humanize.c` is currently only a DWT-init stub.

The goal: make humanization a **built-in guarantee** — every report that leaves the device is
"sufficiently humanized" regardless of how the motion was produced — without breaking
low-latency closed-loop control or corrupting already-shaped input.

## 2. Threat model (what "sufficiently humanized" must defeat)

The hardware layer is already covered: the device enumerates as an ordinary HID mouse, so
USB-descriptor and `SendInput`-style API detection do not apply. All remaining risk is in the
**motion signal itself**, in two tiers:

**Tier 1 — statistical / kinematic (common detectors).** Flags:
- Zero or constant jerk; sustained constant-velocity or constant-acceleration phases (humans
  never hold these — jerk is always nonzero and varying).
- Quantized / repeated values (identical peak velocities, fixed step sizes, repeated params);
  human distributions are continuous.
- Perfectly straight paths, perfectly regular timing cadence, single-frame "teleport" jumps,
  saturated delta fields, superhuman speed/distance.

**Tier 2 — submovement / ballistic analysis (strong detectors, e.g. Activision patent
US11947742B2).** Human aiming decomposes into a fast ballistic primary submovement that
under/overshoots, then smaller corrective submovements, each a pseudo-ballistic velocity bump
with non-zero jerk, separated by perception-action-cycle limits. Flags motion lacking this
structure, "too optimal," or with a **last-minute correction injected right before a click**.

Sources: US11947742B2 (submovement detection); Castle "Bot or Not" (automated-cursor
detection); BeCAPTCHA-Mouse (arXiv 2005.00890) on kinematic feature sets (velocity,
acceleration, jerk, curvature, straightness).

### Tier mapping → two kinds of humanization
- **Signal-level** (jerk, continuous-distribution micro-noise, timing dither, no
  quantization/teleport/saturation, human kinematic caps) is a *bounded perturbation*. It can
  be applied to **every output** because it roughens the signal without replacing intent.
  Defeats **Tier 1 for all modes**.
- **Trajectory-level** (ballistic + corrective submovement structure) is a *path generator*.
  It defeats **Tier 2**, but only for open-loop "go to target" moves; it cannot be applied to a
  closed-loop optimal stream without buffering (latency) or to passthrough without corrupting
  real motion.

## 3. Decision

**Scope: an always-on, signal-level humanization filter** (Tier-1 guarantee for all input
modes). Tier-2 submovement realism remains the responsibility of whatever generates the
trajectory (the host, or the opt-in `smooth.c` generator).

**Mechanism: noise + light velocity dynamics.** A lightly-damped 2nd-order response
(mass-spring-damper — the human motor model) with a ~2–4 ms time constant, plus correlated
micro-noise, plus anti-quantization dither, all displacement-conserving and idle-gated.
Effective added latency ≈ 1–2 frames — the most realism that is still safe for closed-loop.

(Both decisions confirmed during brainstorming.)

## 4. Integration model (how inputs reach the device)

External input senders are the consumers. Confirmed from `rn-mouse/` (a representative
aimbot sender with source) and `Input-injection-testbench/protocols.h`:

- Senders connect to the device as a **kmbox/Ferrum-compatible serial endpoint** and stream
  ASCII commands — `km.move(dx,dy)\r\n`, `km.left/right`, `km.click`, `km.wheel`,
  `km.echo(0)`, `km.buttons(1)` — typically at 4 Mbps. For us that endpoint is the
  **hurra-bridge VCOM/PTY**, which already emulates this surface (`hurra-app/src/ferrum_parser.c`
  + `bridge.c` → Hurra binary). **Integration requires no protocol change.**
- The send pattern is a **dense stream of small relative deltas** off a move-queue/worker —
  i.e. the closed-loop / streaming workload, not sparse target flicks.
- **The host already owns trajectory humanization.** `rn-mouse` runs WindMouse path
  generation, Kalman X/Y filtering, `easeInOut` smoothing, target prediction, distance-based
  speed multipliers, and host-side sub-pixel overflow *before* sending.

Consequences for the design (all reinforce §3):
1. **Filter, not generator.** The device must not re-shape trajectories; WindMouse/Kalman
   output would be double-humanized and corrupted. Confirmed.
2. **The filter adds what the host pipeline lacks.** Kalman/easing produce *too-smooth,
   low-jerk* output — itself a Tier-1 flag. The signal filter re-injects varying jerk,
   continuous micro-noise, timing dither, and anti-quantization, which the host stripped.
   Complementary, not redundant.
3. **Path preservation is a hard requirement.** The filter must conserve displacement and stay
   a small perturbation so the host's aim still lands and its WindMouse curvature survives.
4. **CS2-DMA-class tools send no input** (ESP/radar only, explicitly no kmbox) — they are the
   data-producer half; the aim/input layer (e.g. Pro-CS2's closed aimbot, or `rn-mouse`) is
   separate and is the only thing that touches the device.

## 5. Architecture & placement

- Promote `src/humanize.c` from stub to the always-on filter module. Pure math + per-session
  state; no hardware/protocol knowledge.
  - `void humanize_filter_init(uint32_t interval_us);` — seed RNG (reuse the TRNG/OCOTP/DWT
    seeding currently in `smooth.c`), set time constants from the frame interval, draw
    per-power-cycle personality (noise amplitude, τ jitter) from the hardware UID.
  - `void humanize_filter_frame(int16_t *dx, int16_t *dy);` — in-place perturbation of one
    frame's composite delta, conserving displacement via internal accumulators.
  - `uint32_t humanize_timing_next(uint32_t base_ldval, bool *skip);` — generalize the existing
    PIT-reload jitter so it runs always (respecting the cloned bInterval — internal phase
    wobble only), not just when `smooth.c` is active.
- **Single output chokepoint.** The filter runs on the merged mouse delta (real passthrough +
  any injection) after `kmbox_merge_report` and before `usb_device_send_report`, applied
  exactly once to *every* mouse report — merged, synth (`kmbox_send_pending`), and wheel paths.
  Implementation note for the plan: consolidate the mouse-report send sites or insert the
  filter at each, so coverage is exactly-once with no path missed.
- The filter reads/writes the report's delta fields via the existing `mouse_layout`
  field accessors used by the merge.

## 6. Filter algorithm (per frame, dt = frame interval)

Input: composite per-frame delta `(dx_in, dy_in)`. State: `owed_{x,y}` accumulator, output
velocity `v_{x,y}`, sub-pixel residual `r_{x,y}`, EWMA noise terms, RNG.

1. **Idle gate** — if `dx_in==dy_in==0` and velocity settled → output 0, decay state, return.
   No tremor on a still cursor.
2. **Displacement-conserving velocity dynamics** — `owed += d_in`; emit this frame through a
   lightly-damped 2nd-order response (τ ≈ 2–4 ms) toward draining `owed`; `owed -= emitted`.
   Output velocity gains natural rise/settle/jerk; `owed → 0` guarantees Σout = Σin; effective
   latency ≈ 1–2 frames.
3. **Correlated micro-noise** — zero-mean perpendicular + tangential noise, EWMA-correlated,
   scaled by current speed (reuse `smooth.c` noise/RNG primitives). Adds curvature and varying
   jerk; zero-mean so it does not drift net displacement.
4. **Anti-quantization dither + sub-pixel carry** — accumulate fractional output in `r`, emit
   the integer part, carry the remainder. Steady motion never emits identical repeated values
   and small motion never rounds to zero.
5. **Human kinematic clamp** — cap per-frame magnitude to a human ceiling; overflow carries via
   `owed` (never clip-drop, never teleport). Will not bite genuine passthrough.
6. **Timing jitter** — `humanize_timing_next` dithers the PIT reload within bInterval.

Invariants: displacement conserved (accumulators), bounded latency (τ), idle→zero,
no superhuman/teleport frames.

## 7. Consolidation: one humanization module

Resolved direction: **consolidate to a single clean path; no two subsystems.**

The `smooth.c` trajectory generator (32-slot queue, easing, Fitts gain, overshoot/arc) is
**already dead on the real integration path** — the bridge maps `km.move → hurra_move` (raw
0x10) and never calls `hurra_move_smooth` (0x11); only `examples/hello.c` does. So retiring it
removes complexity without affecting how inputs actually arrive.

Plan:
- Fold the reusable primitives `smooth.c` already has — hardware-seeded RNG, EWMA correlated
  noise, sub-pixel accumulation, PIT timing jitter — into the single `humanize.c` filter.
- Retire the standalone generator (queue + easing + Fitts/overshoot/arc) and its `smooth_*`
  trajectory API. `MOUSE_MOVE_SMOOTH` frames, if ever received, route through the normal
  inject → filter path (i.e. become equivalent to `MOUSE_MOVE` plus the always-on filter).
- Net result: humanization lives in exactly one module (`humanize.c`), on one path, applied to
  every output. The merge's carry + per-frame-cap fixes are subsumed by the filter's step 5.

## 8. Control surface

- **Default-on, no user config required.** The filter initialises to a sensible level at boot
  (`HUMANIZE_DEFAULT_LEVEL`), independent of the host — if a sender never issues a command, the
  guarantee still holds. This is the primary requirement.
- A minimal `km.human(level)` command, levels `0..3` (0 = off, 1 = light, 2 = default,
  3 = strong), only tunes/disables intensity — mainly so the load and aim tests can measure raw
  vs filtered. Keep it trivial: one integer arg mapping to a small preset table of
  (noise amplitude, τ); no per-parameter API. Wired through the Ferrum firmware parser and the
  hurra-app bridge. If wiring proves non-trivial, ship default-on hardcoded and defer the
  command.
- Per-power-cycle personality (noise amplitude, τ) seeded from the hardware UID so two
  sessions/devices are not statistically identical.

## 9. Verification

- **Pure-core unit tests** (host-compiled `humanize.c`): displacement conservation
  (Σout == Σin within rounding over long sequences), idle→zero, bounded latency, kinematic
  clamp + carry. TDD target.
- **Statistical analyzer** (`tools/humanization_analyze.py`): ingest a captured motion trace
  and compute Tier-1 metrics — fraction of zero-jerk frames, repeated-value run lengths,
  peak-velocity histogram continuity, straightness/curvature — with pass thresholds derived
  from a **real human baseline** (the passthrough mouse capture).
- **Acceptance tests:** (a) feed a synthetic straight constant-velocity stream → output must now
  pass the Tier-1 metrics; (b) feed a recorded human trace → the filter must not degrade its
  human-ness. (c) feed a WindMouse/Kalman stream (captured from `rn-mouse`) → output passes
  Tier-1 and total displacement is preserved within tolerance.

## 10. Limitations (explicit)

- Defeats Tier-1 for all input modes. Does **not** manufacture Tier-2 submovement/ballistic
  structure for a closed-loop optimal stream — a straight robotic snap becomes a "noisy straight
  snap," not a human flick. Tier-2 is the host's / opt-in generator's job.
- Adds ~1–2 frames latency by construction (the τ of the velocity dynamics).

## 11. Decisions (resolved)

1. **Consolidate to one module; retire the standalone `smooth.c` generator.** It is already off
   the active integration path. Fold its primitives into `humanize.c`. (§7)
2. **Add `km.human(level)` — minimal, default-on.** Boot default = level 2, applied at init with
   no dependence on host config; the command only tunes/disables. Ship default-on hardcoded if
   the command wiring is non-trivial. (§8)

Overriding principle from review: **keep the code clean and simple** — one path, one module,
small preset table, no speculative configurability.
