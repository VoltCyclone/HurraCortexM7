#pragma once
// Smooth injection — tunable parameters
// Edit these to adjust humanization behavior without touching smooth.c.

// ---- Easing curve shape ----
// Attack phase: fraction of total time spent in fast ramp [0.0, 1.0]
#define SMOOTH_EASE_ATTACK_FRAC      0.30f
// Attack phase: fraction of total distance covered during attack [0.0, 1.0]
#define SMOOTH_EASE_ATTACK_DIST      0.60f

// ---- Spread duration (microseconds) ----
// How long to spread a movement over, by distance bucket.
// Longer = smoother easing, shorter = more responsive.
#define SMOOTH_SPREAD_SMALL_US       3500    // <20 px  (short to minimize cmd/obs gap)
#define SMOOTH_SPREAD_MEDIUM_US      5000    // 20-60 px
#define SMOOTH_SPREAD_LARGE_US       4000    // 60-120 px
#define SMOOTH_SPREAD_XLARGE_US      3000    // >120 px

// Distance thresholds for spread buckets (pixels)
#define SMOOTH_SPREAD_SMALL_PX       20
#define SMOOTH_SPREAD_MEDIUM_PX      60
#define SMOOTH_SPREAD_LARGE_PX       120

// Spread duration jitter: ± this fraction of spread_us
#define SMOOTH_SPREAD_JITTER         0.25f

// Min/max frames per injection
#define SMOOTH_MIN_FRAMES            3
#define SMOOTH_MAX_FRAMES            128

// ---- EWMA noise channels ----
// Alpha for correlated noise (higher = slower drift, longer correlation)
// 0.97 at 1kHz -> ~33ms correlation time (30Hz bandwidth)
#define SMOOTH_EWMA_ALPHA            0.97f
#define SMOOTH_EWMA_BETA             (1.0f - SMOOTH_EWMA_ALPHA)

// ---- Speed perturbation ----
// Fallback speed gain when no Fitts data available
#define SMOOTH_SPEED_GAIN_DEFAULT    1.2f
// Overall speed noise amplitude scaling (controls movement-level CV).
// With EWMA alpha=0.97, speed_noise std ≈ 0.071.
// At SCALE=0.55, short hops (gain≈2.0) → ~0.08 per-frame CV → ~0.16 movement CV
// Long throws (gain≈0.55) → ~0.02 per-frame CV → ~0.04 movement CV
#define SMOOTH_SPEED_NOISE_SCALE     0.55f

// ---- Perpendicular wobble ----
// Base wobble amplitude (pixels, before arc_scale)
#define SMOOTH_PERP_AMPLITUDE        2.0f

// Low-speed angle smoothing: wobble boost at low speeds
// arc_scale = 1 + BOOST / (speed_px + 1)
// At 1px/frame: ~2x, at 3px: ~1.4x, at 8px+: ~1.2x
#define SMOOTH_LOWSPEED_BOOST        1.5f

// ---- Session personality ranges ----
// Arc bias: dominant perpendicular direction (randomized per session)
#define SMOOTH_ARC_BIAS_RANGE        0.4f
// Arc bias O-U drift: dx = RATE * (rand*INPUT - bias*DECAY)
// theta = RATE*DECAY = 0.0000165 -> tau ~60s at 1kHz
// Steady-state std ~0.15 (wander within ±0.4 init range)
#define SMOOTH_ARC_DRIFT_RATE        0.005f
#define SMOOTH_ARC_DRIFT_INPUT       0.30f
#define SMOOTH_ARC_DRIFT_DECAY       0.0033f

// Easing personality: per-session perturbation of attack fraction [-range, +range]
// Positive = longer attack (snappier start), negative = shorter attack (more gradual)
#define SMOOTH_OVERSHOOT_RANGE       0.05f

// Fitts' Law coefficients: MT = a + b * log2(dist + 1)
// Intercept range: [min, min + span]
#define SMOOTH_FITTS_A_MIN           0.08f
#define SMOOTH_FITTS_A_SPAN          0.07f
// Slope range: [min, min + span]
#define SMOOTH_FITTS_B_MIN           0.08f
#define SMOOTH_FITTS_B_SPAN          0.07f
// Gain numerator: gain = numerator / MT, soft-clamped to [min, max]
// 0.50 calibrated so mid-personality spans [0.7, 1.3] for typical movements
#define SMOOTH_FITTS_GAIN_NUM        0.50f
#define SMOOTH_FITTS_GAIN_MIN        0.5f
#define SMOOTH_FITTS_GAIN_MAX        2.5f
// Softplus k for soft clamping (higher = softer transition at boundaries)
#define SMOOTH_FITTS_GAIN_SOFT_K     0.15f

// ---- Noise velocity continuity ----
// Smoothing alpha for noise components only (base easing is unsmoothed)
// Higher = more responsive, less smoothing. No displacement leak.
#define SMOOTH_VELOCITY_ALPHA        0.40f
// Noise decay rate when idle (per frame, prevents stale noise momentum)
#define SMOOTH_VELOCITY_DECAY        0.85f

// ---- Timing humanization (PIT interval jitter) ----
// Simulates real mouse USB polling characteristics:
//   - Occasional missed polls (right-skew)
//   - Gaussian-ish interval spread (natural CV)
//   - Per-session poll rate personality
//
// Poll skip: DISABLED — creates bimodal interval distribution (detected as
// synthetic by analyzers). Pure continuous jitter is sufficient.
// #define SMOOTH_TIMING_SKIP_PROB_INV  25
// #define SMOOTH_TIMING_MAX_CONSEC_SKIP 2

// PIT reload jitter: ±fraction of base LDVAL, applied via triangular
// distribution (sum of 2 uniforms → bell-shaped, unimodal).
// Triangular std = 0.408 * JITTER, so 0.22 → CV ≈ 0.09.
// Real mice at 1kHz: CV ~0.05-0.15.
#define SMOOTH_TIMING_LDVAL_JITTER   0.22f  // ±22% peak, ~9% CV via triangular

// Per-session poll rate personality: slight offset to base interval.
// Real mice vary ±2-5% between units due to oscillator tolerance.
#define SMOOTH_TIMING_RATE_OFFSET    0.05f  // ±5% session bias on interval

// ---- Persistent micro-tremor ----
// Fills zero-movement gaps between injection commands with Brownian-like
// micro-movements during active injection. Mean-reverting to prevent drift.
// Only active for IDLE_TIMEOUT frames after the last injection queue activity;
// once expired, tremor and sub-pixel accumulator are zeroed to prevent
// phantom cursor drift when the system is truly idle.
#define SMOOTH_TREMOR_STEP           0.30f   // px, random walk step per frame
#define SMOOTH_TREMOR_DECAY          0.88f   // mean-reversion factor
#define SMOOTH_TREMOR_IDLE_TIMEOUT   100     // frames after last injection to keep tremor alive

// ---- Noise magnitude clamp ----
// Maximum ratio of noise to base displacement per frame.
// Prevents perpendicular wobble from dominating at low speeds where
// fixed-amplitude noise can exceed actual movement by 10x+.
// At 0.35: a 1px/frame movement allows 0.35px of noise.
#define SMOOTH_NOISE_MAX_RATIO       0.35f

// ---- Dithered rounding ----
// Varies sub-pixel rounding threshold ±RANGE around 0.5 to break up
// repeated identical integer deltas from smooth easing curves.
#define SMOOTH_DITHER_RANGE          0.35f

// ---- PRNG ----
// SFC32 warmup rounds (recommended 12+ for full diffusion)
#define SMOOTH_RNG_WARMUP            15
// EWMA channel priming rounds
#define SMOOTH_EWMA_PRIME_ROUNDS     20
