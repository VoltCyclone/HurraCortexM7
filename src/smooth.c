// Smooth injection queue for Cortex-M7
// Circular queue + asymmetric easing + session-level behavioral model.
// Tick-rate independent: spread frames scale with interval_us.
//
// Humanization model (v3):
//   - SFC32 PRNG — separate instances for personality vs frame noise
//   - EWMA correlated noise channels (speed + perpendicular wobble)
//   - Session personality: arc bias, easing shape, Fitts' Law coefficients
//   - Velocity continuity on noise only (no displacement leak)
//   - Soft-clamped Fitts gain (no distribution pile-up)
//   - Low-speed angle smoothing via adaptive wobble amplitude
//
// All tunable parameters are in smooth_config.h.
// Humanization can be toggled at runtime via smooth_set_humanize().

#include "smooth.h"
#include "smooth_config.h"
#include "imxrt.h"
#include <string.h>
#include <math.h>

// ---- Fixed-point helpers ----

static inline int32_t int_to_fp(int16_t v)
{
	return (int32_t)v << SMOOTH_FP_SHIFT;
}

static inline int16_t fp_to_int(int32_t fp)
{
	return (int16_t)((fp + SMOOTH_FP_HALF) >> SMOOTH_FP_SHIFT);
}

static inline int32_t fp_mul(int32_t a, int32_t b)
{
	return (int32_t)(((int64_t)a * b) >> SMOOTH_FP_SHIFT);
}

static inline int32_t fp_div(int32_t a, int32_t b)
{
	if (b == 0) return 0;
	return (int32_t)(((int64_t)a << SMOOTH_FP_SHIFT) / b);
}

// ---- Fast inverse square root (Quake-style + Newton-Raphson) ----
// IEEE 754 bit-hack gives ~12-bit initial estimate, one NR iteration
// refines to ~22 bits. ~10 cycles vs ~28 cycles for VSQRT+VDIV on M7.
static inline float fast_invsqrt(float x)
{
	union { float f; uint32_t u; } v = { .f = x };
	v.u = 0x5F3759DFu - (v.u >> 1);
	float est = v.f;
	est *= (1.5f - 0.5f * x * est * est); // Newton-Raphson
	return est;
}

// ---- Asymmetric easing (inline, no LUT) ----
// Models natural movement: quadratic attack → cubic deceleration.
// overshoot parameter shifts attack fraction per-session personality.
static inline int32_t ease_asymmetric(int32_t t, float overshoot)
{
	float tf = (float)t * (1.0f / (float)SMOOTH_FP_ONE);
	float attack_frac = SMOOTH_EASE_ATTACK_FRAC + overshoot;
	if (attack_frac < 0.05f) attack_frac = 0.05f;
	if (attack_frac > 0.95f) attack_frac = 0.95f;

	float result;
	if (tf < attack_frac) {
		float n = tf * (1.0f / attack_frac);
		result = n * n * SMOOTH_EASE_ATTACK_DIST;
	} else {
		float tail = 1.0f - attack_frac;
		float n = (tf - attack_frac) * (1.0f / tail);
		float omt = 1.0f - n;
		result = SMOOTH_EASE_ATTACK_DIST
			+ (1.0f - SMOOTH_EASE_ATTACK_DIST) * (1.0f - omt * omt * omt);
	}

	return (int32_t)(result * (float)SMOOTH_FP_ONE);
}

// ---- Queue entry ----
typedef struct {
	int32_t x_remaining_fp;
	int32_t y_remaining_fp;
	int32_t inv_total_fp;    // precomputed FP_ONE / total_frames
	int32_t eased_prev;      // cached eased(progress) from last frame
	float   speed_gain;      // Fitts-calibrated speed noise multiplier
	uint8_t frames_left;
	uint8_t total_frames;
} queue_entry_t;

// ---- State ----
static struct {
	int32_t x_accum_fp;
	int32_t y_accum_fp;
	queue_entry_t queue[SMOOTH_QUEUE_SIZE];
	uint32_t free_mask;   // bitmask: bit set = slot available
	uint8_t count;
	int16_t max_per_frame;
	float   dt;              // seconds per tick
	uint32_t interval_us;    // microseconds per tick
	bool    humanize;        // humanization toggle (runtime)

	// SFC32 PRNG — frame-level noise only
	uint32_t rng_a, rng_b, rng_c, rng_counter;

	// EWMA correlated noise channels (alpha/beta scaled by tick rate)
	float   ewma_alpha;      // per-tick alpha, scaled from SMOOTH_EWMA_ALPHA at 1kHz
	float   ewma_beta;       // 1 - ewma_alpha
	float   speed_noise;     // filtered speed perturbation
	float   perp_noise;      // filtered perpendicular wobble

	// Session personality (seeded from separate RNG at init)
	float   arc_bias;        // perpendicular direction bias
	float   overshoot_bias;  // easing attack fraction perturbation
	float   fitts_a;         // Fitts intercept
	float   fitts_b;         // Fitts slope

	// Noise velocity continuity (smooths only noise, not base easing)
	float   last_noise_vx;
	float   last_noise_vy;

	// Persistent micro-tremor (fills gaps between commands)
	float   tremor_x;
	float   tremor_y;

	// Timing humanization state
	uint8_t consec_skips;        // consecutive skips so far
	float   rate_bias;           // per-session interval offset (fraction)
	// TRNG entropy ring buffer: 32 words cached from hardware TRNG.
	// Double-buffered: 2x16-word fills from TRNG_ENT0-15.
	// At 1kHz consumption, 32 words last 32ms vs TRNG refill of ~0.25ms.
	// Virtually eliminates LFSR fallback.
	uint32_t trng_buf[32];
	uint8_t  trng_idx;           // next word to consume [0..31]
	// xorshift32 fallback when TRNG pool is refilling (~250µs window)
	uint32_t timing_lfsr;
} state;

// ---- SFC32 PRNG ----
// Generic version: operates on external state (used for personality seeding).
static inline uint32_t sfc32_draw(uint32_t *a, uint32_t *b, uint32_t *c,
	uint32_t *ctr)
{
	uint32_t t = *a + *b + (*ctr)++;
	*a = *b ^ (*b >> 9);
	*b = *c + (*c << 3);
	*c = ((*c << 21) | (*c >> 11)) + t;
	return t;
}

static inline float sfc32_uniform(uint32_t *a, uint32_t *b, uint32_t *c,
	uint32_t *ctr)
{
	uint32_t r = sfc32_draw(a, b, c, ctr);
	int32_t balanced = (int32_t)(r >> 8) - 0x800000;
	return (float)balanced * (1.0f / 8388608.0f);
}

// State-based version: uses the frame-noise RNG in state.
static inline uint32_t sfc32_next(void)
{
	return sfc32_draw(&state.rng_a, &state.rng_b, &state.rng_c,
		&state.rng_counter);
}

static inline float smooth_rand_uniform(void)
{
	return sfc32_uniform(&state.rng_a, &state.rng_b, &state.rng_c,
		&state.rng_counter);
}

// ---- Frame count heuristic (integer-only) ----
static uint8_t compute_spread_frames(int32_t abs_x_fp, int32_t abs_y_fp)
{
	int32_t max_comp = abs_x_fp > abs_y_fp ? abs_x_fp : abs_y_fp;
	int32_t px = max_comp >> SMOOTH_FP_SHIFT;

	uint32_t spread_us;
	if (px < SMOOTH_SPREAD_SMALL_PX)       spread_us = SMOOTH_SPREAD_SMALL_US;
	else if (px < SMOOTH_SPREAD_MEDIUM_PX) spread_us = SMOOTH_SPREAD_MEDIUM_US;
	else if (px < SMOOTH_SPREAD_LARGE_PX)  spread_us = SMOOTH_SPREAD_LARGE_US;
	else                                   spread_us = SMOOTH_SPREAD_XLARGE_US;

	if (state.humanize) {
		float jitter = smooth_rand_uniform() * SMOOTH_SPREAD_JITTER;
		spread_us = (uint32_t)((float)spread_us * (1.0f + jitter));
	}

	uint32_t frames = (spread_us + (state.interval_us >> 1)) / state.interval_us;
	if (frames < SMOOTH_MIN_FRAMES) frames = SMOOTH_MIN_FRAMES;
	if (frames > SMOOTH_MAX_FRAMES) frames = SMOOTH_MAX_FRAMES;
	return (uint8_t)frames;
}

// ---- Fast log2 approximation ----
// IEEE 754 trick + 3rd-order polynomial correction.
// Max error ~0.003 over [1, 2^20] — more than adequate for Fitts scaling.
static inline float fast_log2f(float x)
{
	union { float f; uint32_t u; } v = { .f = x };
	float exp_part = (float)((int32_t)(v.u >> 23) - 127);
	v.u = (v.u & 0x007FFFFFu) | 0x3F800000u; // mantissa in [1,2)
	float m = v.f;
	// Minimax polynomial for log2(m) on [1,2)
	return exp_part + (-0.34484843f * m + 2.02466578f) * m - 0.67487759f - 1.0f;
}

// ---- Fast exp2 approximation ----
// Schraudolph-style with polynomial refinement. Max error ~0.1%.
static inline float fast_exp2f(float x)
{
	if (x > 20.0f) x = 20.0f;
	if (x < -20.0f) return 0.0f;
	float fi = (x >= 0) ? (float)(int)(x) : (float)(int)(x) - 1.0f;
	float frac = x - fi;
	// Polynomial for 2^frac on [0,1): 1 + frac*(ln2 + frac*(ln2^2/2))
	float pow_frac = 1.0f + frac * (0.6931472f + frac * (0.2402265f + frac * 0.0558f));
	union { float f; uint32_t u; } v;
	v.u = (uint32_t)((int)(fi) + 127) << 23;
	return v.f * pow_frac;
}

// ---- Fitts' Law speed gain with soft clamping ----
// Uses fast_log2f/fast_exp2f to avoid newlib logf/expf (~50-100 cycles each).
// Softplus-based boundaries prevent distribution pile-up at clamp values.
static float compute_fitts_speed_gain(int32_t abs_x_fp, int32_t abs_y_fp)
{
	float dx = (float)(abs_x_fp >> SMOOTH_FP_SHIFT);
	float dy = (float)(abs_y_fp >> SMOOTH_FP_SHIFT);
	float dist2 = dx * dx + dy * dy;
	float dist = (dist2 > 1.0f) ? dist2 * fast_invsqrt(dist2) : 1.0f;

	float mt = state.fitts_a + state.fitts_b * fast_log2f(dist + 1.0f);
	float gain = SMOOTH_FITTS_GAIN_NUM / mt;

	// Soft lower bound: gain = lo + softplus(gain - lo)
	float k = SMOOTH_FITTS_GAIN_SOFT_K;
	float x_lo = (gain - SMOOTH_FITTS_GAIN_MIN) / k;
	if (x_lo < 10.0f) {
		// softplus(x) = k * log(1 + exp(x)) = k * log2(1 + exp2(x/ln2)) * ln2
		// Simplified: k * log(1 + exp(x)) ≈ k * (ln2 * fast_log2f(1 + fast_exp2f(x/ln2)))
		float ex = fast_exp2f(x_lo * 1.4426950f); // x / ln2
		gain = SMOOTH_FITTS_GAIN_MIN + k * 0.6931472f * fast_log2f(1.0f + ex);
	}

	// Soft upper bound: gain = hi - softplus(hi - gain)
	float x_hi = (SMOOTH_FITTS_GAIN_MAX - gain) / k;
	if (x_hi < 10.0f) {
		float ex = fast_exp2f(x_hi * 1.4426950f);
		gain = SMOOTH_FITTS_GAIN_MAX - k * 0.6931472f * fast_log2f(1.0f + ex);
	}

	return gain;
}

// ---- Public API ----

void smooth_init(uint32_t interval_us)
{
	memset(&state, 0, sizeof(state));
	state.free_mask = (SMOOTH_QUEUE_SIZE == 32) ? 0xFFFFFFFFu
		: ((1u << SMOOTH_QUEUE_SIZE) - 1u);
	state.max_per_frame = 127;
	state.humanize = true;
	state.interval_us = interval_us > 0 ? interval_us : 1000;
	state.dt = (float)state.interval_us / 1000000.0f;

	// Enable DWT cycle counter for PRNG seeding
	volatile uint32_t *dwt_ctrl   = (volatile uint32_t *)0xE0001000;
	volatile uint32_t *dwt_cyccnt = (volatile uint32_t *)0xE0001004;
	*dwt_ctrl |= 1;

	// Seed frame-noise RNG from first DWT read
	uint32_t seed1 = *dwt_cyccnt;
	state.rng_a = seed1 ^ 0xCAFEBABE;
	state.rng_b = seed1 ^ 0xDEADBEEF;
	state.rng_c = seed1 ^ 0x8BADF00D;
	state.rng_counter = 1;
	if (state.rng_a == 0) state.rng_a = 0xCAFEBABE;

	for (int i = 0; i < SMOOTH_RNG_WARMUP; i++) sfc32_next();

	// Personality seed from chip-unique OCOTP fuse ID + DWT (device + temporal
	// entropy). OCOTP shadow registers loaded from OTP fuses at POR.
	// An attacker who recovers seed1 from frame noise cannot derive personality
	// without the per-chip 64-bit hardware UID.
	volatile uint32_t *ocotp_cfg0 = (volatile uint32_t *)0x401F4410;
	volatile uint32_t *ocotp_cfg1 = (volatile uint32_t *)0x401F4420;
	uint32_t hw_uid0 = *ocotp_cfg0;
	uint32_t hw_uid1 = *ocotp_cfg1;
	uint32_t seed2 = *dwt_cyccnt;
	uint32_t p_a = seed2 ^ hw_uid0 ^ 0x12345678;
	uint32_t p_b = seed2 ^ hw_uid1 ^ 0x9ABCDEF0;
	uint32_t p_c = (hw_uid0 ^ hw_uid1) ^ 0xFEDCBA98;
	uint32_t p_ctr = 1;
	if (p_a == 0) p_a = 0x12345678;

	for (int i = 0; i < SMOOTH_RNG_WARMUP; i++)
		sfc32_draw(&p_a, &p_b, &p_c, &p_ctr);

	// Session personality — drawn from personality RNG, not frame RNG
	state.arc_bias = sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)
		* SMOOTH_ARC_BIAS_RANGE;
	state.overshoot_bias = sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)
		* SMOOTH_OVERSHOOT_RANGE;
	state.fitts_a = SMOOTH_FITTS_A_MIN
		+ fabsf(sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)) * SMOOTH_FITTS_A_SPAN;
	state.fitts_b = SMOOTH_FITTS_B_MIN
		+ fabsf(sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)) * SMOOTH_FITTS_B_SPAN;

	// Scale EWMA alpha by tick rate: SMOOTH_EWMA_ALPHA is calibrated for 1kHz.
	// alpha_scaled = alpha^(dt/dt_ref) = alpha^(interval_us/1000)
	// This preserves ~33ms correlation time regardless of poll rate.
	{
		float exponent = (float)state.interval_us / 1000.0f;
		// powf(alpha, exp) = exp2f(exp * log2f(alpha))
		// log2(0.97) ≈ -0.04394
		state.ewma_alpha = fast_exp2f(exponent * fast_log2f(SMOOTH_EWMA_ALPHA));
		if (state.ewma_alpha < 0.5f) state.ewma_alpha = 0.5f;
		if (state.ewma_alpha > 0.999f) state.ewma_alpha = 0.999f;
		state.ewma_beta = 1.0f - state.ewma_alpha;
	}

	// Prime EWMA noise channels (using frame RNG)
	for (int i = 0; i < SMOOTH_EWMA_PRIME_ROUNDS; i++) {
		state.speed_noise = state.ewma_alpha * state.speed_noise
			+ state.ewma_beta * smooth_rand_uniform();
		state.perp_noise  = state.ewma_alpha * state.perp_noise
			+ state.ewma_beta * smooth_rand_uniform();
	}

	// Timing humanization: per-session poll rate bias from personality RNG
	state.rate_bias = sfc32_uniform(&p_a, &p_b, &p_c, &p_ctr)
		* SMOOTH_TIMING_RATE_OFFSET;

	// LFSR fallback seed (used only when TRNG pool is refilling)
	state.timing_lfsr = sfc32_draw(&p_a, &p_b, &p_c, &p_ctr);
	if (state.timing_lfsr == 0) state.timing_lfsr = 0xDEADBEEF;

	// Start hardware TRNG — ring-oscillator sampling, no program mode.
	// Clock gate must be enabled before register access.
	CCM_CCGR6 |= CCM_CCGR6_TRNG(CCM_CCGR_ON);
	// Reset to defaults, clear any error state, enter run mode
	TRNG_MCTL = TRNG_MCTL_RST_DEF | TRNG_MCTL_PRGM;
	TRNG_MCTL = TRNG_MCTL_SAMP_MODE(2) | TRNG_MCTL_TRNG_ACC;
	// Fill 32-word ring: two full TRNG pool reads (512 bits each).
	// Each read of ENT15 auto-triggers the next generation cycle.
	for (int batch = 0; batch < 2; batch++) {
		while (!(TRNG_MCTL & TRNG_MCTL_ENT_VAL)) {}
		volatile uint32_t *ent = &TRNG_ENT0;
		for (int i = 0; i < 16; i++)
			state.trng_buf[batch * 16 + i] = ent[i];
	}
	state.trng_idx = 0;
}

void smooth_inject(int16_t x, int16_t y)
{
	if (x == 0 && y == 0) return;

	if (state.free_mask == 0) {
		state.x_accum_fp += int_to_fp(x);
		state.y_accum_fp += int_to_fp(y);
		return;
	}

	uint8_t slot = (uint8_t)__builtin_ctz(state.free_mask);
	state.free_mask &= state.free_mask - 1; // clear lowest set bit

	int32_t xfp = int_to_fp(x);
	int32_t yfp = int_to_fp(y);
	int32_t ax = xfp >= 0 ? xfp : -xfp;
	int32_t ay = yfp >= 0 ? yfp : -yfp;

	uint8_t frames = compute_spread_frames(ax, ay);

	state.queue[slot].x_remaining_fp = xfp;
	state.queue[slot].y_remaining_fp = yfp;
	state.queue[slot].inv_total_fp   = SMOOTH_FP_ONE / (int32_t)frames;
	state.queue[slot].eased_prev     = 0; // progress starts at 0
	state.queue[slot].speed_gain     = state.humanize
		? compute_fitts_speed_gain(ax, ay)
		: SMOOTH_SPEED_GAIN_DEFAULT;
	state.queue[slot].frames_left    = frames;
	state.queue[slot].total_frames   = frames;
	state.count++;
}

void smooth_process_frame(int16_t *out_x, int16_t *out_y)
{
	int32_t frame_x_fp = 0;
	int32_t frame_y_fp = 0;
	bool has_movement = false;

	float frame_speed_gain = 0.0f;
	float total_weight = 0.0f;

	float overshoot = state.humanize ? state.overshoot_bias : 0.0f;

	// Iterate only active slots via inverted free_mask
	uint32_t active = ~state.free_mask & ((SMOOTH_QUEUE_SIZE == 32)
		? 0xFFFFFFFFu : ((1u << SMOOTH_QUEUE_SIZE) - 1u));
	while (active) {
		uint8_t i = (uint8_t)__builtin_ctz(active);
		active &= active - 1; // clear lowest set bit
		queue_entry_t *e = &state.queue[i];

		if (e->frames_left == 0) {
			state.free_mask |= (1u << i);
			state.count--;
			continue;
		}

		// Use cached eased value from previous frame (avoids double evaluation)
		int32_t eased_now = e->eased_prev;
		int32_t next_progress = SMOOTH_FP_ONE -
			fp_mul(int_to_fp((int16_t)(e->frames_left - 1)), e->inv_total_fp);
		int32_t eased_next = ease_asymmetric(next_progress, overshoot);
		int32_t step_frac  = eased_next - eased_now;

		int32_t remaining_progress = SMOOTH_FP_ONE - eased_now;
		int32_t frame_dx, frame_dy;
		if (remaining_progress > 0) {
			frame_dx = fp_mul(e->x_remaining_fp,
				fp_div(step_frac, remaining_progress));
			frame_dy = fp_mul(e->y_remaining_fp,
				fp_div(step_frac, remaining_progress));
		} else {
			frame_dx = e->x_remaining_fp;
			frame_dy = e->y_remaining_fp;
		}

		e->x_remaining_fp -= frame_dx;
		e->y_remaining_fp -= frame_dy;
		e->eased_prev = eased_next; // cache for next frame
		e->frames_left--;

		if (e->frames_left == 0) {
			frame_dx += e->x_remaining_fp;
			frame_dy += e->y_remaining_fp;
			e->x_remaining_fp = 0;
			e->y_remaining_fp = 0;
			state.free_mask |= (1u << i);
			state.count--;
		}

		float entry_mag = fabsf((float)frame_dx) + fabsf((float)frame_dy);
		frame_speed_gain += e->speed_gain * entry_mag;
		total_weight += entry_mag;

		frame_x_fp += frame_dx;
		frame_y_fp += frame_dy;
		has_movement = true;
	}

	if (total_weight > 0.0f)
		frame_speed_gain /= total_weight;
	else
		frame_speed_gain = SMOOTH_SPEED_GAIN_DEFAULT;

	// ---- Humanization (skipped entirely when disabled) ----
	if (has_movement && state.humanize) {
		// Save base easing displacement — this must be conserved exactly
		int32_t base_x = frame_x_fp;
		int32_t base_y = frame_y_fp;

		// Advance EWMA noise channels (tick-rate-scaled alpha)
		state.speed_noise = state.ewma_alpha * state.speed_noise
			+ state.ewma_beta * smooth_rand_uniform();
		state.perp_noise  = state.ewma_alpha * state.perp_noise
			+ state.ewma_beta * smooth_rand_uniform();

		// Drift arc bias — O-U process, tau ~60s at 1kHz
		state.arc_bias += SMOOTH_ARC_DRIFT_RATE
			* (smooth_rand_uniform() * SMOOTH_ARC_DRIFT_INPUT
			   - state.arc_bias * SMOOTH_ARC_DRIFT_DECAY);

		// Compute noise components (separate from base displacement)
		float noise_x = 0.0f;
		float noise_y = 0.0f;

		// Fitts-calibrated speed perturbation (noise = base * scaled speed_mod)
		float speed_mod = state.speed_noise * SMOOTH_SPEED_NOISE_SCALE
			* frame_speed_gain;
		noise_x += (float)base_x * speed_mod;
		noise_y += (float)base_y * speed_mod;

		// Perpendicular wobble with arc bias and low-speed scaling
		float fx = (float)base_x;
		float fy = (float)base_y;
		float mag2 = fx * fx + fy * fy;
		if (mag2 > 1.0f) {
			float inv_mag = fast_invsqrt(mag2);
			float px = -fy * inv_mag;
			float py =  fx * inv_mag;

			float speed_px = mag2 * inv_mag * (1.0f / (float)SMOOTH_FP_ONE);
			float arc_scale = 1.0f + SMOOTH_LOWSPEED_BOOST / (speed_px + 1.0f);

			float perp_px = (state.arc_bias
				+ state.perp_noise * SMOOTH_PERP_AMPLITUDE) * arc_scale;
			noise_x += px * perp_px * (float)SMOOTH_FP_ONE;
			noise_y += py * perp_px * (float)SMOOTH_FP_ONE;
		}

		// Smooth only the noise for velocity continuity (no displacement leak)
		float smoothed_nx = SMOOTH_VELOCITY_ALPHA * noise_x
			+ (1.0f - SMOOTH_VELOCITY_ALPHA) * state.last_noise_vx;
		float smoothed_ny = SMOOTH_VELOCITY_ALPHA * noise_y
			+ (1.0f - SMOOTH_VELOCITY_ALPHA) * state.last_noise_vy;
		state.last_noise_vx = smoothed_nx;
		state.last_noise_vy = smoothed_ny;

		// Final output: exact base displacement + properly rounded noise
		frame_x_fp = base_x + (int32_t)(smoothed_nx + (smoothed_nx >= 0 ? 0.5f : -0.5f));
		frame_y_fp = base_y + (int32_t)(smoothed_ny + (smoothed_ny >= 0 ? 0.5f : -0.5f));
	} else {
		// Decay noise velocity: idle (no movement) or humanize disabled mid-movement
		state.last_noise_vx *= SMOOTH_VELOCITY_DECAY;
		state.last_noise_vy *= SMOOTH_VELOCITY_DECAY;
	}

	// Persistent micro-tremor: fills zero-movement gaps between commands
	// with Brownian-like random walk. Mean-reverting to prevent drift.
	// Critical for eliminating bimodal interval distribution.
	if (!has_movement && state.humanize) {
		state.tremor_x = state.tremor_x * SMOOTH_TREMOR_DECAY
			+ smooth_rand_uniform() * SMOOTH_TREMOR_STEP;
		state.tremor_y = state.tremor_y * SMOOTH_TREMOR_DECAY
			+ smooth_rand_uniform() * SMOOTH_TREMOR_STEP;
		frame_x_fp = (int32_t)(state.tremor_x * (float)SMOOTH_FP_ONE);
		frame_y_fp = (int32_t)(state.tremor_y * (float)SMOOTH_FP_ONE);
	}

	// Add frame contribution to sub-pixel accumulator
	state.x_accum_fp += frame_x_fp;
	state.y_accum_fp += frame_y_fp;

	// Dithered rounding: randomize threshold ±DITHER_RANGE around 0.5
	// to break up repeated identical integer deltas from smooth easing.
	// Unbiased on average (symmetric around 0.5), no displacement leak
	// since remainder stays in sub-pixel accumulator.
	int16_t ix, iy;
	if (state.humanize) {
		int32_t dx = (int32_t)(smooth_rand_uniform() * SMOOTH_DITHER_RANGE
			* (float)SMOOTH_FP_ONE);
		int32_t dy = (int32_t)(smooth_rand_uniform() * SMOOTH_DITHER_RANGE
			* (float)SMOOTH_FP_ONE);
		ix = (int16_t)((state.x_accum_fp + SMOOTH_FP_HALF + dx) >> SMOOTH_FP_SHIFT);
		iy = (int16_t)((state.y_accum_fp + SMOOTH_FP_HALF + dy) >> SMOOTH_FP_SHIFT);
	} else {
		ix = fp_to_int(state.x_accum_fp);
		iy = fp_to_int(state.y_accum_fp);
	}

	state.x_accum_fp -= int_to_fp(ix);
	state.y_accum_fp -= int_to_fp(iy);

	if (ix > state.max_per_frame) ix = state.max_per_frame;
	if (ix < -state.max_per_frame) ix = -state.max_per_frame;
	if (iy > state.max_per_frame) iy = state.max_per_frame;
	if (iy < -state.max_per_frame) iy = -state.max_per_frame;

	*out_x = ix;
	*out_y = iy;
}

bool smooth_has_pending(void)
{
	return state.count > 0 ||
	       state.x_accum_fp > SMOOTH_FP_HALF ||
	       state.x_accum_fp < -SMOOTH_FP_HALF ||
	       state.y_accum_fp > SMOOTH_FP_HALF ||
	       state.y_accum_fp < -SMOOTH_FP_HALF;
}

void smooth_clear(void)
{
	state.free_mask = (SMOOTH_QUEUE_SIZE == 32) ? 0xFFFFFFFFu
		: ((1u << SMOOTH_QUEUE_SIZE) - 1u);
	state.count = 0;
	state.x_accum_fp = 0;
	state.y_accum_fp = 0;
	state.last_noise_vx = 0.0f;
	state.last_noise_vy = 0.0f;
	state.tremor_x = 0.0f;
	state.tremor_y = 0.0f;
}

void smooth_set_max_per_frame(int16_t max)
{
	if (max < 1) max = 1;
	state.max_per_frame = max;
}

void smooth_set_humanize(bool enabled)
{
	state.humanize = enabled;
}

// ---- Timing humanization ----
// Called from PIT ISR context. Hardware TRNG for entropy with xorshift32
// fallback during the ~250µs TRNG refill window.

static inline uint32_t timing_lfsr_next(void)
{
	uint32_t x = state.timing_lfsr;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state.timing_lfsr = x;
	return x;
}

// Draw one 32-bit random word. Reads from cached 32-word TRNG pool;
// when exhausted, refills 16 words from hardware if ready, else falls back to LFSR.
static inline uint32_t timing_rand32(void)
{
	if (state.trng_idx < 32)
		return state.trng_buf[state.trng_idx++];

	// Cache exhausted — try to refill from TRNG hardware
	if (TRNG_MCTL & TRNG_MCTL_ENT_VAL) {
		volatile uint32_t *ent = &TRNG_ENT0;
		for (int i = 0; i < 16; i++)
			state.trng_buf[i] = ent[i];
		// Shift remaining 16 words down, fill top half
		// Actually: just wrap — refill first 16, consume from 0
		state.trng_idx = 1;
		return state.trng_buf[0];
	}

	// TRNG still generating — use LFSR until next refill
	return timing_lfsr_next();
}

// Uniform float in [-1, +1) from hardware entropy
static inline float timing_rand_uniform(void)
{
	int32_t balanced = (int32_t)(timing_rand32() >> 8) - 0x800000;
	return (float)balanced * (1.0f / 8388608.0f);
}

uint32_t smooth_timing_next(uint32_t base_ldval, bool *out_skip)
{
	*out_skip = false;

	if (!state.humanize) return base_ldval;

	uint32_t r = timing_rand32();

	// Poll skip: r uniform in [0, 2^32), skip if below threshold.
	if (r < (0xFFFFFFFFu / SMOOTH_TIMING_SKIP_PROB_INV)
	    && state.consec_skips < SMOOTH_TIMING_MAX_CONSEC_SKIP) {
		*out_skip = true;
		state.consec_skips++;
		return base_ldval;
	}
	state.consec_skips = 0;

	// LDVAL jitter: session bias + per-tick random offset.
	float offset = state.rate_bias
		+ timing_rand_uniform() * SMOOTH_TIMING_LDVAL_JITTER;
	float result = (float)base_ldval * (1.0f + offset);

	// Clamp to [50%, 200%] of base
	float lo = (float)base_ldval * 0.5f;
	float hi = (float)base_ldval * 2.0f;
	if (result < lo) result = lo;
	if (result > hi) result = hi;

	return (uint32_t)result;
}
