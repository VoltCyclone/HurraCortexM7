// Smooth injection queue for Cortex-M7
// Simple circular queue + asymmetric easing + EWMA correlated noise.
// Tick-rate independent: spread frames scale with interval_us.
// Humanization: two EWMA noise channels (speed + perpendicular wobble)
// directly calibrated to observation-level scoring metrics.

#include "smooth.h"
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
	return (int32_t)((float)a / (float)b * (float)SMOOTH_FP_ONE);
}

// ---- Asymmetric easing (inline, no LUT) ----
// Models natural movement: quadratic attack → cubic deceleration.
// Monotonic [0, FP_ONE], no overshoot, no clamping artifacts.
// Delivers ~60% in first 30% of time (fast attack), then long settle tail.
static inline int32_t ease_asymmetric(int32_t t)
{
	float tf = (float)t * (1.0f / (float)SMOOTH_FP_ONE);

	float result;
	if (tf < 0.30f) {
		// Quadratic attack: fast ramp to 60% completion
		float n = tf * (1.0f / 0.30f);  // [0,1]
		result = n * n * 0.60f;
	} else {
		// Cubic ease-out: gradual deceleration from 60% to 100%
		float n = (tf - 0.30f) * (1.0f / 0.70f);  // [0,1]
		float omt = 1.0f - n;
		result = 0.60f + 0.40f * (1.0f - omt * omt * omt);
	}

	return (int32_t)(result * (float)SMOOTH_FP_ONE);
}

// ---- Queue entry ----
typedef struct {
	int32_t x_remaining_fp;
	int32_t y_remaining_fp;
	int32_t inv_total_fp;    // precomputed FP_ONE / total_frames
	uint8_t frames_left;
	uint8_t total_frames;
	bool    active;
} queue_entry_t;

// ---- State ----
static struct {
	int32_t x_accum_fp;
	int32_t y_accum_fp;
	queue_entry_t queue[SMOOTH_QUEUE_SIZE];
	uint8_t head;
	uint8_t count;
	int16_t max_per_frame;
	float   dt;              // seconds per tick
	uint32_t interval_us;    // microseconds per tick
	uint32_t lfsr;           // LFSR for per-frame noise + jitter
	// EWMA correlated noise channels (alpha=0.97, ~33ms correlation)
	float   speed_noise;     // filtered speed perturbation
	float   perp_noise;      // filtered perpendicular wobble
} state;

// ---- LFSR: uniform [-1.0, 1.0] ----
static inline float smooth_lfsr_uniform(void)
{
	state.lfsr ^= state.lfsr << 13;
	state.lfsr ^= state.lfsr >> 17;
	state.lfsr ^= state.lfsr << 5;
	int32_t balanced = (int32_t)(state.lfsr >> 8) - 0x800000;
	return (float)balanced * (1.0f / 8388608.0f);
}

// ---- Frame count heuristic (integer-only) ----
// Returns frame count scaled to actual tick rate so spread duration
// in wall-clock time is constant regardless of poll rate.
// Durations tuned for the asymmetric easing curve to fully express:
//   small (<20px): 16ms — fine corrections need visible easing
//   medium (20-60px): 12ms — overshoot/settle has room to play
//   large (60-120px): 8ms — ballistic with brief settle
//   very large (>120px): 5ms — fast flick, minimal easing
static uint8_t compute_spread_frames(int32_t abs_x_fp, int32_t abs_y_fp)
{
	int32_t max_comp = abs_x_fp > abs_y_fp ? abs_x_fp : abs_y_fp;
	int32_t px = max_comp >> SMOOTH_FP_SHIFT;

	uint32_t spread_us;
	if (px < 20)       spread_us = 16000;
	else if (px < 60)  spread_us = 12000;
	else if (px < 120) spread_us = 8000;
	else               spread_us = 5000;

	// ±15% jitter on spread duration — temporal variation
	float jitter = smooth_lfsr_uniform() * 0.15f;
	spread_us = (uint32_t)((float)spread_us * (1.0f + jitter));

	uint32_t frames = (spread_us + (state.interval_us >> 1)) / state.interval_us;
	if (frames < 3) frames = 3;
	if (frames > 128) frames = 128;
	return (uint8_t)frames;
}

// ---- Public API ----

void smooth_init(uint32_t interval_us)
{
	memset(&state, 0, sizeof(state));
	state.max_per_frame = 127;
	state.interval_us = interval_us > 0 ? interval_us : 1000;
	state.dt = (float)state.interval_us / 1000000.0f;

	// Enable DWT cycle counter for LFSR seeding
	volatile uint32_t *dwt_ctrl   = (volatile uint32_t *)0xE0001000;
	volatile uint32_t *dwt_cyccnt = (volatile uint32_t *)0xE0001004;
	*dwt_ctrl |= 1;

	state.lfsr = *dwt_cyccnt ^ 0xCAFEBABE;
	if (state.lfsr == 0) state.lfsr = 0xCAFEBABE;

	// Prime EWMA noise channels with a few random steps
	for (int i = 0; i < 20; i++) {
		state.speed_noise = 0.97f * state.speed_noise + 0.03f * smooth_lfsr_uniform();
		state.perp_noise  = 0.97f * state.perp_noise  + 0.03f * smooth_lfsr_uniform();
	}
}

void smooth_inject(int16_t x, int16_t y)
{
	if (x == 0 && y == 0) return;

	// Find a free slot
	if (state.count >= SMOOTH_QUEUE_SIZE) {
		// Queue full — fall back to immediate injection
		state.x_accum_fp += int_to_fp(x);
		state.y_accum_fp += int_to_fp(y);
		return;
	}

	// Scan for an inactive slot
	uint8_t slot = 0;
	for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE; i++) {
		if (!state.queue[i].active) {
			slot = i;
			break;
		}
	}

	int32_t xfp = int_to_fp(x);
	int32_t yfp = int_to_fp(y);
	int32_t ax = xfp >= 0 ? xfp : -xfp;
	int32_t ay = yfp >= 0 ? yfp : -yfp;

	uint8_t frames = compute_spread_frames(ax, ay);

	state.queue[slot].x_remaining_fp = xfp;
	state.queue[slot].y_remaining_fp = yfp;
	state.queue[slot].inv_total_fp   = SMOOTH_FP_ONE / (int32_t)frames;
	state.queue[slot].frames_left    = frames;
	state.queue[slot].total_frames   = frames;
	state.queue[slot].active         = true;
	state.count++;
}

void smooth_process_frame(int16_t *out_x, int16_t *out_y)
{
	// Track this frame's queue contribution separately from sub-pixel remainder
	int32_t frame_x_fp = 0;
	int32_t frame_y_fp = 0;
	float movement_mag = 0.0f;

	uint8_t remaining = state.count;
	if (remaining > 0) {
		for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE && remaining > 0; i++) {
			queue_entry_t *e = &state.queue[i];
			if (!e->active) continue;
			if (e->frames_left == 0) {
				e->active = false;
				state.count--;
				remaining--;
				continue;
			}

			// Compute eased progress step using precomputed reciprocal
			// progress = 1 - frames_left * inv_total
			int32_t progress = SMOOTH_FP_ONE -
				fp_mul(int_to_fp((int16_t)e->frames_left), e->inv_total_fp);
			int32_t next_progress = SMOOTH_FP_ONE -
				fp_mul(int_to_fp((int16_t)(e->frames_left - 1)), e->inv_total_fp);

			int32_t eased_now  = ease_asymmetric(progress);
			int32_t eased_next = ease_asymmetric(next_progress);
			int32_t step_frac  = eased_next - eased_now;

			// Original total was remaining / (remaining progress)
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
			e->frames_left--;

			// On last frame, flush any rounding remainder
			if (e->frames_left == 0) {
				frame_dx += e->x_remaining_fp;
				frame_dy += e->y_remaining_fp;
				e->x_remaining_fp = 0;
				e->y_remaining_fp = 0;
				e->active = false;
				state.count--;
				remaining--;
			}

			// Accumulate into frame contribution
			frame_x_fp += frame_dx;
			frame_y_fp += frame_dy;
			movement_mag += fabsf((float)frame_dx) + fabsf((float)frame_dy);
		}
	}

	// ---- EWMA correlated noise humanization ----
	// Only active when smooth queue is producing movement.
	// Two channels: speed (multiplicative) + perpendicular (additive wobble).
	// Alpha=0.97 at 1kHz -> ~33ms correlation time (30Hz bandwidth).
	if (movement_mag > 0.0f) {
		state.speed_noise = 0.97f * state.speed_noise + 0.03f * smooth_lfsr_uniform();
		state.perp_noise  = 0.97f * state.perp_noise  + 0.03f * smooth_lfsr_uniform();

		// Speed perturbation: multiplicative, targets observed speed_cv ~0.30-0.40
		// EWMA std ~0.071 * K_SPEED=1.2 -> ~8.5% frame-level speed variation.
		// Easing curve's own accel/decel adds ~0.15-0.20 more to observed CV.
		float speed_mod = 1.0f + state.speed_noise * 1.2f;
		frame_x_fp = (int32_t)((float)frame_x_fp * speed_mod);
		frame_y_fp = (int32_t)((float)frame_y_fp * speed_mod);

		// Perpendicular wobble: additive, targets perp_scatter ~0.25 px
		float fx = (float)frame_x_fp;
		float fy = (float)frame_y_fp;
		float mag2 = fx * fx + fy * fy;
		if (mag2 > 1.0f) {
			float inv_mag = 1.0f / sqrtf(mag2);
			// Perpendicular direction (90-degree rotation of movement)
			float px = -fy * inv_mag;
			float py =  fx * inv_mag;
			float perp_px = state.perp_noise * 0.20f;
			frame_x_fp += (int32_t)(px * perp_px * (float)SMOOTH_FP_ONE);
			frame_y_fp += (int32_t)(py * perp_px * (float)SMOOTH_FP_ONE);
		}
	}

	// Add frame contribution to sub-pixel accumulator
	state.x_accum_fp += frame_x_fp;
	state.y_accum_fp += frame_y_fp;

	// Extract integer part, keep fractional remainder
	int16_t ix = fp_to_int(state.x_accum_fp);
	int16_t iy = fp_to_int(state.y_accum_fp);

	state.x_accum_fp -= int_to_fp(ix);
	state.y_accum_fp -= int_to_fp(iy);

	// Clamp to max_per_frame
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
	for (uint8_t i = 0; i < SMOOTH_QUEUE_SIZE; i++)
		state.queue[i].active = false;
	state.count = 0;
	state.x_accum_fp = 0;
	state.y_accum_fp = 0;
}

void smooth_set_max_per_frame(int16_t max)
{
	if (max < 1) max = 1;
	state.max_per_frame = max;
}
