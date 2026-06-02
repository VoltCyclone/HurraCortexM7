#include "humanize.h"
#include <string.h>
#include <math.h>

/* ── tunables ───────────────────────────────────────────────────────── */
#define HZ_DEFAULT_LEVEL   2        /* boot default: on, "normal" */
#define HZ_MAX_PER_FRAME   127      /* human per-frame ceiling (counts) */
#define HZ_IDLE_EPS        0.01f    /* |owed| below this = settled */
#define HZ_TIMING_JITTER   0.12f    /* +/- fraction of base period to jitter */
#define HZ_TIMING_FLOOR    0.80f    /* min multiple of base period */
#define HZ_TIMING_CEIL     1.20f    /* max multiple of base period */

/* Per-level presets: drain rate k (fraction of owed emitted per frame),
 * and perpendicular-noise amplitude. Level 0 = off. */
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
    /* Only S.a is guarded: the 16-iteration warm-up below guarantees nonzero
     * liveness even if b/c happen to seed to zero. */
    for (int i = 0; i < 16; i++) sfc32();
    S.timing_lfsr = sfc32() | 1u;
    S.ewma = 0.85f;
    humanize_set_level(HZ_DEFAULT_LEVEL);
    S.noise_amp *= 1.0f + 0.15f * sfc32_uniform();
    (void)interval_us;  /* intentionally unused: drain rate is level-preset, not interval-scaled */
}

static int16_t drain_axis(float *owed, float *res, float emit_v, float noise) {
    float want = emit_v + noise + *res;
    /* cap to human ceiling */
    float capped = want;
    if (capped >  (float)HZ_MAX_PER_FRAME) capped =  (float)HZ_MAX_PER_FRAME;
    if (capped < -(float)HZ_MAX_PER_FRAME) capped = -(float)HZ_MAX_PER_FRAME;
    float res_in = *res;                     /* residual before this frame's update */
    int16_t out = (int16_t)(capped >= 0 ? (capped + 0.5f) : (capped - 0.5f));
    *res = capped - (float)out;
    /* Drain owed by exactly the integer output; sub-pixel residual is tracked
     * separately in *res so it is not lost.  Noise is zero-mean so we don't
     * track it in owed — but when the cap fires we must not lose signal.
     * Strategy: drain emit_v from owed unconditionally; if the cap cut into
     * the *signal* portion (emit_v), add that cut back so owed carries it. */
    float signal_cut = 0.0f;
    float uncapped_signal = emit_v + res_in; /* signal + pre-update dither, no noise */
    if (uncapped_signal > (float)HZ_MAX_PER_FRAME)
        signal_cut = uncapped_signal - (float)HZ_MAX_PER_FRAME;
    else if (uncapped_signal < -(float)HZ_MAX_PER_FRAME)
        signal_cut = uncapped_signal - (-(float)HZ_MAX_PER_FRAME);
    *owed -= emit_v - signal_cut;
    return out;
}

void humanize_filter(int16_t *dx, int16_t *dy) {
    if (S.level == 0) return;                 /* off: passthrough */

    S.owed_x += (float)*dx;
    S.owed_y += (float)*dy;

    if (fabsf(S.owed_x) < HZ_IDLE_EPS && fabsf(S.owed_y) < HZ_IDLE_EPS &&
        fabsf(S.res_x) < 0.5f && fabsf(S.res_y) < 0.5f) {
        if (S.idle < 1000) S.idle++;
        *dx = 0; *dy = 0;
        return;
    }
    S.idle = 0;

    float ex = S.owed_x * S.drain;
    float ey = S.owed_y * S.drain;

    float speed = sqrtf(ex*ex + ey*ey);
    S.n_perp = S.ewma * S.n_perp + (1.0f - S.ewma) * sfc32_uniform();
    float nmag = S.n_perp * S.noise_amp * speed;
    float nx = 0.0f, ny = 0.0f;
    if (speed > 1e-3f) { nx = -ey / speed * nmag; ny = ex / speed * nmag; }

    *dx = drain_axis(&S.owed_x, &S.res_x, ex, nx);
    *dy = drain_axis(&S.owed_y, &S.res_y, ey, ny);
}

bool humanize_pending(void) {
    return fabsf(S.owed_x) >= HZ_IDLE_EPS || fabsf(S.owed_y) >= HZ_IDLE_EPS;
}

uint32_t humanize_timing_next(uint32_t base_ldval) {
    if (S.level == 0) return base_ldval;
    S.timing_lfsr ^= S.timing_lfsr << 13;
    S.timing_lfsr ^= S.timing_lfsr >> 17;
    S.timing_lfsr ^= S.timing_lfsr << 5;
    float u = (float)(S.timing_lfsr >> 8) * (1.0f / 16777216.0f) - 0.5f;
    float r = (float)base_ldval * (1.0f + HZ_TIMING_JITTER * u);
    float lo = (float)base_ldval * HZ_TIMING_FLOOR, hi = (float)base_ldval * HZ_TIMING_CEIL;
    if (r < lo) r = lo; if (r > hi) r = hi;
    return (uint32_t)r;
}
