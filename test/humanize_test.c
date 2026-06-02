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

    /* (A) Conservation: summed output == summed injected, within rounding. */
    humanize_init(1000);
    humanize_set_level(2);
    long sx = 0;
    for (int i = 0; i < 5000; i++) {           /* steady 3 px/frame stream */
        int16_t dx = 3, dy = 0;
        humanize_filter(&dx, &dy);
        sx += dx;
    }
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

    printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
