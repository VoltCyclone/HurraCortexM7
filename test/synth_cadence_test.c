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
