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
