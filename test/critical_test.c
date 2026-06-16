#include <stdio.h>
#include <stdint.h>
#define HOSTTEST
#include "critical.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

/* On host these are no-ops; the test asserts the *shape* of the API (nesting
 * compiles and returns a token) so the target build can't drift the signature.
 * Hardware reentrancy is validated on-target in a later task, not here. */
int main(void) {
    uint32_t outer = crit_enter();
    uint32_t inner = crit_enter();
    crit_exit(inner);
    crit_exit(outer);
    CHECK(1, "nested enter/exit compiles and links");
    if (failures) { printf("%d FAILURES\n", failures); return 1; }
    printf("critical: all passed\n");
    return 0;
}
