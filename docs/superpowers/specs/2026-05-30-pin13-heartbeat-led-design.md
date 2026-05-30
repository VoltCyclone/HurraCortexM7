# Pin-13 Heartbeat LED (QuadTimer2, UART-status rate) — Design

**Date:** 2026-05-30
**Status:** Approved
**Author:** Claude (with @ramsey)

## Summary

Drive the on-board LED on **pin 13** with a hardware timer so it blinks
autonomously with no CPU in the loop, and encode **UART link status in the blink
rate**. The CPU configures the timer once, then only rewrites the blink period
when the link *state changes* (a rare event) — the hardware produces the blink
itself. This replaces the current CPU-managed per-packet LED flicker and the
100 ms `led_pwm_set(brightness)` throughput indicator, removing that work from
the latency-critical proxy loop.

## Hardware background

Pin 13 = pad **GPIO_B0_03**. Contrary to the stale `// FlexPWM2` comment in the
current `main.c`, GPIO_B0_03 has **no FlexPWM route**; it maps to **QuadTimer2
channel 0** (`TMR2.CH[0]`, the `QTIMER2_TIMER0` function) via IOMUXC **ALT1**,
clock-gated by **CCM_CCGR6 CG14** (`CCM_CCGR6_QTIMER2`).

A single 16-bit QuadTimer channel at its slowest internal clock (IP-bus ÷128 ≈
1.17 MHz) bottoms out near ~18 Hz — too fast for a heartbeat. So we **cascade two
channels**:

- **TMR2 CH1** — free-running prescaler. Counts the IP-bus clock and rolls over to
  produce a steady tick (target **~1 kHz**) on its output.
- **TMR2 CH0** — the pin-13 output stage. Its primary count source is **CH1's
  output** (`TMR_CTRL_PCS` = counter-1 output), and it toggles its output every
  `COMP` ticks to make a 50%-duty square wave. The blink frequency is set purely
  by CH0's compare value, so changing the rate is a single compare-register update.

This is the same low-frequency square-wave-on-QuadTimer technique the Teensy core
`tone()` uses; the implementation mirrors that register sequence. No DMA, no ISR,
zero ongoing CPU.

QuadTimer2 is currently unused by the firmware (PIT0–3 and GPT2 are used
elsewhere), so there is no peripheral contention.

## Components

### New module: `src/led.c` / `src/led.h`
A focused unit that owns pin 13 across its lifecycle (today the LED helpers are
inline no-op stubs in `main.c`, which is already ~320 lines). `led.c` is a pure
hardware unit with **no knowledge of UART** — it exposes a blink-rate mechanism;
the status→rate *policy* lives in `main.c` where the UART counters are known.

```c
// led.h
#include <stdint.h>

void led_init(void);            // configure pin 13 as GPIO output, off
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms); // blocking fatal pattern

void led_heartbeat_start(void);          // hand pin 13 to QuadTimer2; start blinking (idle rate)
void led_heartbeat_set_rate(uint16_t centihz); // update blink rate, glitch-free; arg in 0.01 Hz units
```

- Rate is expressed in **centihertz** (0.01 Hz units) so sub-1 Hz rates (e.g. 0.5 Hz
  = `50`) are representable with an integer API.
- `led_heartbeat_set_rate` rewrites CH0's compare via the **CMPLD1/CMPLD2 preload +
  `TMR_CSCTRL_CL1/CL2`** so the change latches at the next compare boundary
  (glitch-free); it is a couple of 16-bit writes, safe to call from the main loop.

**Lifecycle / pad ownership:**
- **Boot + enumeration** — GPIO digital mode. `led_on/off/toggle` for status,
  `led_blink_forever` for fatal codes (5/8/9). (Driving pin 13 early is safe — the
  "9-blink ARM JTAG DAP Init Error" is a flash/boot-image fault, unrelated to LED
  use.)
- **Running** — `led_heartbeat_start()` switches the pad to ALT1 and starts the
  cascade at the idle rate.
- **Fatal after running** — `led_blink_forever` first stops the QuadTimer and
  restores the pad to GPIO, then blinks (so the post-enum fatal code-6 path works).

### UART status → blink rate (policy in `main.c`)
On a low-rate tick (~100 ms; the cadence currently used by the throughput
indicator), snapshot the kmbox counters and pick a state. Only call
`led_heartbeat_set_rate()` when the state **changes**.

| State  | Condition (since previous tick)                          | Rate    | centihz |
|--------|----------------------------------------------------------|---------|---------|
| ERROR  | `kmbox_uart_overrun()+framing()+noise()` increased        | ~6 Hz   | 600     |
| ACTIVE | `kmbox_rx_byte_count()` increased (command traffic)       | ~2 Hz   | 200     |
| IDLE   | otherwise (alive, no recent host traffic)                | ~0.5 Hz | 50      |

Priority ERROR > ACTIVE > IDLE. All three rates are reachable by the cascade with
a ~1 kHz CH1 tick (CH0 compare = `tick_hz / (2 * blink_hz) - 1`: 50→≈999, 200→≈249,
600→≈82; all < 65536).

### `main.c` edits
- `#include "led.h"`; delete the 7 inline LED stub functions.
- Call `led_init()` once early (before the first `led_on()`).
- Replace the post-enum `led_pwm_init()` call with `led_heartbeat_start()`.
- **Remove the per-packet LED flicker** from the hot loop: delete
  `led_on(); led_off_time = now + 2;`, the `led_off_time` flush, and the local
  `led_off_time`.
- **Repurpose the ~100 ms tick**: instead of computing `brightness` and calling
  `led_pwm_set`, compute the UART state (table above) and call
  `led_heartbeat_set_rate()` on change. Replace locals `led_pwm_update` /
  `led_report_snapshot` with the status snapshot/state the new logic needs.
- Delete `led_pwm_set` (no longer used; the heartbeat is rate-coded, not
  brightness-coded). Per-packet activity remains visible on the external
  LINK/STATE/STATUS diag LEDs.

### Makefile
- Add `src/led.c` to `SRC` (not `HOT_SRC` — setup-only, not hot path). `led.c` is
  protocol-independent; both `make` and `make PROTOCOL=ferrum` must still link.

## Data flow

```
boot → led_init() (GPIO, off) → [enum: led_on/off/toggle, fatal: led_blink_forever]
     → led_heartbeat_start() (pad→ALT1, QuadTimer2 cascade runs at idle rate)

main loop ~100ms tick: snapshot kmbox counters → derive {IDLE,ACTIVE,ERROR}
     → if state changed: led_heartbeat_set_rate(rate)   // single glitch-free compare update
hardware: TMR2 CH1 (÷ → ~1kHz) → TMR2 CH0 (toggle every COMP) → pin 13   // autonomous
```

## Testing / verification

- `make` and `make PROTOCOL=ferrum` both link clean (size report). `led.c` builds
  under both protocols.
- Confirm QuadTimer bit macros used (`TMR_CTRL_CM/PCS/OUTMODE`, `TMR_SCTRL_OEN/OPS`,
  `TMR_CSCTRL_CL1/CL2`, `CCM_CCGR6_QTIMER2`) resolve from `imxrt.h` (verified present).
- Hardware (manual, needs device): after enumeration, pin-13 blinks ~0.5 Hz at
  idle; sending Hurra/Ferrum commands speeds it to ~2 Hz; inducing UART errors
  (e.g. baud mismatch) pushes it to ~6 Hz. Blinking continues unchanged while the
  proxy loop is fully loaded — proving it is hardware-driven.

## Risks & mitigations

- **Cascade register sequence is the fiddly part** — mirror the proven Teensy
  `tone()` QuadTimer cascade + output-toggle sequence; verify on hardware.
- **Pad ownership transitions (GPIO ↔ ALT1)** — `led_blink_forever` reclaims the
  pad before blinking; `led_heartbeat_start` switches to ALT1.
- **Rate-change glitches** — use CMPLD preload (`CL1/CL2`) so compare updates latch
  at a period boundary rather than mid-count.

## Non-goals

- Breathing/fading (would need eDMA) — explicitly out; a square-wave blink is the
  chosen behavior.
- Brightness control / `led_pwm_set` — removed; rate, not brightness, carries
  status.
- Touching the external LINK/STATE/STATUS diag LEDs in `kmbox.c` — unchanged.
