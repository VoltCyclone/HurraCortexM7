# Pin-13 Heartbeat LED Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive pin 13 with an autonomous QuadTimer2 hardware blink whose rate encodes UART status (idle/active/error), replacing the CPU-managed per-packet LED work.

**Architecture:** A focused `src/led.c`/`src/led.h` module owns pin 13. Boot/enumeration uses GPIO7[3]; the running phase hands the pad to a TMR2 cascade (CH1 prescaler → CH0 output, IOMUXC ALT1) that blinks with zero CPU. `main.c` samples kmbox counters every ~100 ms and rewrites the QuadTimer compare (glitch-free) only when the link state changes.

**Tech Stack:** C (bare-metal Cortex-M7, ARM GCC), NXP i.MX RT1062 QuadTimer2 (TMR2) + IOMUXC + GPIO7.

**Testing note:** Bare-metal firmware, no unit harness. "The test" per task is: a standalone cross-compile of `led.c`, then the full `make` / `make PROTOCOL=ferrum` link, plus grep gates. The absolute blink *rate* can only be confirmed on hardware (Task 4) — but the design makes the idle/active/error *ratios* exact regardless of the true IP-bus clock, so only one calibration constant (`LED_HB_K`) might need a hardware tweak.

**Reference:** QuadTimer cascade facts (PCS `0b0001` = clocked by counter-1 output; `OUTMODE(6)` = toggle/50%; `CSCTRL CL1/CL2` = glitch-free `CMPLD`→`COMP` latch; `CCM_CCGR6_QTIMER2` gate) are from the i.MX RT1060 RM ch.16 + Teensy `pwm.c` idioms. Pin 13 = `GPIO_B0_03`, ALT1 = `QTIMER2_TIMER0` = TMR2 CH0; `core/startup.c:382-385` already routes the pad to fast GPIO7[3].

All commands assume CWD `/Users/ramseymcgrath/code/Hurra-v2` on branch `feat/pin13-heartbeat-led` (already created). Toolchain prefix (from `Makefile`): `~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-gcc`.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `src/led.h` | LED module public API | Create |
| `src/led.c` | GPIO7 status LED + QuadTimer2 heartbeat (rate = UART status) | Create |
| `Makefile` | Add `src/led.c` to the build | Modify |
| `src/main.c` | Use the module; remove CPU-managed LED work; status→rate tick | Modify |

---

## Task 1: Create the `led` module (`led.h` + `led.c`)

**Files:**
- Create: `src/led.h`, `src/led.c`

- [ ] **Step 1: Write `src/led.h`**

```c
// src/led.h — on-board LED (pin 13) driver.
// Two modes: GPIO7[3] for boot/enumeration/fatal codes, and an autonomous
// QuadTimer2 heartbeat (running) whose blink rate encodes UART status.
#pragma once
#include <stdint.h>

void led_init(void);     // assert pin 13 as GPIO7 output, off
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms); // blocking fatal

void led_heartbeat_start(void);                 // start autonomous blink at idle rate
void led_heartbeat_set_rate(uint16_t centihz);  // blink rate in 0.01 Hz units; glitch-free
```

- [ ] **Step 2: Write `src/led.c`**

```c
// src/led.c — on-board LED (pin 13) driver.
//
// Pin 13 = pad GPIO_B0_03. core/startup.c routes the B0 bank to fast GPIO7
// (GPR27) and configures pin 13 as GPIO7[3] output; the HardFault handler
// drives it directly. This module owns pin 13 in two modes:
//   * GPIO  (boot / enumeration / fatal codes): GPIO7[3] set/clear/toggle.
//   * Heartbeat (running): QuadTimer2 cascade (CH1 prescaler -> CH0 output),
//     pad muxed to ALT1 = QTIMER2_TIMER0. Autonomous; the blink RATE encodes
//     UART status. Zero CPU once started.
#include "led.h"
#include "imxrt.h"
#include <stdbool.h>

extern void delay(uint32_t msec);

#define LED_GPIO_BIT      (1u << 3)   // GPIO7[3]
#define LED_PAD_ALT_GPIO  5           // ALT5 = GPIO
#define LED_PAD_ALT_QTMR  1           // ALT1 = QTIMER2_TIMER0 (TMR2 CH0)

// --- QuadTimer cascade tuning -------------------------------------------------
// CH1 divides the IP-bus clock (PCS = IP/128) and toggles its OFLAG every
// (LED_CH1_COMP+1) counts, producing the clock for CH0. CH0 toggles pin 13
// every (COMP0+1) of those -> 50% square wave. blink_hz is inversely
// proportional to (COMP0+1), so the RATE RATIOS between states are exact
// regardless of the true IP-bus clock; only the absolute scale (LED_HB_K)
// depends on it. If the measured idle blink isn't ~0.5 Hz, scale LED_HB_K by
// the observed ratio and the active/error rates track automatically.
#define LED_CH1_COMP    1464u         // CH1 modulo (period = COMP+1)
#define LED_HB_K        20000u        // COMP0 = LED_HB_K/centihz - 1 (nominal IP=150MHz)
#define LED_CENTIHZ_MIN 5u            // clamp floor (0.05 Hz)

static bool s_hb_active;

static uint16_t centihz_to_comp(uint16_t centihz)
{
	if (centihz < LED_CENTIHZ_MIN) centihz = LED_CENTIHZ_MIN;
	uint32_t comp = LED_HB_K / centihz;
	if (comp == 0) comp = 1;
	comp -= 1u;
	if (comp > 0xFFFFu) comp = 0xFFFFu;
	return (uint16_t)comp;
}

void led_init(void)
{
	// startup.c already set ALT5 + GPR27 (fast GPIO7) + GDIR; re-assert + off.
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_GPIO;
	GPIO7_GDIR |= LED_GPIO_BIT;
	GPIO7_DR_CLEAR = LED_GPIO_BIT;
	s_hb_active = false;
}

void led_on(void)     { GPIO7_DR_SET    = LED_GPIO_BIT; }
void led_off(void)    { GPIO7_DR_CLEAR  = LED_GPIO_BIT; }
void led_toggle(void) { GPIO7_DR_TOGGLE = LED_GPIO_BIT; }

// Return pin 13 to GPIO7 output, stopping the heartbeat QuadTimer if it had
// taken over the pad. Lets fatal-code blinking work after the heartbeat began.
static void led_reclaim_gpio(void)
{
	if (s_hb_active) {
		IMXRT_TMR2.CH[0].CTRL = 0;   // stop output channel
		IMXRT_TMR2.CH[1].CTRL = 0;   // stop prescaler
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_GPIO;
		GPIO7_GDIR |= LED_GPIO_BIT;
		s_hb_active = false;
	}
}

void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	led_reclaim_gpio();
	if (code == 0) code = 1;
	for (;;) {
		for (uint8_t i = 0; i < code; i++) {
			GPIO7_DR_SET   = LED_GPIO_BIT; delay(on_ms);
			GPIO7_DR_CLEAR = LED_GPIO_BIT; delay(off_ms);
		}
		delay(700);   // gap between repeats of the code
	}
}

void led_heartbeat_set_rate(uint16_t centihz)
{
	if (!s_hb_active) return;
	uint16_t comp = centihz_to_comp(centihz);
	// Glitch-free: write the preload regs; CSCTRL CL1/CL2=1 latches COMP at the
	// next compare boundary instead of mid-count.
	IMXRT_TMR2.CH[0].CMPLD1 = comp;
	IMXRT_TMR2.CH[0].CMPLD2 = comp;
}

void led_heartbeat_start(void)
{
	CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);

	// CH1: prescaler. IP-bus/128, toggle OFLAG every (COMP+1) -> CH0 clock.
	IMXRT_TMR2.CH[1].CTRL   = 0;
	IMXRT_TMR2.CH[1].CNTR   = 0;
	IMXRT_TMR2.CH[1].LOAD   = 0;
	IMXRT_TMR2.CH[1].COMP1  = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].COMP2  = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CMPLD1 = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CMPLD2 = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CSCTRL = TMR_CSCTRL_CL1(1) | TMR_CSCTRL_CL2(1);
	IMXRT_TMR2.CH[1].SCTRL  = 0;
	IMXRT_TMR2.CH[1].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xF) |
	                          TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);

	// CH0: pin-13 output. Clocked by CH1 OFLAG (PCS=0b0001 = counter-1 output),
	// toggle OFLAG every (COMP0+1) -> 50% square; OEN drives the pad.
	uint16_t comp0 = centihz_to_comp(50);   // start at idle ~0.5 Hz
	IMXRT_TMR2.CH[0].CTRL   = 0;
	IMXRT_TMR2.CH[0].CNTR   = 0;
	IMXRT_TMR2.CH[0].LOAD   = 0;
	IMXRT_TMR2.CH[0].COMP1  = comp0;
	IMXRT_TMR2.CH[0].COMP2  = comp0;
	IMXRT_TMR2.CH[0].CMPLD1 = comp0;
	IMXRT_TMR2.CH[0].CMPLD2 = comp0;
	IMXRT_TMR2.CH[0].CSCTRL = TMR_CSCTRL_CL1(1) | TMR_CSCTRL_CL2(1);
	IMXRT_TMR2.CH[0].SCTRL  = TMR_SCTRL_OEN;
	IMXRT_TMR2.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1) |
	                          TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);

	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_QTMR; // hand pad to QuadTimer
	s_hb_active = true;
}
```

- [ ] **Step 3: Standalone compile check (Hurra build flags)**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
TC=~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-gcc
$TC -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb \
  -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=816000000 -DCMD_BAUD=115200 -DPROTOCOL_HURRA \
  -Iinclude -Isrc -Isrc/third_party/TinyFrame -Os -Wall \
  -c src/led.c -o /tmp/led.o && echo "led.c compiles OK"
```
Expected: `led.c compiles OK`, no errors. (If a `TMR_*` / `GPIO7_*` / `IOMUXC_*` macro is reported missing, it exists in `include/imxrt.h` — recheck the name; all used here were verified present.)

- [ ] **Step 4: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add src/led.h src/led.c
git commit -m "feat(led): pin-13 QuadTimer2 heartbeat + GPIO7 status module

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Add `led.c` to the build

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Add `src/led.c` to `SRC`**

In `Makefile`, change the `SRC` list from:
```make
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/actions.c \
      $(PROTO_SRC)
```
to (add `src/led.c`):
```make
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/actions.c src/led.c \
      $(PROTO_SRC)
```
Do NOT add it to `HOT_SRC` — `led.c` is setup-only, not hot path.

- [ ] **Step 2: Build both variants (led.c compiled in; still unused until Task 3)**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
make clean && make 2>&1 | tail -5
make clean && make PROTOCOL=ferrum 2>&1 | tail -4
make clean && make 2>&1 | tail -3   # leave on default Hurra build
```
Expected: all link clean with `size` reports. (`led.c`'s symbols may be GC'd as unused at this stage — that's fine.)

- [ ] **Step 3: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add Makefile
git commit -m "build: compile src/led.c

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Wire `main.c` to the module; remove CPU-managed LED work

**Files:**
- Modify: `src/main.c`

- [ ] **Step 1: Include the module and delete the inline LED stubs**

In `src/main.c`, find the includes near the top and add after the existing `#include`s:
```c
#include "led.h"
```
Then DELETE this entire inline stub block (the comment plus the seven stub functions — `led_pwm_init`, `led_pwm_set`, `led_on`, `led_off`, `led_toggle` (decl+def), `led_blink_forever`, and `led_stage` (decl+def)). Keep the `extern void delay(uint32_t msec);` line above it. The block to delete begins at the comment `// LED helpers — no-ops on the kmbox.` and ends at the `led_stage` definition `static void led_stage(uint8_t n) { (void)n; }`.

- [ ] **Step 2: Call `led_init()` early**

In `main()`, immediately after the existing `kmbox_init();` call, add:
```c
	led_init();
```

- [ ] **Step 3: Start the heartbeat after enumeration (rename the call)**

Find the post-enumeration call `led_pwm_init();` (right after the `while (!usb_device_is_configured())` loop) and change it to:
```c
	led_heartbeat_start();
```

- [ ] **Step 4: Replace the LED loop locals**

Find these three locals before the main `while (1)` loop:
```c
	uint32_t led_off_time = 0; // non-blocking LED pulse
	uint32_t led_pwm_update = millis();
	uint32_t led_report_snapshot = 0;
```
Replace all three with:
```c
	uint32_t led_status_tick = millis();
	uint32_t led_rx_snapshot = 0;
	uint32_t led_err_snapshot = 0;
	uint16_t led_centihz = 0;
```

- [ ] **Step 5: Remove the per-packet LED pulse (two spots)**

Inside the report-forwarding loop, find and DELETE these two lines (they sit right after the `if (sent) { report_count++; } else { drop_count++; }` block):
```c
				led_on();
				led_off_time = now + 2;
```
Then find and DELETE the pulse-off block that sits just before `kmbox_send_pending();`:
```c
		if (led_off_time && now >= led_off_time) {
			led_off();
			led_off_time = 0;
		}
```

- [ ] **Step 6: Replace the brightness tick with a UART-status → rate tick**

Find the 100 ms brightness block:
```c
		if ((now - led_pwm_update) >= 100) {
			uint32_t delta = report_count - led_report_snapshot;
			led_report_snapshot = report_count;
			uint32_t brightness = delta * 10 * 255 / 1000;
			if (brightness > 255) brightness = 255;
			led_pwm_set((uint8_t)brightness);
			led_pwm_update = now;
		}
```
Replace it entirely with:
```c
		// Heartbeat rate reflects UART status. Sampled every 100 ms; the
		// QuadTimer keeps blinking on its own — we only rewrite its compare
		// (glitch-free) when the state actually changes.
		if ((now - led_status_tick) >= 100) {
			led_status_tick = now;
			uint32_t rx  = kmbox_rx_byte_count();
			uint32_t err = kmbox_uart_overrun() + kmbox_uart_framing() +
			               kmbox_uart_noise();
			uint16_t centihz;
			if (err != led_err_snapshot)      centihz = 600; // ERROR  ~6 Hz
			else if (rx != led_rx_snapshot)   centihz = 200; // ACTIVE ~2 Hz
			else                              centihz = 50;  // IDLE   ~0.5 Hz
			led_err_snapshot = err;
			led_rx_snapshot  = rx;
			if (centihz != led_centihz) {
				led_centihz = centihz;
				led_heartbeat_set_rate(centihz);
			}
		}
```

- [ ] **Step 7: Confirm the kmbox accessors are declared**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -E "kmbox_rx_byte_count|kmbox_uart_overrun|kmbox_uart_framing|kmbox_uart_noise" src/kmbox.h
```
Expected: all four declared. If any is missing from `src/kmbox.h`, add its prototype there (signature `uint32_t name(void);`) next to the other `kmbox_*` accessor declarations.

- [ ] **Step 8: Build both variants and grep for leftover references**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -nE "led_pwm_init|led_pwm_set|led_off_time|led_pwm_update|led_report_snapshot|led_stage" src/main.c || echo "no stale LED refs"
make clean && make 2>&1 | tail -6
make clean && make PROTOCOL=ferrum 2>&1 | tail -4
make clean && make 2>&1 | tail -3   # leave on default Hurra build
```
Expected: `no stale LED refs`; both variants link clean with `size` reports. (A `-Wunused-but-set-variable` warning on `report_count`/`drop_count` is acceptable — no `-Werror`; do not remove those counters, they are diagnostics.)

- [ ] **Step 9: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add src/main.c src/kmbox.h
git commit -m "main: pin-13 heartbeat via led module; drop CPU-managed LED work

Replace per-packet LED flicker + 100ms brightness PWM with a UART-status
-> blink-rate tick driving the autonomous QuadTimer2 heartbeat.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Final verification gate (+ hardware calibration)

**Files:** none (verification only)

- [ ] **Step 1: Both variants build clean from scratch**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
make clean && make PROTOCOL=ferrum 2>&1 | tail -4
make clean && make 2>&1 | tail -6
```
Expected: both end with a `size` report, no errors; tree left on default Hurra build with `firmware.hex` present.

- [ ] **Step 2: Module boundary check**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -nE "IMXRT_TMR2|GPIO7_DR|QTIMER|FlexPWM|FLEXPWM" src/main.c || echo "main.c has no LED hardware pokes (all in led.c)"
```
Expected: `main.c has no LED hardware pokes (all in led.c)` — the hardware lives only in `led.c`.

- [ ] **Step 3: Hardware test + rate calibration (manual, requires the device)**

Flash (`make flash`) and observe pin 13 after USB enumeration:
- Idle (no host traffic): slow blink, target ~0.5 Hz.
- Send Hurra/Ferrum commands (`hurra-bridge` + a test tool, or `tools/ferrum_test.py` on a `PROTOCOL=ferrum` build): blink speeds up to ~2 Hz.
- Induce UART errors (connect at a wrong baud so framing/overrun counters climb): blink jumps to ~6 Hz.
- Critically, the blink continues at a steady rate while the proxy is busy forwarding HID reports — proving it is hardware-driven, not CPU-toggled.

If the absolute idle rate is off (e.g. measures ~0.7 Hz instead of ~0.5 Hz because the real IP-bus clock differs from the 150 MHz nominal), scale `LED_HB_K` in `src/led.c` by `measured_hz / target_hz` and rebuild. The active/error rates track automatically (they share the CH1 prescaler; only the COMP0 ratio sets them), so a single constant calibrates all three. If unavailable, note that build + boundary gates passed and hardware calibration is pending.

---

## Self-Review

- **Spec coverage:** QuadTimer2 cascade (CH1→CH0, ALT1) — Task 1 ✓; `led.c`/`led.h` module with the exact spec API (`led_init/on/off/toggle/blink_forever/heartbeat_start/heartbeat_set_rate`) — Task 1 ✓; GPIO7 lifecycle + pad reclaim in `led_blink_forever` — Task 1 ✓; glitch-free CMPLD/CL rate update — Task 1 ✓; Makefile (SRC, not HOT_SRC) — Task 2 ✓; main.c include + `led_init` early + `led_heartbeat_start` post-enum + remove per-packet pulse + status→rate tick + drop `led_pwm_set` — Task 3 ✓; idle/active/error = 50/200/600 centihz (0.5/2/6 Hz) — Task 3 Step 6 ✓; both-variant build + hardware calibration — Task 4 ✓.
- **Placeholder scan:** none — every code edit shows exact code; every command shows expected output.
- **Type consistency:** `led_heartbeat_set_rate(uint16_t centihz)`, `led_heartbeat_start(void)`, `centihz_to_comp(uint16_t)->uint16_t`, and the `led_*` GPIO/blink signatures are identical across `led.h` (Task 1 Step 1), `led.c` (Task 1 Step 2), and the `main.c` call sites (Task 3). The kmbox accessors used in Task 3 Step 6 (`kmbox_rx_byte_count/uart_overrun/uart_framing/uart_noise`, all `uint32_t(void)`) are confirmed in Step 7.
