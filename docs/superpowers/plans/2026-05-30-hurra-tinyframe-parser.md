# Hurra TinyFrame Parser Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a latency-optimized, wire-compatible Hurra binary (TinyFrame) protocol parser to the Hurra-v2 firmware, make it the default protocol, harden the LPUART clock, remove dead code, and update docs.

**Architecture:** Port `imxrtnsy`'s proven Hurra subsystem (`hurra.c`, `proto.h`, vendored TinyFrame, `TF_Config.h`) — already byte-for-byte wire-compatible with `hurra-app` — and graft it onto Hurra-v2's newer CH343B-tuned LPUART3/eDMA transport. Two added latency wins over imxrtnsy: batch `TF_Accept` over each DMA burst, and a single TX flush immediately after the burst drain.

**Tech Stack:** C (bare-metal Cortex-M7, ARM GCC), NXP i.MX RT1062 LPUART3 + eDMA, TinyFrame (MightyPork, MIT).

**Testing note:** This is bare-metal firmware with no unit-test harness. "The test" for each task is the cross-compile build (`make` for Hurra, `make PROTOCOL=ferrum` for Ferrum), plus byte-identical wire-config gates (`diff` against `hurra-app`) and the `size` report. The first full link is in Task 5 (after the Makefile wires everything); Tasks 1–4 stage files that link together there.

**Reference paths (read-only source of truth):**
- `/Users/ramseymcgrath/code/imxrtnsy/src/{hurra.c,hurra.h,proto.h,TF_Config.h,actions.c,actions.h}`
- `/Users/ramseymcgrath/code/imxrtnsy/src/third_party/TinyFrame/`
- `/Users/ramseymcgrath/code/hurra-app/src/TF_Config.h`, `/Users/ramseymcgrath/code/hurra-app/vendor/TinyFrame/TinyFrame.c` (wire-compat gate targets)

All commands below assume CWD `/Users/ramseymcgrath/code/Hurra-v2` on branch `feat/hurra-tinyframe-parser` (already created).

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `src/third_party/TinyFrame/TinyFrame.{c,h}` | Framing/CRC library (vendored, MIT) | Create (copy) |
| `src/TF_Config.h` | TinyFrame wire config (SOF 0x68, CRC16) | Create (copy) |
| `src/proto.h` | Compile-time `proto_*` selector + `proto_feed` | Create |
| `src/hurra.c` / `src/hurra.h` | Hurra binary parser (TinyFrame listeners) | Create (copy + `hurra_feed`) |
| `src/actions.c` / `src/actions.h` | Transport-agnostic actions (superset w/ wheel/invert/swap) | Modify (adopt imxrtnsy superset) |
| `src/kmbox.c` / `src/kmbox.h` | LPUART3/eDMA transport + protocol glue | Modify |
| `Makefile` | Build, protocol selector | Modify |
| `CLAUDE.md`, `README.md` | Docs | Modify |
| `docs/specs/2026-05-18-makcu-binary-protocol-design.md` | Obsolete MAKCU spec | Delete |

---

## Task 1: Vendor TinyFrame + protocol files

**Files:**
- Create: `src/third_party/TinyFrame/TinyFrame.c`, `src/third_party/TinyFrame/TinyFrame.h`
- Create: `src/TF_Config.h`, `src/proto.h`, `src/hurra.h`, `src/hurra.c`

- [ ] **Step 1: Copy the vendored library and protocol files from imxrtnsy**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
mkdir -p src/third_party/TinyFrame
cp /Users/ramseymcgrath/code/imxrtnsy/src/third_party/TinyFrame/TinyFrame.c src/third_party/TinyFrame/
cp /Users/ramseymcgrath/code/imxrtnsy/src/third_party/TinyFrame/TinyFrame.h src/third_party/TinyFrame/
cp /Users/ramseymcgrath/code/imxrtnsy/src/TF_Config.h src/TF_Config.h
cp /Users/ramseymcgrath/code/imxrtnsy/src/proto.h     src/proto.h
cp /Users/ramseymcgrath/code/imxrtnsy/src/hurra.h     src/hurra.h
cp /Users/ramseymcgrath/code/imxrtnsy/src/hurra.c     src/hurra.c
```

- [ ] **Step 2: Wire-compat gate — confirm framing config and library are byte-identical to the app**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
diff src/TF_Config.h /Users/ramseymcgrath/code/hurra-app/src/TF_Config.h && echo "TF_Config OK"
diff src/third_party/TinyFrame/TinyFrame.c /Users/ramseymcgrath/code/hurra-app/vendor/TinyFrame/TinyFrame.c && echo "TinyFrame.c OK"
```
Expected: both print `OK`. (TF_Config.h is functionally identical — only the comment style differs between trees. If `diff` shows ONLY comment-line differences, that is acceptable; the `#define` values must match exactly: SOF 0x68, ID/LEN/TYPE = 1, CRC16, MAX_PAYLOAD_RX 256, SENDBUF_LEN 264. If any `#define` value differs, STOP and reconcile.)

- [ ] **Step 3: Add the batch-feed entry point `hurra_feed` to the header**

In `src/hurra.h`, add this declaration after the existing `void hurra_feed_byte(uint8_t b);` line:

```c
// Feed a contiguous span of received bytes in one call (batch TF_Accept).
// Lower per-byte overhead than calling hurra_feed_byte in a loop.
void hurra_feed(const uint8_t *buf, uint16_t len);
```

- [ ] **Step 4: Implement `hurra_feed` in hurra.c**

In `src/hurra.c`, immediately after the existing `void hurra_feed_byte(uint8_t b) { TF_AcceptChar(&s_tf, b); }` line, add:

```c
void hurra_feed(const uint8_t *buf, uint16_t len) { TF_Accept(&s_tf, buf, (uint32_t)len); }
```

- [ ] **Step 5: Add `proto_feed` to the protocol selector**

In `src/proto.h`, inside the `#if defined(PROTOCOL_HURRA)` branch, after `#define proto_feed_byte hurra_feed_byte`, add:

```c
  static inline void proto_feed(const uint8_t *b, uint16_t n) { hurra_feed(b, n); }
```

In the `#elif defined(PROTOCOL_FERRUM)` branch, after `#define proto_feed_byte ferrum_feed_byte`, add:

```c
  static inline void proto_feed(const uint8_t *b, uint16_t n) {
      for (uint16_t i = 0; i < n; i++) ferrum_feed_byte(b[i]);
  }
```

(This keeps `ferrum.c` untouched — no divergence from imxrtnsy's identical copy.) Ensure `proto.h` includes `<stdint.h>` at the top; if it is not already present, add `#include <stdint.h>` under the `#pragma once`.

- [ ] **Step 6: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add src/third_party/TinyFrame src/TF_Config.h src/proto.h src/hurra.h src/hurra.c
git commit -m "feat(hurra): vendor TinyFrame + Hurra parser, add batch proto_feed

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Adopt the actions.c/.h superset

**Files:**
- Modify: `src/actions.c`, `src/actions.h` (adopt imxrtnsy superset: `act_wheel`, invert/swap getters+setters)

- [ ] **Step 1: Copy imxrtnsy's actions files (Hurra-v2's are a strict subset)**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
cp /Users/ramseymcgrath/code/imxrtnsy/src/actions.c src/actions.c
cp /Users/ramseymcgrath/code/imxrtnsy/src/actions.h src/actions.h
```

- [ ] **Step 2: Verify the change is purely additive**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git diff --stat src/actions.c src/actions.h
```
Expected: only insertions relative to the prior Hurra-v2 versions (the new lines are `s_invert_x/y`, `s_swap_xy`, the swap/invert application in `act_move`, and `act_wheel`/`act_get_*`/`act_set_*`). No unrelated deletions.

- [ ] **Step 3: Sanity-check the symbols hurra.c depends on are present**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -nE "act_wheel|act_get_invert_x|act_set_invert_x|act_get_invert_y|act_set_invert_y|act_get_swap_xy|act_set_swap_xy" src/actions.h
grep -nE "^(uint8_t|int32_t|uint16_t)\s+g_(buttons|pos_x|pos_y|lock_mask|kb_modifier)" src/actions.c
```
Expected: all seven `act_*` declarations present in the header; all five `g_*` globals present in the source.

- [ ] **Step 4: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add src/actions.c src/actions.h
git commit -m "feat(actions): add wheel + invert/swap helpers for Hurra parser

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: kmbox.c / kmbox.h — protocol glue, latency wins, hardening, dead-code removal

**Files:**
- Modify: `src/kmbox.c` (include, init, poll_heavy drain, new accessors, clock hardening, remove `detected_proto`)
- Modify: `src/kmbox.h` (swap accessor declarations)

- [ ] **Step 1: Switch the protocol include and calls from ferrum to proto**

In `src/kmbox.c`, replace the line `#include "smooth.h"` block's sibling include of ferrum. Specifically change the header include near the top from:

```c
#include "kmbox.h"
#include "smooth.h"
#include "imxrt.h"
#include "usb_device.h"
#include <string.h>
```
to add `proto.h` (it pulls in the selected protocol's header):
```c
#include "kmbox.h"
#include "smooth.h"
#include "imxrt.h"
#include "usb_device.h"
#include "proto.h"
#include <string.h>
```

Then replace the protocol calls in `kmbox_init()`:
- `ferrum_set_tx(uart_tx_frame);` → `proto_set_tx(uart_tx_frame);`
- `ferrum_init();` → `proto_init();`

And in `kmbox_poll_fast()`: `ferrum_tick();` → `proto_tick();`

- [ ] **Step 2: Add the LPUART 24 MHz root-clock hardening**

In `src/kmbox.c`, in `kmbox_init()`, as the FIRST statements inside the function (before `CCM_CCGR0 |= CCM_CCGR0_LPUART3(CCM_CCGR_ON);`), insert:

```c
	// Hardening: pin the LPUART root clock to the 24 MHz crystal oscillator
	// (UART_CLK_SEL=1) with no post-divider (UART_CLK_PODF=0) so compute_baud_reg's
	// UART_CLOCK=24e6 assumption is guaranteed rather than inherited from the
	// bootloader/core. RM §14 (CCM): CCM_CSCDR1[UART_CLK_SEL], [UART_CLK_PODF].
	CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL;
```

- [ ] **Step 3: Add the rx-driver-overrun counter and replace the RX drain with batch-feed + per-burst flush**

In `src/kmbox.c`, add the counter near the other static counters (next to `static uint32_t uart_overrun_count;`):

```c
static uint32_t rx_drv_overrun_count;
```

Zero it in `kmbox_init()` alongside the other counter resets (next to `uart_overrun_count = 0;`):

```c
	rx_drv_overrun_count = 0;
```

Then replace the body of `kmbox_poll_heavy()` from the `uint16_t head = ...` line through the end of the `while (rx_tail != head) { ... }` loop with:

```c
	uint16_t head = ((uint32_t)KM_RX_DADDR - (uint32_t)dma_rx_ring) & (DMA_RX_RING_SIZE - 1);
	if (head != rx_tail) {
		GPIO3_DR_TOGGLE = LINK_LED_BIT;
		link_last_rx_time = millis();
		last_rx_activity_time = link_last_rx_time;
	}

	// Driver-overrun heuristic: if the HW write pointer has run >=3/4 of the
	// ring ahead of the SW read pointer, bytes were almost certainly lost (the
	// eDMA ring wraps silently). Count it, snap rp to wp to skip the stale
	// span, and reset the parser so the next valid frame realigns cleanly.
	{
		uint16_t gap = (uint16_t)((head - rx_tail) & (DMA_RX_RING_SIZE - 1));
		if (__builtin_expect(gap > (DMA_RX_RING_SIZE * 3u / 4u), 0)) {
			rx_drv_overrun_count++;
			rx_tail = head;
			proto_reset();
			GPIO1_DR_TOGGLE = STATUS_LED_BIT;
		}
	}

	if (rx_tail != head) {
		// Feed the whole burst in <=2 contiguous spans (batch TF_Accept).
		uint16_t count = (uint16_t)((head - rx_tail) & (DMA_RX_RING_SIZE - 1));
		if (head > rx_tail) {
			proto_feed(&dma_rx_ring[rx_tail], (uint16_t)(head - rx_tail));
		} else {
			proto_feed(&dma_rx_ring[rx_tail],
			           (uint16_t)(DMA_RX_RING_SIZE - rx_tail));
			if (head) proto_feed(&dma_rx_ring[0], head);
		}
		rx_bytes_total += count;
		frames_ok++;                       // counts RX bursts processed
		GPIO3_DR_TOGGLE = STATE_LED_BIT;
		rx_tail = head;
		// Immediate reply flush: any reply/telemetry the parser queued during
		// the feed leaves in one DMA TX on this same poll, instead of waiting
		// for the next poll_fast tick. Protocol-agnostic latency win.
		tx_flush();
	}
```

(This removes the per-byte loop, the per-`\r\n` flush, and the `detected_proto = 1;` write.)

- [ ] **Step 4: Remove the `detected_proto` dead code**

In `src/kmbox.c`:
- Delete the line `static uint8_t detected_proto;`
- Delete the `detected_proto = 0;` line in `baud_change_apply()`.
- Delete the accessor `uint8_t  kmbox_protocol_mode(void) { return detected_proto; }`.

Add the two new accessors next to the other `kmbox_*` stat accessors (e.g. after `uint32_t kmbox_tx_overflow(void) { return tx_overflow_count; }`):

```c
uint32_t kmbox_rx_drv_overrun(void) { return rx_drv_overrun_count; }

uint16_t kmbox_tx_room(void)
{
	// Bytes free in the TX ring. Telemetry emitters skip a frame when the ring
	// would overflow; input listeners never skip.
	uint16_t used = (uint16_t)((tx_head - tx_tail_pos) & (TX_RING_SIZE - 1));
	return (uint16_t)(TX_RING_SIZE - 1 - used);
}
```

- [ ] **Step 5: Update kmbox.h declarations**

In `src/kmbox.h`:
- Delete the line `uint8_t  kmbox_protocol_mode(void); // 0=idle, 1=Ferrum`
- Add, next to the other accessor declarations:

```c
uint32_t kmbox_rx_drv_overrun(void);
uint16_t kmbox_tx_room(void);
```

- [ ] **Step 6: Verify no remaining references to removed symbols**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -rnE "detected_proto|kmbox_protocol_mode|ferrum_init|ferrum_set_tx|ferrum_tick" src/kmbox.c src/kmbox.h src/main.c
```
Expected: no output (all replaced/removed). If `grep` prints anything, fix it before committing.

- [ ] **Step 7: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add src/kmbox.c src/kmbox.h
git commit -m "feat(kmbox): proto_* glue, batch feed + per-burst flush, 24MHz LPUART clock

- route protocol through proto.h (Hurra/Ferrum compile-time switch)
- batch TF_Accept over each DMA burst; single tx_flush after drain (latency)
- pin LPUART root clock to 24MHz oscillator (hardening)
- add kmbox_tx_room / kmbox_rx_drv_overrun for Hurra STATS
- remove vestigial detected_proto / kmbox_protocol_mode

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Makefile — PROTOCOL selector (Hurra default)

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Add the CMD_BAUD-adjacent protocol selector block**

In `Makefile`, immediately after the `CMD_BAUD ?= 115200` line, insert:

```make
# Protocol selector: 'hurra' (binary, TinyFrame, default) or 'ferrum' (ASCII).
PROTOCOL ?= hurra

ifeq ($(PROTOCOL),hurra)
  PROTO_DEF = -DPROTOCOL_HURRA
  PROTO_SRC = src/hurra.c src/third_party/TinyFrame/TinyFrame.c
else ifeq ($(PROTOCOL),ferrum)
  PROTO_DEF = -DPROTOCOL_FERRUM
  PROTO_SRC = src/ferrum.c
else
  $(error PROTOCOL must be 'hurra' or 'ferrum')
endif
```

- [ ] **Step 2: Add PROTO_DEF to DEFINES and the TinyFrame include path**

Change the `DEFINES` block from:
```make
DEFINES = -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=816000000 \
          -DCMD_BAUD=$(CMD_BAUD)
```
to:
```make
DEFINES = -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=816000000 \
          -DCMD_BAUD=$(CMD_BAUD) $(PROTO_DEF)
```

Change the `CFLAGS` include flags from `-Iinclude -Isrc` to:
```make
         -Iinclude -Isrc -Isrc/third_party/TinyFrame
```

- [ ] **Step 3: Swap the fixed ferrum source for PROTO_SRC and fix hot-path objects**

Change the `SRC` list from:
```make
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/ferrum.c src/actions.c
```
to:
```make
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/actions.c \
      $(PROTO_SRC)
```

Change the `HOT_SRC` block from:
```make
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o \
          src/humanize.o src/ferrum.o src/actions.o
```
to:
```make
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o \
          src/humanize.o src/actions.o
ifeq ($(PROTOCOL),hurra)
  HOT_SRC += src/hurra.o src/third_party/TinyFrame/TinyFrame.o
else
  HOT_SRC += src/ferrum.o
endif
```

- [ ] **Step 4: Build the default (Hurra) variant — first full link**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
make clean && make 2>&1 | tail -25
```
Expected: compiles and links with no errors; ends with a `size` report (`text data bss dec hex filename` for `firmware.elf`) and produces `firmware.hex`. If there are errors, fix them (most likely an unresolved symbol indicates a Task 1–3 step was missed).

- [ ] **Step 5: Build the Ferrum variant to prove the switch still works**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
make clean && make PROTOCOL=ferrum 2>&1 | tail -15
make clean && make 2>&1 | tail -5   # leave the tree on the default Hurra build
```
Expected: the Ferrum build compiles and links clean; the final default build also succeeds.

- [ ] **Step 6: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add Makefile
git commit -m "build: add PROTOCOL selector (Hurra default, Ferrum opt-in)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Remove the obsolete MAKCU design doc

**Files:**
- Delete: `docs/specs/2026-05-18-makcu-binary-protocol-design.md`

- [ ] **Step 1: Confirm nothing references it, then delete**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -rln "makcu-binary-protocol-design" . --include=*.md --include=*.c --include=*.h --include=Makefile || echo "no references"
git rm docs/specs/2026-05-18-makcu-binary-protocol-design.md
```
Expected: `no references` (other than the file itself), then the file is staged for deletion. The MAKCU binary protocol was superseded by the Hurra/TinyFrame protocol implemented in this plan.

- [ ] **Step 2: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git commit -m "docs: remove obsolete MAKCU binary protocol spec (superseded by Hurra)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Update CLAUDE.md and README.md

**Files:**
- Modify: `CLAUDE.md`, `README.md`

- [ ] **Step 1: Update CLAUDE.md — Board Target chip name**

In `CLAUDE.md`, in the **Board Target** section, change:
```
  - Host link: CP2102C USB-UART bridge over LPUART3 on Teensy pins 16/17 (ATP carrier UART_RX2/UART_TX2). Moved off pins 0/1 after suspected pad damage.
```
to:
```
  - Host link: WCH CH343 USB-UART bridge over LPUART3 on Teensy pins 16/17 (ATP carrier UART_RX2/UART_TX2). Moved off pins 0/1 after suspected pad damage. CH343 supports up to 6 Mbaud (USB Full Speed, 64-byte bulk MPS).
```

- [ ] **Step 2: Update CLAUDE.md — Protocol section (Hurra default, Ferrum opt-in)**

In `CLAUDE.md`, replace the entire **## Protocol** section with:

```markdown
## Protocol
- **Default: Hurra binary protocol** — TinyFrame-based (SOF `0x68`, 1-byte ID/LEN/TYPE, CRC16), targeting >=8k cmds/sec at 4 Mbps. Built by `make` (no flag). Implementation in `src/hurra.c` + `src/third_party/TinyFrame/`. Host adapter: `hurra-app` (`hurra-bridge`).
- **Opt-in: Ferrum ASCII text protocol** (`make PROTOCOL=ferrum`) — https://ferrumllc.github.io/print.html
  - Wire: `km.<name>(<args>)\r\n` (also accepts `\n` only). Alias `m(x,y)` for move.
  - Default baud 115200, resets to 115200 every power cycle. `km.baud(N)` to bump.
  - No command echo, no `>>>` prompt. `km.version()` -> `kmbox: Ferrum\r\n`.
  - Parser: line accumulator + tokenizer + dispatch table in `src/ferrum.c`.
- Protocol abstraction: `src/proto.h` aliases `proto_*` to the selected parser; `kmbox.c` calls `proto_*` with no `#ifdef`s at call sites.
- Actions: transport-agnostic `act_*` functions in `src/actions.c` (drive `kmbox_inject_*`).
```

- [ ] **Step 3: Update CLAUDE.md — Key Files and Build sections**

In `CLAUDE.md`, in **## Key Files**, add after the `src/kmbox.c` bullet:
```markdown
- `src/hurra.c` / `src/hurra.h` — Hurra binary protocol parser (TinyFrame), default
- `src/proto.h` — compile-time protocol selector (`proto_*` -> Hurra or Ferrum)
- `src/ferrum.c` / `src/ferrum.h` — Ferrum ASCII parser (opt-in via `PROTOCOL=ferrum`)
```

In **## Build**, change `- `make` — builds firmware.hex for MicroMod` to:
```markdown
- `make` — builds firmware.hex for MicroMod (Hurra protocol, default)
- `make PROTOCOL=ferrum` — builds with Ferrum ASCII protocol instead
```
And in the hot-path line, change `kmbox, ferrum, actions, smooth, ...` to `kmbox, hurra (or ferrum), actions, smooth, ...`.

- [ ] **Step 4: Update README.md — title/intro, chip, protocol, build**

In `README.md`:

Replace the intro paragraph (the line starting "Bare-metal USB HID man-in-the-middle firmware ... accepts **Ferrum text protocol** commands") with:
```markdown
Bare-metal USB HID man-in-the-middle firmware for the **SparkFun MicroMod Teensy** (NXP i.MX RT1062). Enumerates a real USB HID device on the host port, replays it on the device port to the Mac/PC, and accepts **Hurra binary protocol** commands over UART (default) to inject mouse/keyboard input on top of the live HID stream. A **Ferrum ASCII** compatibility mode is available via `make PROTOCOL=ferrum`.

The host-side adapter is [`hurra-app`](https://github.com/VoltCyclone/hurra-app) (`hurra-bridge`), which also exposes a Ferrum-compatible virtual COM port for legacy tools.
```

In the **## Hardware** list, change the CP2102C bullet:
```markdown
- **WCH CH343** USB-UART bridge wired to Teensy `RX2`/`TX2` (D16/D17 -> LPUART3; ATP carrier UART_RX2/UART_TX2 headers). USB Full Speed, up to 6 Mbaud, 64-byte bulk MPS.
```

Replace the **## Wire protocol** section body with:
```markdown
**Default — Hurra binary (TinyFrame):** SOF `0x68`, 1-byte ID/LEN/TYPE, CRC16. Little-endian payloads. Driven by `hurra-app`/`hurra-bridge`; see that repo for the host API. Targets >=8k commands/sec at 4 Mbps over the CH343 link.

**Compatibility — Ferrum ASCII** (`make PROTOCOL=ferrum`): `\r\n`-terminated text, 115200 baud (resets to 115200 every power cycle). Reference: <https://ferrumllc.github.io/print.html>.

```
TX: km.version()\r\n
RX: kmbox: Ferrum\r\n
TX: km.move(10, -5)\r\n          # write — no reply
TX: m(2, 0)\r\n                  # alias for km.move
```
```

In **## Build & flash**, change the `make` description so it reads:
```markdown
make                 # produces firmware.hex (Hurra binary protocol, default)
make PROTOCOL=ferrum # build with Ferrum ASCII protocol instead
make flash           # flashes via teensy_loader_cli
```

In the **## Layout** block, change the `src/ferrum.c/.h` line to two lines:
```
src/hurra.c/.h        Hurra binary parser (TinyFrame) — default protocol
src/ferrum.c/.h       Ferrum ASCII parser (opt-in: PROTOCOL=ferrum)
src/proto.h           compile-time protocol selector
```

- [ ] **Step 5: Verify no stale chip/protocol references remain in docs**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -rniE "CP2102|3 Mbaud" CLAUDE.md README.md || echo "no stale chip refs"
```
Expected: `no stale chip refs`.

- [ ] **Step 6: Commit**

```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git add CLAUDE.md README.md
git commit -m "docs: Hurra default protocol, CH343 bridge (was CP2102C), build flags

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Final verification gate

**Files:** none (verification only)

- [ ] **Step 1: Both protocol variants build clean from scratch**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
make clean && make PROTOCOL=ferrum 2>&1 | tail -4
make clean && make 2>&1 | tail -6
```
Expected: both end with a `size` report and no errors; the tree is left on the default Hurra build with `firmware.hex` present.

- [ ] **Step 2: Re-assert the wire-compatibility gate**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
diff src/third_party/TinyFrame/TinyFrame.c /Users/ramseymcgrath/code/hurra-app/vendor/TinyFrame/TinyFrame.c && echo "TinyFrame wire-identical to app"
grep -E "TF_(SOF_BYTE|ID_BYTES|LEN_BYTES|TYPE_BYTES|CKSUM_TYPE)" src/TF_Config.h
```
Expected: `TinyFrame wire-identical to app`, and the five `#define`s show SOF `0x68`, ID/LEN/TYPE = 1, CRC16 — matching the app.

- [ ] **Step 3: Confirm opcode map matches the app's `hurra_types.h`**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -E "TYPE_VERSION|TYPE_MOUSE_MOVE|TYPE_BTN_LEFT|TYPE_CATCH_XY" src/hurra.c
grep -E "HURRA_TYPE_VERSION|HURRA_TYPE_MOUSE_MOVE|HURRA_TYPE_BTN_LEFT|HURRA_TYPE_CATCH_XY" /Users/ramseymcgrath/code/hurra-app/include/hurra_types.h
```
Expected: `VERSION=0x01`, `MOUSE_MOVE=0x10`, `BTN_LEFT=0x20`, `CATCH_XY=0x67` on both sides.

- [ ] **Step 4: Confirm the dead code and stale docs are gone**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
grep -rnE "detected_proto|kmbox_protocol_mode" src/ || echo "dead code gone"
test ! -f docs/specs/2026-05-18-makcu-binary-protocol-design.md && echo "MAKCU spec removed"
grep -rniE "CP2102|3 Mbaud" CLAUDE.md README.md || echo "docs clean"
```
Expected: `dead code gone`, `MAKCU spec removed`, `docs clean`.

- [ ] **Step 5: Hardware bench test (manual, requires the device)**

This is the real end-to-end check — the path that previously failed with `rc=-1`. With Hurra-v2 (default build) flashed (`make flash`) and `hurra-app` built:
```bash
# Terminal 1
/Users/ramseymcgrath/code/hurra-app/build/hurra-bridge --device /dev/cu.usbmodem01 --baud 4000000
# Expect: "version probe ok: fw=\"kmbox: Hurra v1\"" in the bridge log (NOT "version probe FAILED").
# Terminal 2
/Users/ramseymcgrath/code/imxrtnsy/tools/ferrum_aim_test.py --port ~/.hurra-bridge.tty
# Expect: handshake succeeds and the cursor moves.
```
Expected: bridge reports the firmware version (link is live) and the mouse moves. If unavailable, note that the build + wire-compat gates passed and hardware verification is pending.

- [ ] **Step 6: Final summary commit is unnecessary (all work already committed). Confirm clean tree**

Run:
```bash
cd /Users/ramseymcgrath/code/Hurra-v2
git status --short && git log --oneline origin/main..HEAD 2>/dev/null || git log --oneline -8
```
Expected: working tree clean; the branch contains the spec + the six implementation commits.

---

## Self-Review

- **Spec coverage:** New files (Task 1) ✓; actions superset (Task 2) ✓; kmbox proto glue + tx_room/rx_drv_overrun + batch feed + per-burst flush + 24 MHz clock hardening + detected_proto removal (Task 3) ✓; SEVONPEND already present, no task needed (noted) ✓; Makefile PROTOCOL selector, Hurra default (Task 4) ✓; obsolete MAKCU doc removal (Task 5) ✓; CLAUDE.md + README.md incl. CP2102C→CH343 + 6 Mbaud (Task 6) ✓; build + wire-compat + bench verification (Task 7) ✓.
- **Placeholder scan:** no TBD/TODO; every code edit shows exact code; every command shows expected output.
- **Type consistency:** `proto_feed(const uint8_t*, uint16_t)`, `hurra_feed(const uint8_t*, uint16_t)`, `kmbox_tx_room(void)->uint16_t`, `kmbox_rx_drv_overrun(void)->uint32_t` used consistently across Tasks 1, 3, 7 and match `hurra.c`'s call sites.
