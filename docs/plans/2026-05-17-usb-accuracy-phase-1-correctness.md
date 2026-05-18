# USB Accuracy Phase 1 — Control-Path Correctness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop silently dropping control-OUT data over 64 bytes and stop answering control requests (class GET_REPORT/GET_IDLE/GET_PROTOCOL/SET_IDLE/SET_PROTOCOL and standard GET_DESCRIPTOR for unknown types) with stub values instead of forwarding them to the upstream device.

**Architecture:** No new files, no new data structures, no new infrastructure. Three behavioral fixes in `src/usb_device.c`: (1) bump the deferred-OUT buffer from 64 → 512 bytes, (2) make the GET_DESCRIPTOR switch's `default` case forward to upstream via the existing `handle_passthrough()` helper, (3) rewire `handle_class_request()` to forward every class request to upstream except where we have a real reason to answer locally.

**Tech Stack:** Bare-metal C (arm-none-eabi-gcc), i.MX RT1062 EHCI device controller, ARM GCC via PlatformIO toolchain. No host test framework — verification is build + flash + USB descriptor diff via `lsusb -v` on a Linux box (or System Information.app on macOS) and traffic inspection via `usbmon` / Total Phase Beagle if available.

**Spec reference:** `docs/specs/2026-05-17-usb-device-accuracy-design.md` — Phase 1.

---

## Pre-flight

- [ ] **Verify clean working tree for Phase 1 commits**

Run: `git status`
Expected: Existing uncommitted changes (Makefile, kmbox.c, smooth.c, usb_host.c, scripts/) are unrelated to this work and should remain untouched. Confirm `src/usb_device.c` is **not** in the modified list. If it is, stop and ask the user before proceeding — Phase 1 needs a clean baseline for usb_device.c.

- [ ] **Capture pre-change descriptor baseline (manual, requires hardware)**

With the firmware as currently built (commit `d031bd500`), flash a known-good baseline build and plug in one upstream device (start with a boot mouse). On the downstream Linux host (or analyzer):

```bash
lsusb -v -d <vid>:<pid> > /tmp/imxrtnsy-baseline-mouse.txt 2>&1
```

Save this file. It is the regression baseline — Phase 1 must not change observable enumeration for boot-only devices.

If no test hardware available: skip this step and rely on smoke-flash later. Note this in the commit message so reviewers know coverage is limited.

---

## Task 1: Eliminate the 64-byte control-OUT silent truncation

**Files:**
- Modify: `src/usb_device.c:30-35` (`deferred_out` struct)
- Modify: `src/usb_device.c:401` (truncation conditional inside `handle_passthrough`)

**Why:** `handle_passthrough()` receives a control-OUT data phase into `ep0_rx_buf` (512 B), then copies into `deferred_out.data` for later forwarding. The local buffer is 64 B. When `wLength > 64`, the entire transfer is silently discarded by the `if (rxd <= 64)` guard at line 401 — including the `deferred_out.pending = true` flag, so the upstream device never sees the request. Razer Synapse-style vendor SET_REPORT payloads regularly exceed 64 B and silently fail today.

- [ ] **Step 1: Read the existing struct and the truncation site**

Use Read on `src/usb_device.c` lines 30-40 and 393-414 to confirm current state. Expected:
```c
static struct {
    usb_setup_t setup;
    uint8_t     data[64];
    uint16_t    data_len;
    bool        pending;
} deferred_out;
```
and at line 401:
```c
if (rxd <= (int)sizeof(deferred_out.data)) {
    memcpy(&deferred_out.setup, setup, sizeof(*setup));
    memcpy(deferred_out.data, ep0_rx_buf, rxd);
    ...
}
```

- [ ] **Step 2: Enlarge the buffer to 512 bytes**

Use Edit on `src/usb_device.c` to change:
```c
static struct {
    usb_setup_t setup;
    uint8_t     data[64];
    uint16_t    data_len;
    bool        pending;
} deferred_out;
```
to:
```c
static struct {
    usb_setup_t setup;
    uint8_t     data[512];  // matches ep0_rx_buf so we never truncate control-OUT
    uint16_t    data_len;
    bool        pending;
} deferred_out;
```

The `if (rxd <= (int)sizeof(deferred_out.data))` guard at line 401 is now satisfied for any payload `ep0_rx_data()` can return (it caps at 512 internally), so the silent-drop case becomes unreachable — no logic change to that conditional needed.

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build, no new warnings. Confirm `.bss` / `.dmabuffers` section grew by exactly 448 bytes via:
```bash
arm-none-eabi-size firmware.elf
```
Compare to the baseline (previous build size). Difference ≈ 448 B in data/bss.

- [ ] **Step 4: Manual smoke flash (if hardware available)**

```bash
make flash
```
Plug in baseline mouse. Re-run `lsusb -v -d <vid>:<pid> > /tmp/imxrtnsy-task1-mouse.txt 2>&1` and diff against the baseline:
```bash
diff /tmp/imxrtnsy-baseline-mouse.txt /tmp/imxrtnsy-task1-mouse.txt
```
Expected: identical output (this task is invisible to a boot-mouse device since boot mice never send control-OUT > 64 B).

If no hardware: skip and note in commit.

- [ ] **Step 5: Commit**

```bash
git add src/usb_device.c
git commit -m "fix(usb_device): enlarge deferred_out buffer to 512 bytes

Control-OUT payloads from the downstream host > 64 bytes were silently
discarded — handle_passthrough() received the data into ep0_rx_buf but
the local deferred_out.data copy guard rejected it, so deferred_out.pending
stayed false and the upstream device never received the request. Razer-style
vendor SET_REPORT payloads commonly exceed 64 bytes."
```

---

## Task 2: Forward unknown GET_DESCRIPTOR types to upstream

**Files:**
- Modify: `src/usb_device.c` — `handle_get_descriptor()` function (around lines 104-190)

**Why:** Today the switch covers DEVICE / CONFIGURATION / STRING / HID / HID_REPORT and STALLs everything else. Windows 10+ probes BOS (`0x0F`) during enumeration; USB-2.0 high-speed devices may be probed for Device Qualifier (`0x06`) and Other Speed Configuration (`0x07`). STALLing these is a fingerprintable difference and breaks any device that legitimately uses BOS (USB 2.1+ Platform Capabilities, MS OS 2.0 descriptors). A `default → handle_passthrough(setup)` line forwards anything we don't cache.

- [ ] **Step 1: Locate the GET_DESCRIPTOR switch and find its end**

Read `src/usb_device.c` lines 104-200. Find the end of the descriptor `switch (type)` statement and the `if (data && len > 0) { ep0_tx_data(data, len); } else { ep0_stall(); }` tail that handles the result.

- [ ] **Step 2: Add `default` case that forwards to upstream**

The current switch ends with implicit fall-through to the STALL path (data stays NULL). We need to intercept the unknown-type case **before** the STALL path runs.

Find the line that looks like:
```c
        default:
            break;
```
or, if there is no explicit default, find the closing brace of the `switch (type)` block. Add (replacing the existing `default: break;` if present, or inserting before the closing `}`):
```c
        default:
            handle_passthrough(setup);
            return;
```
The `return` is important: `handle_passthrough()` calls `ep0_tx_data()` or `ep0_stall()` itself, so we must not fall through to the trailer that runs `ep0_stall()` again.

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build, no warnings, no size change beyond a handful of bytes.

- [ ] **Step 4: Manual smoke (if hardware)**

Flash and plug baseline mouse. The boot mouse won't have a BOS, so `handle_passthrough` will issue GET_DESCRIPTOR(BOS) upstream, the mouse will STALL, and `handle_passthrough` will STALL downstream — same observable behavior as before. Verify by re-running `lsusb -v` and diffing against `task1` output. Expected: identical.

If you have a USB 2.1+ device handy (anything modern: USB-C dongles, recent gaming mice), plug that in and confirm `lsusb -v` now shows the BOS descriptor section instead of an empty/missing line.

- [ ] **Step 5: Commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): forward unknown GET_DESCRIPTOR types to upstream

Previously BOS (0x0F), Device Qualifier (0x06), Other Speed Config (0x07),
and any other descriptor type not in our switch were STALLed. Now the default
case routes through handle_passthrough() so the upstream device sees and
answers them. Devices that don't support BOS still STALL — handle_passthrough
propagates the upstream STALL — so behavior is unchanged for legacy devices."
```

---

## Task 3: Forward all class control requests we don't truly own

**Files:**
- Modify: `src/usb_device.c:311-341` — `handle_class_request()` function

**Why:** Five of the six HID class requests are currently answered locally with stub data:
- `0x01 GET_REPORT` returns zeros
- `0x02 GET_IDLE` is missing from the switch entirely → STALL
- `0x03 GET_PROTOCOL` returns hardcoded `1`
- `0x0A SET_IDLE` ACKs locally; upstream never sees it
- `0x0B SET_PROTOCOL` ACKs locally; upstream never sees it

Only `0x09 SET_REPORT` (line 328-329) correctly forwards via `handle_passthrough()`. The fix: forward all of them. `handle_passthrough()` already handles both directions (IN: synchronous round-trip; OUT-with-data: receive-then-defer; OUT-no-data: ACK-then-defer).

- [ ] **Step 1: Read the function**

Read `src/usb_device.c` lines 311-341.

- [ ] **Step 2: Replace function body**

Use Edit to replace the entire `handle_class_request` body. Match the old:
```c
static void handle_class_request(const usb_setup_t *setup)
{
	switch (setup->bRequest) {
	case 0x0A: // SET_IDLE
		ep0_tx_data(NULL, 0);
		break;
	case 0x0B: // SET_PROTOCOL
		ep0_tx_data(NULL, 0);
		break;
	case 0x01: // GET_REPORT — return zeros (real data flows via interrupt EPs)
		{
			uint16_t len = setup->wLength;
			if (len > sizeof(ep0_tx_buf)) len = sizeof(ep0_tx_buf);
			memset(ep0_tx_buf, 0, len);
			ep0_tx_data(ep0_tx_buf, len);
		}
		break;
	case 0x09: // SET_REPORT — forward to real device (LED control, etc.)
		handle_passthrough(setup);
		return;
	case 0x03: // GET_PROTOCOL
		{
			static const uint8_t proto = 1; // Report protocol
			ep0_tx_data(&proto, 1);
		}
		break;
	default:
		ep0_stall();
		break;
	}
}
```
Replace with:
```c
static void handle_class_request(const usb_setup_t *setup)
{
	// Forward every class request to the upstream device. handle_passthrough()
	// handles IN (synchronous round-trip), OUT-with-data (receive → ACK → defer
	// forward), and OUT-no-data (ACK → defer forward) directions automatically.
	//
	// Previously we answered five of six HID class requests locally with stub
	// data (GET_REPORT → zeros, GET_PROTOCOL → 1, SET_IDLE/SET_PROTOCOL → ACK,
	// GET_IDLE → STALL because missing from switch). All five are now forwarded.
	handle_passthrough(setup);
}
```

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build, function shrinks substantially, no warnings.

- [ ] **Step 4: Manual smoke (if hardware)**

Flash and plug baseline mouse. Critical regression checks:
1. **Enumeration completes.** `lsusb` shows the device. If enumeration hangs, the upstream device may be STALLing on SET_IDLE — this would now STALL downstream too. Check `dmesg` for descriptor read failures.
2. **Mouse movement still works.** Move the upstream mouse, confirm the downstream pointer moves.
3. **`lsusb -v` is byte-identical to Task 2 output** for descriptor fields. The change affects control requests at runtime, not descriptors at enumeration time.

If you have a HID keyboard with LEDs: connect downstream PC, toggle CapsLock, confirm the LED on the upstream keyboard responds. This exercises SET_REPORT (was already working) and confirms the rewrite did not break it.

- [ ] **Step 5: Commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): forward all HID class control requests to upstream

GET_REPORT, GET_IDLE, GET_PROTOCOL, SET_IDLE, SET_PROTOCOL were all answered
locally with stub data or omitted entirely. The downstream host saw zeros for
GET_REPORT, a hardcoded protocol byte, silent ACK for SET_IDLE/SET_PROTOCOL,
and STALL for GET_IDLE — none of which reflect the real device's state.

handle_class_request() now unconditionally forwards via handle_passthrough(),
which already does the right thing for SET_REPORT and vendor requests. Real
device state is reflected; vendor configuration tools (LED, profiles)
read/write the actual device rather than our stubs."
```

---

## Task 4: End-to-end verification

**Files:** None modified. This task is verification only.

**Why:** Phase 1 makes three independently small changes. Run them together against a small device matrix to confirm composite behavior holds before declaring Phase 1 complete.

- [ ] **Step 1: Build and flash final Phase 1 firmware**

```bash
make clean && make && make flash
```

- [ ] **Step 2: Regression-test baseline mouse**

Plug baseline boot mouse upstream. On downstream Linux host:
```bash
lsusb -v -d <vid>:<pid> > /tmp/imxrtnsy-phase1-mouse.txt 2>&1
diff /tmp/imxrtnsy-baseline-mouse.txt /tmp/imxrtnsy-phase1-mouse.txt
```
Expected: identical (boot mice don't exercise the changed paths).

Move the mouse. Confirm cursor tracks.

- [ ] **Step 3: Test with a USB 2.1+ device (if available)**

Plug a modern device (recent gaming mouse, USB hub, anything with `bcdUSB ≥ 0x0210`). Run `lsusb -v`. Confirm the BOS descriptor section is now populated:
```
Binary Object Store Descriptor:
  bLength                 5
  bDescriptorType        15
  ...
```
If the device passes through but no BOS appears, the device may not actually have one — try a different device.

- [ ] **Step 4: Test with HID-keyboard LED (if available)**

Plug a USB keyboard with LEDs upstream. From the downstream PC, toggle Caps Lock / Num Lock. Confirm the upstream keyboard's LED responds. This exercises SET_REPORT (was working before, regression check).

- [ ] **Step 5: Test with Razer device or any device that issues control-OUT > 64 B (if available)**

Plug a Razer mouse, Razer keyboard, or any device with a vendor configurator. Open the configurator (Razer Synapse on Windows, openrazer on Linux). Attempt to change DPI / RGB / profile. Before Phase 1 these silently failed; after Phase 1 they should reach the device.

If no Razer hardware: this test is skipped. Note in the merge commit.

- [ ] **Step 6: Final commit if any verification adjustments needed; otherwise tag**

If everything passes and no further code changes were needed, no commit is needed for this task. Optionally tag the Phase 1 completion point:
```bash
git tag usb-accuracy-phase1
```

---

## Self-review checklist

Before declaring Phase 1 plan complete, verify:

- [ ] Every task has explicit file:line citations
- [ ] Every code change is shown verbatim (no "similar to above")
- [ ] No "TBD", no "implement error handling appropriately", no "add tests"
- [ ] Manual verification steps are concrete (which command to run, which file to diff against)
- [ ] The hardware-unavailable fallback is explicit at each verification step
- [ ] Commits are scoped: one logical change per commit
- [ ] No reference to Phase 2 or Phase 3 types or functions

Spec coverage check (against `2026-05-17-usb-device-accuracy-design.md`):

| Spec gap | Plan task |
|---|---|
| C1 `deferred_out.data[64]` truncation | Task 1 |
| C2 Unknown GET_DESCRIPTOR types STALL | Task 2 |
| C3 GET_REPORT returns zeros | Task 3 |
| C4 GET_IDLE missing | Task 3 |
| C5 SET_IDLE / GET_PROTOCOL / SET_PROTOCOL local-stub | Task 3 |

All Phase 1 spec items covered.
