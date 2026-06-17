# Hurra v2 — Agent Guide

> Bare-metal USB HID man-in-the-middle firmware for the SparkFun MicroMod Teensy (NXP i.MX RT1062). This file is written for AI coding agents who have no prior context on the project. Read it before making changes.

## Project overview

Hurra v2 sits between a real USB HID device (mouse, keyboard, controller) and a host PC:

```
USB HID device ──→ Teensy USB host (USB2/EHCI) ──┐
                                                  │  firmware proxies + injects
Host PC USB ←── Teensy USB device (USB1/EHCI) ────┘
                            ↑
                            │ injected input
                            │
Host PC USB ──→ CH343 USB-UART ──→ Teensy LPUART3 (pins 16/17)
```

The firmware enumerates the real HID device on its USB host port, replays the descriptors on its USB device port, forwards every real HID report unchanged, and merges in injected mouse/keyboard input received over the serial link. Injected motion is always routed through a built-in humanization filter (sub-pixel jitter, micro-correction, and dwell) so synthetic input blends with the real device stream.

Two command protocols are supported, selected at compile time:

- **Hurra binary** (default) — TinyFrame-based protocol used by `hurra-app` / `hurra-bridge`. Targets ≥8k commands/sec at 4 Mbps.
- **Ferrum ASCII** (`make PROTOCOL=ferrum`) — text protocol for compatibility with legacy tools.

## Technology stack and target

- **MCU:** NXP i.MX RT1062 (Cortex-M7, 600 MHz rated, overclocked to 912 MHz by default).
- **Board:** SparkFun MicroMod Teensy on the MicroMod ATP Carrier Board.
- **Host link:** WCH CH343 USB-UART bridge wired to Teensy pins 16/17 (LPUART3, no flow control). Up to 6 Mbaud over USB Full Speed.
- **Toolchain:** ARM GCC (`arm-none-eabi-gcc`), defaulting to the PlatformIO-bundled toolchain at `~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin`.
- **Build system:** Hand-written `Makefile`; no CMake, no PlatformIO project file.
- **Flashing:** `teensy_loader_cli`.
- **Host tests:** System `cc` (no cross-compile) for host-native unit tests.
- **Host tooling:** Python 3 + `pyserial` (+ `pynput` for the aim test).

## Directory layout

```
Makefile                      Build entry point; protocol/clock/baud selectors
core/                         Reset vector, MPU/cache setup, FlexSPI boot data
  startup.c                   Clocks, FPU, SysTick, vector table, newlib stubs
  bootdata.c                  IVT, FlexSPI config, boot data
  imxrt1062_mm.ld             Linker script (16 MB flash, ITCM/DTCM/RAM layout)
include/imxrt.h               i.MX RT1062 register/peripheral header
src/                          Firmware source
  main.c                      Poll loop: USB host → merge → USB device send
  usb_host.c/.h               EHCI host controller (USB2)
  usb_device.c/.h             EHCI device controller (USB1)
  desc_capture.*              Descriptor + HID report-layout capture
  kmbox.c/.h                  LPUART3 DMA RX/TX ring + HID report merge
  kb_layout.c/.h              Minimal HID keyboard report-ID/layout parser
  actions.c/.h                Transport-agnostic injection helpers (act_*)
  humanize.c/.h               Always-on humanization filter
  hurra.c/.h                  Hurra binary parser (TinyFrame) — default protocol
  ferrum.c/.h                 Ferrum ASCII parser (opt-in via PROTOCOL=ferrum)
  proto.h                     Compile-time protocol selector (proto_* aliases)
  led.c/.h                    On-board LED status/heartbeat driver
  gpt_profile.h               GPT2 free-running 1 µs counter
  synth_cadence.h             Cadence decisions for synthetic mouse reports
  third_party/TinyFrame/      TinyFrame framing library (Hurra protocol)
test/                         Host-native unit tests
  humanize_test.c             Humanization filter conservation/cap/adaptive tests
  motion_test.c               Motion-program (automove/bezier) tests
  kb_layout_test.c            Keyboard report-descriptor layout tests
  synth_cadence_test.c        Synthetic-report cadence tests
tools/                        Host-side test/analysis scripts
  ferrum_test.py              Protocol smoke harness
  ferrum_aim_test.py          Closed-loop aim test against on-screen dots
  ferrum_load_test.py         Latency/throughput/integrity load test
  humanization_analyze.py     Kinematic trace analyzer
docs/                         Design docs and implementation plans
```

## Build commands

```sh
make                              # Build firmware.hex (Hurra binary protocol, default)
make PROTOCOL=ferrum              # Build with Ferrum ASCII protocol instead
make CMD_BAUD=2000000             # Override the default command-link baud
make F_CPU=816000000              # Override the core clock (default 912 MHz)
make flash                        # Flash via teensy_loader_cli
make clean                        # Remove objects and build artifacts
make test                         # Run host-native unit tests
```

Build facts:

- Default `F_CPU` is 912 MHz. The part is rated to 600 MHz; 912 MHz is an intentional overclock. `core/startup.c` interpolates core voltage accordingly, capped at 1575 mV (silicon max). Above ~864 MHz exceeds NXP's recommended 1300 mV limit and trades silicon lifetime for clock.
- `IPG = F_CPU/4` is assumed to be a whole MHz (required for the GPT2 1 µs tick and LED scale math).
- Hot-path sources (`main.o`, `usb_host.o`, `usb_device.o`, `kmbox.o`, `humanize.o`, `actions.o`, plus the protocol object) are compiled with `-O2 -ffast-math` instead of `-Os`.
- The linker script places `.fastrun` and `.text*` into ITCM (instruction tightly-coupled memory) and `.dmabuffers` into non-cacheable OCRAM for DMA coherency.

## Test commands

Host-native unit tests (no hardware):

```sh
make test
```

This builds and runs:
- `test/humanize_test.c` — conservation, idle gate, per-frame cap, field-clip carry, adaptive feed-rate EWMA.
- `test/motion_test.c` — `act_motion_move_dur` and `act_motion_bezier` endpoint conservation, cancel behavior.
- `test/synth_cadence_test.c` — synthetic-report cadence logic.

Hardware-in-the-loop tests (require a flashed Teensy, real mouse on USB host port, and `hurra-app` / `hurra-bridge` PTY for default Hurra builds):

```sh
pip install pyserial

# Smoke test via the bridge (default Hurra firmware)
tools/ferrum_test.py ~/.hurra-bridge.tty smoke

# Smoke test direct (PROTOCOL=ferrum build)
tools/ferrum_test.py /dev/tty.usbserial-XXXX smoke

# Closed-loop aim test
pip install pynput
tools/ferrum_aim_test.py ~/.hurra-bridge.tty

# Load test
tools/ferrum_load_test.py ~/.hurra-bridge.tty

# Humanization kinematic analysis
tools/humanization_analyze.py trace.txt --baseline human.txt
```

## Architecture and runtime

### Startup

`core/startup.c` runs from reset: configures FlexRAM, copies `.text.itcm`/`.data` from flash, enables FPU, sets up the MPU/cache, starts SysTick (1 ms), starts the USB1 PLL, and sets `F_CPU` via `set_arm_clock()`. It then calls `main()`.

### Main loop (`src/main.c`)

1. Wait for a USB device on the host port and capture its descriptors.
2. Set up device-side USB enumeration from the captured descriptors.
3. Enter the infinite poll loop:
   - Service the PIT0 tick (adaptive humanization timing).
   - Poll USB device completions.
   - Drain UART command input (`kmbox_poll_fast` / `kmbox_poll_heavy`).
   - Poll each host interrupt IN endpoint; on completion, merge the real report with any pending injected state and send it to the host PC.
   - Forward any device interrupt OUT traffic to the host-side interrupt OUT endpoint.
   - Emit synthetic mouse/keyboard reports when the real mouse is silent but injected state is pending.
   - Update the heartbeat LED rate based on UART/error/temperature state.

### USB proxy

- `usb_host.c` drives the i.MX RT1062 EHCI controller as a USB host (USB2 PHY).
- `usb_device.c` drives the EHCI device controller (USB1 PHY).
- `desc_capture.c` performs control transfers to fetch device/config/HID report/string/BOS/MS-OS descriptors and stores them in `captured_descriptors_t`.
- Composite devices with up to 7 interrupt IN and 7 interrupt OUT endpoints are supported; see `docs/specs/2026-05-17-usb-device-accuracy-design.md` for the evolution of this support.

### Command transport (`src/kmbox.c`)

- LPUART3 on pins 16/17, 24 MHz UART clock, DMA RX into a 1024-byte ring, DMA TX from a 64-byte buffer fed by a 256-byte software ring.
- ISRs only clear flags; SEVONPEND wakes the main loop's `WFE` on UART activity.
- Auto-reset to the boot baud (`CMD_BAUD`) after 5 s of RX idle.
- Deferred baud changes apply only when TX is fully idle.

### Protocols (`src/proto.h`)

`proto.h` aliases a common `proto_*` API to either `hurra_*` or `ferrum_*` implementations, so `kmbox.c` has no `#ifdef` at call sites. Both protocols must define the same notification symbols; for Ferrum, physical-only telemetry hooks are no-op stubs.

### Injection and actions (`src/actions.c`)

- `act_move(dx, dy)` — inject a relative mouse motion.
- `act_button_set`, `act_click`, `act_wheel` — mouse buttons/wheel.
- `act_kb_down/up/press/isdown/mask` — keyboard state.
- `act_set_invert_x/y`, `act_set_swap_xy` — coordinate transforms applied in `act_move`.
- `act_phys_mask_mouse/key/unmask_all` — physical-input masking (KMBox Net `mask` semantics).
- `act_motion_move_dur` / `act_motion_bezier` — time-bounded motion programs; stepped from `act_motion_tick()` in the poll loop.

### Humanization (`src/humanize.c`)

Always-on filter applied only to injected mouse deltas:

- Perpendicular correlated micro-noise scaled by current speed.
- 127-count per-frame ceiling with carry (no teleport frames).
- Sub-pixel residual carry (anti-quantization).
- Idle gate: zero input eventually produces zero output.
- Timing jitter on the PIT reload value.
- Adaptive feed rate: measures real mouse report arrival intervals via GPT2, EWMA-smoothes them, rejects dropouts/bursts, and slews the PIT base period toward the measured interval.

`HZ_ADAPTIVE_NOISE` is 0 by default; normalized-speed envelope code is compiled out unless explicitly enabled.

## Code style guidelines

- C11, no C++.
- No dynamic allocation in firmware; use static buffers and fixed-size tables.
- No `printf`/`sprintf`/`atoi` in firmware; use small integer formatters when needed (see `ferrum.c`).
- Prefer `uintN_t`/`intN_t`. Avoid `bool` in hot-path structs where packing matters.
- Hot paths are annotated `__attribute__((section(".fastrun")))` and/or compiled with `-O2 -ffast-math`.
- Cold/error paths are annotated `__attribute__((cold, noinline))` to keep them out of ITCM.
- Use `__builtin_expect` sparingly on hot-path branches.
- Avoid 64-bit division (`__aeabi_uldivmod`) in latency-critical code; the project explicitly removed it from the PIT/GPT math paths.
- Use `volatile` only for hardware registers and IRQ-shared variables.
- Place DMA buffers in `.dmabuffers` (`__attribute__((section(".dmabuffers"), aligned(...)))`). The linker asserts that this section stays within the 64 KB non-cacheable MPU window.
- Use `dsb`/`isb` after MPU changes and after W1C register writes in ISRs to avoid spurious re-entry.
- Comments explain *why*, not what. Keep them factual and concise.

## Testing instructions

1. **Before committing a C change**, run `make clean && make` for both protocols:
   ```sh
   make clean && make
   make clean && make PROTOCOL=ferrum
   ```
2. **Run host tests:** `make test`. All three must pass.
3. **If you changed Hurra wire types or payloads**, verify byte compatibility with `hurra-app/include/hurra_types.h`. Cross-repo type codes must match exactly; see `KMBOX_NET_FIRMWARE_REQUIREMENTS.md` §8.
4. **If you changed the merge path, humanization, or motion programs**, run the hardware tests (aim test + load test) and compare kinematic output with `tools/humanization_analyze.py` against a human baseline.
5. **If you changed USB enumeration/descriptor handling**, test with a composite gaming device and, if possible, a Logitech HID++ device (interrupt OUT traffic).

## Security and safety considerations

- This firmware is a USB HID proxy with intentional input-injection capability. Treat it as security-sensitive: a compromised host app can synthesize arbitrary mouse/keyboard input.
- The device enumerates as a generic HID composite device; anti-cheat/EDR fingerprinting is not a design goal, but descriptor accuracy is maintained for correctness.
- The overclocked core runs hot. `main.c` includes a tempmon guard that flags over-temperature via LED/telemetry but **does not downclock** (live re-clocking would corrupt F_CPU-derived timebases). Do not disable or weaken this warning path.
- The firmware intentionally does **no host-side smoothing, masking, or physical-input synthesis** for KMBox Net; all such behavior lives in the firmware. Do not push these semantics to the host bridge.
- Physical-input masking (`TYPE_PHYS_MASK`) suppresses the user's real input downstream while still allowing injected input on the same control. Ensure the merge path applies masking **before** injection and that physical-only telemetry is captured **before** masking.
- Avoid introducing timing side channels or deterministic patterns in the humanization filter. The RNG is seeded from DWT cycle counter and OCOTP UID per power cycle.

## Common gotchas for agents

- **PIT clock is 24 MHz PERCLK**, not `F_CPU/4`. Any PIT math must use `PIT_CLK_HZ = 24000000`.
- **GPT2 prescaler derives from `F_CPU/4` (IPG)** and must produce a 1 µs tick. The build asserts `F_CPU > 528 MHz` because the `set_arm_clock` fallback below that does not program dividers correctly.
- **Protocol selector is compile-time.** You cannot switch protocols at runtime. Add new wire features to both `hurra.c` and the `proto.h` aliases; Ferrum stubs are acceptable for features it does not support.
- **Hot paths live in ITCM.** Large cold functions should be marked `cold`/`noinline` so the linker does not waste ITCM on them.
- **DMA coherency relies on MPU non-cacheability.** Do not move `.dmabuffers` out of the first 64 KB of OCRAM or remove the linker `ASSERT` without widening `core/startup.c` MPU region 10.
- **`make flash` requires `teensy_loader_cli` on PATH.** The Makefile does not build it.

## Files agents touch most often

| File | Typical change |
|------|----------------|
| `src/hurra.c` | New Hurra frame types/listeners, telemetry emitters, stats |
| `src/ferrum.c` | New Ferrum ASCII commands or replies |
| `src/proto.h` | New `proto_*` aliases shared by both protocols |
| `src/kmbox.c` | Merge-path behavior, masking, monitoring, synthetic reports, DMA/UART tuning |
| `src/kb_layout.c/.h` | Keyboard report-descriptor parsing (report-ID / boot layout) |
| `src/usb_host.c` | EHCI host interrupt IN/OUT endpoint halt/error recovery |
| `src/usb_device.c` | EHCI device endpoint halt/error handling, SET/CLEAR_FEATURE |
| `src/actions.c/.h` | New injection actions, motion programs, mask state |
| `src/humanize.c/.h` | Filter algorithm, timing jitter, adaptive feed rate |
| `src/main.c` | Poll-loop ordering, PIT/adaptive timing, thermal guard |
| `Makefile` | New source files, build flags, protocol/baud selectors |
| `test/*` | Unit tests for changed modules |

## Communication with the main agent

If a task is ambiguous, state the ambiguity and the safest default in your summary. Do not guess hardware behavior, wire protocol layouts, or cross-repo invariants. When in doubt, cite the relevant file/line or design doc in `docs/`.
