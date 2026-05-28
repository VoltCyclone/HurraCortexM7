# imxrtnsy Project Memory

## Project
Bare-metal USB proxy firmware for NXP i.MX RT1062. Man-in-the-middle USB HID device that enumerates devices on USB2 host port, replays descriptors on USB1 device port, and forwards/injects HID reports.

## Board Target
- **MicroMod Teensy + ATP Carrier** — only supported board. No Ethernet, no PSRAM, VBUS via carrier jumper.
- Single build tree at repo root:
  - `Makefile` → `-DARDUINO_TEENSY_MICROMOD`, F_CPU=816MHz, `core/imxrt1062_mm.ld` (16128K flash, model id 0x26), `--mcu=TEENSY_MICROMOD`.
  - Host link: CP2102C USB-UART bridge over LPUART3 on Teensy pins 16/17 (ATP carrier UART_RX2/UART_TX2). Moved off pins 0/1 after suspected pad damage.

## Protocol
- **Default: Hurra binary protocol** — TinyFrame-based, targeting ≥8 k cmds/sec at 4 Mbps. Built by `make` (no flag needed). Implementation in `src/hurra.c` + `src/third_party/TinyFrame/`.
- **Opt-in: Ferrum ASCII text protocol** (`make PROTOCOL=ferrum`) — (https://ferrumllc.github.io/print.html)
  - Wire: `km.<name>(<args>)\r\n` (also accepts `\n` only). Alias `m(x,y)` for move.
  - Default baud 115200, resets to 115200 every power cycle. `km.baud(N)` to bump.
  - No command echo, no `>>>` prompt — that's Software API only.
  - `km.version()` → `kmbox: Ferrum\r\n`
  - ~25 commands across mouse, keyboard, locks, callbacks. Implementation in `src/ferrum.c`.
  - Parser: line accumulator + tokenizer + dispatch table in `src/ferrum.c`
- Actions: transport-agnostic `act_*` functions in `src/actions.c` (drive `kmbox_inject_*`)
- Software API (3 Mbaud + PC-side virtual layer): out of scope, requires associated PC app.

## Key Files
- `src/hurra.c` / `src/hurra.h` — Hurra binary protocol (TinyFrame-based), default protocol
- `src/ferrum.c` / `src/ferrum.h` — Ferrum ASCII parser, line accumulator, dispatch table, callbacks (opt-in via `PROTOCOL=ferrum`)
- `src/actions.c` / `src/actions.h` — transport-agnostic `act_*` action helpers
- `src/kmbox.c` — UART transport (LPUART3 on pins 16/17, no flow control), DMA, injection state, HID report merging
- `src/smooth.c` — Smooth motion queue, bezier, humanization, sub-pixel accumulation
- `src/usb_host.c` / `src/usb_device.c` — EHCI host + device controllers
- `src/main.c` — Main loop: poll → merge → send

## Build
- `make` — builds firmware.hex for MicroMod (Hurra protocol, default)
- `make PROTOCOL=ferrum` — builds with Ferrum ASCII protocol instead
- `make flash` — flashes via teensy_loader_cli
- `make clean` — removes objects + artifacts
- ARM GCC via PlatformIO toolchain
- Hot-path files (-O2 + -ffast-math): kmbox, hurra, ferrum, actions, smooth, usb_host, usb_device, humanize
