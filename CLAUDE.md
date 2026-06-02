# imxrtnsy Project Memory

## Project
Bare-metal USB proxy firmware for NXP i.MX RT1062. Man-in-the-middle USB HID device that enumerates devices on USB2 host port, replays descriptors on USB1 device port, and forwards/injects HID reports.

## Board Target
- **MicroMod Teensy + ATP Carrier** — only supported board. No Ethernet, no PSRAM, VBUS via carrier jumper.
- Single build tree at repo root:
  - `Makefile` → `-DARDUINO_TEENSY_MICROMOD`, F_CPU=816MHz, `core/imxrt1062_mm.ld` (16128K flash, model id 0x26), `--mcu=TEENSY_MICROMOD`.
  - Host link: WCH CH343 USB-UART bridge over LPUART3 on Teensy pins 16/17 (ATP carrier UART_RX2/UART_TX2). Moved off pins 0/1 after suspected pad damage. CH343 supports up to 6 Mbaud (USB Full Speed, 64-byte bulk MPS).

## Protocol
- **Default: Hurra binary protocol** — TinyFrame-based (SOF `0x68`, 1-byte ID/LEN/TYPE, CRC16), targeting >=8k cmds/sec at 4 Mbps. Built by `make` (no flag). Implementation in `src/hurra.c` + `src/third_party/TinyFrame/`. Host adapter: `hurra-app` (`hurra-bridge`).
  - Boots at **4 Mbaud** (`CMD_BAUD` default for the hurra build), matching the hurra-bridge/hello default — no `--baud` needed. `km.baud(N)` bumps it; after a bump the firmware auto-resets to the boot default (4 Mbaud) on extended RX idle. Override the build default with `make CMD_BAUD=N`.
- **Opt-in: Ferrum ASCII text protocol** (`make PROTOCOL=ferrum`) — https://ferrumllc.github.io/print.html
  - Wire: `km.<name>(<args>)\r\n` (also accepts `\n` only). Alias `m(x,y)` for move.
  - Default baud 115200, resets to 115200 every power cycle. `km.baud(N)` to bump.
  - No command echo, no `>>>` prompt. `km.version()` -> `kmbox: Ferrum\r\n`.
  - Parser: line accumulator + tokenizer + dispatch table in `src/ferrum.c`.
- Protocol abstraction: `src/proto.h` aliases `proto_*` to the selected parser; `kmbox.c` calls `proto_*` with no `#ifdef`s at call sites.
- Actions: transport-agnostic `act_*` functions in `src/actions.c` (drive `kmbox_inject_*`).

## Key Files
- `src/actions.c` / `src/actions.h` — transport-agnostic `act_*` action helpers
- `src/kmbox.c` — UART transport (LPUART3 on pins 16/17, no flow control), DMA, injection state, HID report merging
- `src/hurra.c` / `src/hurra.h` — Hurra binary protocol parser (TinyFrame), default
- `src/proto.h` — compile-time protocol selector (`proto_*` -> Hurra or Ferrum)
- `src/ferrum.c` / `src/ferrum.h` — Ferrum ASCII parser (opt-in via `PROTOCOL=ferrum`)
- `src/humanize.c` — Always-on humanization filter applied to every injected delta (jitter, micro-correction, sub-pixel carry); the standalone smooth/easing trajectory generator (smooth.c) was retired — humanize.c is now the single humanization path
- `src/usb_host.c` / `src/usb_device.c` — EHCI host + device controllers
- `src/main.c` — Main loop: poll → merge → send

## Build
- `make` — builds firmware.hex for MicroMod (Hurra protocol, default)
- `make PROTOCOL=ferrum` — builds with Ferrum ASCII protocol instead
- `make flash` — flashes via teensy_loader_cli
- `make clean` — removes objects + artifacts
- ARM GCC via PlatformIO toolchain
- Hot-path files (-O2 + -ffast-math): kmbox, hurra (or ferrum), actions, usb_host, usb_device, humanize
