# hurra v2 - USB HS

Bare-metal USB HID man-in-the-middle firmware for the **SparkFun MicroMod Teensy** (NXP i.MX RT1062). Enumerates a real USB HID device on the host port, replays it on the device port to the Mac/PC, and accepts **Hurra binary protocol** commands over UART (default) to inject mouse/keyboard input on top of the live HID stream. A **Ferrum ASCII** compatibility mode is available via `make PROTOCOL=ferrum`.

The host-side adapter is [`hurra-app`](https://github.com/VoltCyclone/hurra-app) (`hurra-bridge`), which also exposes a Ferrum-compatible virtual COM port for legacy tools.

## Hardware

- **SparkFun MicroMod Teensy** on the **MicroMod ATP Carrier Board**.
- **WCH CH343** USB-UART bridge wired to Teensy `RX2`/`TX2` (D16/D17 -> LPUART3; ATP carrier UART_RX2/UART_TX2 headers). USB Full Speed, up to 6 Mbaud, 64-byte bulk MPS.
- A USB HID device (mouse, keyboard, controller) on the Teensy's USB host port.
- Teensy USB device port to the host PC.

```
   USB HID device ──→ Teensy USB-host ──┐
                                        │  (firmware proxies + injects)
   Host PC USB ←── Teensy USB-device ───┘
                          ↑
                          │ km.* commands
                          │
   Host PC USB ──→ CH343 ──→ Teensy LPUART3 (D16/D17)
```

## Wire protocol

**Default — Hurra binary (TinyFrame):** SOF `0x68`, 1-byte ID/LEN/TYPE, CRC16. Little-endian payloads. Driven by `hurra-app`/`hurra-bridge`; see that repo for the host API. Targets >=8k commands/sec at 4 Mbps over the CH343 link.

**Compatibility — Ferrum ASCII** (`make PROTOCOL=ferrum`): `\r\n`-terminated text, 115200 baud (resets to 115200 every power cycle). Reference: <https://ferrumllc.github.io/print.html>.

```
TX: km.version()\r\n
RX: kmbox: Ferrum\r\n
TX: km.move(10, -5)\r\n          # write — no reply
TX: m(2, 0)\r\n                  # alias for km.move
```

## Build & flash

```sh
make                 # produces firmware.hex (Hurra binary protocol, default)
make PROTOCOL=ferrum # build with Ferrum ASCII protocol instead
make flash           # flashes via teensy_loader_cli
```

Requires:
- ARM GCC (the Teensyduino-bundled toolchain at `~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin` by default — edit `Makefile` if yours lives elsewhere).
- `teensy_loader_cli` on `$PATH`.

## Test

```sh
pip install pyserial
tools/ferrum_test.py /dev/tty.usbserial-XXXX smoke
```

That handshakes `km.version()`, nudges the mouse, exercises buttons + wheel, and validates the read forms. For a closed-loop aim test that drives the cursor toward on-screen dots:

```sh
pip install pyserial pynput
tools/ferrum_aim_test.py /dev/tty.usbserial-XXXX
```

## Layout

```
Makefile              ARM GCC build, 816 MHz, -O2 hot path
core/                 reset vector, MPU/cache setup, FlexSPI boot data
include/imxrt.h       i.MX RT1062 register/peripheral header
src/main.c            poll loop: USB host → merge → USB device send
src/usb_host.c/.h     EHCI host (USB2)
src/usb_device.c/.h   EHCI device (USB1)
src/desc_capture.*    descriptor + HID report-layout capture
src/kmbox.c/.h        LPUART3 DMA RX/TX ring + HID merge
src/hurra.c/.h        Hurra binary parser (TinyFrame) — default protocol
src/ferrum.c/.h       Ferrum ASCII parser (opt-in: PROTOCOL=ferrum)
src/proto.h           compile-time protocol selector
src/actions.c/.h      transport-agnostic injection helpers (act_*)
src/smooth.c/.h       bezier-smoothed motion queue
src/humanize.c/.h     sub-pixel jitter + dwell
tools/ferrum_test.py        protocol smoke harness
tools/ferrum_aim_test.py    closed-loop aim test against on-screen dots
```
