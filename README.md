# Hurra v2 — USB HID man-in-the-middle

Bare-metal firmware for the **SparkFun MicroMod Teensy** (NXP i.MX RT1062) that sits between a USB HID device and your computer. It enumerates a real HID device (mouse, keyboard, controller) on its USB host port, replays that device to the host PC on its device port, and lets you **inject your own mouse and keyboard input** on top of the live HID stream over a serial link.

Injected motion is passed through an always-on humanization filter (sub-pixel jitter, micro-correction, and dwell) so synthetic input blends with the real device stream.

Two command protocols are supported:

- **Hurra binary** (default) — fast TinyFrame-based protocol, driven by the host app.
- **Ferrum ASCII** (`make PROTOCOL=ferrum`) — text protocol for compatibility with legacy tools.

The host-side companion app is [`hurra-app`](https://github.com/VoltCyclone/hurra-app) (`hurra-bridge`), which talks the Hurra protocol and also exposes a Ferrum-compatible virtual COM port for older tooling.

## How it works

```
   USB HID device ──→ Teensy USB-host ──┐
                                        │  (firmware proxies + injects)
   Host PC USB ←── Teensy USB-device ───┘
                          ↑
                          │ injected input
                          │
   Host PC USB ──→ CH343 ──→ Teensy LPUART3 (D16/D17)
```

The firmware forwards every real HID report through unchanged, merges in any input you inject over the serial link, and sends the combined stream to the host PC.

## Hardware

- **SparkFun MicroMod Teensy** on the **MicroMod ATP Carrier Board**.
- **WCH CH343** USB-UART bridge wired to Teensy `RX2`/`TX2` (D16/D17 → LPUART3; ATP carrier `UART_RX2`/`UART_TX2` headers). USB Full Speed, up to 6 Mbaud, 64-byte bulk packets.
- A USB HID device (mouse, keyboard, controller) on the Teensy's USB **host** port.
- The Teensy USB **device** port connected to the host PC.

## Command protocols

**Hurra binary (default).** TinyFrame framing — SOF `0x68`, 1-byte ID/LEN/TYPE, CRC16, little-endian payloads. Driven by `hurra-app` / `hurra-bridge`; see that repo for the host API. Targets ≥8k commands/sec at 4 Mbps over the CH343 link. The firmware boots at **4 Mbaud** (matching the bridge's default, so no `--baud` flag is needed); `km.baud(N)` raises the rate, and the firmware falls back to the 4 Mbaud boot default after the link goes idle.

**Ferrum ASCII (`make PROTOCOL=ferrum`).** `\r\n`-terminated text commands at 115200 baud (reset to 115200 on every power cycle). Reference: <https://ferrumllc.github.io/print.html>.

```
TX: km.version()\r\n
RX: kmbox: Ferrum\r\n
TX: km.move(10, -5)\r\n          # write — no reply
TX: m(2, 0)\r\n                  # alias for km.move
```

## Build & flash

```sh
make                 # build firmware.hex (Hurra binary protocol — default)
make PROTOCOL=ferrum # build with the Ferrum ASCII protocol instead
make flash           # flash via teensy_loader_cli
make clean           # remove objects and build artifacts
```

Requirements:

- **ARM GCC** — the Teensyduino-bundled toolchain at `~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin` by default. Edit the `Makefile` if yours lives elsewhere.
- **`teensy_loader_cli`** on your `$PATH`.

## Test

`tools/ferrum_test.py` speaks Ferrum ASCII. Point it at the serial port of a `PROTOCOL=ferrum` build, **or** at the `hurra-bridge` PTY symlink (`~/.hurra-bridge.tty`) when running the default Hurra firmware — not directly at a Hurra build's port.

```sh
pip install pyserial

# Smoke test via the bridge (default Hurra firmware)
tools/ferrum_test.py ~/.hurra-bridge.tty smoke

# Smoke test direct (PROTOCOL=ferrum build)
tools/ferrum_test.py /dev/tty.usbserial-XXXX smoke
```

The smoke test handshakes `km.version()`, nudges the mouse, exercises the buttons and wheel, and validates the read forms.

**Closed-loop aim test** — drives the cursor toward on-screen dots:

```sh
pip install pyserial pynput
tools/ferrum_aim_test.py /dev/tty.usbserial-XXXX
```

**Load test** — measures latency, throughput, and integrity of the command channel under sustained load:

```sh
tools/ferrum_load_test.py ~/.hurra-bridge.tty
```

**Humanization analyzer** — compares a captured motion trace against a real human baseline to check the kinematic signatures anti-cheat detectors look for:

```sh
tools/humanization_analyze.py trace.txt --baseline human.txt
```

A host-native unit test for the humanization filter also runs without hardware:

```sh
make test
```

## Layout

```
Makefile                      ARM GCC build, 816 MHz, -O2 hot path
core/                         reset vector, MPU/cache setup, FlexSPI boot data
include/imxrt.h               i.MX RT1062 register/peripheral header
src/main.c                    poll loop: USB host → merge → USB device send
src/usb_host.c/.h             EHCI host controller (USB2)
src/usb_device.c/.h           EHCI device controller (USB1)
src/desc_capture.*            descriptor + HID report-layout capture
src/kmbox.c/.h                LPUART3 DMA RX/TX ring + HID report merge
src/hurra.c/.h                Hurra binary parser (TinyFrame) — default protocol
src/ferrum.c/.h               Ferrum ASCII parser (opt-in: PROTOCOL=ferrum)
src/proto.h                   compile-time protocol selector
src/actions.c/.h              transport-agnostic injection helpers (act_*)
src/humanize.c/.h             always-on humanization filter (jitter, micro-correction, dwell)
src/led.c/.h                  on-board LED status/heartbeat driver
src/third_party/TinyFrame/    TinyFrame framing library (Hurra protocol)
tools/ferrum_test.py          protocol smoke harness
tools/ferrum_aim_test.py      closed-loop aim test against on-screen dots
tools/ferrum_load_test.py     command-channel latency/throughput/integrity load test
tools/humanization_analyze.py kinematic trace analyzer vs. a human baseline
```
