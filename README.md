# imxrtnsy

Bare-metal USB HID man-in-the-middle firmware for the **SparkFun MicroMod Teensy** (NXP i.MX RT1062). Enumerates a real USB HID device on the host port, replays it on the device port to the Mac/PC, and accepts **Hurra binary protocol** commands over UART to inject mouse/keyboard input on top of the live HID stream.

Drop-in compatible with software written for the [Ferrum](https://ferrumllc.github.io/) and km.box B-family devices. The Ferrum ASCII protocol is still supported as an opt-in (`PROTOCOL=ferrum`).

## Hardware

- **SparkFun MicroMod Teensy** on the **MicroMod ATP Carrier Board**.
- **Silicon Labs CP2102C** USB-UART bridge wired to Teensy `RX2`/`TX2` (D16/D17 → LPUART3; ATP carrier UART_RX2/UART_TX2 headers).
- A USB HID device (mouse, keyboard, controller) on the Teensy's USB host port.
- Teensy USB device port to the host PC.

```
   USB HID device ──→ Teensy USB-host ──┐
                                        │  (firmware proxies + injects)
   Host PC USB ←── Teensy USB-device ───┘
                          ↑
                          │ km.* commands
                          │
   Host PC USB ──→ CP2102C ──→ Teensy LPUART3 (D16/D17)
```

## Wire protocol

The default protocol is **Hurra** — a TinyFrame-based binary protocol targeting ≥8 k commands/sec at 4 Mbps. The legacy **Ferrum** ASCII protocol (`km.<cmd>(…)\r\n`, 115200 baud) is available via `make PROTOCOL=ferrum`.

### Ferrum (ASCII, opt-in: `make PROTOCOL=ferrum`)

ASCII text, `\r\n`-terminated, 115200 baud (resets to 115200 every power cycle). No echo, no `>>> ` prompt. Reference: <https://ferrumllc.github.io/print.html>.

```
TX: km.version()\r\n
RX: kmbox: Ferrum\r\n

TX: km.move(10, -5)\r\n          # write — no reply
TX: km.left(1)\r\n               # left button down
TX: km.left()\r\n                # read — replies 1\r\n
TX: km.click(0)\r\n              # left click (0=L 1=R 2=M 3=rear 4=front)
TX: km.wheel(1)\r\n              # one wheel tick up
TX: km.down(4)\r\n               # press 'A' (HID HUT 1.5 codes)
TX: km.init()\r\n                # release every key/button
TX: m(2, 0)\r\n                  # alias for km.move
```

Full command surface: `version`, `move`, `m`, `left`/`right`/`middle`/`side1`/`side2`, `click`, `wheel`, `lock_m{l,r,m,s1,s2,x,y}`, `catch_xy`, `down`/`up`/`press`/`multidown`/`multiup`/`multipress`, `isdown`, `mask`, `init`, `buttons`/`axes`/`keys` (callbacks), `baud`.

## Build & flash

```sh
make                    # produces firmware.hex (Hurra protocol, default)
make PROTOCOL=ferrum    # opt-in: Ferrum ASCII protocol
make flash              # flashes via teensy_loader_cli
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
src/ferrum.c/.h       Ferrum ASCII parser + dispatch + callbacks
src/actions.c/.h      transport-agnostic injection helpers (act_*)
src/smooth.c/.h       bezier-smoothed motion queue
src/humanize.c/.h     sub-pixel jitter + dwell
tools/ferrum_test.py        protocol smoke harness
tools/ferrum_aim_test.py    closed-loop aim test against on-screen dots
```
