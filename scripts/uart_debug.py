#!/usr/bin/env python3
"""
UART debug probe for the kmbox firmware.

Goal: figure out *where* km.version() falls off the wire without needing
to see the D30/D31/D24 LEDs. Each step narrows the failure to one of:

    [1] firmware totally silent      → no bytes at any baud, any ending
    [2] wrong-baud TX                → bytes only land at non-default baud
    [3] line-ending sensitivity      → only responds to one of \\r / \\n / \\r\\n
    [4] intermittent / partial       → some attempts succeed, others empty

DTR/RTS are explicitly kept low — asserting them resets the Teensy via the
CP2102C on the ATP carrier (the bridge wires DTR to the program/reset line).

Usage:
    python3 scripts/uart_debug.py                  # autodetect
    python3 scripts/uart_debug.py --port /dev/cu.usbserial-0001
"""

import argparse
import fcntl
import glob
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("error: pyserial not installed. `pip3 install pyserial`", file=sys.stderr)
    sys.exit(2)


VERSION_REPLY = b"kmbox: Ferrum\r\n"
IOSSIOSPEED = 0x80045402  # macOS arbitrary-baud ioctl, see uart_bench.py


def force_baud(s, baud):
    if sys.platform != "darwin":
        return
    fcntl.ioctl(s.fileno(), IOSSIOSPEED, struct.pack("I", int(baud)))


def autodetect_port():
    cands = sorted(
        glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/cu.SLAB_USB*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )
    return cands[0] if cands else None


def open_port(port, baud):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.05
    s.write_timeout = None
    s.dtr = False
    s.rts = False
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    force_baud(s, baud)
    s.reset_input_buffer()
    s.reset_output_buffer()
    return s


def hex_dump(buf, max_bytes=80):
    if not buf:
        return "<empty>"
    head = buf[:max_bytes]
    tail = f" …(+{len(buf) - max_bytes} more)" if len(buf) > max_bytes else ""
    return " ".join(f"{b:02X}" for b in head) + tail + f"   ascii={head!r}"


def drain_for(s, seconds):
    deadline = time.time() + seconds
    out = bytearray()
    while time.time() < deadline:
        chunk = s.read(256)
        if chunk:
            out.extend(chunk)
    return bytes(out)


def set_baud(s, baud):
    s.baudrate = baud
    force_baud(s, baud)
    s.reset_input_buffer()


def step1_listen_only(s):
    print(f"\n[1] listen-only @ {s.baudrate} for 2.0s "
          f"(any unsolicited bytes from firmware?)")
    s.reset_input_buffer()
    got = drain_for(s, 2.0)
    print(f"    rx: {hex_dump(got)}")
    return got


def step2_baud_sweep(s):
    print("\n[2] probe km.version()\\r\\n at each plausible baud "
          "(host side; firmware stays at CMD_BAUD=115200)")
    bauds = (115200, 230400, 460800, 921600,
             1000000, 2000000, 3000000, 4000000)
    found = []
    for baud in bauds:
        set_baud(s, baud)
        s.write(b"km.version()\r\n")
        s.flush()
        got = drain_for(s, 1.0)
        match = "✓ EXACT" if got == VERSION_REPLY else \
                ("partial" if got else "—")
        print(f"    {baud:>7d}: {match:<8} {hex_dump(got)}")
        if got:
            found.append((baud, got))
    return found


def step3_line_endings(s):
    print("\n[3] line-ending variants @ 115200 "
          "(spec accepts \\r\\n or just \\n; firmware also handles \\r alone)")
    set_baud(s, 115200)
    for label, le in (("CR    ", b"\r"),
                      ("LF    ", b"\n"),
                      ("CRLF  ", b"\r\n")):
        s.reset_input_buffer()
        s.write(b"km.version()" + le)
        s.flush()
        got = drain_for(s, 0.5)
        match = "✓ EXACT" if got == VERSION_REPLY else \
                ("partial" if got else "—")
        print(f"    {label}: {match:<8} {hex_dump(got)}")


def step4_repeat(s, n=20):
    print(f"\n[4] {n}x probes @ 115200 (k=valid, .=garbage, _=empty)")
    set_baud(s, 115200)
    legend = []
    first_byte_latencies_ms = []
    hits = misses = empty = 0
    for i in range(n):
        s.reset_input_buffer()
        t0 = time.perf_counter()
        s.write(b"km.version()\r\n")
        s.flush()
        # Time-to-first-byte
        first_byte = None
        deadline = time.time() + 0.5
        buf = bytearray()
        while time.time() < deadline:
            chunk = s.read(256)
            if chunk:
                if first_byte is None:
                    first_byte = time.perf_counter()
                buf.extend(chunk)
                if buf.endswith(b"\r\n"):
                    break
        got = bytes(buf)
        if got == VERSION_REPLY:
            hits += 1
            legend.append("k")
            if first_byte:
                first_byte_latencies_ms.append((first_byte - t0) * 1000)
        elif got:
            misses += 1
            legend.append(".")
        else:
            empty += 1
            legend.append("_")
        sys.stdout.write(legend[-1])
        sys.stdout.flush()
    print()
    print(f"    {hits}/{n} valid, {misses}/{n} garbage, {empty}/{n} empty")
    if first_byte_latencies_ms:
        first_byte_latencies_ms.sort()
        med = first_byte_latencies_ms[len(first_byte_latencies_ms) // 2]
        print(f"    first-byte latency: "
              f"min={first_byte_latencies_ms[0]:.1f}ms "
              f"med={med:.1f}ms "
              f"max={first_byte_latencies_ms[-1]:.1f}ms")


def diagnose(silent_on_all_bauds, found_at_bauds):
    print("\n[diagnosis]")
    if silent_on_all_bauds:
        print("  Firmware emits NOTHING at any baud.")
        print("  → Either: (a) firmware is not actually in its main poll loop,")
        print("    (b) LPUART6 TX side is broken (TX pin/IOMUX/CTRL.TE),")
        print("    (c) board not powered or wired upside-down (CP2102C TX/RX swapped),")
        print("    (d) wrong firmware image flashed.")
        print("  Next step: power-cycle the board, re-run, and confirm enumeration")
        print("  via USB host port still works.")
        return
    if found_at_bauds and all(b != 115200 for b, _ in found_at_bauds):
        b, _ = found_at_bauds[0]
        print(f"  Firmware responds at {b}, not 115200.")
        print(f"  → CMD_BAUD doesn't match the firmware's actual rate, OR a previous")
        print(f"    km.baud({b}) wasn't reset by power-cycle.")
        return
    print("  Mixed signals — see step-by-step results above.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (autodetect if omitted)")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if not port:
        print("error: no serial port found; pass --port", file=sys.stderr)
        sys.exit(1)
    print(f"port: {port}")

    s = open_port(port, 115200)
    try:
        time.sleep(0.3)  # let CP2102C latency timer settle
        step1_listen_only(s)
        found = step2_baud_sweep(s)
        step3_line_endings(s)
        step4_repeat(s, n=20)
        diagnose(silent_on_all_bauds=(len(found) == 0),
                 found_at_bauds=found)
    finally:
        # Return host side to 115200 so the next tool starts clean
        try:
            set_baud(s, 115200)
        except Exception:
            pass
        s.close()


if __name__ == "__main__":
    main()
