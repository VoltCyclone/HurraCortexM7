#!/usr/bin/env python3
"""
Ferrum UART throughput / latency benchmark.

Connects to the kmbox over the CP2102C USB-UART bridge and runs three tests:

  1. RTT       — sync round-trip time for km.version()
  2. throughput — fire-and-forget km.move() sustained rate
  3. integrity — interleaved km.version() probes during a flood, checking
                 that every reply arrives byte-perfect

Usage:
    python3 scripts/uart_bench.py                       # autodetect, 115200
    python3 scripts/uart_bench.py --port /dev/cu.usbserial-XXXX
    python3 scripts/uart_bench.py --baud 460800 --count 50000
"""

import argparse
import fcntl
import glob
import statistics
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("error: pyserial not installed. `pip3 install pyserial`", file=sys.stderr)
    sys.exit(2)


VERSION_REPLY = b"kmbox: Ferrum\r\n"

# macOS IOSSIOSPEED ioctl: _IOW('T', 2, speed_t). Sets arbitrary baud on the
# tty driver, bypassing termios. Needed because pyserial's in-place baudrate
# change goes through tcsetattr(B921600) which the macOS CP2102C driver
# accepts but doesn't actually apply.
IOSSIOSPEED = 0x80045402


def force_baud(s, baud):
    """Hard-set the baud via IOSSIOSPEED on macOS. No-op elsewhere."""
    if sys.platform != "darwin":
        return
    buf = struct.pack("I", int(baud))
    fcntl.ioctl(s.fileno(), IOSSIOSPEED, buf)


def autodetect_port():
    candidates = sorted(
        glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/cu.SLAB_USB*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )
    return candidates[0] if candidates else None


def open_port(port, baud):
    # Build the port without opening, suppress DTR/RTS assertion, then open.
    # Default pyserial open asserts DTR which resets the Teensy via the
    # CP2102C on most carrier boards (DTR is wired to the program/reset
    # line). dsrdtr=False keeps the OS from re-toggling on attribute change.
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.5
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


def set_baud(s, new_baud):
    """Send km.baud(N) and switch the host baud in-place via tcsetattr.

    We do NOT close/reopen the port: macOS asserts DTR on open, which on
    a Teensy carrier resets the MCU. That would wipe the baud change and
    reboot the firmware back at CMD_BAUD=115200.
    """
    print(f"  switching to {new_baud} baud...")
    s.write(f"km.baud({new_baud})\r\n".encode())
    s.flush()
    time.sleep(0.5)               # firmware applies in next kmbox_poll
    s.baudrate = new_baud         # pyserial calls tcsetattr — no close
    force_baud(s, new_baud)       # IOSSIOSPEED — macOS workaround
    s.reset_input_buffer()
    return s


def diag_baud_transition(s, new_baud):
    """Confirm whether device transitioned to new_baud after km.baud(N).

    Steps:
      1. At current host baud, send km.version() — should already get a reply.
      2. Send km.baud(new_baud). Device should switch.
      3. Stay at old host baud, send km.version(). If we get *any* coherent
         reply, the device did NOT switch. If we get garbage/nothing, good.
      4. Switch host to new_baud. Send km.version(). If we get the right
         reply, device IS at new_baud. If garbage, host-side baud change
         didn't take.
      5. Switch host back to old baud. Send km.version(). If we get the
         reply, device must already be back at old baud (didn't switch
         in step 2 at all). If nothing, device is at new_baud.
    """
    old_baud = s.baudrate
    print(f"  [diag] testing transition {old_baud} -> {new_baud}")

    # Step 1
    s.reset_input_buffer()
    s.write(b"km.version()\r\n")
    r = s.read_until(b"\n")
    print(f"    step 1 (probe @ {old_baud}):       {r!r}")

    # Step 2
    s.write(f"km.baud({new_baud})\r\n".encode())
    s.flush()
    time.sleep(0.5)

    # Step 3: still at old host baud, after device should have switched
    s.reset_input_buffer()
    s.write(b"km.version()\r\n")
    r = s.read_until(b"\n")
    print(f"    step 3 (host stuck @ {old_baud}):  {r!r}  (want b'' or garbage)")

    # Step 4: switch host (force IOSSIOSPEED on macOS)
    s.baudrate = new_baud
    force_baud(s, new_baud)
    confirmed = (s.baudrate == new_baud)
    print(f"    step 4a host baudrate now:         {s.baudrate} (asked {new_baud}, confirmed={confirmed})")
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write(b"km.version()\r\n")
    r = s.read_until(b"\n")
    print(f"    step 4b (host now @ {new_baud}):    {r!r}  (want {VERSION_REPLY!r})")
    step4_ok = (r == VERSION_REPLY)

    # Step 5: switch back to confirm device is NOT at old baud anymore
    s.baudrate = old_baud
    force_baud(s, old_baud)
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write(b"km.version()\r\n")
    r = s.read_until(b"\n")
    print(f"    step 5 (host back @ {old_baud}):   {r!r}  (want b'' if step 4 ok)")

    # Verdict
    print(f"  [diag] result: ", end="")
    if step4_ok:
        print(f"device successfully at {new_baud} ✓")
    else:
        # Best-guess root cause
        if r == VERSION_REPLY:
            print(f"device DID NOT switch — still at {old_baud}. "
                  f"Firmware-side km.baud() path is broken.")
        else:
            print(f"device responds at neither baud — host tcsetattr likely "
                  f"didn't take effect (IOSSIOSPEED issue?). Try `stty -f "
                  f"{s.port} {new_baud}` as a workaround.")

    # Restore device to old baud for any subsequent tests
    s.baudrate = new_baud
    force_baud(s, new_baud)
    time.sleep(0.05)
    s.write(f"km.baud({old_baud})\r\n".encode())
    s.flush()
    time.sleep(0.3)
    s.baudrate = old_baud
    force_baud(s, old_baud)
    s.reset_input_buffer()
    return s


def rtt_test(s, n=100):
    """Synchronous round-trip: km.version() and time the reply."""
    samples = []
    fails = 0
    for _ in range(n):
        s.reset_input_buffer()
        t0 = time.perf_counter_ns()
        s.write(b"km.version()\r\n")
        reply = s.read_until(b"\n")
        t1 = time.perf_counter_ns()
        if reply == VERSION_REPLY:
            samples.append((t1 - t0) / 1000.0)  # microseconds
        else:
            fails += 1
    if not samples:
        print(f"  RTT: ALL FAILED ({fails} bad replies)")
        return
    samples.sort()
    print(f"  RTT over {len(samples)} samples ({fails} bad):")
    print(f"    min  = {samples[0]:>8.1f} µs")
    print(f"    med  = {samples[len(samples)//2]:>8.1f} µs")
    print(f"    p95  = {samples[int(len(samples)*0.95)]:>8.1f} µs")
    print(f"    max  = {samples[-1]:>8.1f} µs")
    print(f"    mean = {statistics.mean(samples):>8.1f} µs")


def throughput_test(s, count):
    """Fire-and-forget km.move(1,0), then km.version() as a barrier.

    The version reply will not come back until the device has parsed every
    preceding command, so the wall time gives the true end-to-end rate
    (line rate + parser overhead), not just how fast the host can stuff
    bytes into the CP2102C's internal IN buffer.
    """
    cmd = b"km.move(1,0)\r\n"
    blob = cmd * count + b"km.version()\r\n"
    s.reset_input_buffer()
    s.reset_output_buffer()
    t0 = time.perf_counter()
    s.write(blob)
    barrier = s.read_until(b"\n", size=len(VERSION_REPLY) + 8)
    t1 = time.perf_counter()
    if barrier != VERSION_REPLY:
        print(f"  throughput: barrier reply bad: {barrier!r}")
        return
    dt = t1 - t0
    bytes_on_wire = len(blob) + len(VERSION_REPLY)
    cps = count / dt
    bps = len(blob) / dt
    line_rate = s.baudrate / 10  # 8N1 = 10 bits/byte
    print(f"  throughput over {count} commands ({len(blob)} TX bytes, "
          f"{len(VERSION_REPLY)} RX barrier):")
    print(f"    elapsed   = {dt*1000:.2f} ms")
    print(f"    cmds/sec  = {cps:>10,.0f}")
    print(f"    TX B/sec  = {bps:>10,.0f}  ({bps/line_rate*100:.1f}% of line rate)")


def integrity_test(s, flood_count=2000, probe_every=100):
    """Flood km.move() and periodically inject km.version() — verify each reply."""
    s.reset_input_buffer()
    s.reset_output_buffer()
    bad = 0
    probes = 0
    t0 = time.perf_counter()
    for i in range(flood_count):
        s.write(b"km.move(1,0)\r\n")
        if i % probe_every == 0:
            s.write(b"km.version()\r\n")
            reply = s.read_until(b"\n")
            probes += 1
            if reply != VERSION_REPLY:
                bad += 1
                if bad <= 3:
                    print(f"    probe {probes} bad reply: {reply!r}")
    t1 = time.perf_counter()
    print(f"  integrity: {flood_count} floods + {probes} probes in {(t1-t0)*1000:.1f} ms")
    print(f"    bad replies = {bad} / {probes}")


SAFE_RESTORE_BAUD = 115200  # firmware always boots here per Ferrum spec


def restore_safe_baud(s):
    """Best-effort: tell the device + CP2102C to return to 115200.

    Runs unconditionally on exit so a crashed/aborted bench doesn't leave
    the bridge stuck at a non-default rate. IOSSIOSPEED state persists on
    the macOS CP2102C driver across pyserial closes — without this call,
    a subsequent tool opening at 115200 may still clock bytes at whatever
    rate the bridge was last set to.
    """
    if not s or not s.is_open:
        return
    try:
        # If the device might be at a non-default rate, try to bring it
        # back over the wire too. We can't know for sure what baud the
        # device is at, so blast km.baud(115200) at both candidate rates.
        for try_baud in (s.baudrate, SAFE_RESTORE_BAUD):
            try:
                s.baudrate = try_baud
                force_baud(s, try_baud)
                s.write(f"km.baud({SAFE_RESTORE_BAUD})\r\n".encode())
                s.flush()
                time.sleep(0.15)
            except Exception:
                pass
        s.baudrate = SAFE_RESTORE_BAUD
        force_baud(s, SAFE_RESTORE_BAUD)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (autodetect if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=10000,
                    help="commands sent in throughput test")
    ap.add_argument("--rtt-samples", type=int, default=200)
    ap.add_argument("--retarget-baud", type=int, default=0,
                    help="after opening at --baud, send km.baud(N) and reopen at N")
    ap.add_argument("--diag-baud", type=int, default=0,
                    help="diagnose where the km.baud(N) transition fails")
    ap.add_argument("--skip", choices=["rtt", "throughput", "integrity"],
                    action="append", default=[])
    args = ap.parse_args()

    port = args.port or autodetect_port()
    if not port:
        print("error: no serial port found; pass --port", file=sys.stderr)
        sys.exit(1)
    print(f"port: {port} @ {args.baud}")

    s = open_port(port, args.baud)
    try:
        # Drain any boot-time noise and let the CP2102C latency timer settle.
        time.sleep(0.5)
        s.reset_input_buffer()

        # Warm up: a few discarded probes to drain any in-flight bytes and to
        # punch through the CP2102C's first-packet delay (default ~16 ms).
        for _ in range(3):
            s.write(b"km.version()\r\n")
            s.read_until(b"\n")

        # Sanity: confirm we're talking to a Ferrum device.
        s.reset_input_buffer()
        s.write(b"km.version()\r\n")
        sanity = s.read_until(b"\n")
        if sanity != VERSION_REPLY:
            print(f"  sanity FAILED: got {sanity!r}")
            print("  (continuing anyway)")
        else:
            print("  sanity OK")

        if args.diag_baud:
            diag_baud_transition(s, args.diag_baud)
            return

        if args.retarget_baud and args.retarget_baud != args.baud:
            s = set_baud(s, args.retarget_baud)
            s.write(b"km.version()\r\n")
            check = s.read_until(b"\n")
            print(f"  post-retarget sanity: {'OK' if check == VERSION_REPLY else f'FAIL {check!r}'}")

        if "rtt" not in args.skip:
            print("\n[rtt]")
            rtt_test(s, args.rtt_samples)

        if "throughput" not in args.skip:
            print("\n[throughput]")
            throughput_test(s, args.count)

        if "integrity" not in args.skip:
            print("\n[integrity]")
            integrity_test(s)
    finally:
        restore_safe_baud(s)
        s.close()


if __name__ == "__main__":
    main()
