#!/usr/bin/env python3
"""Ferrum closed-loop aim test.

Spawns a fullscreen Tkinter window with red dots. Reads the Mac cursor
position via pynput, sends km.move(dx, dy) over the CP2102C serial link
to drive the cursor toward each target with a smooth-pull controller
(step size proportional to remaining distance — fast at first, slows
as it approaches). Dots turn green when hit.

Setup required:
    1. CP2102C plugged into Mac as the command channel  (/dev/tty.usbserial-*)
    2. Teensy USB device port plugged into Mac          (USB HID injection out)
    3. A real USB mouse plugged into the Teensy carrier's USB HOST port
       — the firmware is a HID proxy; it won't run main() without an
       upstream device to enumerate.  See src/main.c:117.

Usage:
    pip install pynput pyserial
    tools/ferrum_aim_test.py /dev/tty.usbserial-0001

Keys:
    Space   start the run (cycles through all dots in order)
    Click   add a custom target at the click position
    R       reset all dots to red
    ESC     quit
"""

import argparse
import math
import sys
import time
import tkinter as tk

try:
    import serial
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")

try:
    from pynput.mouse import Controller as MouseController
except ImportError:
    sys.exit("pynput required:  pip install pynput")


# Aim controller tuning
HIT_THRESHOLD_PX     = 15    # within this distance = target hit
TIMEOUT_S            = 3.0   # give up on a target after this long
TICK_MS_DEFAULT      = 6     # period between cursor-feedback ticks (~166 Hz)
PULL_GAIN            = 0.35  # step size = clamp(dist * gain, 1, max)
STEP_MAX_PX          = 40    # cap on per-tick movement (total budget)
CMD_STEP_PX_DEFAULT  = 4     # max |dx|,|dy| per km.move(); larger tick steps are split
COOLDOWN_MS          = 200   # idle pause between targets, lets cursor settle


class Dot:
    __slots__ = ("oval_id", "screen_x", "screen_y", "name", "hit",
                 "time_to_hit_s", "iterations")
    def __init__(self, oval_id, screen_x, screen_y, name):
        self.oval_id       = oval_id
        self.screen_x      = screen_x
        self.screen_y      = screen_y
        self.name          = name
        self.hit           = False
        self.time_to_hit_s = None
        self.iterations    = 0


def send_move(ser, dx, dy):
    # No try/except: write_timeout backpressure throttles the caller; we don't
    # want to silently drop moves and lose the closed-loop signal.
    ser.write(f"km.move({dx}, {dy})\r\n".encode("ascii"))


def split_move(sx, sy, max_step):
    """Yield (dx, dy) sub-moves summing to (sx, sy), each |axis| <= max_step.

    Distributes the motion evenly across N sub-moves via cumulative rounding so
    the path is straight and rounding error never exceeds 1 px.
    """
    n = max(1, (max(abs(sx), abs(sy)) + max_step - 1) // max_step)
    prev_x = prev_y = 0
    for i in range(1, n + 1):
        cur_x = round(sx * i / n)
        cur_y = round(sy * i / n)
        yield cur_x - prev_x, cur_y - prev_y
        prev_x, prev_y = cur_x, cur_y


class AimTest:
    def __init__(self, ser, tick_ms=TICK_MS_DEFAULT, cmd_step_px=CMD_STEP_PX_DEFAULT):
        self.ser         = ser
        self.tick_ms     = tick_ms
        self.cmd_step_px = cmd_step_px
        self.mouse       = MouseController()
        self.sent        = 0
        self.run_start_t = 0.0

        self.root = tk.Tk()
        self.root.title("Ferrum Aim Test")
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="#101010")

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.canvas = tk.Canvas(self.root, width=sw, height=sh,
                                bg="#101010", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.status = self.canvas.create_text(
            sw // 2, 32, fill="#cccccc",
            font=("Menlo", 16),
            text="[space] start   [click] add target   [r] reset   [esc] quit"
        )

        self.dots: list[Dot] = []
        self._place_default_grid(sw, sh)

        self.state           = "idle"      # idle | running | between | done
        self.current_idx     = 0
        self.target_start_t  = 0.0
        self.run_start_t     = 0.0

        self.root.bind("<space>",  self.on_space)
        self.root.bind("<Escape>", self.on_escape)
        self.canvas.bind("<Button-1>", self.on_click)
        self.root.bind("r", self.on_reset)
        self.root.bind("R", self.on_reset)

    # ----- dot placement -----

    def _place_default_grid(self, sw, sh):
        # 4 × 2 grid centred on the screen, with some margin
        margin_x = sw // 6
        margin_y = sh // 4
        cols, rows = 4, 2
        for r in range(rows):
            for c in range(cols):
                x = margin_x + c * (sw - 2 * margin_x) // (cols - 1)
                y = margin_y + r * (sh - 2 * margin_y) // (rows - 1)
                self._add_dot(x, y)

    def _add_dot(self, x, y, radius=16):
        name = f"#{len(self.dots) + 1}"
        oid = self.canvas.create_oval(
            x - radius, y - radius, x + radius, y + radius,
            fill="#e02020", outline="#ff8080", width=2,
        )
        self.canvas.create_text(
            x, y + radius + 14,
            fill="#888888", font=("Menlo", 11),
            text=name,
        )
        # Canvas covers full screen on a fullscreen window, so canvas coords
        # match screen coords for cursor comparison.
        self.dots.append(Dot(oid, x, y, name))

    # ----- input handlers -----

    def on_space(self, _evt):
        if self.state in ("idle", "done"):
            self._reset_dots()
            self.state          = "running"
            self.current_idx    = 0
            self.sent           = 0
            self.run_start_t    = time.monotonic()
            self._begin_target()

    def on_escape(self, _evt):
        try:
            self.ser.close()
        except Exception:
            pass
        self.root.destroy()

    def on_click(self, evt):
        if self.state == "idle":
            self._add_dot(evt.x, evt.y)

    def on_reset(self, _evt):
        if self.state in ("idle", "done"):
            self._reset_dots()

    # ----- run -----

    def _reset_dots(self):
        for d in self.dots:
            d.hit           = False
            d.time_to_hit_s = None
            d.iterations    = 0
            self.canvas.itemconfig(d.oval_id, fill="#e02020", outline="#ff8080")
        self._set_status("[space] start   [click] add target   [r] reset   [esc] quit")

    def _begin_target(self):
        if self.current_idx >= len(self.dots):
            self._finish_run()
            return
        d = self.dots[self.current_idx]
        self.canvas.itemconfig(d.oval_id, outline="#ffff00", width=3)
        self.target_start_t = time.monotonic()
        self.root.after(0, self._tick)

    def _tick(self):
        if self.state != "running":
            return
        d = self.dots[self.current_idx]

        cx, cy = self.mouse.position
        dx = d.screen_x - cx
        dy = d.screen_y - cy
        dist = math.hypot(dx, dy)

        self._set_status(
            f"target {self.current_idx + 1}/{len(self.dots)}  "
            f"dist={dist:6.1f}px  iters={d.iterations}"
        )

        if dist <= HIT_THRESHOLD_PX:
            d.hit           = True
            d.time_to_hit_s = time.monotonic() - self.target_start_t
            self.canvas.itemconfig(d.oval_id, fill="#20c020",
                                              outline="#80ff80", width=2)
            self.current_idx += 1
            self.state = "between"
            self.root.after(COOLDOWN_MS, self._after_cooldown)
            return

        if time.monotonic() - self.target_start_t > TIMEOUT_S:
            self.canvas.itemconfig(d.oval_id, fill="#404040",
                                              outline="#808080", width=2)
            self.current_idx += 1
            self.state = "between"
            self.root.after(COOLDOWN_MS, self._after_cooldown)
            return

        # Smooth pull: step proportional to remaining distance, capped.
        step = max(1.0, min(STEP_MAX_PX, dist * PULL_GAIN))
        sx = int(round(dx / dist * step))
        sy = int(round(dy / dist * step))
        if sx == 0 and sy == 0:
            sx = 1 if dx > 0 else (-1 if dx < 0 else 0)
            sy = 1 if dy > 0 else (-1 if dy < 0 else 0)

        for chunk_x, chunk_y in split_move(sx, sy, self.cmd_step_px):
            send_move(self.ser, chunk_x, chunk_y)
            self.sent += 1
        d.iterations += 1
        self.root.after(self.tick_ms, self._tick)

    def _after_cooldown(self):
        self.state = "running"
        self._begin_target()

    def _finish_run(self):
        self.state = "done"
        total = time.monotonic() - self.run_start_t
        hits  = sum(1 for d in self.dots if d.hit)
        avg_t = (
            sum(d.time_to_hit_s for d in self.dots if d.hit) / hits
            if hits else 0.0
        )
        avg_i = (
            sum(d.iterations for d in self.dots if d.hit) / hits
            if hits else 0.0
        )
        rps = self.sent / total if total else 0.0
        print(f"\n=== Run complete ===")
        print(f"Total time: {total:.2f} s")
        print(f"Commands sent: {self.sent}  ({rps:,.0f} km.move/sec, "
              f"tick_ms={self.tick_ms}, cmd_step_px={self.cmd_step_px})")
        print(f"Hits: {hits}/{len(self.dots)}")
        if hits:
            print(f"Avg time-to-hit: {avg_t * 1000:.0f} ms")
            print(f"Avg iterations:  {avg_i:.1f}")
        print("Per-target:")
        for d in self.dots:
            if d.hit:
                print(f"  {d.name:>4}  {d.time_to_hit_s * 1000:5.0f} ms "
                      f"({d.iterations} steps)")
            else:
                print(f"  {d.name:>4}  MISS")
        self._set_status(
            f"done — {hits}/{len(self.dots)} hits in {total:.1f}s   "
            "[space] retry   [r] reset   [esc] quit"
        )

    def _set_status(self, txt):
        self.canvas.itemconfig(self.status, text=txt)

    # ----- runloop -----

    def run(self):
        # Verify the device speaks Ferrum before starting the GUI runloop.
        self.ser.reset_input_buffer()
        self.ser.write(b"km.version()\r\n")
        self.ser.flush()
        deadline = time.monotonic() + 0.5
        buf = b""
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b: continue
            buf += b
            if buf.endswith(b"\r\n"): break
        ver = buf.decode("ascii", "replace").rstrip("\r\n")
        if ver != "kmbox: Ferrum":
            sys.exit(f"device handshake failed: got {ver!r}, expected 'kmbox: Ferrum'")
        print(f"Connected: {ver}")
        self.root.mainloop()


def cmd_load(ser, n, mode):
    """Throughput benchmark. Returns ops/sec."""
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    if mode == "oneway":
        payload = b"km.move(1, 0)\r\n"
        t0 = time.perf_counter()
        for _ in range(n):
            try:
                ser.write(payload)
            except serial.SerialTimeoutException:
                pass
        ser.flush()
        elapsed = time.perf_counter() - t0
        rps = n / elapsed if elapsed else float("inf")
        bps = (len(payload) * n * 8) / elapsed if elapsed else float("inf")
        # Firmware processes one line per USB HID report (~1 kHz cap), so a 5000-cmd
        # burst at 773 cmds/sec leaves a multi-second backlog. Poll km.version() until
        # we get a clean reply or give up after 30 s.
        drain_start = time.perf_counter()
        drain_deadline = drain_start + 30.0
        alive = False
        drain_s = None
        while time.perf_counter() < drain_deadline:
            ser.reset_input_buffer()
            ser.write(b"km.version()\r\n"); ser.flush()
            ack = b""
            probe_deadline = time.perf_counter() + 1.0
            while time.perf_counter() < probe_deadline:
                chunk = ser.read(64)
                if chunk:
                    ack += chunk
                    if ack.endswith(b"\r\n"):
                        break
            if ack.strip().endswith(b"kmbox: Ferrum"):
                alive = True
                drain_s = time.perf_counter() - drain_start
                break
        print(f"\n[load:oneway] {n} km.move() in {elapsed*1000:.1f}ms")
        print(f"    {rps:,.0f} cmds/sec   ({bps/1000:,.1f} kbps wire)")
        if alive:
            print(f"    firmware alive after burst: yes  (queue drained in {drain_s:.2f}s)")
        else:
            print(f"    firmware alive after burst: NO  (no reply within 30s — likely hung)")
        return rps

    payload = b"km.version()\r\n"
    expected = b"kmbox: Ferrum\r\n"
    lats_ms = []
    fails = 0
    t0 = time.perf_counter()
    for _ in range(n):
        t_send = time.perf_counter()
        ser.write(payload); ser.flush()
        buf = b""
        deadline = time.perf_counter() + 0.5
        while time.perf_counter() < deadline:
            chunk = ser.read(64)
            if chunk:
                buf += chunk
                if buf.endswith(b"\r\n"):
                    break
        if buf == expected:
            lats_ms.append((time.perf_counter() - t_send) * 1000)
        else:
            fails += 1
    elapsed = time.perf_counter() - t0
    rps = n / elapsed if elapsed else float("inf")
    print(f"\n[load:rtt] {n} round-trips in {elapsed*1000:.1f}ms")
    print(f"    {rps:,.0f} rtt/sec   ({fails} failed)")
    if lats_ms:
        lats_ms.sort()
        p = lambda q: lats_ms[min(len(lats_ms) - 1, int(len(lats_ms) * q))]
        print(f"    latency ms: min={lats_ms[0]:.2f} "
              f"med={p(0.50):.2f} p95={p(0.95):.2f} max={lats_ms[-1]:.2f}")
    return rps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial device, e.g. /dev/tty.usbserial-0001")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tick-ms", type=int, default=TICK_MS_DEFAULT,
                    help=f"cursor-feedback period in ms (default {TICK_MS_DEFAULT})")
    ap.add_argument("--cmd-step-px", type=int, default=CMD_STEP_PX_DEFAULT,
                    help=f"max |dx|,|dy| per km.move() (default {CMD_STEP_PX_DEFAULT}); "
                         "larger tick steps split into multiple sub-commands")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("aim", help="GUI closed-loop aim test (default)")
    p = sub.add_parser("load", help="benchmark km.move() throughput and report cmds/sec")
    p.add_argument("-n", type=int, default=1000, help="number of commands (default 1000)")
    p.add_argument("--mode", choices=["oneway", "rtt"], default="oneway",
                   help="oneway = fire-and-forget km.move() (default); rtt = round-trip km.version()")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=5.0)
    if args.cmd == "load":
        cmd_load(ser, args.n, args.mode)
        ser.close()
        return
    AimTest(ser, tick_ms=args.tick_ms, cmd_step_px=args.cmd_step_px).run()


if __name__ == "__main__":
    main()
