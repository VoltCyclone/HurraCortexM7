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
HIT_THRESHOLD_PX = 15        # within this distance = target hit
TIMEOUT_S        = 3.0       # give up on a target after this long
TICK_MS          = 12        # period between km.move sends (~83 Hz)
PULL_GAIN        = 0.35      # step size = clamp(dist * gain, 1, max)
STEP_MAX_PX      = 40        # cap on per-tick movement
COOLDOWN_MS      = 200       # idle pause between targets, lets cursor settle


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
    line = f"km.move({dx}, {dy})\r\n".encode("ascii")
    try:
        ser.write(line)
    except serial.SerialTimeoutException:
        pass  # firmware backpressure — drop this tick, next one will resend


class AimTest:
    def __init__(self, ser):
        self.ser   = ser
        self.mouse = MouseController()

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

        send_move(self.ser, sx, sy)
        d.iterations += 1
        self.root.after(TICK_MS, self._tick)

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
        print(f"\n=== Run complete ===")
        print(f"Total time: {total:.2f} s")
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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial device, e.g. /dev/tty.usbserial-0001")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.1)
    AimTest(ser).run()


if __name__ == "__main__":
    main()
