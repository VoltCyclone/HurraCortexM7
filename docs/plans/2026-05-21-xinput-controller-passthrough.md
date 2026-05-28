# Xbox XInput Controller Passthrough + Injection

Status: planned, awaiting implementation green light.

## Goal

Add Xbox One / Xbox Series controller passthrough to the existing USB MITM firmware,
with PC-driven injection over the existing Ferrum / Hurra UART link.

Topology:

```
Xbox-licensed XInput pad (USB2 host port)
        |
        v
  i.MX RT1062 MITM
  (LPUART3 from PC)
        |
        v
Xbox One / Series console (USB1 device port)
```

The MITM clones the upstream pad's descriptors verbatim. Vendor control transfers
(XID security IC handshake, MS OS descriptors) flow through `handle_passthrough()`
in `src/usb_device.c` without modification — already works today, no change required.
The GIP authentication challenge (0x06) lives in interrupt-IN/OUT data and is also
forwarded byte-for-byte because the injection path only mutates GIP input report
frames (0x20).

## Locked scope

| Aspect | Decision |
|---|---|
| Target console | Xbox One / Xbox Series, wired |
| Upstream pad | Xbox-licensed XInput controller (vendor class 0xFF/0x47/0xD0 family) |
| Topology | Single upstream device. Pad mode is **exclusive** with mouse/kb mode. |
| Device identity to console | Verbatim descriptor clone of the upstream pad |
| Injection model | Forward + inject. Override semantics. |
| Button semantics | Replace on controlled bits: `merged = (upstream & ~mask) | (val & mask)` |
| Axis semantics | Per-axis controlled flag; injected value replaces upstream when set |
| Mode selector | Compile-time flag `DEVICE_MODE={MOUSE_KB,PAD}` parallel to `PROTOCOL` |
| Latency target | Best effort first, measure later |
| GIP frame filter | Mutate only when `report[0]==0x20 && report[2]!=0 && report[3]>=14` |

## v1 command surface

Both Ferrum (ASCII) and Hurra (binary) protocols expose the same primitives. Mouse/kb
commands are simply absent in PAD-mode firmware builds.

> **Superseded by Hurra protocol** — MAKCU has been removed. See [Hurra binary protocol spec](../specs/2026-05-23-hurra-binary-protocol-design.md) for the binary opcode surface.

| Ferrum | Hurra opcode | Meaning |
|---|---|---|
| `km.btn(mask, action)` | 0xC0 | Press (action=1) or release (action=0) buttons in `mask`. Held until released. |
| `km.btn(mask)` | 0xC0 get | Return current PC-controlled state of those bits |
| `km.axis(id, value)` | 0xC1 | Set axis to absolute value. id ∈ [0..5] = LT, RT, LX, LY, RX, RY |
| `km.axis(id)` | 0xC1 get | Return current controlled value for axis (or 0 if not controlled) |
| `km.btn.click(mask, count, delay_ms)` | 0xC2 | Press + scheduled release. v1 implements `count=1`; larger counts are accepted but treated as 1. |
| `km.gp.reset()` | 0xC3 | Clear all injection state (release all buttons, un-control all axes) |

Out of scope for v1: multi-press click sequences (count > 1), per-axis "ease to center"
release scheduling, console→pad OUT injection (rumble override), PS4/PS5 profile,
GIP state callbacks back over UART (`proto_notify_pad_state`).

## Architecture: Pragmatic balance

New self-contained gamepad module + thin additions to existing files under
`#if DEVICE_MODE_PAD` guards. No extraction or restructuring of existing mouse/kb code.
Mouse/kb path is unchanged in the MOUSE_KB build.

When a second profile (PS4/PS5) lands, the natural next refactor is to extract a
`gp_actions.c` and a profile dispatch layer. We pay that cost when the second profile
exists, not speculatively.

### GIP Input Report layout (MS-GIPUSB)

```
byte[0]   0x20            frame type = Input Report
byte[1]   flags
byte[2]   sequence        1..255 wrapping, 0 reserved for keepalive
byte[3]   payload length  14 for a standard input frame
byte[4..5]  buttons       uint16 LE
byte[6..7]  left trigger  uint16 LE, 0..1023
byte[8..9]  right trigger uint16 LE, 0..1023
byte[10..11] LX            int16 LE
byte[12..13] LY            int16 LE
byte[14..15] RX            int16 LE
byte[16..17] RY            int16 LE
```

Total frame on wire: 18 bytes (4-byte GIP header + 14-byte payload).

Button bit map (per MS-GIPUSB):

```
buttons LSB:  Y=0x80  X=0x40  B=0x20  A=0x10  View=0x08  Menu=0x04  KeepAlive=0x02  _=0x01
buttons MSB:  RSB=0x80  LSB=0x40  RB=0x20  LB=0x10  D-R=0x08  D-L=0x04  D-D=0x02  D-U=0x01
```

Guide button (Xbox) and Share button (Series X/S) arrive as separate GIP messages,
not part of the 0x20 input frame. Out of scope for v1.

### Dispatch key

Pad merge dispatches on **`iface_class == 0xFF`** (vendor), not on `iface_protocol`.
The protocol byte varies across Xbox controller revisions (0x00, 0x02, 0xD0 all seen
in the wild); the class byte is stable.

## File plan

### New files

- `src/device_mode.h` — translates the Makefile `-DDEVICE_MODE_PAD=1` define into a
  reusable macro consumed across the codebase. Plus an `#error` guard for missing/invalid
  selection.
- `src/gp_xinput.h` — public API and constants:
  - `GIP_REPORT_LEN`, `GIP_OFF_*` byte offsets, `GP_AXIS_*` IDs, Hurra opcodes
  - `bool gip_is_data_frame(const uint8_t *buf, uint8_t len)` (inline-candidate)
  - `void xinput_merge_report(uint8_t *buf, uint8_t len)`
  - `uint8_t xinput_build_synthetic(uint8_t *dst)`
  - `void xinput_seq_sync(uint8_t upstream_seq)`
  - `bool xinput_has_pending(void)`
  - `void xinput_reset(void)`
- `src/gp_xinput.c` — implements the above. Reads `g_gp_*` state from `actions.c`
  directly via `extern`. File-static sequence counter `s_seq` (uint8, 1..255 wrap, 0 reserved).

### Modified files

- `src/actions.h` — append under `#if DEVICE_MODE_PAD`:
  ```c
  extern uint16_t g_gp_btn_mask;
  extern uint16_t g_gp_btn_val;
  extern bool     g_gp_axis_set[6];
  extern int16_t  g_gp_axis_val[6];

  void act_gp_btn(uint16_t mask, uint8_t action);
  void act_gp_axis(uint8_t axis_id, int16_t value);
  void act_gp_btn_click(uint16_t mask, uint8_t count, uint32_t delay_ms);
  void act_gp_reset(void);
  ```
- `src/actions.c` — implement the above plus a `gp_click_sched_t` (mask, deadline_ms,
  pressed); `act_init()` zeroes pad state in PAD mode.
- `src/kmbox.c` — add under `#if DEVICE_MODE_PAD`:
  - `kmbox_cache_gp_endpoint(const captured_descriptors_t *desc)` — find first
    `iface_class==0xFF` interface with `interrupt_in_ep != 0`, cache EP + maxpkt.
  - PAD branch in `kmbox_merge_report`: dispatch on `iface_class == 0xFF`; if
    `gip_is_data_frame(report, len)` then `xinput_seq_sync(report[2])` and
    `xinput_merge_report(report, len)`. Set `merged_this_cycle = true` whether or not
    the frame was a data frame (the upstream frame is still being forwarded).
  - PAD branch in `kmbox_send_pending`: only when `!merged_this_cycle &&
    xinput_has_pending() && cached_gp_ep != 0`, build an 18-byte synthetic frame and
    call `usb_device_send_report(cached_gp_ep, synth, 18)`.
  - PAD branch in `kmbox_poll_fast`: drive `gp_click_sched_t` deadlines.
- `src/kmbox.h` — export `kmbox_cache_gp_endpoint()` under PAD guard.
- `src/main.c`:
  - Add `uint8_t iface_class` to `ep_mapping_t` and populate it from
    `desc.ifaces[i].iface_class` in the ep_map setup loop.
  - SET_PROTOCOL loop: add `if (desc.ifaces[i].iface_class != 3) continue;`
    (unconditional — semantically correct in both modes; HID-only class request).
  - PIT smooth init block (lines ~196-222): wrap in `#if !DEVICE_MODE_PAD`.
  - `kmbox_cache_endpoints(&desc)` → switch to `kmbox_cache_gp_endpoint(&desc)` under
    `#if DEVICE_MODE_PAD`.
  - `kmbox_merge_report` call gains `iface_class` argument.
- `src/ferrum.c` — append under `#if DEVICE_MODE_PAD`:
  - `cmd_gp_btn`, `cmd_gp_axis`, `cmd_gp_btn_click`, `cmd_gp_reset` handlers
  - Four `name_is()` entries in `dispatch()` (`"btn"`, `"axis"`, `"btn.click"`, `"gp.reset"`)
  - The mouse/kb command handlers (`cmd_move`, `cmd_button`, etc.) remain present but
    are dead-code in a PAD build because no caller invokes them and LTO eliminates them.
    If linker errors surface from references to `g_buttons` etc. that don't exist in PAD
    mode, wrap the affected handlers behind `#if !DEVICE_MODE_PAD`.
- `src/hurra.c` — append `h_gp_btn`, `h_gp_axis`, `h_gp_btn_click`, `h_gp_reset` and
  switch cases 0xC0–0xC3 under same guard.
- `Makefile`:
  ```make
  DEVICE_MODE ?= MOUSE_KB
  PROTOCOL    ?= FERRUM

  DEFINES += -DPROTOCOL_$(PROTOCOL)
  ifeq ($(DEVICE_MODE),PAD)
    DEFINES += -DDEVICE_MODE_PAD=1
    SRC     += src/gp_xinput.c
  else ifeq ($(DEVICE_MODE),MOUSE_KB)
    DEFINES += -DDEVICE_MODE_PAD=0
  else
    $(error DEVICE_MODE must be MOUSE_KB or PAD)
  endif
  ```
  Add `gp_xinput.o` to the hot-path list (-O2 -ffast-math) when present.

### Untouched files

`src/proto.h`, `src/usb_host.{c,h}`, `src/usb_device.{c,h}`, `src/desc_capture.{c,h}`,
`src/smooth.c`, `src/humanize.c`. In particular, `handle_passthrough()` in
`usb_device.c` already proxies all vendor control transfers — XID auth and MS OS
descriptor exchanges work without any change.

## Signature change

`kmbox_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len)`
becomes
`kmbox_merge_report(uint8_t iface_class, uint8_t iface_protocol, uint8_t *report, uint8_t len)`.
Only caller is `main.c`. Single-site update.

## Override semantics — exact formulas

### Buttons (replace on controlled bits)

```c
uint16_t upstream = (uint16_t)(report[4] | (report[5] << 8));
uint16_t merged   = (upstream & ~g_gp_btn_mask)
                  | (g_gp_btn_val & g_gp_btn_mask);
report[4] = (uint8_t)(merged & 0xFF);
report[5] = (uint8_t)(merged >> 8);
```

PC can both press and suppress physical buttons within `g_gp_btn_mask`.
Bits not in the mask flow through unmodified.

### Axes (per-axis replace)

For each axis i in 0..5: if `g_gp_axis_set[i]`, overwrite the two LE bytes at the
axis offset with `g_gp_axis_val[i]`. Otherwise the upstream value passes through.

### Synthetic frame emission

Fires from `kmbox_send_pending()` only when:
- `!merged_this_cycle` (no upstream frame was merged this poll cycle)
- `xinput_has_pending()` (some override is currently active)
- `cached_gp_ep != 0`

The synthetic frame:
```
buf[0] = 0x20
buf[1] = 0x00
buf[2] = next_seq()        // 1..255 wrap, never 0
buf[3] = 14
buf[4..5]  = g_gp_btn_val & g_gp_btn_mask
buf[6..9]  = trigger bytes from g_gp_axis_val[0..1] (or 0 if !axis_set)
buf[10..17] = stick bytes from g_gp_axis_val[2..5] (or 0 if !axis_set)
```

`next_seq()` is the file-static `s_seq` in `gp_xinput.c`. `xinput_seq_sync(upstream_seq)`
sets `s_seq` from the upstream sequence number after every merged upstream frame, so
when the controller goes idle and we start synthesizing, our seq continues from where
the upstream left off rather than restarting from 1.

## Build matrix

All four combinations must compile clean:

```
make DEVICE_MODE=MOUSE_KB PROTOCOL=hurra    # current default behavior
make DEVICE_MODE=MOUSE_KB PROTOCOL=ferrum   # Ferrum ASCII opt-in
make DEVICE_MODE=PAD      PROTOCOL=hurra    # XInput + Hurra (new)
make DEVICE_MODE=PAD      PROTOCOL=ferrum   # XInput + Ferrum (new)
```

## Interface 1 (audio iso) and Interface 2 (bulk FW update)

Decision: ignore. The descriptor blob is replayed verbatim, so the console sees
both interfaces and can enumerate alt-setting 0 (zero-endpoint) without issue. The
console only issues `SET_INTERFACE` to alt 1 in two cases:

- Interface 1 alt 1 (audio): only when a headset is detected via the GIP status
  channel. With no headset on the physical pad, this never fires.
- Interface 2 alt 1 (bulk firmware update): only during OTA flows that the user
  must explicitly initiate. With no support on the device side, the controller's
  USB device controller will STALL — acceptable failure mode.

Neither path is exercised during normal gaming. No work required in v1.

## Risk register

| # | Risk | Mitigation |
|---|---|---|
| R1 | Wrong dispatch byte (iface_protocol differs across pad revisions) | Use `iface_class == 0xFF`, which is stable across all licensed XInput controllers. |
| R2 | Synthetic frame seq number desync with upstream | `xinput_seq_sync(report[2])` runs on every merged upstream frame so the synth counter tracks upstream. |
| R3 | Mouse/kb dead code in PAD build refs `g_buttons` etc. that don't exist | Globals stay defined in MOUSE_KB; in PAD they're guarded out. If ferrum.c handlers fail to compile in PAD mode, wrap them under `#if !DEVICE_MODE_PAD`. |
| R4 | EP maxpkt mismatch (captured 20-byte EP vs. 18-byte synth) | Send 18 bytes; ChipIdea handles short packets. If `cached_gp_maxpkt < 18`, clamp. |
| R5 | LTO eliminates a static used only via `extern` | None of the new symbols are static; the `g_gp_*` globals have external linkage. |
| R6 | First synthetic frame seq=0 (illegal) | `next_seq()` skips 0 on wrap. Initial `s_seq=0` increments to 1 on first call. |

## Implementation order

1. `Makefile` + `src/device_mode.h` — wire up the build flag. Verify all four
   build combinations compile clean with no other changes.
2. `src/gp_xinput.h` + `src/gp_xinput.c` skeleton — empty function bodies, just
   to keep the linker happy under DEVICE_MODE=PAD.
3. `src/actions.{h,c}` PAD blocks — globals + `act_gp_*` implementations.
4. `src/gp_xinput.c` core — `gip_is_data_frame`, `xinput_merge_report`,
   `xinput_build_synthetic`, `xinput_seq_sync`, `xinput_has_pending`, `xinput_reset`.
5. `src/kmbox.c` PAD branches — `kmbox_cache_gp_endpoint`, merge branch,
   send_pending branch, click-tick.
6. `src/main.c` — `ep_mapping_t.iface_class`, SET_PROTOCOL guard, PIT guard,
   cache call switch, merge_report signature update.
7. `src/ferrum.c` — `cmd_gp_*` handlers and dispatch entries.
8. `src/hurra.c` — `h_gp_*` handlers and switch cases.
9. Build verification: all four combinations clean.
10. Hardware bring-up (user): real XInput pad + Xbox console + UART host.

## Verification

- `make DEVICE_MODE=MOUSE_KB PROTOCOL=FERRUM` produces a functionally identical
  build to the current default (no behavioral change to the mouse/kb path).
- `make DEVICE_MODE=PAD PROTOCOL=hurra` builds and links without warnings.
- `make DEVICE_MODE=PAD PROTOCOL=ferrum` builds and links without warnings.
- (Future) Unit test for `gip_is_data_frame` and `xinput_merge_report` against
  a captured 18-byte Xbox frame buffer.

## Estimated diff size

~80 lines modified in existing files + ~400 lines new code across `gp_xinput.{h,c}`,
`actions.{h,c}` additions, and command handlers.
