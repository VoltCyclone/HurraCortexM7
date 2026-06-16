# Hurra-v2 firmware changes for KMBox Net endpoint support

**Status:** Implemented on `fix/humanization-injection` — all four features
(enum/stubs, mask, monitor, smoothed moves) landed and build clean under both
`PROTOCOL=hurra` and `PROTOCOL=ferrum`; host unit tests pass. The host side
(`hurra-app`) still stubs these commands (`warn_once` "firmware-pending") and
must add the encoders/decoders to match the wire layouts below — note the
bezier payload is **14 bytes** (see §3).
**Audience:** Whoever implements the firmware side. Assumes familiarity with the
iMXRT firmware in this repo but not with the host-side work that motivates it.
**Companion (host) repo:** `hurra-app` — branch `feat/kmbox-net-endpoint`.
Host spec: `hurra-app/docs/superpowers/specs/2026-06-09-kmbox-net-endpoint-design.md`.

---

## 1. Background and why these changes exist

`hurra-app` is the host-side bridge. It opens the real serial link to a Hurra
device and exposes an input endpoint that third-party tools talk to. Until now
that endpoint was **VCOM/Ferrum** (a virtual COM port speaking the Ferrum ASCII
protocol). We have added a **second endpoint: KMBox Net**, a UDP protocol used by
a large ecosystem of existing tools.

In KMBox Net the *device* is a UDP server: client tools send fixed-layout command
packets to it and it echoes a 16-byte ACK. The host bridge therefore **emulates a
KMBox Net device** — it binds a UDP socket, accepts those packets, and translates
each into a Hurra binary frame over the serial link to this firmware.

### The hard architectural rule: the device does the work, the host only translates

The host bridge is a **thin translator**. It MUST NOT emulate device behaviour.
There is deliberately **no host-side smoothing, no host-side physical-input
synthesis, and no host-side input masking**. Every behaviour that a KMBox Net
client expects must be performed **on this device**, reached through the Hurra
binary protocol.

Why: the same device must behave identically whether driven through the
VCOM/Ferrum endpoint or the KMBox Net endpoint, and we refuse to maintain a
second, drifting implementation of device behaviour on the host. Keeping the
logic here is the whole point.

### What already works without firmware changes

These KMBox Net commands map cleanly onto existing Hurra frames; the host already
forwards them and **no firmware change is needed**:

| KMBox Net command            | Existing Hurra frame the host sends           |
|------------------------------|-----------------------------------------------|
| `connect`                    | (handshake only; host may probe `TYPE_VERSION`)|
| `mouse_move(x,y)`            | `TYPE_MOUSE_MOVE 0x10`                          |
| `mouse_left/right/middle`    | `TYPE_BTN_LEFT/RIGHT/MIDDLE 0x20/0x21/0x22`     |
| `mouse_wheel(n)`             | `TYPE_MOUSE_WHEEL 0x15`                          |
| `mouse_all(btn,x,y,wheel)`   | `TYPE_MOUSE_MO 0x13`                             |
| `keyboard_all(mod,keys[])`   | `TYPE_KB_MULTIDOWN/UP 0x46/0x47` (+ modifiers)   |
| `reboot`                     | `TYPE_REBOOT 0x04`                              |

### What this document covers

Four KMBox Net capabilities need device behaviour the Hurra protocol does **not**
have today. Until the firmware implements them, the host **ACKs the client and
logs `pending firmware: <cmd>` once** — it does nothing else. This document
specifies the four firmware features and the new wire frames that drive them.

The host has already **reserved the wire type codes** in
`hurra-app/include/hurra_types.h`. This firmware MUST use the **same numeric
values** so the two repos stay byte-compatible. The codes are currently free in
this firmware's `enum` in `src/hurra.c` (verified: 0x1B, 0x1C, 0x68, 0x77, and
0x86–0x88 are unused).

---

## 2. Conventions (shared wire contract)

- **Transport:** TinyFrame over the 4 Mbaud serial link, exactly as today. Each
  feature is one new `TYPE_*` value plus a payload, registered with
  `TF_AddTypeListener` in `hurra_init()` (`src/hurra.c`).
- **Endianness:** all multi-byte payload fields are **little-endian**, matching
  every existing Hurra frame (`rd_i16le` / `wr_i16le` helpers in `src/hurra.c`).
- **Oneway vs. reply:** the movement and mask frames are *oneway* (no reply,
  like `TYPE_MOUSE_MOVE`). The telemetry frames are *unsolicited pushes* from the
  device (like `TYPE_TLM_AXIS`), gated by an enable toggle.
- **Type code allocation (MUST match host `hurra_types.h`):**

  | New `TYPE_*`                 | Value  | Direction         | Purpose                              |
  |------------------------------|--------|-------------------|--------------------------------------|
  | `TYPE_MOUSE_MOVE_DUR`        | `0x1B` | host → device     | duration-stepped (smoothed) move     |
  | `TYPE_MOUSE_MOVE_BEZIER`     | `0x1C` | host → device     | cubic Bézier move over a duration    |
  | `TYPE_PHYS_MASK`             | `0x68` | host → device     | enable/disable physical-input mask   |
  | `TYPE_CB_PHYS`               | `0x77` | host → device     | enable/disable physical-only telemetry|
  | `TYPE_TLM_PHYS_AXIS`         | `0x86` | device → host     | physical-only mouse motion telemetry |
  | `TYPE_TLM_PHYS_BUTTONS`      | `0x87` | device → host     | physical-only button telemetry       |
  | `TYPE_TLM_PHYS_KB`           | `0x88` | device → host     | physical-only keyboard telemetry     |

Add these to the `enum` in `src/hurra.c` (lines ~16–68) in their numeric slots,
matching the existing block comments (Mouse 0x10–0x2F, Locks 0x60–0x6F,
callbacks 0x70–0x7F, telemetry 0x80–0x8F).

---

## 3. Feature A — Smoothed / duration moves (`automove`, `bezier`)

### KMBox Net semantics
- `kmNet_mouse_move_auto(x, y, time_ms)` — move a total delta `(x,y)` spread over
  `time_ms`, with a human-like (non-linear, slightly jittered) velocity profile.
- `kmNet_mouse_move_beizer(x, y, ms, x1, y1, x2, y2)` — move along a cubic Bézier
  curve from the origin to `(x,y)` over `ms`, with control points `(x1,y1)` and
  `(x2,y2)` (relative to the start).

Both produce a *trajectory over time*, not a single instantaneous jump.

### Why the firmware must own this
The host must not synthesise the trajectory (that would be host-side emulation,
and it would also flood the serial link with hundreds of tiny `MOUSE_MOVE`
frames). The device already has the right machinery: `src/humanize.c` /
`humanize.h` applies a per-frame jitter/acceleration profile to injected motion,
and the injection path (`kmbox.c: kmbox_take_injection`) already meters injected
delta out across USB frames. What is missing is a **time-bounded, path-aware
source** of injected delta.

### New frames

`TYPE_MOUSE_MOVE_DUR 0x1B` — payload **6 bytes**:
```
int16  dx        (total X delta, LE)
int16  dy        (total Y delta, LE)
uint16 dur_ms    (duration in milliseconds, LE; 0 = treat as immediate)
```

`TYPE_MOUSE_MOVE_BEZIER 0x1C` — payload **14 bytes** (7 little-endian int16):
```
int16 dx     (endpoint X delta, LE)
int16 dy     (endpoint Y delta, LE)
uint16 dur_ms
int16 x1     (control point 1 X, relative to start, LE)
int16 y1
int16 x2     (control point 2 X, relative to start, LE)
int16 y2
```
> Corrected from an earlier "12 bytes": the field list enumerates 7 int16 =
> 14 bytes, which a 2-control-point cubic genuinely needs. The firmware
> (`l_mouse_move_bezier`) validates `msg->len == 14`, and the host's
> `hurra_types.h:0x1C` comment already documents the same 7-field layout.

### Firmware requirements
1. Add `TYPE_MOUSE_MOVE_DUR` and `TYPE_MOUSE_MOVE_BEZIER` to the enum and register
   listeners `l_mouse_move_dur` / `l_mouse_move_bezier` in `hurra_init()`.
2. Each listener validates `msg->len` (6 and 12 respectively; bump
   `s_payload_invalid` and `return TF_STAY` on mismatch, mirroring
   `l_mouse_move`), decodes the fields, and **starts a "motion program"** — an
   internal generator that, on each injection tick, emits the next incremental
   delta along the trajectory until the total/duration is consumed.
3. The generator runs through the **existing injection path** so it composes with
   real-mouse passthrough and respects the adaptive feed rate. Concretely, it
   should feed `act_move()` / the same `inject.mouse_dx/dy` accumulation that
   `act_move` drives, sliced per tick. Reuse `humanize_*` for the velocity
   profile of `automove`; for `bezier`, evaluate the cubic at the time-normalized
   `t` for the current tick and inject the delta since the last evaluated point.
4. **Timing source:** use the same millisecond clock the rest of the firmware
   uses (`millis()` is already used in `kmbox.c`), or the GPT2 1 MHz counter
   (`gpt_profile_us()`) for finer resolution. Step the program from the main loop
   / injection tick, not from an ISR.
5. **Superseding:** a new `MOUSE_MOVE_DUR`/`BEZIER`/`MOUSE_MOVE` while a program
   is running should replace the in-flight program (last-writer-wins), matching
   how a real user redirecting the mouse overrides a prior gesture. Do not queue.
6. **Locks/inverts still apply:** the generated motion must pass through the same
   `act_move` transforms (`s_invert_x/y`, swap) and lock mask the normal path
   uses, so behaviour is identical to a stream of manual moves.
7. Oneway: no reply frame.

### Acceptance
- Host sends `MOUSE_MOVE_DUR(dx=200, dy=0, dur_ms=500)`; the downstream PC sees
  ~200px of rightward motion delivered smoothly over ~0.5s, not one jump.
- Bézier with bowed control points visibly curves.
- Sending a plain `MOUSE_MOVE` mid-program cancels the remaining trajectory.

---

## 4. Feature B — Physical-input monitoring (`monitor`)

### KMBox Net semantics
`kmNet_monitor(port)` plus the `kmNet_monitor_mouse_*()` / `kmNet_monitor_keyboard(vk)`
query calls let a client observe the **physical** keyboard/mouse the user is
operating — the real HID reports arriving on the device's USB-host side, *before*
any injected state is merged in. Clients use this to read the user's true input.

### The problem in the current firmware
The existing telemetry (`TYPE_TLM_AXIS 0x80`, `TYPE_TLM_BUTTONS 0x81`,
`TYPE_TLM_KB 0x83`), toggled by `TYPE_CB_BUTTONS/AXES/KEYS 0x74–0x76`, reports the
**merged** state, not the physical-only state. See `src/kmbox.c`
`kmbox_merge_report()`:

```c
report[doff] |= inject.mouse_buttons;     // physical OR injected
proto_notify_buttons(report[doff]);        // <-- emits MERGED buttons
...
proto_notify_axes((int16_t)done_dx, (int16_t)done_dy, w_tlm);  // merged motion
```

So there is no way for a client to distinguish the user's real input from what
the host injected. `monitor` cannot be implemented on top of the existing
telemetry.

### New frames

`TYPE_CB_PHYS 0x77` — payload **1 byte**: `uint8 enable` (0/1). Enables/disables
emission of the three physical-only telemetry frames below. Oneway, mirrors the
existing `l_cb_btn/axes/keys` listeners.

Device → host pushes (emitted only while `CB_PHYS` is enabled):

`TYPE_TLM_PHYS_AXIS 0x86` — payload **5 bytes** (match `TYPE_TLM_AXIS` layout):
```
int16 dx        (physical mouse X delta this report, LE)
int16 dy        (physical mouse Y delta this report, LE)
int8  wheel     (physical wheel delta)
```

`TYPE_TLM_PHYS_BUTTONS 0x87` — payload **1 byte**:
```
uint8 buttons   (physical button bitmap, BEFORE injected OR)
```

`TYPE_TLM_PHYS_KB 0x88` — payload **8 bytes** (match `TYPE_TLM_KB`):
```
uint8 modifier
uint8 reserved
uint8 keys[6]   (physical HID keycodes held, BEFORE injected merge)
```

### Firmware requirements
1. Add the enum values; add a `l_cb_phys` listener for `TYPE_CB_PHYS` that sets a
   `bool s_cb_phys_enabled` (default false), registered in `hurra_init()`.
2. In `kmbox_merge_report()` (`src/kmbox.c`), **capture the physical report
   fields before the injected state is OR'd/added in**, and when
   `s_cb_phys_enabled`, emit the `TLM_PHYS_*` frames with those pre-merge values.
   The capture point is right at function entry / before the
   `report[doff] |= inject.mouse_buttons` line (~`kmbox.c:781`) and before the
   motion-merge math that produces `done_dx/done_dy`.
3. Use a parallel set of emit helpers (e.g. `proto_notify_phys_buttons/axes/keys`,
   added to `src/proto.h` and implemented in `src/hurra.c` alongside the existing
   `hurra_notify_*`). The Ferrum protocol build does not need them — guard or
   provide no-op stubs so the `PROTOCOL_FERRUM` build still links (see the
   `proto.h` selector; both protocols must define the same `proto_*` symbols).
4. Emit only on change or per-report as appropriate; the existing
   `proto_notify_*` are called per merged report, so per-report physical emission
   is acceptable and simplest. Rate is bounded by the physical poll rate.
5. The per-button/per-key *query* calls in the KMBox API (`monitor_mouse_left`,
   `monitor_keyboard(vk)`, …) are satisfied on the **host** by tracking the latest
   `TLM_PHYS_*` state and answering queries from that cache — **no extra firmware
   frame needed** for the queries. The firmware only needs to stream the three
   `TLM_PHYS_*` frames.

### Acceptance
- With `CB_PHYS` enabled and nothing injected, moving the real mouse produces
  `TLM_PHYS_AXIS` frames whose deltas equal the physical motion.
- Injecting a move via the host does **not** appear in `TLM_PHYS_*` (only in the
  merged `TLM_AXIS`), proving the tap is pre-merge.

---

## 5. Feature C — Physical-input masking (`mask` / `unmask_all`)

### KMBox Net semantics
`kmNet_mask_mouse_left/right/middle/side1/side2/x/y/wheel(enable)` and
`kmNet_mask_keyboard(vk)` / `kmNet_unmask_keyboard(vk)` / `kmNet_unmask_all()`
let a client **block specific physical inputs** from reaching the downstream
(gaming) PC — e.g. swallow the user's real left-click while still allowing
injected clicks. `unmask_all` clears every active mask.

### The problem in the current firmware
The state containers exist but are **never enforced in the merge path**:
- `g_lock_mask` (a `uint16_t` bitmap, `src/actions.c:14`) is set by the
  `TYPE_LOCK_* 0x60–0x66` listeners (`src/hurra.c` `lock_listener`, bit order
  `ml=0, mr=1, mm=2, ms1=3, ms2=4, mx=5, my=6`). Today it only gates **injection**,
  not physical passthrough.
- `g_masked_keys[]` (`src/actions.c:25`, capacity `ACT_MAX_DISABLED_KEYS=32`),
  managed by `act_kb_mask(key, mode)` — also not consulted in the merge path.

Grepping `src/kmbox.c` confirms neither `g_lock_mask` nor `g_masked_keys` is
referenced there, so physical input is never actually suppressed.

### New frame

`TYPE_PHYS_MASK 0x68` — payload **3 bytes**:
```
uint8 domain    (0 = mouse button/axis, 1 = keyboard key)
uint8 code      (domain 0: index 0..6 matching the lock-bit order
                          ml,mr,mm,ms1,ms2,mx,my; plus 7 = wheel
                 domain 1: the HID keycode to mask)
uint8 enable    (1 = mask/suppress this input, 0 = unmask)
```

A dedicated "unmask all" is expressed as `domain=0xFF, code=0, enable=0` (the
firmware clears `g_lock_mask` and `g_masked_keys[]` entirely). The host maps
`kmNet_unmask_all()` to this.

> Rationale for a new frame rather than reusing `TYPE_LOCK_*`: the existing
> `LOCK_*` bits are also used by Ferrum's lock semantics (injection gating); we do
> not want to overload their meaning. `PHYS_MASK` is explicitly about suppressing
> *physical passthrough*. It MAY internally reuse the `g_lock_mask` bits for the
> mouse domain (since the bit order already matches), but the enforcement is new.

### Firmware requirements
1. Add `TYPE_PHYS_MASK` to the enum; register `l_phys_mask` in `hurra_init()`.
   Validate `msg->len == 3`. Update the mask state:
   - domain 0, code 0..6 → set/clear the matching `g_lock_mask` bit (reuse the
     existing bit order). Code 7 (wheel) → a new `g_mask_wheel` bool, or an
     extra bit in `g_lock_mask`.
   - domain 1 → `act_kb_mask(code, enable)` (already exists).
   - domain 0xFF → clear all: `g_lock_mask = 0`, reset `g_masked_keys` via the
     existing reset path, clear `g_mask_wheel`.
2. **Enforce in `kmbox_merge_report()`** (`src/kmbox.c`) — this is the core new
   behaviour. When building the report that goes downstream, **zero out the
   physical contribution** of any masked input *before* it is sent and before
   `proto_notify_*`:
   - For each masked mouse button bit, clear that bit from the physical
     `report[doff]` button byte.
   - For masked X / Y / wheel, zero the corresponding physical delta from the
     report.
   - For masked keys, remove them from the physical keyboard report's key array.
   Do this to the physical fields, then apply the injected merge as usual — so
   injected input on a masked control still works, only the user's physical input
   is suppressed. (This is the KMBox semantic: the cheat injects, the user's real
   input is blocked.)
3. Interaction with monitoring (Feature B): the `TLM_PHYS_*` frames should report
   the **true physical input as seen before masking** (so a client can still
   observe what the user pressed even while it is masked downstream). Capture
   physical telemetry first, then apply masking, then apply injection.
4. Oneway: no reply.

### Acceptance
- Mask mouse-left, then physically click: the downstream PC sees no left-click.
- With the same mask, an injected left-click via the host still registers.
- `monitor` (if enabled) still shows the physical left-click in `TLM_PHYS_BUTTONS`.
- `unmask_all` restores normal passthrough for everything.

---

## 6. Suggested implementation order

1. **Enum + stubs first:** add all seven `TYPE_*` values and empty listeners that
   validate length and `return TF_STAY`. Confirms wire compatibility with the host
   (the host will stop logging `pending firmware` once it gets ACK-equivalent
   behaviour; note Hurra frames are oneway so "ACK" is just non-rejection).
2. **Feature C (mask)** — smallest, highest-value, and exercises the merge-path
   edit that Feature B also needs.
3. **Feature B (monitor)** — adds the pre-merge tap + `TLM_PHYS_*` emit + the
   `proto.h` symbol additions (and Ferrum no-op stubs).
4. **Feature A (smoothed moves)** — the motion-program generator; largest, and
   independent of B/C.

Each feature is independently shippable; the host already degrades gracefully
(ACK + log) for any not-yet-implemented command.

---

## 7. Files you will touch

| File | Change |
|------|--------|
| `src/hurra.c` | Add 7 enum values; add listeners `l_mouse_move_dur`, `l_mouse_move_bezier`, `l_phys_mask`, `l_cb_phys`; register them in `hurra_init()`; add `hurra_notify_phys_*` emitters. |
| `src/kmbox.c` | In `kmbox_merge_report()`: capture pre-merge physical fields (Feature B), enforce masks (Feature C); drive the motion-program tick (Feature A) through the injection path. |
| `src/actions.c` / `src/actions.h` | Mask-state helpers if extending beyond `g_lock_mask`/`g_masked_keys` (e.g. `g_mask_wheel`); possibly a motion-program API. |
| `src/humanize.c` / `src/humanize.h` | Reuse for the `automove` velocity profile; no API change strictly required. |
| `src/proto.h` | Add `proto_notify_phys_buttons/axes/keys` aliases for both `PROTOCOL_HURRA` and `PROTOCOL_FERRUM` (Ferrum = no-op stubs). |

## 8. Cross-repo invariant (do not break)

The numeric `TYPE_*` values in §2 **must** equal those reserved in
`hurra-app/include/hurra_types.h`:

```
HURRA_TYPE_MOUSE_MOVE_DUR     0x1B
HURRA_TYPE_MOUSE_MOVE_BEZIER  0x1C
HURRA_TYPE_PHYS_MASK          0x68
HURRA_TYPE_CB_PHYS            0x77
HURRA_TYPE_TLM_PHYS_AXIS      0x86
HURRA_TYPE_TLM_PHYS_BUTTONS   0x87
HURRA_TYPE_TLM_PHYS_KB        0x88
```

If a payload layout changes during implementation, change it in **both** repos in
the same coordinated PR and update the host's `kmbox`/`input_core` decoders and
this document together.
