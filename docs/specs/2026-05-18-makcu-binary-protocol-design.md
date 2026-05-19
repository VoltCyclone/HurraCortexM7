# MAKCU Binary Protocol Support — Design

**Date:** 2026-05-18
**Status:** Approved, ready for implementation plan
**Author:** Claude (with @ramsey)

## Summary

Add support for the MAKCU v2 binary wire protocol
(`[0x50][CMD][LEN_LO][LEN_HI][PAYLOAD…]`, little-endian, no checksum)
as a **compile-time-exclusive alternative** to the existing Ferrum
ASCII protocol. A new `PROTOCOL` Makefile variable selects which
parser is built:

- `make` → Ferrum (current/shipped behaviour, fully backward
  compatible with v1.1.0)
- `make PROTOCOL=makcu` → MAKCU binary

The transport layer (`kmbox.c` — LPUART6, DMA, baud handling), the
action layer (`actions.c` — `act_move`, `act_button`, etc.), and all
other subsystems are unchanged. Only the parse/dispatch layer differs.

Source: <https://www.makcu.com/en/api> (KM Host Protocol).

## Goals

1. **Ferrum users untouched.** Plain `make` produces a `firmware.hex`
   identical to current `main` HEAD.
2. **MAKCU clients connect.** A MAKCU build advertises identity
   `"MAKCU v2.0"` via opcode `0xBF` and accepts the common mouse,
   keyboard, streaming, and misc opcodes (27 in the first cut — see
   *Coverage* below).
3. **Same code seams.** Both parsers share `actions.c` and `kmbox.c`'s
   transport, so any UART/DMA/baud fix benefits both.
4. **Self-recovery from wrong-baud bursts.** The 25 ms idle-gap reset
   that fixes Ferrum's "first command after baud probe is dropped"
   bug applies to MAKCU's frame state machine too.

## Non-goals (first cut)

- Lock / turbo / remap / catch opcodes
- Absolute positioning (`getpos` tracking, `moveto`)
- Bezier control points in `move` (linear path only)
- Keyboard `disable` / `mask` / `remap` / `init`
- MAKCU legacy `DE AD` framing
- Runtime protocol switching (`PROTOCOL` is compile-time only)
- ASCII-and-binary coexistence in a single build (MAKCU itself supports
  this via first-byte autodetection — we explicitly do not, by user
  request, for flash size and predictability)

## Architecture

### File layout

**New files**

- `src/makcu.h` — public API mirroring `src/ferrum.h` shape:
  `makcu_init`, `makcu_set_tx`, `makcu_feed_byte`, `makcu_reset`,
  `makcu_tick`, `makcu_notify_buttons`, `makcu_notify_axes`,
  `makcu_notify_keys`.
- `src/makcu.c` — frame parser state machine, dispatch, streaming,
  response framing.
- `src/proto.h` — header-only shim. `#ifdef PROTOCOL_MAKCU` aliases
  `proto_init`, `proto_feed_byte`, `proto_set_tx`, `proto_reset`,
  `proto_tick`, `proto_notify_*` to the `makcu_*` family; otherwise to
  the `ferrum_*` family. No code, just `#define`s.

**Modified files**

- `src/kmbox.c` — replace direct `ferrum_*` calls with `proto_*` (5–8
  call sites at `kmbox_init`, the HID merge hot-path notify
  callbacks, and the poll-loop `tick`). No `#ifdef`s in this file.
- `Makefile` —
  - `PROTOCOL ?= ferrum`
  - validate against `{ferrum, makcu}`; error otherwise
  - pass `-DPROTOCOL_FERRUM` or `-DPROTOCOL_MAKCU`
  - include only the selected parser `.c` in `OBJS` (e.g. via a
    `PROTOCOL_OBJS := src/$(PROTOCOL).o` rule)
  - hot-path optimisation list extended to include `src/makcu.c`
    under the same `-O2 -ffast-math` regime as `src/ferrum.c`.

**Unchanged**

- `src/actions.c` / `src/actions.h` — the action layer is the seam
  both parsers cross.
- `src/usb_host.c`, `src/usb_device.c`, `src/smooth.c`,
  `src/humanize.c`, `src/main.c` — protocol-agnostic.

### Build flag matrix

| Command | Behaviour |
|---|---|
| `make` | builds Ferrum (current/v1.1.0 behaviour, byte-identical to pre-merge `firmware.hex`) |
| `make PROTOCOL=makcu` | builds MAKCU |
| `make PROTOCOL=xyz` | error: `PROTOCOL must be ferrum or makcu` |
| `make flash` | flashes whichever was last built |

## `makcu.c` internals

### Frame parser state machine

```c
enum state { WAIT_HDR, GOT_HDR, GOT_CMD, GOT_LEN_LO, READ_PAYLOAD };
static enum state s_state;
static uint8_t    s_cmd;
static uint16_t   s_len;
static uint16_t   s_pos;
static uint8_t    s_buf[260];      /* keyboard `string` max payload */
static uint32_t   s_last_byte_ms;
```

Per byte in `makcu_feed_byte(b)`:

1. **Idle-gap reset**: if `now - s_last_byte_ms > 25 ms` and
   `s_state != WAIT_HDR`, reset to `WAIT_HDR`. Drops mid-frame
   leftovers from wrong-baud bursts. Mirrors the Ferrum fix.
2. `WAIT_HDR`: if `b == 0x50`, advance to `GOT_HDR`. Else stay
   (silent garbage discard).
3. `GOT_HDR`: `s_cmd = b` → `GOT_CMD`.
4. `GOT_CMD`: `s_len = b` → `GOT_LEN_LO`.
5. `GOT_LEN_LO`: `s_len |= b << 8`. If `s_len > sizeof s_buf`, reset
   to `WAIT_HDR` (oversize, drop). If `s_len == 0`, dispatch now.
   Else `s_pos = 0` → `READ_PAYLOAD`.
6. `READ_PAYLOAD`: `s_buf[s_pos++] = b`. When `s_pos == s_len`,
   dispatch then `WAIT_HDR`.

Recovery: on any drop, the parser returns to `WAIT_HDR` and waits for
the next `0x50`. No resync state, no length-window heuristics — cheap
and self-recovering.

### Dispatch

A dense `switch (cmd)` rather than a 256-entry function-pointer
table. Opcodes are sparse and a switch keeps each handler one inlined
function call from the parser entry point. Each handler:

- validates `s_len` matches the expected payload size; returns
  `emit_status(cmd, ERR)` on mismatch;
- unpacks the payload using little-endian helpers (`rd_u16le`,
  `rd_i16le`, `rd_u32le`, `rd_i8`);
- calls one or more `act_*` functions;
- emits a response with `emit_frame(cmd, payload, len)` or
  `emit_status(cmd, status)`.

### Response framing

```c
static void emit_frame(uint8_t cmd, const uint8_t *p, uint16_t n) {
    uint8_t hdr[4] = { 0x50, cmd, (uint8_t)(n & 0xff), (uint8_t)(n >> 8) };
    s_tx(hdr, 4);
    if (n) s_tx(p, n);
}
static inline void emit_status(uint8_t cmd, uint8_t status) {
    uint8_t hdr_and_status[5] = { 0x50, cmd, 0x01, 0x00, status };
    s_tx(hdr_and_status, 5);
}
```

`STATUS_OK = 0x00`, `STATUS_ERR = 0x01` per spec.

### Streaming

Per-stream state for opcodes 0x01, 0x02, 0x0C, 0xA5:

```c
struct stream { uint8_t mode; uint8_t period_ms; uint32_t last_emit_ms; };
static struct stream s_axis, s_buttons, s_mouse, s_keyboard;
```

Latest HID snapshot is held in module-scope caches updated by
`makcu_notify_buttons/axes/keys`, called from the same kmbox.c HID
merge hot-path call sites that today feed Ferrum's callbacks.

`makcu_tick()` (called every poll cycle from `kmbox_poll_fast`)
walks the four streams; for each, if `mode != 0 && now - last_emit
>= period`, builds and emits the snapshot frame. To avoid backing up
TX during stuck transfers, emit is skipped if the TX ring is
>50 % full (`((tx_head - tx_tail_pos) & TX_RING_MASK) > TX_RING_SIZE / 2`).

Mode 1 (raw) and mode 2 (constructed) are accepted and treated
identically in v1 — the distinction is documented but does not change
behaviour.

### Identity

```c
#define MAKCU_IDENTITY "MAKCU v2.0"     /* 10 bytes, no NUL */
```

Sent verbatim as the `0xBF` getter's payload. Length field in the
frame tells the host where it ends. A literal `#define` keeps it
trivial to retune if real MAKCU clients reject this exact string.

### Error policy

| Condition | Response |
|---|---|
| Unrecognised opcode | `emit_status(cmd, ERR)` |
| Payload length mismatch | `emit_status(cmd, ERR)` |
| Argument out of range | `emit_status(cmd, ERR)` |
| Recognised but deferred (e.g. `catch`, `lock`) | `emit_status(cmd, ERR)` |
| Bad header / oversize length | silently drop (parser state reset) |

Clients distinguish "device doesn't speak this" from "host did not send
a valid frame" by whether a response arrived at all.

## Opcode coverage (first cut)

**Mouse — 15 opcodes**

| Op | Name | Behaviour |
|---|---|---|
| 0x04 | click | scheduled click with release timer (Ferrum's existing machinery) |
| 0x05 | getpos | returns `[0, 0]` placeholder; documented |
| 0x06 | invert_x | `act_set_invert_x(state)` / getter mirrors flag |
| 0x07 | invert_y | `act_set_invert_y(state)` / getter mirrors flag |
| 0x08/0x0A/0x11/0x12/0x13 | left/middle/right/side1/side2 | `act_button(mask, state)` / getter reads `g_buttons` |
| 0x0B | mo | composite: `act_set_buttons`, `act_move`, `act_wheel`, `act_pan`, `act_tilt` in order; best-effort batched, not transactional |
| 0x0D | move | `act_move(x, y, segments != 0)` — `cx1/cy1` ignored (bezier deferred) |
| 0x0F | pan | `act_pan(steps)` |
| 0x14 | silent | `act_move(x, y, false)` with no humanization |
| 0x15 | swap_xy | `act_set_swap_xy(state)` |
| 0x16 | tilt | `act_tilt(steps)` |
| 0x18 | wheel | `act_wheel(delta)` |

Deferred: 0x03 catch, 0x09 lock, 0x0E moveto, 0x10 remap_button,
0x17 turbo, 0x19 remap_axis.

**Keyboard — 5 opcodes**

| Op | Name | Behaviour |
|---|---|---|
| 0xA2 | down | `act_key_down(key)` |
| 0xA4 | isdown | returns `[1]` if key in pressed set else `[0]` |
| 0xA7 | press | `act_key_press(key, hold_ms, rand_ms)` |
| 0xA9 | string | per-byte iteration via small ASCII→HID table (lower-case letters, digits, space; unsupported chars dropped) |
| 0xAA | up | `act_key_up(key)` |

Deferred: 0xA1 disable, 0xA3 init, 0xA6 mask, 0xA8 remap.

**Streaming — 4 opcodes**

| Op | Name | Stream payload |
|---|---|---|
| 0x01 | axis | `[dx:i16][dy:i16][wheel:i8]` |
| 0x02 | buttons | `[buttons_lo:u8][buttons_hi:u8]` (hi always 0 in v1) |
| 0x0C | mouse | `[buttons:u8][dx:i16][dy:i16][wheel:i8][pan:i8][tilt:i8]` |
| 0xA5 | keyboard | `[modifiers:u8][keys:u8×14]` (we have 6 — pad with zeros) |

**Misc — 3 opcodes**

| Op | Name | Behaviour |
|---|---|---|
| 0xB1 | baud | GET returns derived rate as `u32`; SET schedules deferred baud-change via existing `pending_baud_rate` machinery |
| 0xBB | reboot | emits OK ack, sets `SCB_AIRCR = 0x05FA0004` after a brief flush delay |
| 0xBF | version | returns `MAKCU_IDENTITY` (10 bytes) |

Deferred: 0xB2 bypass, 0xB3 device, 0xB4 echo, 0xB5 fault, 0xB7 hs,
0xB8 info, 0xB9 led, 0xBA log, 0xBC release, 0xBD screen, 0xBE serial.

**Total v1 surface: 27 opcodes recognised and acted on; every other
opcode returns a well-formed `ERR` status.**

## Testing

### Build verification

1. `make` (Ferrum, default) — must produce a byte-identical
   `firmware.hex` to current `main` HEAD pre-change. CI gate: SHA-256
   compare against the pre-merge artifact. Guarantees zero impact on
   v1.1.0 users.
2. `make PROTOCOL=makcu` — must compile clean at `-Wall` with no new
   warnings.
3. `make PROTOCOL=xyz` — must fail with `PROTOCOL must be ferrum or
   makcu` before invoking the compiler.

### On-device verification (MAKCU build)

A new `scripts/makcu_probe.py` exercising the protocol over
`/dev/cu.usbserial-0001`, running these in order:

1. **Identity** — TX `50 BF 00 00`. RX `50 BF 0A 00 'M' 'A' 'K' 'C'
   'U' ' ' 'v' '2' '.' '0'`.
2. **Move** — TX `50 0D 07 00 <x:i16=5> <y:i16=5> <seg:u8=1>
   <cx1:i8=0> <cy1:i8=0>` (7-byte payload). RX status OK; observe
   5,5 movement on downstream host.
3. **Mo composite** — TX `50 0B 08 00 01 0A 00 0A 00 01 00 00`
   (left button + move + wheel). RX status OK; observe combined
   effect.
4. **Mouse-button getter** — TX `50 08 00 00`. RX `50 08 01 00
   <state:u8>`.
5. **Baud roundtrip** — GET 0xB1 returns 115200; SET to 1_000_000
   returns OK; reopen at 1M; GET returns 1_000_000.
6. **Wrong-baud poison reproduction** — burst at 4M, reopen at
   115200, send `50 BF 00 00` *once*. First attempt must succeed.
   Confirms idle-gap reset works for binary frames.
7. **Streaming** — SET 0x0C mode=1 period=20 ms → host receives
   stream frames at ≈50 Hz; SET mode=0 → stream stops cleanly.
8. **Bad-length frame** — TX `50 0D FF FF`. Parser drops it; the
   next valid frame still parses correctly.
9. **Unsupported opcode** — TX `50 03 00 00` (catch, deferred). RX
   `50 03 01 00 01` (ERR).
10. **Mid-frame garbage gap** — send `50 0D 07 00 <only 3 of 7
    payload bytes>`, wait 50 ms, then send a valid frame. Idle-gap
    resets state, valid frame parses.

### Ferrum regression

- `scripts/uart_bench.py` must pass on the default Ferrum build,
  unchanged.
- `scripts/uart_debug.py` four-step probe still passes.
- The Ferrum idle-gap fix from this session stays — covered by the
  reproductions `probe_repro.py` and `probe_recovery.py`.

## Risks & mitigations

| Risk | Mitigation |
|---|---|
| MAKCU client expects an exact identity string we didn't anticipate | `#define MAKCU_IDENTITY` at the top of `makcu.c` is a one-line retune |
| Two firmware variants in the wild → user confusion about which is flashed | Protocol name baked into a startup-emitted info line; future work to suffix `firmware.hex` filename |
| Streaming emits piling up bytes during a stuck DMA TX | Stream emit checks TX ring fill ratio and skips this period if >50 % full |
| Composite `mo` opcode atomicity — Ferrum's actions are async/queued, not atomic | Documented as best-effort batched, not transactional; matches MAKCU spec which doesn't promise atomicity either |
| Spec ambiguity on lock / turbo / getpos | All deferred — ERR responses keep us spec-conformant for framing even if feature-incomplete |
| MAKCU client probes via ASCII `km.version()\r` first (as your current test app does) | Out of scope: by your decision, a MAKCU build does not speak ASCII. Test apps must speak binary for identity in MAKCU builds. |

## Migration

- v1.1.0 users (Ferrum): no action needed; `make` defaults preserve
  current behaviour.
- New MAKCU users: `make PROTOCOL=makcu && make flash`. Documented in
  `CLAUDE.md` build section.
- Both protocols still share UART transport, baud handling, and DMA —
  a bug fix in `kmbox.c` benefits both.

## Open questions

None. All requirements clarified in brainstorming session:

- Protocol target: MAKCU v2 (0x50 framed). ✓
- Flag semantics: compile-time exclusive. ✓
- Coverage: Ferrum-parity subset + streaming. ✓
- Default build: Ferrum. ✓
- Identity: MAKCU-compatible literal. ✓
- Streaming: include all four opcodes in first cut. ✓

## Source

- MAKCU KM Host Protocol: <https://www.makcu.com/en/api>
- Existing Ferrum parser: `src/ferrum.c`
- Existing action layer: `src/actions.c`
- Existing transport: `src/kmbox.c`
