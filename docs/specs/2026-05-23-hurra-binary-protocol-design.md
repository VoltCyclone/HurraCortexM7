# Hurra protocol — design spec

**Date**: 2026-05-23
**Status**: approved for implementation
**Scope**: Full rip-and-replace of the MAKCU protocol with a new binary protocol named **Hurra**, targeting ≥8 k commands/sec over a CH343B USB-UART bridge at 4 Mbps, parsed on the iMXRT1062 (Teensy MicroMod) using NXP's LPUART eDMA + IDLE-line driver pattern and the TinyFrame framing library.

---

## 1. Architecture

```
┌──────────────────────── PC host ────────────────────────┐    ┌──────── kmbox (Teensy iMXRT 1062) ────────┐
│                                                          │    │                                            │
│  hurra_client.py                                         │    │   ┌── hurra.c (TinyFrame instance) ──┐    │
│   ├─ public API: move/click/buttons/wheel/kb_*/...       │    │   │  TF_Accept(bytes)               │    │
│   ├─ TinyFrame instance (PyTinyFrame)                    │    │   │  Listener table (TF_AddTypeLis- │    │
│   ├─ 64-byte write batching (CH343B FS bulk MPS)         │    │   │   tener) → actions.c calls      │    │
│   └─ pyserial Serial(..., baudrate=4_000_000)            │    │   │  TF_Send for telemetry/replies  │    │
│                              │                           │    │   └─────────────────────────────────┘    │
│            USB FS bulk (CDC-ACM via WCH driver)          │    │                  ▲ feed_byte             │
└──────────────────────────────┼───────────────────────────┘    │   ┌── kmbox.c (LPUART3 eDMA driver) ─┐   │
                               │                                │   │  RX: eDMA circular buf + IDLE    │   │
                               ▼                                │   │       line IRQ → drains into TF  │   │
                          [CH343B chip]  ── 4 Mbps UART  ───────┼─▶ │  TX: ring buffer + eDMA channel  │   │
                                                                │   └──────────────────────────────────┘   │
                                                                └───────────────────────────────────────────┘
```

### Units

| Unit | Lives in | Responsibility |
|---|---|---|
| LPUART eDMA driver | `src/kmbox.c` (extended) | Bytes in/out. RX uses eDMA circular buffer + IDLE-line IRQ (NXP AN12552 pattern). TX uses existing ring + eDMA channel. No protocol awareness. |
| Hurra parser | `src/hurra.c` (new) | Holds the `TinyFrame` instance. Registers a listener per command type. Translates payload bytes ↔ `actions.c` calls. |
| Action layer | `src/actions.c` (unchanged) | Already protocol-agnostic API (`act_move`, `act_click`, `act_kb_down`, …). Both Ferrum and Hurra call into it. |
| Proto selector | `src/proto.h` (extended) | Compile-time switch; adds `PROTOCOL_HURRA` arm. `PROTOCOL_MAKCU` arm removed. |
| PC client | `tools/hurra_client.py` (new) + `tools/hurra_test.py` (new) | Python library + CLI. PyTinyFrame on top of pyserial. Batches writes to 64-byte chunks. |

### Build-time selector

Makefile gets `PROTOCOL ?= hurra` with arms for `hurra` and `ferrum`. Default flips to `hurra`. Ferrum (ASCII) remains buildable via `make PROTOCOL=ferrum`.

### Wire transport stack

```
hurra_client.py  ── TinyFrame Python frames ──┐
                                               ├── pyserial @ 4_000_000 baud
PC kernel ── /dev/cu.wchusbserialXXXX (WCH) ──┘
                      ▼ USB FS bulk OUT (64-byte MPS)
                 CH343B chip
                      ▼ UART @ 4 Mbps, 8-N-1
                 iMXRT LPUART3 RX
                      ▼ eDMA channel → circular buffer in OCRAM
                 IDLE-line IRQ wakes drain task
                      ▼ feeds bytes into TF_Accept()
                 TinyFrame listener dispatches → actions.c
```

---

## 2. Frame format

TinyFrame configuration:

```c
#define TF_SOF_BYTE       0x68         // 'h' for Hurra; distinct from MAKCU 0x50
#define TF_ID_BYTES       1            // 0..255 wraparound
#define TF_LEN_BYTES      1            // payload up to 255 B
#define TF_TYPE_BYTES     1            // 256 command types
#define TF_CKSUM_TYPE     TF_CKSUM_CRC16
#define TF_USE_MUTEX      0
#define TF_MAX_PAYLOAD_RX 256
```

### Wire layout

```
byte:   0    1     2     3     4-5         6 .. 6+LEN-1   6+LEN .. 7+LEN
       ┌────┬─────┬─────┬─────┬───────────┬──────────────┬────────────────┐
       │SOF │ ID  │ LEN │TYPE │ HEAD_CRC16│   PAYLOAD    │ PAYLOAD_CRC16  │
       │0x68│ u8  │ u8  │ u8  │ big-end   │  little-end  │   big-end      │
       └────┴─────┴─────┴─────┴───────────┴──────────────┴────────────────┘
           └─────── TinyFrame head (6 B) ──────┘       └── trailer (2 B) ──┘
```

- **Hot-path move frame**: TYPE=`MOUSE_MOVE`, payload=`[dx:i16][dy:i16]` (4 B). Total: 12 B.
- **Smallest frame** (e.g. `BTN_LEFT` set): 1 B payload. Total: 9 B.
- **Largest frame** (kb_string at the cap): 240 B payload. Total: 248 B.

### Numeric ceiling at 4 Mbps

| Cmd | On-wire bytes | Bits @ 10 bpb | Max cmds/sec |
|---|---:|---:|---:|
| MOUSE_MOVE (12 B) | 12 | 120 | 33,333 |
| BTN_LEFT (9 B) | 9 | 90 | 44,444 |
| Mixed (avg 11 B) | 11 | 110 | 36,363 |

8 k target → ~4.1× headroom on smallest practical frame. Bottleneck above ~30 k would shift to USB FS bulk pacing on the CH343B side.

### Configuration rationale

- **CRC16 on both head and payload**: kills the failure mode where a corrupted `LEN` byte misframes hundreds of bytes. Head-CRC catches it within 6 bytes; parser drops + waits for next SOF.
- **1-byte ID/TYPE/LEN**: smallest TinyFrame header. Pipelining 256+ frames isn't a use case.
- **SOF = 0x68**: lets a host that opens mid-stream resync at the next valid SOF after a CRC fail (worst case ~1 frame of lost bytes).
- **Idle-gap reset 5 ms**: at 4 Mbps that is 2,500 bytes — much larger than any frame, so a real mid-frame gap is unambiguously a desync.
- **Payload little-endian**: matches existing makcu/iMXRT native order and `rd_i16le`/`rd_u32le` helpers in the codebase.

### Wire example: `client.move(50, -30)`

```
68  17  04  10  | DA 7B | 32 00 E2 FF | 9C 4F
─┬  ─┬  ─┬  ─┬    ──┬──   ─────┬────    ──┬──
SOF  ID  LEN TYPE  head        payload    payload
                   CRC16       (dx=50,    CRC16
                               dy=-30)
```

Total 12 bytes; at 4 Mbps wire that frame takes 30 µs.

### Sequence-ID gap detection

TinyFrame ID rolls 0..255. Firmware tracks `last_seen_id`; on every frame received it computes `gap = (id - last_seen_id - 1) & 0xFF`. Gap counter is exposed via the stats frame (§6). Host gets a passive "did the firmware drop anything?" signal without per-frame ACK overhead.

---

## 3. Command catalogue

**Convention**: a request with an empty payload is a *get*; a request with a payload is a *set*. Replies reuse the same TYPE with matching ID — host correlates by ID. Hot-path commands are *oneway* (no reply, no ACK).

### 0x00–0x0F · Admin / diagnostics

| TYPE | Name | Direction | Payload | Notes |
|---:|---|---|---|---|
| 0x00 | PING | H→F, F→H reply | `u32 nonce` | RTT probe |
| 0x01 | VERSION | H→F empty / F→H reply | C-string `"kmbox: Hurra v1"` | |
| 0x02 | STATS | H→F empty (poll) / F→H reply | see §6 struct | also pushed as TYPE=0x84 |
| 0x03 | INIT | H→F oneway | empty | clears state |
| 0x04 | REBOOT | H→F oneway | empty | 20 ms flush delay, then SYSRESETREQ |
| 0x05 | BAUD | H→F u32 (set) or empty (get) | reply `u32 current` | host close→reopen after set |
| 0x06 | SCREEN | H→F `i16 w, i16 h` (set) or empty (get) | reply `i16 w, i16 h` | |

### 0x10–0x2F · Mouse

| TYPE | Name | Payload | Notes |
|---:|---|---|---|
| 0x10 | **MOUSE_MOVE** | `i16 dx, i16 dy` | **hot path, oneway** |
| 0x11 | MOUSE_MOVE_SMOOTH | `i16 dx, i16 dy` | bezier=true |
| 0x12 | MOUSE_SILENT_MOVE | `i16 dx, i16 dy` | bypasses position tracking |
| 0x13 | MOUSE_MO | `u8 buttons, i16 dx, i16 dy, i8 wheel, i8 pan, i8 tilt` (8 B) | MAKCU-equivalent multi-op |
| 0x14 | MOUSE_CLICK | `u8 button, u8 down_ms, u8 up_ms` | |
| 0x15 | MOUSE_WHEEL | `i8 ticks` | |
| 0x16 | MOUSE_GETPOS | empty / reply `i16 x, i16 y` | |
| 0x17 | INVERT_X | `u8 state` or get / reply `u8` | |
| 0x18 | INVERT_Y | `u8 state` or get / reply `u8` | |
| 0x19 | SWAP_XY | `u8 state` or get / reply `u8` | |
| 0x20 | BTN_LEFT | `u8 state (0/1)` or get / reply `u8` | bit 0 of `g_buttons` |
| 0x21 | BTN_RIGHT | " | bit 1 |
| 0x22 | BTN_MIDDLE | " | bit 2 |
| 0x23 | BTN_SIDE1 | " | bit 3 |
| 0x24 | BTN_SIDE2 | " | bit 4 |

### 0x40–0x4F · Keyboard

| TYPE | Name | Payload |
|---:|---|---|
| 0x40 | KB_DOWN | `u8 hid` |
| 0x41 | KB_UP | `u8 hid` |
| 0x42 | KB_PRESS | `u8 hid, u8 hold_ms, u8 rand_ms` |
| 0x43 | KB_ISDOWN | `u8 hid` / reply `u8 state` |
| 0x44 | KB_MASK | `u8 hid, u8 state` |
| 0x45 | KB_STRING | `char[1..240]` |
| 0x46 | KB_MULTIDOWN | `u8 keys[1..6]` |
| 0x47 | KB_MULTIUP | `u8 keys[1..6]` |
| 0x48 | KB_MULTIPRESS | `u8 keys[1..6]` |

### 0x60–0x6F · Locks + catch_xy (Ferrum parity)

| TYPE | Name | Payload | Notes |
|---:|---|---|---|
| 0x60–0x64 | LOCK_ML, MR, MM, MS1, MS2 | `u8 state` or get / reply `u8` | sets bit in `g_lock_mask` |
| 0x65 | LOCK_MX | " | |
| 0x66 | LOCK_MY | " | |
| 0x67 | CATCH_XY | `u16 dur_ms` (request) | **deferred reply** with `i32 dx_accum, i32 dy_accum` when timer expires |

### 0x70–0x7F · Stream / callback enable

| TYPE | Name | Payload | Notes |
|---:|---|---|---|
| 0x70 | STREAM_AXIS_CFG | `u8 mode, u8 period_ms` | MAKCU 0x01 equivalent |
| 0x71 | STREAM_BTN_CFG | `u8 mode, u8 period_ms` | MAKCU 0x02 |
| 0x72 | STREAM_MOUSE_CFG | `u8 mode, u8 period_ms` | MAKCU 0x0C |
| 0x73 | STREAM_KB_CFG | `u8 mode, u8 period_ms` | MAKCU 0xA5 |
| 0x74 | CB_BUTTONS | `u8 enable` | Ferrum-style change-only callback |
| 0x75 | CB_AXES | `u8 enable` | |
| 0x76 | CB_KEYS | `u8 enable` | |

### 0x80–0x8F · Unsolicited firmware-emitted

| TYPE | Name | Payload | When emitted |
|---:|---|---|---|
| 0x80 | TLM_AXIS | `i16 dx, i16 dy, i8 wheel` | STREAM_AXIS_CFG period |
| 0x81 | TLM_BUTTONS | `u8 bitmap` | STREAM_BTN_CFG or CB_BUTTONS on change |
| 0x82 | TLM_MOUSE | `u8 buttons, i16 dx, i16 dy, i8 wheel, i8 pan, i8 tilt` | STREAM_MOUSE_CFG period |
| 0x83 | TLM_KEYBOARD | `u8 modifier, u8 keys[6]` | STREAM_KB_CFG or CB_KEYS on change |
| 0x84 | TLM_STATS | see §6 | every 100 ms (auto) |
| 0x85 | TLM_LOG | C-string | opt-in firmware diagnostics |

### Coverage check vs prior protocols

- MAKCU 0x04/0x05/0x06/0x07/0x08/0x0A/0x0B/0x0D/0x11/0x12/0x13/0x14/0x15/0x18 → mapped (0x14, 0x16, 0x17, 0x18, 0x20, 0x22, 0x13, 0x10, 0x21, 0x23, 0x24, 0x12, 0x19, 0x15).
- MAKCU 0xA2/0xA4/0xA7/0xA9/0xAA → 0x40/0x43/0x42/0x45/0x41.
- MAKCU 0x01/0x02/0x0C/0xA5 → 0x70/0x71/0x72/0x73 (config) + 0x80/0x81/0x82/0x83 (data).
- MAKCU 0xB1/0xBB/0xBD/0xBF → 0x05/0x04/0x06/0x01.
- Ferrum locks (7), catch_xy, multidown/up/press, mask, init, callbacks (3) → 0x60–0x67, 0x46–0x48, 0x44, 0x03, 0x74–0x76.
- Hurra-native additions: PING (0x00), STATS (0x02/0x84), TLM_LOG (0x85).

73 of 256 type slots used; ample room to grow.

---

## 4. Firmware integration

### 4.1 File layout (post-implementation)

```
src/
  proto.h              # extended: PROTOCOL_HURRA arm replaces PROTOCOL_MAKCU
  ferrum.{c,h}         # unchanged
  hurra.{c,h}          # NEW — TinyFrame wrapper, listener table, dispatch
  kmbox.{c,h}          # extended — LPUART3 eDMA RX with IDLE-line IRQ
  actions.{c,h}        # unchanged (already protocol-agnostic)
  TF_Config.h          # NEW — TinyFrame configuration (§2)
  third_party/
    TinyFrame/
      TinyFrame.c      # vendored, MIT
      TinyFrame.h      # vendored
      LICENSE          # upstream MIT
```

`makcu.{c,h}` deleted; `proto.h`'s `#if PROTOCOL_MAKCU` arm replaced with `#if PROTOCOL_HURRA`.

### 4.2 `proto.h` after rewrite

```c
#pragma once

#if defined(PROTOCOL_HURRA)
  #include "hurra.h"
  #define proto_init           hurra_init
  #define proto_feed_byte      hurra_feed_byte
  #define proto_reset          hurra_reset
  #define proto_tick           hurra_tick
  #define proto_set_tx         hurra_set_tx
  typedef hurra_tx_fn proto_tx_fn;
  #define proto_notify_buttons hurra_notify_buttons
  #define proto_notify_axes    hurra_notify_axes
  #define proto_notify_keys    hurra_notify_keys
  #define PROTO_NAME "Hurra"
#elif defined(PROTOCOL_FERRUM)
  #include "ferrum.h"
  #define proto_init           ferrum_init
  #define proto_feed_byte      ferrum_feed_byte
  #define proto_reset          ferrum_reset
  #define proto_tick           ferrum_tick
  #define proto_set_tx         ferrum_set_tx
  typedef ferrum_tx_fn proto_tx_fn;
  #define proto_notify_buttons ferrum_notify_buttons
  #define proto_notify_axes    ferrum_notify_axes
  #define proto_notify_keys    ferrum_notify_keys
  #define PROTO_NAME "Ferrum"
#else
  #error "Define PROTOCOL_FERRUM or PROTOCOL_HURRA (set PROTOCOL in Makefile)"
#endif
```

### 4.3 LPUART3 eDMA RX path

Current per-byte RX ISR is untenable at 4 Mbps (~400k IRQ/sec). Switch to **eDMA circular buffer + IDLE-line interrupt** (NXP AN12552):

```
       ┌────────────── 1 KiB circular buffer in OCRAM ──────────────┐
       │ ... bytes streaming in from LPUART3 RX (no CPU per byte) ...│
       └──────────────────────────────────────────────────────────────┘
              ▲                                              ▲
         eDMA write ptr                                drain ptr
         (HW-advanced)                                 (sw-advanced)
```

- **eDMA channel 0**: TCD configured continuous/circular, DST=`rx_buf`, SRC=`LPUART3->DATA`, request source `kDmaRequestMuxLPUART3Rx`. Major loop = buffer size; minor = 1 byte. Self-replenishing.
- **IDLE-line IRQ** on LPUART3 fires when the line goes idle for ≥1 char-time. ISR snapshots eDMA TCD `CITER` for HW write pointer.
- **Drain in `kmbox_tick()`** (main loop, HID poll rate cadence): compute bytes between SW drain ptr and HW write ptr; feed each through `proto_feed_byte()`. Buffer sized for 1 ms worst-case (4 Mbps / 10 = 400 KB/s × 1 ms = 400 B; 1 KiB gives 2.5× margin).
- **Overflow detection**: if drain falls behind by ≥¾ buffer, increment `rx_drv_overrun` stats counter.

Reference: NXP MCUXpresso example `evkmimxrt1060/driver_examples/lpuart/edma_ring_buffer`. Register-level port is ~50 lines.

### 4.4 TX path

Existing `kmbox.c` TX ring is sufficient — TinyFrame's `WriteImpl` callback enqueues bytes into the ring. Add `kmbox_tx_room()` accessor so telemetry emitters can skip when the ring is >75% full (input commands never skipped; see §6.6).

### 4.5 `hurra.c` skeleton

```c
// src/hurra.c — Hurra protocol parser (TinyFrame-based)
#include "TinyFrame.h"
#include "hurra.h"
#include "actions.h"
#include "kmbox.h"
#include <string.h>

static TinyFrame s_tf;
static hurra_tx_fn s_tx;

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len) {
    if (s_tx) s_tx(buf, (uint16_t)len);
}

// Hot-path listener — keep tiny and branch-free where possible.
static TF_Result l_mouse_move(TinyFrame *tf, TF_Msg *msg) {
    if (msg->len != 4) return TF_STAY;
    int16_t dx = (int16_t)(msg->data[0] | (msg->data[1] << 8));
    int16_t dy = (int16_t)(msg->data[2] | (msg->data[3] << 8));
    act_move(dx, dy, false);
    return TF_STAY;
}

static TF_Result l_ping(TinyFrame *tf, TF_Msg *msg) {
    TF_Msg reply = *msg;
    reply.is_response = true;
    TF_Respond(tf, &reply);
    return TF_STAY;
}
// ... ~30 more listeners, one per TYPE in §3 ...

void hurra_init(void) {
    TF_InitStatic(&s_tf, TF_MASTER);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE, l_mouse_move);
    TF_AddTypeListener(&s_tf, TYPE_PING,       l_ping);
    /* ... full table ... */
}

void hurra_feed_byte(uint8_t b)        { TF_Accept(&s_tf, &b, 1); }
void hurra_set_tx(hurra_tx_fn tx)      { s_tx = tx; }

void hurra_tick(void) {
    TF_Tick(&s_tf);
    // periodic stats push (§6.3), stream emits, deferred catch_xy/reboot
}
```

### 4.6 Hot-path performance budget

```
eDMA RX (HW)  →  IDLE IRQ (3 µs)  →  kmbox_tick drain loop
                                       └─ TF_Accept(byte) × 12   ← state machine
                                            └─ l_mouse_move()    ← 3 lines
                                                 └─ act_move()
```

- Per-byte TF_Accept: ~100 ns on Cortex-M7 @ 816 MHz (state-machine step + CRC update).
- 12 bytes/frame × 100 ns = 1.2 µs CPU/frame.
- At 8000 frames/sec → 9.6 ms/sec CPU = **0.96 % CPU at design rate**. At 30 k frames/sec ceiling → 3.6 % CPU.
- IDLE IRQ fires once per burst, not per frame; overhead negligible at sustained rate.

### 4.7 Makefile changes

```make
PROTOCOL ?= hurra

ifeq ($(PROTOCOL),hurra)
  PROTO_DEF  = -DPROTOCOL_HURRA
  PROTO_SRC  = src/hurra.c src/third_party/TinyFrame/TinyFrame.c
else ifeq ($(PROTOCOL),ferrum)
  PROTO_DEF  = -DPROTOCOL_FERRUM
  PROTO_SRC  = src/ferrum.c
else
  $(error PROTOCOL must be 'hurra' or 'ferrum')
endif

DEFINES  += $(PROTO_DEF)
CFLAGS   += -Isrc/third_party/TinyFrame
SRC       = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
            src/kmbox.c src/humanize.c src/smooth.c src/actions.c \
            $(PROTO_SRC)
```

Default flips to `hurra`. Building Ferrum: `make PROTOCOL=ferrum`.

---

## 5. PC VCOM application

### 5.1 File layout (new)

```
tools/
  hurra_client.py        # library — public API mirrors §3
  hurra_test.py          # CLI:  version | ping | stats | move | click | load | smoke
  hurra_aim_test.py      # port of ferrum_aim_test.py — the 8k+ consumer
  third_party/
    tinyframe_py/        # vendored Python TinyFrame port
      LICENSE
scripts/
  hurra_probe.py         # replaces makcu_probe.py
```

`ferrum_test.py` / `ferrum_aim_test.py` stay — ferrum still buildable via `make PROTOCOL=ferrum`.

### 5.2 Stack

| Layer | Choice | Notes |
|---|---|---|
| Serial | pyserial 3.5+ | already a project dep |
| Framing | TinyFrame Python port (vendored at `tools/third_party/tinyframe_py/`) | matches firmware exactly; MIT |
| macOS custom baud | `fcntl.ioctl(fd, IOSSIOSPEED, baud)` | pyserial's `baudrate=4_000_000` works on Linux; macOS needs IOSSIOSPEED for non-standard rates |
| USB writes | 64-byte chunked writes | aligns with CH343B FS bulk MPS |

### 5.3 CH343B-specific tuning

WCH **CH34xVCP** driver required on macOS for ≥1.5 Mbps. Install via WCH's signed `.dmg` (system extension on Apple Silicon, kext on Intel). README documents the install step.

| Setting | Value | Why |
|---|---|---|
| Baud | 4_000_000 | matches firmware default |
| Hardware flow control | off | no RTS/CTS wired on the ATP carrier |
| RX/TX latency timer (WCH driver) | 1 ms | minimizes echo lag for telemetry streams |
| Write chunk size | 64 B | match USB FS bulk MPS |
| Read chunk size | 512 B | host-side cushion for telemetry bursts |
| `O_NONBLOCK` on read | yes | parser fed in a small reader thread |

**Write batching is the throughput knob.** At 8 k MOUSE_MOVE/sec, naive one-frame-per-write generates 8 k syscalls + 8 k USB transfers — driver overhead dominates and stalls well below 8 k. With 64-byte batching, 12-byte frames pack 5 per write → ~1.6 k writes/sec at 8 k frame rate. Auto-flush after 1 ms idle keeps latency bounded for partial buffers.

### 5.4 `hurra_client.py` public API

```python
class HurraClient:
    def __init__(self, port: str, baud: int = 4_000_000): ...

    # Hot path
    def move(self, dx: int, dy: int) -> None: ...
    def button(self, mask: int, state: int) -> None: ...
    def wheel(self, ticks: int) -> None: ...

    # Request/reply
    def version(self) -> str: ...
    def ping(self, nonce: int | None = None) -> float: ...
    def stats(self) -> Stats: ...
    def getpos(self) -> tuple[int, int]: ...

    # Keyboard
    def kb_down(self, hid: int) -> None: ...
    def kb_up(self, hid: int) -> None: ...
    def kb_press(self, hid: int, hold_ms: int = 80, rand_ms: int = 30) -> None: ...
    def kb_isdown(self, hid: int) -> bool: ...
    def kb_string(self, s: str) -> None: ...
    def kb_multidown(self, keys: list[int]) -> None: ...
    def kb_multiup(self, keys: list[int]) -> None: ...
    def kb_multipress(self, keys: list[int]) -> None: ...

    # Locks + catch
    def lock(self, name: str, state: int | None = None) -> int | None: ...
    def catch_xy(self, dur_ms: int) -> tuple[int, int]: ...   # blocks until reply

    # Telemetry subscriptions
    def stream_axis(self, mode: int, period_ms: int) -> None: ...
    def stream_buttons(self, mode: int, period_ms: int) -> None: ...
    def stream_mouse(self, mode: int, period_ms: int) -> None: ...
    def stream_keyboard(self, mode: int, period_ms: int) -> None: ...
    def on_telemetry(self, type_: int, handler: Callable) -> None: ...
    def on_drops(self, handler: Callable[[int], None]) -> None: ...

    # Lifecycle
    def flush(self) -> None: ...
    def close(self) -> None: ...
```

### 5.5 Write batching

```python
def _buffered_write(self, data: bytes) -> None:
    self._wbuf.extend(data)
    now = time.monotonic()
    if len(self._wbuf) >= 64 or (now - self._last_flush) >= 0.001:
        self._ser.write(self._wbuf)
        self._wbuf.clear()
        self._last_flush = now

# Background flush loop for low-rate callers
def _flush_loop(self):
    while not self._stop:
        time.sleep(0.0005)
        if self._wbuf and (time.monotonic() - self._last_flush) >= 0.001:
            self.flush()
```

Result: 8 k MOUSE_MOVE/sec → ~1.6 k writes/sec at ~60 B avg; sparse 100/sec callers → max 1 ms latency to wire.

### 5.6 `hurra_test.py` CLI

```
hurra_test.py [--port PORT] [--baud 4000000] <subcmd>

  version           # print firmware version
  ping              # RTT probe; prints µs
  stats             # dump firmware stats
  move DX DY        # one move
  click BUTTON      # one click
  load --rate N --duration S [--split-px K]
                    # blast MOUSE_MOVE at N/sec for S sec; report achieved RPS,
                    # latency, firmware drop count, queue drain time
  smoke             # round-trip every command type once; pass/fail per type
```

`load` is the throughput verification (§7.3).

### 5.7 `hurra_aim_test.py`

Direct port of `ferrum_aim_test.py`:
- Replace `m(x, y)\r\n` writes with `client.move(dx, dy)`.
- Replace `cb_axes` toggle with `client.stream_axis(mode=1, period_ms=1)` + `on_telemetry(TYPE_TLM_AXIS, ...)`.
- Keep `--tick-ms` and `--cmd-step-px` knobs identical.
- Add `--baud` (default 4_000_000).
- Run-end report includes RPS, drop count from stats, time-to-hit per target.

Expected outcome: at `--tick-ms=6 --cmd-step-px=4` (current settings), cursor feedback at 166 Hz; at saturation ≥8 k MOUSE_MOVE/sec — ~15× the current 551 cmds/sec ceiling. Closed-loop time-to-hit drops proportionally; bottleneck shifts from wire to `PULL_GAIN` geometric decay.

### 5.8 `hurra_probe.py`

Replaces `scripts/makcu_probe.py`:
- Walks `/dev/cu.wchusbserial*` and `/dev/cu.usbmodem*`.
- Tries baud handshake at default (4 Mbps), then 1.5 Mbps, then 115200.
- Reports detected chip via VID:PID lookup against `system_profiler SPUSBDataType`.
- Prints firmware version + stats once connected.

---

## 6. Reliability, diagnostics, stats frame

The protocol is **fire-and-forget on the hot path, observable everywhere else.** No per-frame ACK on writes; the firmware maintains a stats vector exposed both ways: pull (TYPE 0x02) and push (TYPE 0x84 every 100 ms).

### 6.1 Failure modes and detection

| Failure | Detected by | Counter | Recovery |
|---|---|---|---|
| Single-bit error in header | Head-CRC16 mismatch | `head_crc_err` | drop frame, parser waits for next SOF |
| Single-bit error in payload | Payload-CRC16 mismatch | `payload_crc_err` | drop frame, listener never called |
| Lost frame (host wrote, firmware never saw) | ID gap `(rx_id - last_id - 1) & 0xFF > 0` | `id_gap_total` | counters only; host may resend if it cares |
| Host opens port mid-stream | First N bytes fail CRC ~immediately | `head_crc_err` blip | TF re-syncs on first valid SOF |
| Wrong baud burst | Idle-gap timer (5 ms) elapses mid-frame | `idle_resync` | parser drops partial frame, waits for next SOF |
| eDMA RX overflow | TCD wrap detected by drain | `rx_drv_overrun` | bytes lost; counter surfaces it |
| TX ring full (telemetry can't drain) | `kmbox_tx_room() < N` | `tx_ring_skip` | telemetry frame skipped (input commands never skipped) |
| Malformed payload | listener bounds check | `payload_invalid` | listener returns without acting |

None of these stop the parser; the state machine always converges back to `S_WAIT_SOF` within one frame's worth of bytes after any single fault.

### 6.2 `TLM_STATS` payload (TYPE 0x84, also reply to TYPE 0x02)

```c
struct hurra_stats {
    uint32_t uptime_ms;
    uint32_t rx_frames_ok;
    uint32_t head_crc_err;
    uint32_t payload_crc_err;
    uint32_t id_gap_total;        // sum of gap sizes (not # of gap events)
    uint32_t idle_resync;
    uint32_t rx_drv_overrun;
    uint32_t tx_ring_skip;
    uint32_t payload_invalid;
    uint16_t tx_ring_high_water;  // peak fill since last stats emit
    uint16_t reserved;
} __attribute__((packed));        // 36 B payload → 44 B on wire
```

Cleared on `INIT` (TYPE 0x03). Preserved across `REBOOT` for one cycle via OCRAM non-init storage.

### 6.3 Auto-push cadence

`hurra_tick()` emits TLM_STATS every 100 ms → 10 frames/sec × 44 B = 440 B/sec, **0.01 % of 4 Mbps wire**. Negligible.

Push disabled by `STREAM_STATS_CFG period_ms=0`. Manual polls via TYPE 0x02 still work.

### 6.4 Idle-gap timer math

At 4 Mbps, one byte = 2.5 µs. A real frame transmits 9–264 bytes = 22.5 µs–660 µs. Idle-gap reset at **5 ms (2000 bytes)** ensures real mid-frame gaps are never confused with end-of-frame; wrong-baud bursts followed by reopen at 4 Mbps get cleanly resynced.

### 6.5 Baud-change handshake

```
host        firmware
 │  BAUD(2000000, id=42) ──────────▶│
 │                                  │ kmbox_set_baud(2000000)  (deferred 20 ms)
 │  ◀──────── BAUD reply(id=42, current=2000000)
 │  (host closes port, waits 50 ms, reopens at 2 Mbps)
```

Firmware applies new baud *after* TX flush of the reply frame — same pattern as MAKCU's deferred reboot.

### 6.6 Backpressure policy

1. Input command listeners **never** skip — input is the product.
2. Telemetry stream emits **do** skip when `kmbox_tx_room() < frame_size`; `tx_ring_skip` counter increments.
3. Replies to request frames **never** skip — they're rare and the host is blocked waiting.

### 6.7 Host-side resync after gap

If host observes `id_gap_total` increased by N since last stats push, it knows the firmware missed N frames. Library exposes a `client.on_drops(handler)` callback; application decides:
- Aim test: ignore — next tick's closed-loop correction subsumes the loss.
- Click/keyboard caller: resend dropped frames (host keeps a small sent-buffer keyed by ID for 100 ms).

### 6.8 Explicitly out of scope

- No retransmission protocol (NAK, SACK, windowing). Closed-loop application handles loss tolerance.
- No encryption / authentication. Single-user wired link; not in threat model.
- No multiplexing. One conversation per UART link.

All three are addable later without breaking the wire format. YAGNI for v1.

---

## 7. Performance budget & verification

### 7.1 End-to-end budget at 8 k MOUSE_MOVE/sec target

| Stage | Cost per frame | At 8 k/sec |
|---|---:|---:|
| Host: `client.move()` Python call | ~3 µs | 24 ms/s = 2.4% of 1 core |
| Host: TF encode + CRC16 | ~2 µs | 16 ms/s |
| Host: 64-byte batched write to OS | amortized 0.2 µs | 1.6 ms/s |
| USB FS bulk OUT (12 B in 64-B chunk, 5 frames/chunk) | ~13 µs/chunk → 2.6 µs/frame | 21 ms/s |
| CH343B → UART @ 4 Mbps | 30 µs wire time per frame | 240 ms/s = 24% of wire |
| iMXRT eDMA RX (HW, no CPU) | 0 | 0 |
| TF_Accept × 12 bytes | 1.2 µs | 9.6 ms/s = 0.96% CPU |
| Listener `l_mouse_move` | 0.3 µs | 2.4 ms/s |
| `act_move()` queue insert | ~1 µs | 8 ms/s |
| **End-to-end wall-clock** | **~50 µs** | wire ~24%, CPU ~1% |

Wall-clock per frame well under 125 µs/frame budget at 8 k/sec. Hard ceiling is the wire — 4 Mbps / 12 B = 33,333 frames/sec. Everything else has 30×+ headroom.

### 7.2 Versus current Ferrum baseline

| Metric | Ferrum @ 115200 (now) | Hurra @ 4 Mbps (target) | Gain |
|---|---:|---:|---:|
| Avg bytes per `move` cmd | 15 (ASCII) | 12 (binary) | 1.25× |
| Wire bit budget | 11,520 B/sec | 400,000 B/sec | 34.7× |
| Practical cmds/sec ceiling | 768 (measured 551–773) | 33,333 theoretical; ≥8,000 design target | **15–43×** |
| Firmware CPU @ 1 kHz cmd rate | ~2% (per-byte IRQ) | <0.2% (eDMA) | 10× |
| Per-cmd ACK overhead | 0 (Ferrum is oneway) | 0 (Hurra is oneway hot path) | equal |
| Host syscalls/sec @ 8 k | infeasible | ~1,600 (batched) | 5× fewer than naive |

### 7.3 Verification — three tests gate merge

**Test A: `hurra_test.py load --rate 8000 --duration 30`** (ship gate)
- Blast 8000 MOUSE_MOVE/sec for 30 s = 240,000 frames.
- Pass criteria:
  - achieved RPS ≥ 7,800 (≤2.5% jitter)
  - firmware `id_gap_total` increase < 50 (≤0.02% loss)
  - `head_crc_err` + `payload_crc_err` + `rx_drv_overrun` = 0
  - host RTT (parallel PING every 100 ms) p99 < 10 ms

**Test B: `hurra_test.py load --rate 16000 --duration 10`** (headroom)
- At 2× design rate, expect ≥15 k/sec achieved with ≤0.5% loss.
- Proves 8 k isn't near a cliff.

**Test C: `hurra_test.py smoke`** (correctness)
- Round-trip every TYPE from §3 once; validate each handler.
- Pass = all replies match, no errors logged, all setters reflect on next get.

### 7.4 Bench setup for repeatability

- Teensy MicroMod on ATP carrier, CH343B on LPUART3.
- `make PROTOCOL=hurra CMD_BAUD=4000000 flash`.
- macOS host with CH34xVCP installed; quiescent host port hub.
- Each test 3×; report median + spread. Single-run scheduling outliers don't fail spec.

### 7.5 Pre-existing tools that need updating

- `scripts/uart_debug.py` — add `--protocol hurra` mode (binary frames instead of ASCII probes).
- `scripts/uart_bench.py` — same.
- `tools/ferrum_aim_test.py` — left as-is; port lives in `tools/hurra_aim_test.py`.

---

## 8. Migration plan (full makcu rip-and-replace)

### 8.1 Files to delete

```
src/makcu.c                                      (untracked at spec time — plain rm)
src/makcu.h                                      (untracked — plain rm)
src/makcu.o                                      (build artifact)
firmware-makcu.hex                               (release artifact)
scripts/makcu_probe.py                           (untracked — plain rm)
docs/specs/2026-05-18-makcu-binary-protocol-design.md
.protocol-makcu.stamp                            (untracked build stamp)
```

Most makcu source files are untracked (never committed). For tracked items use `git rm`; for untracked use plain `rm`. The `git grep` verification (§8.5) operates on the working tree, so both are covered.

### 8.2 Files to edit (remove every makcu mention)

| File | Change |
|---|---|
| `src/proto.h` | Replace `PROTOCOL_MAKCU` arm with `PROTOCOL_HURRA`; `#else` error message references HURRA not MAKCU |
| `Makefile` | Add `PROTOCOL ?= hurra` switch with `hurra`/`ferrum` arms; add `src/hurra.c` + TinyFrame vendor sources to `PROTO_SRC` for the hurra arm |
| `docs/plans/2026-05-21-xinput-controller-passthrough.md` | Strip the makcu reference; if the plan is stale, mark superseded and link to this spec |
| `.claude/settings.local.json` | Drop any allow-list entry that hard-codes a makcu path |
| `README.md`, `CLAUDE.md` (if they mention makcu) | Replace references; document the new `PROTOCOL=hurra\|ferrum` flag |
| `tools/ferrum_aim_test.py` header (if any cross-reference) | Update |

### 8.3 Files to create

| Path | Purpose |
|---|---|
| `src/hurra.c` | TinyFrame wrapper, listener table per §3 |
| `src/hurra.h` | Public API: `hurra_init`, `hurra_feed_byte`, `hurra_tick`, `hurra_set_tx`, `hurra_notify_*` |
| `src/TF_Config.h` | TinyFrame config per §2 |
| `src/third_party/TinyFrame/TinyFrame.c` | vendored (MIT) |
| `src/third_party/TinyFrame/TinyFrame.h` | vendored |
| `src/third_party/TinyFrame/LICENSE` | upstream MIT |
| `tools/hurra_client.py` | Python library, full API parity with §3 |
| `tools/hurra_test.py` | CLI harness |
| `tools/hurra_aim_test.py` | Port of `ferrum_aim_test.py` at 4 Mbps |
| `tools/third_party/tinyframe_py/` | vendored Python TinyFrame |
| `tools/third_party/tinyframe_py/LICENSE` | MIT |
| `scripts/hurra_probe.py` | Replaces `makcu_probe.py` |
| `docs/specs/2026-05-23-hurra-binary-protocol-design.md` | this design doc |

### 8.4 Files to extend (existing code, additive)

| File | Change |
|---|---|
| `src/kmbox.c` | Add LPUART3 eDMA RX + IDLE-line IRQ per §4.3. Add `kmbox_tx_room()` accessor. Switch from per-byte RX ISR to eDMA path unconditionally (Ferrum benefits equally). |
| `src/kmbox.h` | Declare `kmbox_tx_room()` |
| `scripts/uart_debug.py` | Add `--protocol hurra` mode for binary-frame probing |
| `scripts/uart_bench.py` | Add hurra-mode benchmark path |

### 8.5 Verification that the rip is complete

Merge gate:

```bash
# 1. No "makcu" / "MAKCU" / "MAKCU_" anywhere
git grep -i 'makcu' && echo "FAIL: makcu references remain" && exit 1 || true

# 2. Both protocols build
make clean && make PROTOCOL=hurra   && [ -f firmware.hex ] || exit 1
make clean && make PROTOCOL=ferrum  && [ -f firmware.hex ] || exit 1

# 3. Ship-gate throughput test (§7.3 Test A)
./tools/hurra_test.py load --rate 8000 --duration 30
```

If `git grep -i makcu` returns any hits, migration is incomplete.

### 8.6 Commit sequence

1. **Add infra, no behavior change** — vendor TinyFrame (C + Python), add `src/TF_Config.h`, add Makefile `PROTOCOL` switch arm for hurra (still defaulting to ferrum). Build passes.
2. **Add LPUART eDMA RX in kmbox.c** — ferrum still builds and works; no functional change for ferrum.
3. **Add `src/hurra.c` + `src/hurra.h`** with full §3 command catalogue. `make PROTOCOL=hurra` succeeds.
4. **Add `tools/hurra_client.py` + `tools/hurra_test.py` + `scripts/hurra_probe.py`**. Run smoke test.
5. **Add `tools/hurra_aim_test.py`**. Validate ≥8 k cmds/sec end-to-end.
6. **Default Makefile to `PROTOCOL=hurra`**.
7. **Rip-and-replace commit** — delete all files in §8.1, edit all files in §8.2, run `git grep -i makcu` → empty. Update README/CLAUDE.md.
8. **Tag release** (e.g. `kmbox-hurra-v1.0`).

Steps 1–6 are reversible without losing makcu. Step 7 is the irreversible cut-over; only run it once the ship gate (Test A) passes on a real bench.
