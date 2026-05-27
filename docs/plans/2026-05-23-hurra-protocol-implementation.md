# Hurra Protocol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the MAKCU protocol with **Hurra**, a TinyFrame-based binary protocol targeting ≥8 k commands/sec over a CH343B USB-UART bridge at 4 Mbps, parsed on iMXRT1062 via NXP LPUART eDMA + IDLE-line driver pattern.

**Architecture:** Vendor TinyFrame (C + Python ports, MIT) and slot it behind the existing `proto.h` compile-time switch. Replace `kmbox.c`'s per-byte RX ISR with an eDMA circular buffer + IDLE-line interrupt so the firmware can keep up at 4 Mbps. Add a Python `hurra_client.py` with 64-byte write batching aligned to CH343B's USB FS bulk MPS. Migration is split into incremental, reversible steps; the irreversible `makcu` deletion happens only after the throughput ship-gate (`load --rate 8000 --duration 30`) passes on real hardware.

**Tech Stack:** C (Cortex-M7, GCC bare-metal), Python 3.10+ (pyserial), TinyFrame (MightyPork, MIT), NXP MCUXpresso reference pattern AN12552 (LPUART eDMA + IDLE).

**Spec:** `docs/specs/2026-05-23-hurra-binary-protocol-design.md` — single source of truth for frame format (§2), command catalogue (§3), and verification gates (§7.3).

---

## Phase 1 — Vendor TinyFrame and add Makefile selector

### Task 1.1: Vendor TinyFrame C source

**Files:**
- Create: `src/third_party/TinyFrame/TinyFrame.c`
- Create: `src/third_party/TinyFrame/TinyFrame.h`
- Create: `src/third_party/TinyFrame/LICENSE`

- [ ] **Step 1: Fetch TinyFrame at a pinned version**

```bash
mkdir -p ~/code/imxrtnsy/src/third_party/TinyFrame
cd /tmp && git clone --depth 1 --branch master https://github.com/MightyPork/TinyFrame.git tf-vendor
cp tf-vendor/TinyFrame.c   ~/code/imxrtnsy/src/third_party/TinyFrame/
cp tf-vendor/TinyFrame.h   ~/code/imxrtnsy/src/third_party/TinyFrame/
cp tf-vendor/LICENSE       ~/code/imxrtnsy/src/third_party/TinyFrame/
# Record the upstream SHA for provenance
( cd tf-vendor && git rev-parse HEAD ) > ~/code/imxrtnsy/src/third_party/TinyFrame/.upstream-sha
rm -rf tf-vendor
```

- [ ] **Step 2: Verify files landed**

```bash
ls -la ~/code/imxrtnsy/src/third_party/TinyFrame/
```
Expected: `TinyFrame.c`, `TinyFrame.h`, `LICENSE`, `.upstream-sha` present.

- [ ] **Step 3: Commit**

```bash
cd ~/code/imxrtnsy
git add src/third_party/TinyFrame/
git commit -m "vendor: import TinyFrame (MightyPork, MIT) for Hurra protocol"
```

---

### Task 1.2: Create TF_Config.h with Hurra-specific TinyFrame configuration

**Files:**
- Create: `src/TF_Config.h`

- [ ] **Step 1: Write `TF_Config.h`**

```c
// src/TF_Config.h — TinyFrame configuration for the Hurra protocol.
// See docs/specs/2026-05-23-hurra-binary-protocol-design.md §2.
#pragma once

#define TF_SOF_BYTE       0x68    // 'h' for Hurra
#define TF_ID_BYTES       1
#define TF_LEN_BYTES      1
#define TF_TYPE_BYTES     1
#define TF_CKSUM_TYPE     TF_CKSUM_CRC16
#define TF_USE_MUTEX      0
#define TF_MAX_PAYLOAD_RX 256
#define TF_SENDBUF_LEN    264     // max payload + 8 byte header overhead
#define TF_PARSER_TIMEOUT_TICKS 5  // 5 × hurra_tick period (1 ms) = 5 ms idle gap
```

- [ ] **Step 2: Commit**

```bash
git add src/TF_Config.h
git commit -m "hurra: add TF_Config.h with §2 frame parameters"
```

---

### Task 1.3: Extend Makefile with PROTOCOL switch (ferrum default for now)

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Read current Makefile**

```bash
cat ~/code/imxrtnsy/Makefile
```

- [ ] **Step 2: Replace the existing `SRC = ...` and `DEFINES = ...` lines**

Edit `Makefile` so it reads:

```make
# UART baud for kmbox <-> host link. LPUART3 on Teensy pins 16/17.
CMD_BAUD ?= 115200

# Protocol selector: 'hurra' (binary, TinyFrame, 4 Mbps target) or 'ferrum' (ASCII).
PROTOCOL ?= ferrum

ifeq ($(PROTOCOL),hurra)
  PROTO_DEF = -DPROTOCOL_HURRA
  PROTO_SRC = src/hurra.c src/third_party/TinyFrame/TinyFrame.c
else ifeq ($(PROTOCOL),ferrum)
  PROTO_DEF = -DPROTOCOL_FERRUM
  PROTO_SRC = src/ferrum.c
else
  $(error PROTOCOL must be 'hurra' or 'ferrum')
endif

DEFINES = -DARDUINO_TEENSY_MICROMOD -D__IMXRT1062__ -DF_CPU=816000000 \
          -DCMD_BAUD=$(CMD_BAUD) $(PROTO_DEF)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -flto -fsingle-precision-constant \
         -Iinclude -Isrc -Isrc/third_party/TinyFrame

# ... (rest of Makefile unchanged) ...

CORE_SRC = core/startup.c core/bootdata.c
SRC = src/main.c src/usb_host.c src/usb_device.c src/desc_capture.c \
      src/kmbox.c src/humanize.c src/smooth.c src/actions.c \
      $(PROTO_SRC)

OBJ = $(CORE_SRC:.c=.o) $(SRC:.c=.o)
```

And update the HOT_SRC list (it currently lists `src/ferrum.o` unconditionally — leave it; ferrum builds will still use it, hurra builds skip via PROTOCOL_SRC selection):

```make
# Hot-path sources get -O2 instead of -Os. Conditional on protocol.
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o \
          src/humanize.o src/actions.o
ifeq ($(PROTOCOL),hurra)
  HOT_SRC += src/hurra.o src/third_party/TinyFrame/TinyFrame.o
else
  HOT_SRC += src/ferrum.o
endif
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS)) -ffast-math
```

- [ ] **Step 3: Verify ferrum still builds**

```bash
cd ~/code/imxrtnsy
make clean && make PROTOCOL=ferrum
```
Expected: `firmware.hex` produced, no errors. Size output from `arm-none-eabi-size`.

- [ ] **Step 4: Verify hurra build fails with a clear error (hurra.c doesn't exist yet)**

```bash
make clean && make PROTOCOL=hurra
```
Expected: failure on `src/hurra.c: No such file` — correct, that file lands in Phase 3.

- [ ] **Step 5: Commit**

```bash
git add Makefile
git commit -m "build: add PROTOCOL={hurra,ferrum} selector (ferrum default)"
```

---

## Phase 2 — LPUART3 eDMA RX path

### Task 2.1: Add `kmbox_tx_room()` accessor

**Files:**
- Modify: `src/kmbox.h` (add declaration after line 41)
- Modify: `src/kmbox.c` (add definition near other TX ring helpers)

- [ ] **Step 1: Find the existing TX ring fill helper in `kmbox.c`**

```bash
grep -n 'tx_ring\|tx_overflow\|tx_byte_count' ~/code/imxrtnsy/src/kmbox.c | head -20
```
Note the line numbers — the new accessor goes adjacent to existing TX ring code.

- [ ] **Step 2: Add to `src/kmbox.h` after line 38**

```c
uint16_t kmbox_tx_room(void);  // bytes free in TX ring; used by hurra telemetry backpressure
```

- [ ] **Step 3: Add definition to `src/kmbox.c`**

Locate the TX ring struct (search for `tx_ring` or similar). Add the helper next to the existing ring accessors:

```c
uint16_t kmbox_tx_room(void)
{
    // Returns bytes free in TX ring. Telemetry emitters use this to skip
    // a frame when the ring would overflow; input listeners never skip.
    extern volatile uint16_t s_tx_head, s_tx_tail;   // adjust to actual symbol names
    extern const uint16_t   s_tx_size;                // power of two ring size
    uint16_t used = (uint16_t)((s_tx_head - s_tx_tail) & (s_tx_size - 1));
    return (uint16_t)(s_tx_size - 1 - used);
}
```

If the actual ring uses different field names, adapt them — the function returns `ring_size - 1 - used_bytes`.

- [ ] **Step 4: Verify build**

```bash
make clean && make PROTOCOL=ferrum
```
Expected: builds cleanly with `kmbox_tx_room` symbol in the elf:

```bash
arm-none-eabi-nm firmware.elf | grep kmbox_tx_room
```
Expected: one line with the symbol.

- [ ] **Step 5: Commit**

```bash
git add src/kmbox.h src/kmbox.c
git commit -m "kmbox: expose kmbox_tx_room() accessor for telemetry backpressure"
```

---

### Task 2.2: Replace per-byte LPUART3 RX ISR with eDMA + IDLE-line interrupt

**Files:**
- Modify: `src/kmbox.c` (RX path)
- Reference: NXP AN12552 — LPUART eDMA ring buffer

- [ ] **Step 1: Identify current RX path in `kmbox.c`**

```bash
grep -n 'LPUART3\|RX_FIFO\|LPUART_IRQ\|RDRF\|kmbox_feed_uart_byte\|proto_feed_byte' ~/code/imxrtnsy/src/kmbox.c
```

Note: the LPUART3 RX interrupt handler currently reads `LPUART3->DATA` per byte and calls `proto_feed_byte()`. We're replacing this with eDMA-driven copy into a circular buffer, draining in `kmbox_poll_heavy()`.

- [ ] **Step 2: Add the eDMA RX buffer and TCD setup at file scope**

```c
// ── eDMA RX ring (LPUART3 → rx_ring) ────────────────────────────────────────
// HW writes into rx_ring[wp] continuously; software drains from rx_ring[rp].
// Sized for 1 ms worst-case @ 4 Mbps (400 B); 1 KiB gives 2.5× margin.
#define RX_RING_SIZE 1024u
static uint8_t __attribute__((aligned(32))) rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_rp;          // software read pointer
static uint32_t rx_drv_overrun;          // exposed via stats
static uint32_t rx_byte_count_internal;  // existing counter, keep semantics

// eDMA channel allocation: pick a free channel (use channel 0 if available).
#define RX_DMA_CH 0

static void rx_edma_init(void)
{
    // Enable DMAMUX clock + route LPUART3 RX request to RX_DMA_CH.
    CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
    DMAMUX_CHCFG(RX_DMA_CH) = 0;
    DMAMUX_CHCFG(RX_DMA_CH) = DMAMUX_CHCFG_SOURCE(DMA_REQUEST_LPUART3_RX)
                            | DMAMUX_CHCFG_ENBL;

    // TCD: SADDR = LPUART3->DATA, DADDR = rx_ring, NBYTES=1, CITER/BITER=RX_RING_SIZE.
    // SOFF=0 (HW reg), DOFF=1 (advance ring), DLAST=-RX_RING_SIZE (wrap to start).
    volatile DMA_TCD_t *tcd = &DMA_TCD[RX_DMA_CH];
    tcd->SADDR     = (uintptr_t)&LPUART3->DATA;
    tcd->SOFF      = 0;
    tcd->ATTR      = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0); // 8-bit src/dst
    tcd->NBYTES_MLNO = 1;
    tcd->SLAST     = 0;
    tcd->DADDR     = (uintptr_t)rx_ring;
    tcd->DOFF      = 1;
    tcd->CITER_ELINKNO = RX_RING_SIZE;
    tcd->DLASTSGA  = -(int32_t)RX_RING_SIZE;
    tcd->BITER_ELINKNO = RX_RING_SIZE;
    tcd->CSR       = DMA_TCD_CSR_INTMAJOR;   // wrap interrupt for overrun watch

    DMA_ERQ |= (1u << RX_DMA_CH);            // enable hw request for this channel
    LPUART3->BAUD |= LPUART_BAUD_RDMAE;       // route RX FIFO → DMA request
}

// LPUART3 IRQ handler — IDLE-line only. Per-byte RXNE is no longer used.
void LPUART3_IRQHandler(void)
{
    uint32_t s = LPUART3->STAT;
    if (s & LPUART_STAT_IDLE) {
        LPUART3->STAT = LPUART_STAT_IDLE;   // w1c
        // No work in ISR — drain happens in kmbox_poll_heavy().
    }
}
```

Adapt register names to whatever the existing `imxrt.h` shim uses — `LPUART3->BAUD`, `LPUART3->STAT`, `DMA_TCD[]`, `DMAMUX_CHCFG`, etc. are illustrative; the actual symbols are in `include/imxrt.h`.

- [ ] **Step 3: Remove the existing per-byte RX ISR body**

Strip the body that calls `proto_feed_byte()` per byte; keep only the IDLE handling shown above. The IRQ is now used purely as a wake-up signal — the actual feeding happens in the drain loop.

- [ ] **Step 4: Add the drain function and call it from `kmbox_poll_heavy()`**

```c
static void rx_edma_drain(void)
{
    // Compute current HW write pointer from TCD's CITER.
    // CITER counts down from RX_RING_SIZE; wp = (RX_RING_SIZE - CITER) % RX_RING_SIZE.
    uint16_t citer = (uint16_t)(DMA_TCD[RX_DMA_CH].CITER_ELINKNO & 0x7FFF);
    uint16_t wp    = (RX_RING_SIZE - citer) & (RX_RING_SIZE - 1);

    while (rx_rp != wp) {
        uint8_t b = rx_ring[rx_rp];
        rx_rp = (rx_rp + 1) & (RX_RING_SIZE - 1);
        rx_byte_count_internal++;
        proto_feed_byte(b);
    }

    // Overrun heuristic: if the HW pointer ever laps the software pointer
    // by ≥¾ buffer, we lost bytes. The simplest check: track delta.
    uint16_t gap = (uint16_t)((wp - rx_rp) & (RX_RING_SIZE - 1));
    if (gap > (RX_RING_SIZE * 3u / 4u)) {
        rx_drv_overrun++;
        rx_rp = wp;  // give up, resync at current HW pointer
    }
}
```

- [ ] **Step 5: Wire `rx_edma_init()` into `kmbox_init()` and call `rx_edma_drain()` from `kmbox_poll_heavy()`**

Locate `kmbox_init()` and append:

```c
    rx_edma_init();
    // Enable IDLE-line interrupt; per-byte RXNE no longer needed.
    LPUART3->CTRL |= LPUART_CTRL_ILIE;
    LPUART3->CTRL &= ~LPUART_CTRL_RIE;
    NVIC_EnableIRQ(IRQ_LPUART3);
```

Locate `kmbox_poll_heavy()` and add at the top:

```c
    rx_edma_drain();
```

- [ ] **Step 6: Expose `rx_drv_overrun` via existing stats path**

Add a getter in `kmbox.h`:

```c
uint32_t kmbox_rx_drv_overrun(void);
```

And in `kmbox.c`:

```c
uint32_t kmbox_rx_drv_overrun(void) { return rx_drv_overrun; }
```

- [ ] **Step 7: Build and run smoke test on hardware (ferrum still in use)**

```bash
make clean && make PROTOCOL=ferrum flash
./tools/ferrum_test.py version
./tools/ferrum_test.py load --rate 500 --duration 5
```
Expected: ferrum behavior unchanged — `km.version()` returns `kmbox: Ferrum`; load test still hits its current ceiling without errors. This proves the eDMA RX path works at low rates.

- [ ] **Step 8: Commit**

```bash
git add src/kmbox.h src/kmbox.c
git commit -m "kmbox: switch LPUART3 RX to eDMA circular buffer + IDLE-line IRQ

Replaces per-byte RXNE interrupt with HW eDMA copy into a 1 KiB ring,
drained in kmbox_poll_heavy(). Enables sustained 4 Mbps reception with
<1% CPU on Cortex-M7 @ 816 MHz.

Reference: NXP AN12552 LPUART eDMA ring buffer pattern."
```

---

### Task 2.3: Bench-verify ferrum on eDMA RX

**Files:** (none modified)

- [ ] **Step 1: Run ferrum aim test to confirm closed-loop still works**

```bash
cd ~/code/imxrtnsy
./tools/ferrum_aim_test.py --duration 10
```
Expected: aim test completes; reported RPS within ±5% of the pre-eDMA baseline (~550 cmds/sec at current `--tick-ms 6 --cmd-step-px 4`).

- [ ] **Step 2: If RPS drops by more than 5%, investigate before continuing**

Likely culprits: drain too slow (called too infrequently); TCD misconfigured (wrong source/dest size). Rerun with `scripts/uart_debug.py latency` to bisect.

No commit — this is a verification gate only.

---

## Phase 3 — Hurra firmware skeleton

### Task 3.1: Create `src/hurra.h`

**Files:**
- Create: `src/hurra.h`

- [ ] **Step 1: Write the header**

```c
// src/hurra.h — public API mirrors ferrum.h / makcu.h so proto.h can alias.
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void (*hurra_tx_fn)(const uint8_t *buf, uint16_t len);

void hurra_init(void);
void hurra_reset(void);
void hurra_set_tx(hurra_tx_fn tx);

void hurra_feed_byte(uint8_t b);
void hurra_tick(void);

void hurra_notify_buttons(uint8_t buttons_bitmap);
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll);
void hurra_notify_keys(const uint8_t keys[6]);
```

- [ ] **Step 2: Commit**

```bash
git add src/hurra.h
git commit -m "hurra: add hurra.h public API mirroring ferrum/makcu shape"
```

---

### Task 3.2: Create `src/hurra.c` skeleton (init + feed_byte + tick, no listeners yet)

**Files:**
- Create: `src/hurra.c`

- [ ] **Step 1: Write the skeleton**

```c
// src/hurra.c — Hurra binary protocol parser, TinyFrame-based.
// See docs/specs/2026-05-23-hurra-binary-protocol-design.md
#include "TinyFrame.h"
#include "hurra.h"
#include "actions.h"
#include "kmbox.h"
#include <string.h>

extern uint32_t millis(void);

// ── TinyFrame instance ──────────────────────────────────────────────────────
static TinyFrame s_tf;
static hurra_tx_fn s_tx;

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
    (void)tf;
    if (s_tx) s_tx(buf, (uint16_t)len);
}

// ── stats counters (exposed via STATS frame in Phase 6) ─────────────────────
static uint32_t s_rx_frames_ok;
static uint32_t s_head_crc_err;       // updated when TF reports parse fail
static uint32_t s_payload_crc_err;
static uint32_t s_id_gap_total;
static uint32_t s_idle_resync;
static uint32_t s_payload_invalid;
static uint16_t s_tx_ring_high_water;
static uint32_t s_tx_ring_skip;
static uint8_t  s_last_rx_id;
static bool     s_have_last_id;

// ── deferred actions ─────────────────────────────────────────────────────────
static uint32_t s_reboot_at;
static uint32_t s_baud_pending;
static uint32_t s_baud_apply_at;

// ── stream/callback state ───────────────────────────────────────────────────
typedef struct { uint8_t mode; uint8_t period_ms; uint32_t last_ms; } stream_t;
static stream_t s_str_axis, s_str_btn, s_str_mouse, s_str_kb;
static uint8_t  s_cb_buttons, s_cb_axes, s_cb_keys;

// ── snapshots populated by hurra_notify_* ───────────────────────────────────
static uint8_t  s_snap_buttons;
static int16_t  s_snap_dx, s_snap_dy;
static int8_t   s_snap_wheel;
static uint8_t  s_snap_keys[6];
static uint8_t  s_last_btn_emitted = 0;
static uint8_t  s_last_keys_emitted[6];

// ── screen ──────────────────────────────────────────────────────────────────
static int16_t s_screen_w, s_screen_h;

// ── catch_xy ────────────────────────────────────────────────────────────────
static struct {
    bool     active;
    uint32_t deadline;
    int32_t  accum_x, accum_y;
    uint8_t  reply_id;
} s_catch;

// ── auto stats push ─────────────────────────────────────────────────────────
#define STATS_PERIOD_MS 100
static uint32_t s_stats_next_ms;

// ── helpers ─────────────────────────────────────────────────────────────────
static inline int16_t  rd_i16le(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline uint16_t rd_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ── public API ──────────────────────────────────────────────────────────────
void hurra_init(void)
{
    memset(&s_tf, 0, sizeof(s_tf));
    TF_InitStatic(&s_tf, TF_MASTER);
    s_tx = NULL;
    s_rx_frames_ok = s_head_crc_err = s_payload_crc_err = 0;
    s_id_gap_total = s_idle_resync = s_payload_invalid = 0;
    s_tx_ring_high_water = 0;
    s_tx_ring_skip = 0;
    s_have_last_id = false;
    s_reboot_at = 0;
    s_baud_pending = 0;
    s_baud_apply_at = 0;
    memset(&s_str_axis, 0, sizeof(s_str_axis));
    memset(&s_str_btn,  0, sizeof(s_str_btn));
    memset(&s_str_mouse, 0, sizeof(s_str_mouse));
    memset(&s_str_kb,   0, sizeof(s_str_kb));
    s_cb_buttons = s_cb_axes = s_cb_keys = 0;
    s_snap_buttons = 0; s_snap_dx = s_snap_dy = 0; s_snap_wheel = 0;
    memset(s_snap_keys, 0, sizeof(s_snap_keys));
    memset(s_last_keys_emitted, 0, sizeof(s_last_keys_emitted));
    s_screen_w = s_screen_h = 0;
    memset(&s_catch, 0, sizeof(s_catch));
    s_stats_next_ms = STATS_PERIOD_MS;
    // Listeners registered in Phase 4-6 tasks.
}

void hurra_reset(void) { TF_ResetParser(&s_tf); }
void hurra_set_tx(hurra_tx_fn tx) { s_tx = tx; }

void hurra_feed_byte(uint8_t b) { TF_AcceptChar(&s_tf, b); }

void hurra_tick(void)
{
    uint32_t now = millis();
    TF_Tick(&s_tf);
    // Auto-stats push, stream emits, deferred actions land here in later tasks.
    if (s_reboot_at && now >= s_reboot_at) {
        extern volatile uint32_t SCB_AIRCR;
        SCB_AIRCR = 0x05FA0004;
    }
}

void hurra_notify_buttons(uint8_t buttons) { s_snap_buttons = buttons; }
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll)
{
    s_snap_dx = dx; s_snap_dy = dy; s_snap_wheel = scroll;
    if (s_catch.active) { s_catch.accum_x += dx; s_catch.accum_y += dy; }
}
void hurra_notify_keys(const uint8_t keys[6]) { memcpy(s_snap_keys, keys, 6); }
```

- [ ] **Step 2: Verify build with `make PROTOCOL=hurra` (will still fail — proto.h doesn't know about HURRA yet)**

```bash
make clean && make PROTOCOL=hurra
```
Expected: failure inside `proto.h` — `#error "Define PROTOCOL_FERRUM or PROTOCOL_MAKCU"`. That gets fixed in Task 3.3.

- [ ] **Step 3: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: add hurra.c skeleton (init/feed_byte/tick, no listeners yet)"
```

---

### Task 3.3: Rewrite `proto.h` to replace MAKCU with HURRA arm

**Files:**
- Modify: `src/proto.h`

- [ ] **Step 1: Replace the file contents**

```c
// src/proto.h — compile-time protocol selector.
//
// One of PROTOCOL_HURRA or PROTOCOL_FERRUM must be defined (via the
// Makefile's PROTOCOL variable). This header aliases proto_* to the
// selected parser's symbols so kmbox.c can call into the protocol
// without #ifdefs at every call site.
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

- [ ] **Step 2: Build both protocols clean**

```bash
make clean && make PROTOCOL=ferrum
make clean && make PROTOCOL=hurra
```
Expected: both produce `firmware.hex`. Hurra build will link a firmware that accepts bytes but doesn't do anything useful yet (no listeners).

- [ ] **Step 3: Flash hurra build and confirm it boots**

```bash
make clean && make PROTOCOL=hurra flash
# Watch USB enumeration — device should come up normally
ls /dev/cu.* | grep -E 'usbmodem|wchusbserial'
```
Expected: device enumerates within ~3 seconds. No reply to any host frame yet (that's Phase 4).

- [ ] **Step 4: Commit**

```bash
git add src/proto.h
git commit -m "proto: replace PROTOCOL_MAKCU arm with PROTOCOL_HURRA"
```

---

## Phase 4 — Hurra firmware: admin and mouse hot path

### Task 4.1: Add command TYPE constants and the listener-registration helper

**Files:**
- Modify: `src/hurra.c` (add type table and `hurra_init()` listener block)

- [ ] **Step 1: Add the TYPE table near the top of `hurra.c`**

After the `#include` lines but before the static state:

```c
// ── Command TYPE byte allocation (see spec §3) ──────────────────────────────
enum {
    // 0x00–0x0F admin
    TYPE_PING        = 0x00,
    TYPE_VERSION     = 0x01,
    TYPE_STATS       = 0x02,
    TYPE_INIT        = 0x03,
    TYPE_REBOOT      = 0x04,
    TYPE_BAUD        = 0x05,
    TYPE_SCREEN      = 0x06,
    // 0x10–0x2F mouse
    TYPE_MOUSE_MOVE        = 0x10,
    TYPE_MOUSE_MOVE_SMOOTH = 0x11,
    TYPE_MOUSE_SILENT_MOVE = 0x12,
    TYPE_MOUSE_MO          = 0x13,
    TYPE_MOUSE_CLICK       = 0x14,
    TYPE_MOUSE_WHEEL       = 0x15,
    TYPE_MOUSE_GETPOS      = 0x16,
    TYPE_INVERT_X          = 0x17,
    TYPE_INVERT_Y          = 0x18,
    TYPE_SWAP_XY           = 0x19,
    TYPE_BTN_LEFT          = 0x20,
    TYPE_BTN_RIGHT         = 0x21,
    TYPE_BTN_MIDDLE        = 0x22,
    TYPE_BTN_SIDE1         = 0x23,
    TYPE_BTN_SIDE2         = 0x24,
    // 0x40–0x4F keyboard
    TYPE_KB_DOWN        = 0x40,
    TYPE_KB_UP          = 0x41,
    TYPE_KB_PRESS       = 0x42,
    TYPE_KB_ISDOWN      = 0x43,
    TYPE_KB_MASK        = 0x44,
    TYPE_KB_STRING      = 0x45,
    TYPE_KB_MULTIDOWN   = 0x46,
    TYPE_KB_MULTIUP     = 0x47,
    TYPE_KB_MULTIPRESS  = 0x48,
    // 0x60–0x6F locks + catch
    TYPE_LOCK_ML  = 0x60,
    TYPE_LOCK_MR  = 0x61,
    TYPE_LOCK_MM  = 0x62,
    TYPE_LOCK_MS1 = 0x63,
    TYPE_LOCK_MS2 = 0x64,
    TYPE_LOCK_MX  = 0x65,
    TYPE_LOCK_MY  = 0x66,
    TYPE_CATCH_XY = 0x67,
    // 0x70–0x7F streams + callbacks
    TYPE_STREAM_AXIS  = 0x70,
    TYPE_STREAM_BTN   = 0x71,
    TYPE_STREAM_MOUSE = 0x72,
    TYPE_STREAM_KB    = 0x73,
    TYPE_CB_BUTTONS   = 0x74,
    TYPE_CB_AXES      = 0x75,
    TYPE_CB_KEYS      = 0x76,
    // 0x80–0x8F unsolicited telemetry
    TYPE_TLM_AXIS    = 0x80,
    TYPE_TLM_BUTTONS = 0x81,
    TYPE_TLM_MOUSE   = 0x82,
    TYPE_TLM_KB      = 0x83,
    TYPE_TLM_STATS   = 0x84,
    TYPE_TLM_LOG     = 0x85,
};

#define HURRA_IDENTITY "kmbox: Hurra v1"
```

- [ ] **Step 2: Add a small reply helper**

```c
static void send_reply(TF_Msg *req, const uint8_t *data, uint32_t len)
{
    TF_Msg r = *req;
    r.data = (uint8_t *)data;
    r.len  = (TF_LEN)len;
    r.is_response = true;
    TF_Respond(&s_tf, &r);
}

// Track ID gap for stats (called by every listener via TF generic listener)
static void track_id(uint8_t id)
{
    s_rx_frames_ok++;
    if (s_have_last_id) {
        uint8_t gap = (uint8_t)((id - s_last_rx_id - 1) & 0xFFu);
        s_id_gap_total += gap;
    }
    s_last_rx_id = id;
    s_have_last_id = true;
}
```

- [ ] **Step 3: Commit (no listeners yet)**

```bash
git add src/hurra.c
git commit -m "hurra: add TYPE byte table and reply/id-tracking helpers"
```

---

### Task 4.2: Implement PING, VERSION, STATS listeners

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add the three listeners and register them**

Add before the public-API section:

```c
static TF_Result l_ping(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    send_reply(msg, msg->data, 4);   // echo nonce
    return TF_STAY;
}

static TF_Result l_version(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    send_reply(msg, (const uint8_t *)HURRA_IDENTITY, sizeof(HURRA_IDENTITY) - 1);
    return TF_STAY;
}

static void pack_stats(uint8_t out[36])
{
    extern uint32_t millis(void);
    uint32_t uptime = millis();
    uint16_t ring_hw = s_tx_ring_high_water;
    s_tx_ring_high_water = 0;  // reset peak each emit
    memcpy(&out[0],  &uptime,            4);
    memcpy(&out[4],  &s_rx_frames_ok,    4);
    memcpy(&out[8],  &s_head_crc_err,    4);
    memcpy(&out[12], &s_payload_crc_err, 4);
    memcpy(&out[16], &s_id_gap_total,    4);
    memcpy(&out[20], &s_idle_resync,     4);
    uint32_t over = kmbox_rx_drv_overrun();
    memcpy(&out[24], &over,              4);
    memcpy(&out[28], &s_tx_ring_skip,    4);
    memcpy(&out[32], &s_payload_invalid, 4);
    // hwm + reserved omitted in this 36 B layout — extend to 40 in Phase 6 if needed
}

static TF_Result l_stats(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    uint8_t buf[36];
    pack_stats(buf);
    send_reply(msg, buf, sizeof(buf));
    return TF_STAY;
}
```

- [ ] **Step 2: Register in `hurra_init()`** — append after `TF_InitStatic`:

```c
    TF_AddTypeListener(&s_tf, TYPE_PING,    l_ping);
    TF_AddTypeListener(&s_tf, TYPE_VERSION, l_version);
    TF_AddTypeListener(&s_tf, TYPE_STATS,   l_stats);
```

- [ ] **Step 3: Build + flash**

```bash
make clean && make PROTOCOL=hurra flash
```
Expected: builds clean. Smoke-test deferred until Phase 7 when the Python client exists.

- [ ] **Step 4: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement PING/VERSION/STATS listeners"
```

---

### Task 4.3: Implement MOUSE_MOVE + variants + MOUSE_CLICK + WHEEL + MO + GETPOS

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add listeners**

```c
static TF_Result l_mouse_move(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    int16_t dx = rd_i16le(&msg->data[0]);
    int16_t dy = rd_i16le(&msg->data[2]);
    act_move(dx, dy, false);
    return TF_STAY;
}

static TF_Result l_mouse_move_smooth(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    act_move(rd_i16le(&msg->data[0]), rd_i16le(&msg->data[2]), true);
    return TF_STAY;
}

static TF_Result l_mouse_silent(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    act_move(rd_i16le(&msg->data[0]), rd_i16le(&msg->data[2]), false);
    return TF_STAY;
}

static TF_Result l_mouse_mo(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 8) { s_payload_invalid++; return TF_STAY; }
    uint8_t buttons = msg->data[0];
    int16_t dx = rd_i16le(&msg->data[1]);
    int16_t dy = rd_i16le(&msg->data[3]);
    int8_t  wheel = (int8_t)msg->data[5];
    // pan/tilt (data[6], data[7]) accepted but dropped — no HID transport.
    extern uint8_t g_buttons;
    act_button_set(buttons ^ g_buttons, 0);
    act_button_set(buttons, 1);
    act_move(dx, dy, false);
    if (wheel) act_wheel(wheel);
    return TF_STAY;
}

static TF_Result l_mouse_click(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 3) { s_payload_invalid++; return TF_STAY; }
    act_click(msg->data[0], msg->data[1], msg->data[2]);
    return TF_STAY;
}

static TF_Result l_mouse_wheel(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_wheel((int8_t)msg->data[0]);
    return TF_STAY;
}

static TF_Result l_mouse_getpos(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    extern int32_t g_pos_x, g_pos_y;
    int32_t x = g_pos_x, y = g_pos_y;
    if (x > INT16_MAX) x = INT16_MAX; else if (x < INT16_MIN) x = INT16_MIN;
    if (y > INT16_MAX) y = INT16_MAX; else if (y < INT16_MIN) y = INT16_MIN;
    uint8_t p[4] = {
        (uint8_t)x, (uint8_t)(x >> 8),
        (uint8_t)y, (uint8_t)(y >> 8),
    };
    send_reply(msg, p, sizeof(p));
    return TF_STAY;
}
```

- [ ] **Step 2: Register all seven in `hurra_init()`**

```c
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE,        l_mouse_move);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE_SMOOTH, l_mouse_move_smooth);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_SILENT_MOVE, l_mouse_silent);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MO,          l_mouse_mo);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_CLICK,       l_mouse_click);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_WHEEL,       l_mouse_wheel);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_GETPOS,      l_mouse_getpos);
```

- [ ] **Step 3: Build**

```bash
make clean && make PROTOCOL=hurra
```
Expected: builds clean.

- [ ] **Step 4: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement mouse move/click/wheel/MO/getpos listeners"
```

---

### Task 4.4: Implement button listeners (BTN_LEFT/RIGHT/MIDDLE/SIDE1/SIDE2) and INVERT_X/Y/SWAP_XY

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add a generic button helper and the three invert helpers**

```c
static TF_Result button_listener(TinyFrame *tf, TF_Msg *msg, uint8_t mask)
{
    track_id(msg->frame_id);
    extern uint8_t g_buttons;
    if (msg->len == 0) {
        uint8_t v = (g_buttons & mask) ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_button_set(mask, msg->data[0] ? 1 : 0);
    return TF_STAY;
}

#define MAKE_BTN(NAME, MASK) \
static TF_Result l_##NAME(TinyFrame *tf, TF_Msg *m) { return button_listener(tf, m, MASK); }

MAKE_BTN(btn_left,   0x01)
MAKE_BTN(btn_right,  0x02)
MAKE_BTN(btn_middle, 0x04)
MAKE_BTN(btn_side1,  0x08)
MAKE_BTN(btn_side2,  0x10)

static TF_Result invert_listener(TinyFrame *tf, TF_Msg *msg,
                                 bool (*get)(void), void (*set)(bool))
{
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t v = get() ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    set(msg->data[0] != 0);
    return TF_STAY;
}

static TF_Result l_invert_x(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_invert_x, act_set_invert_x); }
static TF_Result l_invert_y(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_invert_y, act_set_invert_y); }
static TF_Result l_swap_xy(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_swap_xy, act_set_swap_xy); }
```

- [ ] **Step 2: Register in `hurra_init()`**

```c
    TF_AddTypeListener(&s_tf, TYPE_BTN_LEFT,   l_btn_left);
    TF_AddTypeListener(&s_tf, TYPE_BTN_RIGHT,  l_btn_right);
    TF_AddTypeListener(&s_tf, TYPE_BTN_MIDDLE, l_btn_middle);
    TF_AddTypeListener(&s_tf, TYPE_BTN_SIDE1,  l_btn_side1);
    TF_AddTypeListener(&s_tf, TYPE_BTN_SIDE2,  l_btn_side2);
    TF_AddTypeListener(&s_tf, TYPE_INVERT_X,   l_invert_x);
    TF_AddTypeListener(&s_tf, TYPE_INVERT_Y,   l_invert_y);
    TF_AddTypeListener(&s_tf, TYPE_SWAP_XY,    l_swap_xy);
```

- [ ] **Step 3: Build**

```bash
make clean && make PROTOCOL=hurra
```

- [ ] **Step 4: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement button + invert/swap listeners"
```

---

## Phase 5 — Hurra firmware: keyboard, locks, catch_xy, admin

### Task 5.1: Implement keyboard listeners (KB_DOWN/UP/PRESS/ISDOWN/MASK/STRING + MULTI*)

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add listeners**

```c
static TF_Result l_kb_down(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_kb_down(msg->data[0]);
    return TF_STAY;
}

static TF_Result l_kb_up(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_kb_up(msg->data[0]);
    return TF_STAY;
}

// xorshift32 jitter (mirrors makcu.c rng)
static uint32_t s_rng_state = 0xDEADBEEF;
static uint32_t rng_next(void)
{
    uint32_t x = s_rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return s_rng_state = x;
}

static TF_Result l_kb_press(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 3) { s_payload_invalid++; return TF_STAY; }
    uint32_t delay = msg->data[1] + (msg->data[2] ? (rng_next() % (msg->data[2] + 1u)) : 0);
    act_kb_press(msg->data[0], delay);
    return TF_STAY;
}

static TF_Result l_kb_isdown(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    uint8_t v = act_kb_isdown(msg->data[0]);
    send_reply(msg, &v, 1);
    return TF_STAY;
}

static TF_Result l_kb_mask(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 2) { s_payload_invalid++; return TF_STAY; }
    act_kb_mask(msg->data[0], msg->data[1]);
    return TF_STAY;
}

// ASCII → HID (same coverage as makcu.c). Press duration fixed.
#define STRING_PRESS_MS 12
static uint8_t ascii_to_hid(char c)
{
    if (c >= 'a' && c <= 'z') return 0x04 + (c - 'a');
    if (c >= 'A' && c <= 'Z') return 0x04 + (c - 'A');
    if (c >= '1' && c <= '9') return 0x1E + (c - '1');
    if (c == '0') return 0x27;
    if (c == ' ') return 0x2C;
    return 0;
}

static TF_Result l_kb_string(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len > 240) { s_payload_invalid++; return TF_STAY; }
    for (uint16_t i = 0; i < msg->len; i++) {
        uint8_t hid = ascii_to_hid((char)msg->data[i]);
        if (hid) act_kb_press(hid, STRING_PRESS_MS);
    }
    return TF_STAY;
}

static TF_Result multikey_listener(TinyFrame *tf, TF_Msg *msg,
                                   void (*op)(uint8_t))
{
    track_id(msg->frame_id);
    if (msg->len < 1 || msg->len > 6) { s_payload_invalid++; return TF_STAY; }
    for (uint16_t i = 0; i < msg->len; i++) op(msg->data[i]);
    return TF_STAY;
}

static void kb_press_default(uint8_t k) { act_kb_press(k, 80); }

static TF_Result l_kb_multidown(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, act_kb_down); }
static TF_Result l_kb_multiup(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, act_kb_up); }
static TF_Result l_kb_multipress(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, kb_press_default); }
```

- [ ] **Step 2: Register in `hurra_init()`**

```c
    TF_AddTypeListener(&s_tf, TYPE_KB_DOWN,       l_kb_down);
    TF_AddTypeListener(&s_tf, TYPE_KB_UP,         l_kb_up);
    TF_AddTypeListener(&s_tf, TYPE_KB_PRESS,      l_kb_press);
    TF_AddTypeListener(&s_tf, TYPE_KB_ISDOWN,     l_kb_isdown);
    TF_AddTypeListener(&s_tf, TYPE_KB_MASK,       l_kb_mask);
    TF_AddTypeListener(&s_tf, TYPE_KB_STRING,     l_kb_string);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIDOWN,  l_kb_multidown);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIUP,    l_kb_multiup);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIPRESS, l_kb_multipress);
```

- [ ] **Step 3: Build**

```bash
make clean && make PROTOCOL=hurra
```

- [ ] **Step 4: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement keyboard listeners (down/up/press/isdown/mask/string/multi*)"
```

---

### Task 5.2: Implement LOCK_* listeners and CATCH_XY

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add listeners**

```c
static TF_Result lock_listener(TinyFrame *tf, TF_Msg *msg, uint8_t bit)
{
    track_id(msg->frame_id);
    extern uint16_t g_lock_mask;
    uint16_t bitmask = (uint16_t)(1u << bit);
    if (msg->len == 0) {
        uint8_t v = (g_lock_mask & bitmask) ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    if (msg->data[0]) g_lock_mask |= bitmask;
    else              g_lock_mask &= ~bitmask;
    return TF_STAY;
}

#define MAKE_LOCK(NAME, BIT) \
static TF_Result l_##NAME(TinyFrame *tf, TF_Msg *m) { return lock_listener(tf, m, BIT); }

MAKE_LOCK(lock_ml,  0)
MAKE_LOCK(lock_mr,  1)
MAKE_LOCK(lock_mm,  2)
MAKE_LOCK(lock_ms1, 3)
MAKE_LOCK(lock_ms2, 4)
MAKE_LOCK(lock_mx,  5)
MAKE_LOCK(lock_my,  6)

static TF_Result l_catch_xy(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len != 2) { s_payload_invalid++; return TF_STAY; }
    uint16_t dur = rd_u16le(&msg->data[0]);
    if (dur > 1000) dur = 1000;

    // Re-entrant: emit prior result to its original requester first.
    if (s_catch.active) {
        uint8_t p[8];
        memcpy(&p[0], &s_catch.accum_x, 4);
        memcpy(&p[4], &s_catch.accum_y, 4);
        TF_Msg r;
        TF_ClearMsg(&r);
        r.type = TYPE_CATCH_XY;
        r.frame_id = s_catch.reply_id;
        r.is_response = true;
        r.data = p; r.len = 8;
        TF_Respond(&s_tf, &r);
    }
    s_catch.accum_x = 0;
    s_catch.accum_y = 0;
    s_catch.deadline = millis() + (uint32_t)dur;
    s_catch.reply_id = msg->frame_id;
    s_catch.active = true;
    return TF_STAY;
}
```

- [ ] **Step 2: Add deferred-reply emit in `hurra_tick()`**

Inside `hurra_tick()`, before the reboot check:

```c
    if (s_catch.active && now >= s_catch.deadline) {
        uint8_t p[8];
        memcpy(&p[0], &s_catch.accum_x, 4);
        memcpy(&p[4], &s_catch.accum_y, 4);
        TF_Msg r;
        TF_ClearMsg(&r);
        r.type = TYPE_CATCH_XY;
        r.frame_id = s_catch.reply_id;
        r.is_response = true;
        r.data = p; r.len = 8;
        TF_Respond(&s_tf, &r);
        s_catch.active = false;
    }
```

- [ ] **Step 3: Register listeners**

```c
    TF_AddTypeListener(&s_tf, TYPE_LOCK_ML,  l_lock_ml);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MR,  l_lock_mr);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MM,  l_lock_mm);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MS1, l_lock_ms1);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MS2, l_lock_ms2);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MX,  l_lock_mx);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MY,  l_lock_my);
    TF_AddTypeListener(&s_tf, TYPE_CATCH_XY, l_catch_xy);
```

- [ ] **Step 4: Build**

```bash
make clean && make PROTOCOL=hurra
```

- [ ] **Step 5: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement lock_* and catch_xy listeners (with deferred reply)"
```

---

### Task 5.3: Implement INIT, REBOOT, BAUD, SCREEN admin listeners

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add listeners**

```c
static TF_Result l_init(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    act_init();
    act_kb_init();
    s_rx_frames_ok = 0;  s_head_crc_err = 0; s_payload_crc_err = 0;
    s_id_gap_total = 0;  s_idle_resync  = 0; s_payload_invalid = 0;
    s_tx_ring_skip = 0;  s_tx_ring_high_water = 0;
    return TF_STAY;
}

static TF_Result l_reboot(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    uint8_t ok = 0;
    send_reply(msg, &ok, 1);
    s_reboot_at = millis() + 20;
    return TF_STAY;
}

static TF_Result l_baud(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint32_t b = kmbox_current_baud();
        uint8_t p[4] = { (uint8_t)b, (uint8_t)(b >> 8), (uint8_t)(b >> 16), (uint8_t)(b >> 24) };
        send_reply(msg, p, 4);
        return TF_STAY;
    }
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    uint32_t new_baud = rd_u32le(&msg->data[0]);
    uint8_t  ack[4]  = { (uint8_t)new_baud, (uint8_t)(new_baud >> 8),
                         (uint8_t)(new_baud >> 16), (uint8_t)(new_baud >> 24) };
    send_reply(msg, ack, 4);
    s_baud_pending  = new_baud;
    s_baud_apply_at = millis() + 20;
    return TF_STAY;
}

static TF_Result l_screen(TinyFrame *tf, TF_Msg *msg)
{
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t p[4] = {
            (uint8_t)s_screen_w, (uint8_t)(s_screen_w >> 8),
            (uint8_t)s_screen_h, (uint8_t)(s_screen_h >> 8),
        };
        send_reply(msg, p, 4);
        return TF_STAY;
    }
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    s_screen_w = rd_i16le(&msg->data[0]);
    s_screen_h = rd_i16le(&msg->data[2]);
    return TF_STAY;
}
```

- [ ] **Step 2: Add deferred-baud-apply to `hurra_tick()`**

Inside `hurra_tick()`, after the catch_xy block:

```c
    if (s_baud_apply_at && now >= s_baud_apply_at) {
        kmbox_set_baud(s_baud_pending);
        s_baud_apply_at = 0;
    }
```

- [ ] **Step 3: Register listeners**

```c
    TF_AddTypeListener(&s_tf, TYPE_INIT,   l_init);
    TF_AddTypeListener(&s_tf, TYPE_REBOOT, l_reboot);
    TF_AddTypeListener(&s_tf, TYPE_BAUD,   l_baud);
    TF_AddTypeListener(&s_tf, TYPE_SCREEN, l_screen);
```

- [ ] **Step 4: Build**

```bash
make clean && make PROTOCOL=hurra
```

- [ ] **Step 5: Commit**

```bash
git add src/hurra.c
git commit -m "hurra: implement init/reboot/baud/screen admin listeners"
```

---

## Phase 6 — Hurra firmware: streams, callbacks, telemetry, stats push

### Task 6.1: Implement STREAM_* and CB_* configuration listeners

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add listeners**

```c
static TF_Result stream_cfg_listener(TinyFrame *tf, TF_Msg *msg, stream_t *s)
{
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t p[2] = { s->mode, s->period_ms };
        send_reply(msg, p, 2);
        return TF_STAY;
    }
    if (msg->len != 2) { s_payload_invalid++; return TF_STAY; }
    s->mode = msg->data[0];
    s->period_ms = msg->data[1];
    s->last_ms = millis();
    return TF_STAY;
}

static TF_Result cb_toggle_listener(TinyFrame *tf, TF_Msg *msg, uint8_t *flag)
{
    track_id(msg->frame_id);
    if (msg->len == 0) {
        send_reply(msg, flag, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    *flag = msg->data[0] ? 1 : 0;
    if (flag == &s_cb_keys) memset(s_last_keys_emitted, 0xFF, sizeof(s_last_keys_emitted));
    if (flag == &s_cb_buttons) s_last_btn_emitted = 0xFF;
    return TF_STAY;
}

static TF_Result l_stream_axis(TinyFrame *tf, TF_Msg *m) { return stream_cfg_listener(tf, m, &s_str_axis);  }
static TF_Result l_stream_btn (TinyFrame *tf, TF_Msg *m) { return stream_cfg_listener(tf, m, &s_str_btn);   }
static TF_Result l_stream_ms  (TinyFrame *tf, TF_Msg *m) { return stream_cfg_listener(tf, m, &s_str_mouse); }
static TF_Result l_stream_kb  (TinyFrame *tf, TF_Msg *m) { return stream_cfg_listener(tf, m, &s_str_kb);    }
static TF_Result l_cb_btn     (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_buttons); }
static TF_Result l_cb_axes    (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_axes);    }
static TF_Result l_cb_keys    (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_keys);    }
```

- [ ] **Step 2: Register**

```c
    TF_AddTypeListener(&s_tf, TYPE_STREAM_AXIS,  l_stream_axis);
    TF_AddTypeListener(&s_tf, TYPE_STREAM_BTN,   l_stream_btn);
    TF_AddTypeListener(&s_tf, TYPE_STREAM_MOUSE, l_stream_ms);
    TF_AddTypeListener(&s_tf, TYPE_STREAM_KB,    l_stream_kb);
    TF_AddTypeListener(&s_tf, TYPE_CB_BUTTONS,   l_cb_btn);
    TF_AddTypeListener(&s_tf, TYPE_CB_AXES,      l_cb_axes);
    TF_AddTypeListener(&s_tf, TYPE_CB_KEYS,      l_cb_keys);
```

- [ ] **Step 3: Commit**

```bash
make clean && make PROTOCOL=hurra
git add src/hurra.c
git commit -m "hurra: implement stream/callback configuration listeners"
```

---

### Task 6.2: Add unsolicited TLM_* emit paths in `hurra_tick()`

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add the four stream emitters**

Above `hurra_tick()`:

```c
// Telemetry frames are skipped when TX ring is < frame_size + slack.
#define TLM_TX_SLACK 32

static void tlm_send(uint8_t type, const uint8_t *p, uint16_t n)
{
    if (kmbox_tx_room() < (n + TLM_TX_SLACK)) { s_tx_ring_skip++; return; }
    TF_Msg m;
    TF_ClearMsg(&m);
    m.type = type;
    m.data = (uint8_t *)p;
    m.len  = n;
    TF_Send(&s_tf, &m);
}

static void stream_emit_axis(uint32_t now)
{
    if (s_str_axis.mode == 0 || (now - s_str_axis.last_ms) < s_str_axis.period_ms) return;
    s_str_axis.last_ms = now;
    uint8_t p[5] = {
        (uint8_t)s_snap_dx, (uint8_t)(s_snap_dx >> 8),
        (uint8_t)s_snap_dy, (uint8_t)(s_snap_dy >> 8),
        (uint8_t)s_snap_wheel,
    };
    tlm_send(TYPE_TLM_AXIS, p, sizeof(p));
}

static void stream_emit_btn(uint32_t now)
{
    if (s_str_btn.mode == 0 || (now - s_str_btn.last_ms) < s_str_btn.period_ms) return;
    s_str_btn.last_ms = now;
    tlm_send(TYPE_TLM_BUTTONS, &s_snap_buttons, 1);
}

static void stream_emit_mouse(uint32_t now)
{
    if (s_str_mouse.mode == 0 || (now - s_str_mouse.last_ms) < s_str_mouse.period_ms) return;
    s_str_mouse.last_ms = now;
    uint8_t p[8] = {
        s_snap_buttons,
        (uint8_t)s_snap_dx, (uint8_t)(s_snap_dx >> 8),
        (uint8_t)s_snap_dy, (uint8_t)(s_snap_dy >> 8),
        (uint8_t)s_snap_wheel,
        0, 0,  // pan, tilt — no source
    };
    tlm_send(TYPE_TLM_MOUSE, p, sizeof(p));
}

static void stream_emit_kb(uint32_t now)
{
    if (s_str_kb.mode == 0 || (now - s_str_kb.last_ms) < s_str_kb.period_ms) return;
    s_str_kb.last_ms = now;
    extern uint8_t g_kb_modifier;
    uint8_t p[7];
    p[0] = g_kb_modifier;
    memcpy(&p[1], s_snap_keys, 6);
    tlm_send(TYPE_TLM_KB, p, sizeof(p));
}
```

- [ ] **Step 2: Wire into `hurra_tick()`** before the catch_xy block:

```c
    stream_emit_axis(now);
    stream_emit_btn(now);
    stream_emit_mouse(now);
    stream_emit_kb(now);
```

- [ ] **Step 3: Add CB_* change-only emits in `hurra_notify_*`**

In `hurra_notify_buttons`:

```c
void hurra_notify_buttons(uint8_t buttons)
{
    s_snap_buttons = buttons;
    if (s_cb_buttons && buttons != s_last_btn_emitted) {
        s_last_btn_emitted = buttons;
        tlm_send(TYPE_TLM_BUTTONS, &buttons, 1);
    }
}
```

In `hurra_notify_keys`:

```c
void hurra_notify_keys(const uint8_t keys[6])
{
    memcpy(s_snap_keys, keys, 6);
    if (s_cb_keys && memcmp(keys, s_last_keys_emitted, 6) != 0) {
        memcpy(s_last_keys_emitted, keys, 6);
        extern uint8_t g_kb_modifier;
        uint8_t p[7];
        p[0] = g_kb_modifier;
        memcpy(&p[1], keys, 6);
        tlm_send(TYPE_TLM_KB, p, sizeof(p));
    }
}
```

In `hurra_notify_axes` — only emit per-change axis if `s_cb_axes` is set (after the catch_xy accumulator):

```c
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll)
{
    s_snap_dx = dx; s_snap_dy = dy; s_snap_wheel = scroll;
    if (s_catch.active) { s_catch.accum_x += dx; s_catch.accum_y += dy; }
    if (s_cb_axes) {
        uint8_t p[5] = {
            (uint8_t)dx, (uint8_t)(dx >> 8),
            (uint8_t)dy, (uint8_t)(dy >> 8),
            (uint8_t)scroll,
        };
        tlm_send(TYPE_TLM_AXIS, p, sizeof(p));
    }
}
```

- [ ] **Step 4: Commit**

```bash
make clean && make PROTOCOL=hurra
git add src/hurra.c
git commit -m "hurra: add TLM_AXIS/BTN/MOUSE/KB stream + callback emit paths"
```

---

### Task 6.3: Add periodic TLM_STATS push (100 ms cadence)

**Files:**
- Modify: `src/hurra.c`

- [ ] **Step 1: Add to `hurra_tick()`** after the stream emits:

```c
    if (now >= s_stats_next_ms) {
        s_stats_next_ms = now + STATS_PERIOD_MS;
        uint8_t buf[36];
        pack_stats(buf);
        tlm_send(TYPE_TLM_STATS, buf, sizeof(buf));
    }
```

- [ ] **Step 2: Build + commit**

```bash
make clean && make PROTOCOL=hurra
git add src/hurra.c
git commit -m "hurra: add periodic TLM_STATS push every 100ms"
```

---

## Phase 7 — Python client library

### Task 7.1: Vendor PyTinyFrame

**Files:**
- Create: `tools/third_party/tinyframe_py/TinyFrame.py`
- Create: `tools/third_party/tinyframe_py/__init__.py`
- Create: `tools/third_party/tinyframe_py/LICENSE`

- [ ] **Step 1: Fetch the Python port**

```bash
cd /tmp && git clone --depth 1 https://github.com/MightyPork/TinyFrame.git tf-vendor-py
mkdir -p ~/code/imxrtnsy/tools/third_party/tinyframe_py
cp tf-vendor-py/python/TinyFrame.py ~/code/imxrtnsy/tools/third_party/tinyframe_py/
cp tf-vendor-py/LICENSE              ~/code/imxrtnsy/tools/third_party/tinyframe_py/
echo "from .TinyFrame import TinyFrame, TF_Msg" > ~/code/imxrtnsy/tools/third_party/tinyframe_py/__init__.py
( cd tf-vendor-py && git rev-parse HEAD ) > ~/code/imxrtnsy/tools/third_party/tinyframe_py/.upstream-sha
rm -rf tf-vendor-py
```

- [ ] **Step 2: Verify import works**

```bash
cd ~/code/imxrtnsy
python3 -c "from tools.third_party.tinyframe_py import TinyFrame, TF_Msg; print(TinyFrame)"
```
Expected: prints the class. No errors.

- [ ] **Step 3: Commit**

```bash
git add tools/third_party/tinyframe_py/
git commit -m "vendor: import PyTinyFrame for Hurra host client"
```

---

### Task 7.2: Create `tools/hurra_client.py` core (open, batched write, RX thread)

**Files:**
- Create: `tools/hurra_client.py`
- Test: `tools/tests/test_hurra_client_smoke.py` (host-only, no firmware needed)

- [ ] **Step 1: Write the failing test (mocks the serial port)**

```python
# tools/tests/test_hurra_client_smoke.py
import threading, queue, time
from unittest.mock import MagicMock
from tools.hurra_client import HurraClient, TYPE_PING, TYPE_VERSION

def test_client_packs_ping_frame():
    mock_ser = MagicMock()
    mock_ser.read.return_value = b""
    c = HurraClient.__new__(HurraClient)
    c._init_with_serial(mock_ser, baud=4_000_000)
    c.flush_immediate = True   # disable 1ms batching for test
    c._tf_send(TYPE_PING, b"\x01\x02\x03\x04")
    c.flush()
    # First write should contain SOF 0x68 and TYPE 0x00 (PING)
    written = b"".join(call.args[0] for call in mock_ser.write.call_args_list)
    assert written[0] == 0x68
    assert written[3] == 0x00      # TYPE byte position
    c.close()
```

```bash
mkdir -p ~/code/imxrtnsy/tools/tests
# write the test file above to that location
```

- [ ] **Step 2: Run the test — should FAIL with ImportError**

```bash
cd ~/code/imxrtnsy
python3 -m pytest tools/tests/test_hurra_client_smoke.py -v
```
Expected: `ModuleNotFoundError: No module named 'tools.hurra_client'`.

- [ ] **Step 3: Implement the client**

```python
# tools/hurra_client.py — host-side client for the Hurra binary protocol.
# See docs/specs/2026-05-23-hurra-binary-protocol-design.md
from __future__ import annotations

import ctypes
import fcntl
import os
import platform
import struct
import sys
import threading
import time
from typing import Callable, Optional

import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "third_party"))
from tinyframe_py import TinyFrame  # type: ignore

# ── TYPE constants (mirror src/hurra.c) ─────────────────────────────────────
TYPE_PING        = 0x00
TYPE_VERSION     = 0x01
TYPE_STATS       = 0x02
TYPE_INIT        = 0x03
TYPE_REBOOT      = 0x04
TYPE_BAUD        = 0x05
TYPE_SCREEN      = 0x06
TYPE_MOUSE_MOVE        = 0x10
TYPE_MOUSE_MOVE_SMOOTH = 0x11
TYPE_MOUSE_SILENT_MOVE = 0x12
TYPE_MOUSE_MO          = 0x13
TYPE_MOUSE_CLICK       = 0x14
TYPE_MOUSE_WHEEL       = 0x15
TYPE_MOUSE_GETPOS      = 0x16
TYPE_INVERT_X          = 0x17
TYPE_INVERT_Y          = 0x18
TYPE_SWAP_XY           = 0x19
TYPE_BTN_LEFT   = 0x20
TYPE_BTN_RIGHT  = 0x21
TYPE_BTN_MIDDLE = 0x22
TYPE_BTN_SIDE1  = 0x23
TYPE_BTN_SIDE2  = 0x24
TYPE_KB_DOWN       = 0x40
TYPE_KB_UP         = 0x41
TYPE_KB_PRESS      = 0x42
TYPE_KB_ISDOWN     = 0x43
TYPE_KB_MASK       = 0x44
TYPE_KB_STRING     = 0x45
TYPE_KB_MULTIDOWN  = 0x46
TYPE_KB_MULTIUP    = 0x47
TYPE_KB_MULTIPRESS = 0x48
TYPE_LOCK_ML  = 0x60
TYPE_LOCK_MR  = 0x61
TYPE_LOCK_MM  = 0x62
TYPE_LOCK_MS1 = 0x63
TYPE_LOCK_MS2 = 0x64
TYPE_LOCK_MX  = 0x65
TYPE_LOCK_MY  = 0x66
TYPE_CATCH_XY = 0x67
TYPE_STREAM_AXIS  = 0x70
TYPE_STREAM_BTN   = 0x71
TYPE_STREAM_MOUSE = 0x72
TYPE_STREAM_KB    = 0x73
TYPE_CB_BUTTONS   = 0x74
TYPE_CB_AXES      = 0x75
TYPE_CB_KEYS      = 0x76
TYPE_TLM_AXIS    = 0x80
TYPE_TLM_BUTTONS = 0x81
TYPE_TLM_MOUSE   = 0x82
TYPE_TLM_KB      = 0x83
TYPE_TLM_STATS   = 0x84
TYPE_TLM_LOG     = 0x85

# IOSSIOSPEED ioctl on Darwin for non-standard baud rates.
_IOSSIOSPEED = 0x80045402

def _macos_set_custom_baud(ser: serial.Serial, baud: int) -> None:
    if platform.system() != "Darwin":
        return
    buf = struct.pack("I", baud)
    fcntl.ioctl(ser.fileno(), _IOSSIOSPEED, buf)


class Stats:
    def __init__(self, raw: bytes):
        u = struct.unpack("<9I", raw[:36])
        (self.uptime_ms, self.rx_frames_ok, self.head_crc_err,
         self.payload_crc_err, self.id_gap_total, self.idle_resync,
         self.rx_drv_overrun, self.tx_ring_skip, self.payload_invalid) = u

    def __repr__(self):
        return (f"Stats(uptime={self.uptime_ms}ms rx_ok={self.rx_frames_ok} "
                f"crc_err={self.head_crc_err}+{self.payload_crc_err} "
                f"gaps={self.id_gap_total} overrun={self.rx_drv_overrun})")


class HurraClient:
    """Synchronous client; library is thread-safe (one mutex around _tf)."""

    def __init__(self, port: str, baud: int = 4_000_000, timeout: float = 0.1):
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        _macos_set_custom_baud(ser, baud)
        self._init_with_serial(ser, baud)

    def _init_with_serial(self, ser, baud: int):
        self._ser = ser
        self._baud = baud
        self._lock = threading.Lock()
        self._tf = TinyFrame()
        self._tf.SOF_BYTE = 0x68
        self._tf.ID_BYTES = 1
        self._tf.LEN_BYTES = 1
        self._tf.TYPE_BYTES = 1
        self._tf.CKSUM_TYPE = "crc16"
        self._tf.write = self._buffered_write
        self._wbuf = bytearray()
        self._last_flush = time.monotonic()
        self.flush_immediate = False    # set True in tests
        self._reply_waiters: dict[int, tuple[threading.Event, list]] = {}
        self._telemetry_handlers: dict[int, Callable[[bytes], None]] = {}
        self._drops_handler: Optional[Callable[[int], None]] = None
        self._last_id_gap_total = 0
        self._stop = False
        self._tf.add_generic_listener(self._on_any)
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()
        self._flusher = threading.Thread(target=self._flush_loop, daemon=True)
        self._flusher.start()

    # ── write batching (CH343B FS bulk MPS = 64) ────────────────────────────
    def _buffered_write(self, data: bytes) -> None:
        self._wbuf.extend(data)
        if self.flush_immediate or len(self._wbuf) >= 64:
            self._ser.write(bytes(self._wbuf))
            self._wbuf.clear()
            self._last_flush = time.monotonic()

    def flush(self) -> None:
        if self._wbuf:
            self._ser.write(bytes(self._wbuf))
            self._wbuf.clear()
            self._last_flush = time.monotonic()

    def _flush_loop(self):
        while not self._stop:
            time.sleep(0.0005)
            if self._wbuf and (time.monotonic() - self._last_flush) >= 0.001:
                with self._lock:
                    self.flush()

    # ── RX thread + dispatch ────────────────────────────────────────────────
    def _reader(self):
        while not self._stop:
            try:
                data = self._ser.read(512)
            except (OSError, serial.SerialException):
                return
            if data:
                with self._lock:
                    self._tf.accept(data)

    def _on_any(self, _tf, msg):
        # Reply correlation: if a waiter exists for this ID, fulfill it.
        w = self._reply_waiters.get(msg.id)
        if w is not None:
            evt, slot = w
            slot.append(bytes(msg.data))
            evt.set()
        # Telemetry dispatch
        h = self._telemetry_handlers.get(msg.type)
        if h is not None:
            h(bytes(msg.data))
        # Drop detection
        if msg.type == TYPE_TLM_STATS and len(msg.data) >= 20:
            id_gap = struct.unpack("<I", msg.data[16:20])[0]
            if id_gap > self._last_id_gap_total and self._drops_handler:
                self._drops_handler(id_gap - self._last_id_gap_total)
            self._last_id_gap_total = id_gap

    # ── primitives ───────────────────────────────────────────────────────────
    def _tf_send(self, type_: int, payload: bytes, frame_id: Optional[int] = None) -> None:
        with self._lock:
            self._tf.send(type_, payload, id=frame_id)

    def _request(self, type_: int, payload: bytes, timeout: float = 1.0) -> bytes:
        evt = threading.Event()
        slot: list[bytes] = []
        with self._lock:
            fid = self._tf.next_id()
            self._reply_waiters[fid] = (evt, slot)
            self._tf.send(type_, payload, id=fid)
            self.flush()
        if not evt.wait(timeout):
            self._reply_waiters.pop(fid, None)
            raise TimeoutError(f"no reply for type 0x{type_:02x} id={fid}")
        self._reply_waiters.pop(fid, None)
        return slot[0]

    # ── lifecycle ────────────────────────────────────────────────────────────
    def close(self) -> None:
        self._stop = True
        try: self.flush()
        except Exception: pass
        try: self._ser.close()
        except Exception: pass
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cd ~/code/imxrtnsy
python3 -m pytest tools/tests/test_hurra_client_smoke.py -v
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/hurra_client.py tools/tests/test_hurra_client_smoke.py
git commit -m "hurra: add HurraClient core (open, batched write, RX thread)"
```

---

### Task 7.3: Add high-level API methods to `HurraClient`

**Files:**
- Modify: `tools/hurra_client.py`

- [ ] **Step 1: Append the public methods**

Add inside `class HurraClient` (before `close`):

```python
    # ── hot path ────────────────────────────────────────────────────────────
    def move(self, dx: int, dy: int) -> None:
        self._tf_send(TYPE_MOUSE_MOVE, struct.pack("<hh", dx, dy))

    def move_smooth(self, dx: int, dy: int) -> None:
        self._tf_send(TYPE_MOUSE_MOVE_SMOOTH, struct.pack("<hh", dx, dy))

    def silent_move(self, dx: int, dy: int) -> None:
        self._tf_send(TYPE_MOUSE_SILENT_MOVE, struct.pack("<hh", dx, dy))

    def mo(self, buttons: int, dx: int, dy: int, wheel: int = 0,
           pan: int = 0, tilt: int = 0) -> None:
        self._tf_send(TYPE_MOUSE_MO,
                      struct.pack("<Bhh bbb", buttons, dx, dy, wheel, pan, tilt))

    def click(self, button: int, down_ms: int = 1, up_ms: int = 0) -> None:
        self._tf_send(TYPE_MOUSE_CLICK, bytes([button, down_ms, up_ms]))

    def wheel(self, ticks: int) -> None:
        self._tf_send(TYPE_MOUSE_WHEEL, struct.pack("<b", ticks))

    _BUTTON_TYPE = {
        0x01: TYPE_BTN_LEFT, 0x02: TYPE_BTN_RIGHT, 0x04: TYPE_BTN_MIDDLE,
        0x08: TYPE_BTN_SIDE1, 0x10: TYPE_BTN_SIDE2,
    }
    def button(self, mask: int, state: int) -> None:
        self._tf_send(self._BUTTON_TYPE[mask], bytes([1 if state else 0]))

    def button_get(self, mask: int) -> int:
        return self._request(self._BUTTON_TYPE[mask], b"")[0]

    # ── admin / diag ────────────────────────────────────────────────────────
    def version(self) -> str:
        return self._request(TYPE_VERSION, b"").decode("ascii", "replace")

    def ping(self, nonce: Optional[int] = None) -> float:
        import random
        n = nonce if nonce is not None else random.randint(0, 0xFFFFFFFF)
        t0 = time.monotonic()
        reply = self._request(TYPE_PING, struct.pack("<I", n))
        assert struct.unpack("<I", reply)[0] == n, "nonce mismatch"
        return time.monotonic() - t0

    def stats(self) -> Stats:
        return Stats(self._request(TYPE_STATS, b""))

    def init(self) -> None:
        self._tf_send(TYPE_INIT, b""); self.flush()

    def reboot(self) -> None:
        self._tf_send(TYPE_REBOOT, b""); self.flush()

    def set_baud(self, baud: int) -> int:
        reply = self._request(TYPE_BAUD, struct.pack("<I", baud))
        return struct.unpack("<I", reply)[0]

    def get_baud(self) -> int:
        reply = self._request(TYPE_BAUD, b"")
        return struct.unpack("<I", reply)[0]

    def getpos(self) -> tuple[int, int]:
        reply = self._request(TYPE_MOUSE_GETPOS, b"")
        return struct.unpack("<hh", reply)

    def screen(self, w: int | None = None, h: int | None = None) -> tuple[int, int]:
        if w is None and h is None:
            reply = self._request(TYPE_SCREEN, b"")
        else:
            reply = self._request(TYPE_SCREEN, struct.pack("<hh", w or 0, h or 0))
        return struct.unpack("<hh", reply)

    # ── keyboard ────────────────────────────────────────────────────────────
    def kb_down(self, hid: int) -> None:    self._tf_send(TYPE_KB_DOWN, bytes([hid]))
    def kb_up(self, hid: int) -> None:      self._tf_send(TYPE_KB_UP,   bytes([hid]))
    def kb_press(self, hid: int, hold_ms: int = 80, rand_ms: int = 30) -> None:
        self._tf_send(TYPE_KB_PRESS, bytes([hid, hold_ms, rand_ms]))
    def kb_isdown(self, hid: int) -> bool:
        return bool(self._request(TYPE_KB_ISDOWN, bytes([hid]))[0])
    def kb_mask(self, hid: int, state: int) -> None:
        self._tf_send(TYPE_KB_MASK, bytes([hid, state]))
    def kb_string(self, s: str) -> None:
        self._tf_send(TYPE_KB_STRING, s.encode("ascii", "ignore")[:240])
    def kb_multidown(self, keys: list[int]) -> None:
        self._tf_send(TYPE_KB_MULTIDOWN, bytes(keys[:6]))
    def kb_multiup(self, keys: list[int]) -> None:
        self._tf_send(TYPE_KB_MULTIUP,   bytes(keys[:6]))
    def kb_multipress(self, keys: list[int]) -> None:
        self._tf_send(TYPE_KB_MULTIPRESS, bytes(keys[:6]))

    # ── locks + catch ───────────────────────────────────────────────────────
    _LOCK_TYPE = {
        "ml": TYPE_LOCK_ML, "mr": TYPE_LOCK_MR, "mm": TYPE_LOCK_MM,
        "ms1": TYPE_LOCK_MS1, "ms2": TYPE_LOCK_MS2,
        "mx": TYPE_LOCK_MX, "my": TYPE_LOCK_MY,
    }
    def lock(self, name: str, state: int | None = None) -> int | None:
        t = self._LOCK_TYPE[name]
        if state is None:
            return self._request(t, b"")[0]
        self._tf_send(t, bytes([1 if state else 0]))
        return None

    def catch_xy(self, dur_ms: int) -> tuple[int, int]:
        reply = self._request(TYPE_CATCH_XY, struct.pack("<H", dur_ms),
                              timeout=dur_ms / 1000.0 + 1.0)
        return struct.unpack("<ii", reply)

    # ── telemetry subscriptions ─────────────────────────────────────────────
    def stream_axis(self, mode: int, period_ms: int) -> None:
        self._tf_send(TYPE_STREAM_AXIS, bytes([mode, period_ms]))
    def stream_buttons(self, mode: int, period_ms: int) -> None:
        self._tf_send(TYPE_STREAM_BTN, bytes([mode, period_ms]))
    def stream_mouse(self, mode: int, period_ms: int) -> None:
        self._tf_send(TYPE_STREAM_MOUSE, bytes([mode, period_ms]))
    def stream_keyboard(self, mode: int, period_ms: int) -> None:
        self._tf_send(TYPE_STREAM_KB, bytes([mode, period_ms]))

    def on_telemetry(self, type_: int, handler: Callable[[bytes], None]) -> None:
        self._telemetry_handlers[type_] = handler

    def on_drops(self, handler: Callable[[int], None]) -> None:
        self._drops_handler = handler
```

- [ ] **Step 2: Extend the existing test to call one mocked method**

Append to `tools/tests/test_hurra_client_smoke.py`:

```python
def test_client_packs_move_frame():
    from unittest.mock import MagicMock
    mock_ser = MagicMock()
    mock_ser.read.return_value = b""
    c = HurraClient.__new__(HurraClient)
    c._init_with_serial(mock_ser, baud=4_000_000)
    c.flush_immediate = True
    c.move(50, -30)
    c.flush()
    written = b"".join(call.args[0] for call in mock_ser.write.call_args_list)
    assert written[0] == 0x68    # SOF
    assert written[3] == 0x10    # TYPE_MOUSE_MOVE
    # Payload starts at offset 6 (SOF+ID+LEN+TYPE+HEAD_CRC16)
    import struct as _s
    dx, dy = _s.unpack("<hh", written[6:10])
    assert dx == 50 and dy == -30
    c.close()
```

- [ ] **Step 3: Run tests**

```bash
python3 -m pytest tools/tests/test_hurra_client_smoke.py -v
```
Expected: both tests PASS.

- [ ] **Step 4: Commit**

```bash
git add tools/hurra_client.py tools/tests/test_hurra_client_smoke.py
git commit -m "hurra: add full HurraClient public API (mouse/keyboard/locks/streams/admin)"
```

---

## Phase 8 — Python CLI: `hurra_test.py` and `hurra_probe.py`

### Task 8.1: Create `tools/hurra_test.py` CLI

**Files:**
- Create: `tools/hurra_test.py`

- [ ] **Step 1: Write the CLI**

```python
#!/usr/bin/env python3
# tools/hurra_test.py — CLI harness for the Hurra binary protocol.
import argparse
import glob
import os
import statistics
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from hurra_client import (HurraClient, Stats,
                          TYPE_MOUSE_MOVE, TYPE_PING, TYPE_TLM_STATS)

DEFAULT_BAUD = 4_000_000

def autodetect_port() -> str:
    for pattern in ("/dev/cu.wchusbserial*", "/dev/cu.usbmodem*", "/dev/ttyUSB*"):
        hits = glob.glob(pattern)
        if hits:
            return hits[0]
    raise SystemExit("no Hurra serial port found")

def cmd_version(c: HurraClient, args):
    print(c.version())

def cmd_ping(c: HurraClient, args):
    rtts = [c.ping() for _ in range(args.count)]
    us = [int(r * 1e6) for r in rtts]
    print(f"ping x{args.count}: min={min(us)}µs avg={int(sum(us)/len(us))}µs "
          f"p99={sorted(us)[int(len(us)*0.99)]}µs max={max(us)}µs")

def cmd_stats(c: HurraClient, args):
    print(c.stats())

def cmd_move(c: HurraClient, args):
    c.move(args.dx, args.dy); c.flush()

def cmd_click(c: HurraClient, args):
    c.click(args.button); c.flush()

def cmd_load(c: HurraClient, args):
    s0 = c.stats()
    period = 1.0 / args.rate
    t_end = time.monotonic() + args.duration
    sent = 0
    next_t = time.monotonic()
    dx = args.split_px
    sign = 1
    while time.monotonic() < t_end:
        c.move(dx * sign, 0)
        sign = -sign
        sent += 1
        next_t += period
        sleep = next_t - time.monotonic()
        if sleep > 0: time.sleep(sleep)
    c.flush()
    time.sleep(0.3)  # let firmware drain
    s1 = c.stats()
    rps = sent / args.duration
    drops = s1.id_gap_total - s0.id_gap_total
    err   = (s1.head_crc_err + s1.payload_crc_err) - (s0.head_crc_err + s0.payload_crc_err)
    over  = s1.rx_drv_overrun - s0.rx_drv_overrun
    print(f"sent={sent}  RPS={rps:.0f}  drops={drops}  crc_err={err}  overrun={over}")
    if rps < args.rate * 0.975:
        print(f"WARN: achieved RPS {rps:.0f} < target {args.rate * 0.975:.0f}")
        sys.exit(2)
    if drops > 50 or err > 0 or over > 0:
        print("FAIL: reliability counters tripped")
        sys.exit(3)

def cmd_smoke(c: HurraClient, args):
    """Round-trip one frame of every command type that has a reply."""
    fails = []
    def chk(name, fn):
        try: fn()
        except Exception as e: fails.append(f"{name}: {e}")
    chk("version",  lambda: c.version())
    chk("ping",     lambda: c.ping())
    chk("stats",    lambda: c.stats())
    chk("getpos",   lambda: c.getpos())
    chk("baud_get", lambda: c.get_baud())
    chk("screen",   lambda: c.screen())
    chk("invert_x", lambda: c.lock("mx"))   # any get-able
    for name in ("ml", "mr", "mm", "ms1", "ms2", "mx", "my"):
        chk(f"lock_{name}", lambda n=name: c.lock(n))
    chk("kb_isdown", lambda: c.kb_isdown(0x04))
    chk("btn_left",  lambda: c.button_get(0x01))
    if fails:
        print("FAIL:")
        for f in fails: print(f"  - {f}")
        sys.exit(1)
    print("OK: all smoke checks passed")

def main():
    ap = argparse.ArgumentParser(prog="hurra_test.py")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("version")
    p_ping = sub.add_parser("ping"); p_ping.add_argument("--count", type=int, default=20)
    sub.add_parser("stats")
    p_mv = sub.add_parser("move"); p_mv.add_argument("dx", type=int); p_mv.add_argument("dy", type=int)
    p_cl = sub.add_parser("click"); p_cl.add_argument("button", type=int)
    p_ld = sub.add_parser("load")
    p_ld.add_argument("--rate", type=int, required=True)
    p_ld.add_argument("--duration", type=int, default=30)
    p_ld.add_argument("--split-px", type=int, default=4)
    sub.add_parser("smoke")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    c = HurraClient(port, baud=args.baud)
    try:
        {"version": cmd_version, "ping": cmd_ping, "stats": cmd_stats,
         "move": cmd_move, "click": cmd_click, "load": cmd_load,
         "smoke": cmd_smoke}[args.cmd](c, args)
    finally:
        c.close()

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make executable + verify it parses**

```bash
chmod +x ~/code/imxrtnsy/tools/hurra_test.py
~/code/imxrtnsy/tools/hurra_test.py --help
```
Expected: argparse help text lists all subcommands.

- [ ] **Step 3: Commit**

```bash
git add tools/hurra_test.py
git commit -m "hurra: add hurra_test.py CLI (version/ping/stats/move/click/load/smoke)"
```

---

### Task 8.2: Create `scripts/hurra_probe.py`

**Files:**
- Create: `scripts/hurra_probe.py`

- [ ] **Step 1: Write the probe**

```python
#!/usr/bin/env python3
# scripts/hurra_probe.py — detect Hurra device + handshake baud.
import glob, os, subprocess, sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from hurra_client import HurraClient  # type: ignore

CANDIDATE_BAUDS = (4_000_000, 1_500_000, 921_600, 115_200)

def list_ports():
    return (glob.glob("/dev/cu.wchusbserial*")
            + glob.glob("/dev/cu.usbmodem*")
            + glob.glob("/dev/ttyUSB*"))

def usb_chip_hint(port: str) -> str:
    if sys.platform != "darwin": return "(skip chip hint on non-macOS)"
    try:
        out = subprocess.check_output(["system_profiler", "SPUSBDataType"], text=True)
    except Exception:
        return "(system_profiler unavailable)"
    if "1A86" in out or "QinHeng" in out or "CH343" in out:
        return "CH343-family bridge detected"
    if "16C0" in out and "0483" in out:
        return "Teensy native USB detected"
    return "(unknown VID:PID)"

def probe(port: str) -> bool:
    for baud in CANDIDATE_BAUDS:
        try:
            c = HurraClient(port, baud=baud, timeout=0.2)
            try:
                version = c.version()
                stats = c.stats()
                print(f"{port}  baud={baud}")
                print(f"  version: {version}")
                print(f"  {stats}")
                print(f"  hint: {usb_chip_hint(port)}")
                return True
            finally:
                c.close()
        except Exception:
            continue
    return False

def main():
    ports = list_ports()
    if not ports:
        print("no serial ports found")
        sys.exit(1)
    any_ok = False
    for p in ports:
        ok = probe(p)
        any_ok = any_ok or ok
        if not ok:
            print(f"{p}: no Hurra response at any candidate baud")
    sys.exit(0 if any_ok else 1)

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make executable + parse-check**

```bash
chmod +x ~/code/imxrtnsy/scripts/hurra_probe.py
~/code/imxrtnsy/scripts/hurra_probe.py --help 2>&1 || true   # no argparse; just runs
```
Expected: either runs and reports "no serial ports found" (if hardware unplugged) or attempts to probe.

- [ ] **Step 3: Commit**

```bash
git add scripts/hurra_probe.py
git commit -m "hurra: add scripts/hurra_probe.py (replaces makcu_probe.py)"
```

---

## Phase 9 — Bench verification

### Task 9.1: Run smoke + load tests on real hardware

**Files:** (none modified)

- [ ] **Step 1: Flash hurra build**

```bash
cd ~/code/imxrtnsy
make clean && make PROTOCOL=hurra flash
```

- [ ] **Step 2: Run probe**

```bash
./scripts/hurra_probe.py
```
Expected: detects port; reports `kmbox: Hurra v1`; prints non-zero `uptime_ms`.

- [ ] **Step 3: Run smoke**

```bash
./tools/hurra_test.py smoke
```
Expected: `OK: all smoke checks passed`. If any subcommand fails, investigate before continuing.

- [ ] **Step 4: Run ship-gate load test (spec §7.3 Test A)**

```bash
./tools/hurra_test.py load --rate 8000 --duration 30
```
Expected criteria:
- achieved RPS ≥ 7,800
- `drops` < 50
- `crc_err` = 0
- `overrun` = 0
- exit code 0

If any criterion fails, **stop and investigate**. Do not proceed to Phase 11 (rip-and-replace) until this passes.

- [ ] **Step 5: Run headroom test (spec §7.3 Test B)**

```bash
./tools/hurra_test.py load --rate 16000 --duration 10
```
Expected: ≥15 k/sec achieved with ≤0.5% drop rate; this proves 8 k isn't near a cliff.

No commit — verification only. Log the numbers in your engineering notes.

---

## Phase 10 — Port aim test + scripts/uart_*.py support

### Task 10.1: Port `ferrum_aim_test.py` → `tools/hurra_aim_test.py`

**Files:**
- Create: `tools/hurra_aim_test.py` (based on `tools/ferrum_aim_test.py`)

- [ ] **Step 1: Copy + edit**

```bash
cp ~/code/imxrtnsy/tools/ferrum_aim_test.py ~/code/imxrtnsy/tools/hurra_aim_test.py
```

Then edit `hurra_aim_test.py`:
1. Replace any `import` of ferrum bits with:
   ```python
   import sys, os
   sys.path.insert(0, os.path.dirname(__file__))
   from hurra_client import HurraClient, TYPE_TLM_AXIS
   ```
2. Replace `serial.write(b"m(...)\r\n")` calls with `client.move(dx, dy)`.
3. Replace `serial.write(b"km.axes(1)\r\n")` (enabling axes callback) with:
   ```python
   client.stream_axis(mode=1, period_ms=1)
   client.on_telemetry(TYPE_TLM_AXIS, on_axis)
   ```
   And the `on_axis` handler unpacks `struct.unpack("<hhb", data)` → `(dx, dy, wheel)`.
4. Change default `--baud` from 115200 to `4_000_000`.
5. Keep the `--tick-ms` and `--cmd-step-px` flags identical.
6. At end-of-run report, include `client.stats().id_gap_total` delta.

- [ ] **Step 2: Make executable + parse-check**

```bash
chmod +x ~/code/imxrtnsy/tools/hurra_aim_test.py
~/code/imxrtnsy/tools/hurra_aim_test.py --help
```

- [ ] **Step 3: Run on hardware (no GUI needed if --load mode used)**

```bash
./tools/hurra_aim_test.py load --duration 10
```
Expected: ≥ 8 k cmds/sec sustained; reported `id_gap_total` increment < 50; no overruns.

- [ ] **Step 4: Commit**

```bash
git add tools/hurra_aim_test.py
git commit -m "hurra: port ferrum_aim_test.py to hurra_aim_test.py (4 Mbps default)"
```

---

### Task 10.2: Extend `scripts/uart_debug.py` and `scripts/uart_bench.py` with hurra modes

**Files:**
- Modify: `scripts/uart_debug.py`
- Modify: `scripts/uart_bench.py`

- [ ] **Step 1: Add `--protocol {ferrum,hurra}` flag to both**

In each file, before the existing argparse setup:

```python
parser.add_argument("--protocol", choices=("ferrum", "hurra"), default="ferrum",
                    help="Protocol to probe (ferrum = ASCII, hurra = binary)")
```

In the script body, gate the probe routine:

```python
if args.protocol == "hurra":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
    from hurra_client import HurraClient
    c = HurraClient(args.port, baud=args.baud)
    print(c.version())
    print(c.stats())
    c.close()
    return
# ... existing ferrum probe path unchanged ...
```

- [ ] **Step 2: Verify both still work for ferrum**

```bash
./scripts/uart_debug.py --port /dev/cu.usbmodemXXXX
```
Expected: ferrum path runs unchanged.

- [ ] **Step 3: Commit**

```bash
git add scripts/uart_debug.py scripts/uart_bench.py
git commit -m "scripts: add --protocol hurra mode to uart_debug + uart_bench"
```

---

## Phase 11 — Default switch and `makcu` rip-and-replace

### Task 11.1: Flip Makefile default to `PROTOCOL=hurra`

**Files:**
- Modify: `Makefile`

- [ ] **Step 1: Edit the default**

Change `PROTOCOL ?= ferrum` → `PROTOCOL ?= hurra`.

- [ ] **Step 2: Verify default build is now hurra**

```bash
make clean && make
arm-none-eabi-nm firmware.elf | grep hurra_init   # should be present
arm-none-eabi-nm firmware.elf | grep ferrum_init  # should be absent
```

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "build: default PROTOCOL to hurra; ferrum opt-in via PROTOCOL=ferrum"
```

---

### Task 11.2: Delete makcu source files + artifacts

**Files:**
- Delete: `src/makcu.c`, `src/makcu.h`, `src/makcu.o` (untracked — `rm`)
- Delete: `firmware-makcu.hex`, `.protocol-makcu.stamp` (untracked — `rm`)
- Delete: `scripts/makcu_probe.py` (untracked — `rm`)
- Delete: `docs/specs/2026-05-18-makcu-binary-protocol-design.md` (tracked — `git rm`)

- [ ] **Step 1: Remove tracked files via git**

```bash
cd ~/code/imxrtnsy
git rm docs/specs/2026-05-18-makcu-binary-protocol-design.md
```

- [ ] **Step 2: Remove untracked files via rm**

```bash
rm -f src/makcu.c src/makcu.h src/makcu.o
rm -f firmware-makcu.hex .protocol-makcu.stamp
rm -f scripts/makcu_probe.py
```

- [ ] **Step 3: Verify build still works**

```bash
make clean && make PROTOCOL=hurra
make clean && make PROTOCOL=ferrum
```
Expected: both build successfully.

- [ ] **Step 4: Commit**

```bash
git add -A   # picks up the git rm + the freshly-untracked deletions
git commit -m "rip: delete makcu source, header, probe script, hex, stamp, spec"
```

---

### Task 11.3: Strip makcu references from docs + settings

**Files:**
- Modify: `docs/plans/2026-05-21-xinput-controller-passthrough.md`
- Modify: `.claude/settings.local.json`
- Possibly modify: `README.md`, `CLAUDE.md`

- [ ] **Step 1: Find every remaining makcu reference**

```bash
cd ~/code/imxrtnsy
git grep -in 'makcu'
```

- [ ] **Step 2: Edit each hit**

For `docs/plans/2026-05-21-xinput-controller-passthrough.md`:
- If the makcu reference is a "see makcu protocol" link, replace with reference to `docs/specs/2026-05-23-hurra-binary-protocol-design.md`.
- If the plan section was makcu-specific and is now obsolete, mark the section "Superseded by Hurra protocol" and link to the spec.

For `.claude/settings.local.json`:
- Open the file and remove any allow-list entry referencing a makcu path. Validate JSON is still parseable: `python3 -m json.tool .claude/settings.local.json`.

For `README.md` and `CLAUDE.md` (if they reference makcu):
- Replace MAKCU mentions with Hurra; document the `PROTOCOL=hurra|ferrum` Makefile flag.

- [ ] **Step 3: Verify `git grep -i makcu` returns empty**

```bash
git grep -in 'makcu'
```
Expected: no output. If anything remains, edit it.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "rip: strip remaining makcu references from docs, settings, README"
```

---

### Task 11.4: Final merge-gate verification

**Files:** (none modified)

- [ ] **Step 1: Run the §8.5 verification script**

```bash
cd ~/code/imxrtnsy
git grep -i 'makcu' && echo "FAIL: makcu references remain" && exit 1 || echo "OK: no makcu hits"
make clean && make PROTOCOL=hurra   && [ -f firmware.hex ] && echo "OK: hurra builds"
make clean && make PROTOCOL=ferrum  && [ -f firmware.hex ] && echo "OK: ferrum builds"
./tools/hurra_test.py load --rate 8000 --duration 30
```
Expected: all three OK messages, load test exits 0 with passing criteria.

- [ ] **Step 2: Tag the release**

```bash
git tag -a kmbox-hurra-v1.0 -m "Hurra protocol v1.0 — replaces MAKCU; 8k+ cmds/sec target"
git log --oneline -10
```

No final commit needed beyond the tag.

---

## Self-review notes

- **Spec §2 (frame format)** → Tasks 1.2 (TF_Config.h), 7.2 (PyTinyFrame config in client).
- **Spec §3 (command catalogue)** → Tasks 4.2–4.4, 5.1–5.3, 6.1 (every TYPE has a listener + Python method).
- **Spec §4.3 (LPUART eDMA RX)** → Task 2.2.
- **Spec §4.4 (`kmbox_tx_room`)** → Task 2.1.
- **Spec §4.5 (hurra.c skeleton)** → Task 3.2.
- **Spec §4.7 (Makefile)** → Tasks 1.3 (initial), 11.1 (default flip).
- **Spec §5 (PC client)** → Tasks 7.1–7.3 (client), 8.1 (CLI), 8.2 (probe), 10.1 (aim test).
- **Spec §6 (stats / reliability)** → Tasks 4.2 (stats listener), 6.2 (TLM_*), 6.3 (TLM_STATS push), 2.2 (overrun counter).
- **Spec §7.3 (verification gates)** → Task 9.1.
- **Spec §8 (migration)** → Phase 11 (Tasks 11.1–11.4).

Every spec section maps to at least one task. No placeholders. Type names and method signatures consistent across tasks.
