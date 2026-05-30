# Hurra TinyFrame Parser for Hurra-v2 — Design

**Date:** 2026-05-30
**Status:** Approved
**Author:** Claude (with @ramsey)

## Summary

Give the Hurra-v2 firmware a latency-optimized, **wire-compatible** Hurra binary
protocol parser (TinyFrame-based) so the `hurra-app` host bridge works against it.
Make **Hurra the default protocol** (`make`), with Ferrum ASCII available via
`make PROTOCOL=ferrum`. Also harden the LPUART clock setup, remove dead code, and
bring the docs current (including the CP2102C → WCH CH343 correction).

### Background

`hurra-app` speaks the **Hurra binary protocol** (TinyFrame: SOF `0x68`, 1-byte
ID/LEN/TYPE, CRC16). The shipped Hurra-v2 firmware speaks only **Ferrum ASCII**
and contains no TinyFrame parser, so the bridge's binary frames are never decoded
(symptom: `km.version()` probe times out, `rc=-1`, mouse does not move).

A sibling fork, **`imxrtnsy`**, already contains a complete, tested, app-wire-
compatible Hurra subsystem (verified: identical `TinyFrame.c`, identical
`TF_Config.h`, matching opcode map). Hurra-v2 appears to be `imxrtnsy` forked
earlier, stripped to Ferrum-only, then given newer CH343B/LPUART3 transport tuning.

## Strategy

**Port `imxrtnsy`'s proven Hurra subsystem and graft it onto Hurra-v2's newer
CH343B-tuned transport.** No clean-sheet parser (that would risk wire
incompatibility with the app). Keep Hurra-v2's superior transport (1024-byte eDMA
ring, IDLE-line wake, auto-baud-reset, LINK/STATE/STATUS LED scheme).

## Components & Changes

### 1. New files (copied from imxrtnsy, unchanged — preserves wire compatibility)
- `src/hurra.c`, `src/hurra.h` — TinyFrame type listeners (admin/mouse/keyboard/
  locks/streams/telemetry), deferred actions (baud/reboot/catch_xy), stats.
- `src/proto.h` — compile-time `proto_*` alias layer selecting Hurra or Ferrum.
- `src/TF_Config.h` — SOF `0x68`, ID/LEN/TYPE = 1 byte, CRC16. Byte-identical to
  `hurra-app/src/TF_Config.h`.
- `src/third_party/TinyFrame/TinyFrame.{c,h}` — identical to the app's vendored copy.

### 2. `actions.c` / `actions.h` — adopt imxrtnsy superset
Add `act_wheel`, `act_get/set_invert_x`, `act_get/set_invert_y`,
`act_get/set_swap_xy`, their backing statics, and the swap/invert application in
`act_move`. Hurra-v2's current `actions.c` is a strict subset; `ferrum.c` is
byte-identical across both trees and still compiles against the superset.

### 3. `kmbox.c` — surgical integration (keep Hurra-v2 transport)
- Switch the protocol calls from `ferrum_*` to `proto_*` (via `proto.h`):
  `proto_init/reset/tick/set_tx/notify_buttons/notify_axes/notify_keys`, and
  feeding through `proto_feed`.
- Add `kmbox_tx_room()` — free bytes in the 256-entry TX ring.
- Add `kmbox_rx_drv_overrun()` — port imxrtnsy's ¾-ring driver-overrun heuristic
  and counter. Both are consumed by `hurra.c`'s STATS frame.
- **Latency win A — batch feed:** add `proto_feed(buf, len)` to `proto.h`
  (Hurra → batch `TF_Accept`; Ferrum → per-byte loop). Drain the eDMA ring in
  ≤2 contiguous spans rather than per byte.
- **Latency win B — immediate reply flush:** replace the per-`\r\n` `tx_flush()`
  with a single `tx_flush()` immediately after the burst drain, so a reply or
  telemetry frame leaves in one DMA TX on the same poll it was produced. Protocol-
  agnostic; lower latency than waiting for the next `poll_fast` tick.

### 4. Hardening
- `kmbox_init`: explicitly pin the LPUART root clock to the 24 MHz oscillator —
  `CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL;`
  (PODF = 0, SEL = 1 → 24 MHz) so `compute_baud_reg`'s `UART_CLOCK = 24e6`
  assumption is guaranteed rather than inherited from the bootloader/core.
- SEVONPEND: already set at `main.c:95` (`SCB_SCR |= SCB_SCR_SEVONPEND;`). No change.

### 5. Dead-code cleanup (firmware-scoped, targeted)
- Remove vestigial `detected_proto` and `kmbox_protocol_mode()` (protocol auto-
  detect is moot under the compile-time switch).
- Remove the per-newline `frames_ok`/`STATE_LED` bookkeeping subsumed by the
  per-burst flush (retain a generic RX-activity LED signal).
- Delete the obsolete `docs/specs/2026-05-18-makcu-binary-protocol-design.md`
  (superseded — the project chose Hurra/TinyFrame, not MAKCU).

### 6. Makefile
Add imxrtnsy's `PROTOCOL` selector (`PROTOCOL ?= hurra`), `PROTO_DEF`/`PROTO_SRC`,
the `-Isrc/third_party/TinyFrame` include, and hot-path objects (`hurra.o`,
`TinyFrame.o` compiled at `-O2 -ffast-math`). Ferrum remains buildable via
`make PROTOCOL=ferrum`.

### 7. Docs
- `CLAUDE.md` and `README.md`: protocol section → **Hurra binary default / Ferrum
  opt-in**; correct the bridge chip **CP2102C → WCH CH343** and "up to 3 Mbaud" →
  "up to 6 Mbaud"; add `hurra.c`/`proto.h`/TinyFrame to the key-files list; update
  build commands; document the `0x68` TinyFrame wire format and the `hurra-app`
  pairing.

## Data Flow (Hurra build)

```
host → CH343 → LPUART3 RX → eDMA ring → kmbox_poll_heavy (drain burst)
     → proto_feed → TF_Accept → type listener → act_* → kmbox_inject_* → USB device

reply/telemetry: TF_Respond/TF_Send → TF_WriteImpl → s_tx (uart_tx_frame)
     → TX ring → tx_flush (single DMA TX after burst)
```

## Testing / Verification

- `make` (Hurra) and `make PROTOCOL=ferrum` both compile clean; capture `size` output.
- Gate: `TF_Config.h` and `TinyFrame.c` byte-identical to `hurra-app`'s copies.
- Bench (hardware): `hurra-bridge --device … --baud 4000000` + `ferrum_aim_test.py`
  handshake — `km.version()` → bridge `hurra_version` → firmware `l_version` reply
  (the previously failing path), then a move test.

## Risks & Mitigations

- **Adopting the `actions.c` superset could affect the Ferrum path** — mitigated by
  keeping `make PROTOCOL=ferrum` in the build/test matrix.
- **Per-burst flush changes Ferrum reply timing slightly** — acceptable; a burst is
  ≈ one line at these baud rates and `poll_fast` still backstops the flush.
- **Two near-duplicate firmware trees (Hurra-v2, imxrtnsy) remain** — out of scope
  here; noted for a future consolidation decision.

## Non-goals

- Consolidating Hurra-v2 and imxrtnsy into one tree.
- Changing the Hurra wire protocol or opcode map.
- Host-side (`hurra-app`) changes — the app is already correct; only the firmware
  side was missing.
