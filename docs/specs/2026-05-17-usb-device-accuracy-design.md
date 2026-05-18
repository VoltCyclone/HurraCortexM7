# USB Device Accuracy — Composite Enumeration & Vendor Report Pass-Through

**Date:** 2026-05-17
**Status:** Design — pending review
**Scope:** `src/usb_device.{c,h}`, `src/usb_host.{c,h}`, `src/desc_capture.{c,h}`, `src/main.c`

## Problem

The MITM device port (USB1) does not faithfully reproduce the upstream device on the host port (USB2). Two user-reported symptoms, both confirmed by code audit:

1. **Composite enumeration is incomplete.** Multi-interface devices (gaming kbd+mouse combos, multi-function dongles) lose interfaces and endpoints. The downstream PC sees a truncated device.
2. **Vendor and class control-pipe reports are not forwarded.** Class `GET_REPORT`, `GET_IDLE`, `GET_PROTOCOL`, `SET_PROTOCOL`, `SET_IDLE` are answered locally (zeros / stubs) instead of being relayed to the upstream device. Vendor SET_REPORT payloads over 64 bytes are silently truncated. Interrupt OUT endpoints are not supported at all.

## Root causes (with file:line)

| # | Issue | Location |
|---|---|---|
| C1 | Control OUT payloads > 64 B silently dropped (`deferred_out.data[64]`) | `usb_device.c:32`, `:401` |
| C2 | Unknown GET_DESCRIPTOR types STALL (BOS, Device Qualifier, Other Speed) | `usb_device.c:104-190` |
| C3 | Class GET_REPORT returns zeros | `usb_device.c:320-326` |
| C4 | Class GET_IDLE (0x02) missing — STALLs | `usb_device.c:311-341` |
| C5 | SET_IDLE / GET_PROTOCOL / SET_PROTOCOL answered locally, not forwarded | `usb_device.c:314-318, 331-335` |
| C6 | Interrupt OUT endpoints filtered out at capture (`& 0x80`) | `desc_capture.c:84` |
| C7 | Interrupt OUT endpoints unconfigured on device side | `usb_device.c:202-235` |
| C8 | `MAX_INT_EPS = 4` — endpoints 5+ NAK silently | `usb_device.h:53`, `usb_device.c:216` |
| C9 | `MAX_INTR_EPS = 4` — host-side IN polling cap | `usb_host.h:72` |
| C10 | `MAX_INTERFACES = 4` — interface 5+ never captured | `desc_capture.h:9` |
| C11 | HID report-descriptor capture gated on `iface_class == 3` | `desc_capture.c:67, 151` |
| C12 | LANGID is captured then discarded; reply hardcoded `0x0409` | `desc_capture.c:170-176`, `usb_device.c:124-127` |
| C13 | `iConfiguration` and per-interface `iInterface` strings not captured | `desc_capture.c:177-197` |
| C14 | BOS descriptor never captured | `desc_capture.c` (absent) |
| C15 | Microsoft OS 1.0 string `0xEE` never probed at capture time | `desc_capture.c` (absent) |

## Non-goals

- **Multiple configurations.** Single config (config 0) only. Multi-config devices are vanishingly rare for HID and out of scope.
- **High-bandwidth or isochronous endpoints.** HID is interrupt + control only.
- **Modifying or filtering the captured descriptors** (no VID/PID rewriting). Pure 1:1 replay.
- **Anti-cheat fingerprint cloning beyond descriptor accuracy.** Public evidence that major anti-cheats (EAC/BattlEye/Vanguard) parse USB descriptor fields on input devices is not corroborated; we are fixing for correctness, not for an unverified fingerprint surface.

## Design

Three phases. Each phase is independently shippable, testable, and adds no regression risk to earlier phases.

---

### Phase 1 — Correctness fixes (no new capability)

**Goal:** stop silently dropping data and stop answering control requests we don't actually know the answer to.

**Changes:**

1. **`deferred_out.data` 64 → 512.** Match `ep0_rx_buf` size. This eliminates the silent truncation for Razer-style vendor SET_REPORT and any other large control-OUT payload. Cost: 448 B in `.dmabuffers`.

2. **`handle_get_descriptor` default case → `handle_passthrough(setup)`.** Today the switch covers DEVICE / CONFIGURATION / STRING / HID / HID_REPORT and STALLs everything else. Change `default: break;` to `default: handle_passthrough(setup); return;`. Covers BOS, Device Qualifier, Other Speed Config, and anything future devices throw at us, free.

3. **Route class control requests through `handle_passthrough`:**
   - `0x01 GET_REPORT` (was: returns zeros)
   - `0x02 GET_IDLE` (was: STALL — case missing entirely)
   - `0x03 GET_PROTOCOL` (was: returns `0x01`)
   - `0x0A SET_IDLE` (was: ACK locally)
   - `0x0B SET_PROTOCOL` (was: ACK locally)
   - `0x09 SET_REPORT` already passes through, leave as-is.

   `handle_passthrough` already handles both IN (synchronous round-trip with 200ms timeout) and OUT-with-data (RX → ACK → defer forward) correctly. Risk: synchronous IN passthrough blocks the device-side EP0 for up to 200ms; on a healthy upstream this is sub-ms, but a misbehaving device could stall enumeration. Acceptable since `ep0_stall()` is the failure mode anyway.

**LoC:** ~30. **RAM:** +448 B. **Files touched:** `usb_device.c`, `usb_device.h`.

---

### Phase 2 — Descriptor completeness & composite caps

**Goal:** capture and replay every descriptor field the upstream device exposes; lift silent caps that drop interfaces/endpoints.

**Changes:**

1. **Lift silent caps.**
   - `MAX_INT_EPS` 4 → 7 (`USB_DEV_NUM_ENDPOINTS - 1`, the hardware ceiling).
   - `MAX_INTR_EPS` 4 → 7 (host side, symmetric).
   - `MAX_INTERFACES` 4 → 8.
   - `MAX_STRINGS` 8 → 16 (to fit device strings + iConfiguration + per-interface iInterface).

2. **HID report-descriptor capture for any HID descriptor.** Drop the `iface_class == 3` gate at `desc_capture.c:67` (parse-time) and `:151` (capture-time). Gate instead on "interface contained a HID descriptor" (track a per-iface bool `has_hid_desc` set when `dtype == USB_DESC_HID` is observed inside that interface).

3. **Capture & replay LANGID verbatim.** Add `uint8_t langid_desc[8]; uint8_t langid_desc_len;` to `captured_descriptors_t`. During capture, save the raw string-0 response. In `usb_device.c` handle_get_descriptor's STRING handler, if the requested string index is 0, return the captured raw descriptor instead of the hardcoded `{0x04, 0x03, 0x09, 0x04}`.

4. **Generic string-capture loop.** Replace the hand-rolled iManufacturer/iProduct/iSerial loop with: walk parsed interfaces and config descriptor to collect *all* referenced string indices (`device_desc[14..16]`, `config_desc[6]`, each parsed interface's `iInterface` byte). Dedup, fetch each. Store with their original USB index so lookups by index work.

5. **BOS descriptor capture.** New helper `capture_bos(desc)`: GET_DESCRIPTOR(BOS, 0, 0, 5) → read total length → GET_DESCRIPTOR(BOS, 0, 0, total). Store in `bos_desc[256]` + `bos_desc_len`. If the upstream device STALLs (USB 2.0 device that doesn't support BOS), `bos_desc_len = 0` — Phase 1's passthrough default will then forward future BOS requests anyway, but capture lets us answer without round-trip latency.

   On the device side, add `case USB_DESC_BOS (0x0F)` in handle_get_descriptor: if `bos_desc_len > 0`, return cached buffer; else fall through to passthrough.

6. **MS OS 1.0 capture.** New helper `capture_ms_os(desc)`: GET_DESCRIPTOR(STRING, 0xEE, 0, 0x12). Validate response starts with `0x12, 0x03` and contains UTF-16LE `"MSFT100"` at offset 2. If valid, store the full descriptor (so subsequent string-0xEE requests are answered locally and byte-identical) and stash the vendor-code byte at offset 0x10. The vendor-specific MS OS feature-descriptor requests (Compatible ID, Extended Properties) already go through the vendor-passthrough path in Phase 1 — no extra device-side code needed.

**LoC:** ~150. **RAM:** ~2.8 KB (3 extra `captured_iface_t` × 530 B + BOS buf 256 B + langid 8 B + 8 extra strings × 128 B ≈ 1 KB plus the bigger arrays in usb_device.c for the lifted EP caps ≈ 720 B). **Files touched:** all four.

---

### Phase 3 — Interrupt OUT endpoint pass-through

**Goal:** support host → upstream-device interrupt OUT traffic. Real-world impact: Logitech HID++ (DPI / profile / battery reports flow over interrupt OUT) and other 3-interface gaming devices with vendor interfaces using bidirectional interrupt pipes.

**Architectural shape:**

```
downstream PC ──interrupt OUT──▶ USB1 device EP                                 (existing direction reversed)
                                       │
                                       ▼
                              usb_device RX completion
                                       │
                                       ▼
                              main.c forward loop
                                       │
                                       ▼
                       usb_host_interrupt_out_send(slot)
                                       │
                                       ▼
                        USB2 host periodic OUT qTD
                                       │
                                       ▼
                              upstream device EP
```

**Changes:**

1. **`desc_capture.c`:** drop the `(ep_addr & 0x80)` direction filter at line 84. Track per-interface up to one IN and one OUT interrupt endpoint (`captured_iface_t.interrupt_in_ep`, `.interrupt_out_ep`, with maxpkt/interval for each). No known HID device exposes more than one OUT interrupt EP per interface, so 1-per-iface is sufficient.

2. **`usb_host.c`:** add `usb_host_interrupt_out_init(slot, addr, ep, maxpkt)` and `usb_host_interrupt_out_send(slot, data, len)`. Use a parallel QH/qTD pool. The OUT qTD is one-shot (not periodic) since we only fire on data arrival, not on a polling interval. (USB periodic-list scheduling: interrupt OUT QH still lives in the periodic frame list at the device's bInterval so the host transmits in the right uframes, but the qTD is re-armed on demand by us, not auto-rearmed.)

3. **`usb_device.c`:**
   - Add `dtd_int_rx[MAX_OUT_EPS]`, `int_rx_buf[MAX_OUT_EPS][2][64]` (double-banked, matches IN side).
   - `configure_all_interrupt_out_endpoints()`: walk `cap_desc->ifaces`, for each `interrupt_out_ep != 0`, configure OUT dQH on `endptctrl_reg(ep)` and prime first RX.
   - New `usb_device_poll_out()`: scan OUT EPs for completed RX (`dtd_int_rx[slot].token & DTD_ACTIVE` cleared), return the slot index, byte count, buffer pointer, and re-prime the other bank.

4. **`main.c`:** in the main poll loop, after the existing IN forward block, add a parallel OUT forward block:
   ```c
   for (uint8_t m = 0; m < num_out_mappings; m++) {
       int n = usb_device_poll_out(out_map[m].dev_ep, &buf, &len);
       if (n > 0) usb_host_interrupt_out_send(out_map[m].host_slot, buf, n);
   }
   ```

5. **Scope cap:** up to 7 interrupt OUT endpoints (HW ceiling, symmetric with IN). Real devices use 0-2; the cap is just to remove the silent-truncation footgun.

**LoC:** ~200. **RAM:** ~1 KB. **Files touched:** all four.

---

## Data structures (final)

```c
// desc_capture.h
#define MAX_INTERFACES          8
#define MAX_STRINGS             16
#define MAX_BOS_DESC_SIZE       256
#define MAX_LANGID_DESC_SIZE    8

typedef struct {
    uint8_t  iface_num;
    uint8_t  iface_class;
    uint8_t  iface_subclass;
    uint8_t  iface_protocol;
    uint8_t  iface_string_idx;     // NEW: iInterface

    uint8_t  interrupt_in_ep;      // RENAMED from interrupt_ep
    uint16_t interrupt_in_maxpkt;
    uint8_t  interrupt_in_interval;

    uint8_t  interrupt_out_ep;     // NEW: 0 if none
    uint16_t interrupt_out_maxpkt;
    uint8_t  interrupt_out_interval;

    bool     has_hid_desc;         // NEW: replaces iface_class==3 gate
    uint8_t  hid_report_desc[MAX_HID_REPORT_DESC_SIZE];
    uint16_t hid_report_desc_len;
} captured_iface_t;

typedef struct {
    uint8_t  device_desc[18];
    uint8_t  device_desc_len;
    uint8_t  config_desc[MAX_CONFIG_DESC_SIZE];
    uint16_t config_desc_len;
    uint8_t  config_string_idx;                  // NEW: iConfiguration

    captured_iface_t ifaces[MAX_INTERFACES];
    uint8_t  num_ifaces;

    uint8_t  string_desc[MAX_STRINGS][MAX_STRING_DESC_SIZE];
    uint8_t  string_desc_len[MAX_STRINGS];
    uint8_t  string_index[MAX_STRINGS];
    uint8_t  num_strings;

    uint8_t  langid_desc[MAX_LANGID_DESC_SIZE];  // NEW
    uint8_t  langid_desc_len;                    // NEW

    uint8_t  bos_desc[MAX_BOS_DESC_SIZE];        // NEW
    uint16_t bos_desc_len;                       // NEW (0 if device has no BOS)

    uint8_t  ms_os_desc[18];                     // NEW: full MS_OS 1.0 string at 0xEE
    uint8_t  ms_os_desc_len;                     // NEW (0 if device has no MS_OS)
    uint8_t  ms_os_vendor_code;                  // NEW: byte at offset 0x10

    uint8_t  ep0_maxpkt;
    uint8_t  dev_addr;
    bool     valid;
} captured_descriptors_t;
```

## Error handling

- **Upstream STALL on capture (BOS, MS_OS, etc.):** zero-length, not fatal. Phase 1's passthrough default still answers downstream requests live.
- **Upstream STALL on runtime passthrough:** propagate via `ep0_stall()` (already done in `handle_passthrough`). Downstream PC retries or gives up — same as if we weren't in the path.
- **Upstream control transfer timeout (200ms):** ditto, STALL downstream.
- **Capture-time iInterface fetch failure:** dedup-skip; the index won't be in the string table, downstream lookup will STALL on that specific index. Non-fatal.

## Testing strategy

Manual, with a USB analyzer (Beagle USB 12 / Total Phase or `usbmon` on a Linux host loop) for ground truth.

For each phase, against at least:
- a boot-mouse-only mouse (regression baseline),
- a composite gaming kbd+mouse combo with media keys (5+ interfaces),
- a Logitech G-series mouse (HID++, interrupt OUT — Phase 3 specifically),
- a Razer mouse if available (vendor SET_REPORT > 64 B — Phase 1 specifically).

Per phase, capture both the direct enumeration and the MITM enumeration of the same physical device, diff the descriptor bytes. Phase 1 success = no STALLs on any standard / class request the direct device honors. Phase 2 success = byte-identical config descriptor and string table. Phase 3 success = Logitech G HUB (or `solaar` on Linux) successfully reads battery / DPI through the MITM.

No automated tests are feasible without a USB simulator or hardware-in-the-loop rig the project doesn't currently have. Spec deliberately stops short of building that infrastructure.

## Rollout

Phase 1 first — it's the highest-impact fix (silent data loss) and the smallest change. Land, verify regression-free on the boot-mouse baseline, then Phase 2, then Phase 3.

Each phase is a separate commit with its own description so revert is easy if something breaks in the field.
