# USB Accuracy Phase 2 — Composite Caps & Descriptor Completeness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lift the silent caps that drop interfaces and endpoints during composite enumeration (`MAX_INT_EPS=4`, `MAX_INTR_EPS=4`, `MAX_INTERFACES=4`, `MAX_STRINGS=8`), capture HID report descriptors for any interface that declares one (not just `iface_class == 3`), capture and replay the device's real LANGID instead of hardcoding `0x0409`, capture all referenced string indices (including `iConfiguration` and per-interface `iInterface`), and capture/replay BOS and Microsoft OS 1.0 descriptors.

**Architecture:** Extends `captured_descriptors_t` and `captured_iface_t` with new fields. Adds three new capture helpers (`capture_strings_all`, `capture_bos`, `capture_ms_os`) inside `desc_capture.c`. Touches `usb_device.c`'s `handle_get_descriptor()` STRING and adds a new BOS case. All work is constant additions — no infrastructure restructuring. Phase 1's passthrough default safely backs up every new descriptor case (if capture fails or device STALLs, Phase 1's `default: handle_passthrough()` handles the live request).

**Tech Stack:** Bare-metal C, i.MX RT1062, same toolchain as Phase 1.

**Spec reference:** `docs/specs/2026-05-17-usb-device-accuracy-design.md` — Phase 2.

**Prerequisite:** Phase 1 plan (`2026-05-17-usb-accuracy-phase-1-correctness.md`) must be merged. Phase 2 relies on Phase 1's passthrough default as the fallback for capture failures.

---

## Pre-flight

- [ ] **Verify Phase 1 merged and tree clean**

Run: `git log --oneline -3` — expected: Phase 1 commits present (tagged `usb-accuracy-phase1` if Phase 1 was tagged).
Run: `git status` — expected: no modified files under `src/usb_device.c`, `src/desc_capture.c`, `src/desc_capture.h`.

- [ ] **Capture pre-change descriptor baseline for a composite device (hardware)**

If you have a composite device with > 4 interfaces (e.g. Logitech G502 has multiple HID interfaces, a Razer Huntsman has keyboard + media + vendor + LED), plug it upstream. On the downstream host:

```bash
lsusb -v -d <vid>:<pid> > /tmp/imxrtnsy-phase1-composite.txt 2>&1
```

This is the regression *and improvement* baseline: Phase 2 should preserve all interfaces in the diff (currently missing) and add string-table / BOS sections (currently absent).

---

## Task 1: Lift the silent caps in headers and verify build size

**Files:**
- Modify: `src/usb_device.h:53` (`MAX_INT_EPS`)
- Modify: `src/usb_host.h:72` (`MAX_INTR_EPS`)
- Modify: `src/desc_capture.h:9` (`MAX_INTERFACES`)
- Modify: `src/desc_capture.h:8` (`MAX_STRINGS`)

**Why:** Four independent compile-time caps silently truncate at 4 or 8. After the change: 7 interrupt IN endpoints (`USB_DEV_NUM_ENDPOINTS - 1`, the controller's hardware max), 7 host-side interrupt IN slots (symmetric), 8 interfaces, 16 strings (covers device manufacturer/product/serial + iConfiguration + up to 8 iInterface strings + 4 reserve).

- [ ] **Step 1: Read each header to confirm current values**

Read `src/usb_device.h:50-60`. Confirm `#define MAX_INT_EPS 4`.
Read `src/usb_host.h:70-80`. Confirm `#define MAX_INTR_EPS 4`.
Read `src/desc_capture.h:1-15`. Confirm `#define MAX_INTERFACES 4` and `#define MAX_STRINGS 8`.

- [ ] **Step 2: Bump each constant**

Edit `src/usb_device.h`:
```c
// Maximum interrupt IN endpoints for composite device support
#define MAX_INT_EPS 4
```
to:
```c
// Maximum interrupt IN endpoints for composite device support.
// Hardware ceiling is USB_DEV_NUM_ENDPOINTS - 1 (EP0 reserved for control).
#define MAX_INT_EPS 7
```

Edit `src/usb_host.h`:
```c
#define MAX_INTR_EPS 4
```
to:
```c
#define MAX_INTR_EPS 7
```

Edit `src/desc_capture.h`:
```c
#define MAX_STRINGS             8
#define MAX_INTERFACES          4
```
to:
```c
#define MAX_STRINGS             16
#define MAX_INTERFACES          8
```

- [ ] **Step 3: Build and measure RAM impact**

Run: `make`
Expected: clean build, no warnings.

Run: `arm-none-eabi-size firmware.elf`
Compare to Phase 1 final size. Expected growth in `data`/`bss` ≈ `(3 extra MAX_INT_EPS slots × (32 dtd + 128 buf) = 480 B) + (4 extra MAX_INTERFACES × ~535 B captured_iface_t = 2140 B) + (8 extra MAX_STRINGS × 128 B = 1024 B)` ≈ 3.6 KB. Confirm DTCM / OCRAM still has headroom (i.MX RT1062 has 1 MB OCRAM, 256 KB DTCM, this is well under).

If the build fails to fit, stop and re-evaluate (very unlikely given the headroom).

- [ ] **Step 4: Smoke-flash and verify regression-free**

If hardware:
```bash
make flash
```
Plug baseline mouse, run `lsusb -v` and diff against the Phase 1 mouse output. Expected: identical (cap bumps alone change nothing observable for boot mice).

- [ ] **Step 5: Commit**

```bash
git add src/usb_device.h src/usb_host.h src/desc_capture.h
git commit -m "feat(usb): lift silent caps for composite device support

MAX_INT_EPS    4 → 7  (i.MX RT1062 EHCI hardware ceiling for non-EP0)
MAX_INTR_EPS   4 → 7  (host-side symmetric)
MAX_INTERFACES 4 → 8  (covers realistic composite devices)
MAX_STRINGS    8 → 16 (device + iConfiguration + per-interface iInterface)

Composite devices with > 4 interrupt IN endpoints, > 4 interfaces, or > 8
string descriptors were silently truncated during enumeration: extra endpoints
got no dQH allocated and NAK'd; extra interfaces were never parsed; extra
strings returned STALL. RAM cost ≈ 3.6 KB in .dmabuffers / .bss; fits
comfortably in OCRAM."
```

---

## Task 2: Extend `captured_iface_t` and `captured_descriptors_t` schema

**Files:**
- Modify: `src/desc_capture.h` — struct definitions, new constants

**Why:** Phase 2 captures more state per interface (`has_hid_desc` flag replaces the class-3 gate, `iface_string_idx` for iInterface lookup) and more descriptor types in the top-level struct (BOS, MS OS, LANGID, iConfiguration index). Adding the fields up front lets Task 3+ fill them in without further schema churn.

- [ ] **Step 1: Read current header**

Read `src/desc_capture.h` lines 1-40 to see current shape.

- [ ] **Step 2: Add new size constants**

Edit `src/desc_capture.h` to add (after the existing `#define MAX_STRINGS 16` line):
```c
#define MAX_BOS_DESC_SIZE       256
#define MAX_LANGID_DESC_SIZE    8
#define MS_OS_1_0_STRING_SIZE   18  // Fixed: bLength=0x12, signature "MSFT100", vendor code byte
```

- [ ] **Step 3: Extend `captured_iface_t`**

Edit `src/desc_capture.h`. Change:
```c
typedef struct {
	uint8_t  iface_num;            // bInterfaceNumber
	uint8_t  iface_class;          // bInterfaceClass (3 = HID)
	uint8_t  iface_subclass;       // bInterfaceSubClass
	uint8_t  iface_protocol;       // bInterfaceProtocol
	uint8_t  interrupt_ep;         // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_maxpkt;     // max packet size for interrupt EP
	uint8_t  interrupt_interval;   // polling interval
	uint8_t  hid_report_desc[MAX_HID_REPORT_DESC_SIZE];
	uint16_t hid_report_desc_len;  // 0 if not HID or not fetched
} captured_iface_t;
```
to:
```c
typedef struct {
	uint8_t  iface_num;            // bInterfaceNumber
	uint8_t  iface_class;          // bInterfaceClass (3 = HID)
	uint8_t  iface_subclass;       // bInterfaceSubClass
	uint8_t  iface_protocol;       // bInterfaceProtocol
	uint8_t  iface_string_idx;     // iInterface string index (0 if none)
	uint8_t  interrupt_ep;         // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_maxpkt;     // max packet size for interrupt EP
	uint8_t  interrupt_interval;   // polling interval
	bool     has_hid_desc;         // true if interface contained a HID descriptor
	uint8_t  hid_report_desc[MAX_HID_REPORT_DESC_SIZE];
	uint16_t hid_report_desc_len;  // 0 if not HID or not fetched
} captured_iface_t;
```

Note: `interrupt_ep` / `interrupt_maxpkt` / `interrupt_interval` are kept as-is for Phase 2. Phase 3 renames them to `interrupt_in_ep` and adds the OUT counterparts.

- [ ] **Step 4: Extend `captured_descriptors_t`**

Edit `src/desc_capture.h`. Change:
```c
typedef struct {
	uint8_t  device_desc[18];
	uint8_t  device_desc_len;
	uint8_t  config_desc[MAX_CONFIG_DESC_SIZE];
	uint16_t config_desc_len;
	captured_iface_t ifaces[MAX_INTERFACES];
	uint8_t  num_ifaces;
	uint8_t  string_desc[MAX_STRINGS][MAX_STRING_DESC_SIZE];
	uint8_t  string_desc_len[MAX_STRINGS];
	uint8_t  string_index[MAX_STRINGS]; // Original USB string index for each
	uint8_t  num_strings;
	uint8_t  ep0_maxpkt;
	uint8_t  dev_addr;

	bool valid;
} captured_descriptors_t;
```
to:
```c
typedef struct {
	uint8_t  device_desc[18];
	uint8_t  device_desc_len;
	uint8_t  config_desc[MAX_CONFIG_DESC_SIZE];
	uint16_t config_desc_len;
	uint8_t  config_string_idx;                     // iConfiguration

	captured_iface_t ifaces[MAX_INTERFACES];
	uint8_t  num_ifaces;

	uint8_t  string_desc[MAX_STRINGS][MAX_STRING_DESC_SIZE];
	uint8_t  string_desc_len[MAX_STRINGS];
	uint8_t  string_index[MAX_STRINGS];             // Original USB string index for each
	uint8_t  num_strings;

	uint8_t  langid_desc[MAX_LANGID_DESC_SIZE];     // String-0 LANGID table, captured verbatim
	uint8_t  langid_desc_len;                       // 0 if capture failed (replay falls back)
	uint16_t langid;                                // First language ID parsed from langid_desc

	uint8_t  bos_desc[MAX_BOS_DESC_SIZE];           // BOS descriptor blob, replayed verbatim
	uint16_t bos_desc_len;                          // 0 if device has no BOS (USB 2.0)

	uint8_t  ms_os_desc[MS_OS_1_0_STRING_SIZE];     // MS OS 1.0 string at index 0xEE
	uint8_t  ms_os_desc_len;                        // 0 if device has no MS_OS_1.0
	uint8_t  ms_os_vendor_code;                     // Byte at offset 0x10 of ms_os_desc

	uint8_t  ep0_maxpkt;
	uint8_t  dev_addr;
	bool     valid;
} captured_descriptors_t;
```

- [ ] **Step 5: Build to confirm schema compiles cleanly**

Run: `make`
Expected: clean build. Any in-tree references to removed fields (none — we only added) cause compile errors here; expect none.

- [ ] **Step 6: Commit**

```bash
git add src/desc_capture.h
git commit -m "feat(desc_capture): extend captured state for full descriptor replay

Adds per-interface has_hid_desc flag and iface_string_idx, plus top-level
config_string_idx, langid_desc/langid, bos_desc, and ms_os_desc fields.
Fields are zero-initialized via memset and unused until subsequent commits
fill them in; this commit is schema-only."
```

---

## Task 3: Drop the `iface_class == 3` gate on HID descriptor handling

**Files:**
- Modify: `src/desc_capture.c:66-78` (parse-time gate in `parse_config_descriptor`)
- Modify: `src/desc_capture.c:149-167` (capture-time gate in `capture_descriptors`)

**Why:** Some HID-style devices declare interfaces with vendor class (`0xFF`) that still contain a HID descriptor and a report descriptor. The current code only handles `iface_class == 3` and silently skips everything else. The correct gate is "did the parsed interface contain a HID descriptor."

- [ ] **Step 1: Read both sites**

Read `src/desc_capture.c` lines 33-95 and 149-167.

- [ ] **Step 2: Update `parse_config_descriptor` to record `has_hid_desc` and drop class gate**

Edit `src/desc_capture.c`. Change the existing HID-descriptor handling block:
```c
		} else if (dtype == USB_DESC_HID && dlen >= 9 && cur_iface != NULL) {
			if (cur_iface->iface_class == 3) {
				uint8_t num_descs = p[5];
				for (uint8_t i = 0; i < num_descs; i++) {
					if (6 + i * 3 + 2 < dlen) {
						uint8_t rtype = p[6 + i * 3];
						uint16_t rlen = p[7 + i * 3] | (p[8 + i * 3] << 8);
						if (rtype == USB_DESC_HID_REPORT) {
							cur_iface->hid_report_desc_len = rlen;
						}
					}
				}
			}
		}
```
to:
```c
		} else if (dtype == USB_DESC_HID && dlen >= 9 && cur_iface != NULL) {
			cur_iface->has_hid_desc = true;
			uint8_t num_descs = p[5];
			for (uint8_t i = 0; i < num_descs; i++) {
				if (6 + i * 3 + 2 < dlen) {
					uint8_t rtype = p[6 + i * 3];
					uint16_t rlen = p[7 + i * 3] | (p[8 + i * 3] << 8);
					if (rtype == USB_DESC_HID_REPORT) {
						cur_iface->hid_report_desc_len = rlen;
					}
				}
			}
		}
```

- [ ] **Step 3: Update `parse_config_descriptor` to record `iface_string_idx`**

In the same function, find the INTERFACE descriptor handling block (lines 46-65 area):
```c
		if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
			uint8_t alt_setting = p[3];
			...
			if (desc->num_ifaces < MAX_INTERFACES) {
				cur_iface = &desc->ifaces[desc->num_ifaces++];
				memset(cur_iface, 0, sizeof(*cur_iface));
				cur_iface->iface_num      = p[2];
				cur_iface->iface_class    = p[5];
				cur_iface->iface_subclass = p[6];
				cur_iface->iface_protocol = p[7];
			} else {
				cur_iface = NULL;
			}
		}
```
Change the assignment block to:
```c
			if (desc->num_ifaces < MAX_INTERFACES) {
				cur_iface = &desc->ifaces[desc->num_ifaces++];
				memset(cur_iface, 0, sizeof(*cur_iface));
				cur_iface->iface_num        = p[2];
				cur_iface->iface_class      = p[5];
				cur_iface->iface_subclass   = p[6];
				cur_iface->iface_protocol   = p[7];
				cur_iface->iface_string_idx = p[8];  // iInterface
			} else {
				cur_iface = NULL;
			}
```

- [ ] **Step 4: Update capture loop to use `has_hid_desc` gate**

Edit `src/desc_capture.c`. Change:
```c
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		captured_iface_t *iface = &desc->ifaces[i];
		if (iface->iface_class != 3) continue;
		if (iface->hid_report_desc_len == 0) continue;
```
to:
```c
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		captured_iface_t *iface = &desc->ifaces[i];
		if (!iface->has_hid_desc) continue;
		if (iface->hid_report_desc_len == 0) continue;
```

Also update the SET_IDLE loop further down. Find:
```c
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (desc->ifaces[i].iface_class != 3) continue;
		setup.bmRequestType = 0x21; // Host-to-Device, Class, Interface
		setup.bRequest = 0x0A;      // HID SET_IDLE
		...
	}
```
Change the gate to:
```c
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (!desc->ifaces[i].has_hid_desc) continue;
		setup.bmRequestType = 0x21; // Host-to-Device, Class, Interface
		setup.bRequest = 0x0A;      // HID SET_IDLE
		...
	}
```

(Note: SET_IDLE may STALL legally — the existing code ignores failures, keep that.)

- [ ] **Step 5: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 6: Smoke-test (if hardware)**

Flash and re-enumerate baseline mouse. Compare `lsusb -v` to previous output. Expected: identical for class-3 devices.

If a vendor-class-with-HID device is available, confirm its HID report descriptor section now appears in `lsusb -v` output (it would have been empty before).

- [ ] **Step 7: Commit**

```bash
git add src/desc_capture.c
git commit -m "feat(desc_capture): capture HID report descriptors regardless of interface class

Some devices declare HID descriptors inside vendor-class interfaces (class
0xFF). The HID 1.11 spec doesn't require class=3 for a HID descriptor to be
present — class=3 just unlocks the boot-protocol interfaces.

Gate is now 'interface contains a HID descriptor' (has_hid_desc flag) rather
than 'iface_class == 3'. Applies to both report-descriptor capture and the
SET_IDLE init pass. Also captures iInterface string index per interface for
later string-table lookup."
```

---

## Task 4: Capture the LANGID descriptor verbatim and store the parsed language ID

**Files:**
- Modify: `src/desc_capture.c:170-176` (LANGID fetch block)

**Why:** The current code fetches string 0 (LANGID table), parses out the first language ID, then discards the raw bytes. `usb_device.c:124-127` replies to string-0 requests with a hardcoded `{0x04, 0x03, 0x09, 0x04}` (always 0x0409 English). Storing the raw LANGID descriptor lets us replay byte-identical.

- [ ] **Step 1: Locate the LANGID fetch**

Read `src/desc_capture.c:169-176`.

- [ ] **Step 2: Replace fetch to store the raw bytes**

Edit `src/desc_capture.c`. Change:
```c
	uint8_t str_buf[MAX_STRING_DESC_SIZE];
	setup = make_get_descriptor(USB_DESC_STRING, 0, 0, 4);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, str_buf, 2000);
	uint16_t langid = 0x0409; // Default to English
	if (ret >= 4 && str_buf[1] == USB_DESC_STRING) {
		langid = str_buf[2] | (str_buf[3] << 8);
	}
```
to:
```c
	// Capture the full LANGID descriptor (string index 0). We replay it
	// verbatim downstream so the host sees the device's actual language list.
	setup = make_get_descriptor(USB_DESC_STRING, 0, 0, MAX_LANGID_DESC_SIZE);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->langid_desc, 2000);
	desc->langid = 0x0409; // Default if device returns garbage / STALLs
	desc->langid_desc_len = 0;
	if (ret >= 4 && desc->langid_desc[1] == USB_DESC_STRING) {
		desc->langid_desc_len = (uint8_t)ret;
		desc->langid = desc->langid_desc[2] | (desc->langid_desc[3] << 8);
	}
	uint16_t langid = desc->langid;  // local alias used by subsequent string fetches
```

The `uint16_t langid` local alias keeps the rest of the function (which uses `langid` as a local variable name) compiling unchanged.

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/desc_capture.c
git commit -m "feat(desc_capture): store LANGID descriptor verbatim for replay

Captured the full string-0 LANGID table bytes (not just the first language
ID). Device-side handler (next commit) replays them so the downstream host
sees the device's actual supported-language list instead of a hardcoded
0x0409 English-only response."
```

---

## Task 5: Replay the captured LANGID from the device-side handler

**Files:**
- Modify: `src/usb_device.c:124-127` (or wherever the STRING handler builds the langid response)

**Why:** Without this change Task 4's capture is unused.

- [ ] **Step 1: Read the STRING handler**

Read `src/usb_device.c:120-145`. Find the block that handles `wValue & 0xFF == 0` (string index 0, the LANGID request). It currently looks like:
```c
case USB_DESC_STRING:
	if (index == 0) {
		static const uint8_t langid_desc[] = {0x04, 0x03, 0x09, 0x04};
		data = langid_desc;
		len  = sizeof(langid_desc);
	} else {
		// lookup in cap_desc->string_desc
		...
	}
	break;
```
(actual code may differ slightly — read it to confirm structure).

- [ ] **Step 2: Replace the hardcoded langid path**

Edit `src/usb_device.c`. Change the `if (index == 0)` branch to:
```c
		if (index == 0) {
			if (cap_desc->langid_desc_len > 0) {
				data = cap_desc->langid_desc;
				len  = cap_desc->langid_desc_len;
			} else {
				// Fallback: synthesize 0x0409 if capture failed.
				static const uint8_t langid_fallback[] = {0x04, 0x03, 0x09, 0x04};
				data = langid_fallback;
				len  = sizeof(langid_fallback);
			}
		} else {
```

- [ ] **Step 3: Build, flash, verify (if hardware)**

Run: `make && make flash`
Plug a device that advertises multiple languages (most don't, but checking that single-language devices still work covers the common path).

Run: `lsusb -v -d <vid>:<pid>` and confirm the string-0 line under "Descriptor Set" matches the upstream device. For a single-language English-only device this looks the same as before; for a multi-language device the difference is now visible.

- [ ] **Step 4: Commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): replay captured LANGID instead of hardcoded 0x0409

Devices that advertise multiple language IDs (LANGIDs) at string index 0
had their full list collapsed to a single hardcoded 0x0409 (English) when
the downstream host queried it. Now the captured raw descriptor is replayed
byte-for-byte. Falls back to 0x0409 only if capture failed."
```

---

## Task 6: Capture all referenced string indices (generic loop)

**Files:**
- Modify: `src/desc_capture.c` — the string-capture loop (around lines 177-197)

**Why:** Currently only iManufacturer (`device_desc[14]`), iProduct (`device_desc[15]`), iSerialNumber (`device_desc[16]`) are captured. iConfiguration (`config_desc[6]`) and per-interface iInterface (`iface_string_idx` from Task 3) are referenced in the descriptors but never fetched; the downstream host that requests them gets a NULL from the lookup, which falls through to `ep0_stall()`.

- [ ] **Step 1: Capture iConfiguration into the schema field**

This piggybacks on the existing parse: after `desc->config_desc_len = (uint16_t)ret;` (around line 147) and before `parse_config_descriptor(desc);`, add:
```c
	desc->config_string_idx = desc->config_desc[6]; // iConfiguration
```

- [ ] **Step 2: Replace the hand-rolled string loop with a generic collector**

Find the current loop:
```c
	uint8_t string_indices[3] = {
		desc->device_desc[14], // iManufacturer
		desc->device_desc[15], // iProduct
		desc->device_desc[16], // iSerialNumber
	};

	desc->num_strings = 0;
	for (int i = 0; i < 3; i++) {
		if (string_indices[i] == 0) continue;
		if (desc->num_strings >= MAX_STRINGS) break;

		setup = make_get_descriptor(USB_DESC_STRING, string_indices[i],
			langid, MAX_STRING_DESC_SIZE);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->string_desc[desc->num_strings], 2000);
		if (ret > 0) {
			desc->string_desc_len[desc->num_strings] = ret;
			desc->string_index[desc->num_strings] = string_indices[i];
			desc->num_strings++;
		}
	}
```
Replace with:
```c
	// Collect every string index referenced by the device, config, and
	// interface descriptors. Dedup so we don't fetch the same index twice
	// (common: many devices use the same string for multiple iInterface fields).
	uint8_t string_indices[3 + 1 + MAX_INTERFACES];
	uint8_t string_indices_count = 0;
	string_indices[string_indices_count++] = desc->device_desc[14]; // iManufacturer
	string_indices[string_indices_count++] = desc->device_desc[15]; // iProduct
	string_indices[string_indices_count++] = desc->device_desc[16]; // iSerialNumber
	string_indices[string_indices_count++] = desc->config_string_idx;
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		string_indices[string_indices_count++] = desc->ifaces[i].iface_string_idx;
	}

	desc->num_strings = 0;
	for (uint8_t i = 0; i < string_indices_count; i++) {
		uint8_t idx = string_indices[i];
		if (idx == 0) continue;

		// Dedup: skip if already captured
		bool already = false;
		for (uint8_t j = 0; j < desc->num_strings; j++) {
			if (desc->string_index[j] == idx) { already = true; break; }
		}
		if (already) continue;
		if (desc->num_strings >= MAX_STRINGS) break;

		setup = make_get_descriptor(USB_DESC_STRING, idx,
			langid, MAX_STRING_DESC_SIZE);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->string_desc[desc->num_strings], 2000);
		if (ret > 0) {
			desc->string_desc_len[desc->num_strings] = ret;
			desc->string_index[desc->num_strings] = idx;
			desc->num_strings++;
		}
	}
```

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 4: Smoke-test (if hardware)**

Plug a device that has named interfaces (most gaming peripherals do — e.g. "Razer Mouse", "Razer Multimedia"). Run `lsusb -v` and confirm `iInterface 4 Razer Multimedia` style lines now resolve to actual strings instead of unresolved indices.

- [ ] **Step 5: Commit**

```bash
git add src/desc_capture.c
git commit -m "feat(desc_capture): capture iConfiguration and per-interface iInterface strings

The previous hand-rolled string loop only captured iManufacturer / iProduct /
iSerialNumber. Composite devices reference more strings — iConfiguration at
config_desc[6] and one iInterface per interface descriptor. Untouched indices
caused the downstream host's GET_STRING request to STALL.

Refactored to a generic collector: dedups indices (common when interfaces
share names) and bounds by MAX_STRINGS=16."
```

---

## Task 7: Capture the BOS descriptor

**Files:**
- Modify: `src/desc_capture.c` — add `capture_bos()` helper and call site
- Modify: `src/desc_capture.h` — add `USB_DESC_BOS` macro

**Why:** USB 2.1+ devices have a BOS descriptor that Windows 10+ probes during enumeration. With Phase 1's passthrough default the request is forwarded live, but caching it eliminates per-probe latency and is byte-identical replay.

- [ ] **Step 1: Add the descriptor type macro**

Edit `src/desc_capture.h` (or `src/usb_host.h` which already has other USB_DESC macros — prefer the one closer to where USB_DESC_DEVICE etc. live). Find:
```c
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22
```
Add:
```c
#define USB_DESC_BOS            0x0F
```

- [ ] **Step 2: Add `capture_bos()` helper above `capture_descriptors()`**

Edit `src/desc_capture.c`. Insert after `parse_config_descriptor()` and before `capture_descriptors()`:
```c
static void capture_bos(captured_descriptors_t *desc)
{
	// Probe BOS header (5 bytes: bLength, bDescriptorType, wTotalLength, bNumDeviceCaps)
	usb_setup_t setup = make_get_descriptor(USB_DESC_BOS, 0, 0, 5);
	uint8_t hdr[5];
	int ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, hdr, 2000);

	// Device doesn't support BOS — STALL. Not fatal; Phase 1 passthrough handles
	// live requests if the host probes anyway.
	if (ret < 5 || hdr[1] != USB_DESC_BOS) {
		desc->bos_desc_len = 0;
		return;
	}

	uint16_t total_len = hdr[2] | (hdr[3] << 8);
	if (total_len < 5) {
		desc->bos_desc_len = 0;
		return;
	}
	if (total_len > MAX_BOS_DESC_SIZE) total_len = MAX_BOS_DESC_SIZE;

	setup = make_get_descriptor(USB_DESC_BOS, 0, 0, total_len);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->bos_desc, 2000);
	if (ret < 5 || desc->bos_desc[1] != USB_DESC_BOS) {
		desc->bos_desc_len = 0;
		return;
	}
	desc->bos_desc_len = (uint16_t)ret;
}
```

- [ ] **Step 3: Wire `capture_bos()` into `capture_descriptors()`**

Edit `src/desc_capture.c`. Find the `SET_IDLE for each HID interface` loop (near the end of `capture_descriptors`). Add immediately **before** that loop (so BOS capture runs after the device is configured but before the HID-specific class requests):
```c
	capture_bos(desc);
```

- [ ] **Step 4: Build and commit**

Run: `make`
Expected: clean build.

```bash
git add src/desc_capture.c src/desc_capture.h
git commit -m "feat(desc_capture): capture BOS descriptor for USB 2.1+ devices

Probes the device's BOS during enumeration: header first (5 bytes) to learn
total length, then full descriptor. Stored as raw bytes for verbatim replay
by the device side (next commit). Devices that STALL on BOS (USB 2.0)
leave bos_desc_len = 0 — Phase 1's passthrough default still handles live
requests from the downstream host."
```

---

## Task 8: Serve the captured BOS from the device side

**Files:**
- Modify: `src/usb_device.c` — add `USB_DESC_BOS` case to `handle_get_descriptor`

**Why:** Without this case, downstream BOS requests fall through to Phase 1's passthrough default and incur a live round-trip every time. Cached replay is the optimization. (Functional behavior is unchanged for working devices; latency is the only difference.)

- [ ] **Step 1: Add the case**

Edit `src/usb_device.c`. Inside the descriptor `switch` in `handle_get_descriptor`, add a new case (placement: with the other GET_DESCRIPTOR type cases, before the `default`):
```c
		case USB_DESC_BOS:
			if (cap_desc->bos_desc_len > 0) {
				data = cap_desc->bos_desc;
				len  = cap_desc->bos_desc_len;
			} else {
				handle_passthrough(setup);
				return;
			}
			break;
```

If `USB_DESC_BOS` is defined in `desc_capture.h` and not visible to `usb_device.c`, add an include — but `usb_device.c` already `#include "desc_capture.h"` indirectly via `usb_device.h:4`, so the macro is in scope.

- [ ] **Step 2: Build and commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): replay captured BOS descriptor

Cached BOS bytes are served directly when the downstream host requests
GET_DESCRIPTOR(BOS). If the upstream device had no BOS (USB 2.0), falls
through to Phase 1's passthrough default — observable behavior unchanged."
```

---

## Task 9: Capture MS OS 1.0 descriptor

**Files:**
- Modify: `src/desc_capture.c` — add `capture_ms_os_1_0()` helper and call site

**Why:** Microsoft OS 1.0 lives at fixed string index `0xEE` with signature `"MSFT100"`. The host queries it during enumeration before any driver loads. Caching the response means the downstream host's probe is answered immediately and byte-identical. The follow-up vendor-specific Compatible-ID and Extended-Properties requests already pass through via Phase 1's vendor-request route — no additional handling needed.

- [ ] **Step 1: Add `capture_ms_os_1_0()` helper**

Edit `src/desc_capture.c`. Insert after `capture_bos()`:
```c
static void capture_ms_os_1_0(captured_descriptors_t *desc)
{
	// MS OS 1.0 lives at string index 0xEE. Fixed 18-byte response with
	// signature "MSFT100" at offset 2 (UTF-16LE).
	usb_setup_t setup = make_get_descriptor(USB_DESC_STRING, 0xEE, 0,
		MS_OS_1_0_STRING_SIZE);
	uint8_t buf[MS_OS_1_0_STRING_SIZE];
	int ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, buf, 2000);

	if (ret < MS_OS_1_0_STRING_SIZE || buf[1] != USB_DESC_STRING) {
		desc->ms_os_desc_len = 0;
		return;
	}

	// Verify "MSFT100" signature at offset 2 (UTF-16LE: M S F T 1 0 0)
	static const uint8_t sig[] = {
		'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0
	};
	if (memcmp(&buf[2], sig, sizeof(sig)) != 0) {
		desc->ms_os_desc_len = 0;
		return;
	}

	memcpy(desc->ms_os_desc, buf, MS_OS_1_0_STRING_SIZE);
	desc->ms_os_desc_len = MS_OS_1_0_STRING_SIZE;
	desc->ms_os_vendor_code = buf[16]; // bMS_VendorCode at offset 0x10
}
```

- [ ] **Step 2: Wire into `capture_descriptors()`**

Edit `src/desc_capture.c`. Immediately after the `capture_bos(desc);` call added in Task 7, add:
```c
	capture_ms_os_1_0(desc);
```

- [ ] **Step 3: Build and commit**

```bash
git add src/desc_capture.c
git commit -m "feat(desc_capture): probe MS OS 1.0 descriptor at string index 0xEE

Microsoft OS 1.0 descriptor is queried by Windows during enumeration via
GET_STRING(0xEE) and must contain signature 'MSFT100' to be valid. Captured
into ms_os_desc[18] with the vendor code byte separately stashed. Vendor
follow-up requests (Compatible ID, Extended Properties) already pass through
via the vendor-request routing in Phase 1, so this capture only saves the
initial probe latency."
```

---

## Task 10: Serve the captured MS OS string from the device side

**Files:**
- Modify: `src/usb_device.c` — special-case string index 0xEE in the STRING handler

**Why:** Symmetric to Task 8 for BOS.

- [ ] **Step 1: Augment the STRING handler**

Edit `src/usb_device.c`. Inside the `case USB_DESC_STRING:` block, after the `if (index == 0)` LANGID handling and before the general `cap_desc->string_desc` lookup, add:
```c
		} else if (index == 0xEE && cap_desc->ms_os_desc_len > 0) {
			data = cap_desc->ms_os_desc;
			len  = cap_desc->ms_os_desc_len;
		} else {
```

(Effectively converts the existing `else { /* lookup */ }` into `else if { 0xEE special case } else { /* lookup */ }`.)

If the device had no MS OS descriptor (`ms_os_desc_len == 0`) and the downstream host requests 0xEE, the lookup loop won't find it either, and the request falls through to `ep0_stall()` — same as a real device that doesn't implement MS OS would. Correct.

- [ ] **Step 2: Build and commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): replay captured MS OS 1.0 string at index 0xEE

Mirror of the BOS replay path: cached MS OS 1.0 descriptor is served for
GET_STRING(0xEE) requests. Devices that did not implement MS OS 1.0 fall
through to the standard string lookup (which returns nothing for 0xEE),
which then STALLs — matching the real device's behavior."
```

---

## Task 11: End-to-end verification

**Files:** None modified.

- [ ] **Step 1: Build and flash final Phase 2 firmware**

```bash
make clean && make && make flash
```

- [ ] **Step 2: Regression: baseline mouse**

Plug baseline boot mouse. `lsusb -v` diff against Phase 1 output. Expected: identical except possibly a now-present iManufacturer/iProduct/iSerial string that was already captured before — no functional change.

- [ ] **Step 3: Composite device test (5+ interfaces)**

Plug a composite device with 5+ interfaces. `lsusb -v -d <vid>:<pid>`. Confirm:
- All interfaces appear (compare interface numbers/counts to the direct-plug capture)
- Per-interface iInterface strings resolve (not "(no string)" or empty)
- iConfiguration resolves if device has one
- If device has BOS: appears in output
- If device has MS OS 1.0: `lsusb -v` shows the 0xEE string with `MSFT100` signature

If any field differs from direct-plug, capture both and compare — it indicates a still-missing case.

- [ ] **Step 4: Vendor-class HID interface test (if available)**

If you have a gaming mouse with a vendor-class HID interface (rare; some Logitech and Razer devices), confirm its HID report descriptor now appears in `lsusb -v`.

- [ ] **Step 5: Tag Phase 2 completion**

```bash
git tag usb-accuracy-phase2
```

---

## Self-review checklist

- [ ] Every task has explicit file:line citations
- [ ] Every code change shows the exact `from` and `to`
- [ ] No TBDs, no "handle errors appropriately"
- [ ] Manual verification steps are concrete commands with expected output
- [ ] All new struct fields are defined in Task 2 before being referenced in Tasks 4-10
- [ ] `MS_OS_1_0_STRING_SIZE` constant is consistent (18) everywhere it appears
- [ ] `USB_DESC_BOS` (0x0F) macro definition is in Task 7 and referenced in Tasks 7 and 8

Spec coverage check:

| Spec gap | Plan task |
|---|---|
| C8 `MAX_INT_EPS = 4` | Task 1 |
| C9 `MAX_INTR_EPS = 4` | Task 1 |
| C10 `MAX_INTERFACES = 4` | Task 1 |
| C11 HID report capture gated on iface_class==3 | Task 3 |
| C12 LANGID hardcoded | Tasks 4 + 5 |
| C13 iConfiguration / iInterface strings not captured | Tasks 3 + 6 |
| C14 BOS never captured | Tasks 7 + 8 |
| C15 MS OS 1.0 never probed | Tasks 9 + 10 |

All Phase 2 spec items covered.
