# USB Accuracy Phase 3 — Interrupt OUT Endpoint Pass-Through Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Support host → upstream-device interrupt OUT traffic. Concretely: when the downstream PC writes to an interrupt OUT endpoint that the upstream device exposes (e.g. Logitech HID++ report IDs `0x10`/`0x11` flowing over the dedicated vendor interface's interrupt OUT EP, or a 3-interface gaming mouse's EP4 OUT used for configuration commands), forward the bytes verbatim to the upstream device. Today both the capture stage and both stacks ignore the OUT direction entirely.

**Architecture:** Mirrors the existing interrupt IN infrastructure on both the host and device sides. On the device side, OUT endpoints get a parallel set of dQHs (`qh_idx = ep*2`), dTDs, and double-banked RX buffers. On the host side, OUT QHs are linked into the periodic frame list at the same bInterval as the IN side, but their QTDs are armed on-demand (when downstream data arrives) rather than auto-rearmed every poll. `main.c`'s forward loop gains a parallel "device-side OUT → host-side OUT" path. Adds a per-interface `interrupt_out_ep` field (and renames the existing `interrupt_ep` → `interrupt_in_ep` for clarity).

**Tech Stack:** Bare-metal C, EHCI on i.MX RT1062, same toolchain.

**Spec reference:** `docs/specs/2026-05-17-usb-device-accuracy-design.md` — Phase 3.

**Prerequisite:** Phase 2 plan (`2026-05-17-usb-accuracy-phase-2-descriptors.md`) merged. Phase 3 reuses Phase 2's lifted caps and `has_hid_desc` infrastructure.

---

## Pre-flight

- [ ] **Verify Phase 2 merged**

Run: `git log --oneline -16` — expected: Phase 1 + Phase 2 commits present (tags `usb-accuracy-phase1`, `usb-accuracy-phase2` if used).
Run: `git status` — expected: clean (or only unrelated edits from prior sessions: Makefile, kmbox.c, smooth.c).

- [ ] **Identify test device for end-to-end verification (hardware)**

Phase 3 is hardest to validate without the right hardware. Best test targets, in order of preference:
1. **Logitech G-series mouse** (G502, G Pro, G703) — uses HID++ over interrupt IN+OUT on a dedicated vendor interface. Verify with `solaar` on Linux (`solaar config` should successfully read battery, profile, DPI through the MITM).
2. **Any gaming mouse with on-device configurator** that uses a vendor interface — capture USB traffic on direct-plug vs MITM with Wireshark + usbmon and confirm interrupt OUT URBs reach the device.
3. **Wireless dongle with bidirectional vendor protocol** — Logitech Unifying, similar.

If none available, Phase 3 ships as "code-complete, hardware-validation pending."

---

## Task 1: Schema — add OUT endpoint fields, rename IN for clarity

**Files:**
- Modify: `src/desc_capture.h` — `captured_iface_t`

**Why:** Add fields for the OUT direction (`interrupt_out_ep`, `interrupt_out_maxpkt`, `interrupt_out_interval`) and rename the existing IN fields for symmetry. The rename is a one-shot cost paid here; it makes Tasks 3-11 read naturally.

- [ ] **Step 1: Read the current struct**

Read `src/desc_capture.h` lines 10-30 (post-Phase-2 shape).

- [ ] **Step 2: Rename and extend**

Edit `src/desc_capture.h`. Find:
```c
	uint8_t  interrupt_ep;         // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_maxpkt;     // max packet size for interrupt EP
	uint8_t  interrupt_interval;   // polling interval
```
Replace with:
```c
	uint8_t  interrupt_in_ep;       // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_in_maxpkt;   // max packet size for interrupt IN EP
	uint8_t  interrupt_in_interval; // polling interval for IN
	uint8_t  interrupt_out_ep;      // OUT EP addr (0x01 etc), 0 if none
	uint16_t interrupt_out_maxpkt;  // max packet size for interrupt OUT EP
	uint8_t  interrupt_out_interval;// polling interval for OUT
```

- [ ] **Step 3: Build to identify all referenced sites**

Run: `make`
Expected: **build will fail** with errors at every reference to `interrupt_ep`, `interrupt_maxpkt`, `interrupt_interval`. Use the compiler errors as a worklist for the next task.

This task intentionally breaks the build to make the rename's reach visible. Don't commit yet — the rename is half-done. Continue to Task 2 which finishes the rename across in-tree call sites.

---

## Task 2: Update parse_config_descriptor and downstream call sites for the rename + OUT capture

**Files:**
- Modify: `src/desc_capture.c` — `parse_config_descriptor` endpoint handling (around lines 79-89)
- Modify: `src/main.c` — every reference to `interrupt_ep` / `interrupt_maxpkt` / `interrupt_interval`
- Modify: `src/usb_device.c` — `configure_all_interrupt_endpoints` and any other site using these fields

**Why:** Finish the Task 1 rename and make the parser capture both IN and OUT interrupt endpoints.

- [ ] **Step 1: Update parser to capture both directions**

Edit `src/desc_capture.c`. Find:
```c
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && cur_iface != NULL) {
			uint8_t ep_addr = p[2];
			uint8_t ep_attr = p[3];
			uint16_t ep_maxpkt = p[4] | (p[5] << 8);
			uint8_t ep_interval = p[6];
			if ((ep_attr & 3) == 3 && (ep_addr & 0x80)) {
				cur_iface->interrupt_ep       = ep_addr;
				cur_iface->interrupt_maxpkt   = ep_maxpkt;
				cur_iface->interrupt_interval = ep_interval;
			}
		}
```
Replace with:
```c
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && cur_iface != NULL) {
			uint8_t ep_addr = p[2];
			uint8_t ep_attr = p[3];
			uint16_t ep_maxpkt = p[4] | (p[5] << 8);
			uint8_t ep_interval = p[6];
			if ((ep_attr & 3) == 3) {  // Interrupt endpoint (Type bits = 11b)
				if (ep_addr & 0x80) {
					// IN direction (bit 7 set in address)
					cur_iface->interrupt_in_ep       = ep_addr;
					cur_iface->interrupt_in_maxpkt   = ep_maxpkt;
					cur_iface->interrupt_in_interval = ep_interval;
				} else {
					// OUT direction
					cur_iface->interrupt_out_ep       = ep_addr;
					cur_iface->interrupt_out_maxpkt   = ep_maxpkt;
					cur_iface->interrupt_out_interval = ep_interval;
				}
			}
		}
```

- [ ] **Step 2: Update main.c references**

Read `src/main.c` around the lines the compiler flagged. Expected sites (post-Phase-2):
- ~line 156: `if (desc.ifaces[i].interrupt_ep == 0) continue;`
- ~line 159: `uint8_t ep = desc.ifaces[i].interrupt_ep & 0x0F;`
- ~line 162: `desc.ifaces[i].interrupt_maxpkt`
- ~line 166: `ep_map[slot].maxpkt = desc.ifaces[i].interrupt_maxpkt;`
- ~line 176: `uint8_t bint = desc.ifaces[i].interrupt_interval;`

Edit each: replace `interrupt_ep` → `interrupt_in_ep`, `interrupt_maxpkt` → `interrupt_in_maxpkt`, `interrupt_interval` → `interrupt_in_interval`. Do NOT add OUT handling yet — Task 11 will add a parallel OUT mapping loop separately to keep the diff focused.

- [ ] **Step 3: Update usb_device.c references**

Read `src/usb_device.c` around the compiler-flagged lines. Expected: `configure_all_interrupt_endpoints()` (around line 202-235) walks `cap_desc->ifaces[i].interrupt_ep` and `.interrupt_maxpkt`. Rename each to `.interrupt_in_ep` / `.interrupt_in_maxpkt`. (OUT counterpart will be added in Task 8.)

- [ ] **Step 4: Build — should now compile clean**

Run: `make`
Expected: clean build. If any references remain, fix them and retry.

- [ ] **Step 5: Smoke-flash and verify regression-free**

If hardware:
```bash
make flash
```
Plug baseline mouse. `lsusb -v` and confirm enumeration still works (no functional change — we renamed fields and captured OUT EPs into a struct field nobody reads yet).

- [ ] **Step 6: Commit Tasks 1 + 2 together**

```bash
git add src/desc_capture.h src/desc_capture.c src/usb_device.c src/main.c
git commit -m "refactor(usb): rename interrupt_ep → interrupt_in_ep, capture OUT EPs

Adds interrupt_out_ep / interrupt_out_maxpkt / interrupt_out_interval to
captured_iface_t and updates the parser to populate them when the descriptor
walk hits an interrupt endpoint with bit 7 clear in the address. Renames the
existing fields to interrupt_in_ep / *_maxpkt / *_interval for symmetry.

Captured OUT fields are read by no code yet — Tasks 3-11 wire them through
the host and device stacks. Observable behavior unchanged."
```

---

## Task 3: Host-side OUT endpoint storage and helpers

**Files:**
- Modify: `src/usb_host.h` — add OUT-specific cap and prototypes
- Modify: `src/usb_host.c` — add OUT arrays parallel to the IN ones

**Why:** Replicate the IN-side per-slot state for OUT direction. OUT slots are independent from IN slots (a device may have IN-only or IN+OUT or OUT-only interfaces).

- [ ] **Step 1: Add OUT cap and prototypes in header**

Edit `src/usb_host.h`. After the existing interrupt-IN section:
```c
#define MAX_INTR_EPS 7

void usb_host_interrupt_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt);
int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len);
int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len);
void usb_host_interrupt_dump_state(void);
```
Add:
```c
#define MAX_INTR_OUT_EPS 7

void usb_host_interrupt_out_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt);
// Returns true if the send was armed (QTD primed). Returns false if a previous
// send on this slot is still in flight — caller should retry next poll cycle.
bool usb_host_interrupt_out_send(uint8_t index, const uint8_t *data, uint16_t len);
```

- [ ] **Step 2: Add parallel OUT arrays in usb_host.c**

Edit `src/usb_host.c`. After the existing IN arrays (around line 27):
```c
static ehci_qh_t   qh_intr_out[MAX_INTR_OUT_EPS]
	__attribute__((section(".dmabuffers"), aligned(64)));
static ehci_qtd_t  qtd_intr_out[MAX_INTR_OUT_EPS]
	__attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t     intr_out_buf[MAX_INTR_OUT_EPS][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
static bool        intr_out_initialized[MAX_INTR_OUT_EPS];
static bool        intr_out_transfer_active[MAX_INTR_OUT_EPS];
static uint32_t    intr_out_prime_time[MAX_INTR_OUT_EPS];
static uint8_t     intr_out_dev_addr[MAX_INTR_OUT_EPS];
static uint8_t     intr_out_ep_num[MAX_INTR_OUT_EPS];
static uint8_t     num_intr_out_eps = 0;
```

- [ ] **Step 3: Build (just structures so far — no errors expected)**

Run: `make`
Expected: clean build. Unused functions/vars warnings are acceptable until Task 4 wires them in.

- [ ] **Step 4: Commit**

```bash
git add src/usb_host.h src/usb_host.c
git commit -m "feat(usb_host): scaffolding for interrupt OUT endpoint state

Adds parallel arrays for QH, QTD, double-banked buffer, busy/active flags,
per-slot dev_addr and ep_num — mirroring the existing IN-side state. No
functions reference them yet; subsequent commits add init, send, and
periodic-list linkage. MAX_INTR_OUT_EPS=7 matches MAX_INTR_EPS."
```

---

## Task 4: `usb_host_interrupt_out_init` and `usb_host_interrupt_out_send`

**Files:**
- Modify: `src/usb_host.c` — add init and send functions

**Why:** OUT EPs need a QH set up with PID OUT, address, max-packet, and the appropriate split-transaction masks for full/low speed via the embedded TT. Unlike IN, the QTD is **not** auto-rearmed every interval — the caller arms a fresh QTD only when there's new data.

- [ ] **Step 1: Add `usb_host_interrupt_out_init` near the existing init**

Edit `src/usb_host.c`. After `usb_host_interrupt_init()` (around line 509), add:
```c
void usb_host_interrupt_out_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt)
{
	if (index >= MAX_INTR_OUT_EPS) return;

	ehci_qh_t *qh = &qh_intr_out[index];
	memset(qh, 0, sizeof(*qh));
	uint32_t cap0 = 0;
	cap0 |= ((uint32_t)maxpkt << 16);
	cap0 |= ((uint32_t)device_speed << 12);
	cap0 |= ((uint32_t)(ep & 0x0F) << 8);
	cap0 |= addr;
	qh->capabilities[0] = cap0;
	uint32_t cap1 = (1 << 30); // Mult = 1
	if (device_speed == USB_SPEED_HIGH) {
		cap1 |= 0xFF;          // S-mask: all µFrames
	} else {
		cap1 |= 0x01;          // S-mask: start-split in µFrame 0
		cap1 |= (0x1C << 8);   // C-mask: complete-split in µFrames 2,3,4
	}
	qh->capabilities[1] = cap1;

	qh->next = QTD_TERMINATE;
	qh->alt_next = QTD_TERMINATE;
	qh->token = 0;

	intr_out_initialized[index]     = true;
	intr_out_transfer_active[index] = false;
	intr_out_dev_addr[index]        = addr;
	intr_out_ep_num[index]          = ep & 0x0F;

	if (index >= num_intr_out_eps)
		num_intr_out_eps = index + 1;

	link_periodic_schedule();  // Both IN and OUT QHs share the periodic frame list
}
```

- [ ] **Step 2: Add `usb_host_interrupt_out_send` immediately after init**

Same file, after the init:
```c
bool usb_host_interrupt_out_send(uint8_t index, const uint8_t *data, uint16_t len)
{
	if (index >= MAX_INTR_OUT_EPS || !intr_out_initialized[index]) return false;
	if (len > sizeof(intr_out_buf[0])) return false;

	// Check whether the previous QTD has completed; if still active, refuse —
	// caller must drain before sending the next packet.
	if (intr_out_transfer_active[index]) {
		uint32_t token = qh_intr_out[index].token;
		if (token & QTD_TOKEN_ACTIVE) {
			// Still in flight — bail unless we've been waiting too long
			if ((millis() - intr_out_prime_time[index]) <= 100) {
				return false;
			}
			// Stuck QTD: clear and re-arm below
			qh_intr_out[index].token = token & QTD_TOKEN_TOGGLE;
			qh_intr_out[index].next  = QTD_TERMINATE;
			asm volatile("dsb" ::: "memory");
		}
		intr_out_transfer_active[index] = false;
	}

	ehci_qh_t *qh = &qh_intr_out[index];
	uint8_t *buf  = intr_out_buf[index];
	memcpy(buf, data, len);

	uint32_t toggle = qh->token & QTD_TOKEN_TOGGLE;
	qh->next     = QTD_TERMINATE;
	qh->alt_next = QTD_TERMINATE;
	qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_OUT |
		QTD_TOKEN_NBYTES(len) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC;
	{
		uint32_t a = (uint32_t)buf;
		qh->buffer[0] = a;
		a &= 0xFFFFF000;
		qh->buffer[1] = a + 0x1000;
		qh->buffer[2] = a + 0x2000;
		qh->buffer[3] = a + 0x3000;
		qh->buffer[4] = a + 0x4000;
	}
	asm volatile("dsb" ::: "memory");

	intr_out_transfer_active[index] = true;
	intr_out_prime_time[index]      = millis();
	return true;
}
```

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build. Warning about `link_periodic_schedule` may now appear if its signature didn't previously include OUT QHs — addressed in Task 5.

- [ ] **Step 4: Commit**

```bash
git add src/usb_host.c
git commit -m "feat(usb_host): usb_host_interrupt_out_init + ..._send

Mirrors the IN init for QH setup (PID, address, maxpkt, S/C-masks for
full-speed split transactions via embedded TT). usb_host_interrupt_out_send
arms a single-shot QTD with PID OUT when called; if the previous QTD is
still in flight, returns false so the caller can retry next poll. If the
previous transfer has been stuck > 100ms, recovers by clearing the QTD
before arming the new one.

OUT QHs are not yet linked into the periodic frame list — Task 5 extends
link_periodic_schedule() to include them."
```

---

## Task 5: Link OUT QHs into the periodic frame list

**Files:**
- Modify: `src/usb_host.c` — `link_periodic_schedule()`

**Why:** EHCI requires interrupt endpoints (IN and OUT) to be reached via the periodic frame list. Currently only IN QHs are linked. Without this change, the controller never visits the OUT QH and `usb_host_interrupt_out_send()` arms a QTD that never executes.

- [ ] **Step 1: Read current `link_periodic_schedule`**

Read `src/usb_host.c` around lines 380-410 (the function visible from grep results).

- [ ] **Step 2: Extend to include OUT QHs**

Edit `link_periodic_schedule()`. The current logic walks `intr_initialized[]`, chains each via `horizontal_link`, and points `periodic_list[i]` at the head. Extend to walk OUT QHs as well, chaining them after the last IN QH:

Find the current function body. Locate the loop that builds the IN chain (each QH's `horizontal_link` points to the next IN QH). Add an analogous OUT pass that continues the chain. The exact rewrite depends on the current code's loop structure — read it carefully and write the rewrite to keep the chain ordering: `IN0 → IN1 → ... → INn → OUT0 → OUT1 → ... → OUTm → terminate`.

Concrete shape (after reading the existing function, adapt to match its style):
```c
static void link_periodic_schedule(void)
{
	// Build chain: IN slots first, then OUT slots.
	// Each QH's horizontal_link points to the next QH or terminate (bit 0 = T).

	// Walk IN QHs in reverse so each link points to the *next-later* one.
	uint32_t next_link = 1; // terminate
	for (int i = MAX_INTR_OUT_EPS - 1; i >= 0; i--) {
		if (!intr_out_initialized[i]) continue;
		qh_intr_out[i].horizontal_link = next_link;
		next_link = (uint32_t)&qh_intr_out[i] | 0x02; // type=QH
	}
	for (int i = MAX_INTR_EPS - 1; i >= 0; i--) {
		if (!intr_initialized[i]) continue;
		qh_intr[i].horizontal_link = next_link;
		next_link = (uint32_t)&qh_intr[i] | 0x02; // type=QH
	}

	// Pick the head — first initialized IN, else first initialized OUT, else terminate
	uint32_t head = 1;
	for (uint8_t i = 0; i < MAX_INTR_EPS; i++) {
		if (intr_initialized[i]) { head = (uint32_t)&qh_intr[i] | 0x02; break; }
	}
	if (head == 1) {
		for (uint8_t i = 0; i < MAX_INTR_OUT_EPS; i++) {
			if (intr_out_initialized[i]) { head = (uint32_t)&qh_intr_out[i] | 0x02; break; }
		}
	}

	asm volatile("dsb" ::: "memory");
	for (int i = 0; i < 32; i++) {
		periodic_list[i] = head;
	}
}
```

**Important:** the existing code may already build the chain in a different order or use different bookkeeping. Reading the actual function before editing is mandatory — the snippet above is a model, not a paste-in. The invariant to preserve: every frame slot in `periodic_list[]` points to a QH chain that terminates with `T=1`, all initialized QHs are present, and chain ordering does not loop back to itself.

- [ ] **Step 3: Build and smoke-test**

Run: `make`
Expected: clean build.

If hardware:
```bash
make flash
```
Plug baseline mouse. Confirm IN still works (mouse cursor moves). This is the regression check — if `link_periodic_schedule` broke IN chain ordering, mouse input stops.

- [ ] **Step 4: Commit**

```bash
git add src/usb_host.c
git commit -m "feat(usb_host): link interrupt OUT QHs into periodic frame list

link_periodic_schedule() now chains IN slots followed by OUT slots into a
single per-frame chain reachable from periodic_list[]. EHCI requires
interrupt endpoints (both directions) to live on the periodic list; without
this OUT QHs were primed but never visited by the controller.

Regression-tested against baseline mouse (IN-only): cursor input still
works, so the IN chain ordering is preserved."
```

---

## Task 6: Device-side OUT endpoint storage

**Files:**
- Modify: `src/usb_device.h` — add `MAX_INT_OUT_EPS`
- Modify: `src/usb_device.c` — add parallel OUT arrays + slot map

**Why:** Mirror the IN side state. RX dTDs are double-banked (matching the IN side's TX double-bank) so RX can re-prime the other bank as soon as a packet completes.

- [ ] **Step 1: Add cap in header**

Edit `src/usb_device.h`. After the existing `MAX_INT_EPS`:
```c
// Maximum interrupt OUT endpoints — symmetric with MAX_INT_EPS, supports
// passing host→device traffic on vendor interfaces (Logitech HID++, etc.)
#define MAX_INT_OUT_EPS 7
```

Also add to the public API near `usb_device_send_report`:
```c
// Drain a completed RX from the given device OUT EP. Returns:
//   > 0  : bytes received, *data points into DMA buffer (valid until next call)
//   = 0  : no completion yet
//   < 0  : EP not configured / error
int usb_device_poll_out(uint8_t ep_num, uint8_t **data_ptr);
```

- [ ] **Step 2: Add parallel OUT arrays in usb_device.c**

Edit `src/usb_device.c`. After the IN arrays (around lines 14-21):
```c
static usb_dev_dtd_t dtd_int_rx[MAX_INT_OUT_EPS]
	__attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t int_rx_buf[MAX_INT_OUT_EPS][2][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
```

After the IN slot-map state (around lines 24-28):
```c
static uint8_t ep_to_slot_out[USB_DEV_NUM_ENDPOINTS]; // OUT EP num -> slot
static uint8_t num_int_out_eps;
static uint8_t out_active_bank_mask;  // bit set = EP using bank 1, clear = bank 0
static uint8_t out_pending_mask;      // bit set = EP has a completed packet waiting to be drained
```

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build, possibly unused-variable warnings for the new state until subsequent tasks reference it.

- [ ] **Step 4: Commit**

```bash
git add src/usb_device.h src/usb_device.c
git commit -m "feat(usb_device): scaffolding for interrupt OUT endpoint RX

Parallel dTD, double-banked buffer, slot map, and bookkeeping for OUT
direction. Mirrors the IN-side layout. usb_device_poll_out prototype added
to the public API; implementation comes in Task 7."
```

---

## Task 7: Device-side `configure_all_interrupt_out_endpoints` + `prime_int_out_ep`

**Files:**
- Modify: `src/usb_device.c`

**Why:** Configure each OUT endpoint's dQH on the device controller and prime initial RX. Symmetric with the existing IN-side `configure_all_interrupt_endpoints()`.

- [ ] **Step 1: Add `prime_int_out_ep` helper above `configure_all_interrupt_endpoints`**

Edit `src/usb_device.c`. Find the existing `prime_int_ep` (around line 45). Insert immediately after:
```c
// Prime an interrupt OUT endpoint to receive into the given bank buffer.
static void prime_int_out_ep(uint8_t ep_num, uint8_t slot, uint8_t bank, uint16_t maxpkt)
{
	uint8_t qh_idx = ep_num * 2;  // OUT side of the dQH pair

	dtd_int_rx[slot].next  = DTD_TERMINATE;
	dtd_int_rx[slot].token = DTD_ACTIVE | DTD_IOC | DTD_TOTAL_BYTES(maxpkt);
	dtd_int_rx[slot].buffer[0] = (uint32_t)int_rx_buf[slot][bank];
	dtd_int_rx[slot].buffer[1] = ((uint32_t)int_rx_buf[slot][bank] + 4096) & ~0xFFFu;

	dqh_list[qh_idx].next  = (uint32_t)&dtd_int_rx[slot];
	dqh_list[qh_idx].token = 0;
	asm volatile("dsb" ::: "memory");

	USB1_ENDPTPRIME = (1 << ep_num);  // RX prime bit = ep_num (no +16 like TX)
	if (bank)
		out_active_bank_mask |= (1 << ep_num);
	else
		out_active_bank_mask &= ~(1 << ep_num);
}
```

- [ ] **Step 2: Add `configure_all_interrupt_out_endpoints` next to its IN sibling**

Edit `src/usb_device.c`. Find `configure_all_interrupt_endpoints()` (around line 202-235). Add immediately after:
```c
static void configure_all_interrupt_out_endpoints(void)
{
	num_int_out_eps = 0;
	memset(ep_to_slot_out, 0xFF, sizeof(ep_to_slot_out));

	for (uint8_t i = 0; i < cap_desc->num_ifaces; i++) {
		uint8_t ep_addr = cap_desc->ifaces[i].interrupt_out_ep;
		if (ep_addr == 0) continue;

		uint8_t ep_num = ep_addr & 0x0F;
		if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) continue;
		if (num_int_out_eps >= MAX_INT_OUT_EPS) break;

		uint8_t slot = num_int_out_eps;
		ep_to_slot_out[ep_num] = slot;
		uint16_t maxpkt = cap_desc->ifaces[i].interrupt_out_maxpkt;
		if (maxpkt > sizeof(int_rx_buf[0][0])) maxpkt = sizeof(int_rx_buf[0][0]);

		// Configure dQH for OUT direction (qh_idx = ep_num * 2)
		uint8_t qh_idx = ep_num * 2;
		dqh_list[qh_idx].config = DQH_MAX_PACKET(maxpkt);
		dqh_list[qh_idx].next   = DTD_TERMINATE;
		dqh_list[qh_idx].token  = 0;

		// ENDPTCTRL: RX bit 0-7 controls OUT direction. Enable + interrupt type.
		volatile uint32_t *ctrl = endptctrl_reg(ep_num);
		*ctrl |= (1 << 7)               // RX enable
		      |  (3 << 2);              // RX type = interrupt (binary 11)

		// Prime initial RX into bank 0
		prime_int_out_ep(ep_num, slot, 0, maxpkt);

		num_int_out_eps++;
	}
}
```

- [ ] **Step 3: Wire the new function into `usb_device_init`**

Find `usb_device_init` (around line 199). Locate the existing call to `configure_all_interrupt_endpoints()`. Add immediately after:
```c
	configure_all_interrupt_out_endpoints();
```

- [ ] **Step 4: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 5: Smoke-test (if hardware)**

If hardware: flash and plug baseline mouse. Expected: no regression (boot mouse has no interrupt OUT EP, so configure_all_interrupt_out_endpoints() does nothing and the rest of enumeration is unchanged). If a device with an interrupt OUT EP is available, plug it and look in `dmesg` on the downstream host for endpoint configuration confirmation.

- [ ] **Step 6: Commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): configure interrupt OUT endpoints from captured descriptors

For each captured interface with interrupt_out_ep != 0, allocate a slot,
program the dQH on the OUT direction (qh_idx = ep*2), enable RX in
ENDPTCTRL, and prime the first receive into bank 0. Up to MAX_INT_OUT_EPS=7
endpoints. Boot-only devices are unaffected (no interrupt OUT EPs)."
```

---

## Task 8: Device-side OUT completion polling — `usb_device_poll_out`

**Files:**
- Modify: `src/usb_device.c`

**Why:** Drain RX completions from the device-side OUT endpoints and return the buffer pointer + byte count so main.c can forward to the upstream device. Symmetric with IN side's poll-driven completion handling.

- [ ] **Step 1: Add `usb_device_poll_out` near the end of the file**

Edit `src/usb_device.c`. Add (good placement: near `usb_device_send_report`, with the other public functions):
```c
int usb_device_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{
	if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) return -1;
	uint8_t slot = ep_to_slot_out[ep_num];
	if (slot == 0xFF) return -1;  // EP not configured for OUT

	uint32_t token = dtd_int_rx[slot].token;
	if (token & DTD_ACTIVE) {
		return 0;  // Still receiving
	}
	if (token & (DTD_HALTED | DTD_BUFFER_ERR | DTD_XACT_ERR)) {
		// Error — re-prime on the same bank and report none
		uint8_t bank = (out_active_bank_mask >> ep_num) & 1;
		uint16_t maxpkt = sizeof(int_rx_buf[0][0]);
		prime_int_out_ep(ep_num, slot, bank, maxpkt);
		return 0;
	}

	uint8_t completed_bank = (out_active_bank_mask >> ep_num) & 1;
	uint32_t remaining = (token >> 16) & 0x7FFF;
	uint16_t maxpkt = sizeof(int_rx_buf[0][0]);
	uint16_t got = (uint16_t)(maxpkt - remaining);

	// Hand caller a pointer to the completed bank
	*data_ptr = int_rx_buf[slot][completed_bank];

	// Re-prime on the other bank so the next packet can land while caller
	// is forwarding this one upstream.
	uint8_t next_bank = completed_bank ^ 1;
	prime_int_out_ep(ep_num, slot, next_bank, maxpkt);

	return (int)got;
}
```

- [ ] **Step 2: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add src/usb_device.c
git commit -m "feat(usb_device): usb_device_poll_out drains completed OUT RX

Returns bytes received on a given device OUT EP plus a pointer to the
double-banked buffer (zero-copy). Re-primes the other bank immediately so
back-to-back transfers don't drop packets while the caller forwards. Errors
(halt, buffer err, transaction err) re-prime the same bank and return 0
(no data), preserving the toggle state."
```

---

## Task 9: main.c — set up OUT mapping at enumeration

**Files:**
- Modify: `src/main.c`

**Why:** During enumeration, walk the captured interfaces, register each OUT EP with `usb_host_interrupt_out_init`, and record the mapping for the forward loop.

- [ ] **Step 1: Read the IN mapping setup**

Read `src/main.c` lines 145-180 to see the existing IN setup pattern. Confirm the `ep_mapping_t` struct definition (line ~47).

- [ ] **Step 2: Add an OUT mapping struct + array**

Edit `src/main.c`. After the existing `ep_mapping_t` definition, add:
```c
typedef struct {
	uint8_t host_slot;     // index into usb_host's intr_out arrays
	uint8_t dev_ep_num;    // device-side EP number to poll
	uint16_t maxpkt;
} ep_out_mapping_t;
```

In the enumeration block where `ep_map[]` and `num_ep_mappings` are declared:
```c
	ep_mapping_t ep_map[MAX_INTR_EPS];
	uint8_t num_ep_mappings = 0;
```
Add adjacent:
```c
	ep_out_mapping_t out_map[MAX_INTR_OUT_EPS];
	uint8_t num_out_mappings = 0;
```

- [ ] **Step 3: Add the OUT init loop after the IN init loop**

After the existing IN setup loop (around lines 155-170), add:
```c
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_out_ep == 0) continue;
		if (num_out_mappings >= MAX_INTR_OUT_EPS) break;

		uint8_t slot = num_out_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_out_ep & 0x0F;

		usb_host_interrupt_out_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_out_maxpkt);

		out_map[slot].host_slot  = slot;
		out_map[slot].dev_ep_num = ep;
		out_map[slot].maxpkt     = desc.ifaces[i].interrupt_out_maxpkt;
		num_out_mappings++;
	}
```

- [ ] **Step 4: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add src/main.c
git commit -m "feat(main): register interrupt OUT endpoints with host stack at enum

For each captured interface with interrupt_out_ep != 0, initializes a host-
side OUT slot and records dev_ep_num → host_slot mapping. The forward loop
(next commit) reads this mapping to know where to send drained OUT data."
```

---

## Task 10: main.c — forward loop for device-OUT → host-OUT

**Files:**
- Modify: `src/main.c` — main poll loop, after the IN forward block

**Why:** The actual data pipe. Whenever a packet completes on the device side's OUT EP, ship it to the upstream device via the host stack.

- [ ] **Step 1: Locate the existing IN forward block**

Read `src/main.c` lines 240-270 to confirm the IN forward pattern (around `for (uint8_t m = 0; m < num_ep_mappings; m++)`).

- [ ] **Step 2: Add parallel OUT forward block immediately after**

After the closing brace of the IN forward loop, add:
```c
		for (uint8_t m = 0; m < num_out_mappings; m++) {
			uint8_t *out_data = NULL;
			int n = usb_device_poll_out(out_map[m].dev_ep_num, &out_data);
			if (n > 0 && out_data) {
				// Best-effort: if host-side OUT is still busy, drop this packet.
				// Real-world OUT traffic is low rate (vendor config), so back-pressure
				// via drop is acceptable. If this becomes a problem, add a small ring.
				(void)usb_host_interrupt_out_send(out_map[m].host_slot, out_data, (uint16_t)n);
			}
		}
```

- [ ] **Step 3: Build**

Run: `make`
Expected: clean build.

- [ ] **Step 4: Smoke-flash and regression-test**

If hardware: flash, plug baseline mouse. Confirm cursor still moves (IN forward not broken).

- [ ] **Step 5: Commit**

```bash
git add src/main.c
git commit -m "feat(main): forward interrupt OUT packets from device side to upstream

For each registered OUT mapping, poll the device-side OUT EP for completed
RX and immediately ship the bytes to the upstream device via the host
stack's OUT send. If the host-side OUT is still busy with a previous send
the packet is dropped — vendor-config traffic is low-rate enough that this
back-pressure strategy is acceptable. Drop in production reveals itself as
'sometimes the LED doesn't change on first click' — add a small ring buffer
if observed."
```

---

## Task 11: End-to-end verification

**Files:** None modified.

- [ ] **Step 1: Final build and flash**

```bash
make clean && make && make flash
```

- [ ] **Step 2: Regression — baseline mouse**

Plug boot mouse. Confirm cursor tracks. Run `lsusb -v` and diff against Phase 2 output. Expected: identical for IN-only devices.

- [ ] **Step 3: Logitech G-series test (the headline case)**

Plug a Logitech G-series mouse (G502, G Pro, G703, etc.) upstream. On Linux downstream:

```bash
# Install solaar if not present
solaar show       # Should list the mouse
solaar config <device> dpi   # Should read DPI from the device
solaar config <device> dpi 1600   # Should set DPI
```

Before Phase 3: `solaar` enumerates the device but config reads time out (HID++ replies flow over interrupt IN but commands must go over interrupt OUT, which was unsupported).
After Phase 3: full configuration works.

- [ ] **Step 4: USB analyzer capture (if available)**

With Beagle USB 12 / Total Phase Data Center or a Linux box running `usbmon`:
1. Capture upstream-direct enumeration + a configurator session of the test device.
2. Capture MITM enumeration + same configurator session.
3. Diff URB payloads. Expected: every URB on the direct trace has a matching URB on the MITM trace (timing may differ).

- [ ] **Step 5: Tag Phase 3 completion**

```bash
git tag usb-accuracy-phase3
```

---

## Self-review checklist

- [ ] Every task has explicit file:line citations
- [ ] Every code change shows the exact from/to (or full new function body)
- [ ] No TBDs, no "handle errors appropriately"
- [ ] All struct field rename references are accounted for (Tasks 1 + 2 list every site)
- [ ] `MAX_INTR_OUT_EPS` (host header) and `MAX_INT_OUT_EPS` (device header) — naming intentional, mirror the existing asymmetry between `MAX_INTR_EPS` (host) and `MAX_INT_EPS` (device)
- [ ] `prime_int_out_ep` PRIME bit is `(1 << ep_num)` not `(1 << (16 + ep_num))` — the latter is the TX (IN) side
- [ ] `qh_idx = ep_num * 2` for OUT (matches device-side dQH layout: `[ep*2] = RX, [ep*2+1] = TX`)
- [ ] OUT QH chain in `link_periodic_schedule` does not loop or terminate the wrong way; regression-tested in Task 5

Spec coverage check:

| Spec gap | Plan task |
|---|---|
| C6 OUT EPs filtered out at capture | Tasks 1 + 2 |
| C7 OUT EPs unconfigured on device side | Tasks 6 + 7 |
| New: host-side OUT scheduling | Tasks 3 + 4 + 5 |
| New: main.c forward path | Tasks 9 + 10 |

All Phase 3 spec items covered.
