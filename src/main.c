// main.c — USB HID proxy for Teensy 4.1

#include <stdint.h>
#include <stddef.h>
#include "imxrt.h"
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "kmbox.h"
#include "humanize.h"
#include "gpt_profile.h"
#include "led.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

typedef struct {
	uint8_t  host_slot;
	uint8_t  dev_ep_num;
	uint16_t maxpkt;
	uint8_t  iface_protocol;  // 1=keyboard, 2=mouse (for kmbox merge)
} ep_mapping_t;

typedef struct {
	uint8_t host_slot;     // index into usb_host's intr_out arrays
	uint8_t dev_ep_num;    // device-side EP number to poll
	uint16_t maxpkt;
} ep_out_mapping_t;

static volatile bool pit_tick_pending;
static volatile uint32_t pit_next_ldval; // precomputed by main loop
static uint32_t pit_base_ldval;          // slewed reload value (tracks measured poll rate)
static bool pit_locked;                  // adaptive base has converged on the measured rate
                                         // (main-loop only: set in tick handler, read by LED status)

// PIT input clock = PERCLK = 24 MHz crystal oscillator on this board.
// core/startup.c sets CCM_CSCMR1 PERCLK_CLK_SEL=1 (24 MHz XTAL) and PERCLK_PODF=/1.
// This is independent of F_CPU/ARM/IPG — do not derive it from F_CPU.
#define PIT_CLK_HZ 24000000u
#define PIT_INTERVAL_US_MIN 125u
#define PIT_INTERVAL_US_MAX 10000u

// Overclock thermal guard (see status-tick in main). Trip high, clear low for
// hysteresis. The i.MX RT1062 junction is rated to 105°C; we warn well under
// it since the overclock already runs the core hot. Detection only — we never
// re-clock live (would corrupt F_CPU-derived timebases). Adjust for airflow.
#define OVERTEMP_TRIP_C   85
#define OVERTEMP_CLEAR_C  73

// Convert a desired period in microseconds to a PIT LDVAL (counts-1).
// PIT_CLK_HZ is an exact multiple of 1 MHz, so counts = us * (clk/1e6) with
// pure 32-bit arithmetic — no __aeabi_uldivmod. Max 10000 µs * 24 = 240000.
_Static_assert(PIT_CLK_HZ % 1000000u == 0, "PIT clock must be a whole MHz");
static inline uint32_t pit_ldval_from_us(uint32_t interval_us)
{
	if (interval_us < PIT_INTERVAL_US_MIN) interval_us = PIT_INTERVAL_US_MIN;
	if (interval_us > PIT_INTERVAL_US_MAX) interval_us = PIT_INTERVAL_US_MAX;
	uint32_t counts = interval_us * (PIT_CLK_HZ / 1000000u);
	return counts ? counts - 1u : 0u;
}

static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF;
	PIT_LDVAL0 = pit_next_ldval; // precomputed, no FPU in ISR
	pit_tick_pending = true;
	// The TFLG W1C is a posted write; without a DSB the NVIC can still see
	// the IRQ asserted at exception return and immediately re-enter — a
	// spurious double tick (timing jitter on the injection cadence).
	__asm volatile("dsb" ::: "memory");
}

// Static to keep off the stack (~3.6KB struct)
static captured_descriptors_t desc;

static uint32_t room_count, hot_count;
static int32_t  hot_temp;

static void tempmon_init(void)
{
	CCM_CCGR2 |= CCM_CCGR2_OCOTP_CTRL(CCM_CCGR_ON);
	uint32_t ana1 = HW_OCOTP_ANA1;
	hot_count  = (ana1 >> 20) & 0xFFF;
	hot_temp   = (int32_t)((ana1 >> 12) & 0xFF);
	uint32_t ana2 = HW_OCOTP_ANA2;
	room_count = (ana2 >> 20) & 0xFFF;
	if (room_count == 0 || room_count == hot_count)
		room_count = hot_count - 35; // fallback: avoid div-by-zero
	TEMPMON_TEMPSENSE0_CLR = TEMPMON_CTRL0_POWER_DOWN;
	TEMPMON_TEMPSENSE1 = TEMPMON_CTRL1_MEASURE_FREQ(0x03FF); // ~2Hz
	TEMPMON_TEMPSENSE0_SET = TEMPMON_CTRL0_MEASURE_TEMP;
}

static int8_t tempmon_read(void)
{
	uint32_t raw = (TEMPMON_TEMPSENSE0 >> 8) & 0xFFF;
	if (raw == 0) return 0;
	// T = hot_temp - (raw - hot_count) * (hot_temp - 25) / (room_count - hot_count)
	int32_t num = ((int32_t)raw - (int32_t)hot_count) * (hot_temp - 25);
	int32_t den = (int32_t)room_count - (int32_t)hot_count;
	if (den == 0) return 0;
	return (int8_t)(hot_temp - num / den);
}

int main(void)
{
	SCB_SCR |= SCB_SCR_SEVONPEND;

	kmbox_init();
	led_init();

	// PIT0: humanization timer — clock/ISR now, rate set after enumeration
	CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
	PIT_MCR = 0; // enable module, timers run in debug
	PIT_TCTRL0 = 0; // stopped until we know the mouse poll interval
	PIT_TFLG0 = PIT_TFLG_TIF;
	attachInterruptVector(IRQ_PIT, pit0_isr);
	NVIC_SET_PRIORITY(IRQ_PIT, 64);
	NVIC_ENABLE_IRQ(IRQ_PIT);

	tempmon_init();
	gpt_profile_init(); // GPT2: free-running µs counter for profiling

	led_on();

	usb_host_init();
	led_off();
	usb_host_power_on();
	while (!usb_host_device_connected()) {
		usb_host_power_on();
		kmbox_poll_fast(); // respond to UART identity probes during init
		if (kmbox_rx_pending()) kmbox_poll_heavy();
		__asm volatile("wfe");
	}

	led_on();
	delay(10);
	usb_host_port_reset();

	uint8_t speed = usb_host_device_speed();

	if (!capture_descriptors(&desc)) {
		led_blink_forever(5, 100, 100);
	}

	// capture_descriptors() already sends SET_CONFIG and SET_IDLE.
	// Send SET_PROTOCOL (Report Protocol) for each HID interface.
	usb_setup_t setup;
	int ret;
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		// SET_PROTOCOL: bmRequestType=0x21, bRequest=0x0B
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0B; // SET_PROTOCOL
		setup.wValue = 1;      // 1 = Report Protocol (not Boot Protocol)
		setup.wIndex = desc.ifaces[i].iface_num;
		setup.wLength = 0;
		ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL, 2000);
		(void)ret;
	}
	ep_mapping_t ep_map[MAX_INTR_EPS];
	uint8_t num_ep_mappings = 0;
	ep_out_mapping_t out_map[MAX_INTR_OUT_EPS];
	uint8_t num_out_mappings = 0;

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_in_ep == 0) continue;
		if (num_ep_mappings >= MAX_INTR_EPS) break;
		uint8_t slot = num_ep_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_in_ep & 0x0F;

		usb_host_interrupt_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_in_maxpkt);

		ep_map[slot].host_slot       = slot;
		ep_map[slot].dev_ep_num      = ep;
		ep_map[slot].maxpkt          = desc.ifaces[i].interrupt_in_maxpkt;
		ep_map[slot].iface_protocol  = desc.ifaces[i].iface_protocol;
		num_ep_mappings++;
	}

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

	// Start PIT0 at the mouse's poll rate (from bInterval + device speed)
	{
		uint32_t interval_us = 1000; // default 1ms = 1kHz
		for (uint8_t i = 0; i < desc.num_ifaces; i++) {
			if (desc.ifaces[i].iface_protocol != 2) continue;
			uint8_t bint = desc.ifaces[i].interrupt_in_interval;
			if (bint == 0) bint = 1;
			if (speed == USB_SPEED_HIGH) {
				// High-speed: 2^(bInterval-1) * 125 µs. Clamp shift to avoid UB.
				uint8_t shift = (bint > 1) ? ((bint - 1u) > 30u ? 30u : (bint - 1u)) : 0u;
				interval_us = 125u << shift;
			} else {
				// Full/low speed: bInterval in ms
				interval_us = (uint32_t)bint * 1000u;
			}
			break; // use first mouse interface
		}
		// Clamp to [125µs, 10ms] — sane range for humanized injection
		if (interval_us < 125) interval_us = 125;
		if (interval_us > 10000) interval_us = 10000;
		// PIT is clocked by PERCLK = 24 MHz OSC (core/startup.c sets
		// PERCLK_CLK_SEL=1, PERCLK_PODF=/1) — NOT the IPG/ARM clock. Using
		// F_CPU/4 here made every period ~8.5x too long. See PIT_CLK_HZ.
		uint32_t ldval = pit_ldval_from_us(interval_us);
		pit_base_ldval = ldval;
		pit_next_ldval = ldval;
		PIT_LDVAL0 = ldval;
		PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
		humanize_init(interval_us);
	}
	kmbox_cache_endpoints(&desc);
	if (!usb_device_init(&desc)) {
		led_blink_forever(9, 80, 120);
	}
	led_off();
	uint32_t dev_wait_start = millis();
	uint32_t dev_led_toggle = millis();
	while (!usb_device_is_configured()) {
		usb_device_poll();
		kmbox_poll_fast(); // respond to UART identity probes during init
		if (kmbox_rx_pending()) kmbox_poll_heavy();
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}

		if ((millis() - dev_wait_start) > 30000) {
			led_blink_forever(8, 80, 120);
		}
	}
	led_off();
	led_heartbeat_start();
	uint32_t report_count = 0;
	uint32_t drop_count = 0;
	uint32_t loop_count = 0;
	uint32_t led_status_tick = millis();
	uint32_t led_rx_snapshot = 0;
	uint32_t led_err_snapshot = 0;
	uint16_t led_centihz = 0;
	bool overtemp = false;          // sticky once tripped until temp recovers

	while (1) {
		uint32_t now = millis();
		bool did_work = false;

		// --- Latency-critical: humanized PIT tick ---
		if (pit_tick_pending) {
			pit_tick_pending = false;
			did_work = true;
			// Adaptive feed rate: slew the base period toward the measured
			// real poll interval (never jump — an abrupt period change is both
			// detectable and can phase-beat the host). humanize_target_ldval
			// returns 0 until it has a confident measurement, in which case we
			// keep the enumerated nominal base. RM-correct: the new value is
			// applied at the next reload via the precompute-in-ISR pattern.
			uint32_t tgt = humanize_target_ldval(PIT_CLK_HZ);
			if (tgt) {
				int32_t err = (int32_t)tgt - (int32_t)pit_base_ldval;
				int32_t step = err >> 5;           // ~3%/tick proportional slew
				if (step == 0 && err != 0)         // guarantee final convergence:
					step = (err > 0) ? 1 : -1;     // >>5 truncates small +err to 0
				pit_base_ldval = (uint32_t)((int32_t)pit_base_ldval + step);
				// Locked when the slewed base sits within ~1.5% of the measured
				// target (|err| <= tgt/64). Drives LED feedback only.
				int32_t aerr = err < 0 ? -err : err;
				pit_locked = (aerr <= (int32_t)(tgt >> 6));
			} else {
				pit_locked = false;                // no confident measurement yet
			}
			pit_next_ldval = humanize_timing_next(pit_base_ldval);
		}

		// --- USB device EP completion (unblock EPs for next send) ---
		usb_device_poll();

		// --- Command input ---
		kmbox_poll_fast();
		if (kmbox_rx_pending()) {
			kmbox_poll_heavy();
			did_work = true;
		}

		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			uint8_t *rpt_ptr = NULL;
			ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				// Timestamp real mouse-report arrival (GPT2 1 MHz, single-load,
				// atomic). This is the precise "a report just arrived" point;
				// only the mouse interface drives the adaptive feed rate.
				if (ep_map[m].iface_protocol == 2)
					humanize_record_arrival(gpt_profile_us());
				kmbox_merge_report(ep_map[m].iface_protocol,
					rpt_ptr, ret);
				bool sent = usb_device_send_report(
					ep_map[m].dev_ep_num, rpt_ptr, ret);
				if (sent) {
					report_count++;
				} else {
					drop_count++;
				}
			}
		}
		for (uint8_t m = 0; m < num_out_mappings; m++) {
			usb_host_interrupt_out_poll(out_map[m].host_slot);
		}
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
		kmbox_send_pending();
		if (!did_work)
			__asm volatile("wfe");

		// Heartbeat rate reflects UART status. Sampled every 100 ms; the
		// QuadTimer keeps blinking on its own — we only rewrite its compare
		// (glitch-free) when the state actually changes.
		if ((now - led_status_tick) >= 100) {
			led_status_tick = now;
			// Thermal guard for the overclock. The die sensor updates ~2 Hz
			// (tempmon_init). We do NOT dynamically downclock: F_CPU is baked
			// into the GPT2 µs tick, LED scale, and PIT math, and re-clocking
			// live would corrupt those timebases and can glitch USB mid-frame.
			// Instead we surface heat — a fast distinct LED pattern + telemetry
			// — so it is visible before damage accrues. ~12°C hysteresis stops
			// the heartbeat flapping at the threshold. Tune for your airflow.
			int8_t tc = tempmon_read();
			if (!overtemp && tc >= OVERTEMP_TRIP_C)      overtemp = true;
			else if (overtemp && tc <= OVERTEMP_CLEAR_C) overtemp = false;

			uint32_t rx  = kmbox_rx_byte_count();
			uint32_t err = kmbox_uart_overrun() + kmbox_uart_framing() +
			               kmbox_uart_noise();
			uint16_t centihz;
			if (overtemp)                     centihz = 1200; // OVERTEMP ~12 Hz (highest priority)
			else if (err != led_err_snapshot) centihz = 600; // ERROR    ~6 Hz
			else if (rx != led_rx_snapshot)   centihz = 200; // ACTIVE   ~2 Hz
			else if (pit_locked)              centihz = 100; // LOCKED   ~1 Hz
			else                              centihz = 50;  // ACQUIRING ~0.5 Hz
			// Idle-slot only: UART status (ERROR/ACTIVE) keeps priority. When the
			// link is quiet, the heartbeat reports adaptive feed-rate convergence —
			// slow (0.5 Hz) while acquiring/idle, doubling to 1 Hz once the PIT
			// base has locked onto the measured poll rate. No UART involved.
			led_err_snapshot = err;
			led_rx_snapshot  = rx;
			if (centihz != led_centihz) {
				led_centihz = centihz;
				led_heartbeat_set_rate(centihz);
			}
		}
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected()) {
				led_blink_forever(6, 80, 80);
			}
		}
	}
}
