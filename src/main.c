// main.c — USB HID proxy for Teensy 4.1

#include <stdint.h>
#include <stddef.h>
#include "imxrt.h"
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "kmbox.h"
#include "smooth.h"
#include "humanize.h"
#include "gpt_profile.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// LED helpers — no-ops on the kmbox.  Pin 13 (GPIO_B0_03 / GPIO7 bit 3) is
// the on-board orange LED and is also the channel the MKL02 bootloader
// uses to talk to the ARM core over JTAG/SWD during reset.  Driving it as
// GPIO or FlexPWM2 from user code is what wedged the chip into the
// "9-blink ARM JTAG DAP Init Error" state on first boot.  The kmbox just
// doesn't touch the LED.
static void led_pwm_init(void) { }
static void led_pwm_set(uint8_t brightness) { (void)brightness; }
static void led_on(void)  { }
static void led_off(void) { }
static void led_toggle(void) __attribute__((unused));
static void led_toggle(void) { }
static void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	(void)code; (void)on_ms; (void)off_ms;
	while (1) delay(1000);
}
static void led_stage(uint8_t n) __attribute__((unused));
static void led_stage(uint8_t n) { (void)n; }
static void led_wait_once(uint8_t pulses, uint32_t on_ms, uint32_t off_ms, uint32_t gap_ms)
{
	(void)pulses; (void)on_ms; (void)off_ms;
	delay(gap_ms);
}

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
static uint32_t pit_base_ldval;          // nominal reload value (set after enumeration)

static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF;
	PIT_LDVAL0 = pit_next_ldval; // precomputed, no FPU in ISR
	pit_tick_pending = true;
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

	// PIT0: smooth injection timer — clock/ISR now, rate set after enumeration
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
	uint32_t host_wait_loops = 0;
	while (!usb_host_device_connected()) {
		usb_host_power_on();
		kmbox_poll(); // respond to UART identity probes during init
		__asm volatile("wfe");
		led_wait_once(1, 70, 120, 650);
		host_wait_loops++;
		if (host_wait_loops > 60u) {
			led_blink_forever(7, 80, 120);
		}
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
			if (speed == 2) {
				// High-speed: 2^(bInterval-1) * 125 µs
				interval_us = 125u << (bint > 1 ? bint - 1 : 0);
			} else {
				// Full/low speed: bInterval in ms
				interval_us = (uint32_t)bint * 1000u;
			}
			break; // use first mouse interface
		}
		// Clamp to [125µs, 10ms] — sane range for smooth injection
		if (interval_us < 125) interval_us = 125;
		if (interval_us > 10000) interval_us = 10000;
		uint32_t ipg_mhz = (F_CPU / 4u) / 1000000u; // IPG = ARM / 4
		uint32_t ldval = (ipg_mhz * interval_us) - 1;
		pit_base_ldval = ldval;
		pit_next_ldval = ldval;
		PIT_LDVAL0 = ldval;
		PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
		smooth_init(interval_us);
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
		kmbox_poll(); // respond to UART identity probes during init
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}

		if ((millis() - dev_wait_start) > 30000) {
			led_blink_forever(8, 80, 120);
		}
	}
	led_off();
	led_pwm_init();
	uint32_t report_count = 0;
	uint32_t drop_count = 0;
	uint32_t loop_count = 0;
	uint32_t led_off_time = 0; // non-blocking LED pulse
	uint32_t led_pwm_update = millis();
	uint32_t led_report_snapshot = 0;

	while (1) {
		uint32_t now = millis();
		bool did_work = false;

		// --- Latency-critical: smooth injection first ---
		if (pit_tick_pending) {
			pit_tick_pending = false;
			did_work = true;
			bool skip = false;
			uint32_t next_ldval = smooth_timing_next(pit_base_ldval, &skip);
			pit_next_ldval = next_ldval;
			if (!skip) {
				int16_t sx, sy;
				smooth_process_frame(&sx, &sy);
				if (sx || sy) kmbox_inject_smooth(sx, sy);
			}
		}

		// --- USB device EP completion (unblock EPs for next send) ---
		usb_device_poll();

		// --- Command input ---
		kmbox_poll();

		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			uint8_t *rpt_ptr = NULL;
			ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				kmbox_merge_report(ep_map[m].iface_protocol,
					rpt_ptr, ret);
				bool sent = usb_device_send_report(
					ep_map[m].dev_ep_num, rpt_ptr, ret);
				if (sent) {
					report_count++;
				} else {
					drop_count++;
				}
				led_on();
				led_off_time = now + 2;
			}
		}
		if (led_off_time && now >= led_off_time) {
			led_off();
			led_off_time = 0;
		}
		kmbox_send_pending();
		if (!did_work)
			__asm volatile("wfe");

		if ((now - led_pwm_update) >= 100) {
			uint32_t delta = report_count - led_report_snapshot;
			led_report_snapshot = report_count;
			uint32_t brightness = delta * 10 * 255 / 1000;
			if (brightness > 255) brightness = 255;
			led_pwm_set((uint8_t)brightness);
			led_pwm_update = now;
		}
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected()) {
				led_blink_forever(6, 80, 80);
			}
		}
	}
}
