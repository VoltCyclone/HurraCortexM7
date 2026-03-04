// imxrtnsy — USB proxy for Teensy 4.1
// Phase 1: Capture descriptors from device on USB2 (host)
// Phase 2: Replay as device on USB1, forward HID reports
// Supports composite devices with multiple HID interfaces.

#include <stdint.h>
#include <stddef.h>
#include "imxrt.h"
#include "uart.h"
#include "usb_host.h"
#include "usb_device.h"
#include "desc_capture.h"
#include "kmbox.h"
#include "smooth.h"
#if TFT_ENABLED
#include "tft_display.h"
#endif

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// LED on pin 13 (GPIO7 bit 3, configured in startup)
// When TFT is enabled, pin 13 is LPSPI4_SCK — LED functions become no-ops.
#if TFT_ENABLED
static void led_on(void)  { }
static void led_off(void) { }
static void led_toggle(void) __attribute__((unused));
static void led_toggle(void) { }
static void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	(void)code; (void)on_ms; (void)off_ms;
	tft_display_error("FATAL");
	while (1) delay(1000);
}
#else
static void led_on(void)  { GPIO7_DR_SET = (1 << 3); }
static void led_off(void) { GPIO7_DR_CLEAR = (1 << 3); }
static void led_toggle(void) __attribute__((unused));
static void led_toggle(void) { GPIO7_DR_TOGGLE = (1 << 3); }
static void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	while (1) {
		for (uint8_t i = 0; i < code; i++) {
			led_on();
			delay(on_ms);
			led_off();
			delay(off_ms);
		}
		delay(600);
	}
}
#endif

static void led_stage(uint8_t n) __attribute__((unused));
static void led_stage(uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		led_on();
		delay(80);
		led_off();
		delay(120);
	}
	delay(250);
}

static void led_wait_once(uint8_t pulses, uint32_t on_ms, uint32_t off_ms, uint32_t gap_ms)
{
	for (uint8_t i = 0; i < pulses; i++) {
		led_on();
		delay(on_ms);
		led_off();
		delay(off_ms);
	}
	delay(gap_ms);
}

// Endpoint mapping: host poll slot -> device EP number
typedef struct {
	uint8_t  host_slot;
	uint8_t  dev_ep_num;
	uint16_t maxpkt;
	uint8_t  iface_protocol;  // 1=keyboard, 2=mouse (for kmbox merge)
} ep_mapping_t;

// ---- PIT0: 1kHz tick for smooth injection timing ----
static volatile bool pit_tick_pending;
static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF; // clear interrupt flag
	pit_tick_pending = true;
}

// Static to keep off the stack (~3.6KB struct)
static captured_descriptors_t desc;

int main(void)
{
	// Enable SEVONPEND so WFE wakes on any pending interrupt
	SCB_SCR |= SCB_SCR_SEVONPEND;

	uart_init();
	kmbox_init();

	// PIT0: smooth injection timer — clock/ISR now, rate set after enumeration
	CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
	PIT_MCR = 0; // enable module, timers run in debug
	PIT_TCTRL0 = 0; // stopped until we know the mouse poll interval
	PIT_TFLG0 = PIT_TFLG_TIF;
	attachInterruptVector(IRQ_PIT, pit0_isr);
	NVIC_SET_PRIORITY(IRQ_PIT, 64);
	NVIC_ENABLE_IRQ(IRQ_PIT);

	uart_puts("\r\n\r\nimxrtnsy: USB proxy\r\n");
	uart_puts("Teensy 4.1 — i.MX RT1062\r\n\r\n");
	led_on();

	// ---- Phase 1: Host init + descriptor capture ----

	usb_host_init();
	led_off();
	usb_host_power_on();
	uint32_t host_wait_loops = 0;
	while (!usb_host_device_connected()) {
		usb_host_power_on();
		// State: waiting for device on USB2 host port — idle between blinks
		__asm volatile("wfe");
		led_wait_once(1, 70, 120, 650);
		host_wait_loops++;

		// Timeout after ~45s of connect attempts.
		if (host_wait_loops > 60u) {
			// No host-side USB device detected within timeout window
			led_blink_forever(7, 80, 120);
		}
	}

	led_on();
	delay(10);
	usb_host_port_reset();

	uint8_t speed = usb_host_device_speed();

	if (!capture_descriptors(&desc)) {
		uart_puts("\r\n=== CAPTURE FAILED ===\r\n");
		led_blink_forever(5, 100, 100);
	}

	uart_puts("\r\n=== CAPTURE COMPLETE ===\r\n");

	// ---- Phase 2: Device stack + proxy ----

	// Set configuration on the real device so it starts generating reports
	usb_setup_t setup;
	setup.bmRequestType = 0x00;
	setup.bRequest = USB_REQ_SET_CONFIG;
	setup.wValue = desc.config_desc[5]; // bConfigurationValue
	setup.wIndex = 0;
	setup.wLength = 0;
	int ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
		&setup, NULL);
	if (ret < 0) {
		uart_puts("  SET_CONFIGURATION failed!\r\n");
	}
	delay(10);

	// Initialize host-side interrupt polling for all endpoints
	// Send SET_IDLE(0) and SET_PROTOCOL(1=report) to each HID interface.
	// Some devices (including Razer) won't send interrupt reports until these
	// class-specific requests have been issued.
	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		// SET_IDLE: bmRequestType=0x21 (class, interface), bRequest=0x0A
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0A; // SET_IDLE
		setup.wValue = 0;      // duration=0 (indefinite), report ID=0 (all)
		setup.wIndex = i;      // interface number
		setup.wLength = 0;
		usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL);

		// SET_PROTOCOL: bmRequestType=0x21, bRequest=0x0B
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0B; // SET_PROTOCOL
		setup.wValue = 1;      // 1 = Report Protocol (not Boot Protocol)
		setup.wIndex = i;
		setup.wLength = 0;
		usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL);
	}
	ep_mapping_t ep_map[MAX_INTR_EPS];
	uint8_t num_ep_mappings = 0;

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		if (desc.ifaces[i].interrupt_ep == 0) continue;
		if (num_ep_mappings >= MAX_INTR_EPS) {
			uart_puts("  WARNING: too many interrupt EPs, skipping\r\n");
			break;
		}
		uint8_t slot = num_ep_mappings;
		uint8_t ep = desc.ifaces[i].interrupt_ep & 0x0F;

		usb_host_interrupt_init(slot, desc.dev_addr, ep,
			desc.ifaces[i].interrupt_maxpkt);

		ep_map[slot].host_slot       = slot;
		ep_map[slot].dev_ep_num      = ep;
		ep_map[slot].maxpkt          = desc.ifaces[i].interrupt_maxpkt;
		ep_map[slot].iface_protocol  = desc.ifaces[i].iface_protocol;
		num_ep_mappings++;
	}

	// Start PIT0 at the mouse's poll rate (from bInterval + device speed)
	{
		uint32_t interval_us = 1000; // default 1ms = 1kHz
		for (uint8_t i = 0; i < desc.num_ifaces; i++) {
			if (desc.ifaces[i].iface_protocol != 2) continue;
			uint8_t bint = desc.ifaces[i].interrupt_interval;
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
		uint32_t ldval = (150u * interval_us) - 1; // IPG = 150 MHz
		PIT_LDVAL0 = ldval;
		PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
	}

	// Cache mouse/keyboard EP numbers for fast injection dispatch
	kmbox_cache_endpoints(&desc);

	// Initialize USB1 device stack
	if (!usb_device_init(&desc)) {
		led_blink_forever(9, 80, 120);
	}

	// Wait for host PC to enumerate us — must call usb_device_poll()
	// continuously so SETUP packets are answered within the USB spec
	// timeout.  Blocking LED patterns here cause multi-minute enumeration.
	led_off();
	uint32_t dev_wait_start = millis();
	uint32_t dev_led_toggle = millis();
	while (!usb_device_is_configured()) {
		usb_device_poll();
		// Non-blocking LED toggle every 250 ms
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}

		if ((millis() - dev_wait_start) > 30000) {
			// USB1 device side not being configured by PC host
			led_blink_forever(8, 80, 120);
		}
	}
	led_off();

	// ---- TFT display init (after USB enumeration, before main loop) ----
#if TFT_ENABLED
	tft_display_init();
#endif

	// Hardware NAK rate-limits polling (1 frame = 1ms full-speed, 125us high-speed).
	uart_puts("\r\n=== PROXY ACTIVE (");
	uart_putdec(num_ep_mappings);
	uart_puts(" endpoints) ===\r\n");

	uint8_t report_buf[64];
	uint32_t report_count = 0;
	uint32_t drop_count = 0;
	uint32_t loop_count = 0;
	uint32_t last_heartbeat = millis();
	uint32_t led_off_time = 0; // non-blocking LED pulse

#if TFT_ENABLED
	uint32_t last_tft_update = millis();
	uint32_t prev_report_count = 0;
	uint32_t prev_report_time = millis();
	uint32_t reports_per_sec = 0;
#endif

	while (1) {
		usb_device_poll();
		kmbox_poll();

		// Process smooth queue at PIT-driven 1kHz rate
		if (pit_tick_pending) {
			pit_tick_pending = false;
			int16_t sx, sy;
			smooth_process_frame(&sx, &sy);
			if (sx || sy) kmbox_inject_smooth(sx, sy);
		}

		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			ret = usb_host_interrupt_poll(ep_map[m].host_slot,
				report_buf, ep_map[m].maxpkt);
			if (ret > 0) {
				kmbox_merge_report(ep_map[m].iface_protocol,
					report_buf, ret);
				bool sent = usb_device_send_report(
					ep_map[m].dev_ep_num, report_buf, ret);
				if (sent) {
					report_count++;
				} else {
					drop_count++;
				}

				// Non-blocking LED pulse on forwarded report
				led_on();
				led_off_time = millis() + 2;
			}
		}

		// Turn off LED after pulse expires
		if (led_off_time && millis() >= led_off_time) {
			led_off();
			led_off_time = 0;
		}

		// Send injected-only reports if no real report was merged this cycle
		kmbox_send_pending();

#if TFT_ENABLED
		// TFT display update at ~30 Hz (33ms intervals)
		if ((millis() - last_tft_update) >= 33) {
			// Compute report rate over the interval
			uint32_t now = millis();
			uint32_t dt = now - prev_report_time;
			if (dt > 0)
				reports_per_sec = ((report_count - prev_report_count) * 1000) / dt;
			prev_report_count = report_count;
			prev_report_time = now;

			tft_proxy_stats_t st = {
				.host_connected    = usb_host_device_connected(),
				.device_configured = usb_device_is_configured(),
				.kmbox_active      = kmbox_frame_count() > 0,
				.protocol_mode     = 0,
				.num_endpoints     = num_ep_mappings,
				.device_speed      = speed,
				.report_count      = report_count,
				.drop_count        = drop_count,
				.reports_per_sec   = reports_per_sec,
				.smooth_active     = smooth_has_pending(),
				.smooth_queue_depth = 0,
				.smooth_queue_max  = SMOOTH_QUEUE_SIZE,
				.inject_count      = 0,
				.kmbox_frames_ok   = kmbox_frame_count(),
				.kmbox_frames_err  = kmbox_error_count(),
				.uart_rx_bytes     = kmbox_rx_byte_count(),
				.uptime_sec        = now / 1000,
			};
			tft_display_update(&st);
			last_tft_update = now;
		}
#endif

		// Disconnect check
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected()) {
				uart_puts("Mouse disconnected!\r\n");
				led_blink_forever(6, 80, 80);
			}
		}
	}
}
