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
#include "gpt_profile.h"
#if TFT_ENABLED
#include "tft_display.h"
#endif
#if NET_ENABLED
#include "kmnet.h"
#include "udp.h"
#include "enet.h"
#endif
#if TOUCH_ENABLED
#include "ft6206.h"
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
static void led_pwm_init(void) { }
static void led_pwm_set(uint8_t brightness) { (void)brightness; }
#else
// ---- FlexPWM2 channel A02 on pin 13 (GPIO_B0_03 ALT1) ----
// Hardware PWM for LED brightness: zero CPU overhead after setup.
// PWM frequency ~1kHz at IPG/128 prescaler.
static bool led_pwm_active;

static void led_pwm_init(void)
{
	// Enable FlexPWM2 clock
	CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);

	// Submodule 2: center-aligned PWM, IPG clock with /128 prescaler
	// IPG = 204MHz/128 = ~1.59MHz, period=1594 -> ~1kHz PWM
	FLEXPWM2_SM2CTRL2 = 0;
	FLEXPWM2_SM2CTRL  = (7 << 4); // PRSC = /128
	FLEXPWM2_SM2INIT  = 0;
	FLEXPWM2_SM2VAL0  = 0;
	FLEXPWM2_SM2VAL1  = 1594;  // period
	FLEXPWM2_SM2VAL2  = 0;     // PWMA rising edge
	FLEXPWM2_SM2VAL3  = 0;     // PWMA falling edge (duty=0 initially)
	FLEXPWM2_SM2DISMAP0 = 0xF000; // no fault disable
	FLEXPWM2_SM2DISMAP1 = 0xF000;

	// Load registers, run submodule 2
	FLEXPWM2_MCTRL |= (1 << 6);  // LDOK submodule 2
	FLEXPWM2_MCTRL |= (1 << 10); // RUN submodule 2
	FLEXPWM2_OUTEN |= (1 << 10); // PWMA_EN submodule 2

	// Mux pin 13 (GPIO_B0_03) to ALT1 = FLEXPWM2_PWMA02
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 1;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SPEED(2);

	led_pwm_active = true;
}

// Set LED brightness 0-255 via PWM duty cycle (zero CPU after register write)
static void led_pwm_set(uint8_t brightness)
{
	if (!led_pwm_active) return;
	uint16_t duty = ((uint32_t)brightness * 1594) >> 8;
	FLEXPWM2_SM2VAL3 = duty;
	FLEXPWM2_MCTRL |= (1 << 6); // LDOK to latch new VAL3
}

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

// ---- PIT0: smooth injection timing with humanized jitter ----
static volatile bool pit_tick_pending;
static volatile bool pit_tick_skip;      // true = suppress this frame (missed poll)
static uint32_t pit_base_ldval;          // nominal reload value (set after enumeration)

static void pit0_isr(void)
{
	PIT_TFLG0 = PIT_TFLG_TIF;

	// Compute next interval + skip decision (all integer, ~12 cycles)
	bool skip = false;
	uint32_t next_ldval = smooth_timing_next(pit_base_ldval, &skip);
	PIT_LDVAL0 = next_ldval;

	pit_tick_skip = skip;
	pit_tick_pending = true;
}

// Static to keep off the stack (~3.6KB struct)
static captured_descriptors_t desc;

// ---- TEMPMON: CPU die temperature ----
// Calibration from OCOTP fuses (read once at init)
static uint32_t room_count, hot_count;
static int32_t  hot_temp;

static void tempmon_init(void)
{
	// Enable OCOTP clock for fuse reads
	CCM_CCGR2 |= CCM_CCGR2_OCOTP_CTRL(CCM_CCGR_ON);

	// Read factory calibration from ANA1 fuse:
	// bits [31:20] = hot_count (12-bit), bits [19:12] = hot_temp (8-bit)
	uint32_t ana1 = HW_OCOTP_ANA1;
	hot_count  = (ana1 >> 20) & 0xFFF;
	hot_temp   = (int32_t)((ana1 >> 12) & 0xFF);
	// room_count is stored as offset from hot_count in some revisions;
	// ANA2 has the actual room count on IMXRT1062
	uint32_t ana2 = HW_OCOTP_ANA2;
	room_count = (ana2 >> 20) & 0xFFF;
	if (room_count == 0) room_count = hot_count - 35; // fallback

	// Power up TEMPMON, enable periodic measurement
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

// ---- USB descriptor helpers for display ----
static uint16_t desc_vid(void)
{
	return (uint16_t)desc.device_desc[8] | ((uint16_t)desc.device_desc[9] << 8);
}

static uint16_t desc_pid(void)
{
	return (uint16_t)desc.device_desc[10] | ((uint16_t)desc.device_desc[11] << 8);
}

// Decode USB string descriptor (UTF-16LE → ASCII, truncate to outlen-1 chars)
static void desc_product_string(char *out, int outlen)
{
	out[0] = '\0';
	// iProduct string index is device_desc[15]
	uint8_t target_idx = desc.device_desc[15];
	if (target_idx == 0) return;

	// Find this string index in the captured strings
	for (uint8_t i = 0; i < desc.num_strings; i++) {
		if (desc.string_index[i] == target_idx && desc.string_desc_len[i] > 2) {
			const uint8_t *sd = desc.string_desc[i];
			int slen = (desc.string_desc_len[i] - 2) / 2; // UTF-16 char count
			if (slen > outlen - 1) slen = outlen - 1;
			for (int j = 0; j < slen; j++) {
				uint16_t ch = sd[2 + j * 2] | (sd[3 + j * 2] << 8);
				out[j] = (ch > 0x7E || ch < 0x20) ? '?' : (char)ch;
			}
			out[slen] = '\0';
			return;
		}
	}
}

int main(void)
{
	// Enable SEVONPEND so WFE wakes on any pending interrupt
	SCB_SCR |= SCB_SCR_SEVONPEND;

	uart_init();
#if NET_ENABLED
	kmnet_init();
#endif
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

	uart_puts("\r\n\r\nimxrtnsy: USB proxy\r\n");
	uart_puts("Teensy 4.1 — i.MX RT1062\r\n\r\n");
	led_on();

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

	// Cache USB descriptor info for display
	uint16_t usb_vid = desc_vid();
	uint16_t usb_pid = desc_pid();
	char usb_product[22];
	desc_product_string(usb_product, sizeof(usb_product));

	usb_setup_t setup;
	setup.bmRequestType = 0x00;
	setup.bRequest = USB_REQ_SET_CONFIG;
	setup.wValue = desc.config_desc[5]; // bConfigurationValue
	setup.wIndex = 0;
	setup.wLength = 0;
	int ret = usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
		&setup, NULL, 2000);
	if (ret < 0) {
		uart_puts("  SET_CONFIGURATION failed!\r\n");
	}
	delay(10);

	for (uint8_t i = 0; i < desc.num_ifaces; i++) {
		// SET_IDLE: bmRequestType=0x21 (class, interface), bRequest=0x0A
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0A; // SET_IDLE
		setup.wValue = 0;      // duration=0 (indefinite), report ID=0 (all)
		setup.wIndex = i;      // interface number
		setup.wLength = 0;
		usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL, 2000);

		// SET_PROTOCOL: bmRequestType=0x21, bRequest=0x0B
		setup.bmRequestType = 0x21;
		setup.bRequest = 0x0B; // SET_PROTOCOL
		setup.wValue = 1;      // 1 = Report Protocol (not Boot Protocol)
		setup.wIndex = i;
		setup.wLength = 0;
		usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt,
			&setup, NULL, 2000);
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
		uint32_t ipg_mhz = (F_CPU / 4u) / 1000000u; // IPG = ARM / 4
		uint32_t ldval = (ipg_mhz * interval_us) - 1;
		pit_base_ldval = ldval;
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
		if ((millis() - dev_led_toggle) >= 250) {
			led_toggle();
			dev_led_toggle = millis();
		}

		if ((millis() - dev_wait_start) > 30000) {
			led_blink_forever(8, 80, 120);
		}
	}
	led_off();
	led_pwm_init(); // Switch pin 13 to FlexPWM (no-op when TFT active)
#if TFT_ENABLED
	tft_display_init();
#endif
#if TOUCH_ENABLED
	ft6206_init();
#endif
	uart_puts("\r\n=== PROXY ACTIVE (");
	uart_putdec(num_ep_mappings);
	uart_puts(" endpoints) ===\r\n");

	uint8_t report_buf[64];
	uint32_t report_count = 0;
	uint32_t drop_count = 0;
	uint32_t loop_count = 0;
	uint32_t last_heartbeat = millis();
	uint32_t led_off_time = 0; // non-blocking LED pulse
	uint32_t led_pwm_update = millis();
	uint32_t led_report_snapshot = 0;

#if TFT_ENABLED
	uint32_t last_tft_update = millis();
	uint32_t prev_report_count = 0;
	uint32_t prev_report_time = millis();
	uint32_t reports_per_sec = 0;
#endif

	while (1) {
		usb_device_poll();
#if NET_ENABLED
		kmnet_poll();
#else
		kmbox_poll();
#endif
		bool did_work = false;
		if (pit_tick_pending) {
			pit_tick_pending = false;
			did_work = true;
			if (!pit_tick_skip) {
				int16_t sx, sy;
				smooth_process_frame(&sx, &sy);
				if (sx || sy) kmbox_inject_smooth(sx, sy);
			}
			// When skip=true, no report this tick — mimics missed USB poll.
			// Accumulated sub-pixel state preserved; next frame catches up.
		}

		for (uint8_t m = 0; m < num_ep_mappings; m++) {
			uint8_t *rpt_ptr = NULL;
			ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot,
				&rpt_ptr, ep_map[m].maxpkt);
			if (ret > 0 && rpt_ptr) {
				did_work = true;
				// Merge directly into DMA buffer — eliminates one memcpy
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
				led_off_time = millis() + 2;
			}
		}
		if (led_off_time && millis() >= led_off_time) {
			led_off();
			led_off_time = 0;
		}
		kmbox_send_pending();

		// Sleep until next event when idle — PIT ISR, USB completion,
		// or UART DMA will set the event flag via SEVONPEND.
		// Reduces power draw and frees bus bandwidth for DMA.
		if (!did_work)
			__asm volatile("wfe");

#if TFT_ENABLED
		if ((millis() - last_tft_update) >= 33) {
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
				.cpu_temp_c        = tempmon_read(),
				.usb_vid           = usb_vid,
				.usb_pid           = usb_pid,
#if NET_ENABLED
				.net_connected     = kmnet_client_connected(),
				.net_link_up       = kmnet_link_up(),
				.net_ip            = udp_get_ip(),
				.net_port          = kmnet_get_port(),
				.net_uuid          = kmnet_get_uuid(),
				.net_rx_count      = kmnet_rx_count(),
				.net_tx_count      = kmnet_tx_count(),
#endif
			};
			__builtin_memcpy(st.usb_product, usb_product, sizeof(usb_product));
			tft_display_update(&st);
			last_tft_update = now;

#if TOUCH_ENABLED
			touch_point_t tp;
			if (ft6206_poll(&tp) && tp.valid) {
				if (tft_display_touch(tp.x, tp.y)) {
					const tft_settings_t *cfg = tft_display_get_settings();
					smooth_set_max_per_frame(cfg->smooth_max);
				}
			}
#endif
		}
#endif
		// PWM LED brightness: maps reports/sec to 0-255.
		// ~1000 rpt/s = full brightness, 0 = off. Updated every 100ms.
		if ((millis() - led_pwm_update) >= 100) {
			uint32_t delta = report_count - led_report_snapshot;
			led_report_snapshot = report_count;
			// delta per 100ms -> rpt/s = delta * 10, map to 0-255
			uint32_t brightness = delta * 10 * 255 / 1000;
			if (brightness > 255) brightness = 255;
			led_pwm_set((uint8_t)brightness);
			led_pwm_update = millis();
		}
		if ((++loop_count & 0x3FF) == 0) {
			if (!usb_host_device_connected()) {
				uart_puts("Mouse disconnected!\r\n");
				led_blink_forever(6, 80, 80);
			}
		}
	}
}
