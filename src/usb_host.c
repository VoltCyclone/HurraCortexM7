// usb_host.c — EHCI host on USB2, polled completion, DMA in .dmabuffers
//
// CACHE NOTE: the `asm volatile("dsb")` barriers around qTD arming below are
// sufficient WITHOUT any SCB_CleanInvalidateDCache/arm_dcache_* calls *because*
// all .dmabuffers live in the first 64 KB of OCRAM, which core/startup.c marks
// non-cacheable via MPU region 10. On non-cacheable Normal memory a DSB is all
// that's needed to order descriptor writes before the controller reads them.
// If you ever make that region cacheable, you MUST add explicit cache
// maintenance here or DMA will read stale descriptors. (Linker ASSERT in
// core/imxrt1062_mm.ld guards the 64 KB window.)

#include <string.h>
#include "imxrt.h"
#include "usb_host.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

static ehci_qh_t   qh_async   __attribute__((section(".dmabuffers"), aligned(64)));
static ehci_qtd_t  qtd_setup  __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qtd_t  qtd_data   __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qtd_t  qtd_status __attribute__((section(".dmabuffers"), aligned(32)));
static usb_setup_t setup_buf  __attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t     xfer_buf[2048] __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qh_t   qh_intr[MAX_INTR_EPS]
	__attribute__((section(".dmabuffers"), aligned(64)));
static uint8_t     intr_buf[MAX_INTR_EPS][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
static bool        intr_initialized[MAX_INTR_EPS];
static bool        intr_transfer_active[MAX_INTR_EPS];
static uint32_t    intr_prime_time[MAX_INTR_EPS];
static uint8_t     intr_dev_addr[MAX_INTR_EPS];
static uint8_t     intr_ep_num[MAX_INTR_EPS];
static uint8_t     num_intr_eps = 0;
static ehci_qh_t   qh_intr_out[MAX_INTR_OUT_EPS]
	__attribute__((section(".dmabuffers"), aligned(64)));
static uint8_t     intr_out_buf[MAX_INTR_OUT_EPS][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
static bool        intr_out_initialized[MAX_INTR_OUT_EPS];
static bool        intr_out_transfer_active[MAX_INTR_OUT_EPS];
static uint32_t    intr_out_prime_time[MAX_INTR_OUT_EPS];
static uint8_t     intr_out_dev_addr[MAX_INTR_OUT_EPS];
static uint8_t     intr_out_ep_num[MAX_INTR_OUT_EPS];
static uint8_t     num_intr_out_eps = 0;
static uint32_t periodic_list[32] __attribute__((section(".dmabuffers"), aligned(4096)));

static uint8_t device_speed = USB_SPEED_FULL;

// EHCI qTD/QH buffer-page-cross writes. Length-bounded variants — see
// usb_host.c performance design notes. Both setup_buf and xfer_buf are
// aligned(32) in .dmabuffers, so for any transfer with len <= 64 the
// worst-case reach (0xFE0 + 64 = 0x1040) cannot cross into buffer[2];
// for len <= 2048 the worst-case reach (0xFE0 + 2048 = 0x17E0) cannot
// cross into buffer[3].
static __attribute__((always_inline)) inline void
set_qtd_buffers_small(volatile uint32_t *b, const void *buf)
{
	uint32_t a = (uint32_t)buf;
	b[0] = a;
	b[1] = (a & 0xFFFFF000u) + 0x1000u;
}

static __attribute__((always_inline)) inline void
set_qtd_buffers_medium(volatile uint32_t *b, const void *buf)
{
	uint32_t a = (uint32_t)buf;
	b[0] = a;
	a &= 0xFFFFF000u;
	b[1] = a + 0x1000u;
	b[2] = a + 0x2000u;
}

static void usb2_isr(void)
{
	USB2_USBSTS = USB2_USBSTS; // W1C all pending status bits
	__asm volatile("dsb" ::: "memory"); // flush W1C before exception return
}

static inline void host_power_on(void)
{
	GPIO8_DR_SET = (1u << 26); // GPIO8_26 = GPIO_EMC_40 (Teensy 4.1)
}

static void host_power_init(void)
{
	// Teensy 4.1 USB host VBUS switch control: GPIO_EMC_40 (GPIO8_26)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5; // ALT5 = GPIO8_IO26
	IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008; // weak/slow drive per USBHost_t36
	GPIO8_GDIR |= (1u << 26);
	host_power_on();
}

static void host_led_mark(uint8_t code)
{
	(void)code;
}

bool usb_host_init(void)
{
	const uint32_t timeout_loops = 4000000u;
	host_led_mark(1);
	uint32_t timeout = timeout_loops;
	while (1) {
		uint32_t n = CCM_ANALOG_PLL_USB2;
		if (n & CCM_ANALOG_PLL_USB2_DIV_SELECT) {
			CCM_ANALOG_PLL_USB2_CLR = 0xC000; // clear DIV_SELECT + ENABLE
			CCM_ANALOG_PLL_USB2_SET = CCM_ANALOG_PLL_USB2_BYPASS;
			CCM_ANALOG_PLL_USB2_CLR = CCM_ANALOG_PLL_USB2_POWER |
				CCM_ANALOG_PLL_USB2_DIV_SELECT |
				CCM_ANALOG_PLL_USB2_ENABLE |
				CCM_ANALOG_PLL_USB2_EN_USB_CLKS;
		} else if (!(n & CCM_ANALOG_PLL_USB2_ENABLE)) {
			CCM_ANALOG_PLL_USB2_SET = CCM_ANALOG_PLL_USB2_ENABLE;
		} else if (!(n & CCM_ANALOG_PLL_USB2_POWER)) {
			CCM_ANALOG_PLL_USB2_SET = CCM_ANALOG_PLL_USB2_POWER;
		} else if (!(n & CCM_ANALOG_PLL_USB2_LOCK)) {
		} else if (n & CCM_ANALOG_PLL_USB2_BYPASS) {
			CCM_ANALOG_PLL_USB2_CLR = CCM_ANALOG_PLL_USB2_BYPASS;
		} else if (!(n & CCM_ANALOG_PLL_USB2_EN_USB_CLKS)) {
			CCM_ANALOG_PLL_USB2_SET = CCM_ANALOG_PLL_USB2_EN_USB_CLKS;
		} else {
			break;
		}

		if (--timeout == 0) break;
	}
	CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);
	host_led_mark(2);
	USBPHY2_CTRL_CLR = USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE;
	USBPHY2_CTRL_SET = USBPHY_CTRL_ENUTMILEVEL2 |
		USBPHY_CTRL_ENUTMILEVEL3;
	USBPHY2_PWD = 0; // Power up all PHY sections
	host_power_init();
	delay(25); // Allow VBUS to ramp and stabilize
	USB2_USBCMD |= USB_USBCMD_RST;
	timeout = timeout_loops;
	while (USB2_USBCMD & USB_USBCMD_RST) {
		if (--timeout == 0) break;
	}
	host_led_mark(3);
	USB2_USBMODE = USB_USBMODE_CM(3);
	for (int i = 0; i < 32; i++) {
		periodic_list[i] = 1; // T-bit = 1, terminate
	}
	asm volatile("dsb" ::: "memory");
	memset(&qh_async, 0, sizeof(qh_async));
	qh_async.horizontal_link = (uint32_t)&qh_async | 0x02; // Type=QH, point to self
	qh_async.capabilities[0] = (1 << 15); // Head of reclamation list (H-bit)
	qh_async.next = QTD_TERMINATE;
	qh_async.alt_next = QTD_TERMINATE;
	qh_async.token = 0;
	asm volatile("dsb" ::: "memory");
	USB2_SBUSCFG = 1; // burst-aligned bus access (per PJRC)
	// Enable USB completion interrupt so SEVONPEND wakes WFE
	// immediately on transfer completion (sub-µs vs ~1 ms PIT0 tick).
	// The ISR just clears USBSTS; transfer results come from QH tokens.
	attachInterruptVector(IRQ_USB2, usb2_isr);
	NVIC_SET_PRIORITY(IRQ_USB2, 144); // low priority, below DMA (96)
	NVIC_ENABLE_IRQ(IRQ_USB2);
	USB2_USBINTR = USB_USBINTR_UE; // interrupt on transfer completion
	USB2_PERIODICLISTBASE = (uint32_t)periodic_list;
	USB2_FRINDEX = 0;
	USB2_ASYNCLISTADDR = 0; // No async list yet (per PJRC; set when first transfer)
	USB2_USBCMD = USB_USBCMD_ITC(1) | USB_USBCMD_RS |
		USB_USBCMD_ASP(3) | USB_USBCMD_ASPE |
		USB_USBCMD_PSE |
		USB_USBCMD_FS_2 | USB_USBCMD_FS_1(1);
	USB2_PORTSC1 |= USB_PORTSC1_PP;
	host_led_mark(4);

	return true;
}

void usb_host_power_on(void)
{
	host_power_on();
}

bool usb_host_device_connected(void)
{
	return (USB2_PORTSC1 & USB_PORTSC1_CCS) != 0;
}

uint8_t usb_host_device_speed(void)
{
	return device_speed;
}

void usb_host_port_reset(void)
{
	#define PORTSC_W1C_MASK  (USB_PORTSC1_CSC | (1u<<3) | (1u<<5))

	uint32_t portsc = USB2_PORTSC1;
	portsc &= ~PORTSC_W1C_MASK;  // Don't write-back W1C bits
	USB2_PORTSC1 = (portsc & ~USB_PORTSC1_PE) | USB_PORTSC1_PR;
	delay(50); // USB spec: hold reset for at least 50ms
	portsc = USB2_PORTSC1;
	portsc &= ~PORTSC_W1C_MASK;
	USB2_PORTSC1 = portsc & ~USB_PORTSC1_PR;
	uint32_t timeout = millis() + 500;
	while (!(USB2_PORTSC1 & USB_PORTSC1_PE)) {
		if (millis() > timeout) return;
	}
	portsc = USB2_PORTSC1;
	uint32_t pspd = (portsc >> 26) & 3;
	if (pspd == 0) device_speed = USB_SPEED_FULL;
	else if (pspd == 1) device_speed = USB_SPEED_LOW;
	else if (pspd == 2) device_speed = USB_SPEED_HIGH;
	USB2_PORTSC1 |= USB_PORTSC1_CSC;
	delay(10); // Recovery time after reset
}

static void setup_qh_for_control(uint8_t addr, uint8_t maxpkt, uint8_t speed)
{
	memset(&qh_async, 0, sizeof(qh_async));
	qh_async.horizontal_link = (uint32_t)&qh_async | 0x02;

	uint32_t cap0 = 0;
	cap0 |= (15 << 28);
	if (speed != USB_SPEED_HIGH)
		cap0 |= (1 << 27);
	cap0 |= ((uint32_t)maxpkt << 16);
	cap0 |= (1 << 15);
	cap0 |= (1 << 14);
	cap0 |= ((uint32_t)speed << 12);
	cap0 |= (0 << 8);
	cap0 |= addr;

	qh_async.capabilities[0] = cap0;

	qh_async.capabilities[1] = (1 << 30);

	qh_async.next = QTD_TERMINATE;
	qh_async.alt_next = QTD_TERMINATE;
	qh_async.token = 0;

	asm volatile("dsb" ::: "memory");
}

static int execute_transfer(uint32_t timeout_ms)
{
	USB2_ASYNCLISTADDR = (uint32_t)&qh_async;
	USB2_USBSTS = USB2_USBSTS;

	if (!(USB2_USBCMD & USB_USBCMD_ASE)) {
		USB2_USBCMD |= USB_USBCMD_ASE;
	}

	uint32_t start = millis();
	while (1) {
		uint32_t qh_token = qh_async.token;
		if (qh_token & QTD_TOKEN_HALTED) {
			return -1;
		}
		uint32_t st_token = qtd_status.token;
		if (!(st_token & QTD_TOKEN_ACTIVE)) {
			if (st_token & (QTD_TOKEN_HALTED | QTD_TOKEN_BUFERR |
				QTD_TOKEN_BABBLE | QTD_TOKEN_XACTERR)) {
				return -1;
			}
			return 0; // Success — all qTDs completed
		}

		if ((millis() - start) > timeout_ms) {
			return -1;
		}

		// Sleep until USB completion ISR (or any other event) wakes us.
		// SEVONPEND + USBINTR_UE means a completed transfer raises SEV
		// without re-entering the ISR; the qTD tokens are checked above.
		__asm volatile("wfe");
	}
}

int usb_host_control_transfer(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data, uint32_t timeout_ms)
{
	uint16_t wLength = setup->wLength;
	bool is_in = (setup->bmRequestType & 0x80) != 0;
	memcpy(&setup_buf, setup, 8);
	setup_qh_for_control(addr, maxpkt, device_speed);
	memset(&qtd_setup, 0, sizeof(qtd_setup));
	qtd_setup.alt_next = QTD_TERMINATE;
	qtd_setup.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_SETUP |
		QTD_TOKEN_NBYTES(8) | QTD_TOKEN_CERR(3);
	set_qtd_buffers_small(qtd_setup.buffer, &setup_buf);

	if (wLength > 0) {
		memset(&qtd_data, 0, sizeof(qtd_data));
		qtd_data.alt_next = QTD_TERMINATE;
		qtd_data.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(wLength) | QTD_TOKEN_CERR(3) |
			(is_in ? QTD_TOKEN_PID_IN : QTD_TOKEN_PID_OUT);
		if (is_in) {
			memset(xfer_buf, 0, wLength);
		} else {
			memcpy(xfer_buf, data, wLength);
		}
		set_qtd_buffers_medium(qtd_data.buffer, xfer_buf);

		memset(&qtd_status, 0, sizeof(qtd_status));
		qtd_status.next = QTD_TERMINATE;
		qtd_status.alt_next = QTD_TERMINATE;
		qtd_status.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(0) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC |
			(is_in ? QTD_TOKEN_PID_OUT : QTD_TOKEN_PID_IN);

		qtd_setup.next = (uint32_t)&qtd_data;
		qtd_data.next = (uint32_t)&qtd_status;
	} else {
		memset(&qtd_status, 0, sizeof(qtd_status));
		qtd_status.next = QTD_TERMINATE;
		qtd_status.alt_next = QTD_TERMINATE;
		qtd_status.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(0) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC |
			QTD_TOKEN_PID_IN;
		qtd_setup.next = (uint32_t)&qtd_status;
	}
	asm volatile("dsb" ::: "memory");
	qh_async.next = (uint32_t)&qtd_setup;
	qh_async.token = 0; // Clear any previous status
	asm volatile("dsb" ::: "memory");
	int result = execute_transfer(timeout_ms);
	if (result < 0) return -1;
	if (is_in && wLength > 0) {
		uint32_t remaining = (qtd_data.token >> 16) & 0x7FFF;
		uint32_t transferred = wLength - remaining;
		memcpy(data, xfer_buf, transferred);
		return (int)transferred;
	}

	return 0;
}

void usb_host_control_transfer_fire(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data)
{
	uint16_t wLength = setup->wLength;
	memcpy(&setup_buf, setup, 8);
	setup_qh_for_control(addr, maxpkt, device_speed);

	memset(&qtd_setup, 0, sizeof(qtd_setup));
	qtd_setup.alt_next = QTD_TERMINATE;
	qtd_setup.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_SETUP |
		QTD_TOKEN_NBYTES(8) | QTD_TOKEN_CERR(3);
	set_qtd_buffers_small(qtd_setup.buffer, &setup_buf);

	if (wLength > 0 && data) {
		memset(&qtd_data, 0, sizeof(qtd_data));
		qtd_data.alt_next = QTD_TERMINATE;
		qtd_data.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(wLength) | QTD_TOKEN_CERR(3) |
			QTD_TOKEN_PID_OUT;
		memcpy(xfer_buf, data, wLength);
		set_qtd_buffers_medium(qtd_data.buffer, xfer_buf);

		memset(&qtd_status, 0, sizeof(qtd_status));
		qtd_status.next = QTD_TERMINATE;
		qtd_status.alt_next = QTD_TERMINATE;
		qtd_status.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(0) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC |
			QTD_TOKEN_PID_IN;

		qtd_setup.next = (uint32_t)&qtd_data;
		qtd_data.next = (uint32_t)&qtd_status;
	} else {
		memset(&qtd_status, 0, sizeof(qtd_status));
		qtd_status.next = QTD_TERMINATE;
		qtd_status.alt_next = QTD_TERMINATE;
		qtd_status.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(0) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC |
			QTD_TOKEN_PID_IN;
		qtd_setup.next = (uint32_t)&qtd_status;
	}

	asm volatile("dsb" ::: "memory");
	qh_async.next = (uint32_t)&qtd_setup;
	qh_async.token = 0;
	asm volatile("dsb" ::: "memory");

	// Kick the async schedule — hardware takes it from here
	USB2_ASYNCLISTADDR = (uint32_t)&qh_async;
	USB2_USBSTS = USB2_USBSTS;
	if (!(USB2_USBCMD & USB_USBCMD_ASE))
		USB2_USBCMD |= USB_USBCMD_ASE;
}

bool usb_host_control_async_busy(void)
{
	return (qtd_status.token & QTD_TOKEN_ACTIVE) != 0;
}

static void link_periodic_schedule(void)
{
	// Find the first initialized OUT slot — used as the "next" pointer
	// when an IN slot is the last initialized IN, but OUT slots exist.
	uint32_t first_out_link = 0x01; // T-bit by default
	for (uint8_t i = 0; i < num_intr_out_eps; i++) {
		if (intr_out_initialized[i]) {
			first_out_link = (uint32_t)&qh_intr_out[i] | 0x02;
			break;
		}
	}

	// Chain IN slots — each points to the next initialized IN, or to the
	// first initialized OUT if no more IN, or terminates.
	for (uint8_t i = 0; i < num_intr_eps; i++) {
		if (!intr_initialized[i]) continue;
		uint32_t next_link = first_out_link;
		for (uint8_t j = i + 1; j < num_intr_eps; j++) {
			if (intr_initialized[j]) {
				next_link = (uint32_t)&qh_intr[j] | 0x02; // type=QH
				break;
			}
		}
		qh_intr[i].horizontal_link = next_link;
	}

	// Chain OUT slots — each points to the next initialized OUT or terminates.
	for (uint8_t i = 0; i < num_intr_out_eps; i++) {
		if (!intr_out_initialized[i]) continue;
		uint32_t next_link = 0x01; // T-bit: terminate
		for (uint8_t j = i + 1; j < num_intr_out_eps; j++) {
			if (intr_out_initialized[j]) {
				next_link = (uint32_t)&qh_intr_out[j] | 0x02;
				break;
			}
		}
		qh_intr_out[i].horizontal_link = next_link;
	}

	// Chain head: first IN if any, else first OUT if any, else terminate.
	uint32_t head = 0x01;
	for (uint8_t i = 0; i < num_intr_eps; i++) {
		if (intr_initialized[i]) {
			head = (uint32_t)&qh_intr[i] | 0x02;
			break;
		}
	}
	if (head == 0x01) {
		head = first_out_link;
	}

	for (int i = 0; i < 32; i++) {
		periodic_list[i] = head;
	}
	asm volatile("dsb" ::: "memory");
}

static uint32_t intr_halt_count[MAX_INTR_EPS];
static uint32_t intr_timeout_count[MAX_INTR_EPS];
static uint32_t intr_error_count[MAX_INTR_EPS];
// OUT-side diagnostics — only timeout is currently observable since out_send
// detects only the ACTIVE-past-100ms case; halt/error decoding for completed
// OUT QTDs would require checking before overwriting the token next call.
static uint32_t intr_out_timeout_count[MAX_INTR_OUT_EPS];
static uint32_t intr_poll_debug_count;

// Send CLEAR_FEATURE(ENDPOINT_HALT) via fire-and-forget to clear a stalled EP.
// Uses the async schedule; caller should check usb_host_control_async_busy() first.
static usb_setup_t clear_halt_setup __attribute__((section(".dmabuffers"), aligned(32)));

static void intr_clear_halt(uint8_t index)
{
	if (usb_host_control_async_busy()) return; // don't clobber in-flight transfer
	clear_halt_setup.bmRequestType = 0x02; // host-to-device, standard, endpoint
	clear_halt_setup.bRequest = 1;         // CLEAR_FEATURE
	clear_halt_setup.wValue = 0;           // ENDPOINT_HALT
	clear_halt_setup.wIndex = 0x80 | intr_ep_num[index]; // IN endpoint
	clear_halt_setup.wLength = 0;
	usb_host_control_transfer_fire(intr_dev_addr[index], 64,
		&clear_halt_setup, NULL);
}

// Recover a halted periodic QH: disable S-mask to prevent HC access,
// clear halt, re-prime transfer, then restore S-mask.
__attribute__((cold, noinline))
static void intr_halt_recover(uint8_t index, uint16_t len)
{
	ehci_qh_t *qh = &qh_intr[index];
	uint8_t *buf = intr_buf[index];

	intr_halt_count[index]++;

	// Disable S-mask so HC won't touch this QH during re-prime
	uint32_t saved_cap1 = qh->capabilities[1];
	qh->capabilities[1] = saved_cap1 & ~0xFFu; // zero S-mask
	asm volatile("dsb" ::: "memory");

	// Clear halt and re-prime
	uint32_t toggle = qh->token & QTD_TOKEN_TOGGLE;
	qh->next     = QTD_TERMINATE;
	qh->alt_next = QTD_TERMINATE;
	qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
		QTD_TOKEN_NBYTES(len) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC;
	set_qtd_buffers_small(qh->buffer, buf);
	asm volatile("dsb" ::: "memory");

	// Restore S-mask to re-enable periodic processing
	qh->capabilities[1] = saved_cap1;
	asm volatile("dsb" ::: "memory");

	intr_transfer_active[index] = true;
	intr_prime_time[index] = millis();

	// Tell the device to clear its STALL condition
	intr_clear_halt(index);
}

void usb_host_interrupt_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt)
{
	if (index >= MAX_INTR_EPS) return;

	ehci_qh_t *qh = &qh_intr[index];
	memset(qh, 0, sizeof(*qh));
	uint32_t cap0 = 0;
	cap0 |= (0 << 28);               // NAK reload = 0 (not used for periodic)
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

	intr_initialized[index] = true;
	intr_transfer_active[index] = false;
	intr_dev_addr[index] = addr;
	intr_ep_num[index] = ep & 0x0F;
	intr_halt_count[index] = 0;
	intr_timeout_count[index] = 0;
	intr_error_count[index] = 0;

	if (index >= num_intr_eps)
		num_intr_eps = index + 1;

	link_periodic_schedule();
}

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
	intr_out_timeout_count[index]   = 0;

	if (index >= num_intr_out_eps)
		num_intr_out_eps = index + 1;

	asm volatile("dsb" ::: "memory");
	link_periodic_schedule();  // Both IN and OUT QHs will share the periodic frame list
}

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
			intr_out_timeout_count[index]++;
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
	set_qtd_buffers_small(qh->buffer, buf);
	asm volatile("dsb" ::: "memory");

	intr_out_transfer_active[index] = true;
	intr_out_prime_time[index]      = millis();
	return true;
}

void usb_host_interrupt_dump_state(void)
{
}

// Internal poll: primes QH, checks completion, handles halt/error/timeout.
// On success returns bytes transferred and sets *buf_out to the DMA buffer.
// Returns 0 when not yet complete, -1 on error.
__attribute__((section(".fastrun")))
static int intr_poll_internal(uint8_t index, uint16_t len, uint8_t **buf_out)
{
	ehci_qh_t *qh  = &qh_intr[index];
	uint8_t   *buf = intr_buf[index];

	if (__builtin_expect(!intr_transfer_active[index], 0)) {
		uint32_t toggle = qh->token & QTD_TOKEN_TOGGLE;
		qh->next     = QTD_TERMINATE;
		qh->alt_next = QTD_TERMINATE;
		qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
			QTD_TOKEN_NBYTES(len) | QTD_TOKEN_CERR(3) | QTD_TOKEN_IOC;
		set_qtd_buffers_small(qh->buffer, buf);
		asm volatile("dsb" ::: "memory");
		intr_transfer_active[index] = true;
		intr_prime_time[index] = millis();
		return 0;
	}

	uint32_t token = qh->token;

	if (__builtin_expect(!!(token & QTD_TOKEN_ACTIVE), 1)) {
		if (__builtin_expect((millis() - intr_prime_time[index]) > 100, 0)) {
			qh->token = token & QTD_TOKEN_TOGGLE;
			qh->next = QTD_TERMINATE;
			asm volatile("dsb" ::: "memory");
			intr_transfer_active[index] = false;
			intr_timeout_count[index]++;
			return -1;
		}
		return 0;
	}

	intr_transfer_active[index] = false;

	if (__builtin_expect(!!(token & QTD_TOKEN_HALTED), 0)) {
		intr_halt_recover(index, len);
		return 0;
	}

	if (__builtin_expect(!!(token & (QTD_TOKEN_BUFERR | QTD_TOKEN_BABBLE | QTD_TOKEN_XACTERR)), 0)) {
		intr_error_count[index]++;
		qh->token = token & QTD_TOKEN_TOGGLE;
		qh->next = QTD_TERMINATE;
		asm volatile("dsb" ::: "memory");
		return -1;
	}

	uint32_t remaining = (token >> 16) & 0x7FFF;
	uint32_t transferred = len - remaining;
	if (transferred > 0)
		*buf_out = buf;
	return (int)transferred;
}

int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len)
{
	if (index >= MAX_INTR_EPS || !intr_initialized[index]) return -1;
	uint8_t *buf_ptr = NULL;
	int ret = intr_poll_internal(index, len, &buf_ptr);
	if (ret > 0 && buf_ptr)
		memcpy(data, buf_ptr, ret);
	return ret;
}

__attribute__((section(".fastrun")))
int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len)
{
	if (index >= MAX_INTR_EPS || !intr_initialized[index]) return -1;
	return intr_poll_internal(index, len, data_ptr);
}
