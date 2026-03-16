// EHCI USB Host controller driver for Teensy 4.1 USB2 port
// Bare-metal, polled transfer completion (QH token checks).
// A minimal USB2 ISR clears USBSTS so SEVONPEND can wake WFE
// on transfer completion, giving sub-µs wakeup instead of waiting
// for the next PIT0 tick (~1 ms).
//
// DMA buffers are in a non-cacheable MPU region (.dmabuffers at 0x20200000).
// No manual cache maintenance needed — bare DSBs drain the write buffer
// before the USB controller is told to read DMA structures.

#include <string.h>
#include "imxrt.h"
#include "usb_host.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

// Statically allocated EHCI structures in DMA-capable RAM
static ehci_qh_t   qh_async   __attribute__((section(".dmabuffers"), aligned(64)));
static ehci_qtd_t  qtd_setup  __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qtd_t  qtd_data   __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qtd_t  qtd_status __attribute__((section(".dmabuffers"), aligned(32)));
static usb_setup_t setup_buf  __attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t     xfer_buf[2048] __attribute__((section(".dmabuffers"), aligned(32)));
static ehci_qh_t   qh_intr[MAX_INTR_EPS]
	__attribute__((section(".dmabuffers"), aligned(64)));
static ehci_qtd_t  qtd_intr[MAX_INTR_EPS]
	__attribute__((section(".dmabuffers"), aligned(32)));
static uint8_t     intr_buf[MAX_INTR_EPS][64]
	__attribute__((section(".dmabuffers"), aligned(32)));
static bool        intr_initialized[MAX_INTR_EPS];
static bool        intr_transfer_active[MAX_INTR_EPS];
static uint32_t    intr_prime_time[MAX_INTR_EPS];
static uint8_t     num_intr_eps = 0;
static uint32_t periodic_list[32] __attribute__((section(".dmabuffers"), aligned(4096)));

static uint8_t device_speed = USB_SPEED_FULL;

// Minimal USB2 ISR — clears USBSTS so the interrupt deasserts.
// The NVIC pending → event path (SEVONPEND) wakes WFE in the main loop.
// Transfer results are consumed by polling QH/qTD tokens, not USBSTS.
static void usb2_isr(void)
{
	USB2_USBSTS = USB2_USBSTS; // W1C all pending status bits
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

// Set up QH for a control endpoint on a given device address
static void setup_qh_for_control(uint8_t addr, uint8_t maxpkt, uint8_t speed)
{
	memset(&qh_async, 0, sizeof(qh_async));
	qh_async.horizontal_link = (uint32_t)&qh_async | 0x02; // QH type, point to self

	// Capabilities word 0:
	// NAK reload = 15, max packet size, head flag, data toggle from qTD,
	// speed, endpoint 0, address
	uint32_t cap0 = 0;
	cap0 |= (15 << 28);            // NAK reload count
	if (speed != USB_SPEED_HIGH) {
		cap0 |= (1 << 27);         // C-bit: control endpoint flag (required for FS/LS)
	}
	cap0 |= ((uint32_t)maxpkt << 16); // max packet length
	cap0 |= (1 << 15);             // H-bit (head of reclamation)
	cap0 |= (1 << 14);             // DTC - data toggle comes from qTD
	cap0 |= ((uint32_t)speed << 12); // EPS - endpoint speed
	cap0 |= (0 << 8);              // endpoint 0
	cap0 |= addr;                  // device address

	qh_async.capabilities[0] = cap0;

	// Capabilities word 1: FrameTag mult = 1 (one transaction per micro-frame)
	qh_async.capabilities[1] = (1 << 30); // Mult = 1

	qh_async.next = QTD_TERMINATE;
	qh_async.alt_next = QTD_TERMINATE;
	qh_async.token = 0;

	asm volatile("dsb" ::: "memory");
}

// Execute the qTD chain currently linked to the async QH.
// The chain always ends with qtd_status (which has Active=1 initially).
// Returns 0 on success, -1 on error.
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
	{
		uint32_t addr = (uint32_t)&setup_buf;
		qtd_setup.buffer[0] = addr;
		addr &= 0xFFFFF000;
		qtd_setup.buffer[1] = addr + 0x1000;
		qtd_setup.buffer[2] = addr + 0x2000;
		qtd_setup.buffer[3] = addr + 0x3000;
		qtd_setup.buffer[4] = addr + 0x4000;
	}

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
		{
			uint32_t addr = (uint32_t)xfer_buf;
			qtd_data.buffer[0] = addr;
			addr &= 0xFFFFF000;
			qtd_data.buffer[1] = addr + 0x1000;
			qtd_data.buffer[2] = addr + 0x2000;
			qtd_data.buffer[3] = addr + 0x3000;
			qtd_data.buffer[4] = addr + 0x4000;
		}

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

// Fire-and-forget control OUT transfer — sets up DMA, kicks hardware, returns immediately.
// Result is ignored; hardware completes (or STALLs) on its own.
// Caller must check usb_host_control_async_busy() to avoid clobbering an in-flight transfer.
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
	{
		uint32_t a = (uint32_t)&setup_buf;
		qtd_setup.buffer[0] = a;
		a &= 0xFFFFF000;
		qtd_setup.buffer[1] = a + 0x1000;
	}

	if (wLength > 0 && data) {
		memset(&qtd_data, 0, sizeof(qtd_data));
		qtd_data.alt_next = QTD_TERMINATE;
		qtd_data.token = QTD_TOKEN_ACTIVE | QTD_TOKEN_TOGGLE |
			QTD_TOKEN_NBYTES(wLength) | QTD_TOKEN_CERR(3) |
			QTD_TOKEN_PID_OUT;
		memcpy(xfer_buf, data, wLength);
		{
			uint32_t a = (uint32_t)xfer_buf;
			qtd_data.buffer[0] = a;
			a &= 0xFFFFF000;
			qtd_data.buffer[1] = a + 0x1000;
		}

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
	for (uint8_t i = 0; i < num_intr_eps; i++) {
		if (!intr_initialized[i]) continue;
		uint32_t next_link = 0x01; // T-bit: terminate
		for (uint8_t j = i + 1; j < num_intr_eps; j++) {
			if (intr_initialized[j]) {
				next_link = (uint32_t)&qh_intr[j] | 0x02; // type=QH
				break;
			}
		}
		qh_intr[i].horizontal_link = next_link;
	}
	uint32_t head = 0x01; // T-bit if none
	for (uint8_t i = 0; i < num_intr_eps; i++) {
		if (intr_initialized[i]) {
			head = (uint32_t)&qh_intr[i] | 0x02; // type=QH
			break;
		}
	}

	for (int i = 0; i < 32; i++) {
		periodic_list[i] = head;
	}
	asm volatile("dsb" ::: "memory");
}

static uint32_t intr_halt_count[MAX_INTR_EPS];
static uint32_t intr_timeout_count[MAX_INTR_EPS];
static uint32_t intr_error_count[MAX_INTR_EPS];
static uint32_t intr_poll_debug_count;

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
	intr_halt_count[index] = 0;
	intr_timeout_count[index] = 0;
	intr_error_count[index] = 0;

	if (index >= num_intr_eps)
		num_intr_eps = index + 1;

	link_periodic_schedule();
}

void usb_host_interrupt_dump_state(void)
{
}

int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len)
{
	if (index >= MAX_INTR_EPS || !intr_initialized[index]) return -1;

	ehci_qh_t *qh  = &qh_intr[index];
	uint8_t   *buf = intr_buf[index];

	if (!intr_transfer_active[index]) {
		uint32_t toggle = qh->token & QTD_TOKEN_TOGGLE;

		qh->next     = QTD_TERMINATE;
		qh->alt_next = QTD_TERMINATE;
		qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
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

		intr_transfer_active[index] = true;
		intr_prime_time[index] = millis();
		return 0;
	}

	uint32_t token = qh->token;

	if (token & QTD_TOKEN_ACTIVE) {
		if ((millis() - intr_prime_time[index]) > 100) {
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

	if (token & QTD_TOKEN_HALTED) {
		// Re-prime immediately instead of wasting a poll cycle
		intr_halt_count[index]++;
		uint32_t toggle = token & QTD_TOKEN_TOGGLE;
		qh->next     = QTD_TERMINATE;
		qh->alt_next = QTD_TERMINATE;
		qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
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
		intr_transfer_active[index] = true;
		intr_prime_time[index] = millis();
		return 0;
	}

	if (token & (QTD_TOKEN_BUFERR | QTD_TOKEN_BABBLE | QTD_TOKEN_XACTERR)) {
		intr_error_count[index]++;
		qh->token = token & QTD_TOKEN_TOGGLE;
		qh->next = QTD_TERMINATE;
		asm volatile("dsb" ::: "memory");
		return -1;
	}
	uint32_t remaining = (token >> 16) & 0x7FFF;
	uint32_t transferred = len - remaining;
	if (transferred > 0) {
		memcpy(data, buf, transferred);
	}
	return (int)transferred;
}

int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len)
{
	if (index >= MAX_INTR_EPS || !intr_initialized[index]) return -1;

	ehci_qh_t *qh  = &qh_intr[index];
	uint8_t   *buf = intr_buf[index];

	if (!intr_transfer_active[index]) {
		uint32_t toggle = qh->token & QTD_TOKEN_TOGGLE;
		qh->next     = QTD_TERMINATE;
		qh->alt_next = QTD_TERMINATE;
		qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
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
		intr_transfer_active[index] = true;
		intr_prime_time[index] = millis();
		return 0;
	}

	uint32_t token = qh->token;

	if (token & QTD_TOKEN_ACTIVE) {
		if ((millis() - intr_prime_time[index]) > 100) {
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

	if (token & QTD_TOKEN_HALTED) {
		// Re-prime immediately instead of wasting a poll cycle
		intr_halt_count[index]++;
		uint32_t toggle = token & QTD_TOKEN_TOGGLE;
		qh->next     = QTD_TERMINATE;
		qh->alt_next = QTD_TERMINATE;
		qh->token    = toggle | QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_IN |
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
		intr_transfer_active[index] = true;
		intr_prime_time[index] = millis();
		return 0;
	}

	if (token & (QTD_TOKEN_BUFERR | QTD_TOKEN_BABBLE | QTD_TOKEN_XACTERR)) {
		intr_error_count[index]++;
		qh->token = token & QTD_TOKEN_TOGGLE;
		qh->next = QTD_TERMINATE;
		asm volatile("dsb" ::: "memory");
		return -1;
	}

	uint32_t remaining = (token >> 16) & 0x7FFF;
	uint32_t transferred = len - remaining;
	if (transferred > 0)
		*data_ptr = buf; // return pointer directly into DMA buffer
	return (int)transferred;
}
