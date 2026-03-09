// Minimal startup for Teensy 4.1 (i.MX RT1062)
// Derived from Teensy 4 cores (MIT License, Paul Stoffregen)
// Stripped to bare essentials: clock, cache, systick, USB PLL

#include <stdint.h>
#include <string.h>
#include "imxrt.h"

// From linker script
extern unsigned long _stextload;
extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _sdataload;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _flexram_bank_config;
extern unsigned long _estack;
extern unsigned long _heap_start;
extern unsigned long _heap_end;

// Vector table in RAM
__attribute__ ((used, aligned(1024), section(".vectorsram")))
void (* volatile _VectorsRam[NVIC_NUM_INTERRUPTS + 16])(void);

// Global clock tracking
volatile uint32_t F_CPU_ACTUAL = 396000000;
volatile uint32_t systick_millis_count = 0;
volatile uint32_t systick_cycle_count = 0;
volatile uint32_t scale_cpu_cycles_to_microseconds = 0;
uint32_t systick_safe_read;

extern int main(void);

// Forward declarations
static void memory_copy(uint32_t *dest, const uint32_t *src, uint32_t *dest_end);
static void memory_clear(uint32_t *dest, uint32_t *dest_end);
static void configure_cache(void);
static void configure_systick(void);
static void usb_pll_start(void);
static void reset_PFD(void);

// HardFault handler — rapid LED blink (pin 13 = GPIO7 bit 3)
void hardfault_isr(void)
{
	// GPIO7 pin 3 already configured as output by startup
	volatile uint32_t i;
	while (1) {
		GPIO7_DR_SET = (1 << 3);
		for (i = 0; i < 600000; i++) asm("");
		GPIO7_DR_CLEAR = (1 << 3);
		for (i = 0; i < 600000; i++) asm("");
	}
}

// Default interrupt handler
void unused_interrupt_vector(void)
{
	while (1) asm("WFI");
}

// SysTick ISR
void systick_isr(void)
{
	systick_millis_count++;
}

// PendSV ISR (unused, but needed by vector table setup)
void pendablesrvreq_isr(void)
{
}

// --------------- Clock speed ---------------
// Simplified from clockspeed.c

#define FLASHMEM __attribute__((section(".flashmem")))

FLASHMEM uint32_t set_arm_clock(uint32_t frequency)
{
	uint32_t cbcdr = CCM_CBCDR;
	uint32_t cbcmr = CCM_CBCMR;

	if (frequency > 528000000) {
		// Step 1: Raise core voltage
		// 600 MHz → TRG(15) ~1.25V; 816 MHz → TRG(19) ~1.30V
		uint32_t dcdc = DCDC_REG3;
		dcdc &= ~DCDC_REG3_TRG_MASK;
		dcdc |= DCDC_REG3_TRG(frequency > 600000000 ? 19 : 15);
		DCDC_REG3 = dcdc;
		while (!(DCDC_REG0 & DCDC_REG0_STS_DC_OK)) ;

		// Step 2: Switch CPU to safe clock (PERIPH_CLK2) before touching ARM PLL.
		// Use USB1 PLL (480 MHz) / 4 = 120 MHz as safe clock.
		if (!(cbcdr & CCM_CBCDR_PERIPH_CLK_SEL)) {
			cbcdr &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
			cbcdr |= CCM_CBCDR_PERIPH_CLK2_PODF(3); // divide by 4
			CCM_CBCDR = cbcdr;
			cbcmr &= ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
			cbcmr |= CCM_CBCMR_PERIPH_CLK2_SEL(0); // select USB1 PLL
			CCM_CBCMR = cbcmr;
			while (CCM_CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY) ;
			cbcdr |= CCM_CBCDR_PERIPH_CLK_SEL; // switch to PERIPH_CLK2
			CCM_CBCDR = cbcdr;
			while (CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) ;
		}

		// Step 3: Configure ARM PLL (PLL1)
		// PLL1 output = 24 MHz * DIV_SELECT / 2
		// ARM clock = PLL1 / ARM_PODF(1+1) = PLL1 / 2
		// So DIV_SELECT = frequency * 2 / (24 MHz / 2) = frequency / 6 MHz
		// 600 MHz: DIV=100, PLL=1200 MHz, ARM=600 MHz
		// 816 MHz: DIV=136, PLL=1632 MHz, ARM=816 MHz
		uint32_t div_select = frequency / 6000000;
		CCM_ANALOG_PLL_ARM = CCM_ANALOG_PLL_ARM_ENABLE |
			CCM_ANALOG_PLL_ARM_DIV_SELECT(div_select);
		while (!(CCM_ANALOG_PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK)) ;

		// Step 4: Set ARM_PODF = divide by 2
		CCM_CACRR = CCM_CACRR_ARM_PODF(1);
		while (CCM_CDHIPR & CCM_CDHIPR_ARM_PODF_BUSY) ;

		// Step 5: Set AHB divider (divide by 1 → AHB = target frequency)
		cbcdr = CCM_CBCDR;
		cbcdr &= ~(CCM_CBCDR_AHB_PODF_MASK | (0x07 << 16));
		cbcdr |= CCM_CBCDR_AHB_PODF(0) | CCM_CBCDR_SEMC_PODF(7);
		CCM_CBCDR = cbcdr;
		while (CCM_CDHIPR & CCM_CDHIPR_AHB_PODF_BUSY) ;

		// Step 6: Set IPG divider = 4 (e.g. 816/4 = 204 MHz, 600/4 = 150 MHz)
		cbcdr = CCM_CBCDR;
		cbcdr &= ~CCM_CBCDR_IPG_PODF_MASK;
		cbcdr |= CCM_CBCDR_IPG_PODF(3);
		CCM_CBCDR = cbcdr;

		// Step 7: Select divided PLL1 on PRE_PERIPH mux, then switch back
		cbcmr = CCM_CBCMR;
		cbcmr &= ~CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK;
		cbcmr |= CCM_CBCMR_PRE_PERIPH_CLK_SEL(3);
		CCM_CBCMR = cbcmr;
		CCM_CBCDR &= ~CCM_CBCDR_PERIPH_CLK_SEL; // switch back from PERIPH_CLK2
		while (CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) ;

		F_CPU_ACTUAL = frequency;
	} else {
		F_CPU_ACTUAL = 396000000;
	}
	scale_cpu_cycles_to_microseconds = 0xFFFFFFFFu / (F_CPU_ACTUAL / 1000000u);
	return F_CPU_ACTUAL;
}

// --------------- MPU / Cache ---------------

#define NOEXEC      SCB_MPU_RASR_XN
#define READONLY    SCB_MPU_RASR_AP(7)
#define READWRITE   SCB_MPU_RASR_AP(3)
#define NOACCESS    SCB_MPU_RASR_AP(0)
#define MEM_CACHE_WT    SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C
#define MEM_CACHE_WB    SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_CACHE_WBWA  SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_NOCACHE     SCB_MPU_RASR_TEX(1)
#define DEV_NOCACHE     SCB_MPU_RASR_TEX(2)
#define SIZE_32B    (SCB_MPU_RASR_SIZE(4) | SCB_MPU_RASR_ENABLE)
#define SIZE_32K    (SCB_MPU_RASR_SIZE(14) | SCB_MPU_RASR_ENABLE)
#define SIZE_64K    (SCB_MPU_RASR_SIZE(15) | SCB_MPU_RASR_ENABLE)
#define SIZE_128K   (SCB_MPU_RASR_SIZE(16) | SCB_MPU_RASR_ENABLE)
#define SIZE_256K   (SCB_MPU_RASR_SIZE(17) | SCB_MPU_RASR_ENABLE)
#define SIZE_512K   (SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)
#define SIZE_1M     (SCB_MPU_RASR_SIZE(19) | SCB_MPU_RASR_ENABLE)
#define SIZE_16M    (SCB_MPU_RASR_SIZE(23) | SCB_MPU_RASR_ENABLE)
#define SIZE_64M    (SCB_MPU_RASR_SIZE(25) | SCB_MPU_RASR_ENABLE)
#define SIZE_1G     (SCB_MPU_RASR_SIZE(29) | SCB_MPU_RASR_ENABLE)
#define SIZE_4G     (SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE)
#define REGION(n)   (SCB_MPU_RBAR_REGION(n) | SCB_MPU_RBAR_VALID)

FLASHMEM static void configure_cache(void)
{
	SCB_MPU_CTRL = 0;

	uint32_t i = 0;
	SCB_MPU_RBAR = 0x00000000 | REGION(i++);
	SCB_MPU_RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_4G;

	SCB_MPU_RBAR = 0x00000000 | REGION(i++); // ITCM
	SCB_MPU_RASR = MEM_NOCACHE | READONLY | SIZE_512K;

	SCB_MPU_RBAR = 0x00000000 | REGION(i++); // trap NULL deref
	SCB_MPU_RASR = DEV_NOCACHE | NOACCESS | SIZE_32B;

	SCB_MPU_RBAR = 0x00200000 | REGION(i++); // Boot ROM
	SCB_MPU_RASR = MEM_CACHE_WT | READONLY | SIZE_128K;

	SCB_MPU_RBAR = 0x20000000 | REGION(i++); // DTCM
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_512K;

	// Stack guard disabled for bring-up stability.
	// The minimal linker/startup layout can place this region where valid
	// early accesses occur, causing immediate faults before main().

	SCB_MPU_RBAR = 0x20200000 | REGION(i++); // RAM (AXI bus)
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1M;

	SCB_MPU_RBAR = 0x40000000 | REGION(i++); // Peripherals
	SCB_MPU_RASR = DEV_NOCACHE | READWRITE | NOEXEC | SIZE_64M;

	SCB_MPU_RBAR = 0x60000000 | REGION(i++); // QSPI Flash
	SCB_MPU_RASR = MEM_CACHE_WBWA | READONLY | SIZE_16M;

	SCB_MPU_RBAR = 0x70000000 | REGION(i++); // FlexSPI2
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

	SCB_MPU_RBAR = 0x80000000 | REGION(i++); // SEMC
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1G;

	// DMA buffer overlay: non-cacheable (higher region # wins over region 5)
	// Eliminates all manual cache maintenance for USB + TFT DMA structures
	SCB_MPU_RBAR = 0x20200000 | REGION(10);
#if NET_ENABLED
	// 128K for USB (~4KB) + TFT (~41KB) + ENET (~15KB) buffers
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_128K;
#else
	// 64K covers USB buffers (~4KB) + ST7735 TFT framebuffers (~41KB)
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_64K;
#endif

	asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
	SCB_MPU_CTRL = SCB_MPU_CTRL_ENABLE;

	asm("dsb"); asm("isb");
	SCB_CACHE_ICIALLU = 0;
	asm("dsb"); asm("isb");
	SCB_CCR |= (SCB_CCR_IC | SCB_CCR_DC);
}

// --------------- SysTick (100kHz external) ---------------

#define SYSTICK_EXT_FREQ 100000

static void configure_systick(void)
{
	_VectorsRam[14] = pendablesrvreq_isr;
	_VectorsRam[15] = systick_isr;
	SYST_RVR = (SYSTICK_EXT_FREQ / 1000) - 1;
	SYST_CVR = 0;
	SYST_CSR = SYST_CSR_TICKINT | SYST_CSR_ENABLE;
	SCB_SHPR3 = 0x20200000; // SysTick + PendSV = priority 32
	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	systick_cycle_count = ARM_DWT_CYCCNT;
}

// --------------- USB PLL ---------------
// We need USB1 PLL running for the clock tree even though we use USB2 for host.
// USB2 PLL is started separately in our host driver.

FLASHMEM static void usb_pll_start(void)
{
	while (1) {
		uint32_t n = CCM_ANALOG_PLL_USB1;
		if (n & CCM_ANALOG_PLL_USB1_DIV_SELECT) {
			CCM_ANALOG_PLL_USB1_CLR = 0xC000;
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_BYPASS;
			CCM_ANALOG_PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_POWER |
				CCM_ANALOG_PLL_USB1_DIV_SELECT |
				CCM_ANALOG_PLL_USB1_ENABLE |
				CCM_ANALOG_PLL_USB1_EN_USB_CLKS;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_ENABLE)) {
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_ENABLE;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_POWER)) {
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_POWER;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_LOCK)) continue;
		if (n & CCM_ANALOG_PLL_USB1_BYPASS) {
			CCM_ANALOG_PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_BYPASS;
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_EN_USB_CLKS)) {
			CCM_ANALOG_PLL_USB1_SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS;
			continue;
		}
		return;
	}
}

FLASHMEM static void reset_PFD(void)
{
	CCM_ANALOG_PFD_528_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
	CCM_ANALOG_PFD_528 = 0x2018101B;
	CCM_ANALOG_PFD_480_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
	CCM_ANALOG_PFD_480 = 0x13110D0C;
}

// --------------- Timing functions ---------------

uint32_t millis(void)
{
	return systick_millis_count;
}

uint32_t micros(void)
{
	uint32_t smc, scc;
	do {
		__asm__ volatile("ldrex %0, [%1]" : "=r"(smc) : "r"(&systick_safe_read));
		smc = systick_millis_count;
		scc = systick_cycle_count;
	} while (__builtin_expect(({
		uint32_t tmp;
		__asm__ volatile("strex %0, %1, [%2]" : "=&r"(tmp) : "r"(1), "r"(&systick_safe_read));
		tmp;
	}), 0));
	uint32_t cyccnt = ARM_DWT_CYCCNT;
	asm volatile("" : : : "memory");
	uint32_t ccdelta = cyccnt - scc;
	uint32_t frac = ((uint64_t)ccdelta * scale_cpu_cycles_to_microseconds) >> 32;
	if (frac > 1000) frac = 1000;
	return 1000 * smc + frac;
}

void delay(uint32_t msec)
{
	uint32_t start = micros();
	if (msec == 0) return;
	while (1) {
		while ((micros() - start) >= 1000) {
			if (--msec == 0) return;
			start += 1000;
		}
	}
}

// --------------- Reset Handler ---------------

static void ResetHandler2(void);

__attribute__((section(".startup"), naked))
void ResetHandler(void)
{
	IOMUXC_GPR_GPR17 = (uint32_t)&_flexram_bank_config;
	IOMUXC_GPR_GPR16 = 0x00200007;
	IOMUXC_GPR_GPR14 = 0x00AA0000;
	__asm__ volatile("mov sp, %0" : : "r" ((uint32_t)&_estack) : "memory");
	ResetHandler2();
	__builtin_unreachable();
}

__attribute__((section(".startup"), noinline, noreturn))
static void ResetHandler2(void)
{
	unsigned int i;
	__asm__ volatile("dsb":::"memory");

	// Hardware settling delay — official Teensy startup requires these
	// "Some optimization with LTO won't start without this delay, but why?"
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");

	PMU_MISC0_SET = 1 << 3; // Use bandgap-based bias currents for best performance

	// Additional settling delay after PMU change — critical for boot stability
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");

	// Early pin mux for LED13 so fault handlers have visible output.
	// Do not block here: continue to memory init and main().
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5;        // ALT5 = GPIO
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7);
	IOMUXC_GPR_GPR27 = 0xFFFFFFFF;                // enable fast GPIO7
	GPIO7_GDIR |= (1 << 3);

	// Initialize memory
	memory_copy(&_stext, &_stextload, &_etext);
	memory_copy(&_sdata, &_sdataload, &_edata);
	memory_clear(&_sbss, &_ebss);

	// Enable FPU
	SCB_CPACR = 0x00F00000;

	// Set up blank interrupt vector table
	for (i = 0; i < NVIC_NUM_INTERRUPTS + 16; i++) _VectorsRam[i] = &unused_interrupt_vector;
	_VectorsRam[3] = hardfault_isr;  // HardFault = vector 3
	for (i = 0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, 128);
	SCB_VTOR = (uint32_t)_VectorsRam;

	reset_PFD();

	// Keep optional fault escalations disabled during bring-up.
	// HardFault handler still catches fatal issues.

	// Clock config
	CCM_CSCMR1 = (CCM_CSCMR1 & ~CCM_CSCMR1_PERCLK_PODF(0x3F)) | CCM_CSCMR1_PERCLK_CLK_SEL;
	CCM_CSCDR1 = (CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL;

	// Fast GPIO
	IOMUXC_GPR_GPR26 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR27 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR28 = 0xFFFFFFFF;
	IOMUXC_GPR_GPR29 = 0xFFFFFFFF;

	configure_cache();
	configure_systick();
	usb_pll_start();
	reset_PFD();

#ifdef F_CPU
	set_arm_clock(F_CPU);
#endif

	// Undo PIT timer usage by ROM
	CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
	PIT_MCR = 0;
	PIT_TCTRL0 = 0;
	PIT_TCTRL1 = 0;
	PIT_TCTRL2 = 0;
	PIT_TCTRL3 = 0;

	// Initialize RTC
	if (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) {
		SNVS_LPSRTCLR = 1546300800u << 15;
		SNVS_LPSRTCMR = 1546300800u >> 17;
		SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
	}
	SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;

	// We deliberately skip usb_init() — no USB device stack.
	// USB host (USB2) is initialized from main().

	// LED pin 13 (GPIO_B0_03 = GPIO7 bit 3) — for debug
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5; // ALT5 = GPIO
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7);
	GPIO7_GDIR |= (1 << 3);

	main();
	while (1) asm("WFI");
}

// --------------- Memory helpers (must be in .startup section) ---------------

__attribute__((section(".startup"), noinline))
static void memory_copy(uint32_t *dest, const uint32_t *src, uint32_t *dest_end)
{
	asm volatile(
	"	cmp	%[src], %[dest]		\n"
	"	beq.n	2f			\n"
	"1:	ldr.w	r3, [%[src]], #4	\n"
	"	str.w	r3, [%[dest]], #4	\n"
	"	cmp	%[end], %[dest]		\n"
	"	bhi.n	1b			\n"
	"2:					\n"
	: [dest] "+r" (dest), [src] "+r" (src) : [end] "r" (dest_end) : "r3", "memory");
}

__attribute__((section(".startup"), noinline))
static void memory_clear(uint32_t *dest, uint32_t *dest_end)
{
	asm volatile(
	"	ldr	r3, =0			\n"
	"1:	str.w	r3, [%[dest]], #4	\n"
	"	cmp	%[end], %[dest]		\n"
	"	bhi.n	1b			\n"
	: [dest] "+r" (dest) : [end] "r" (dest_end) : "r3", "memory");
}

// --------------- Newlib syscall stubs ---------------

#include <errno.h>
#include <sys/stat.h>

char *__brkval = (char *)&_heap_start;

void * _sbrk(int incr)
{
	char *prev = __brkval;
	if (incr != 0) {
		if (prev + incr > (char *)&_heap_end) {
			errno = ENOMEM;
			return (void *)-1;
		}
		__brkval = prev + incr;
	}
	return prev;
}

int _read(int file __attribute__((unused)), char *ptr __attribute__((unused)), int len __attribute__((unused))) { return 0; }
int _close(int fd __attribute__((unused))) { return -1; }
int _fstat(int fd __attribute__((unused)), struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd __attribute__((unused))) { return 1; }
int _lseek(int fd __attribute__((unused)), long long offset __attribute__((unused)), int whence __attribute__((unused))) { return -1; }
void _exit(int status __attribute__((unused))) { while (1) asm("WFI"); }
int _write(int file __attribute__((unused)), char *ptr __attribute__((unused)), int len __attribute__((unused))) { return len; }
int _kill(int pid __attribute__((unused)), int sig __attribute__((unused))) { return -1; }
int _getpid(void) { return 1; }

// yield() - called from delay()
void yield(void) {}
