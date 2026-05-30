// src/led.c — on-board LED (pin 13) driver.
//
// Pin 13 = pad GPIO_B0_03. core/startup.c routes the B0 bank to fast GPIO7
// (GPR27) and configures pin 13 as GPIO7[3] output; the HardFault handler
// drives it directly. This module owns pin 13 in two modes:
//   * GPIO  (boot / enumeration / fatal codes): GPIO7[3] set/clear/toggle.
//   * Heartbeat (running): QuadTimer2 cascade (CH1 prescaler -> CH0 output),
//     pad muxed to ALT1 = QTIMER2_TIMER0. Autonomous; the blink RATE encodes
//     UART status. Zero CPU once started.
#include "led.h"
#include "imxrt.h"
#include <stdbool.h>

extern void delay(uint32_t msec);

#define LED_GPIO_BIT      (1u << 3)   // GPIO7[3]
#define LED_PAD_ALT_GPIO  5           // ALT5 = GPIO
#define LED_PAD_ALT_QTMR  1           // ALT1 = QTIMER2_TIMER0 (TMR2 CH0)

// --- QuadTimer cascade tuning -------------------------------------------------
// CH1 divides the IP-bus clock (PCS = IP/128) and toggles its OFLAG every
// (LED_CH1_COMP+1) counts, producing the clock for CH0. CH0 toggles pin 13
// every (COMP0+1) of those -> 50% square wave. blink_hz is inversely
// proportional to (COMP0+1), so the RATE RATIOS between states are exact
// regardless of the true IP-bus clock; only the absolute scale (LED_HB_K)
// depends on it. If the measured idle blink isn't ~0.5 Hz, scale LED_HB_K by
// the observed ratio and the active/error rates track automatically.
#define LED_CH1_COMP    1464u         // CH1 modulo (period = COMP+1)
#define LED_HB_K        20000u        // COMP0 = LED_HB_K/centihz - 1 (nominal IP=150MHz)
#define LED_CENTIHZ_MIN 5u            // clamp floor (0.05 Hz)

static bool s_hb_active;

static uint16_t centihz_to_comp(uint16_t centihz)
{
	if (centihz < LED_CENTIHZ_MIN) centihz = LED_CENTIHZ_MIN;
	uint32_t comp = LED_HB_K / centihz;
	if (comp == 0) comp = 1;
	comp -= 1u;
	if (comp > 0xFFFFu) comp = 0xFFFFu;
	return (uint16_t)comp;
}

void led_init(void)
{
	// startup.c already set ALT5 + GPR27 (fast GPIO7) + GDIR; re-assert + off.
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_GPIO;
	GPIO7_GDIR |= LED_GPIO_BIT;
	GPIO7_DR_CLEAR = LED_GPIO_BIT;
	s_hb_active = false;
}

void led_on(void)     { GPIO7_DR_SET    = LED_GPIO_BIT; }
void led_off(void)    { GPIO7_DR_CLEAR  = LED_GPIO_BIT; }
void led_toggle(void) { GPIO7_DR_TOGGLE = LED_GPIO_BIT; }

// Return pin 13 to GPIO7 output, stopping the heartbeat QuadTimer if it had
// taken over the pad. Lets fatal-code blinking work after the heartbeat began.
static void led_reclaim_gpio(void)
{
	if (s_hb_active) {
		IMXRT_TMR2.CH[0].CTRL = 0;   // stop output channel
		IMXRT_TMR2.CH[1].CTRL = 0;   // stop prescaler
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_GPIO;
		GPIO7_GDIR |= LED_GPIO_BIT;
		s_hb_active = false;
	}
}

void led_blink_forever(uint8_t code, uint32_t on_ms, uint32_t off_ms)
{
	led_reclaim_gpio();
	if (code == 0) code = 1;
	for (;;) {
		for (uint8_t i = 0; i < code; i++) {
			GPIO7_DR_SET   = LED_GPIO_BIT; delay(on_ms);
			GPIO7_DR_CLEAR = LED_GPIO_BIT; delay(off_ms);
		}
		delay(700);   // gap between repeats of the code
	}
}

void led_heartbeat_set_rate(uint16_t centihz)
{
	if (!s_hb_active) return;
	uint16_t comp = centihz_to_comp(centihz);
	// Glitch-free: write the preload regs; CSCTRL CL1/CL2=1 latches COMP at the
	// next compare boundary instead of mid-count.
	IMXRT_TMR2.CH[0].CMPLD1 = comp;
	IMXRT_TMR2.CH[0].CMPLD2 = comp;
}

void led_heartbeat_start(void)
{
	CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);

	// CH1: prescaler. IP-bus/128, toggle OFLAG every (COMP+1) -> CH0 clock.
	IMXRT_TMR2.CH[1].CTRL   = 0;
	IMXRT_TMR2.CH[1].CNTR   = 0;
	IMXRT_TMR2.CH[1].LOAD   = 0;
	IMXRT_TMR2.CH[1].COMP1  = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].COMP2  = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CMPLD1 = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CMPLD2 = LED_CH1_COMP;
	IMXRT_TMR2.CH[1].CSCTRL = TMR_CSCTRL_CL1(1) | TMR_CSCTRL_CL2(1);
	IMXRT_TMR2.CH[1].SCTRL  = 0;
	IMXRT_TMR2.CH[1].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xF) |
	                          TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);

	// CH0: pin-13 output. Clocked by CH1 OFLAG (PCS=0b0101=5 = counter-1 output;
	// QuadTimer PCS: 0-3=counter inputs, 4-7=counter outputs, 8-15=IPbus/1..128).
	// Toggle OFLAG every (COMP0+1) -> 50% square; OEN drives the pad.
	uint16_t comp0 = centihz_to_comp(50);   // start at idle ~0.5 Hz
	IMXRT_TMR2.CH[0].CTRL   = 0;
	IMXRT_TMR2.CH[0].CNTR   = 0;
	IMXRT_TMR2.CH[0].LOAD   = 0;
	IMXRT_TMR2.CH[0].COMP1  = comp0;
	IMXRT_TMR2.CH[0].COMP2  = comp0;
	IMXRT_TMR2.CH[0].CMPLD1 = comp0;
	IMXRT_TMR2.CH[0].CMPLD2 = comp0;
	IMXRT_TMR2.CH[0].CSCTRL = TMR_CSCTRL_CL1(1) | TMR_CSCTRL_CL2(1);
	IMXRT_TMR2.CH[0].SCTRL  = TMR_SCTRL_OEN;
	IMXRT_TMR2.CH[0].CTRL   = TMR_CTRL_CM(1) | TMR_CTRL_PCS(5) |
	                          TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);

	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = LED_PAD_ALT_QTMR; // hand pad to QuadTimer
	s_hb_active = true;
}
