// ft6206.c — FT6206 capacitive touch driver for i.MX RT1062

#include "ft6206.h"
#include "imxrt.h"

extern uint32_t millis(void);
extern void delay(uint32_t msec);

#define FT_ADDR            0x38
#define FT_REG_DEVMODE     0x00
#define FT_REG_NUM_TOUCHES 0x02
#define FT_REG_P1_XH       0x03
#define FT_REG_P1_XL       0x04
#define FT_REG_P1_YH       0x05
#define FT_REG_P1_YL       0x06
#define FT_REG_THRESHHOLD  0x80
#define FT_REG_CTRL        0x86
#define FT_REG_RATE        0x88
#define FT_REG_G_MODE      0xA4
#define FT_REG_VENDID      0xA8
#define FT_REG_CHIPID      0xA3

#define FT_EVENT_PRESS     0x00
#define FT_EVENT_LIFT      0x01
#define FT_EVENT_CONTACT   0x02

#define I2C_TIMEOUT_US     5000

static bool     g_was_touched;
static uint32_t g_last_event_ms;

static bool i2c_wait_tx(void)
{
	uint32_t t = millis();
	while (!(LPI2C1_MSR & LPI2C_MSR_TDF)) {
		if (LPI2C1_MSR & (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF)) {
			LPI2C1_MSR = LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF;
			return false;
		}
		if ((millis() - t) > 5) return false;
	}
	return true;
}

static bool i2c_wait_rx(void)
{
	uint32_t t = millis();
	while (!(LPI2C1_MSR & LPI2C_MSR_RDF)) {
		if (LPI2C1_MSR & (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF)) {
			LPI2C1_MSR = LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF;
			return false;
		}
		if ((millis() - t) > 5) return false;
	}
	return true;
}

static void i2c_wait_stop(void)
{
	uint32_t t = millis();
	while (!(LPI2C1_MSR & LPI2C_MSR_SDF)) {
		if ((millis() - t) > 5) break;
	}
	LPI2C1_MSR = LPI2C_MSR_SDF;
}

static bool i2c_write_reg(uint8_t reg, uint8_t val)
{
	// Clear any pending errors/flags
	LPI2C1_MSR = LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF |
	             LPI2C_MSR_SDF | LPI2C_MSR_EPF;

	// START + address (write)
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(FT_ADDR << 1);

	// Register address
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_TRANSMIT | LPI2C_MTDR_DATA(reg);

	// Data byte
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_TRANSMIT | LPI2C_MTDR_DATA(val);

	// STOP
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_STOP;

	i2c_wait_stop();
	return true;
}

static bool i2c_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{

	LPI2C1_MSR = LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF |
	             LPI2C_MSR_SDF | LPI2C_MSR_EPF;
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA(FT_ADDR << 1);
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_TRANSMIT | LPI2C_MTDR_DATA(reg);
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_START | LPI2C_MTDR_DATA((FT_ADDR << 1) | 1);
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_RECEIVE | LPI2C_MTDR_DATA(len - 1);
	for (uint8_t i = 0; i < len; i++) {
		if (!i2c_wait_rx()) return false;
		buf[i] = (uint8_t)(LPI2C1_MRDR & 0xFF);
	}
	if (!i2c_wait_tx()) return false;
	LPI2C1_MTDR = LPI2C_MTDR_CMD_STOP;

	i2c_wait_stop();
	return true;
}
// Bit-bang SCL to recover a stuck I2C bus (SDA held low by slave).
// Muxes pin 19 as GPIO output, toggles SCL up to 16 times, then restores.
static void i2c_bus_recover(void)
{
	// Mux pin 19 (GPIO_AD_B1_00) as GPIO1_IO16
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 5; // ALT5 = GPIO
	GPIO1_GDIR |= (1 << 16);
	for (int i = 0; i < 16; i++) {
		GPIO1_DR_CLEAR = (1 << 16);
		for (volatile int d = 0; d < 200; d++) {}
		GPIO1_DR_SET = (1 << 16);
		for (volatile int d = 0; d < 200; d++) {}
	}
	GPIO1_GDIR &= ~(1 << 16);
}

bool ft6206_init(void)
{
	// FT6206 needs ~300ms after power-on before I2C is ready
	delay(350);

	CCM_CCGR2 |= CCM_CCGR2_LPI2C1(CCM_CCGR_ON);
	CCM_CSCDR2 = (CCM_CSCDR2 & ~(CCM_CSCDR2_LPI2C_CLK_SEL |
	              CCM_CSCDR2_LPI2C_CLK_PODF(0x1F))) |
	             CCM_CSCDR2_LPI2C_CLK_SEL; // 24 MHz OSC

	// Recover bus before muxing to LPI2C — clears stuck SDA
	i2c_bus_recover();

	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 3; // ALT3 = LPI2C1_SCL
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 3; // ALT3 = LPI2C1_SDA
	// I2C pads: open-drain mandatory, slow slew prevents ringing
	uint32_t i2c_pad = IOMUXC_PAD_ODE | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE |
	                   IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS |
	                   IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SPEED(0);
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 = i2c_pad;
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_01 = i2c_pad;
	IOMUXC_LPI2C1_SCL_SELECT_INPUT = 1;
	IOMUXC_LPI2C1_SDA_SELECT_INPUT = 1;
	LPI2C1_MCR = LPI2C_MCR_RST;
	LPI2C1_MCR = 0;

	// Fast Mode ~400 kHz on 24 MHz LPI2C clock
	LPI2C1_MCCR0 = LPI2C_MCCR0_CLKHI(14) | LPI2C_MCCR0_CLKLO(28) |
	               LPI2C_MCCR0_SETHOLD(8) | LPI2C_MCCR0_DATAVD(4);
	LPI2C1_MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);

	// Glitch filters: 2 cycles on SCL and SDA (removes <84 ns spikes)
	// Bus idle timeout: ~100 µs, pin-low timeout: ~200 µs
	LPI2C1_MCFGR2 = LPI2C_MCFGR2_FILTSCL(2) | LPI2C_MCFGR2_FILTSDA(2) |
	                LPI2C_MCFGR2_BUSIDLE(2400);
	LPI2C1_MCFGR3 = LPI2C_MCFGR3_PINLOW(4800);

	LPI2C1_MFCR = LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0);
	LPI2C1_MCR = LPI2C_MCR_MEN;

	// Retry vendor + chip ID — FT6206 may need a moment after bus recovery
	uint8_t vendor_id = 0, chip_id = 0;
	for (int attempt = 0; attempt < 5; attempt++) {
		if (i2c_read_regs(FT_REG_VENDID, &vendor_id, 1) && vendor_id == 0x11) {
			i2c_read_regs(FT_REG_CHIPID, &chip_id, 1);
			if (chip_id == 0x06)
				break;
		}
		delay(50);
	}
	if (vendor_id != 0x11 || chip_id != 0x06) return false;

	// Touch sensitivity: 20-80. Lower = more sensitive.
	i2c_write_reg(FT_REG_THRESHHOLD, 40);

	// Report rate: 0x01 = 10 ms period = 100 Hz
	i2c_write_reg(FT_REG_RATE, 0x01);

	// Stay in active mode (not monitor/hibernate)
	i2c_write_reg(FT_REG_CTRL, 0x00);

	// Trigger mode: INT stays low while touched (don't need the INT pin,
	// but trigger mode keeps registers updated continuously for polling)
	i2c_write_reg(FT_REG_G_MODE, 0x01);

	g_was_touched = false;
	g_last_event_ms = 0;

	return true;
}

touch_point_t ft6206_read(void)
{
	touch_point_t pt = { .x = 0, .y = 0, .valid = false };

	// Read 5 bytes: NUM_TOUCHES + P1 coordinate registers.
	// No INT pin connected — just poll touch count + coordinates.
	uint8_t buf[5];
	if (!i2c_read_regs(FT_REG_NUM_TOUCHES, buf, 5)) return pt;

	uint8_t touches = buf[0] & 0x0F;
	if (touches == 0 || touches > 2) return pt;

	uint16_t raw_x = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
	uint16_t raw_y = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];
	pt.x = raw_x;
	pt.y = raw_y;
	pt.valid = true;
	return pt;
}

bool ft6206_is_touched(void)
{
	uint8_t touches = 0;
	if (!i2c_read_regs(FT_REG_NUM_TOUCHES, &touches, 1)) return false;
	touches &= 0x0F;
	return (touches > 0 && touches <= 2);
}

bool ft6206_poll(touch_point_t *pt)
{
	touch_point_t raw = ft6206_read();
	uint32_t now = millis();

	if (raw.valid) {
		if (!g_was_touched) {
			if ((now - g_last_event_ms) < 100) {
				*pt = (touch_point_t){ .x = 0, .y = 0, .valid = false };
				return false;
			}
			g_was_touched = true;
			g_last_event_ms = now;
			*pt = raw;
			return true;
		}
		*pt = raw;
		return false;
	}
	if (g_was_touched) {
		g_was_touched = false;
		g_last_event_ms = now;
	}
	*pt = (touch_point_t){ .x = 0, .y = 0, .valid = false };
	return false;
}
