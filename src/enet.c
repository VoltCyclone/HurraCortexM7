// enet.c — Bare-metal ENET MAC + DP83825I PHY driver for Teensy 4.1
// Adapted from QNEthernet's register-level driver (driver_teensy41.c).
// Pure C, no Arduino/lwIP dependencies.

#if !NET_ENABLED
// Empty translation unit when NET disabled
typedef int _enet_unused;
#else

#include "enet.h"
#include "imxrt.h"
#include <string.h>

extern void delay(uint32_t msec);

// Busy-wait microsecond delay using ARM cycle counter (DWT_CYCCNT)
static void delay_us(uint32_t us)
{
	uint32_t cycles = (F_CPU / 1000000u) * us;
	uint32_t start = ARM_DWT_CYCCNT;
	while ((ARM_DWT_CYCCNT - start) < cycles) {}
}

// ---- Configuration ----
#define RX_COUNT   5
#define TX_COUNT   5
#define BUF_SIZE   1536  // multiple of 64, >= max Ethernet frame (1518+pad)

// ---- Buffer descriptor (matches ENET hardware format) ----
typedef struct {
	uint16_t length;
	uint16_t status;
	void    *buffer;
	uint16_t extend0;
	uint16_t extend1;
	uint16_t checksum;
	uint8_t  prototype;
	uint8_t  headerlen;
	uint16_t unused0;
	uint16_t extend2;
	uint32_t timestamp;
	uint16_t unused1;
	uint16_t unused2;
	uint16_t unused3;
	uint16_t unused4;
} enet_bd_t;

// RX status bits
#define BD_RX_EMPTY    0x8000
#define BD_RX_WRAP     0x2000
#define BD_RX_LAST     0x0800
#define BD_RX_TRUNC    0x0001
#define BD_RX_OVERRUN  0x0002
#define BD_RX_CRC      0x0004
#define BD_RX_NONOCT   0x0010
#define BD_RX_LENVIOLN 0x0020
#define BD_RX_ERRMASK  (BD_RX_TRUNC | BD_RX_OVERRUN | BD_RX_CRC | BD_RX_NONOCT | BD_RX_LENVIOLN)

// TX status bits
#define BD_TX_READY    0x8000
#define BD_TX_WRAP     0x2000
#define BD_TX_LAST     0x0800
#define BD_TX_CRC      0x0400

// ---- DMA buffers (non-cacheable via MPU region 10) ----
static enet_bd_t rx_ring[RX_COUNT] __attribute__((section(".dmabuffers"), aligned(64)));
static enet_bd_t tx_ring[TX_COUNT] __attribute__((section(".dmabuffers"), aligned(64)));
static uint8_t rx_bufs[RX_COUNT][BUF_SIZE] __attribute__((section(".dmabuffers"), aligned(64)));
static uint8_t tx_bufs[TX_COUNT][BUF_SIZE] __attribute__((section(".dmabuffers"), aligned(64)));

static volatile enet_bd_t *rx_bd_ptr;
static volatile enet_bd_t *tx_bd_ptr;
static bool phy_initialized;

// ---- MDIO (PHY register access via MII management frame) ----

static uint16_t mdio_read(uint16_t regaddr)
{
	ENET_EIR = ENET_EIR_MII;
	ENET_MMFR = ENET_MMFR_ST(1) | ENET_MMFR_OP(2) | ENET_MMFR_PA(0) |
	            ENET_MMFR_RA(regaddr) | ENET_MMFR_TA(2);
	while (!(ENET_EIR & ENET_EIR_MII)) {}
	uint16_t data = ENET_MMFR & 0xFFFF;
	ENET_EIR = ENET_EIR_MII;
	return data;
}

static void mdio_write(uint16_t regaddr, uint16_t data)
{
	ENET_EIR = ENET_EIR_MII;
	ENET_MMFR = ENET_MMFR_ST(1) | ENET_MMFR_OP(1) | ENET_MMFR_PA(0) |
	            ENET_MMFR_RA(regaddr) | ENET_MMFR_TA(2) | ENET_MMFR_DATA(data);
	while (!(ENET_EIR & ENET_EIR_MII)) {}
	ENET_EIR = ENET_EIR_MII;
}

// ---- Clock and pin setup ----

static void enable_enet_clocks(void)
{
	CCM_CCGR1 |= CCM_CCGR1_ENET(CCM_CCGR_ON);

	// PLL6: 50 MHz for RMII reference clock
	CCM_ANALOG_PLL_ENET_SET = CCM_ANALOG_PLL_ENET_BYPASS;
	CCM_ANALOG_PLL_ENET_CLR = CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC(3)
	                        | CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT(3)
	                        | CCM_ANALOG_PLL_ENET_DIV_SELECT(3);
	CCM_ANALOG_PLL_ENET_SET = CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN
	                        | CCM_ANALOG_PLL_ENET_ENABLE
	                        | CCM_ANALOG_PLL_ENET_DIV_SELECT(1);
	CCM_ANALOG_PLL_ENET_CLR = CCM_ANALOG_PLL_ENET_POWERDOWN;
	while (!(CCM_ANALOG_PLL_ENET & CCM_ANALOG_PLL_ENET_LOCK)) {}
	CCM_ANALOG_PLL_ENET_CLR = CCM_ANALOG_PLL_ENET_BYPASS;

	// Drive REFCLK as output to PHY
	IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~IOMUXC_GPR_GPR1_ENET1_CLK_SEL)
	                | IOMUXC_GPR_GPR1_ENET_IPG_CLK_S_EN
	                | IOMUXC_GPR_GPR1_ENET1_TX_CLK_DIR;
}

static void configure_phy_pins(void)
{
	// Strap pins (PHY address = 0, RMII slave mode)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = IOMUXC_PAD_PUE | IOMUXC_PAD_PKE
	                                  | IOMUXC_PAD_DSE(7);  // PhyAdd[0] = 0 (pull-down)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = IOMUXC_PAD_PUE | IOMUXC_PAD_PKE
	                                  | IOMUXC_PAD_DSE(7);  // PhyAdd[1] = 0
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = IOMUXC_PAD_PUS(3) | IOMUXC_PAD_PUE
	                                  | IOMUXC_PAD_PKE | IOMUXC_PAD_DSE(7);  // RMII slave (pull-up)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = IOMUXC_PAD_PUE | IOMUXC_PAD_PKE
	                                  | IOMUXC_PAD_DSE(7);  // Auto MDIX enable

	// Reset and power pins as GPIO outputs (GPIO7 = fast GPIO2)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_15 = IOMUXC_PAD_DSE(7);  // PWRDN
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_14 = IOMUXC_PAD_DSE(7);  // RST_N
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = 5;  // ALT5 = GPIO2_IO15 (fast: GPIO7)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = 5;  // ALT5 = GPIO2_IO14
	GPIO7_GDIR |= (1 << 15) | (1 << 14);
	GPIO7_DR_CLEAR = (1 << 15);  // Power down initially
	GPIO7_DR_SET   = (1 << 14);  // Deassert reset initially

	// MDIO + MDC pins
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_15 = IOMUXC_PAD_PUS(3) | IOMUXC_PAD_PUE
	                                  | IOMUXC_PAD_PKE | IOMUXC_PAD_ODE
	                                  | IOMUXC_PAD_DSE(5) | IOMUXC_PAD_SRE;  // MDIO (open-drain, pull-up)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_14 = IOMUXC_PAD_PUS(2) | IOMUXC_PAD_PUE
	                                  | IOMUXC_PAD_PKE | IOMUXC_PAD_SPEED(3)
	                                  | IOMUXC_PAD_DSE(5) | IOMUXC_PAD_SRE;  // MDC
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15 = 0;  // ALT0 = ENET_MDIO
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14 = 0;  // ALT0 = ENET_MDC
	IOMUXC_ENET_MDIO_SELECT_INPUT = 2;     // GPIO_B1_15_ALT0
}

static void configure_rmii_pins(void)
{
	uint32_t pad = IOMUXC_PAD_PUS(2) | IOMUXC_PAD_PUE | IOMUXC_PAD_PKE
	             | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_DSE(5) | IOMUXC_PAD_SRE;

	// Reset strap pads to RMII signal pads
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = pad;  // RXD0
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = pad;  // RXD1
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = pad;  // CRS_DV
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = pad;  // RXER
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_07 = pad;  // TXD0
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_08 = pad;  // TXD1
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_09 = pad;  // TXEN

	// RMII mux (ALT3)
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04 = 3;  // RXD0
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05 = 3;  // RXD1
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06 = 3;  // CRS_DV
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11 = 3;  // RXER
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07 = 3;  // TXD0
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08 = 3;  // TXD1
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09 = 3;  // TXEN

	// REFCLK (ALT6 + SION)
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_10 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SRE;
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10 = 6 | 0x10;  // ALT6 + SION

	// Select inputs
	IOMUXC_ENET_IPG_CLK_RMII_SELECT_INPUT = 1;  // GPIO_B1_10_ALT6
	IOMUXC_ENET0_RXDATA_SELECT_INPUT = 1;
	IOMUXC_ENET1_RXDATA_SELECT_INPUT = 1;
	IOMUXC_ENET_RXEN_SELECT_INPUT    = 1;
	IOMUXC_ENET_RXERR_SELECT_INPUT   = 1;
}

// ---- PHY initialization ----

static bool init_phy(void)
{
	enable_enet_clocks();
	configure_phy_pins();

	// REFCLK must be present at PHY before reset
	IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_10 = IOMUXC_PAD_DSE(6) | IOMUXC_PAD_SRE;
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10 = 6 | 0x10;
	ENET_MSCR = ENET_MSCR_MII_SPEED(9);  // MDC = 50MHz / (9+1) / 2 = 2.5MHz

	// Power on, reset PHY
	GPIO7_DR_SET   = (1 << 15);   // Power on (PWRDN high)
	delay(50);
	GPIO7_DR_CLEAR = (1 << 14);   // Assert reset
	delay_us(25);
	GPIO7_DR_SET   = (1 << 14);   // Deassert reset
	delay(2);                      // Wait for SMI ready

	// Verify DP83825I PHY ID
	uint16_t id1 = mdio_read(0x02);  // PHYIDR1
	uint16_t id2 = mdio_read(0x03);  // PHYIDR2
	if (id1 != 0x2000 || (id2 & 0xFFF0) != 0xA140) {
		GPIO7_GDIR &= ~((1u << 15) | (1u << 14));
		return false;
	}

	// LED: link status, active high, 10Hz blink
	mdio_write(0x18, (1 << 9) | (1 << 7));  // LEDCR: 10Hz blink, active high

	// RCSR: RMII 50MHz clock select, 2-bit elasticity buffer
	mdio_write(0x17, (1 << 7) | 1);

	phy_initialized = true;
	return true;
}

// ---- Public API ----

void enet_get_mac(uint8_t mac[6])
{
	uint32_t m1 = HW_OCOTP_MAC1;
	uint32_t m0 = HW_OCOTP_MAC0;
	mac[0] = (uint8_t)(m1 >> 8);
	mac[1] = (uint8_t)(m1);
	mac[2] = (uint8_t)(m0 >> 24);
	mac[3] = (uint8_t)(m0 >> 16);
	mac[4] = (uint8_t)(m0 >> 8);
	mac[5] = (uint8_t)(m0);
}

bool enet_init(void)
{
	if (!init_phy()) return false;

	configure_rmii_pins();

	// Initialize descriptor rings
	memset(rx_ring, 0, sizeof(rx_ring));
	memset(tx_ring, 0, sizeof(tx_ring));

	for (int i = 0; i < RX_COUNT; i++) {
		rx_ring[i].buffer = rx_bufs[i];
		rx_ring[i].status = BD_RX_EMPTY;
	}
	rx_ring[RX_COUNT - 1].status |= BD_RX_WRAP;

	for (int i = 0; i < TX_COUNT; i++) {
		tx_ring[i].buffer = tx_bufs[i];
		tx_ring[i].status = BD_TX_CRC;
	}
	tx_ring[TX_COUNT - 1].status |= BD_TX_WRAP;

	rx_bd_ptr = &rx_ring[0];
	tx_bd_ptr = &tx_ring[0];

	// Disable interrupts (we poll)
	ENET_EIMR = 0;

	// Receive control: RMII mode, CRC stripped, padding removed
	ENET_RCR = ENET_RCR_NLC
	         | ENET_RCR_MAX_FL(1522)
	         | ENET_RCR_CFEN
	         | ENET_RCR_CRCFWD
	         | ENET_RCR_PADEN
	         | ENET_RCR_RMII_MODE
	         | ENET_RCR_FCE
	         | ENET_RCR_MII_MODE;

	// Transmit control: insert MAC address, full duplex
	ENET_TCR = ENET_TCR_ADDINS | ENET_TCR_ADDSEL(0) | ENET_TCR_FDEN;

	// Hardware checksum acceleration (IP + protocol)
	ENET_TACC = 0;  // We build checksums in software for our minimal stack
	ENET_RACC = ENET_RACC_LINEDIS | ENET_RACC_PADREM;

	ENET_TFWR = 0x100;  // Store and forward
	ENET_RSFL = 0;

	// Point ENET at descriptor rings
	ENET_RDSR = (uint32_t)rx_ring;
	ENET_TDSR = (uint32_t)tx_ring;
	ENET_MRBR = BUF_SIZE;

	ENET_RXIC = 0;
	ENET_TXIC = 0;

	// Program MAC address
	uint8_t mac[6];
	enet_get_mac(mac);
	ENET_PALR = ((uint32_t)mac[0] << 24) | ((uint32_t)mac[1] << 16)
	          | ((uint32_t)mac[2] << 8)   | mac[3];
	ENET_PAUR = ((uint32_t)mac[4] << 24) | ((uint32_t)mac[5] << 16) | 0x8808;

	ENET_OPD  = 0x10014;
	ENET_RSEM = 0;
	ENET_MIBC = 0;
	ENET_IAUR = 0;
	ENET_IALR = 0;
	ENET_GAUR = 0;
	ENET_GALR = 0;

	// Clear pending interrupts, enable MAC
	ENET_EIR = 0x7FFF8000;
	ENET_ECR = 0x70000000 | ENET_ECR_DBSWP | ENET_ECR_EN1588 | ENET_ECR_ETHEREN;

	// Activate RX/TX
	ENET_RDAR = ENET_RDAR_RDAR;
	ENET_TDAR = ENET_TDAR_TDAR;

	return true;
}

bool enet_link_up(void)
{
	if (!phy_initialized) return false;
	uint16_t bmsr = mdio_read(0x01);  // BMSR
	return (bmsr & (1 << 2)) != 0;    // Link Status bit
}

int enet_rx(const uint8_t **frame_out)
{
	volatile enet_bd_t *bd = rx_bd_ptr;

	if (bd->status & BD_RX_EMPTY)
		return 0;

	// Check for errors
	if (bd->status & BD_RX_ERRMASK) {
		// Release this descriptor and move on
		bd->status = (bd->status & BD_RX_WRAP) | BD_RX_EMPTY;
		ENET_RDAR = ENET_RDAR_RDAR;
		if (bd->status & BD_RX_WRAP)
			rx_bd_ptr = &rx_ring[0];
		else
			rx_bd_ptr++;
		return 0;
	}

	*frame_out = (const uint8_t *)bd->buffer;
	return bd->length;
}

void enet_rx_release(void)
{
	volatile enet_bd_t *bd = rx_bd_ptr;
	bool wrap = (bd->status & BD_RX_WRAP) != 0;
	bd->status = (wrap ? BD_RX_WRAP : 0) | BD_RX_EMPTY;
	ENET_RDAR = ENET_RDAR_RDAR;

	if (wrap)
		rx_bd_ptr = &rx_ring[0];
	else
		rx_bd_ptr++;
}

bool enet_tx(const uint8_t *frame, uint16_t len)
{
	if (len > BUF_SIZE) return false;

	volatile enet_bd_t *bd = tx_bd_ptr;

	// Wait for descriptor to become available
	uint32_t timeout = 100000;
	while ((bd->status & BD_TX_READY) && --timeout) {}
	if (!timeout) return false;

	memcpy(bd->buffer, frame, len);
	bd->length = len;
	bd->status = (bd->status & BD_TX_WRAP) | BD_TX_CRC | BD_TX_LAST | BD_TX_READY;
	ENET_TDAR = ENET_TDAR_TDAR;

	if (bd->status & BD_TX_WRAP)
		tx_bd_ptr = &tx_ring[0];
	else
		tx_bd_ptr++;

	return true;
}

#endif // NET_ENABLED
