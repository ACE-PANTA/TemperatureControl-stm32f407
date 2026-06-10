#include "eth.h"
#include "delay.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * Global configuration (set before Eth_Init)
 * ============================================================ */
u8 g_eth_mac[6]     = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
u8 g_eth_ip[4]      = {192, 168, 1, 100};
u8 g_eth_gateway[4] = {192, 168, 1, 1};
u8 g_eth_netmask[4] = {255, 255, 255, 0};
u16 g_tcp_port      = 8000;

/* ============================================================
 * Internal state
 * ============================================================ */
static u8  g_phy_addr   = 0xFF;  /* Detected PHY address (0xFF=not probed) */
static u8  g_link_up    = 0;     /* 1 = PHY link is up */
static u8  g_eth_error  = ETH_ERR_NOT_INITED;  /* Last error code */
static u32 g_phy_id1    = 0;     /* PHY ID register 1 (for diagnostics) */
static u32 g_phy_id2    = 0;     /* PHY ID register 2 (for diagnostics) */
static u8  g_init_done  = 0;     /* 1 = Eth_Init completed successfully */

/* Packet counters (reset on Eth_Init, live for diagnostics) */
static u16 g_rx_cnt     = 0;     /* Total frames received */
static u16 g_tx_cnt     = 0;     /* Total frames transmitted */
static u16 g_arp_cnt    = 0;     /* ARP requests handled */
static u16 g_icmp_cnt   = 0;     /* ICMP echo requests handled */

/* ============================================================
 * DMA descriptors & buffers
 * ============================================================ */
static EthDmaDesc g_rx_desc[ETH_RX_DESC_CNT];
static EthDmaDesc g_tx_desc[ETH_TX_DESC_CNT];
static u8 g_rx_buf[ETH_RX_DESC_CNT][ETH_RX_BUF_SIZE];
static u8 g_tx_buf[ETH_TX_DESC_CNT][ETH_TX_BUF_SIZE];

/* ============================================================
 * TCP state machine
 * ============================================================ */
#define TCP_STATE_CLOSED      0
#define TCP_STATE_LISTEN      1
#define TCP_STATE_SYN_RCVD    2
#define TCP_STATE_ESTABLISHED 3
#define TCP_STATE_LAST_ACK    4

typedef struct {
	u8  state;             /* TCP state */
	u32 remote_ip;         /* Remote IP (network order) */
	u16 remote_port;       /* Remote port (network order) */
	u32 local_seq;         /* Local sequence number */
	u32 remote_seq;        /* Remote sequence number */
	u32 remote_ack;        /* Last acknowledged remote seq */
	u8  tx_buf[2048];     /* TX buffer for outgoing data */
	u16 tx_len;            /* Bytes in TX buffer */
	u8  rx_buf[2048];     /* RX buffer for incoming data */
	u16 rx_len;            /* Bytes in RX buffer */
	u32 rx_callback;       /* User callback for received data */
} TcpConnection;

static TcpConnection g_tcp;
static u32 g_local_seq_init;        /* Initial sequence number */
static u32 g_arp_table_ip;          /* Cached ARP entry IP */
static u8  g_arp_table_mac[6];      /* Cached ARP entry MAC */
static u8  g_arp_valid;

/* RX callback function pointer */
static TcpRxCallback g_tcp_rx_cb = 0;

/* ============================================================
 * Hardware helpers
 * ============================================================ */

/* Reset LAN8720A PHY via PB0 */
void Eth_ResetPhy(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	gpio.GPIO_Pin  = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	/* LAN8720A reset: low → wait → high → wait */
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	delay_ms(10);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	delay_ms(10);
}

/* GPIO config for RMII pins */
static void Eth_Gpio_Init(void)
{
	GPIO_InitTypeDef gpio;

	/* ================================================================
	 * BUGFIX #1: Enable SYSCFG clock and select RMII mode.
	 * Without this, the MAC operates in MII mode while the board
	 * is wired for RMII → NO communication at all.
	 * ================================================================ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Select RMII mode: set bit 23 in SYSCFG->PMC.
	 * Use direct register write to avoid pulling in stm32f4xx_syscfg.c */
	SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

	/* Enable clocks for all GPIO ports used by RMII */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA
	                       | RCC_AHB1Periph_GPIOB
	                       | RCC_AHB1Periph_GPIOC
	                       | RCC_AHB1Periph_GPIOG, ENABLE);

	/* PA1: RMII_REF_CLK, PA2: MDIO, PA7: RMII_CRS_DV */
	gpio.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	/* PB11: RMII_TX_EN, PB12: RMII_TXD0, PB13: RMII_TXD1 */
	gpio.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	/* PC1: MDC, PC4: RMII_RXD0, PC5: RMII_RXD1 */
	gpio.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &gpio);

	/* Connect pins to AF11 (Ethernet) */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7,  GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,  GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4,  GPIO_AF_ETH);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource5,  GPIO_AF_ETH);
}

/* ============================================================
 * PHY (LAN8720A) Management Interface (MDIO / MDC)
 * ============================================================ */
#define PHY_ADDR_0      0x00   /* LAN8720A address when PHYAD0=L */
#define PHY_ADDR_1      0x01   /* LAN8720A address when PHYAD0=H */
#define PHY_BCR         0x00   /* Basic Control Register */
#define PHY_BSR         0x01   /* Basic Status Register */
#define PHY_ID1         0x02   /* PHY ID 1 */
#define PHY_ID2         0x03   /* PHY ID 2 */
#define PHY_SPECIAL     0x1F   /* Special Control/Status */

/* BUGFIX #2: Set Clock Range for HCLK=168MHz.
 * HCLK 150-200 MHz → CR = Div102 → MDC = 168/102 ≈ 1.65 MHz.
 * Default CR=0 gives MDC=168/42=4MHz, exceeding LAN8720A's 2.5MHz max
 * → unreliable MDIO read/write, PHY detection failure. */
#define MACMIIAR_CR_MASK  ((u32)0x0000003C)   /* Bits [5:2] */
#define MACMIIAR_CR_168MHZ ETH_MACMIIAR_CR_Div102  /* 0x00000010 */

static u32 Eth_ReadPhy(u16 reg)
{
	u32 timeout;

	/* Clear PA[15:11], MR[10:6], CR[5:2]; keep MB/MW */
	ETH->MACMIIAR = (ETH->MACMIIAR & ~((u32)0x00000FFC))
	                | ((u32)(g_phy_addr << 11) & ETH_MACMIIAR_PA)
	                | ((u32)(reg << 6) & ETH_MACMIIAR_MR)
	                | MACMIIAR_CR_168MHZ
	                | ETH_MACMIIAR_MB;
	/* Wait for completion */
	timeout = 0;
	while ((ETH->MACMIIAR & ETH_MACMIIAR_MB) && (timeout < 0x00010000))
	{
		timeout++;
	}
	return (u32)(ETH->MACMIIDR & 0xFFFF);
}

static void Eth_WritePhy(u16 reg, u16 value)
{
	u32 timeout;

	ETH->MACMIIAR = (ETH->MACMIIAR & ~((u32)0x00000FFC))
	                | ((u32)(g_phy_addr << 11) & ETH_MACMIIAR_PA)
	                | ((u32)(reg << 6) & ETH_MACMIIAR_MR)
	                | MACMIIAR_CR_168MHZ
	                | ETH_MACMIIAR_MW
	                | ETH_MACMIIAR_MB;
	ETH->MACMIIDR = (u32)value;
	timeout = 0;
	while ((ETH->MACMIIAR & ETH_MACMIIAR_MB) && (timeout < 0x00010000))
	{
		timeout++;
	}
}

/* BUGFIX #3: Auto-detect PHY address (some boards use 0, others use 1).
 * PHYAD0 pin on LAN8720A determines the address. Common boards:
 *   - Waveshare Open407Z:   addr 1
 *   - ALIENTEK Explorer:    addr 0
 *   - Most STM32F407 boards: addr 0 or 1
 *
 * Returns 1 if PHY found and link is up, 0 otherwise. */
static u8 Eth_PhyInit(void)
{
	u32 phy_id1, phy_id2, status;
	u8  addr;

	/* Try all possible PHY addresses */
	phy_id1 = 0xFFFF;
	for (addr = 0; addr <= 1; addr++)
	{
		g_phy_addr = addr;

		phy_id1 = Eth_ReadPhy(PHY_ID1);
		if ((phy_id1 != 0xFFFF) && (phy_id1 != 0x0000))
		{
			phy_id2 = Eth_ReadPhy(PHY_ID2);
			g_phy_id1 = phy_id1;
			g_phy_id2 = phy_id2;
			break;
		}
	}

	if ((phy_id1 == 0xFFFF) || (phy_id1 == 0x0000))
	{
		g_link_up   = 0;
		g_phy_addr  = 0xFF;
		g_eth_error = ETH_ERR_NO_PHY;
		g_phy_id1   = 0;
		g_phy_id2   = 0;
		return 0;
	}

	/* Wait for auto-negotiation to complete */
	{
		u32 to = 0;
		while (!(Eth_ReadPhy(PHY_BSR) & 0x0020) && (to < 0x000F0000))
			to++;
		if (to >= 0x000F0000)
		{
			g_link_up   = 0;
			g_eth_error = ETH_ERR_ANEG_TIMEOUT;
			return 0;
		}
	}

	/* Check link status */
	status = Eth_ReadPhy(PHY_BSR);
	if (!(status & 0x0004))
	{
		g_link_up   = 0;
		g_eth_error = ETH_ERR_NO_LINK;
		return 0;
	}

	g_link_up   = 1;
	g_eth_error = ETH_ERR_OK;
	return 1;
}

/* Public: get current link status */
u8 Eth_GetLinkStatus(void)
{
	/* Re-read live link status */
	if (g_link_up)
	{
		u32 status = Eth_ReadPhy(PHY_BSR);
		if (!(status & 0x0004))
		{
			g_link_up   = 0;
			g_eth_error = ETH_ERR_NO_LINK;
		}
	}
	return g_link_up;
}

/* Public: get last error code (serial-friendly diagnostic) */
u8 Eth_GetErrorCode(void)
{
	return g_eth_error;
}

/* Public: get detected PHY address (0xFF = not found) */
u8 Eth_GetPhyAddr(void)
{
	return g_phy_addr;
}

/* Public: fill buf with human-readable diagnostic string
 * Format: "LINK:1,PHY:0,ERR:0,PHYID:0007C0F1,RX:5,TX:5,ARP:2,ICMP:3"
 *   LINK:    1=link up, 0=link down
 *   PHY:     PHY address (0 or 1), or 255 if not found
 *   ERR:     error code (0=OK, see ETH_ERR_* defines)
 *   PHYID:   8 hex digits: PHY ID1<<16 | PHY ID2 (LAN8720A=0x0007C0F1)
 *   RX/TX:   total received / transmitted frames since init
 *   ARP:     ARP requests replied
 *   ICMP:    ICMP echo replies sent (ping responses)
 */
void Eth_GetDiag(char *buf, u16 max_len)
{
	u32 link, aneg;
	u8  err;

	if (!g_init_done)
	{
		if (buf && max_len > 32)
		{
			sprintf(buf, "LINK:0,PHY:%d,ERR:%d,PHYID:%08lX,RX:0,TX:0,ARP:0,ICMP:0",
			        g_phy_addr == 0xFF ? 255 : (int)g_phy_addr,
			        (int)g_eth_error,
			        (g_phy_id1 << 16) | g_phy_id2);
		}
		return;
	}

	/* Live re-read for most accurate PHY status */
	link = Eth_ReadPhy(PHY_BSR);
	aneg = (link & 0x0020) ? 1 : 0;
	link = (link & 0x0004) ? 1 : 0;
	err  = g_eth_error;

	if (buf && max_len > 48)
	{
		sprintf(buf, "LINK:%lu,PHY:%d,ERR:%d,PHYID:%08lX,RX:%u,TX:%u,ARP:%u,ICMP:%u",
		        link,
		        g_phy_addr == 0xFF ? 255 : (int)g_phy_addr,
		        (int)err,
		        (g_phy_id1 << 16) | g_phy_id2,
		        g_rx_cnt, g_tx_cnt, g_arp_cnt, g_icmp_cnt);
	}
}

/* ============================================================
 * MAC + DMA initialization
 * ============================================================ */
static void Eth_MacDma_Init(void)
{
	u32 i;

	/* Enable ETH peripheral clock & SYSCFG */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC
	                       | RCC_AHB1Periph_ETH_MAC_Tx
	                       | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);

	/* ---- Reset MAC and DMA ---- */
	/* First, perform a soft reset on the DMA */
	ETH->DMABMR |= ETH_DMABMR_SR;
	while (ETH->DMABMR & ETH_DMABMR_SR) {}

	/* ---- MAC Configuration ---- */
	/* Set MAC address */
	ETH->MACA0HR = ((u32)g_eth_mac[5] << 8)
	               | ((u32)g_eth_mac[4] << 0);
	ETH->MACA0LR = ((u32)g_eth_mac[3] << 24)
	               | ((u32)g_eth_mac[2] << 16)
	               | ((u32)g_eth_mac[1] << 8)
	               | ((u32)g_eth_mac[0] << 0);

	/* BUGFIX #4: MAC configuration with FES (100Mbps) for RMII.
	 * RMII operates at 100Mbps; without FES, MAC runs at 10Mbps
	 * while PHY auto-negotiates to 100Mbps → speed mismatch.
	 * Also added ROD (Receive Own Disable) to prevent loopback. */
	ETH->MACCR = ETH_MACCR_RE       /* Receiver enable */
	             | ETH_MACCR_TE       /* Transmitter enable */
	             | ETH_MACCR_APCS     /* Automatic pad/CRC stripping */
	             | ETH_MACCR_DM       /* Duplex mode (full) */
	             | ETH_MACCR_FES      /* Fast Ethernet Speed = 100Mbps */
	             | ETH_MACCR_ROD      /* Receive Own Disable */
	             | ETH_MACCR_IFG_96Bit; /* Interframe gap 96 bit times */

	/* Frame filter: accept all multicast/broadcast, pass all to application */
	ETH->MACFFR = ETH_MACFFR_RA;   /* Receive All */

	/* Flow control: disable */
	ETH->MACFCR = 0x00000000;

	/* ---- DMA Configuration ---- */
	/* Initialize RX descriptors */
	for (i = 0; i < ETH_RX_DESC_CNT; i++)
	{
		g_rx_desc[i].status  = ETH_DMARxDESC_OWN;  /* Owned by DMA */
		g_rx_desc[i].control = ETH_RX_BUF_SIZE;     /* Buffer size */
		g_rx_desc[i].buffer1 = (u32)&g_rx_buf[i][0];
		if (i < (ETH_RX_DESC_CNT - 1))
			g_rx_desc[i].buffer2_next = (u32)&g_rx_desc[i + 1];
		else
			g_rx_desc[i].buffer2_next = (u32)&g_rx_desc[0];  /* Wrap around */
	}

	/* Initialize TX descriptors */
	for (i = 0; i < ETH_TX_DESC_CNT; i++)
	{
		g_tx_desc[i].status  = 0;  /* Not owned by DMA */
		g_tx_desc[i].buffer1 = (u32)&g_tx_buf[i][0];
		if (i < (ETH_TX_DESC_CNT - 1))
			g_tx_desc[i].buffer2_next = (u32)&g_tx_desc[i + 1];
		else
			g_tx_desc[i].buffer2_next = (u32)&g_tx_desc[0];  /* Wrap around */
	}

	/* DMA bus mode: store-and-forward, little-endian descriptor */
	ETH->DMABMR = ETH_DMABMR_AAB        /* Address-aligned beats */
	              | ETH_DMABMR_FB          /* Fixed burst */
	              | ETH_DMABMR_PBL_32Beat; /* 32-beat programmable burst length */

	/* DMA operation mode */
	ETH->DMAOMR = ETH_DMAOMR_RSF     /* RX store-and-forward */
	              | ETH_DMAOMR_TSF     /* TX store-and-forward */
	              | ETH_DMAOMR_OSF     /* Operate on second frame */
	              | ETH_DMAOMR_FEF;    /* Forward error frames */

	/* Set descriptor addresses */
	ETH->DMARDLAR = (u32)&g_rx_desc[0];
	ETH->DMATDLAR = (u32)&g_tx_desc[0];

	/* Enable interrupts: normal interrupt summary + receive */
	ETH->DMAIER = ETH_DMAIER_NISE    /* Normal interrupt */
	              | ETH_DMAIER_RIE;    /* Receive interrupt */

	/* Enable DMA TX and RX */
	ETH->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;

	/* Start MAC transmission and reception */
	ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
}

/* ============================================================
 * Public: Initialize everything
 *
 * Init order matters:
 *   1. GPIO + SYSCFG (RMII mode selection)
 *   2. Enable ETH MAC clocks (so MDIO registers are accessible)
 *   3. PHY reset + init (MDIO read/write → auto-detect addr)
 *   4. Full MAC + DMA init (requires PHY to be ready)
 * ============================================================ */
u8 Eth_Init(void)
{
	Eth_Gpio_Init();
	Eth_ResetPhy();

	/* BUGFIX #5: Enable ETH MAC clocks BEFORE PHY init.
	 * MDIO (MACMIIAR, MACMIIDR) registers are inside the ETH MAC
	 * peripheral and are inaccessible without clock. Previously
	 * Eth_PhyInit() was called before Eth_MacDma_Init() which
	 * enables the clock → PHY reads returned garbage. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC
	                       | RCC_AHB1Periph_ETH_MAC_Tx
	                       | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);

	/* Reset packet counters */
	g_rx_cnt   = 0;
	g_tx_cnt   = 0;
	g_arp_cnt  = 0;
	g_icmp_cnt = 0;

	/* PHY init (MDIO communication) */
	if (!Eth_PhyInit())
	{
		g_init_done = 0;
		return 0;   /* PHY init failed (error code set in g_eth_error) */
	}

	/* Full MAC + DMA configuration */
	Eth_MacDma_Init();
	g_init_done = 1;
	return 1;
}

/* ============================================================
 * Send raw Ethernet frame
 * ============================================================ */
void Eth_SendFrame(u8 *buffer, u16 len)
{
	static u8 tx_idx = 0;
	u32 timeout;

	/* Wait for descriptor to be free */
	timeout = 0;
	while ((g_tx_desc[tx_idx].status & ETH_DMATxDESC_OWN)
	        && (timeout < 0x00100000))
	{
		timeout++;
	}

	if (g_tx_desc[tx_idx].status & ETH_DMATxDESC_OWN)
	{
		return;   /* All TX descriptors busy, drop frame */
	}

	/* Copy frame to TX buffer */
	memcpy((void *)g_tx_desc[tx_idx].buffer1, buffer, len);

	/* Set descriptor: give ownership to DMA, set buffer size, mark as first+last */
	g_tx_desc[tx_idx].control = (len & 0x0FFF)
	                            | ETH_DMATxDESC_FS      /* First segment */
	                            | ETH_DMATxDESC_LS       /* Last segment */
	                            | ETH_DMATxDESC_IC       /* Interrupt on completion */
	                            | ETH_DMATxDESC_OWN;     /* DMA owns this descriptor */

	/* Advance TX index */
	tx_idx = (tx_idx + 1) % ETH_TX_DESC_CNT;

	/* Wake up DMA if it's suspended */
	if (ETH->DMASR & ETH_DMASR_TBUS)
	{
		ETH->DMASR = ETH_DMASR_TBUS;
		ETH->DMATPDR = 0;
	}

	g_tx_cnt++;
}

/* ============================================================
 * Receive raw Ethernet frame
 * Returns frame length, or 0 if no frame available
 * ============================================================ */
static u8 g_rx_idx = 0;

u16 Eth_RecvFrame(u8 *buffer, u16 max_len)
{
	u32 status;
	u16 len;

	/* Check if current descriptor has a frame (not owned by DMA) */
	status = g_rx_desc[g_rx_idx].status;
	if (status & ETH_DMARxDESC_OWN)
	{
		return 0;   /* No frame */
	}

	/* Check for errors */
	if ((status & ETH_DMARxDESC_ES)       /* Error summary */
	    || !(status & ETH_DMARxDESC_LS)    /* Last descriptor */
	    || !(status & ETH_DMARxDESC_FS))   /* First descriptor */
	{
		/* Error: reset descriptor and skip */
		g_rx_desc[g_rx_idx].status = ETH_DMARxDESC_OWN;
		g_rx_idx = (g_rx_idx + 1) % ETH_RX_DESC_CNT;
		return 0;
	}

	/* Get frame length.
	 * BUGFIX #6: MACCR APCS (Automatic Pad/CRC Stripping) is enabled,
	 * so the MAC already strips the 4-byte CRC before DMA.
	 * RDES0 FL field reports length WITHOUT CRC → do NOT subtract 4.
	 * Subtracting 4 truncates every frame by 4 bytes, corrupting
	 * ICMP data → bad checksum → ping silently dropped by PC. */
	len = (u16)((status & 0x3FFF0000) >> 16);

	if (len > max_len) len = max_len;

	/* Copy frame data */
	memcpy(buffer, (void *)g_rx_desc[g_rx_idx].buffer1, len);

	/* Return descriptor to DMA */
	g_rx_desc[g_rx_idx].status = ETH_DMARxDESC_OWN;
	g_rx_idx = (g_rx_idx + 1) % ETH_RX_DESC_CNT;

	/* Update tail pointer to avoid FIFO overflow */
	ETH->DMARPDR = 0;

	g_rx_cnt++;
	return len;
}

/* ============================================================
 * Checksum utilities
 * ============================================================ */
u16 Eth_Checksum16(const void *data, u16 len)
{
	u32 sum = 0;
	const u8 *p = (const u8 *)data;
	u16 i;

	for (i = 0; i < len; i += 2)
	{
		u16 word = (u16)p[i] << 8;
		if ((i + 1) < len)
			word |= p[i + 1];
		sum += word;
	}

	/* Add carry */
	while (sum >> 16)
		sum = (sum & 0xFFFF) + (sum >> 16);

	return (u16)(~sum);
}

/* IP header checksum (uses only the header, not pseudo-header) */
u16 Eth_IpChecksum(const void *iphdr)
{
	const u8 *p = (const u8 *)iphdr;
	u8 ihl = (*p & 0x0F) * 4;   /* IHL in bytes */
	return Eth_Checksum16(iphdr, ihl);
}

/* TCP pseudo-header + TCP segment checksum */
static u16 TcpChecksum(const u8 *src_ip, const u8 *dst_ip,
                       const u8 *tcp_data, u16 tcp_len)
{
	u32 sum = 0;
	u16 i;

	/* Pseudo-header */
	sum += ((u16)src_ip[0] << 8) | src_ip[1];
	sum += ((u16)src_ip[2] << 8) | src_ip[3];
	sum += ((u16)dst_ip[0] << 8) | dst_ip[1];
	sum += ((u16)dst_ip[2] << 8) | dst_ip[3];
	sum += 6;            /* Protocol = TCP */
	sum += tcp_len;

	/* TCP header + data */
	for (i = 0; i < tcp_len; i += 2)
	{
		u16 word = (u16)tcp_data[i] << 8;
		if ((i + 1) < tcp_len)
			word |= tcp_data[i + 1];
		sum += word;
	}

	while (sum >> 16)
		sum = (sum & 0xFFFF) + (sum >> 16);

	return (u16)(~sum);
}

/* ============================================================
 * ARP: Handle ARP request, send reply
 * ============================================================ */
static void Eth_HandleArp(const u8 *frame, u16 len)
{
	const EthHeader  *eth = (const EthHeader *)frame;
	const ArpPacket  *arp = (const ArpPacket *)(frame + sizeof(EthHeader));
	ArpPacket        *arp_reply;
	u8               *reply_frame;
	u8               reply_buf[64];

	if (len < (sizeof(EthHeader) + sizeof(ArpPacket)))
		return;

	/* Only handle ARP request (oper=1) for our IP */
	if ((arp->oper != htons(1))
	    || (arp->tpa[0] != g_eth_ip[0])
	    || (arp->tpa[1] != g_eth_ip[1])
	    || (arp->tpa[2] != g_eth_ip[2])
	    || (arp->tpa[3] != g_eth_ip[3]))
	{
		return;
	}

	/* Cache remote MAC/IP for future use */
	g_arp_table_ip = *((u32 *)arp->spa);
	memcpy(g_arp_table_mac, arp->sha, 6);
	g_arp_valid = 1;

	/* Build ARP reply */
	reply_frame = reply_buf;
	memcpy(reply_frame, eth, sizeof(EthHeader) + sizeof(ArpPacket));
	eth   = (const EthHeader *)reply_frame;
	arp_reply = (ArpPacket *)(reply_frame + sizeof(EthHeader));

	/* Set destination MAC to requester's MAC, source to ours */
	memcpy(reply_frame, arp->sha, 6);         /* Dest = requester */
	memcpy(reply_frame + 6, g_eth_mac, 6);    /* Src = us */
	arp_reply->oper = htons(2);               /* Reply */
	memcpy(arp_reply->tha, arp->sha, 6);      /* Target HW = requester */
	memcpy(arp_reply->tpa, arp->spa, 4);      /* Target IP = requester */
	memcpy(arp_reply->sha, g_eth_mac, 6);     /* Sender HW = us */
	memcpy(arp_reply->spa, g_eth_ip, 4);      /* Sender IP = us */

	Eth_SendFrame(reply_buf, sizeof(EthHeader) + sizeof(ArpPacket));
	g_arp_cnt++;
}

/* ============================================================
 * ICMP: Handle echo request (ping), send echo reply
 * ============================================================ */
static void Eth_HandleIcmp(const u8 *frame, u16 len,
                           const u8 *src_ip, const u8 *dst_ip)
{
	u8             reply[2048];
	u8             *reply_buf;
	IcmpEchoHeader *icmp_reply;
	IpHeader       *ip_reply;
	EthHeader      *eth_reply;
	u16            icmp_len, ip_payload_len;

	icmp_len = len - sizeof(EthHeader) - sizeof(IpHeader);
	if (icmp_len < sizeof(IcmpEchoHeader))
		return;

	{
		const IcmpEchoHeader *icmp = (const IcmpEchoHeader *)
			(frame + sizeof(EthHeader) + sizeof(IpHeader));

		if (icmp->type != 8)   /* Only handle echo request */
			return;
	}

	/* Build echo reply */
	reply_buf = reply;

	/* Copy full frame; we will modify in place */
	memcpy(reply_buf, frame, len);

	eth_reply = (EthHeader *)reply_buf;
	ip_reply  = (IpHeader *)(reply_buf + sizeof(EthHeader));

	/* Swap MAC */
	memcpy(eth_reply->dest_mac, eth_reply->src_mac, 6);
	memcpy(eth_reply->src_mac, g_eth_mac, 6);

	/* Swap IP */
	memcpy(ip_reply->dst_ip, ip_reply->src_ip, 4);
	memcpy(ip_reply->src_ip, g_eth_ip, 4);
	ip_reply->ttl = 64;
	ip_reply->hdr_checksum = 0;
	ip_reply->hdr_checksum = Eth_IpChecksum(ip_reply);

	/* ICMP: change type to 0 (echo reply), recalculate checksum */
	icmp_reply = (IcmpEchoHeader *)(reply_buf + sizeof(EthHeader) + sizeof(IpHeader));
	icmp_reply->type = 0;          /* Echo reply */
	icmp_reply->checksum = 0;
	ip_payload_len = ntohs(ip_reply->total_len) - sizeof(IpHeader);
	icmp_reply->checksum = Eth_Checksum16(icmp_reply, ip_payload_len);

	Eth_SendFrame(reply_buf, len);
	g_icmp_cnt++;
}

/* ============================================================
 * TCP input processing
 * ============================================================ */
static void Tcp_SendPacket(u32 dst_ip, u16 dst_port,
                           u8 flags, const u8 *data, u16 data_len);

static void Eth_HandleTcp(const u8 *frame, u16 len,
                          const u8 *src_ip, const u8 *dst_ip)
{
	const IpHeader *iphdr    = (const IpHeader *)(frame + sizeof(EthHeader));
	const TcpHeader *tcphdr  = (const TcpHeader *)(frame + sizeof(EthHeader)
	                                               + sizeof(IpHeader));
	u16       ip_total_len;
	u16       tcp_len;
	u16       data_offset;
	u8        flags;
	u32       remote_ip;
	u16       remote_port;

	ip_total_len = ntohs(iphdr->total_len);
	tcp_len      = ip_total_len - sizeof(IpHeader);
	data_offset  = (tcphdr->data_offset >> 4) * 4;
	flags        = tcphdr->flags;
	remote_ip    = *((u32 *)src_ip);
	remote_port  = tcphdr->src_port;

	/* ---- RST: reset connection ---- */
	if (flags & TCP_FLAG_RST)
	{
		if (g_tcp.state != TCP_STATE_CLOSED
		    && g_tcp.remote_ip == remote_ip
		    && g_tcp.remote_port == remote_port)
		{
			g_tcp.state = TCP_STATE_CLOSED;
		}
		return;
	}

	/* ---- SYN received ---- */
	if ((flags & TCP_FLAG_SYN) && !(flags & TCP_FLAG_ACK))
	{
		if (g_tcp.state == TCP_STATE_LISTEN
		    && ntohs(tcphdr->dst_port) == g_tcp_port)
		{
			/* Accept connection: SYN → SYN-ACK */
			g_tcp.state       = TCP_STATE_SYN_RCVD;
			g_tcp.remote_ip   = remote_ip;
			g_tcp.remote_port = remote_port;
			g_tcp.remote_seq  = ntohl(tcphdr->seq_num) + 1;
			g_local_seq_init  = 0x12345678;   /* Could use random */
			g_tcp.local_seq   = g_local_seq_init;
			g_tcp.rx_len      = 0;
			g_tcp.tx_len      = 0;

			Tcp_SendPacket(remote_ip, remote_port,
			               TCP_FLAG_SYN | TCP_FLAG_ACK, 0, 0);
			g_tcp.local_seq++;
		}
		else if (g_tcp.state != TCP_STATE_CLOSED
		         && g_tcp.remote_ip == remote_ip
		         && g_tcp.remote_port == remote_port)
		{
			/* Duplicate SYN — resend SYN-ACK */
			g_tcp.local_seq = g_local_seq_init;
			Tcp_SendPacket(remote_ip, remote_port,
			               TCP_FLAG_SYN | TCP_FLAG_ACK, 0, 0);
			g_tcp.local_seq++;
		}
		return;
	}

	/* ---- Not our connection ---- */
	if (g_tcp.state == TCP_STATE_CLOSED
	    || g_tcp.remote_ip != remote_ip
	    || g_tcp.remote_port != remote_port)
	{
		/* Send RST unless it's a RST itself */
		Tcp_SendPacket(remote_ip, remote_port, TCP_FLAG_RST, 0, 0);
		return;
	}

	/* ---- ACK: process acknowledgment ---- */
	if (flags & TCP_FLAG_ACK)
	{
		u32 ack = ntohl(tcphdr->ack_num);

		/* Remove acknowledged data from TX buffer */
		if (g_tcp.tx_len > 0 && ack != g_tcp.local_seq)
		{
			u32 acked = ack - g_tcp.local_seq;
			if (acked <= g_tcp.tx_len)
			{
				memmove(g_tcp.tx_buf, g_tcp.tx_buf + acked,
				        g_tcp.tx_len - acked);
				g_tcp.tx_len -= (u16)acked;
			}
		}

		/* Update remote sequence from SYN-ACK handshake */
		if (g_tcp.state == TCP_STATE_SYN_RCVD)
		{
			g_tcp.state = TCP_STATE_ESTABLISHED;
			g_tcp.remote_seq = ntohl(tcphdr->seq_num);
		}

		/* ---- Data in this segment ---- */
		if (tcp_len > data_offset)
		{
			const u8 *tcp_data = (const u8 *)tcphdr + data_offset;
			u16      seg_len   = tcp_len - data_offset;
			u32      seq       = ntohl(tcphdr->seq_num);
			u32      expected  = g_tcp.remote_seq;

			if (seq == expected)
			{
				/* In-order data: append to RX buffer */
				if (g_tcp.rx_len + seg_len <= sizeof(g_tcp.rx_buf))
				{
					memcpy(g_tcp.rx_buf + g_tcp.rx_len,
					       tcp_data, seg_len);
					g_tcp.rx_len += seg_len;
				}
				g_tcp.remote_seq += seg_len;

				/* ACK the received data */
				Tcp_SendPacket(remote_ip, remote_port,
				               TCP_FLAG_ACK, 0, 0);

				/* Notify user callback */
				if (g_tcp_rx_cb && g_tcp.rx_len > 0)
				{
					g_tcp_rx_cb(g_tcp.rx_buf, g_tcp.rx_len);
					g_tcp.rx_len = 0;
				}
			}
			else
			{
				/* Out-of-order: send ACK with expected sequence */
				/* Set ack_num to expected sequence */
				Tcp_SendPacket(remote_ip, remote_port,
				               TCP_FLAG_ACK, 0, 0);
			}
		}
	}

	/* ---- FIN: connection close ---- */
	if (flags & TCP_FLAG_FIN)
	{
		g_tcp.remote_seq++;
		Tcp_SendPacket(remote_ip, remote_port,
		               TCP_FLAG_ACK, 0, 0);
		if (g_tcp.state == TCP_STATE_ESTABLISHED)
		{
			g_tcp.state = TCP_STATE_LAST_ACK;
		}
		else if (g_tcp.state == TCP_STATE_LAST_ACK)
		{
			g_tcp.state = TCP_STATE_CLOSED;
			g_tcp.tx_len = 0;
			g_tcp.rx_len = 0;
		}
	}
}

/* ---- Send a TCP packet ---- */
static void Tcp_SendPacket(u32 dst_ip, u16 dst_port,
                           u8 flags, const u8 *data, u16 data_len)
{
	u8        frame_buf[2048];
	EthHeader *eth;
	IpHeader  *ip;
	TcpHeader *tcp;
	u8        *tcp_data;
	u16       tcp_hdr_len, total_ip_len, total_frame_len;

	/* Check ARP cache */
	if (!g_arp_valid || g_arp_table_ip != dst_ip)
	{
		return;   /* Need ARP first; data will be retransmitted */
	}

	tcp_hdr_len     = 20;   /* TCP header without options */
	total_ip_len    = sizeof(IpHeader) + tcp_hdr_len + data_len;
	total_frame_len = sizeof(EthHeader) + total_ip_len;

	if (total_frame_len > sizeof(frame_buf))
		return;

	eth      = (EthHeader *)frame_buf;
	ip       = (IpHeader *)(frame_buf + sizeof(EthHeader));
	tcp      = (TcpHeader *)(frame_buf + sizeof(EthHeader) + sizeof(IpHeader));
	tcp_data = frame_buf + sizeof(EthHeader) + sizeof(IpHeader) + tcp_hdr_len;

	/* Fill Ethernet header */
	memcpy(eth->dest_mac, g_arp_table_mac, 6);
	memcpy(eth->src_mac,  g_eth_mac, 6);
	eth->ethertype = htons(0x0800);   /* IPv4 */

	/* Fill IP header */
	memset(ip, 0, sizeof(IpHeader));
	ip->ver_ihl   = 0x45;    /* Version 4, IHL = 5 */
	ip->total_len = htons(total_ip_len);
	ip->id        = 0;
	ip->ttl       = 64;
	ip->protocol  = 6;       /* TCP */
	memcpy(ip->src_ip, g_eth_ip, 4);
	memcpy(ip->dst_ip, (u8 *)&dst_ip, 4);
	ip->hdr_checksum = 0;
	ip->hdr_checksum = Eth_IpChecksum(ip);

	/* Fill TCP header */
	memset(tcp, 0, tcp_hdr_len);
	tcp->src_port   = htons(g_tcp_port);
	tcp->dst_port   = dst_port;
	tcp->seq_num    = htonl(g_tcp.local_seq);
	tcp->ack_num    = htonl(g_tcp.remote_seq);
	tcp->data_offset = (tcp_hdr_len / 4) << 4;   /* Data offset in 4-byte words */
	tcp->flags      = flags;
	tcp->window     = htons(4096);

	/* Copy data */
	if (data && data_len > 0)
		memcpy(tcp_data, data, data_len);

	/* Calculate TCP checksum */
	tcp->checksum = TcpChecksum(g_eth_ip, (u8 *)&dst_ip,
	                            (u8 *)tcp, tcp_hdr_len + data_len);

	Eth_SendFrame(frame_buf, total_frame_len);
}

/* ============================================================
 * Main processing: call from main loop
 * ============================================================ */
void Eth_Process(void)
{
	u8     frame[2048];
	u16    len;
	EthHeader *eth;
	u16    ethertype;

	while ((len = Eth_RecvFrame(frame, sizeof(frame))) > 0)
	{
		if (len < sizeof(EthHeader))
			continue;

		eth = (EthHeader *)frame;
		ethertype = ntohs(eth->ethertype);

		/* Must be for us or broadcast */
		if (!(eth->dest_mac[0] & 0x01)     /* Not multicast/broadcast */
		    && memcmp(eth->dest_mac, g_eth_mac, 6) != 0)
			continue;

		switch (ethertype)
		{
		case 0x0806:   /* ARP */
			Eth_HandleArp(frame, len);
			break;

		case 0x0800:   /* IPv4 */
			{
				const IpHeader *iphdr = (const IpHeader *)
					(frame + sizeof(EthHeader));
				u8 *src_ip = (u8 *)iphdr->src_ip;
				u8 *dst_ip = (u8 *)iphdr->dst_ip;

				/* Check: is this for our IP? */
				if (dst_ip[0] != g_eth_ip[0]
				    || dst_ip[1] != g_eth_ip[1]
				    || dst_ip[2] != g_eth_ip[2]
				    || dst_ip[3] != g_eth_ip[3])
					break;

				switch (iphdr->protocol)
				{
				case 1:   /* ICMP */
					Eth_HandleIcmp(frame, len, src_ip, dst_ip);
					break;

				case 6:   /* TCP */
					Eth_HandleTcp(frame, len, src_ip, dst_ip);
					break;

				default:
					break;
				}
			}
			break;

		default:
			break;
		}
	}

	/* Retransmit pending TCP data if any */
	if (g_tcp.state == TCP_STATE_ESTABLISHED
	    && g_tcp.tx_len > 0
	    && g_arp_valid)
	{
		/* Simple: just try to send (lossy, but good enough for telemetry) */
		Tcp_SendPacket(g_tcp.remote_ip, g_tcp.remote_port,
		               TCP_FLAG_PSH | TCP_FLAG_ACK,
		               g_tcp.tx_buf, g_tcp.tx_len);
	}
}

/* ============================================================
 * TCP Server Public API
 * ============================================================ */
u16 Tcp_Send(const u8 *data, u16 len)
{
	u16 space;

	if (g_tcp.state != TCP_STATE_ESTABLISHED || len == 0)
		return 0;

	space = sizeof(g_tcp.tx_buf) - g_tcp.tx_len;
	if (len > space) len = space;
	if (len == 0) return 0;

	memcpy(g_tcp.tx_buf + g_tcp.tx_len, data, len);
	g_tcp.tx_len += len;

	/* Try to send immediately */
	if (g_arp_valid)
	{
		Tcp_SendPacket(g_tcp.remote_ip, g_tcp.remote_port,
		               TCP_FLAG_PSH | TCP_FLAG_ACK,
		               g_tcp.tx_buf, g_tcp.tx_len);
	}

	return len;
}

u8 Tcp_IsConnected(void)
{
	return (g_tcp.state == TCP_STATE_ESTABLISHED) ? 1 : 0;
}

u16 Tcp_Available(void)
{
	return g_tcp.rx_len;
}

u16 Tcp_Read(u8 *buf, u16 max_len)
{
	u16 to_read;

	if (g_tcp.rx_len == 0 || buf == 0) return 0;

	to_read = (g_tcp.rx_len < max_len) ? g_tcp.rx_len : max_len;
	memcpy(buf, g_tcp.rx_buf, to_read);

	/* Shift remaining data */
	if (to_read < g_tcp.rx_len)
	{
		memmove(g_tcp.rx_buf, g_tcp.rx_buf + to_read,
		        g_tcp.rx_len - to_read);
	}
	g_tcp.rx_len -= to_read;

	return to_read;
}

void Tcp_SetRxCallback(TcpRxCallback cb)
{
	g_tcp_rx_cb = cb;
}
