#ifndef __ETH_H
#define __ETH_H

#include "sys.h"

/* ============================================================
 * STM32F407 + LAN8720A RMII Ethernet Driver
 *
 * Pin Map (RMII):
 *   PA1  - ETH_RMII_REF_CLK  (50MHz clock input)
 *   PA2  - ETH_MDIO           (management data I/O)
 *   PA7  - ETH_RMII_CRS_DV   (carrier sense / data valid)
 *   PC1  - ETH_MDC            (management clock)
 *   PC4  - ETH_RMII_RXD0     (receive data bit 0)
 *   PC5  - ETH_RMII_RXD1     (receive data bit 1)
 *   PB11 - ETH_RMII_TX_EN    (transmit enable)
 *   PB12 - ETH_RMII_TXD0     (transmit data bit 0)
 *   PB13 - ETH_RMII_TXD1     (transmit data bit 1)
 *   PB0  - ETH_RESET          (LAN8720A reset, active low)
 * ============================================================ */

#define ETH_RX_BUF_SIZE   1536   /* Max Ethernet frame */
#define ETH_TX_BUF_SIZE   1536
#define ETH_RX_DESC_CNT   4      /* Number of RX descriptors */
#define ETH_TX_DESC_CNT   4      /* Number of TX descriptors */

/* ---- ETH DMA Descriptor bit definitions (补全, 不在 stm32f4xx.h 中) ---- */
#define ETH_DMARxDESC_OWN    ((u32)0x80000000)  /* RDES0[31]: Own bit */
#define ETH_DMARxDESC_ES     ((u32)0x00008000)  /* RDES0[15]: Error Summary */
#define ETH_DMARxDESC_FS     ((u32)0x00000200)  /* RDES0[9]:  First Descriptor */
#define ETH_DMARxDESC_LS     ((u32)0x00000100)  /* RDES0[8]:  Last Descriptor */
#define ETH_DMARxDESC_RCH    ((u32)0x00004000)  /* RDES1[14]: Second address chained */

#define ETH_DMATxDESC_OWN    ((u32)0x80000000)  /* TDES0[31]: Own bit */
#define ETH_DMATxDESC_IC     ((u32)0x40000000)  /* TDES0[30]: Interrupt on Completion */
#define ETH_DMATxDESC_FS     ((u32)0x20000000)  /* TDES0[29]: First Segment */
#define ETH_DMATxDESC_LS     ((u32)0x10000000)  /* TDES0[28]: Last Segment */
#define ETH_DMATxDESC_TCH    ((u32)0x00100000)  /* TDES0[20]: Second address chained */

/* Ethernet frame header */
typedef __packed struct {
	u8  dest_mac[6];
	u8  src_mac[6];
	u16 ethertype;        /* Big-endian */
} EthHeader;

/* ARP packet */
typedef __packed struct {
	u16 htype;            /* Hardware type (1=Ethernet) */
	u16 ptype;            /* Protocol type (0x0800=IPv4) */
	u8  hlen;             /* Hardware address length */
	u8  plen;             /* Protocol address length */
	u16 oper;             /* Operation (1=request, 2=reply) */
	u8  sha[6];           /* Sender hardware address */
	u8  spa[4];           /* Sender protocol address */
	u8  tha[6];           /* Target hardware address */
	u8  tpa[4];           /* Target protocol address */
} ArpPacket;

/* IP header (minimal, no options) */
typedef __packed struct {
	u8  ver_ihl;          /* Version(4) + IHL(4) */
	u8  dscp_ecn;
	u16 total_len;
	u16 id;
	u16 flags_frag;
	u8  ttl;
	u8  protocol;
	u16 hdr_checksum;
	u8  src_ip[4];
	u8  dst_ip[4];
} IpHeader;

/* ICMP echo header */
typedef __packed struct {
	u8  type;             /* 8=echo request, 0=echo reply */
	u8  code;
	u16 checksum;
	u16 identifier;
	u16 sequence;
} IcmpEchoHeader;

/* TCP header */
typedef __packed struct {
	u16 src_port;
	u16 dst_port;
	u32 seq_num;
	u32 ack_num;
	u8  data_offset;      /* High nibble: data offset in 4-byte words */
	u8  flags;
	u16 window;
	u16 checksum;
	u16 urgent_ptr;
} TcpHeader;

/* TCP flags */
#define TCP_FLAG_FIN  0x01
#define TCP_FLAG_SYN  0x02
#define TCP_FLAG_RST  0x04
#define TCP_FLAG_PSH  0x08
#define TCP_FLAG_ACK  0x10
#define TCP_FLAG_URG  0x20

/* ============================================================
 * Public API
 * ============================================================ */

/* ---- Ethernet error codes (for serial diagnostics) ---- */
#define ETH_ERR_OK              0x00   /* No error, link up */
#define ETH_ERR_NO_PHY          0x01   /* PHY not responding at any address */
#define ETH_ERR_NO_LINK         0x02   /* PHY found but no cable link */
#define ETH_ERR_ANEG_TIMEOUT    0x03   /* Auto-negotiation timed out */
#define ETH_ERR_NOT_INITED      0x04   /* Eth_Init() was never called */

/* MAC address (6 bytes), configurable before Eth_Init */
extern u8 g_eth_mac[6];

/* Local IP address (4 bytes), configurable before Eth_Init */
extern u8 g_eth_ip[4];
extern u8 g_eth_gateway[4];
extern u8 g_eth_netmask[4];

/* TCP server port (configurable before Eth_Init) */
extern u16 g_tcp_port;

/* Ethernet DMA descriptor (32 bytes, must be 4-byte aligned) */
typedef __align(4) struct {
	volatile u32 status;
	u32 control;
	u32 buffer1;
	u32 buffer2_next;     /* Next descriptor pointer or second buffer */
	u32 ext_status;
	u32 reserved1;
	u32 time_stamp_low;
	u32 time_stamp_high;
} EthDmaDesc;

/* ---- API ---- */
void Eth_ResetPhy(void);
u8   Eth_Init(void);
u8   Eth_GetLinkStatus(void);  /* Returns 1 if Ethernet link is up */
u8   Eth_GetErrorCode(void);   /* Returns ETH_ERR_* code for diagnostics */
u8   Eth_GetPhyAddr(void);     /* Returns detected PHY address (0 or 1) */
void Eth_GetDiag(char *buf, u16 max_len); /* Fill buf with diagnostic string */
void Eth_SendFrame(u8 *buffer, u16 len);
u16  Eth_RecvFrame(u8 *buffer, u16 max_len);

/* Network stack processing — call periodically from main loop */
void Eth_Process(void);

/* TCP server: send data to connected client. Returns bytes queued. */
u16  Tcp_Send(const u8 *data, u16 len);

/* TCP server: check if a client is connected */
u8   Tcp_IsConnected(void);

/* TCP server: get number of bytes available to read */
u16  Tcp_Available(void);

/* TCP server: read received data into buffer, returns bytes read */
u16  Tcp_Read(u8 *buf, u16 max_len);

/* Receive callback: user registers a function to process incoming TCP data */
typedef void (*TcpRxCallback)(const u8 *data, u16 len);
void Tcp_SetRxCallback(TcpRxCallback cb);

/* ============================================================
 * Utility: checksum, byte swap
 * ============================================================ */
u16  Eth_Checksum16(const void *data, u16 len);
u16  Eth_IpChecksum(const void *iphdr);

/* Convert u16/u32 between host and network byte order */
static __inline u16 htons(u16 x) { return (x >> 8) | (x << 8); }
static __inline u32 htonl(u32 x) {
	return ((x >> 24) & 0xFF) | ((x >> 8) & 0xFF00)
	     | ((x << 8) & 0xFF0000) | ((x << 24) & 0xFF000000);
}
#define ntohs(x) htons(x)
#define ntohl(x) htonl(x)

#endif /* __ETH_H */
