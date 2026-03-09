// udp.c — Minimal UDP/IP/ARP stack for bare-metal Teensy 4.1
// Handles: ARP request/reply, IPv4 (no fragmentation), UDP, ICMP echo reply
// Single-entry ARP cache, static IP only.

#if !NET_ENABLED
typedef int _udp_unused;
#else

#include "udp.h"
#include "enet.h"
#include <string.h>

// ---- Protocol constants ----
#define ETH_ALEN       6
#define ETH_HLEN       14
#define ETH_TYPE_ARP   0x0806
#define ETH_TYPE_IPV4  0x0800

#define IP_PROTO_ICMP  1
#define IP_PROTO_UDP   17

#define ARP_OP_REQUEST 1
#define ARP_OP_REPLY   2

// ---- Network byte order helpers ----
static inline uint16_t htons(uint16_t v) { return __builtin_bswap16(v); }
static inline uint32_t htonl(uint32_t v) { return __builtin_bswap32(v); }
static inline uint16_t ntohs(uint16_t v) { return __builtin_bswap16(v); }
static inline uint32_t ntohl(uint32_t v) { return __builtin_bswap32(v); }

// Read big-endian values from unaligned memory
static inline uint16_t rd16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static inline uint32_t rd32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

// Write big-endian values to unaligned memory
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline void wr32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

// ---- State ----
static uint32_t our_ip;
static uint32_t our_netmask;
static uint32_t our_gateway;
static uint8_t  our_mac[ETH_ALEN];

// Single-entry ARP cache
static uint32_t arp_cache_ip;
static uint8_t  arp_cache_mac[ETH_ALEN];
static bool     arp_cache_valid;

// TX frame buffer (reused for all outgoing frames)
static uint8_t tx_frame[1518];

// ---- Internet checksum (RFC 1071) ----
static uint16_t ip_checksum(const void *data, uint16_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t sum = 0;
	for (uint16_t i = 0; i < len - 1; i += 2)
		sum += ((uint32_t)p[i] << 8) | p[i + 1];
	if (len & 1)
		sum += (uint32_t)p[len - 1] << 8;
	while (sum >> 16)
		sum = (sum & 0xFFFF) + (sum >> 16);
	return (uint16_t)~sum;
}

// ---- Ethernet frame helpers ----

static void eth_build_header(uint8_t *frame, const uint8_t *dst_mac, uint16_t ethertype)
{
	memcpy(frame, dst_mac, ETH_ALEN);
	memcpy(frame + ETH_ALEN, our_mac, ETH_ALEN);
	wr16(frame + 12, ethertype);
}

// ---- ARP handling ----

static void arp_send_reply(const uint8_t *target_mac, uint32_t target_ip)
{
	eth_build_header(tx_frame, target_mac, ETH_TYPE_ARP);
	uint8_t *arp = tx_frame + ETH_HLEN;
	wr16(arp + 0, 1);             // hardware type: Ethernet
	wr16(arp + 2, ETH_TYPE_IPV4); // protocol type: IPv4
	arp[4] = 6;                   // hardware addr len
	arp[5] = 4;                   // protocol addr len
	wr16(arp + 6, ARP_OP_REPLY);
	memcpy(arp + 8,  our_mac, ETH_ALEN);
	wr32(arp + 14, our_ip);
	memcpy(arp + 18, target_mac, ETH_ALEN);
	wr32(arp + 24, target_ip);
	enet_tx(tx_frame, ETH_HLEN + 28);
}

static void arp_send_request(uint32_t target_ip)
{
	static const uint8_t broadcast[ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	eth_build_header(tx_frame, broadcast, ETH_TYPE_ARP);
	uint8_t *arp = tx_frame + ETH_HLEN;
	wr16(arp + 0, 1);
	wr16(arp + 2, ETH_TYPE_IPV4);
	arp[4] = 6;
	arp[5] = 4;
	wr16(arp + 6, ARP_OP_REQUEST);
	memcpy(arp + 8,  our_mac, ETH_ALEN);
	wr32(arp + 14, our_ip);
	memset(arp + 18, 0, ETH_ALEN);  // target MAC unknown
	wr32(arp + 24, target_ip);
	enet_tx(tx_frame, ETH_HLEN + 28);
}

static void handle_arp(const uint8_t *frame, int len)
{
	if (len < ETH_HLEN + 28) return;
	const uint8_t *arp = frame + ETH_HLEN;

	// Only handle Ethernet/IPv4 ARP
	if (rd16(arp) != 1 || rd16(arp + 2) != ETH_TYPE_IPV4) return;

	uint16_t op = rd16(arp + 6);
	uint32_t sender_ip = rd32(arp + 14);
	uint32_t target_ip = rd32(arp + 24);

	// Cache sender's MAC/IP (learn from any ARP packet)
	arp_cache_ip = sender_ip;
	memcpy(arp_cache_mac, arp + 8, ETH_ALEN);
	arp_cache_valid = true;

	if (op == ARP_OP_REQUEST && target_ip == our_ip) {
		arp_send_reply(arp + 8, sender_ip);
	}
}

// ---- ICMP echo reply ----

static void handle_icmp(const uint8_t *frame, int len,
                        const uint8_t *ip_hdr, uint16_t ip_payload_len)
{
	const uint8_t *icmp = ip_hdr + 20;
	if (ip_payload_len < 8) return;
	if (icmp[0] != 8) return;  // Only handle echo request (type 8)

	// Build reply: swap src/dst IP, type=0 (echo reply)
	uint16_t total = ETH_HLEN + 20 + ip_payload_len;
	if (total > sizeof(tx_frame)) return;

	eth_build_header(tx_frame, frame + ETH_ALEN, ETH_TYPE_IPV4);

	uint8_t *ip = tx_frame + ETH_HLEN;
	memcpy(ip, ip_hdr, 20 + ip_payload_len);
	// Swap src/dst IP
	memcpy(ip + 12, ip_hdr + 16, 4);  // dst = original src
	wr32(ip + 16, our_ip);             // src = us
	ip[8] = 64;  // TTL
	// Clear IP checksum, recalculate
	ip[10] = 0; ip[11] = 0;
	uint16_t ipcsum = ip_checksum(ip, 20);
	wr16(ip + 10, ipcsum);

	// ICMP: type=0 (echo reply), recalculate checksum
	uint8_t *icmp_out = ip + 20;
	icmp_out[0] = 0;  // type = echo reply
	icmp_out[2] = 0; icmp_out[3] = 0;  // clear checksum
	uint16_t icmp_csum = ip_checksum(icmp_out, ip_payload_len);
	wr16(icmp_out + 2, icmp_csum);

	enet_tx(tx_frame, total);
}

// ---- IPv4 / UDP handling ----

// Saved UDP packet data (valid until next udp_poll)
static udp_packet_t last_pkt;
static bool pkt_ready;

static void handle_ipv4(const uint8_t *frame, int len)
{
	if (len < ETH_HLEN + 20) return;
	const uint8_t *ip = frame + ETH_HLEN;

	// Version must be 4, IHL must be 5 (no options)
	if ((ip[0] >> 4) != 4) return;
	uint8_t ihl = (ip[0] & 0x0F) * 4;
	if (ihl < 20) return;

	uint16_t total_len = rd16(ip + 2);
	if (ETH_HLEN + total_len > len) return;

	// Drop fragments (MF flag or fragment offset != 0)
	uint16_t frag = rd16(ip + 6);
	if ((frag & 0x3FFF) != 0 && (frag & 0x2000) != 0) return;

	// Check destination: our IP or broadcast
	uint32_t dst_ip = rd32(ip + 16);
	if (dst_ip != our_ip && dst_ip != 0xFFFFFFFF &&
	    dst_ip != (our_ip | ~our_netmask))
		return;

	// Validate IP header checksum
	uint16_t csum = ip_checksum(ip, ihl);
	if (csum != 0) return;

	uint8_t proto = ip[9];
	uint16_t ip_payload_len = total_len - ihl;
	const uint8_t *payload = ip + ihl;

	// Cache sender MAC for ARP (learn from IP packets too)
	uint32_t src_ip = rd32(ip + 12);
	arp_cache_ip = src_ip;
	memcpy(arp_cache_mac, frame + ETH_ALEN, ETH_ALEN);
	arp_cache_valid = true;

	if (proto == IP_PROTO_ICMP) {
		handle_icmp(frame, len, ip, ip_payload_len);
		return;
	}

	if (proto != IP_PROTO_UDP) return;
	if (ip_payload_len < 8) return;

	// Parse UDP header
	last_pkt.src_port = rd16(payload);
	last_pkt.dst_port = rd16(payload + 2);
	uint16_t udp_len  = rd16(payload + 4);
	if (udp_len < 8 || udp_len > ip_payload_len) return;

	last_pkt.src_ip = src_ip;
	last_pkt.data   = payload + 8;
	last_pkt.len    = udp_len - 8;
	pkt_ready = true;
}

// ---- Public API ----

void udp_init(uint32_t ip, uint32_t netmask, uint32_t gateway)
{
	our_ip = ip;
	our_netmask = netmask;
	our_gateway = gateway;
	enet_get_mac(our_mac);
	arp_cache_valid = false;
	pkt_ready = false;
}

bool udp_poll(udp_packet_t *pkt)
{
	pkt_ready = false;

	const uint8_t *frame;
	int len = enet_rx(&frame);
	if (len <= 0) return false;

	if (len < ETH_HLEN) {
		enet_rx_release();
		return false;
	}

	uint16_t ethertype = rd16(frame + 12);

	if (ethertype == ETH_TYPE_ARP) {
		handle_arp(frame, len);
	} else if (ethertype == ETH_TYPE_IPV4) {
		handle_ipv4(frame, len);
	}

	enet_rx_release();

	if (pkt_ready && pkt) {
		*pkt = last_pkt;
		return true;
	}
	return false;
}

bool udp_send(uint32_t dst_ip, uint16_t dst_port, uint16_t src_port,
              const void *data, uint16_t len)
{
	if (len > 1472) return false;  // MTU - IP(20) - UDP(8)

	// Resolve destination MAC
	const uint8_t *dst_mac;
	static const uint8_t broadcast_mac[ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	if (dst_ip == 0xFFFFFFFF || dst_ip == (our_ip | ~our_netmask)) {
		dst_mac = broadcast_mac;
	} else if (arp_cache_valid && arp_cache_ip == dst_ip) {
		dst_mac = arp_cache_mac;
	} else {
		// Try to use cached MAC even if IP doesn't match exactly
		// (works for point-to-point on same subnet)
		if (arp_cache_valid && ((dst_ip ^ arp_cache_ip) & our_netmask) == 0) {
			dst_mac = arp_cache_mac;
		} else {
			// Send ARP request — caller should retry
			arp_send_request(dst_ip);
			return false;
		}
	}

	uint16_t total_len = ETH_HLEN + 20 + 8 + len;
	if (total_len > sizeof(tx_frame)) return false;

	// Ethernet header
	eth_build_header(tx_frame, dst_mac, ETH_TYPE_IPV4);

	// IPv4 header (20 bytes, no options)
	uint8_t *ip = tx_frame + ETH_HLEN;
	ip[0]  = 0x45;        // version=4, IHL=5
	ip[1]  = 0;           // DSCP/ECN
	wr16(ip + 2, 20 + 8 + len);  // total length
	static uint16_t ip_id;
	wr16(ip + 4, ip_id++);  // identification
	wr16(ip + 6, 0x4000);   // don't fragment
	ip[8]  = 64;           // TTL
	ip[9]  = IP_PROTO_UDP;
	ip[10] = 0; ip[11] = 0; // checksum (filled below)
	wr32(ip + 12, our_ip);
	wr32(ip + 16, dst_ip);
	uint16_t ipcsum = ip_checksum(ip, 20);
	wr16(ip + 10, ipcsum);

	// UDP header (8 bytes)
	uint8_t *udp = ip + 20;
	wr16(udp + 0, src_port);
	wr16(udp + 2, dst_port);
	wr16(udp + 4, 8 + len);
	wr16(udp + 6, 0);  // checksum = 0 (optional in IPv4)

	// Payload
	memcpy(udp + 8, data, len);

	return enet_tx(tx_frame, total_len);
}

uint32_t udp_get_ip(void)
{
	return our_ip;
}

#endif // NET_ENABLED
