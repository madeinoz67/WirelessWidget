/**
 * \ingroup iparm
 * \defgroup tuip uTasker to uIPv6 Interface
 * @{
 */

/*
 * uTasker to Contiki Reference - used to run uIPv6 on uTasker
 */
#ifndef __TUIP_H__
#define __TUIP_H__

/* Set your 6LoWPAN Prefix Here. This is assigned by someone to you */
/* aka: if you had the prefix 2002:DB8:1e1:1::/64, set this to:
#define set_6lowpan_prefix  uip_ip6addr(&ipaddr, 0x2002, 0x0DB8, 0x1e1, 1, 0, 0, 0, 0)
*/

#define DEFAULT_6LOWPAN_PREFIX   {0x2002, 0x0DB8, 0x1e1, 1, 0, 0, 0, 0}

#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

struct timer {
  unsigned long expiration;
  unsigned long interval;
};

struct etimer {
  unsigned char fake;
};

void print6addr (uint8_t addr[]);
void sprint_ip(char * s, uint16_t * addr);
unsigned char sscanf_ip(char * s, uint16_t * addr);

void timer_periodic_nd6(uint8_t if_num, uint32_t interval);
void timer_netif_dad(uint8_t if_num, uint32_t interval);
void timer_netif_periodic(uint8_t if_num, uint32_t interval);
void timer_netif_rs(uint8_t if_num, uint32_t interval);
void timer_netif_dad_stop(uint8_t if_num);
void timer_netif_rs_stop(uint8_t if_num);
void timer_netif_ra(uint8_t if_num, uint32_t interval);
void timer_netif_ra_unicast(uint8_t if_num, uint32_t interval);

void timer_set(struct timer * t, unsigned long interval);
int timer_expired(struct timer * t);
unsigned long timer_remaining(struct timer * t);

void tuip_init(void);
void tuip_init_ethernet(uint8_t mac[]);
void tuip_init_802154(uint16_t panid, uint16_t shortaddress);
int ipv6_in_sixlowpan(uint16_t src);
int ipv6_in_ethernet(ETHERNET_FRAME *frame);

void tuip_rs_done_callback(void);

void tcpip_ipv6_output(void);

uint8_t ipv6_lookup_addr(char * string, uint16_t node_short);

/** Rimebuf fake stuff **/
typedef struct {
	uint8_t  src[2];
	uint8_t  dest[2];
	uint8_t * data;
	uint16_t datalen;
} tuip_sixlowpanbuf_t;

extern tuip_sixlowpanbuf_t tuip_sixlowpanbuf;

#define MSB(x)   ((uint8_t *)&x)[1]
#define LSB(x)   ((uint8_t *)&x)[0]

//((ftData*)(mac_buffer+1)->payload)

typedef union {
	uint8_t  u8[2];
} sixlowpanaddr_t;

extern sixlowpanaddr_t sixlowpanaddr_null;

#define sixlowpanbuf_dataptr()       tuip_sixlowpanbuf.data
#define sixlowpanbuf_datalen()       tuip_sixlowpanbuf.datalen
#define sixlowpanbuf_set_datalen(x)  tuip_sixlowpanbuf.datalen = x

#define sixlowpanbuf_addr(x)			tuip_sixlowpanbuf.x
#define SIXLOWPANBUF_ADDR_RECEIVER   dest
#define SIXLOWPANBUF_ADDR_SENDER     src

#define sixlowpanbuf_set_addr(x, y)     tuip_sixlowpanbuf.x[0] = MSB(y); \
		                           tuip_sixlowpanbuf.x[1] = LSB(y)
#define sixlowpanaddr_copy(x, y)        (x)->u8[0] = (y)->u8[0]; (x)->u8[1] = (y)->u8[1]

#define  sixlowpanbuf_clear()        tuip_sixlowpanbuf.datalen = 0

void macsixlowpanDataRequest(uint16_t addr, uint8_t len, uint8_t * data);

void tuip_udp_appcall(void);

#define CLOCK_SECOND SEC

#endif /* __tuip_H__ */
