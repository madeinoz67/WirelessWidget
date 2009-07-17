/* Copyright (c) 2008-2009  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  $Id: tuip.c,v 1.2 2009/05/29 12:36:09 coflynn Exp $
*/
/**
 * \addtogroup tuip
 * @{
 */


#include <ctype.h>
#include "tuip.h"
#include "interfaces.h"
#include "net/uip.h"
#include "net/uip-nd6.h"
#include "net/uip-netif.h"
#include "net/sicslowpan.h"
// RUM includes
#include "system.h"
#include "mac_associate.h" /* Needed for MAXNODES */
#include "sixlowpan_wake.h"
#include "sensors.h"


#if IPV6LOWPAN || defined(DOXYGEN)

#define PRINTF(x)
#define UIP_LOG(x)
#define ERROR(x)

int tcpip_output(uip_lladdr_t * lladdr);

void uip_send_rip(void);

tuip_sixlowpanbuf_t tuip_sixlowpanbuf;
sixlowpanaddr_t sixlowpanaddr_null;


uip_ipaddr_t lowpan_prefix;

/** @brief If a node has been talked to with IPv6 */
uint8_t talked_to[MAXNODES];

#if UIP_CONF_IPV6_QUEUE_PKT
/** @brief Buffer used when queuing a packet */
u8_t uip_queue_buf[UIP_BUFSIZE + 2];
/** @brief Length of data stored in queue buffer */
u16_t uip_queue_len;
#endif

/************** USER API FOR IPv6 ********************************/

void tuip_routing_init(uip_ipaddr_t * prefix);

/**
 * @brief Processes an incoming IPv6 frame from the ethernet, and
 *        passes it to the uIPv6 stack
 */
void tuip_init(void)
{
    static uint8_t sixlowpanbuf_data[127];

    tuip_sixlowpanbuf.data = sixlowpanbuf_data;

    sicslowpan_init();

    //NutTimerStart(500, callback_periodic, NULL, 0);
}

/**
 * @brief Initialise Ethernet interface for uIPv6
 * @param mac The 6-byte Ethernet MAC address for this interface
 */
void tuip_init_ethernet(uint8_t mac[])
{
    uint8_t i;

    //Set uIP to ethernet interfaces
    tuip_interface_set(INTERFACE_ETHERNET);

    //Ethernet MAC address
    for (i = 0; i < 6; i++)
        uip_lladdr[interface_number].addr[i] = mac[i];

    //Init this interface as router
    uip_init(1800);

    PRINTF("Ethernet interface: IPv6 Support Enabled\n");

    uip_ipaddr_t ipaddr;

    /* The prefix - if you have no router, put your prefix here in HEX */
    /* If router is found this  prefix is ignored anyway */
    uip_ip6addr(&ipaddr, 0xBBBB, 0, 0, 0, 0, 0, 0, 0);

    tuip_routing_init(&ipaddr);
}

/**
 * @brief Initialise 802.15.4 (6lowpan) interface for uIPv6
 * @param panid The PANID this device is on
 * @param shortaddress The short address of this device
 */
void tuip_init_802154(uint16_t panid, uint16_t shortaddress)
{
    //Set uIP to 802.15.4 interfaces
    tuip_interface_set(INTERFACE_802154);

    //Store address
    uip_lladdr[interface_number].addr[0] = MSB(shortaddress);
    uip_lladdr[interface_number].addr[1] = LSB(shortaddress);

    //Store PANID
    tuip_panid[0] = MSB(panid);
    tuip_panid[1] = LSB(panid);

    //Init this interface as router...
    uip_init(1000);

    PRINTF("802.15.4 interface: IPv6 Support Enabled\n");

    /* Init 6LoWPAN context with this prefix */
    sicslowpan_add_context(&lowpan_prefix);

    /* Init IPv6 stuff with this prefix */
    tuip_routing_init(&lowpan_prefix);
    tuip_interface_set(INTERFACE_ETHERNET);

    /* Send RIPng message with this prefix */
    //uip_send_rip(NutTimerStart(30000, uip_send_rip, NULL, 0), NULL);

}


uip_ipaddr_t prefixes[NUM_INTERFACES];

/**
 * @brief Initialise routing for the selected interface
 * @param prefix The prefix that this link should advertise, if
 *               no other router is found. IGNORED if another
 *               router is found, instead that prefix is used.
 */
void tuip_routing_init(uip_ipaddr_t * prefix)
{
    prefixes[interface_number] = *prefix;

    /* We set this flag as 'used' */
    prefixes[interface_number].u8[15] = 0xff;
}

#define UDP_PORT_SENSOR   0xF0B3
#define RUM_MAXPAYLOAD    113

static uint16_t uip_udpLen;
static uint8_t  uip_udpBuf[RUM_MAXPAYLOAD]; //This won't be bigger than maximum RUM length...
static uint8_t  reentCheck;
/**
 * \brief Called to send a UDP packet to a specific end-node.
 * \param addr Short address of node to send to
 * \param len Length of data
 * \param data Pointer to data
 *
 * This function is used to respond to send data from the coordinator
 * to end nodes / routers.
 */
void sixlowpan_sensorSend(u16 addr, u8 len, u8 * data)
{
    if (len > RUM_MAXPAYLOAD)
       return;

    //Check we have a 802.15.4 prefix
    if (uip_netif_physical_if[INTERFACE_802154].addresses[1].state != PREFERRED)
    {
        return;
    }

    //Check buffer is free
    if (uip_udpLen || reentCheck)
        return;

    //Copy data to temporary buffer
    uip_udpLen = len;
    memcpy(uip_udpBuf, data, len);

    /* Send the data */
    tuip_interface_set(INTERFACE_802154);

    //1: Setup a new UDP connection
    struct uip_udp_conn local_conn;

    local_conn.lport = HTONS(UDP_PORT_SENSOR);
    local_conn.rport = HTONS(UDP_PORT_SENSOR);
    local_conn.ttl = 64;

    //Address is same as 802.15.4 side, but with last 16 bits different
    local_conn.ripaddr = uip_netif_physical_if[INTERFACE_802154].addresses[1].ipaddr;
    local_conn.ripaddr.u16[7] = HTONS(addr);

    //2: Set global connection to local one
    uip_udp_conn = &local_conn;

    //3: Call uIP stack, forcing a UDP send event, ONLY if we are no longer in UIP stack..
    //if (reentCheck == 0)
    //{
    uip_process(UIP_UDP_TIMER);
    //}

    // Directly call output function.
    tcpip_ipv6_output();

    return;
}


/************** CALLBACK FUNCTIONS FOR uIP ***********************/

extern void *uip_sappdata;
extern u16_t uip_slen;

/**
 * \brief Called from uIP stack to get user data
 *
 * This function allows the user to insert the UDP data into the outgoing
 * packet.
 */
void tuip_udp_appcall(void)
{
    if (uip_flags == UIP_POLL)
    {
        //Check if we have data to send
        if (uip_udpLen)
        {
            memcpy(uip_sappdata, uip_udpBuf, uip_udpLen);
            uip_slen = uip_udpLen;
            uip_udpLen = 0;
        }
    }
    else if (uip_flags == UIP_NEWDATA)
    {
        //Avoid possible reentrancy
        reentCheck = 1;

        //Reset slen to zero
        uip_slen = 0;

        //Call user function
        if ((APP == SENSOR) && (uip_udp_conn->lport == HTONS(UDP_PORT_SENSOR)))
        {
            sensorRcvPacket((u8 *)uip_sappdata);
        }

        reentCheck = 0;
    }
}

/**
 * @brief Periodic callback function required by uIPv6
 */
void callback_periodic(void)
	{
#if UIP_TCP
#error UNSUPPORTED

          for(i = 0; i < UIP_CONNS; ++i) {
            if(uip_conn_active(i)) {
              /* Only restart the timer if there are active
                 connections. */
              etimer_restart(&periodic);
              uip_periodic(i);
#if UIP_CONF_IPV6
              tcpip_ipv6_output();
#else
              if(uip_len > 0) {
		PRINTF("tcpip_output from periodic len %d\n", uip_len);
                tcpip_output();
		PRINTF("tcpip_output after periodic len %d\n", uip_len);
              }
#endif /* UIP_CONF_IPV6 */
            }
          }
#endif /* UIP_TCP */
#if UIP_CONF_IP_FORWARD
#error UNSUPPORTED
          uip_fw_periodic();
#endif /* UIP_CONF_IP_FORWARD */

	return;
	}

/**
 * @brief Function called when router solicitation is done
 */
void tuip_rs_done_callback(void)
{
	if (uip_nd6_choose_defrouter() == NULL) {
		/* Add our prefix */
		if (prefixes[interface_number].u8[15]) {
			prefixes[interface_number].u8[15] = 0x00;
			uip_nd6_prefix_add(&prefixes[interface_number], 64, 0, PREFIX_FLAG_ADV);
			uip_netif_addr_add(&prefixes[interface_number], 64, 0, AUTOCONF);

		}
	}

	if (uip_netif_physical_if[interface_number].routing_enabled) {
		  if (uip_nd6_choose_defrouter() != NULL) {

			  //We should not be default router!!!
			  uip_netif_physical_if[interface_number].router_lifetime = 0;
		  }

	  	  uip_netif_sched_send_ra();
	  }

	/* Print addresses */
	if (interface_number == INTERFACE_ETHERNET) {
		fnDebugMsg("Ethernet");
	} else if (interface_number == INTERFACE_802154) {
		fnDebugMsg("6LowPAN");
	} else {
		fnDebugMsg("Other");
	}
	fnDebugMsg(" addresses:\r\n");

	uip_netif_printaddrs();


}

void tuip_uipcall(void)
{
	return;
}

typedef struct {
	uip_ipaddr_t prefix;
	uint16_t routetag;
	uint8_t prefixlen;
	uint8_t metric;

} uip_rte_t;

typedef struct {
	uint8_t command;
	uint8_t version;
	uint16_t zeros;
	uip_rte_t rte[2];
} uip_rip_t;

#define UIP_IP_BUF                ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
/**< Pointer to IP header */
#define UIP_UDP_BUF          ((struct uip_udp_hdr *)&uip_buf[UIP_LLIPH_LEN])
#define UIP_RIPNG_BUF        ((uip_rip_t *)&uip_buf[UIP_LLIPH_LEN + UIP_UDPH_LEN])


#define UIP_RIPNG_HOP_LIMIT 255
#define UIP_RIPNG_PORT      521
#define UIP_RIPNG_LEN       sizeof(uip_rip_t)

#define UIP_RIPNG_RESPONSE  2
#define UIP_RIPNG_VERSION   1


/**
 * @brief Send a RIP frame to advertise our route
 */
void uip_send_rip(void)
{

	tuip_interface_set(INTERFACE_ETHERNET);

	/* Right now just sends a RIP message with information about the 802.15.4 route */
	  /* Send to all routers from link-local */
	  uip_create_linklocal_allrouters_mcast((&UIP_IP_BUF->destipaddr));
	  UIP_IP_BUF->destipaddr.u8[15] = 0x09; /* Change address */

	  uip_netif_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);

	  /* IP header fields */
	  uip_ext_len = 0;
	  UIP_IP_BUF->vtc = 0x60;
	  UIP_IP_BUF->tcflow = 0;
	  UIP_IP_BUF->flow = 0;
	  UIP_IP_BUF->proto = UIP_PROTO_UDP;
	  UIP_IP_BUF->ttl = UIP_RIPNG_HOP_LIMIT;

      /* UDP header */
	  UIP_UDP_BUF->destport = HTONS(UIP_RIPNG_PORT);
	  UIP_UDP_BUF->srcport = HTONS(UIP_RIPNG_PORT);
	  UIP_UDP_BUF->udplen = HTONS(UIP_UDPH_LEN + UIP_RIPNG_LEN);
	  UIP_UDP_BUF->udpchksum = 0; /* Fill in checksum later */

	  UIP_RIPNG_BUF->command = UIP_RIPNG_RESPONSE;
	  UIP_RIPNG_BUF->version = UIP_RIPNG_VERSION;
	  UIP_RIPNG_BUF->zeros = 0;

	  /* Where we send the message to */
	  UIP_RIPNG_BUF->rte[0].metric = 0xFF; //Next-hop indicator
	  UIP_RIPNG_BUF->rte[0].routetag = 0;
	  UIP_RIPNG_BUF->rte[0].prefixlen = 0;

	  if (uip_netif_physical_if[interface_number].addresses[0].type == NOT_USED) {
		  uip_len = 0;
		  return;
	  }

	  UIP_RIPNG_BUF->rte[0].prefix = uip_netif_physical_if[interface_number].addresses[0].ipaddr;

	  tuip_interface_set(INTERFACE_802154);

	  struct uip_nd6_prefix * list = uip_nd6_prefix_get_list();

	  tuip_interface_set(INTERFACE_ETHERNET);

	  if (list[1].used == 0) {
		  uip_len = 0;
		  return;
	  }

	  /* The prefix */
	  UIP_RIPNG_BUF->rte[1].prefix = list[1].ipaddr;
	  UIP_RIPNG_BUF->rte[1].prefixlen = list[1].length;
	  UIP_RIPNG_BUF->rte[1].routetag = 0;
	  UIP_RIPNG_BUF->rte[1].metric = 1; /* Destination totally reachable */



	  uip_len = UIP_IPH_LEN + UIP_UDPH_LEN + UIP_RIPNG_LEN;
	  uint16_t len = UIP_UDPH_LEN + UIP_RIPNG_LEN;

	  UIP_IP_BUF->len[0] = MSB(len);
	  UIP_IP_BUF->len[1] = LSB(len);

	  UIP_UDP_BUF->udpchksum = ~uip_udpchksum();

	  tcpip_ipv6_output();
}

uint8_t sendrip;

/*** Callbacks that would be in tcpip.c ***/
#if UIP_CONF_IPV6_REASSEMBLY
#error "Unsupported now, sorry!"
void callback_reass(HANDLE h, void * data)
	{
	uip_reass_over();
	tcpip_ipv6_output();

	return;
	}
#endif

#define E_NETIF_PERIODIC_ND6_EXP  0x10
#define E_NETIF_DAD_EXP			  0x20
#define E_NETIF_RS_EXP            0x30
#define E_NETIF_RA_EXP            0x40
#define E_NETIF_RA_UNICAST_EXP    0x50
#define E_NETIF_PERIODIC_EXP      0x60

/**
 * @brief Event used to manage timers for uIPv6
 */
void fnTaskTimerTuip(TTASKTABLE * ptrTaskTable)
{
	QUEUE_HANDLE PortIDInternal = ptrTaskTable->TaskID;                  // queue ID for task input
    unsigned char ucInputMessage[SMALL_MESSAGE];                        // reserve space for receiving messages

    char event = 0;

	while( fnRead( PortIDInternal, ucInputMessage, HEADER_LENGTH )) {
	    		// switch depending on message source
	        	switch ( ucInputMessage[ MSG_SOURCE_TASK ] ) {

	        	case TIMER_EVENT:

	        		interface_number = ucInputMessage[ MSG_TIMER_EVENT ] & 0x03;
	        		event = ucInputMessage[ MSG_TIMER_EVENT] & 0xF8;

	        		switch(event) {

	        				case E_NETIF_PERIODIC_ND6_EXP:
	        					/* Restart periodic timer */
	        					timer_periodic_nd6(interface_number, 0);
	        					uip_nd6_periodic();
	        					break;

	        				case E_NETIF_DAD_EXP:
	        					uip_netif_dad();
	        					break;

	        				case E_NETIF_RS_EXP:
	        					uip_netif_send_rs();
	        					break;

	        				case E_NETIF_RA_EXP:
	        					uip_netif_send_multicast_ra();
	        					break;

	        				case E_NETIF_RA_UNICAST_EXP:
	        					uip_netif_send_unicast_ra();
	        					break;

	        				case E_NETIF_PERIODIC_EXP:
	        					/* Restart periodic timer */
	        					timer_netif_periodic(interface_number, 0);
	        					uip_netif_periodic();
	        					break;

	        				default:
	        					return;
	        					break;

	        			}

	        			tcpip_ipv6_output();
			    break;
	        default:
	        	 // flush any unexpected messages (assuming they arrived from another task)
	            fnRead( PortIDInternal, ucInputMessage, ucInputMessage[MSG_CONTENT_LENGTH]);
	        }

        }
	return;
}


#if UIP_CONF_IPV6_REASSEMBLY
static NUTTIMERINFO *reassHandle;
#endif

/**
 * @brief Starts the periodic neighbor discovery timer (required by uIPv6)
 */
void timer_periodic_nd6(uint8_t if_num, uint32_t interval)
{
	static uint32_t savedInterval = 40;

	unsigned char event = E_NETIF_PERIODIC_ND6_EXP;

	event |= (0x03) & if_num;

	if (interval) {
		savedInterval = interval;
	}

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, savedInterval, event);
}

/**
 * @brief Starts the DAD timer (required by uIPv6)
 */
void timer_netif_dad(uint8_t if_num, uint32_t interval)
{
	unsigned char event = E_NETIF_DAD_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, interval, event);
}

/**
 * @brief Starts the periodic netif timer (required by uIPv6)
 */
void timer_netif_periodic(uint8_t if_num, uint32_t interval)
{
	static uint32_t savedInterval = 40;

	unsigned char event = E_NETIF_PERIODIC_EXP;

	event |= (0x03) & if_num;

	if (interval) {
		savedInterval = interval;
	}

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, savedInterval, event);
}

/**
 * @brief Starts the RS timer (required by uIPv6)
 */
void timer_netif_rs(uint8_t if_num, uint32_t interval)
{
	unsigned char event = E_NETIF_RS_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, interval, event);
}

/**
 * @brief Starts the RA timer (required by uIPv6)
 */
void timer_netif_ra(uint8_t if_num, uint32_t interval)
{
	unsigned char event = E_NETIF_RA_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, interval, event);
}

/**
 * @brief Starts the unicast RA timer (required by uIPv6)
 */
void timer_netif_ra_unicast(uint8_t if_num, uint32_t interval)
{
	unsigned char event = E_NETIF_RA_UNICAST_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalMonoTimer(TASK_TUIP_TIMER, interval, event);
}

/**
 * @brief Stops the DAD Timer (required by uIPv6)
 */
void timer_netif_dad_stop(uint8_t if_num)
{
	unsigned char event = E_NETIF_DAD_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalStopTimer(TASK_TUIP_TIMER, event);
}

/**
 * @brief Stops the RS Timer (required by uIPv6)
 */
void timer_netif_rs_stop(uint8_t if_num)
{
	unsigned char event = E_NETIF_RS_EXP;

	event |= (0x03) & if_num;

	uTaskerGlobalStopTimer(TASK_TUIP_TIMER, event);
}

/**
 * @brief Sets a timer (required by uIPv6)
 */
void timer_set(struct timer * t, unsigned long interval)
	{
	t->interval = interval;
	t->expiration = uTaskerSystemTick + interval;
	}

/**
 * @brief Gets if timer is expired (required by uIPv6)
 */
int timer_expired(struct timer * t)
	{

	if (t->expiration < uTaskerSystemTick)
		return 1;

	return 0;
	}

/**
 * @brief Gets amount of time left in timer (required by uIPv6)
 */
unsigned long timer_remaining(struct timer * t)
	{
	return t->expiration - uTaskerSystemTick;
	}

/**
 * @brief exit function (requred by uIPv6)
 */
void exit(int ev)
	{
	ERROR("Exit called by uIPv6!\n");

	while(1);
	}

/**
 * @brief Finds a nodes IPv6 Address Based on 802.15.4 Short Address
 *
 */
uint8_t ipv6_lookup_addr(char * string, uint16_t node_short)
{
    uint8_t i;

	/* Do we have enough space to know about this node? */
	if (node_short >= MAXNODES)
	{
	    *string = 0;
	    return 0;
	}

	/* Does this node speak IPv6? */
	if (talked_to[node_short] == 0)
	{
	    *string = 0;
	    return 0;
	}

	uint16_t address[8];
	uint8_t address_to_use = 0;

	/* Figure out if we have something more than link-local address */
	if (uip_netif_physical_if[INTERFACE_802154].addresses[1].state == PREFERRED)
	{
	    address_to_use = 1;
	}

	/* Address will be same as coord except for last byte... */
    for(i = 0; i < 7; i++)
    {
        address[i] = uip_netif_physical_if[INTERFACE_802154].addresses[address_to_use].ipaddr.u16[i];
    }

    /* Short address forms lower byte */
	address[7] = htons(node_short);

	sprint_ip(string, address);

	return 1;
}


#define UIP_ICMP_BUF ((struct uip_icmp_hdr *)&uip_buf[UIP_LLIPH_LEN + uip_ext_len])
#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])


/**
 * @brief Called by uIPv6 to output a packet.
 *
 * Finds the link-local address of the IP packet
 * from the neighbor table. If it is unknown, will
 * do the required neighbor discovery.
 */
void tcpip_ipv6_output(void)
{
  struct uip_nd6_neighbor *nbc = NULL;
  struct uip_nd6_defrouter *dr = NULL;

  if(uip_len == 0)
    return;

  if(uip_len > UIP_LINK_MTU){
    UIP_LOG("tcpip_ipv6_output: Packet to big");
    uip_len = 0;
    return;
  }
  if(uip_is_addr_unspecified(&UIP_IP_BUF->destipaddr)){
    UIP_LOG("tcpip_ipv6_output: Destination address unspecified");
    uip_len = 0;
    return;
  }
  if(!uip_is_addr_mcast(&UIP_IP_BUF->destipaddr)) {

    /* Check if sending on 802.15.4 we don't mean to send on ethernet... */
    if (interface_number == INTERFACE_802154)
    {
        if(!uip_nd6_is_addr_onlink(&UIP_IP_BUF->destipaddr))
        {
            interface_number = INTERFACE_ETHERNET;
        }
    }

    /*If destination is on link */
    if(uip_nd6_is_addr_onlink(&UIP_IP_BUF->destipaddr)){
      nbc = uip_nd6_nbrcache_lookup(&UIP_IP_BUF->destipaddr);
    } else {

      /* We should check if packet is destined to ethernet... */
      if (interface_number == INTERFACE_802154) {
          interface_number = INTERFACE_ETHERNET;
      }


      /*destination is not on link*/
      dr = uip_nd6_choose_defrouter();
      if(dr != NULL){
        nbc = dr->nb;
      } else {
        /* shall we send a icmp error message destination unreachable ?*/
        UIP_LOG("tcpip_ipv6_output: Destination off-link but no router");
        uip_len = 0;
        return;
      }
    }

    /* If sending on 6LoWPAN Network - we don't need to do NS, we can figure
     * out the L2 address from the L3 address */
    if (interface_number == INTERFACE_802154) {

        uip_lladdr_t lladdr;
        lladdr.addr[0] = (&UIP_IP_BUF->destipaddr)->u8[14];
        lladdr.addr[1] = (&UIP_IP_BUF->destipaddr)->u8[15];

        /*multicast IP detination address */
        tcpip_output(&lladdr);
        uip_len = 0;

        return;
    }

    /* there are two cases where the entry logically does not exist:
     * 1 it really does not exist. 2 it is in the NO_STATE state */
    if (nbc == NULL || nbc->state == NO_STATE) {
      if (nbc == NULL) {
        /* create neighbor cache entry, original packet is replaced by NS*/
        nbc = uip_nd6_nbrcache_add(&UIP_IP_BUF->destipaddr, NULL, 0, INCOMPLETE);
      } else {
        nbc->state = INCOMPLETE;
      }

#if UIP_CONF_IPV6_QUEUE_PKT
        /* Stop memory leaks */
        if (nbc->queue_buf_len) {
        	uip_queue_len = 0;
        	nbc->queue_buf_len = 0;
        }
#endif

#if UIP_CONF_IPV6_QUEUE_PKT
      /* copy outgoing pkt in the queuing buffer for later transmmit */
       if (uip_queue_len == 0) {

    	   /* Assign the global queue buffer */
		  nbc->queue_buf = uip_queue_buf;
		  uip_queue_len = uip_len;

		  memcpy(nbc->queue_buf, UIP_IP_BUF, uip_len);
		  nbc->queue_buf_len = uip_len;

       }
#endif
      /* RFC4861, 7.2.2:
       * "If the source address of the packet prompting the solicitation is the
       * same as one of the addresses assigned to the outgoing interface, that
       * address SHOULD be placed in the IP Source Address of the outgoing
       * solicitation.  Otherwise, any one of the addresses assigned to the
       * interface should be used."*/
      if(uip_netif_is_addr_my_unicast(&UIP_IP_BUF->srcipaddr)){
        uip_nd6_io_ns_output(&UIP_IP_BUF->srcipaddr, NULL, &nbc->ipaddr);
      } else {
        uip_nd6_io_ns_output(NULL, NULL, &nbc->ipaddr);
      }

      timer_set(&(nbc->last_send),
                uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
      nbc->count_send = 1;
    } else {
      if (nbc->state == INCOMPLETE){
        PRINTF("tcpip_ipv6_output: neighbor cache entry incomplete\n");

#if UIP_CONF_IPV6_QUEUE_PKT
        /* Stop memory leaks */
        if (nbc->queue_buf_len) {
        	uip_queue_len = 0;
        	nbc->queue_buf_len = 0;
        }
#endif

#if UIP_CONF_IPV6_QUEUE_PKT

        if (uip_queue_len == 0) {
			/* copy outgoing pkt in the queuing buffer for later transmmit */

			/* Assign the global queue buffer */
			  nbc->queue_buf = uip_queue_buf;
			  uip_queue_len = uip_len;

			  memcpy(nbc->queue_buf, UIP_IP_BUF, uip_len);
			  nbc->queue_buf_len = uip_len;

        }

#endif /*UIP_CONF_IPV6_QUEUE_PKT*/
        return;
      }
      /* if running NUD (nbc->state == STALE, DELAY, or PROBE ) keep
         sending in parallel see rfc 4861 Node behavior in section 7.7.3*/

      if (nbc->state == STALE){
        nbc->state = DELAY;
        timer_set(&(nbc->reachable),
                  UIP_ND6_DELAY_FIRST_PROBE_TIME*CLOCK_SECOND);
        PRINTF("tcpip_ipv6_output: neighbor cache entry stale moving to delay\n");
      }

      timer_set(&(nbc->last_send),
                uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);

      tcpip_output(&(nbc->lladdr));


#if UIP_CONF_IPV6_QUEUE_PKT
      /* Send the queued packets from here, may not be 100% perfect though.
       * This happens in a few cases, for example when instead of receiving a
       * NA after sendiong a NS, you receive a NS with SLLAO: the entry moves
       *to STALE, and you must both send a NA and the queued packet
       */
      if(nbc->queue_buf_len != 0) {
        uip_len = nbc->queue_buf_len;
        memcpy(UIP_IP_BUF, nbc->queue_buf, uip_len);

        uip_queue_len = 0;
        nbc->queue_buf_len = 0;
        tcpip_output(&(nbc->lladdr));
      }
#endif /*UIP_CONF_IPV6_QUEUE_PKT*/

      uip_len = 0;
      return;
    }
  }

  /*multicast IP detination address */
  tcpip_output(NULL);
  uip_len = 0;
  uip_ext_len = 0;

}


/************** IPV6 UTILITY FUNCTIONS ***************************/

/**
 * @brief Prints an IPv6 address in fully expanded form to
 *        the debug port.
 */
void print6addr (uint8_t addr[])
{
    unsigned char i;
    char temp[6];

    fnDebugMsg(" ");
    for(i=0; i < 16; i++) {
        sprintf(temp, "%02x", addr[i]);
        fnDebugMsg(temp);
    }
}

/**
 * @brief Formats an IPv6 address to a string, compressing
 *        zeros if possible.
 */
void sprint_ip(char * s, uint16_t * addr)
{
  unsigned char zerocnt = 0;
  unsigned char numprinted = 0;

  while (numprinted < 8)
  {
    //Address is zero, have we used our ability to
    //replace a bunch with : yet?
    if ((*addr == 0) && (zerocnt == 0))
    {
        //How mant zeros?
        while((*(addr + zerocnt) == 0) && ((zerocnt + numprinted) < 8) )
        {
            zerocnt++;
        }

        //just one, don't waste our zeros...
        if (zerocnt == 1)
        {
            *s++ = '0';
            numprinted++;
            addr++;
            zerocnt = 0;
        } else
        {

            //If printing all these zeros will take us to
            //end, still print the last zero
            if ((zerocnt + numprinted) == 8)
            {
                zerocnt--;
            }

            //Cool - can replace a bunch of zeros
            addr += zerocnt;
            numprinted += zerocnt;
        }
    }
    //Normal address, just print it
    else
    {
        s += sprintf(s, "%x", (unsigned int)(ntohs(*addr)));
        addr++;
        numprinted++;
    }

    //Don't print : on last one
    if (numprinted != 8)
    {
        *s++ = ':';
    }

  }// end while

  *s = 0;
}// sprint_ip


uint8_t hexdigit2num(char c);
uint16_t hex2short(char s[]);

/**
 * @brief Formats a string to an IPv6 Address
 */
unsigned char sscanf_ip(char * s, uint16_t Addr[])
{

  unsigned char numGroups = 8;

  char *        ptrStart  = s;
  char *        ptrEnd    = s;
  unsigned char addrGroup = 0;

  char          tempNumber[5];
  unsigned char i;

  while (numGroups)
  {
      //Find next group / end
      while ((*ptrEnd != ':') && (*ptrEnd != 0))
          ptrEnd++;

      //Length Error!
      if ((ptrEnd - ptrStart) > 4)
          return 0;

      //Check for double-':'
      if (*(ptrEnd - 1) == ':')
      {
           //Check for a string that just ends too
           if (*ptrEnd == 0)
           {
               Addr[addrGroup] = 0;
               addrGroup++;
               numGroups = 0; //Set to zero to break out of loop
               continue;
           }

           //Figure out how many zeros we are replacing
           char *    start = s;
           unsigned char groupCnt = 0;

           while(*start)
           {
               if (*start == ':')
                   groupCnt++;

               start++;
           }

           unsigned char replaceCnt = 8 - groupCnt;

           while(replaceCnt)
           {
                Addr[addrGroup] = 0;
                numGroups--;
                replaceCnt--;
                addrGroup++;
           }

           ptrEnd++;
           ptrStart = ptrEnd;
           continue;

      }


      i = 0;
      while ((*ptrStart != ':') && (i < 4))
      {
          tempNumber[i] = *ptrStart;
          ptrStart++;
          i++;
      }
      tempNumber[i] = 0;

      Addr[addrGroup] = hex2short(tempNumber);

      //Work on next section now
      ptrStart = ptrEnd + 1;
      ptrEnd = ptrStart;
      addrGroup++;

      numGroups--;
  }


  //Check we found all 8 groups of 16 bits OK
  if (addrGroup == 8)
      return 1;
  else
      return 0;
}// sscanf_ip

uint16_t hex2short(char s[])
{
    uint16_t        hex;
    uint16_t        digitHex;
    unsigned char   nums;
    unsigned char   i;

    //Finding number of digits
    for(nums = 0; (isalnum(s[nums]) && (nums < 4)); nums++)
    {
        ;
    }

    hex = 0;
    i = 0;

    for (; nums; nums--)
    {
        digitHex = hexdigit2num(s[i]);

        if (nums == 4)
            digitHex *= 0x1000;

        if (nums == 3)
            digitHex *= 0x100;

        if (nums == 2)
            digitHex *= 0x10;

        if (nums == 1)
            digitHex *= 0x1;

        hex += digitHex;
        i++;
    }

    return hex;
}

uint8_t hexdigit2num(char c)
{
    c = tolower(c);

    switch(c)
    {
        case '0': return 0;
        case '1': return 1;
        case '2': return 2;
        case '3': return 3;
        case '4': return 4;
        case '5': return 5;
        case '6': return 6;
        case '7': return 7;
        case '8': return 8;
        case '9': return 9;
        case 'a': return 10;
        case 'b': return 11;
        case 'c': return 12;
        case 'd': return 13;
        case 'e': return 14;
        case 'f': return 15;
        default:  return 0;
    }

    return 0;
}

/*
 * @}
 */

#endif
