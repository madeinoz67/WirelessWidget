/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *         Network interface and stateless autoconfiguration (RFC 4862)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 *
 */

/*
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */

#include "uip-nd6.h"
#include "uip-netif.h"
#include "../lib/random.h"
#include "../interfaces.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if IPV6LOWPAN || defined(DOXYGEN)

#define UIPDEBUG 0
#if UIPDEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((u8_t *)addr)[0], ((u8_t *)addr)[1], ((u8_t *)addr)[2], ((u8_t *)addr)[3], ((u8_t *)addr)[4], ((u8_t *)addr)[5], ((u8_t *)addr)[6], ((u8_t *)addr)[7], ((u8_t *)addr)[8], ((u8_t *)addr)[9], ((u8_t *)addr)[10], ((u8_t *)addr)[11], ((u8_t *)addr)[12], ((u8_t *)addr)[13], ((u8_t *)addr)[14], ((u8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5])

#define sprint6addr(str, addr) sprintf(str, " %02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X ", addr.u8[0], addr.u8[1], addr.u8[2], addr.u8[3], addr.u8[4], addr.u8[5], addr.u8[6], addr.u8[7], addr.u8[8], addr.u8[9], addr.u8[10], addr.u8[11], addr.u8[12], addr.u8[13], addr.u8[14], addr.u8[15])


#if UIP_LOGGING
void uip_log(char *msg);
#define UIP_LOG(m) uip_log(m)
#else
#define UIP_LOG PRINTF
#endif /* UIP_LOGGING == 1 */

/*---------------------------------------------------------------------------*/
/** \brief The single physical interface */
struct uip_netif uip_netif_physical_if[NUM_INTERFACES];
/** \brief The interface address on which dad is being performed */
static struct uip_netif_addr * dad_ifaddr[NUM_INTERFACES];
/** \brief Number of ns already sent for DAD*/
static u8_t dad_ns[NUM_INTERFACES];
/** \brief RS timer, to schedule RS sending */
/** \brief number of rs already sent */
static u8_t rs_count[NUM_INTERFACES];
/** \brief number of ra already sent */
static u8_t ra_count[NUM_INTERFACES];
/** \brief index for loops */
static u8_t i;



/** \brief remove an interface address, argument type is uip_netif_addr* */
#define uip_netif_addr_rm(addr) do {     \
    PRINTF("Removing address");          \
    PRINT6ADDR(&addr->ipaddr);           \
    PRINTF("\n");                        \
    addr->state = NOT_USED;              \
  } while(0)
/*---------------------------------------------------------------------------*/
void
uip_netif_init(uint32_t router_life)
{
  /* INITIALIZE INTERFACE (default values for now) */
  uip_netif_physical_if[interface_number].link_mtu = UIP_LINK_MTU;
  uip_netif_physical_if[interface_number].cur_hop_limit = UIP_TTL;
  uip_netif_physical_if[interface_number].base_reachable_time = UIP_ND6_REACHABLE_TIME;
  uip_netif_physical_if[interface_number].reachable_time = uip_netif_compute_reachable_time();
  uip_netif_physical_if[interface_number].retrans_timer = UIP_ND6_RETRANS_TIMER;
  uip_netif_physical_if[interface_number].dup_addr_detect_transmit = 1;
  uip_netif_physical_if[interface_number].router_lifetime = router_life;
  uip_netif_physical_if[interface_number].routing_enabled = 0;

  /*
   * STATELESS AUTOCONFIGURATION of the link local address. We set it to
   * infinite (this will become really true once DAD succeeds)
   */
  uip_ip6addr(&(uip_netif_physical_if[interface_number].addresses[0].ipaddr),
              0xfe80,0,0,0,0,0,0,0);
  uip_netif_addr_autoconf_set(&(uip_netif_physical_if[interface_number].addresses[0].ipaddr), &uip_lladdr[interface_number]);
  uip_netif_physical_if[interface_number].addresses[0].state = TENTATIVE;
  uip_netif_physical_if[interface_number].addresses[0].type = MANUAL;
  uip_netif_physical_if[interface_number].addresses[0].is_infinite = 1;

  /* set all other addresses to NOT_USED initialy */
  for(i = 1; i < UIP_CONF_NETIF_MAX_ADDRESSES; i ++)
    uip_netif_physical_if[interface_number].addresses[i].state = NOT_USED;

  uip_ip6addr_u8(&(uip_netif_physical_if[interface_number].solicited_node_mcastaddr),
                 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0xff,
                 uip_lladdr[interface_number].addr[UIP_LLADDR_LEN - 3],
                 uip_lladdr[interface_number].addr[UIP_LLADDR_LEN - 2],
                 uip_lladdr[interface_number].addr[UIP_LLADDR_LEN - 1]);
  /* Start DAD */
  uip_netif_sched_dad(&(uip_netif_physical_if[interface_number].addresses[0]));

  /* Find router (send rs to all-routers multicast group))  */
  uip_netif_sched_send_rs();

  //We only start as a router AFTER we understand if there is another router hanging around
  if (router_life != 0) {
	  uip_netif_physical_if[interface_number].routing_enabled = 1;
  }

  /* Reset the timer */
  //etimer_set(&uip_netif_timer_periodic, CLOCK_SECOND);
  timer_netif_periodic(interface_number, CLOCK_SECOND);
}

/*---------------------------------------------------------------------------*/
void
uip_netif_printaddrs(void) {

	char str[50];

	for(i = 0; i < UIP_CONF_NETIF_MAX_ADDRESSES; i++) {
		if(uip_netif_physical_if[interface_number].addresses[i].state != NOT_USED) {
			fnDebugMsg("Number\t\tState\t\tIP Address\r\n");
			sprintf(str, "   %d \t\t   %d\t\t", i, uip_netif_physical_if[interface_number].addresses[i].state);
			fnDebugMsg(str);
			//sprint6addr(str, uip_netif_physical_if[interface_number].addresses[i].ipaddr);
			sprint_ip(str, uip_netif_physical_if[interface_number].addresses[i].ipaddr.u16);
			fnDebugMsg(str);
			fnDebugMsg("\r\n");
		}
	}

}

/*---------------------------------------------------------------------------*/
void
uip_netif_periodic(void) {
  for(i = 1; i < UIP_CONF_NETIF_MAX_ADDRESSES; i++) {
    if((uip_netif_physical_if[interface_number].addresses[i].state != NOT_USED) &&
       (uip_netif_physical_if[interface_number].addresses[i].is_infinite != 1) &&
       (timer_expired(&uip_netif_physical_if[interface_number].addresses[i].vlifetime)) &&
       (1) ) {
      uip_netif_addr_rm((&uip_netif_physical_if[interface_number].addresses[i]));
    }
  }
  //etimer_reset(&uip_netif_timer_periodic);
  return;
}

/*---------------------------------------------------------------------------*/
u32_t
uip_netif_compute_reachable_time(void)
{
  return (u32_t)(uip_netif_physical_if[interface_number].base_reachable_time * UIP_ND6_MIN_RANDOM_FACTOR) + ((u16_t)(random_rand() << 8) + (u16_t)random_rand()) % (u32_t)(uip_netif_physical_if[interface_number].base_reachable_time *(UIP_ND6_MAX_RANDOM_FACTOR - UIP_ND6_MIN_RANDOM_FACTOR));
}


/*---------------------------------------------------------------------------*/
u8_t
uip_netif_is_addr_my_solicited(uip_ipaddr_t *ipaddr)
{
  if(uip_ipaddr_cmp(ipaddr, &uip_netif_physical_if[interface_number].solicited_node_mcastaddr))
    return 1;
  return 0;
}

/*---------------------------------------------------------------------------*/
struct uip_netif_addr *
uip_netif_addr_lookup(uip_ipaddr_t *ipaddr, u8_t length, uip_netif_type type) {
  for(i = 0; i < UIP_CONF_NETIF_MAX_ADDRESSES; i ++) {
    if((uip_netif_physical_if[interface_number].addresses[i].state != NOT_USED) &&
       (uip_netif_physical_if[interface_number].addresses[i].type == type || type == 0) &&
       (uip_ipaddr_prefixcmp(&(uip_netif_physical_if[interface_number].addresses[i].ipaddr), ipaddr, length))) {
      return &uip_netif_physical_if[interface_number].addresses[i];
    }
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_netif_addr_add(uip_ipaddr_t *ipaddr, u8_t length, clock_time_t vlifetime, uip_netif_type type) {

  /* check prefix has the right length if we are doing autoconf */
  if((type == AUTOCONF) && (length != UIP_DEFAULT_PREFIX_LEN)) {
    UIP_LOG("Error: UNSUPPORTED PREFIX LENGTH");
    return;
  }

  /* check if addr does not already exist and find a free entry */
  for(i = 0; i < UIP_CONF_NETIF_MAX_ADDRESSES; ++i) {
    if(uip_netif_physical_if[interface_number].addresses[i].state == NOT_USED){
      /*
       * Copying address
       * If we are doing autoconf, ipaddr is a prefix, we copy the 128 bits
       * of it, then overwrite the last 64 bits with the interface ID at
       * next if statement.
       * Otherwise ipaddr is an address, we just copy it
       */
      uip_ipaddr_copy(&uip_netif_physical_if[interface_number].addresses[i].ipaddr, ipaddr);
      if(type == AUTOCONF) {
        /* construct address from prefix and layer2 id */
        uip_netif_addr_autoconf_set(&uip_netif_physical_if[interface_number].addresses[i].ipaddr, &uip_lladdr[interface_number]);
      }
      /* setting state, type */
      uip_netif_physical_if[interface_number].addresses[i].state = TENTATIVE;
      uip_netif_physical_if[interface_number].addresses[i].type = type;
      /* setting lifetime timer if lieftime is not infinite */
      if(vlifetime != 0) {
        timer_set(&(uip_netif_physical_if[interface_number].addresses[i].vlifetime), vlifetime);
        uip_netif_physical_if[interface_number].addresses[i].is_infinite = 0;
      } else {
        uip_netif_physical_if[interface_number].addresses[i].is_infinite = 1;
      }
      PRINTF("Created new address");
      PRINT6ADDR(&uip_netif_physical_if[interface_number].addresses[i].ipaddr);
      PRINTF("for interface %d\n", interface_number);

      /* schedule DAD */
      uip_netif_sched_dad(&uip_netif_physical_if[interface_number].addresses[i]);

      return;
    }
  }

  /* If we did not find space, log */
  UIP_LOG("ADDRESS LIST FULL");
  return;
}


/*---------------------------------------------------------------------------*/
void
uip_netif_addr_autoconf_set(uip_ipaddr_t *ipaddr, uip_lladdr_t *lladdr)
{
  /* We consider only links with IEEE EUI-64 identifier or
     IEEE 48-bit MAC addresses */
  if(UIP_LLADDR_LEN == 8) {
	  memcpy(ipaddr->u8 + 8, lladdr, UIP_LLADDR_LEN);
	  ipaddr->u8[8] ^= 0x02;
  } else if (UIP_LLADDR_LEN == 6) {
  memcpy(ipaddr->u8 + 8, lladdr, 3);
  ipaddr->u8[11] = 0xff;
  ipaddr->u8[12] = 0xfe;
  memcpy(ipaddr->u8 + 13, (u8_t*)lladdr + 3, 3);
  ipaddr->u8[8] ^= 0x02;
  } else if (UIP_LLADDR_LEN == 2) {
  //Per RFC4944
  ipaddr->u8[15] = lladdr->addr[1];
  ipaddr->u8[14] = lladdr->addr[0];
  ipaddr->u8[13] = 0;
  ipaddr->u8[12] = 0xfe;
  ipaddr->u8[11] = 0xff;
  ipaddr->u8[10] = 0;
  ipaddr->u8[9] = tuip_panid[1];
  ipaddr->u8[8] = tuip_panid[0];

  /* However, in the    resultant interface identifier,
   * the "Universal/Local" (U/L) bit SHALL be set to zero
   *  in keeping with the fact that this is not a globally
   *  unique value.
   *
   *  This means in the IP address this bit MUST be SET
   */
  ipaddr->u8[8] |= (0x02);

  } else {
  UIP_LOG("CAN NOT BUILD INTERFACE IDENTIFIER");
  UIP_LOG("THE STACK IS GOING TO SHUT DOWN");
  UIP_LOG("THE HOST WILL BE UNREACHABLE");
  exit(-1);
  }
}


/*---------------------------------------------------------------------------*/
u8_t
get_match_length(uip_ipaddr_t *src, uip_ipaddr_t *dst)
{
  u8_t j, k, x_or;
  u8_t len = 0;
  for(j = 0; j < 16; j ++) {
    if(src->u8[j] == dst->u8[j]) {
      len += 8;
    } else {
      x_or = src->u8[j] ^ dst->u8[j];
      for(k = 0; k < 8; k ++) {
        if((x_or & 0x80) == 0){
          len ++;
          x_or <<= 1;
        }
        else {
          break;
        }
      }
      break;
    }
  }
  return len;
}


/*---------------------------------------------------------------------------*/
void
uip_netif_select_src(uip_ipaddr_t *src, uip_ipaddr_t *dst)
{
  u8_t best = 0; /* number of bit in common with best match*/
  u8_t n = 0;
  u8_t index = 0;

  if(!uip_is_addr_link_local(dst) && !uip_is_addr_mcast(dst)) {
    for(i = 1; i < UIP_CONF_NETIF_MAX_ADDRESSES; ++i) {
      if(uip_netif_physical_if[interface_number].addresses[i].state == PREFERRED){
        n = get_match_length(dst, &(uip_netif_physical_if[interface_number].addresses[i].ipaddr));
        if(n >= best){
          best = n;
          index = i;
        }
      }
    }
  }

  uip_ipaddr_copy(src, &(uip_netif_physical_if[interface_number].addresses[index].ipaddr));
  return;
}


/*---------------------------------------------------------------------------*/
void
uip_netif_sched_dad(struct uip_netif_addr *ifaddr)
{
  if(ifaddr->state != TENTATIVE){
    UIP_LOG("DAD called with non tentative address");
    return;
  }
  /*
   * check dad is not running
   */
  if(dad_ifaddr[interface_number] != NULL){
    UIP_LOG("DAD already running");
    return;
  }
  /*
   * Set the interface address that is going through DAD
   */
  dad_ifaddr[interface_number] = ifaddr;

  PRINTF("Scheduling DAD for ipaddr:");
  PRINT6ADDR(&dad_ifaddr[interface_number]->ipaddr);
  PRINTF("\n");

  //etimer_set(&uip_netif_timer_dad[interface_number], random_rand()%(UIP_ND6_MAX_RTR_SOLICITATION_DELAY*CLOCK_SECOND));
  timer_netif_dad(interface_number, random_rand()%(UIP_ND6_MAX_RTR_SOLICITATION_DELAY*CLOCK_SECOND));
}


/*---------------------------------------------------------------------------*/
void
uip_netif_dad(void)
{
  /*
   * check if dad is running
   */
  if(dad_ifaddr[interface_number] == NULL){
    PRINTF("uip_netif_dad: DAD not running\n");
    return;
  }
  /*
   * send dup_addr_detect_transmit NS for DAD
   */
  if(dad_ns[interface_number] < uip_netif_physical_if[interface_number].dup_addr_detect_transmit) {
    uip_nd6_io_ns_output(NULL, NULL, &dad_ifaddr[interface_number]->ipaddr);
    dad_ns[interface_number]++;
    //etimer_set(&uip_netif_timer_dad[interface_number], uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
    timer_netif_dad(interface_number, uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
    return;
  }
  /*
   * If we arrive here it means DAD succeeded, otherwise the dad process
   * would have been interrupted in nd6_dad_ns/na_input
   */
  PRINTF("DAD succeeded for ipaddr:");
  PRINT6ADDR(&(dad_ifaddr[interface_number]->ipaddr));
  PRINTF("\n");

  timer_netif_dad_stop(interface_number);
  dad_ifaddr[interface_number]->state = PREFERRED;
  dad_ifaddr[interface_number] = NULL;
  dad_ns[interface_number] = 0;
  /*
   * check if we need to run DAD on another address
   * This is necessary because if you receive a RA,
   * you might want to run DAD for several addresses
   * Considering that we have structures to do DAD
   * for one address only, we start DAD for the subsequent
   * addresses here
   */
  PRINTF("END of DAD\n");
  for(i = 0; i < UIP_CONF_NETIF_MAX_ADDRESSES; i ++){
    if(uip_netif_physical_if[interface_number].addresses[i].state != NOT_USED){
      PRINTF("address %d : ",i);
      PRINT6ADDR(&(uip_netif_physical_if[interface_number].addresses[i].ipaddr));
      PRINTF("\n");
    }
    if(uip_netif_physical_if[interface_number].addresses[i].state == TENTATIVE){
      uip_netif_sched_dad(&uip_netif_physical_if[interface_number].addresses[i]);
      return;
    }
  }
}


/*---------------------------------------------------------------------------*/
void
uip_netif_dad_failed(uip_ipaddr_t *ipaddr)
{

  UIP_LOG("DAD FAILED");
  UIP_LOG("THE STACK IS GOING TO SHUT DOWN");
  UIP_LOG("THE HOST WILL BE UNREACHABLE");

  if(uip_ipaddr_cmp(&dad_ifaddr[interface_number]->ipaddr, ipaddr)){
	timer_netif_dad_stop(interface_number);
    dad_ifaddr[interface_number]->state = NOT_USED;
    dad_ifaddr[interface_number] = NULL;
    dad_ns[interface_number] = 0;
  }

  exit(-1);
}


/*---------------------------------------------------------------------------*/
void
uip_netif_sched_send_rs(void)
{
  /* before a host sends an initial solicitation, it SHOULD delay the
     transmission for a random amount of time between 0 and
     UIP_ND6_MAX_RTR_SOLICITATION_DELAY. */
  if(rs_count[interface_number] == 0){
   // etimer_set(&uip_netif_timer_rs[interface_number], random_rand()%(UIP_ND6_MAX_RTR_SOLICITATION_DELAY*CLOCK_SECOND));
	timer_netif_rs(interface_number, random_rand()%(UIP_ND6_MAX_RTR_SOLICITATION_DELAY*CLOCK_SECOND));
    PRINTF("Scheduling RS\n");
  }
}


/*---------------------------------------------------------------------------*/
void
uip_netif_send_rs(void)
{


  if((uip_nd6_choose_defrouter() == NULL) && (rs_count[interface_number] < UIP_ND6_MAX_RTR_SOLICITATIONS)){
    //PRINTF("Sending RS %d\n", rs_count[interface_number]);
    uip_nd6_io_rs_output();
    rs_count[interface_number]++;
    timer_netif_rs(interface_number, UIP_ND6_RTR_SOLICITATION_INTERVAL * CLOCK_SECOND);
  } else {
    PRINTF("Router found ? (boolean): %d\n", (uip_nd6_choose_defrouter() != NULL));
    //etimer_stop(&uip_netif_timer_rs[interface_number]);
    timer_netif_rs_stop(interface_number);
    rs_count[interface_number] = 0;
    //Perform rs_done callback
    tuip_rs_done_callback();
  }
}

/*---------------------------------------------------------------------------*/
uint32_t
uip_netif_get_ra_rand(void)
{
	uint32_t delay;

	/* Select random delay between 0 and MAX_RTR_ADV_INTERVAL - MIN_RTR_ADV_INTERVAL */
	delay = random_rand()%(UIP_ND6_MAX_RTR_ADV_INTERVAL*CLOCK_SECOND - UIP_ND6_MIN_RTR_ADV_INTERVAL*CLOCK_SECOND);

	/* Add to that MIN_RTR_ADV_INTERVAL, giving a delay inbetween MIN and MAX */
	delay += (UIP_ND6_MIN_RTR_ADV_INTERVAL*CLOCK_SECOND);

	return delay;
}

/*---------------------------------------------------------------------------*/
void
uip_netif_sched_send_ra(void)
{
  /* First one delays by smaller amount */
  if(ra_count[interface_number] == 0){
	uint32_t delay;

	delay = uip_netif_get_ra_rand();

	if (delay > (UIP_ND6_MAX_INITIAL_RTR_ADV_INTERVAL*CLOCK_SECOND)){
		delay = UIP_ND6_MAX_INITIAL_RTR_ADV_INTERVAL*CLOCK_SECOND;
	}

	timer_netif_ra(interface_number, delay);
    PRINTF("Scheduling RA\n");
  }
}


/*---------------------------------------------------------------------------*/
void
uip_netif_send_multicast_ra(void)
{

	/* For the first few advertisements (up to
   MAX_INITIAL_RTR_ADVERTISEMENTS) sent from an interface when it
   becomes an advertising interface, if the randomly chosen interval is
   greater than MAX_INITIAL_RTR_ADVERT_INTERVAL, the timer SHOULD be set
   to MAX_INITIAL_RTR_ADVERT_INTERVAL instead.
   */
  uint32_t delay;
  delay = uip_netif_get_ra_rand();

  if(ra_count[interface_number] < UIP_ND6_MAX_INITIAL_RTR_ADVERTISEMENTS){
	if (delay > (UIP_ND6_MAX_INITIAL_RTR_ADV_INTERVAL*CLOCK_SECOND)){
		delay = UIP_ND6_MAX_INITIAL_RTR_ADV_INTERVAL*CLOCK_SECOND;
	}
  }

  //PRINTF("Sending RA number %d\n", ra_count);

  uip_nd6_io_ra_output(NULL, NULL);

  ra_count[interface_number]++;
  timer_netif_ra(interface_number, delay);
}


/*---------------------------------------------------------------------------*/
static uint8_t unicast_ra_waiting = 0;
static uip_ipaddr_t * unicast_ra_src;
static uip_ipaddr_t * unicast_ra_dest;
static uip_ipaddr_t local_src;
static uip_ipaddr_t local_dest;

void uip_netif_sched_unicast_ra(uip_ipaddr_t *src, uip_ipaddr_t *dest)
{
	if (!unicast_ra_waiting) {

		if (src != NULL) {
			local_src = *src;
			unicast_ra_src = &local_src;
		} else {
			unicast_ra_src = NULL;
		}

		if (dest != NULL) {
			local_dest = *dest;
			unicast_ra_dest = &local_dest;
		} else {
			unicast_ra_dest = NULL;
		}

		/* RA timer busy */
		unicast_ra_waiting = 1;

		/* Start timer */
		timer_netif_ra_unicast(interface_number, random_rand()%(int)(UIP_ND6_MAX_RTR_RA_DELAY_TIME*CLOCK_SECOND));
	} else {
		PRINTF("ERROR: Processing RS already when rx'd another RS. Dropping last RS.\n");
	}
}
/*---------------------------------------------------------------------------*/
void uip_netif_send_unicast_ra(void)
{
	unicast_ra_waiting = 0;

	PRINTF("Sending unicast RA\n");

	uip_nd6_io_ra_output(unicast_ra_src, unicast_ra_dest);
}

/** @} */

#endif
