/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *         Neighbor discovery (RFC 4861)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

/*
 * Copyright (C) 1995, 1996, 1997, and 1998 WIDE Project.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
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
 */

#include "uip-nd6.h"
#include "uip-netif.h"
#include "../lib/random.h"

#include <string.h>

#if IPV6LOWPAN || defined(DOXYGEN)

#define UIPDEBUG 0
#if UIPDEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((u8_t *)addr)[0], ((u8_t *)addr)[1], ((u8_t *)addr)[2], ((u8_t *)addr)[3], ((u8_t *)addr)[4], ((u8_t *)addr)[5], ((u8_t *)addr)[6], ((u8_t *)addr)[7], ((u8_t *)addr)[8], ((u8_t *)addr)[9], ((u8_t *)addr)[10], ((u8_t *)addr)[11], ((u8_t *)addr)[12], ((u8_t *)addr)[13], ((u8_t *)addr)[14], ((u8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#if UIP_LOGGING
#include <stdio.h>
void uip_log(char *msg);
#define UIP_LOG(m) uip_log(m)
#else
#define UIP_LOG PRINTF
#endif /* UIP_LOGGING == 1 */

/** Remove a neighbor cache entry */
#define uip_nd6_nbrcache_rm(neighbor) do {     \
    PRINTF("Removing neighbor with ip addr");  \
    PRINT6ADDR(&neighbor->ipaddr);             \
    PRINTF("\n");                              \
    neighbor->used = 0;                        \
  } while(0)


extern uint8_t interface_number;

/**
 * \brief Timer for maintenance of neighbor cache, prefix list, and default
 * router lists
 */
//struct etimer uip_nd6_timer_periodic[NUM_INTERFACES] = INTERFACE_LIST;
/** \brief Neighor cache */
static struct uip_nd6_neighbor uip_nd6_nbrcache_list[NUM_INTERFACES][UIP_CONF_ND6_MAX_NEIGHBORS];
/** \brief Default router list */
static struct uip_nd6_defrouter uip_nd6_defrouter_list[NUM_INTERFACES][UIP_CONF_ND6_MAX_DEFROUTERS];
/** \brief Prefix list */
static struct uip_nd6_prefix uip_nd6_prefix_list[NUM_INTERFACES][UIP_CONF_ND6_MAX_PREFIXES];

/** \name Temp variables*/
/** @{*/
static u8_t i;
static struct uip_nd6_prefix *prefix;
static struct uip_nd6_neighbor *neighbor;
static struct uip_nd6_defrouter *router;
/** @}*/


void
uip_nd6_init(void)
{
  /* INITIALIZE NEIGHBOR DISCOVERY*/
  for(i = 0; i < UIP_CONF_ND6_MAX_NEIGHBORS; i ++) {
    uip_nd6_nbrcache_list[interface_number][i].used = 0;
  }
  for(i = 0; i < UIP_CONF_ND6_MAX_DEFROUTERS; i ++) {
    uip_nd6_defrouter_list[interface_number][i].used = 0;
  }
  for(i = 0; i < UIP_CONF_ND6_MAX_PREFIXES; i ++) {
    uip_nd6_prefix_list[interface_number][i].used = 0;
  }

  /* create link local prefix */
  uip_ip6addr(&(uip_nd6_prefix_list[interface_number][0].ipaddr), 0xfe80, 0, 0, 0, 0, 0, 0, 0);
  uip_nd6_prefix_list[interface_number][0].length = UIP_DEFAULT_PREFIX_LEN;
  uip_nd6_prefix_list[interface_number][0].used = 1;
  uip_nd6_prefix_list[interface_number][0].is_infinite = 1;

  /* we check the ND structures every 100ms */
  //etimer_set(&uip_nd6_timer_periodic[interface_number], 0.1 * CLOCK_SECOND);
  timer_periodic_nd6(interface_number, 0.1 * CLOCK_SECOND);
}

struct uip_nd6_neighbor *
uip_nd6_nbrcache_lookup(uip_ipaddr_t *ipaddr)
{
  neighbor = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_NEIGHBORS; i ++) {
    if(uip_nd6_nbrcache_list[interface_number][i].used == 0) {
      continue;
    }
    if(uip_ipaddr_cmp(&uip_nd6_nbrcache_list[interface_number][i].ipaddr, ipaddr)) {
      neighbor = &uip_nd6_nbrcache_list[interface_number][i];
      break;
    }
  }
  return neighbor;
}

struct uip_nd6_neighbor *
uip_nd6_nbrcache_lookup_from_lladdr(uip_lladdr_t *lladdr)
{
  neighbor = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_NEIGHBORS; i++) {
    if(uip_nd6_nbrcache_list[interface_number][i].used == 0) {
      continue;
    }
    if(memcmp(&uip_nd6_nbrcache_list[interface_number][i].lladdr, lladdr, UIP_LLADDR_LEN) == 0) {
      neighbor = &uip_nd6_nbrcache_list[interface_number][i];
      break;
    }
  }
  return neighbor;
}

struct uip_nd6_neighbor *
uip_nd6_nbrcache_add(uip_ipaddr_t *ipaddr, uip_lladdr_t *lladdr,
		     u8_t isrouter, uip_neighbor_state state)
{
  neighbor = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_NEIGHBORS; ++i) {
    if(uip_nd6_nbrcache_list[interface_number][i].used == 0) {
      break;
    }
  }
  if(i == UIP_CONF_ND6_MAX_NEIGHBORS){
    i = (u8_t)(random_rand()%UIP_CONF_ND6_MAX_NEIGHBORS);
    UIP_LOG("CACHE FULL");
  }

  neighbor = &(uip_nd6_nbrcache_list[interface_number][i]);
  uip_ipaddr_copy(&(neighbor->ipaddr), ipaddr);
  if(lladdr != NULL){
   // memcpy(&(neighbor->lladdr), lladdr, UIP_LLADDR_LEN);
	for(i = 0; i < UIP_LLADDR_LEN; i++)
	  neighbor->lladdr.addr[i] = lladdr->addr[i];
  } else {
    memset(&(neighbor->lladdr), 0, UIP_LLADDR_LEN);
  }
  PRINTF("Adding neighbor with ip addr");
  PRINT6ADDR(ipaddr);
  PRINTF("link addr");
  PRINTLLADDR((&(neighbor->lladdr)));
  PRINTF("state %d\n", state);

  neighbor->isrouter = isrouter;
  neighbor->state = state;
  /* timers are set separately, for now we put them in expired state */
  timer_set(&(neighbor->reachable),0);
  timer_set(&(neighbor->last_send),0);
  neighbor->count_send = 0;
  neighbor->used = 1;
  return neighbor;
}


struct uip_nd6_defrouter *
uip_nd6_defrouter_lookup(struct uip_nd6_neighbor *neighbor)
{
  router = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_DEFROUTERS; i ++) {
    if(uip_nd6_defrouter_list[interface_number][i].used == 0) {
      continue;
    }
    if(uip_nd6_defrouter_list[interface_number][i].nb == neighbor) {
      router = &uip_nd6_defrouter_list[interface_number][i];
      break;
    }
  }
  return router;
}

struct uip_nd6_defrouter *
uip_nd6_choose_defrouter(void)
{
  router = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_DEFROUTERS; i ++) {
    if(uip_nd6_defrouter_list[interface_number][i].used == 0) {
      continue;
    }
    if(uip_nd6_defrouter_list[interface_number][i].nb == NULL){
      continue;
    }
    if(uip_nd6_defrouter_list[interface_number][i].nb->state == INCOMPLETE ||
       uip_nd6_defrouter_list[interface_number][i].nb->state == NO_STATE){
      if(router == NULL)
        router = &(uip_nd6_defrouter_list[interface_number][i]);
    } else {
      router = &(uip_nd6_defrouter_list[interface_number][i]);
      break;
    }
  }
  return router;
}

void
uip_nd6_defrouter_rm(struct uip_nd6_defrouter *router)
{
  PRINTF("Removing defrouter with ip addr");
  PRINT6ADDR(&((router->nb)->ipaddr));
  PRINTF("\n");
  router->used = 0;
  if(((router->nb)->state) == NO_STATE){
    /* Also remove the corresponding neighbor cache entry */
    uip_nd6_nbrcache_rm(router->nb);
  }
}

struct uip_nd6_defrouter *
uip_nd6_defrouter_add(struct uip_nd6_neighbor *neighbor, clock_time_t interval)
{
  router = NULL;

  for(i = 0; i < UIP_CONF_ND6_MAX_DEFROUTERS; ++i) {
    if(uip_nd6_defrouter_list[interface_number][i].used == 0) {
      break;
    }
  }
  if(i == UIP_CONF_ND6_MAX_DEFROUTERS){
    UIP_LOG("Defrouter list full");
    i = (u8_t)(random_rand()%UIP_CONF_ND6_MAX_DEFROUTERS);
  }

  router = &(uip_nd6_defrouter_list[interface_number][i]);
  router->nb = neighbor;

  PRINTF("Adding defrouter with ip addr");
  PRINT6ADDR(&neighbor->ipaddr);
  PRINTF("\n");

  timer_set(&(router->lifetime),interval);
  uip_nd6_defrouter_list[interface_number][i].used = 1;

  return router;
}

u8_t
uip_nd6_is_addr_onlink(uip_ipaddr_t *ipaddr)
{

  for(i = 0; i < UIP_CONF_ND6_MAX_PREFIXES; i ++) {
    if(uip_nd6_prefix_list[interface_number][i].used == 0){
      continue;
    }
    if(uip_ipaddr_prefixcmp(&uip_nd6_prefix_list[interface_number][i].ipaddr,
                            ipaddr, uip_nd6_prefix_list[interface_number][i].length)){
      return 1;
    }
  }
  return 0;
}

struct uip_nd6_prefix *
uip_nd6_prefix_lookup(uip_ipaddr_t *ipaddr)
{
  prefix = NULL;

  for (i = 0; i < UIP_CONF_ND6_MAX_PREFIXES; i ++) {
    if (uip_nd6_prefix_list[interface_number][i].used == 0){
      continue;
    }
    if (uip_ipaddr_cmp(&uip_nd6_prefix_list[interface_number][i].ipaddr, ipaddr)) {
      prefix = &uip_nd6_prefix_list[interface_number][i];
      break;
    }
  }
  return prefix;
}

struct uip_nd6_prefix *
uip_nd6_prefix_add(uip_ipaddr_t *ipaddr, u8_t length, clock_time_t interval, u8_t flags){

  prefix = NULL;

  /*
   * we start at 1: we do not want to overwrite the link local
   * prefix
   */
  for(i = 1; i < UIP_CONF_ND6_MAX_PREFIXES; ++i) {
    if(uip_nd6_prefix_list[interface_number][i].used == 0) {
      break;
    }
  }
  if(i ==  UIP_CONF_ND6_MAX_PREFIXES){
    UIP_LOG("Prefix list full");
    /*
     * we do not want it to pick the  first entry, which
     * is the link local prefix
     */
    i = (u8_t)(random_rand()%(UIP_CONF_ND6_MAX_PREFIXES -1) + 1);
    PRINTF("Prefix list full, picking index %x\n\n", i);
  }
  prefix = &(uip_nd6_prefix_list[interface_number][i]);
  uip_ipaddr_copy(&(prefix->ipaddr), ipaddr);
  prefix->length = length;

  PRINTF("Adding prefix ");
  PRINT6ADDR(&prefix->ipaddr);
  PRINTF("length %d, vlifetime * CLOCK_SECOND %ld\n", length, interval);

  if(interval != 0){
    timer_set(&(prefix->vlifetime),interval);
    prefix->is_infinite = 0;
  } else {
    prefix->is_infinite = 1;
  }

  uip_nd6_prefix_list[interface_number][i].used = 1;

  if (flags & PREFIX_FLAG_ADV) {
	  uip_nd6_prefix_list[interface_number][i].is_advertised = 1;
  } else {
	  uip_nd6_prefix_list[interface_number][i].is_advertised = 0;
  }

  return prefix;
}


void uip_nd6_prefix_rm(struct uip_nd6_prefix *prefix) {
    PRINTF("Removing prefix ");
    PRINT6ADDR(&prefix->ipaddr);
    PRINTF("length %d\n", prefix->length);
    prefix->used = 0;
}

struct uip_nd6_prefix *
  uip_nd6_prefix_get_list(void){

	return uip_nd6_prefix_list[interface_number];
}

void
uip_nd6_periodic(void)
{

  //etimer_reset(&uip_nd6_timer_periodic[interface_number]);

  /*PERIODIC PROCESSING FOR DEFAULT ROUTER LIST*/
  router = NULL;
  for(i = 0; i < UIP_CONF_ND6_MAX_DEFROUTERS; i ++) {
    if(uip_nd6_defrouter_list[interface_number][i].used == 1) {
      router = &(uip_nd6_defrouter_list[interface_number][i]);

      /* XXX when run on a platform with a 16-bit clock_time_t, the
      timer_expired() below causes the default route to be immediately
      be removed causing communucation problems. We comment it out as
      a quick-fix to this problem on the Atmel RAven platform, but
      since this is needed for IPv6 compliance, we will solve the
      problem by making clock_time_t 32 bits instead after the uIPv6
      snapshot release.

      if(timer_expired(&(router->lifetime))) {
	uip_nd6_defrouter_rm(router);
      }

      */
    }
  }
  /*PERIODIC PROCESSING FOR NEIGHBOR CACHE*/
  neighbor = NULL;
  for(i = 0; i < UIP_CONF_ND6_MAX_NEIGHBORS; i ++) {
    if(uip_nd6_nbrcache_list[interface_number][i].used == 1) {
      neighbor = &(uip_nd6_nbrcache_list[interface_number][i]);
      switch (neighbor->state) {
        case INCOMPLETE:
          if(neighbor->count_send >= UIP_ND6_MAX_MULTICAST_SOLICIT) {
            uip_nd6_nbrcache_list[interface_number][i].used = 0;
          }
          else if(timer_expired(&(neighbor->last_send))) {
            PRINTF("INCOMPLETE: NS %d\n",neighbor->count_send+1);
            uip_nd6_io_ns_output(NULL, NULL, &neighbor->ipaddr);
            timer_set(&(neighbor->last_send),
                      uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
            neighbor->count_send++;
          }
          break;
        case REACHABLE:
          if(timer_expired(&(neighbor->reachable))) {
            PRINTF("REACHABLE: moving to STALE (");
            PRINT6ADDR(&neighbor->ipaddr);
            PRINTF(")\n");

            neighbor->state = STALE;
            neighbor->count_send = 0;
          }
          break;
        case DELAY:
          if(timer_expired(&(neighbor->reachable))) {
            neighbor->state = PROBE;
            neighbor->count_send = 0;
            PRINTF("DELAY: moving to PROBE + NS %d\n", neighbor->count_send+1);
            uip_nd6_io_ns_output(NULL, &neighbor->ipaddr, &neighbor->ipaddr);
            timer_set(&(neighbor->last_send),
                      uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
            neighbor->count_send++;
          }
          break;
        case PROBE:
          if(neighbor->count_send >= UIP_ND6_MAX_UNICAST_SOLICIT) {
            PRINTF("PROBE END \n");
            neighbor->count_send = 0;
            /* remove any corresponding default router (if any)*/
            router = uip_nd6_defrouter_lookup(neighbor);
            if(router != NULL)
              uip_nd6_defrouter_rm(router);
            /* remove neighbor cache entry */
            uip_nd6_nbrcache_rm(neighbor);
            continue;
          }
          if(timer_expired(&(neighbor->last_send))){
            PRINTF("PROBE: NS %d\n",neighbor->count_send+1);
            uip_nd6_io_ns_output(NULL, &neighbor->ipaddr, &neighbor->ipaddr);
            timer_set(&(neighbor->last_send),
                      uip_netif_physical_if[interface_number].retrans_timer/1000*CLOCK_SECOND);
            neighbor->count_send++;
          }
          break;
        default:
          break;
      }
    }
  }
  /*PERIODIC PROCESSING FOR PREFIX LIST*/
  prefix = NULL;
  /*
   * we start with i=1 because the first prefix is the link-local prefix
   * and therefore has INFINITE an lifetime
   */
  for(i = 1; i < UIP_CONF_ND6_MAX_PREFIXES; i ++) {
    if(uip_nd6_prefix_list[interface_number][i].used == 1) {
      prefix = &(uip_nd6_prefix_list[interface_number][i]);
      if((prefix->is_infinite == 0) &&
         (timer_expired(&(prefix->vlifetime)))) {
        /* remove prefix */
        uip_nd6_prefix_rm(prefix);
      }
    }
  }
}
/** @} */

#endif
