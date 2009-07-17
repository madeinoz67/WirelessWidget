/**
 * \addtogroup sicslowpan
 * @{
 */
/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * $Id: sicslowpan.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
 */
/**
 * \file
 *         6lowpan implementation (RFC4944 and draft-hui-6lowpan-hc-01)
 *
 * \author Adam Dunkels <adam@sics.se>
 * \author Nicolas Tsiftes <nvt@sics.se>
 * \author Niclas Finne <nfi@sics.se>
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

#include <string.h>

#include "contiki.h"
#include "tcpip.h"
#include "uip.h"
#include "uip-netif.h"
#include "sicslowpan.h"
#include "uip-icmp6.h"
// RUM includes
#include "mac.h"
#include "sixlowpan_wake.h"

#if IPV6LOWPAN || defined(DOXYGEN)

#undef UIPDEBUG
#define UIPDEBUG 0
#if UIPDEBUG
u8_t p;
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((u8_t *)addr)[0], ((u8_t *)addr)[1], ((u8_t *)addr)[2], ((u8_t *)addr)[3], ((u8_t *)addr)[4], ((u8_t *)addr)[5], ((u8_t *)addr)[6], ((u8_t *)addr)[7], ((u8_t *)addr)[8], ((u8_t *)addr)[9], ((u8_t *)addr)[10], ((u8_t *)addr)[11], ((u8_t *)addr)[12], ((u8_t *)addr)[13], ((u8_t *)addr)[14], ((u8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5],lladdr->addr[6], lladdr->addr[7])
#define PRINTSIXLOWPANBUF() PRINTF("SIXLOWPAN buffer: "); for(p = 0; p < sixlowpanbuf_datalen(); p++){PRINTF("%.2X", *(sixlowpan_ptr + p));} PRINTF("\n")
#define PRINTUIPBUF() PRINTF("UIP buffer: "); for(p = 0; p < uip_len; p++){PRINTF("%.2X", uip_buf[p]);}PRINTF("\n")
#define PRINTSICSLOWPANBUF() PRINTF("SICSLOWPAN buffer: "); for(p = 0; p < sicslowpan_len; p++){PRINTF("%.2X", sicslowpan_buf[p]);}PRINTF("\n")
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(lladdr)
#define PRINTSIXLOWPANBUF()
#define PRINTUIPBUF()
#define PRINTSICSLOWPANBUF()
#endif /* UIPDEBUG == 1*/

#if UIP_LOGGING
#include <stdio.h>
void uip_log(char *msg);
#define UIP_LOG(m) uip_log(m)
#else
#define UIP_LOG(m)
#endif /* UIP_LOGGING == 1 */

/** \name Pointers in the sixlowpan buffer
 *  @{
 */
#define SIXLOWPAN_FRAG_BUF               ((struct sicslowpan_frag_hdr *)sixlowpan_ptr)
#define SIXLOWPAN_HC1_BUF                ((struct sicslowpan_hc1_hdr *)(sixlowpan_ptr + sixlowpan_hdr_len))
#define SIXLOWPAN_HC1_HC_UDP_BUF  ((struct sicslowpan_hc1_hc_udp_hdr *)(sixlowpan_ptr + sixlowpan_hdr_len))
#define SIXLOWPAN_IP_BUF                         ((struct uip_ip_hdr *)(sixlowpan_ptr + sixlowpan_hdr_len))
#define SIXLOWPAN_IPHC_BUF              ((struct sicslowpan_iphc_hdr *)(sixlowpan_ptr + sixlowpan_hdr_len))
/** @} */

/** \name Pointers in the sicslowpan and uip buffer
 *  @{
 */
#define SICSLOWPAN_IP_BUF   ((struct uip_ip_hdr *)&sicslowpan_buf[UIP_LLH_LEN])
#define SICSLOWPAN_UDP_BUF ((struct uip_udp_hdr *)&sicslowpan_buf[UIP_LLIPH_LEN])

#define UIP_IP_BUF          ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF          ((struct uip_udp_hdr *)&uip_buf[UIP_LLIPH_LEN])
#define UIP_ICMP_BUF            ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ND6_NS_BUF            ((struct uip_nd6_ns *)&uip_buf[uip_l2_l3_icmp_hdr_len])

/** @} */

extern void ipv6_in_sicslowpan(void);

/** \brief Size of the 802.15.4 payload (127byte - 25 for MAC header) */
#define MAC_MAX_PAYLOAD 102

/** \name General variables
 *  @{
 */

/**
 * A pointer to the sixlowpan buffer.
 * We initialize it to the beginning of the sixlowpan buffer, then
 * access different fields by updating the offset sixlowpan_hdr_len.
 */
static u8_t *sixlowpan_ptr;

/**
 * sixlowpan_hdr_len is the total length of (the processed) 6lowpan headers
 * (fragment headers, IPV6 or HC1, HC2, and HC1 and HC2 non compressed
 * fields).
 */
static u8_t sixlowpan_hdr_len;

/**
 * The length of the payload in the Rime buffer.
 * The payload is what comes after the compressed or uncompressed
 * headers (can be the IP payload if the IP header only is compressed
 * or the UDP payload if the UDP header is also compressed)
 */
static u8_t sixlowpan_payload_len;

/**
 * uncomp_hdr_len is the length of the headers before compression (if HC2
 * is used this includes the UDP header in addition to the IP header).
 */
static u8_t uncomp_hdr_len;
/** @} */

#if SICSLOWPAN_CONF_FRAG
/** \name Fragmentation related variables
 *  @{
 */
/**
 * The buffer used for the 6lowpan reassembly.
 * This buffer contains only the IPv6 packet (no MAC header, 6lowpan, etc).
 * It has a fix size as we do not use dynamic memory allocation.
 */
static u8_t sicslowpan_buf[UIP_BUFSIZE];

/** The total length of the IPv6 packet in the sicslowpan_buf. */
static u16_t sicslowpan_len;

/**
 * length of the ip packet already sent / received.
 * It includes IP and transport headers.
 */
static u16_t processed_ip_len;

/** Datagram tag to be put in the fragments I send. */
static u16_t my_tag;

/** When reassembling, the tag in the fragments being merged. */
static u16_t reass_tag;

/** When reassembling, the source address of the fragments being merged */
sixlowpanaddr_t frag_sender;

/** Reassembly %process %timer. */
static struct timer reass_timer;


/** @} */
#else /* SICSLOWPAN_CONF_FRAG */
/** The buffer used for the 6lowpan processing is uip_buf.
    We do not use any additional buffer.*/
#define sicslowpan_buf uip_buf
#define sicslowpan_len uip_len
#endif /* SICSLOWPAN_CONF_FRAG */


#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01
/** \name HC01 specific variables
 *  @{
 */
/** Addresses contexts for IPHC. */
static struct sicslowpan_addr_context
addr_contexts[SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS];

/** pointer to an address context. */
static struct sicslowpan_addr_context *context;

/** pointer to the byte where to write next inline field. */
static u8_t *hc01_ptr;

/** Index for loops. */
static u8_t i;
/** @} */


/*--------------------------------------------------------------------*/
/** \name HC01 related functions
 * @{                                                                 */


/** \brief find the context corresponding to prefix ipaddr */
static struct sicslowpan_addr_context*
addr_context_lookup_by_prefix(uip_ipaddr_t *ipaddr) {
  for(i = 0; i < SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS; i++) {
    if((addr_contexts[i].used == 1) &&
       uip_ipaddr_prefixcmp(&addr_contexts[i].prefix, ipaddr, 64)) {
      return &addr_contexts[i];
    }
  }
  return NULL;
}

/*--------------------------------------------------------------------*/
/** \brief find the context with the given number */
static struct sicslowpan_addr_context*
addr_context_lookup_by_number(u8_t number) {
  for(i = 0; i < SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS; i++) {
    if((addr_contexts[i].used == 1) &&
       addr_contexts[i].number == number) {
      return &addr_contexts[i];
    }
  }
  return NULL;
}
/*--------------------------------------------------------------------*/
/** \brief Lookup if an IP address is totally compressible */
uint8_t is_addr_sixlowpan_compressible(uip_ipaddr_t * ipaddr, uip_lladdr_t * lladdr) {

    if (addr_context_lookup_by_prefix(ipaddr)) {
        if uip_is_addr_mac_addr_based(ipaddr, lladdr) {
            return 1;
        }
    }

    return 0;
}

/*--------------------------------------------------------------------*/
/**
 * \brief Compress IP/UDP header
 *
 * This function is called by the 6lowpan code to create a compressed
 * 6lowpan packet in the sixlowpanbuf buffer from a full IPv6 packet in the
 * uip_buf buffer.
 *
 *
 * HC01 (draft-hui-6lowpan-hc, version 1)\n
 *
 * \note We do not support ISA100_UDP header compression
 *
 * For LOWPAN_UDP compression, we either compress both ports or none.
 * General format with LOWPAN_UDP compression is
 * \verbatim
 *                       1                   2                   3
 * 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | HC01 Dsp      | HC01 encoding                 |  non          |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | compressed IPv6 fields .....                                  |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | LOWPAN_UDP    | non compressed UDP fields ...                 |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | L4 data ...                                                   |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 * \note The context number 00 is reserved for the link local prefix.
 * For unicast addresses, if we cannot compress the prefix, we neither
 * compress the IID.
 * \param destaddr L2 destination address, needed to compress IP
 * dest
 */
/*
 * Address compression logic (multicast only applies to dest):
 * If multicast:
 *   If flags (see RFC4291 section 2.7) are all 0 AND
 *   the 112-bit group id is mapable to a 9-bit group
 *   (for now all nodes and all routers groups are
 *   mapable),
 *     we compress to 16 bits
 * Else unicast:
 *   If we have a context for the prefix,
 *     we elide 64 bits prefix.
 *     If the IID can be inferred from lower layers,
 *       we elide 64 bits IID
 *     else
 *       if first 49 bits of IID are 0,
 *         we compress IID to 16 bits (with first = 0
 *         to differentiate from multicast)
 *
 */
static void
compress_hdr_hc01(sixlowpanaddr_t *sixlowpan_destaddr)
{
  hc01_ptr = sixlowpan_ptr + 3;
  /*
   * As we copy some bit-length fields, in the IPHC encoding bytes,
   * we sometimes use |=
   * If the field is 0, and the current bit value in memory is 1,
   * this does not work. We therefore reset the IPHC encoding here
   */
  memset(SIXLOWPAN_IPHC_BUF->encoding, 0, 2);

  SIXLOWPAN_IPHC_BUF->dispatch = SICSLOWPAN_DISPATCH_IPHC;
  /*
   * Version, traffic class, flow label
   * If flow label is 0, compress it. If traffic class is 0, compress it
   * We have to process both in the same time as the offset of traffic class
   * depends on the presence of version and flow label
   */
  if(((UIP_IP_BUF->tcflow & 0x0F) == 0) &&
     (UIP_IP_BUF->flow == 0)) {
    /* version and flow label can be compressed */
    SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_VF_C;
    if(((UIP_IP_BUF->vtc & 0x0F) == 0) &&
       ((UIP_IP_BUF->tcflow & 0xF0) == 0)) {
      /* compress (elide) all */
      SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_TC_C;
    } else {
      /* compress only version and flow label */
      *hc01_ptr = (UIP_IP_BUF->vtc << 4) |
        (UIP_IP_BUF->tcflow >> 4);
      hc01_ptr += 1;
    }
  } else {
    /* version and flow label cannot be compressed */
    if(((UIP_IP_BUF->vtc & 0x0F) == 0) &&
       ((UIP_IP_BUF->tcflow & 0xF0) == 0)) {
      /* compress only traffic class */
      SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_TC_C;
      *hc01_ptr = (UIP_IP_BUF->vtc & 0xF0) |
        (UIP_IP_BUF->tcflow & 0x0F);
      memcpy(hc01_ptr + 1, &UIP_IP_BUF->flow, 2);
      hc01_ptr += 3;
    } else {
      /* compress nothing */
      memcpy(hc01_ptr, &UIP_IP_BUF->vtc, 4);
      hc01_ptr += 4;
    }
  }

  /* Note that the payload length is always compressed */

  /* Next header. We compress it if UDP */
#if UIP_CONF_UDP
  if(UIP_IP_BUF->proto == UIP_PROTO_UDP) {
    SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_NH_C;
  } else {
#endif /*UIP_CONF_UDP*/
    *hc01_ptr = UIP_IP_BUF->proto;
    hc01_ptr += 1;
#if UIP_CONF_UDP
  }
#endif /*UIP_CONF_UDP*/

  /*
   * Hop limit
   * if 1: compress, encoding is 01
   * if 64: compress, encoding is 10
   * if 255: compress, encoding is 11
   * else do not compress
   */
  switch(UIP_IP_BUF->ttl) {
    case 1:
      SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_TTL_1;
      break;
    case 64:
      SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_TTL_64;
      break;
    case 255:
      SIXLOWPAN_IPHC_BUF->encoding[0] |= SICSLOWPAN_IPHC_TTL_255;
      break;
    default:
      *hc01_ptr = UIP_IP_BUF->ttl;
      hc01_ptr += 1;
      break;
  }

  /* source address - cannot be multicast */
  if((context = addr_context_lookup_by_prefix(&UIP_IP_BUF->srcipaddr))
     != NULL) {
    /* elide the prefix */
    SIXLOWPAN_IPHC_BUF->encoding[1] |= context->number << 4;
    if(uip_is_addr_mac_addr_based(&UIP_IP_BUF->srcipaddr, &uip_lladdr[interface_number])){
      /* elide the IID */
      SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_SAM_0;
    } else {
      if(sicslowpan_is_iid_16_bit_compressable(&UIP_IP_BUF->srcipaddr)){
        /* compress IID to 16 bits */
        SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_SAM_16;
        memcpy(hc01_ptr, &UIP_IP_BUF->srcipaddr.u16[7], 2);
        hc01_ptr += 2;
      } else {
        /* do not compress IID */
        SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_SAM_64;
        memcpy(hc01_ptr, &UIP_IP_BUF->srcipaddr.u16[4], 8);
        hc01_ptr += 8;
      }
    }
  } else {
    /* send the full address */
    SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_SAM_I;
    memcpy(hc01_ptr, &UIP_IP_BUF->srcipaddr.u16[0], 16);
    hc01_ptr += 16;
  }

  /* dest address*/
  if(uip_is_addr_mcast(&UIP_IP_BUF->destipaddr)) {
    /* Address is multicast, try to compress */
    if(sicslowpan_is_mcast_addr_compressable(&UIP_IP_BUF->destipaddr)) {
      SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_16;
      /* 3 first bits = 101 */
      *hc01_ptr = SICSLOWPAN_IPHC_MCAST_RANGE;
      /* bits 3-6 = scope = bits 8-11 in 128 bits address */
      *hc01_ptr |= (UIP_IP_BUF->destipaddr.u8[1] & 0x0F) << 1;
      /*
       * bits 7 - 15 = 9-bit group
       * We just copy the last byte because it works
       * with currently supported groups
       */
      *(hc01_ptr + 1) = UIP_IP_BUF->destipaddr.u8[15];
      hc01_ptr += 2;
    } else {
      /* send the full address */
      SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_I;
      memcpy(hc01_ptr, &UIP_IP_BUF->destipaddr.u16[0], 16);
      hc01_ptr += 16;
    }
  } else {
    /* Address is unicast, try to compress */
    if((context = addr_context_lookup_by_prefix(&UIP_IP_BUF->destipaddr)) != NULL) {
      /* elide the prefix */
      SIXLOWPAN_IPHC_BUF->encoding[1] |= context->number;
      if(uip_is_addr_mac_addr_based(&UIP_IP_BUF->destipaddr, (uip_lladdr_t *)sixlowpan_destaddr)) {
        /* elide the IID */
        SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_0;
      } else {
        if(sicslowpan_is_iid_16_bit_compressable(&UIP_IP_BUF->destipaddr)) {
          /* compress IID to 16 bits */
          SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_16;
          memcpy(hc01_ptr, &UIP_IP_BUF->destipaddr.u16[7], 2);
          hc01_ptr += 2;
        } else {
          /* do not compress IID */
          SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_64;
          memcpy(hc01_ptr, &UIP_IP_BUF->destipaddr.u16[4], 8);
          hc01_ptr += 8;
        }
      }
    } else {
      /* send the full address */
      SIXLOWPAN_IPHC_BUF->encoding[1] |= SICSLOWPAN_IPHC_DAM_I;
      memcpy(hc01_ptr, &UIP_IP_BUF->destipaddr.u16[0], 16);
      hc01_ptr += 16;
    }
  }
  uncomp_hdr_len = UIP_IPH_LEN;

#if UIP_CONF_UDP
  /* UDP header compression */
  if(UIP_IP_BUF->proto == UIP_PROTO_UDP) {
    if(HTONS(UIP_UDP_BUF->srcport)  >= SICSLOWPAN_UDP_PORT_MIN &&
       HTONS(UIP_UDP_BUF->srcport)  <  SICSLOWPAN_UDP_PORT_MAX &&
       HTONS(UIP_UDP_BUF->destport) >= SICSLOWPAN_UDP_PORT_MIN &&
       HTONS(UIP_UDP_BUF->destport) <  SICSLOWPAN_UDP_PORT_MAX) {
      /* we can compress. Copy compressed ports, full chcksum */
      *hc01_ptr = SICSLOWPAN_NHC_UDP_C;
      *(hc01_ptr + 1) =
        (u8_t)((HTONS(UIP_UDP_BUF->srcport) -
                SICSLOWPAN_UDP_PORT_MIN) << 4) +
        (u8_t)((HTONS(UIP_UDP_BUF->destport) -
                SICSLOWPAN_UDP_PORT_MIN));
      memcpy(hc01_ptr + 2, &UIP_UDP_BUF->udpchksum, 2);
      hc01_ptr += 4;
    } else {
      /* we cannot compress. Copy uncompressed ports, full chcksum */
      *hc01_ptr = SICSLOWPAN_NHC_UDP_I;
      memcpy(hc01_ptr + 1, &UIP_UDP_BUF->srcport, 4);
      memcpy(hc01_ptr + 5, &UIP_UDP_BUF->udpchksum, 2);
      hc01_ptr += 7;
    }
    uncomp_hdr_len += UIP_UDPH_LEN;
  }
#endif /*UIP_CONF_UDP*/
  sixlowpan_hdr_len = hc01_ptr - sixlowpan_ptr;
  return;
}

/*--------------------------------------------------------------------*/
/**
 * \brief Uncompress HC01 (i.e., IPHC and LOWPAN_UDP) headers and put
 * them in sicslowpan_buf
 *
 * This function is called by the input function when the dispatch is
 * HC01.
 * We %process the packet in the sixlowpan buffer, uncompress the header
 * fields, and copy the result in the sicslowpan buffer.
 * At the end of the decompression, sixlowpan_hdr_len and uncompressed_hdr_len
 * are set to the appropriate values
 *
 * \param ip_len Equal to 0 if the packet is not a fragment (IP length
 * is then inferred from the L2 length), non 0 if the packet is a 1st
 * fragment.
 */

/* Processing Details
 * - IP header
 *   We process the fields in their order of appearance in the normal
 *   IP header, with two exceptions:
 *   - next header: field when it is compressed: we need to reach the
 *     NHC encoding to know which is the next header
 *   - length: we need to know the length of headers in sixlowpan buffer
 *     (i.e. the final value of sixlowpan_hdr_len)
 * - Addresses processing
 *   We do the same for src and dest, even though a multicast source
 *   address is wrong. IP layer will handle this. The logic is:
 *
 *   Switch(compression)
 *     case no compression: copy full address
 *     case 64bit compressed address:
 *       find the context, copy prefix from context,
 *       copy IID from packet
 *     case all 128 bit of the address are ellided:
 *       find the context, copy prefix from context,
 *       infer IID from L2 address
 *     case 16 bit compressed address:
 *       if 1st bit = 0 (unicast case)
 *         copy prefix from context, then 48 zeros
 *         then 16 last bits from packet
 *       else (multicast case)
 *         first byte = FF, flags = 0, copy scope
 *         from packet, infer 112 bits group ID from
 *         9 bit group id
 *
 * - UDP header, for LOWPAN_UDP compression
 *   The only trick is that we fill the length field at the end of
 *   the function.
 */

static void
uncompress_hdr_hc01(u16_t ip_len) {
  hc01_ptr = sixlowpan_ptr + sixlowpan_hdr_len + 3;

  /* Version and flow label */
  if((SIXLOWPAN_IPHC_BUF->encoding[0] & 0x40) == 0) {
    /* Version and flow label are carried inline */
    if((SIXLOWPAN_IPHC_BUF->encoding[0] & 0x80) == 0) {
      /* Traffic class is carried inline */
      memcpy(&SICSLOWPAN_IP_BUF->vtc, hc01_ptr, 4);
      hc01_ptr += 4;
    } else {
      /* Traffic class is compressed */
      SICSLOWPAN_IP_BUF->vtc = 0x60;
      SICSLOWPAN_IP_BUF->tcflow = *hc01_ptr & 0x0F;
      memcpy(&SICSLOWPAN_IP_BUF->flow, hc01_ptr + 1, 2);
      hc01_ptr += 3;
    }
  } else {
    /* Version and flow label are compressed */
    if((SIXLOWPAN_IPHC_BUF->encoding[0] & 0x80) == 0) {
      /* Traffic class is inline */
      SICSLOWPAN_IP_BUF->vtc = 0x60 | (*hc01_ptr >> 4);
      SICSLOWPAN_IP_BUF->tcflow = *hc01_ptr << 4;
      hc01_ptr += 1;
    } else {
      /* Traffic class is compressed */
      SICSLOWPAN_IP_BUF->vtc = 0x60;
      SICSLOWPAN_IP_BUF->tcflow = 0;
    }
    SICSLOWPAN_IP_BUF->flow = 0;
  }
  /* Next Header */
  if((SIXLOWPAN_IPHC_BUF->encoding[0] & 0x20) == 0) {
    /* Next header is carried inline */
    SICSLOWPAN_IP_BUF->proto = *hc01_ptr;
    hc01_ptr += 1;
  }

  /* Hop limit */
  switch(SIXLOWPAN_IPHC_BUF->encoding[0] & 0x18) {
    case SICSLOWPAN_IPHC_TTL_1:
      SICSLOWPAN_IP_BUF->ttl = 1;
      break;
    case SICSLOWPAN_IPHC_TTL_64:
      SICSLOWPAN_IP_BUF->ttl = 64;
      break;
    case SICSLOWPAN_IPHC_TTL_255:
      SICSLOWPAN_IP_BUF->ttl = 255;
      break;
    case SICSLOWPAN_IPHC_TTL_I:
      SICSLOWPAN_IP_BUF->ttl = *hc01_ptr;
      hc01_ptr += 1;
      break;
  }

  /* Source address */
  context =
    addr_context_lookup_by_number((SIXLOWPAN_IPHC_BUF->encoding[1] & 0x30) >> 4);

  switch(SIXLOWPAN_IPHC_BUF->encoding[1] & 0xC0) {
    case SICSLOWPAN_IPHC_SAM_0:
      if(context == NULL) {
        PRINTF("sicslowpan uncompress_hdr: error context not found\n");
        return;
      }
      /* copy prefix from context */
      memcpy(&SICSLOWPAN_IP_BUF->srcipaddr, context->prefix, 8);
      /* infer IID from L2 address */
      uip_netif_addr_autoconf_set(&SICSLOWPAN_IP_BUF->srcipaddr,
                                  (uip_lladdr_t *)sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_SENDER));
      break;
    case SICSLOWPAN_IPHC_SAM_16:
      if((*hc01_ptr & 0x80) == 0) {
        /* unicast address */
        if(context == NULL) {
          PRINTF("sicslowpan uncompress_hdr: error context not found\n");
          return;
        }
        memcpy(&SICSLOWPAN_IP_BUF->srcipaddr, context->prefix, 8);
        /* copy 6 NULL bytes then 2 last bytes of IID */
        memset(&SICSLOWPAN_IP_BUF->srcipaddr.u8[8], 0, 6);
        memcpy(&SICSLOWPAN_IP_BUF->srcipaddr.u8[14], hc01_ptr, 2);
        hc01_ptr += 2;
      } else {
        /* multicast address check the 9-bit group-id is known */
        if(sicslowpan_is_mcast_addr_decompressable(hc01_ptr)) {
          SICSLOWPAN_IP_BUF->srcipaddr.u8[0] = 0xFF;
          SICSLOWPAN_IP_BUF->srcipaddr.u8[1] = (*hc01_ptr >> 1) & 0x0F;
          memset(&SICSLOWPAN_IP_BUF->srcipaddr.u8[2], 0, 13);
          SICSLOWPAN_IP_BUF->srcipaddr.u8[15] = *(hc01_ptr + 1);
          hc01_ptr += 2;
        } else {
          PRINTF("sicslowpan uncompress_hdr: error unknown compressed mcast address\n");
          return;
        }
      }
      break;
    case SICSLOWPAN_IPHC_SAM_64:
      if(context == NULL) {
        PRINTF("sicslowpan uncompress_hdr: error context not found\n");
        return;
      }
      /* copy prefix from context */
      memcpy(&SICSLOWPAN_IP_BUF->srcipaddr, context->prefix, 8);
      /* copy IID from packet */
      memcpy(&SICSLOWPAN_IP_BUF->srcipaddr.u8[8], hc01_ptr, 8);
      hc01_ptr += 8;
      break;
    case SICSLOWPAN_IPHC_SAM_I:
      /* copy whole address from packet */
      memcpy(&SICSLOWPAN_IP_BUF->srcipaddr.u8[0], hc01_ptr, 16);
      hc01_ptr += 16;
      break;
  }

  /* Destination address */
  context = addr_context_lookup_by_number(SIXLOWPAN_IPHC_BUF->encoding[1] & 0x03);

  switch(SIXLOWPAN_IPHC_BUF->encoding[1] & 0x0C) {
    case SICSLOWPAN_IPHC_DAM_0:
      if(context == NULL) {
        PRINTF("sicslowpan uncompress_hdr: error context not found\n");
        return;
      }
      /* copy prefix from context */
      memcpy(&SICSLOWPAN_IP_BUF->destipaddr, context->prefix, 8);
      /* infer IID from L2 address */
      uip_netif_addr_autoconf_set(&SICSLOWPAN_IP_BUF->destipaddr,
                                  (uip_lladdr_t *)sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_RECEIVER));
      break;
    case SICSLOWPAN_IPHC_DAM_16:
      if((*hc01_ptr & 0x80) == 0) {
        /* unicast address */
        if(context == NULL) {
          PRINTF("sicslowpan uncompress_hdr: error context not found\n");
          return;
        }
        memcpy(&SICSLOWPAN_IP_BUF->destipaddr, context->prefix, 8);
        /* copy 6 NULL bytes then 2 last bytes of IID */
        memset(&SICSLOWPAN_IP_BUF->destipaddr.u8[8], 0, 6);
        memcpy(&SICSLOWPAN_IP_BUF->destipaddr.u8[14], hc01_ptr, 2);
        hc01_ptr += 2;
      } else {
        /* multicast address check the 9-bit group-id is known */
        if(sicslowpan_is_mcast_addr_decompressable(hc01_ptr)) {
          SICSLOWPAN_IP_BUF->destipaddr.u8[0] = 0xFF;
          SICSLOWPAN_IP_BUF->destipaddr.u8[1] = (*hc01_ptr >> 1) & 0x0F;
          memset(&SICSLOWPAN_IP_BUF->destipaddr.u8[2], 0, 13);
          SICSLOWPAN_IP_BUF->destipaddr.u8[15] = *(hc01_ptr + 1);
          hc01_ptr += 2;
        } else {
          PRINTF("sicslowpan uncompress_hdr: error unknown compressed mcast address\n");
          return;
        }
      }
      break;
    case SICSLOWPAN_IPHC_DAM_64:
      if(context == NULL) {
        PRINTF("sicslowpan uncompress_hdr: error context not found\n");
        return;
      }
      memcpy(&SICSLOWPAN_IP_BUF->destipaddr, context->prefix, 8);
      memcpy(&SICSLOWPAN_IP_BUF->destipaddr.u8[8], hc01_ptr, 8);
      hc01_ptr += 8;
      break;
    case SICSLOWPAN_IPHC_DAM_I:
      /* copy whole address from packet */
      memcpy(&SICSLOWPAN_IP_BUF->destipaddr.u8[0], hc01_ptr, 16);
      hc01_ptr += 16;
      break;
  }
  uncomp_hdr_len += UIP_IPH_LEN;

  /* Next header processing - continued */
  if((SIXLOWPAN_IPHC_BUF->encoding[0] & 0x20) != 0) {
    /* The next header is compressed, NHC is following */
    if((*hc01_ptr & 0xFC) == SICSLOWPAN_NHC_UDP_ID) {
      SICSLOWPAN_IP_BUF->proto = UIP_PROTO_UDP;
      switch(*hc01_ptr) {
        case SICSLOWPAN_NHC_UDP_C:
          /* 1 byte for NHC, 1 byte for ports, 2 bytes chksum */
          SICSLOWPAN_UDP_BUF->srcport = HTONS(SICSLOWPAN_UDP_PORT_MIN +
                                              (*(hc01_ptr + 1) >> 4));
          SICSLOWPAN_UDP_BUF->destport = HTONS(SICSLOWPAN_UDP_PORT_MIN +
                                               ((*(hc01_ptr + 1)) & 0x0F));
          memcpy(&SICSLOWPAN_UDP_BUF->udpchksum, hc01_ptr + 2, 2);
          hc01_ptr += 4;
          break;
        case SICSLOWPAN_NHC_UDP_I:
          /* 1 byte for NHC, 4 byte for ports, 2 bytes chksum */
          memcpy(&SICSLOWPAN_UDP_BUF->srcport, hc01_ptr + 1, 2);
          memcpy(&SICSLOWPAN_UDP_BUF->destport, hc01_ptr + 3, 2);
          memcpy(&SICSLOWPAN_UDP_BUF->udpchksum, hc01_ptr + 5, 2);
          hc01_ptr += 7;
          break;
        default:
          PRINTF("sicslowpan uncompress_hdr: error unsupported UDP compression\n");
          return;
      }
      uncomp_hdr_len += UIP_UDPH_LEN;
    }
  }

  sixlowpan_hdr_len = hc01_ptr - sixlowpan_ptr;

  /* IP length field. */
  if(ip_len == 0) {
    /* This is not a fragmented packet */
    SICSLOWPAN_IP_BUF->len[0] = 0;
    SICSLOWPAN_IP_BUF->len[1] = sixlowpanbuf_datalen() - sixlowpan_hdr_len + uncomp_hdr_len - UIP_IPH_LEN;
  } else {
    /* This is a 1st fragment */
    SICSLOWPAN_IP_BUF->len[0] = (ip_len - UIP_IPH_LEN) >> 8;
    SICSLOWPAN_IP_BUF->len[1] = (ip_len - UIP_IPH_LEN) & 0x00FF;
  }

  /* length field in UDP header */
  if(SICSLOWPAN_IP_BUF->proto == UIP_PROTO_UDP) {
    memcpy(&SICSLOWPAN_UDP_BUF->udplen, &SICSLOWPAN_IP_BUF->len[0], 2);
  }
  return;
}
/** @} */
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01*/


#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1
/*--------------------------------------------------------------------*/
/** \name HC1 compression and uncompression functions
 *  @{                                                                */
/*--------------------------------------------------------------------*/
/**
 * \brief Compress IP/UDP header using HC1 and HC_UDP
 *
 * This function is called by the 6lowpan code to create a compressed
 * 6lowpan packet in the sixlowpanbuf buffer from a full IPv6 packet in the
 * uip_buf buffer.
 *
 *
 * If we can compress everything, we use HC1 dispatch, if not we use
 * IPv6 dispatch.\n
 * We can compress everything if:
 *   - IP version is
 *   - Flow label and traffic class are 0
 *   - Both src and dest ip addresses are link local
 *   - Both src and dest interface ID are recoverable from lower layer
 *     header
 *   - Next header is either ICMP, UDP or TCP
 * Moreover, if next header is UDP, we try to compress it using HC_UDP.
 * This is feasible is both ports are between F0B0 and F0B0 + 15\n\n
 *
 * Resulting header structure:
 * - For ICMP, TCP, non compressed UDP\n
 *   HC1 encoding = 11111010 (UDP) 11111110 (TCP) 11111100 (ICMP)\n
 * \verbatim
 *                      1                   2                   3
 * 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | LoWPAN HC1 Dsp | HC1 encoding  | IPv6 Hop limit| L4 hdr + data|
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 *
 * - For compressed UDP
 *   HC1 encoding = 11111011, HC_UDP encoding = 11100000\n
 * \verbatim
 *                      1                   2                   3
 * 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | LoWPAN HC1 Dsp| HC1 encoding  |  HC_UDP encod.| IPv6 Hop limit|
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | src p.| dst p.| UDP checksum                  | L4 data...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 *
 * \param sixlowpan_destaddr L2 destination address, needed to compress the
 * IP destination field
 */
static void
compress_hdr_hc1(sixlowpanaddr_t *sixlowpan_destaddr)
{
  /*
   * Check if all the assumptions for full compression
   * are valid :
   */
  if(UIP_IP_BUF->vtc != 0x60 ||
     UIP_IP_BUF->tcflow != 0 ||
     UIP_IP_BUF->flow != 0 ||
     !uip_is_addr_link_local(&UIP_IP_BUF->srcipaddr) ||
     !uip_is_addr_mac_addr_based(&UIP_IP_BUF->srcipaddr, &uip_lladdr[interface_number]) ||
     !uip_is_addr_link_local(&UIP_IP_BUF->destipaddr) ||
     !uip_is_addr_mac_addr_based(&UIP_IP_BUF->destipaddr,
                                 (uip_lladdr_t *)sixlowpan_destaddr) ||
     (UIP_IP_BUF->proto != UIP_PROTO_ICMP6 &&
      UIP_IP_BUF->proto != UIP_PROTO_UDP &&
      UIP_IP_BUF->proto != UIP_PROTO_TCP))
  {
    /*
     * IPV6 DISPATCH
     * Something cannot be compressed, use IPV6 DISPATCH,
     * compress nothing, copy IPv6 header in sixlowpan buffer
     */
    *sixlowpan_ptr = SICSLOWPAN_DISPATCH_IPV6;
    sixlowpan_hdr_len += SICSLOWPAN_IPV6_HDR_LEN;
    memcpy(sixlowpan_ptr + sixlowpan_hdr_len, UIP_IP_BUF, UIP_IPH_LEN);
    sixlowpan_hdr_len += UIP_IPH_LEN;
    uncomp_hdr_len += UIP_IPH_LEN;
  } else {
    /*
     * HC1 DISPATCH
     * maximum compresssion:
     * All fields in the IP header but Hop Limit are elided
     * If next header is UDP, we compress UDP header using HC2
     */
    SIXLOWPAN_HC1_BUF->dispatch = SICSLOWPAN_DISPATCH_HC1;
    uncomp_hdr_len += UIP_IPH_LEN;
    switch(UIP_IP_BUF->proto) {
      case UIP_PROTO_ICMP6:
        /* HC1 encoding and ttl */
        SIXLOWPAN_HC1_BUF->encoding = 0xFC;
        SIXLOWPAN_HC1_BUF->ttl = UIP_IP_BUF->ttl;
        sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
        break;
#if UIP_CONF_TCP
      case UIP_PROTO_TCP:
        /* HC1 encoding and ttl */
        SIXLOWPAN_HC1_BUF->encoding = 0xFE;
        SIXLOWPAN_HC1_BUF->ttl = UIP_IP_BUF->ttl;
        sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
        break;
#endif /* UIP_CONF_TCP */
#if UIP_CONF_UDP
      case UIP_PROTO_UDP:
        /*
         * try to compress UDP header (we do only full compression).
         * This is feasible if both src and dest ports are between
         * SICSLOWPAN_UDP_PORT_MIN and SICSLOWPAN_UDP_PORT_MIN + 15
         */
        PRINTF("local/remote port %u/%u\n",UIP_UDP_BUF->srcport,UIP_UDP_BUF->destport);
        if(HTONS(UIP_UDP_BUF->srcport)  >= SICSLOWPAN_UDP_PORT_MIN &&
           HTONS(UIP_UDP_BUF->srcport)  <  SICSLOWPAN_UDP_PORT_MAX &&
           HTONS(UIP_UDP_BUF->destport) >= SICSLOWPAN_UDP_PORT_MIN &&
           HTONS(UIP_UDP_BUF->destport) <  SICSLOWPAN_UDP_PORT_MAX) {
          /* HC1 encoding */
          SIXLOWPAN_HC1_HC_UDP_BUF->hc1_encoding = 0xFB;

          /* HC_UDP encoding, ttl, src and dest ports, checksum */
          SIXLOWPAN_HC1_HC_UDP_BUF->hc_udp_encoding = 0xE0;
          SIXLOWPAN_HC1_HC_UDP_BUF->ttl = UIP_IP_BUF->ttl;
          SIXLOWPAN_HC1_HC_UDP_BUF->ports = (u8_t)((HTONS(UIP_UDP_BUF->srcport) -
                                               SICSLOWPAN_UDP_PORT_MIN) << 4) +
            (u8_t)((HTONS(UIP_UDP_BUF->destport) - SICSLOWPAN_UDP_PORT_MIN));
          SIXLOWPAN_HC1_HC_UDP_BUF->udpchksum = UIP_UDP_BUF->udpchksum;
          sixlowpan_hdr_len += SICSLOWPAN_HC1_HC_UDP_HDR_LEN;
          uncomp_hdr_len += UIP_UDPH_LEN;
        } else {
          /* HC1 encoding and ttl */
          SIXLOWPAN_HC1_BUF->encoding = 0xFA;
          SIXLOWPAN_HC1_BUF->ttl = UIP_IP_BUF->ttl;
          sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
        }
        break;
#endif /*UIP_CONF_UDP*/
    }
  }
  return;
}

/*--------------------------------------------------------------------*/
/**
 * \brief Uncompress HC1 (and HC_UDP) headers and put them in
 * sicslowpan_buf
 *
 * This function is called by the input function when the dispatch is
 * HC1.
 * We %process the packet in the sixlowpan buffer, uncompress the header
 * fields, and copy the result in the sicslowpan buffer.
 * At the end of the decompression, sixlowpan_hdr_len and uncompressed_hdr_len
 * are set to the appropriate values
 *
 * \param ip_len Equal to 0 if the packet is not a fragment (IP length
 * is then inferred from the L2 length), non 0 if the packet is a 1st
 * fragment.
 */
static void
uncompress_hdr_hc1(u16_t ip_len) {
  /* version, traffic class, flow label */
  SICSLOWPAN_IP_BUF->vtc = 0x60;
  SICSLOWPAN_IP_BUF->tcflow = 0;
  SICSLOWPAN_IP_BUF->flow = 0;

  /* src and dest ip addresses */
  uip_ip6addr(&SICSLOWPAN_IP_BUF->srcipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);
  uip_netif_addr_autoconf_set(&SICSLOWPAN_IP_BUF->srcipaddr,
                              (uip_lladdr_t *)sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_SENDER));
  uip_ip6addr(&SICSLOWPAN_IP_BUF->destipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);
  uip_netif_addr_autoconf_set(&SICSLOWPAN_IP_BUF->destipaddr,
                              (uip_lladdr_t *)sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_RECEIVER));

  uncomp_hdr_len += UIP_IPH_LEN;

  /* Next header field */
  switch(SIXLOWPAN_HC1_BUF->encoding & 0x06) {
    case SICSLOWPAN_HC1_NH_ICMP6:
      SICSLOWPAN_IP_BUF->proto = UIP_PROTO_ICMP6;
      SICSLOWPAN_IP_BUF->ttl = SIXLOWPAN_HC1_BUF->ttl;
      sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
      break;
#if UIP_CONF_TCP
    case SICSLOWPAN_HC1_NH_TCP:
      SICSLOWPAN_IP_BUF->proto = UIP_PROTO_TCP;
      SICSLOWPAN_IP_BUF->ttl = SIXLOWPAN_HC1_BUF->ttl;
      sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
      break;
#endif/* UIP_CONF_TCP */
#if UIP_CONF_UDP
    case SICSLOWPAN_HC1_NH_UDP:
      SICSLOWPAN_IP_BUF->proto = UIP_PROTO_UDP;
      if(SIXLOWPAN_HC1_HC_UDP_BUF->hc1_encoding & 0x01) {
        /* UDP header is compressed with HC_UDP */
        if(SIXLOWPAN_HC1_HC_UDP_BUF->hc_udp_encoding !=
           SICSLOWPAN_HC_UDP_ALL_C) {
          PRINTF("sicslowpan (uncompress_hdr), packet not supported");
          return;
        }
        /* IP TTL */
        SICSLOWPAN_IP_BUF->ttl = SIXLOWPAN_HC1_HC_UDP_BUF->ttl;
        /* UDP ports, len, checksum */
        SICSLOWPAN_UDP_BUF->srcport = HTONS(SICSLOWPAN_UDP_PORT_MIN +
                                            (SIXLOWPAN_HC1_HC_UDP_BUF->ports >> 4));
        SICSLOWPAN_UDP_BUF->destport = HTONS(SICSLOWPAN_UDP_PORT_MIN +
                                             (SIXLOWPAN_HC1_HC_UDP_BUF->ports & 0x0F));
        SICSLOWPAN_UDP_BUF->udpchksum = SIXLOWPAN_HC1_HC_UDP_BUF->udpchksum;
        uncomp_hdr_len += UIP_UDPH_LEN;
        sixlowpan_hdr_len += SICSLOWPAN_HC1_HC_UDP_HDR_LEN;
      } else {
        sixlowpan_hdr_len += SICSLOWPAN_HC1_HDR_LEN;
      }
      break;
#endif/* UIP_CONF_UDP */
    default:
      /* this shouldn't happen, drop */
      return;
  }

  /* IP length field. */
  if(ip_len == 0) {
    /* This is not a fragmented packet */
    SICSLOWPAN_IP_BUF->len[0] = 0;
    SICSLOWPAN_IP_BUF->len[1] = sixlowpanbuf_datalen() - sixlowpan_hdr_len + uncomp_hdr_len - UIP_IPH_LEN;
  } else {
    /* This is a 1st fragment */
    SICSLOWPAN_IP_BUF->len[0] = (ip_len - UIP_IPH_LEN) >> 8;
    SICSLOWPAN_IP_BUF->len[1] = (ip_len - UIP_IPH_LEN) & 0x00FF;
  }
  /* length field in UDP header */
  if(SICSLOWPAN_IP_BUF->proto == UIP_PROTO_UDP) {
    memcpy(&SICSLOWPAN_UDP_BUF->udplen, &SICSLOWPAN_IP_BUF->len[0], 2);
  }
  return;
}
/** @} */
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1*/


#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_IPV6
/*--------------------------------------------------------------------*/
/** \name IPv6 dispatch "compression" function
 * @{                                                                 */
/*--------------------------------------------------------------------*/
/* \brief Packets "Compression" when only IPv6 dispatch is used
 *
 * There is no compression in this case, all fields are sent
 * inline. We just add the IPv6 dispatch byte before the packet.
 * \verbatim
 * 0               1                   2                   3
 * 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | IPv6 Dsp      | IPv6 header and payload ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 */
static void
compress_hdr_ipv6(sixlowpanaddr_t *sixlowpan_destaddr) {
  *sixlowpan_ptr = SICSLOWPAN_DISPATCH_IPV6;
  sixlowpan_hdr_len += SICSLOWPAN_IPV6_HDR_LEN;
  memcpy(sixlowpan_ptr + sixlowpan_hdr_len, UIP_IP_BUF, UIP_IPH_LEN);
  sixlowpan_hdr_len += UIP_IPH_LEN;
  uncomp_hdr_len += UIP_IPH_LEN;
  return;
}
/** @} */
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_IPV6*/



/*--------------------------------------------------------------------*/
/** \name Input/output functions common to all compression schemes
 * @{                                                                 */
/*--------------------------------------------------------------------*/
/**
 * \brief This function is called by the 6lowpan code to send out a
 * packet.
 * \param dest the link layer destination address of the packet
 */
static void
send_packet(sixlowpanaddr_t *dest)
{

  uint16_t destaddr = htons(*(uint16_t *)dest);

  if (destaddr == 0) {
	  destaddr = 0xffff; /*16-bit broadcast address */
	  return; /* We don't send broadcast packets on 802.15.4 though! */
  }

  //Send packet
  macsixlowpanDataRequest(destaddr, sixlowpanbuf_datalen(), sixlowpanbuf_dataptr());

  sixlowpanbuf_clear();
}

/** \brief Take an IP packet and format it to be sent on an 802.15.4
 *  network using 6lowpan.
 *  \param localdest The MAC address of the destination
 *
 *  The IP packet is initially in uip_buf. Its header is compressed
 *  and if necessary it is fragmented. The resulting
 *  packet/fragments are put in sixlowpanbuf and delivered to the 802.15.4
 *  MAC.
 */
u8_t
sicslowpan_send(uip_lladdr_t *localdest)
{
  /* The MAC address of the destination of the packet */
  sixlowpanaddr_t dest;


  /* init */
  uncomp_hdr_len = 0;
  sixlowpan_hdr_len = 0;

  /* reset sixlowpan buffer */
  sixlowpanbuf_clear();
  sixlowpan_ptr = sixlowpanbuf_dataptr();


  /*
   * RUM doesn't use broadcast - it's not good for mesh networks.
   * Instead we should look at NS, and guess at link-local and IP address
   * based on target address for instance.
   */
  if (localdest == NULL) {

	  /* NS Packet */
	  if ((UIP_IP_BUF->proto == UIP_PROTO_ICMP6) && (UIP_ICMP_BUF->type == ICMP6_NS)){
		   dest.u8[0] = UIP_ND6_NS_BUF->tgtipaddr.u8[14];
		   dest.u8[1] = UIP_ND6_NS_BUF->tgtipaddr.u8[15];

		   /* Force IP address to most compressible for 6lowpan */
		   UIP_IP_BUF->destipaddr = UIP_ND6_NS_BUF->tgtipaddr;

	  } else {
		  return 0;
	  }
  } else {
	  dest.u8[0] = localdest->addr[0];
	  dest.u8[1] = localdest->addr[1];
  }



  PRINTF("sicslowpan output: sending packet len %d\n", uip_len);

  /* Try to compress the headers */
#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1
  compress_hdr_hc1(&dest);
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1*/
#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_IPV6
  compress_hdr_ipv6(&dest);
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_IPV6*/
#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01
  compress_hdr_hc01(&dest);
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01*/
  PRINTF("sicslowpan output: header of len %d\n", sixlowpan_hdr_len);

  if(uip_len - uncomp_hdr_len > MAC_MAX_PAYLOAD - sixlowpan_hdr_len) {
#if SICSLOWPAN_CONF_FRAG
    /*
     * The outbound IPv6 packet is too large to fit into a single 15.4
     * packet, so we fragment it into multiple packets and send them.
     * The first fragment contains frag1 dispatch, then
     * IPv6/HC1/HC01/HC_UDP dispatchs/headers.
     * The following fragments contain only the fragn dispatch.
     */

    /* Create 1st Fragment */
    PRINTF("sicslowpan output: 1rst fragment ");

    /* move HC1/HC01/IPv6 header */
    memmove(sixlowpan_ptr + SICSLOWPAN_FRAG1_HDR_LEN, sixlowpan_ptr, sixlowpan_hdr_len);

    /*
     * FRAG1 dispatch + header
     * Note that the length is in units of 8 bytes
     */
    SIXLOWPAN_FRAG_BUF->dispatch_size =
      htons((SICSLOWPAN_DISPATCH_FRAG1 << 8) | uip_len);
    SIXLOWPAN_FRAG_BUF->tag = htons(my_tag);

    /* Copy payload and send */
    sixlowpan_hdr_len += SICSLOWPAN_FRAG1_HDR_LEN;
    sixlowpan_payload_len = (MAC_MAX_PAYLOAD - sixlowpan_hdr_len) & 0xf8;
    PRINTF("(len %d, tag %d)\n", sixlowpan_payload_len, my_tag);
    memcpy(sixlowpan_ptr + sixlowpan_hdr_len,
           (void *)UIP_IP_BUF + uncomp_hdr_len, sixlowpan_payload_len);
    sixlowpanbuf_set_datalen(sixlowpan_payload_len + sixlowpan_hdr_len);
    send_packet(&dest);

    /* set processed_ip_len to what we already sent from the IP payload*/
    processed_ip_len = sixlowpan_payload_len + uncomp_hdr_len;

    /*
     * Create following fragments
     * Datagram tag is already in the buffer, we need to set the
     * FRAGN dispatch and for each fragment, the offset
     */
    sixlowpan_hdr_len = SICSLOWPAN_FRAGN_HDR_LEN;
    SIXLOWPAN_FRAG_BUF->dispatch_size =
      htons((SICSLOWPAN_DISPATCH_FRAGN << 8) | uip_len);
    sixlowpan_payload_len = (MAC_MAX_PAYLOAD - sixlowpan_hdr_len) & 0xf8;
    while(processed_ip_len < uip_len){
      PRINTF("sicslowpan output: fragment ");
      SIXLOWPAN_FRAG_BUF->offset = processed_ip_len >> 3;

      /* Copy payload and send */
      if(uip_len - processed_ip_len < sixlowpan_payload_len){
        /* last fragment */
        sixlowpan_payload_len = uip_len - processed_ip_len;
      }
      PRINTF("(offset %d, len %d, tag %d)\n",
             processed_ip_len >> 3, sixlowpan_payload_len, my_tag);
      memcpy(sixlowpan_ptr + sixlowpan_hdr_len,
             (void *)UIP_IP_BUF + processed_ip_len, sixlowpan_payload_len);
      sixlowpanbuf_set_datalen(sixlowpan_payload_len + sixlowpan_hdr_len);
      send_packet(&dest);
      processed_ip_len += sixlowpan_payload_len;
    }

    /* end: reset global variables  */
    my_tag++;
    processed_ip_len = 0;
#else /* SICSLOWPAN_CONF_FRAG */
    PRINTF("sicslowpan output: Packet too large to be sent without fragmentation support; dropping packet\n");
    return 0;
#endif /* SICSLOWPAN_CONF_FRAG */
  } else {
    /*
     * The packet does not need to be fragmented
     * copy "payload" and send
     */
    memcpy(sixlowpan_ptr + sixlowpan_hdr_len, (void *)UIP_IP_BUF + uncomp_hdr_len,
           uip_len - uncomp_hdr_len);
    sixlowpanbuf_set_datalen(uip_len - uncomp_hdr_len + sixlowpan_hdr_len);
    send_packet(&dest);
  }
  return 1;
}

/*--------------------------------------------------------------------*/
/** \brief Process a received 6lowpan packet.
 *
 *  The 6lowpan packet is put in sixlowpanbuf by the MAC. If its a frag1 or
 *  a non-fragmented packet we first uncompress the IP header. The
 *  6lowpan payload and possibly the uncompressed IP header are then
 *  copied in siclowpan_buf. If the IP packet is complete it is copied
 *  to uip_buf and the IP layer is called.
 *
 * \note We do not check for overlapping sicslowpan fragments
 * (it is a SHALL in the RFC 4944 and should never happen)
 */
void sixlowpan_DataIndication(ftData *frame, uint8_t payloadlen)
{
	tuip_interface_set(INTERFACE_802154);

  /* size of the IP packet (read from fragment) */
  u16_t frag_size = 0;
  /* offset of the fragment in the IP packet */
  u8_t frag_offset = 0;
#if SICSLOWPAN_CONF_FRAG
  /* tag of the fragment */
  u16_t frag_tag = 0;
#endif /*SICSLOWPAN_CONF_FRAG*/

  /* init */
  uncomp_hdr_len = 0;
  sixlowpan_hdr_len = 0;

  /* The MAC puts the 15.4 payload inside the SIXLOWPAN data buffer */
  sixlowpan_ptr = sixlowpanbuf_dataptr();

  /* This might be used by the 'awake' code */
  uint16_t srcaddr = frame->originAddr;

  sixlowpanbuf_set_datalen(payloadlen);
  memcpy(sixlowpan_ptr, frame->payload, payloadlen);
  sixlowpanbuf_set_addr(SIXLOWPANBUF_ADDR_SENDER, frame->originAddr);
  sixlowpanbuf_set_addr(SIXLOWPANBUF_ADDR_RECEIVER, frame->finalDestAddr);
  //tuip_panid[0] = MSB(frame->panid);
  //tuip_panid[1] = LSB(frame->panid);

  /* Very first thing we check - possible Atmel-specific extension to
   * support sleeping nodes. If first byte of payload is 0x01, this is
   * not a normal 6LoWPAN packet. */
  if (*(uint8_t *)sixlowpan_ptr == ATMEL_GET_AWAKE)
  {
      sixlowpanSleep_awakeReceived(srcaddr);
      return;
  }

#if SICSLOWPAN_CONF_FRAG
  /* if reassembly timed out, cancel it */
  if(timer_expired(&reass_timer)){
    sicslowpan_len = 0;
    processed_ip_len = 0;
  }
  /*
   * Since we don't support the mesh and broadcast header, the first header
   * we look for is the fragmentation header
   */
  switch((ntohs(SIXLOWPAN_FRAG_BUF->dispatch_size) & 0xf800) >> 8) {
    case SICSLOWPAN_DISPATCH_FRAG1:
      PRINTF("sicslowpan input: FRAG1 ");
      frag_offset = 0;
      frag_size = (ntohs(SIXLOWPAN_FRAG_BUF->dispatch_size) & 0x07ff);
      frag_tag = ntohs(SIXLOWPAN_FRAG_BUF->tag);
      PRINTF("size %d, tag %d, offset %d)\n",
             frag_size, frag_tag, frag_offset);
      sixlowpan_hdr_len += SICSLOWPAN_FRAG1_HDR_LEN;

      break;
    case SICSLOWPAN_DISPATCH_FRAGN:
      /*
       * set offset, tag, size
       * Offset is in units of 8 bytes
       */
      PRINTF("sicslowpan input: FRAGN ");
      frag_offset = SIXLOWPAN_FRAG_BUF->offset;
      frag_tag = ntohs(SIXLOWPAN_FRAG_BUF->tag);
      frag_size = (ntohs(SIXLOWPAN_FRAG_BUF->dispatch_size) & 0x07ff);
      PRINTF("size %d, tag %d, offset %d)\n",
             frag_size, frag_tag, frag_offset);
      sixlowpan_hdr_len += SICSLOWPAN_FRAGN_HDR_LEN;
      break;
    default:
      break;
  }

  if(processed_ip_len > 0) {
    /* reassembly is ongoing */
    if((frag_size > 0 &&
        (frag_size != sicslowpan_len ||
         reass_tag  != frag_tag ||
         !sixlowpanaddr_cmp(&frag_sender, sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_SENDER))))  ||
       frag_size == 0) {
      /*
       * the packet is a fragment that does not belong to the packet
       * being reassembled or the packet is not a fragment.
       */
      PRINTF("sicslowpan input: Dropping 6lowpan packet\n");
      return;
    }
  } else {
    /*
     * reassembly is off
     * start it if we received a fragment
     */
    if(frag_size > 0){
      sicslowpan_len = frag_size;
      reass_tag = frag_tag;
      timer_set(&reass_timer, SICSLOWPAN_REASS_MAXAGE*CLOCK_SECOND);
      PRINTF("sicslowpan input: INIT FRAGMENTATION (len %d, tag %d)\n",
             sicslowpan_len, reass_tag);
      sixlowpanaddr_copy(&frag_sender, sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_SENDER));
    }
  }

  if(sixlowpan_hdr_len == SICSLOWPAN_FRAGN_HDR_LEN) {
    /* this is a FRAGN, skip the header compression dispatch section */
    goto copypayload;
  }
#endif /* SICSLOWPAN_CONF_FRAG */

  /* Process next dispatch and headers */
  switch(SIXLOWPAN_HC1_BUF->dispatch) {
#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1
    case SICSLOWPAN_DISPATCH_HC1:
      PRINTF("sicslowpan input: HC1\n");
      uncompress_hdr_hc1(frag_size);
      break;
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC1*/
#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01
    case SICSLOWPAN_DISPATCH_IPHC:
      PRINTF("sicslowpan input: IPHC\n");
      uncompress_hdr_hc01(frag_size);
      break;
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01*/
    case SICSLOWPAN_DISPATCH_IPV6:
      PRINTF("sicslowpan input: IPV6\n");
      sixlowpan_hdr_len += SICSLOWPAN_IPV6_HDR_LEN;

      /* Put uncompressed IP header in sicslowpan_buf. */
      memcpy(SICSLOWPAN_IP_BUF, sixlowpan_ptr + sixlowpan_hdr_len, UIP_IPH_LEN);

      /* Update uncomp_hdr_len and sixlowpan_hdr_len. */
      sixlowpan_hdr_len += UIP_IPH_LEN;
      uncomp_hdr_len += UIP_IPH_LEN;
      break;
    default:
      /* unknown header */
      PRINTF("sicslowpan input: unknown dispatch\n");
      return;
  }


#if SICSLOWPAN_CONF_FRAG
 copypayload:
#endif /*SICSLOWPAN_CONF_FRAG*/
  /*
   * copy "payload" from the sixlowpan buffer to the sicslowpan_buf
   * if this is a first fragment or not fragmented packet,
   * we have already copied the compressed headers, uncomp_hdr_len
   * and sixlowpan_hdr_len are non 0, frag_offset is.
   * If this is a subsequent fragment, this is the contrary.
   */
  sixlowpan_payload_len = sixlowpanbuf_datalen() - sixlowpan_hdr_len;
  memcpy((void *)SICSLOWPAN_IP_BUF + uncomp_hdr_len + (u16_t)(frag_offset << 3), sixlowpan_ptr + sixlowpan_hdr_len, sixlowpan_payload_len);

  /* update processed_ip_len if fragment, sicslowpan_len otherwise */

#if SICSLOWPAN_CONF_FRAG
  if(frag_size > 0){
    if(processed_ip_len == 0) {
      processed_ip_len += uncomp_hdr_len;
    }
    processed_ip_len += sixlowpan_payload_len;
  } else {
#endif /* SICSLOWPAN_CONF_FRAG */
    sicslowpan_len = sixlowpan_payload_len + uncomp_hdr_len;
#if SICSLOWPAN_CONF_FRAG
  }

  /*
   * If we have a full IP packet in sicslowpan_buf, deliver it to
   * the IP stack
   */
  if(processed_ip_len == 0 || (processed_ip_len == sicslowpan_len)){
    PRINTF("sicslowpan input: IP packet ready (length %d)\n",
           sicslowpan_len);
    memcpy((void *)UIP_IP_BUF, (void *)SICSLOWPAN_IP_BUF, sicslowpan_len);
    uip_len = sicslowpan_len;
    sicslowpan_len = 0;
    processed_ip_len = 0;
#endif /* SICSLOWPAN_CONF_FRAG */

    ipv6_in_sixlowpan(htons(*((uint16_t *)sixlowpanbuf_addr(SIXLOWPANBUF_ADDR_SENDER))));
#if SICSLOWPAN_CONF_FRAG
  }
#endif /* SICSLOWPAN_CONF_FRAG */
  return;
}
/** @} */

/*--------------------------------------------------------------------*/
/* \brief 6lowpan add context function                                */
/*--------------------------------------------------------------------*/
void
sicslowpan_add_context(uip_ipaddr_t * prefix)
{
	  for(i = 1; i < SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS; i++) {
	    if (addr_contexts[i].used == 0) {
	  	  addr_contexts[1].used = 1;
	  	  addr_contexts[1].number = i;

	  	  for(i = 0; i < 8; i++) {
	  		  addr_contexts[1].prefix[i] = prefix->u8[i];
	  	  }

	  	  break; /* Done */
	    }
	  }

	  return;
}

/*--------------------------------------------------------------------*/
/* \brief 6lowpan init function (called by the MAC layer)             */
/*--------------------------------------------------------------------*/
void
sicslowpan_init(void)
{

#if SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01
  /*
   * Initialize the address contexts
   * Context 00 is link local context
   * Other contexts are NULL at init
   */
  addr_contexts[0].used = 1;
  addr_contexts[0].number = SICSLOWPAN_IPHC_ADDR_CONTEXT_LL;
  addr_contexts[0].prefix[0] = 0xfe;
  addr_contexts[0].prefix[1] = 0x80;

  for(i = 1; i < SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS; i++) {
    addr_contexts[i].used = 0;
  }
#endif /*SICSLOWPAN_CONF_COMPRESSION == SICSLOWPAN_CONF_COMPRESSION_HC01*/
}
/*--------------------------------------------------------------------*/
/** @} */

#endif
