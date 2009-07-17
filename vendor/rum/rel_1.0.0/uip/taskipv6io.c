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

  $Id: taskipv6io.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/
/**
 * \addtogroup tuip
 * @{
 */

#include <string.h>
#include "config.h"
#include "net/uip.h"
#include "net/sicslowpan.h"
// RUM includes
#include "sixlowpan_wake.h"
#include "mac_associate.h" /* Needed for MAXNODES */

#if IPV6LOWPAN || defined(DOXYGEN)

extern uint8_t talked_to[MAXNODES];

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF          ((struct uip_udp_hdr *)&uip_buf[UIP_LLIPH_LEN])

/**
 * @brief Send a frame out on either ethernet or 802.15.4
 */
int tcpip_output(uip_lladdr_t * lladdr)
	{
	static const unsigned char ucIP_ProtV6[] = {(unsigned char)(PROTOCOL_IPv6>>8), (unsigned char)(PROTOCOL_IPv6)};
	unsigned char ucData[14];

	uip_lladdr_t newaddr;

		/** Ethernet **/
	if (interface_number == INTERFACE_ETHERNET) {

		/* Fix multicast link-level addresses */
		if (lladdr == NULL){


			newaddr.addr[0] = 0x33;
			newaddr.addr[1] = 0x33;
			newaddr.addr[2] = UIP_IP_BUF->destipaddr.u8[12];
			newaddr.addr[3] = UIP_IP_BUF->destipaddr.u8[13];
			newaddr.addr[4] = UIP_IP_BUF->destipaddr.u8[14];
			newaddr.addr[5] = UIP_IP_BUF->destipaddr.u8[15];

			lladdr = &newaddr;
		}


	    uMemcpy(&ucData[0], lladdr->addr, MAC_LENGTH);                      // Add datalink (Ethernet addresses) information
	    uMemcpy(&ucData[MAC_LENGTH], &network.ucOurMAC[0], MAC_LENGTH);
	    uMemcpy(&ucData[2*MAC_LENGTH], ucIP_ProtV6, sizeof(ucIP_ProtV6));

	    //Add datalink
	    fnWrite(Ethernet_handle, ucData, 14);

		//Add IP data
		fnWrite(Ethernet_handle, uip_buf, uip_len);

		//Done with you
		uip_len = 0;

		//Send frame
		fnWrite(Ethernet_handle, 0, 0);


		/** 6LowPan **/
	} else if (interface_number == INTERFACE_802154) {

	    /* Check for special 'w' message sent to port 61618. This is the
	     * specific 'wakeup' message used in this network and deserves
	     * special handling. */
	    if ((UIP_IP_BUF->proto == UIP_PROTO_UDP) &&
	         (is_addr_sixlowpan_compressible(&UIP_IP_BUF->destipaddr, lladdr)) &&
	         (HTONS(UIP_UDP_BUF->destport) == WAKEUP_PORT) &&
	         (*((char *)(&UIP_UDP_BUF->udpchksum) + 2) == 'w') )
	    {
	        //We are sending this node the 'WAKEUP' packet. Make note of that and look
	        //for a response.
	        sixlowpanSleep_wakeupSent(HTONS(UIP_IP_BUF->destipaddr.u16[7]), &UIP_IP_BUF->srcipaddr,
	                HTONS(UIP_UDP_BUF->srcport));
	    }

		sicslowpan_send(lladdr);

	}

	return 0;
	}


/**
 * @brief Processes an incoming IPv6 frame from the ethernet, and
 *        passes it to the uIPv6 stack
 */
int ipv6_in_ethernet(ETHERNET_FRAME *frame)
{
    /* Check destination MAC */
    if ((frame->ptEth->ethernet_destination_MAC[0] == 0x33) &&
       (frame->ptEth->ethernet_destination_MAC[0] == 0x33))
    {
        ; /* Multicast - always accept */
    }
    else
    {
        /* Unicast - check address carefully */
        uint8_t i;
        for(i = 0; i < 6; i++)
        {
            if(frame->ptEth->ethernet_destination_MAC[i] != uip_lladdr[INTERFACE_ETHERNET].addr[i])
                return 0;
        }
    }

    if (uip_len) {
        fnDebugMsg("Sorry, had to drop IPv6 packet\n");
        return -1;
    }

    tuip_interface_set(INTERFACE_ETHERNET);
    uip_len = frame->frame_size - 18; /* 14 Bytes of ethernet header, 4 bytes of checksum */
                                      /* Should get this frmo IPv6 instead maybe? */
    memcpy(uip_buf, frame->ptEth->ucData, uip_len);

    uip_input();

    /* This could cause problems - need to check */
    tcpip_ipv6_output();


    return 0;
}

/**
 * @brief Processes an incoming IPv6 frame from the 802.15.4, and
 *        passes it to the uIPv6 stack
 */
int ipv6_in_sixlowpan(uint16_t src)
{
    /* Remember we've talked to this node... */
    if (src < MAXNODES)
    {
        talked_to[src] = 1;
    }

    /* Check if this node is responding to a wake up */
    if (src < MAXNODES)
    {
        sixlowpanSleep_dataReceived(src);
    }

    tuip_interface_set(INTERFACE_802154);
    uip_input();
    tcpip_ipv6_output();

    return 0;
}

#endif

/*
 * @}
 */


