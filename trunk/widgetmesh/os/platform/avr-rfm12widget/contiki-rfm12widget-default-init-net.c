//$Id$//
/***************************************************************************
 Copyright (C) 2009 Stephen Eaton

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 Originally written by Stephen Eaton.
 Please post support questions to the WirelessWidget Discussion Group.
 http://code.google.com/p/strobit/
 ***************************************************************************/
/*!

 Created on: 01/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file contiki-rfm12widget-default-init-net.c
 \ingroup rfm12widget

 \brief network initialization for the RFM12WidgetBoard

 */
/**************************************************************************/
#include "contiki-rfm12widget.h"
//#include "contiki-net.h"


void
init_net(void)
{
  //start radio driver
   rfm12_drvr_init(rfm12_config);

/*  uip_ipaddr_t ipprefix;

  uip_ip6addr(&ipprefix, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);

  uip_netif_addr_add(&ipprefix, UIP_DEFAULT_PREFIX_LEN, 0, AUTOCONF);
  uip_nd6_prefix_add(&ipprefix, UIP_DEFAULT_PREFIX_LEN, 0);
*/
}
