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

 \file contiki-rfm12widget-default-init-lowlevel.c
 \ingroup rfm12widget

 \brief RFM12Widget Board low level hardware initialization

 */
/**************************************************************************/
#include "contiki-rfm12widget.h"


void
init_lowlevel(void)
{

  /* configure onboard LEDS */
  leds_init();

  /* Configure rs232 port for debugging */
  rs232_init(RS232_PORT_0, USART_BAUD_57600, USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);

  /* Redirect stdout to default port */
  rs232_redirect_stdout (RS232_PORT_0);
}
