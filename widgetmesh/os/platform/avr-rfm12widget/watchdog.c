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
 $Id$
 Created on: 01/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file watchdog.c
 \ingroup rfm12widgetboard

 \brief watchdog timer implementation for the WidgetBoard
 */
/**************************************************************************/

#include "dev/watchdog.h"
#include <avr/wdt.h>

static int stopped = 0;

 /*---------------------------------------------------------------------------*/
 void
 watchdog_init(void)
 {
    stopped = 0;
    watchdog_stop();
 }
 /*---------------------------------------------------------------------------*/
 void
 watchdog_start(void)
 {
     stopped--;
     if(!stopped){
         wdt_enable(WDTO_1S);
     }
 }
 /*---------------------------------------------------------------------------*/
 void
 watchdog_periodic(void)
 {
    if(!stopped){
       wdt_reset();
    }
 }
 /*---------------------------------------------------------------------------*/
 void
 watchdog_stop(void)
 {
    wdt_disable();
    stopped++;
 }
 /*---------------------------------------------------------------------------*/
 void
 watchdog_reboot(void)
 {
    wdt_enable(0);
    while(1);
 }
 /*---------------------------------------------------------------------------*/
