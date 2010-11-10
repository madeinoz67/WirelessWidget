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
 Created on: 25/09/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file mac.c
 \ingroup mac MAC - Media Access Control Layer

 \brief
 */
/**************************************************************************/

#include "widgetmesh.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

extern process_event_t event_mac_rx;

/*********************** Process Declarations **********************/
PROCESS(mac_process, "MAC Process");            ///< Main process for the MAC layer
/*******************************************************************/

void wm_mac_init(void)
{
    // Set up the processes.
    // First start the mac process, then allocate the rx event
    process_start(&mac_process, NULL);
    event_mac_rx = process_alloc_event();

}


/**************************************************************************/
/*!
    Handle the rx events from the mac process. If the driver receives a valid frame,
    it will send an event to the mac
    process. The mac process will then call the event handler which retrieves
    the frame from the rx queue and parses it. Once parsed, it will be handled
    according to the frame type. If its a command frame, it gets sent to the
    command handler, a data frame gets sent to the next higher layer, etc...
*/
/**************************************************************************/
static void mac_eventhandler(process_event_t event)
{
    buffer_t *buf; //*buf_out;

    if (event == event_mac_rx)
      {
        LOG("MAC_EVENTHANDLER: Rx event occurred.\n");
      }
}


/**************************************************************************/
/*!
    This is the main process for the mac layer. Actually, its more like
    a gateway into the protocol stack. All rx data must post an  event to this process.
*/
/**************************************************************************/
//lint -e{715} Symbol 'data' not referenced
//lint -e{818} Pointer parameter 'data' could be declared as pointing to const
PROCESS_THREAD(mac_process, ev, data)
{
    PROCESS_BEGIN();

    //lint -e{716} while(1) ...
    while (1)
    {
        PROCESS_WAIT_EVENT();
        mac_eventhandler(ev);
    }

    //line -e{527} Warning 527: Unreachable code at token ';'
    // due to contiki process macro. its okay.
    PROCESS_END();
}
