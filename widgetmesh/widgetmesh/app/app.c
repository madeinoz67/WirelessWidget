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
 Created on: 20/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file app.c
 \ingroup

 \brief
 */
/**************************************************************************/

#include "widgetmesh.h"

/*********************** Process Declarations **********************/
PROCESS(app_process, "APP Process"); ///< Declaration for the APP process
/*******************************************************************/

extern process_event_t event_app_tx;
extern process_event_t event_app_rx;

void app_init(void)
{
  event_app_tx     = process_alloc_event();
  event_app_rx     = process_alloc_event();

  process_start(&app_process, NULL);
}


/**
 * This is the exit point of the APP layer Received data.
 *
 * We return any received data to the user process for them to handle
 */
void
app_rx_handler(void)
{

}

/**
 * This is the exit path of the APP layer
 * The data is sent out to the lower layer
 *
 */
void
app_tx_handler(void)
{

}

/**************************************************************************/
/*!
    This is the main process that handles the different APP events.
*/
/**************************************************************************/
//lint -e{715} Info 715: Symbol 'data' (line 257) not referenced
//lint -e{818} Pointer parameter 'data' could be declared as pointing to const
PROCESS_THREAD(app_process, ev, data)
{
    PROCESS_BEGIN();

    //lint -e{716} Info 716: while(1) ...
    // looping forever on purpose
    while (1)
    {
        PROCESS_WAIT_EVENT();
        if (ev == event_app_rx)
        {
            app_rx_handler();
        }
        else if (ev == event_app_tx)
        {
            app_tx_handler();
        }
    }
    PROCESS_END();
}
