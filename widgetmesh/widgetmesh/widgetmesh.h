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
 Created on: 16/09/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file widgetmesh.h
 \ingroup stack

 \brief WidgetMesh header file

	 The main header file for the WidgetMesh stack.
 */
/**************************************************************************/

#ifndef WIDGETMESH_H_
#define WIDGETMESH_H_

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "constants.h"
#include "types.h"
#include "buf.h"
//#include "app.h"
#include "dll.h"
//#include "mac.h"

#if (WIDGETMESH_DEBUG)
    #include <stdio.h>
#endif

enum
{
    WM_SUCCESS                 = 0,
    WM_SUCCESS_DATA_PENDING    = 1,
    WM_CHANNEL_ACCESS_FAILURE  = 3,
    WM_NO_ACK                  = 5,
    WM_INVALID                 = 7
} wm_result_t;


typedef struct
{
  uint8_t destination;
} wm_message_t;

// Widget Mesh Events
process_event_t event_wm_send_done;             //< Tx done
process_event_t event_wm_send_next;             //< Tx Send Next Message

process_event_t event_wm_receive;               //< rx event

process_event_t event_wm_neighbor_update;       //< Neighbor table updated
process_event_t event_wm_neighbor_admit;        //< Neighbor Added to table
process_event_t event_wm_neighbor_expired;      //< Neighbor entry expired
process_event_t event_wm_neighbor_evicted;      //< Neighbor removed

process_event_t event_dll_completed;            //< Data Link Layer completed

process_event_t event_mac_rx;                   //< MAC rx event


// Widget Mesh API
void wm_init(void);

wm_result_t wm_send(wm_message_t* msg);
wm_result_t wm_send_changed(wm_message_t *msg);
wm_result_t wm_send_cancel(wm_message_t *msg);

wm_neighbor_t* wm_neighbor_query(uint8_t address);
wm_neighbor_t* wm_neighbor_get(uint8_t i);
uint8_t wm_neighbor_max(void);

wm_result_t wm_neighbor_insert(wm_neighbor_t neighbor);
wm_result_t wm_neighbor_remove(wm_neighbor_t neighbor);

wm_result_t wm_neighbor_adjust(wm_neighbor_t neighbor);
wm_result_t wm_neighbor_listen(wm_neighbor_t neighbor);

wm_result_t wm_neighbor_find();
wm_result_t wm_neighbor_find_done();



wm_result_t

#endif /* WIDGETMESH_H_ */
