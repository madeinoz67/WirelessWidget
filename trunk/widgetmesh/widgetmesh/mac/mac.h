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
 Author: seaton

 \file mac.h
 \ingroup

 \brief
 */
/**************************************************************************/

#ifndef MAC_H_
#define MAC_H_

#include "constants.h"
#include "contiki.h"
#include "types.h"
#include "buf.h"

/**
 *  \brief  frame control field
 */
typedef union{
    /** \brief  Structure of bitfields for the FCF */
    struct{
        uint8_t frameType : 3;          /**< Frame type field */
        uint8_t ackRequired : 1;        /**< Is an ack frame required? */
        uint8_t destAddrMode : 1;       /**< Destination address mode */
        uint8_t srcAddrMode : 1;        /**< Source address mode*/
    };
    uint8_t byte_val;                   /**< A byte-wide value for the entire FCF */
}fcf_t;

/**
 *   \brief
 */
typedef struct
{
    uint8_t      cap_info;      ///< Capability info for this device
} cmd_assoc_req_t;

typedef struct
{
    uint8_t     addr;                   ///< Short address given to the device requesting association
    uint8_t     assoc_status;           ///< Assocation request status
} cmd_assoc_resp_t;


typedef struct
{
    uint8_t cmd_id;                             ///< MAC cmd frame command ID
    union
    {
        cmd_assoc_req_t         assoc_req;      ///< Assocation request cmd frame
        cmd_assoc_resp_t        assoc_resp;     ///< Association response cmd frame
    };
} mac_cmd_t;



typedef struct _pan_descr_t
{
    uint8_t     coord_addr;     ///< Address of the router sending the beacon
    uint8_t     coord_pan_id;   ///< Network ID of this network
    uint8_t     superfrm_spec;  ///< Misc info for this network
    bool        rtr_cap;        ///< Capacity available for routers to join
    uint8_t     depth;          ///< Network depth of this router
    bool        end_dev_cap;    ///< Capacity available for end devices to join
    bool        pot_parent;     ///< Potential Parent for this device
} pan_descr_t;

typedef struct
{
  uint8_t           src_addr;       ///< Src address (usually this device's address)
  uint8_t           dest_addr;      ///< Dest address
  buffer_t          *buf;           ///< Data buffer to be transmitted
  uint8_t           msdu_handle;    ///< Data handle identifier for this frame
  uint8_t           tx_options;     ///< Transmission options (ack request, indirect)
} mac_data_req_t;

typedef struct
{
  uint8_t       coord_addr;     ///< Address of parent to join
  uint8_t       coord_pan_id;   ///< PAN ID of network to join
  uint8_t       capability;     ///< Capability info of this device
} mac_assoc_req_t ;


typedef struct
{
  uint8_t     dev_addr;       ///< Address of device requesting association
  uint8_t     capability;     ///< Capability info of device requesting association
} mac_assoc_ind_t;

typedef struct
{
  uint8_t       addr;                   ///< Network address
  uint8_t       status;                 ///< Status of association request
} mac_assoc_resp_t;


/*! Main process for the MAC layer */
PROCESS_NAME(mac_process);

// mac
void wm_mac_init(void);

#endif /* MAC_H_ */
