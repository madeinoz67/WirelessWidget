#ifndef APP_H_
#define APP_H_
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

 \file app.h
 \ingroup

 \brief
 */
/**************************************************************************/

/****************************************************************/
/*!
    enumerated status definitions for the application framework
*/
/****************************************************************/
enum APP_STATUS_ENUMS
{
    // app status enums
    APP_SUCCESS              = 0x00,     ///< Application request successfully executed
    APP_INV_REQUESTTYPE      = 0x80,     ///< Invalid application request
    APP_NOT_SUPPORTED        = 0x81,     ///< Requested service not supported on this device
    APP_TIMEOUT              = 0x82,     ///< Timeout occurred on requested operation
    APP_INSUFFICIENT_SPACE   = 0x83,     ///< No storage space left for requested operation
    APP_NOT_PERMITTED        = 0x84,     ///< Requested operation is not permitted
    APP_TABLE_FULL           = 0x85,     ///< No table space to support requested operation
    APP_NO_FREE_BUFS         = 0x86,     ///< No free buffers for requested operation
    APP_ERROR                = 0x87      ///< Generic error for requested operation
};

/*! Main process for the APP layer */
//PROCESS_NAME(app_process);

void app_init(void);

void app_tx(uint8_t dest_addr, uint8_t * data, uint8_t len, uint8_t mode);
void app_rx(buffer_t *buf);

void app_rx_handler(void);
void app_tx_handler(void);

#endif /* APP_H_ */
