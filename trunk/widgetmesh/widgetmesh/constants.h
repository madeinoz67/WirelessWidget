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
 Please post support questions to the Strobit Discussion Group.
 http://code.google.com/p/strobit/
 ***************************************************************************/
/*!
 $Id: $
 Created on: 16/09/2009
 Author: seaton

 \file constants.h
 \ingroup

 \brief
 */
/**************************************************************************/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define MAX_BUF_POOL_SIZE               2       ///< Define the number of frame buffers here

#define aMaxPHYPacketSize               66      ///< Actual size of frames we're using

// max data payload sizes
#define MAX_PHY_PAYLOAD                 65      ///< Radio payload size 66 - 1
#define MAX_MAC_PAYLOAD                 61      ///< Max MAC payload size 66 - 5
#define MAX_DL_PAYLOAD                  110     ///< Max NWK payload size 127 - 9 - 8
#define MAX_APP_PAYLOAD                 99      ///< Max Application payload size 127 - 9 - 8 - 10 (currently 1 less...need to find out why)
#define DEFAULT_SCAN_DURATION           10      ///< Default time to wait before timing out a network or energy scan

// Capability Info
#define CAP_MAINS_POWERED               1                    ///< Mains powered - must be 1 if its a router.
#define CAP_RX_ON_WHEN_IDLE             CAP_MAINS_POWERED    ///< Rx on when idle - rx will be on when idle if its mains powered. otherwise not.
#
#endif /* CONSTANTS_H_ */
