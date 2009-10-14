#ifndef RFM12_FRAME_H_
#define RFM12_FRAME_H_
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
 Created on: 13/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file rfm12_frame.h
 \ingroup

 \brief
 */
/**************************************************************************/
#include <stdint.h>

/******************************************************************************
 * @name        RFM12 Packet specification
 * @{
 */

const uint8_t syncword[2] = {0x2D,0xD4};

// header: number of bytes in packet including header
struct rfm12_header {
  uint8_t  length;      //< length of packet
  uint8_t  src_addr;    //< senders address
  uint8_t  dest_addr;   //< receivers address
  uint8_t  ctrl;        //< packet control
} __attribute__((packed));

#define PREAMBLE_SIZE           6
#define PREAMBLE                0xAA
#define SYNCWORD_SIZE           (sizeof (syncword))
#define HDR_SIZE                (sizeof (struct rfm12_header))
#define CRC_SIZE                2
#define TAIL_SIZE               2
#define TAIL                    0xAA
///@}
#endif /* RFM12_FRAME_H_ */
