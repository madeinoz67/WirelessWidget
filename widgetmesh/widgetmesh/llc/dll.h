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

 \file dll.h
 \ingroup dll

 \brief  Data Link Layer header file

 */
/**************************************************************************/

#ifndef DLL_H_
#define DLL_H_

typedef struct
{
  uint8_t  address;             //< address of destination
  uint8_t *message;             //< pointer to message structure
  uint8_t  reliability  :1;     //< on or off

  uint16_t  delta_clock;        //< feedback - clock difference
} wm_neighbor_t;

// data link layer
void wm_dll_init();

#endif /* DLL_H_ */
