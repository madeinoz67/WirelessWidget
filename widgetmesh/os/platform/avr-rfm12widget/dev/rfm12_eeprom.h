#ifndef RFM12_EEPROM_H_
#define RFM12_EEPROM_H_
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
 Created on: 12/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file rfm12_eeprom.h
 \ingroup

 \brief
 */
/**************************************************************************/

#include <avr/eeprom.h>
#include "types.h"

void rfm12_eeprom_write(U16 addr, U8 *buf, U16 size);
void rfm12_eeprom_read(U16 addr, U8 *buf, U16 size);

#endif /* RFM12_EEPROM_H_ */
