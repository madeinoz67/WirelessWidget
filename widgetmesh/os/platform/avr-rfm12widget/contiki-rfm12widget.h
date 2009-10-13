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

 Created on: 01/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file contiki-rfm12widget.h
 \ingroup rfm12widget

 \brief Contiki kernel for RFM12WidgetBoard

 */
#ifndef __CONTIKI_RFM12WIDGET_H__
#define __CONTIKI_RFM12WIDGET_H__

#include "contiki.h"
#include "contiki-lib.h"

#include "lib/sensors.h"

#include "dev/rs232.h"
#include "dev/watchdog.h"

//#include "dev/button-sensor.h"
//#include "dev/radio-sensor.h"
//#include "dev/battery-sensor.h"
//#include "dev/temperature-sensor.h"

#include "dev/rfm12_drvr.h"

void init_lowlevel(void);
void init_net(void);

#endif /* #ifndef __CONTIKI_RFM12WIDGET_H__ */
