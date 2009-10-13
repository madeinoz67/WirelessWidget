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

 \file contiki-rfm12widget-main.c
 \ingroup rfm12widget

 \brief RFM12Widget platform mainline.

 */
/**************************************************************************/

#include <avr/pgmspace.h>
//#include <avr/fuse.h>
//#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <stdio.h>
#include "contiki-rfm12widget.h"

//#include "lib/mmem.h"
//#include "loader/symbols-def.h"
//#include "loader/symtab.h"
//#include <stdbool.h>
//#include "mac.h"
//#include "sicslowmac.h"
//#include "sicslowpan.h"
//#include "ieee-15-4-manager.h"

#include "contiki.h"
//#include "contiki-net.h"
#include "contiki-lib.h"

//#include "dev/rs232.h"


//#include "sicslowmac.h"

#if 0
FUSES =
	{
		.low = 0xe6,
		.high = 0xdf,
		.extended = 0xff,
	};


/* Put default MAC address in EEPROM */
//uint8_t mac_address[8] EEMEM = {0x02, 0x11, 0x22, 0xff, 0xfe, 0x33, 0x44, 0x55};
#endif

PROCINIT(&etimer_process);

int
main(void)
{
  //calibrate_rc_osc_32k(); //CO: Had to comment this out

  /* Initialize hardware */
  init_lowlevel();

  /* Clock */
  clock_init();

  /* Process subsystem */
  process_init();

  /* Register initial processes */
  procinit_init();

  /* Autostart processes */
  autostart_start(autostart_processes);

  //start the network
  //init_net();

  // initialiase watchdog timer
  watchdog_init();

  static int r;

  printf_P(PSTR("\n********BOOTING RFM12Widget*********\n"));

  printf_P(PSTR("System online.\n"));

  /* Main scheduler loop */
  while(1) {
    process_run();

    watchdog_start();

    do{
    	watchdog_periodic();
        r = process_run();
    } while(r>0);

    watchdog_stop();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();

  }
  return 0;
}
