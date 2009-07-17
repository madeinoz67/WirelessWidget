/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: main.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#include <stdlib.h>
#include <stdio.h>
#include "stdbool.h"
#include <avr/interrupt.h>
#include <string.h>
#include "radio.h"
#include "mac.h"
#include "mac_event.h"
#include "timer.h"
#include "serial.h"
#include "mac_start.h"
#include "mac_data.h"
#include "mac_associate.h"
#include "system.h"
#include <avr/wdt.h>
#if PLATFORM == RAVENUSB
#include "ravenusb/usb_task.h"
#include "ravenusb/cdc_task.h"
#else
// Make the compiler accept these unused function names
#define usb_task_init()
#define cdc_task_init()
#define usb_task()
#define cdc_task()
#endif // RAVENUSB

/* On restart WDT will be enabled with shortest timeout if previously
   enabled. Must turn it off before anything else, attempting to
   disable within the main() function will take too long. */
   
void disable_wdt(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3")));
    void disable_wdt(void)
    {
      MCUSR = 0;
      wdt_disable();
    }

/**
   @brief Main function of the program.
*/
int main(void)
{
    // Setup clock speed
    halSetupClock();

    if (PLATFORM == RAVENUSB)
        // For USB
        Clear_prescaler();

    // init serial
    if (SERIAL)
        serial_init(NULL);

    // Init the timer system for the MAC
    timerInit();

    // Init the USB connection on USB platforms
    if (PLATFORM == RAVENUSB)
    {
        usb_task_init();
        cdc_task_init();
    }

    // Init the (sample) application
    appInit();

     // Main forever loop for the application.
    for(;;)
    {
        // Turn interrupts on if they are off.  They should always be on
        // at this point in the loop, so just turn them on every time, in
        // case interrupts were inadvertently turned off elsewhere.
        sei();

        // Task functions called from main loop.  Either add your own task loop
        // or edit the example appTask().
        appTask();
        macTask();
        // USB tasks
        if (PLATFORM == RAVENUSB)
        {
            usb_task();
            cdc_task();
        }
    }
    return 0;
}
