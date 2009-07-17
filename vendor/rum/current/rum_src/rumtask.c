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
  $Id: rumtask.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

// Utasker
#include "config.h"

#include "mac_event.h"
#include "arm_app.h"
#include "system.h"
#include "arm_timer.h"
#include "arm_timer_event.h"


// ---------- Implementation ----------
/**
    @brief This is the mainloop for RUM tasking. The iState only runs once during
    init and then allows macTask() and timerTask() to run in a loop.

    @param ptrTaskTable Queue ID for task input.
*/
extern void fnTaskRUM(TTASKTABLE *ptrTaskTable)
{
    static int iState = 0;
    volatile USOCKET Socket;

    // Only do the initialization 1 time.
    if(!iState)
    {
        iState = 1;

        // Print out our IP address to the serial port when enabled.
        CHAR cBuf[16];
        fnDebugMsg("IP address = ");
        fnIPStr(&network.ucOurIP[0], cBuf);
        fnDebugMsg(cBuf);

        // Init the app.
        appInit();

        fnDebugMsg("\r\nRUM Tasking activated... \r\n ");

        // Set up the timer used for the application.
        timerInit();
        timerStart();

        // Get TCP Socket and listen on FTP control port, INFINITE_TIMEOUT
        // (0xFFFF) timeout means never timeout.
        Socket = fnGetTCP_Socket(TOS_MINIMISE_DELAY, INFINITE_TIMEOUT, tcpListener);

        // For functions in other files expose the Socket as global host_socket.
        host_socket = Socket;

        // Listen on port 6464 (a server listens) and the '0' parameter says
        // to use the maximum window size a connection can accept.
        host_socket2 = fnTCP_Listen(host_socket, TCP_PORT, 0);

        // Create a file on the SD card for recording data.
        armAppInit();
    }

    // Call mac event
    macTask();

    // Call timer task.
    timerTask();
}
