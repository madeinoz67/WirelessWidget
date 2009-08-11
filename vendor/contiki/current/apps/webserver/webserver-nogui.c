/*
 * Copyright (c) 2002, Adam Dunkels.
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution. 
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.  
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
 *
 * This file is part of the Contiki OS.
 *
 * $Id: webserver-nogui.c,v 1.5 2008/02/08 22:53:32 oliverschmidt Exp $
 *
 */

#include <string.h>
#include <stdio.h>

#include "contiki.h"
#include "sys/log.h"

#include "http-strings.h"
#include "webserver-nogui.h"
#include "httpd.h"

PROCESS(webserver_nogui_process, "Web server");

AUTOSTART_PROCESSES(&webserver_nogui_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(webserver_nogui_process, ev, data)
{
  PROCESS_BEGIN();

  httpd_init();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    httpd_appcall(data);
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
webserver_log_file(uip_ipaddr_t *requester, char *file)
{
#if LOG_CONF_ENABLED
  char buf[18];

  /* Print out IP address of requesting host. */
  sprintf(buf, "%d.%d.%d.%d: ", requester->u8[0], requester->u8[1],
				requester->u8[2], requester->u8[3]);
  log_message(buf, file);
#endif /* LOG_CONF_ENABLED */
}
/*---------------------------------------------------------------------------*/
void
webserver_log(char *msg)
{
  log_message(msg, "");
}
/*---------------------------------------------------------------------------*/
