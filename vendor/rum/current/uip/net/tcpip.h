/**
 * \addtogroup uip
 * @{
 */

/**
 * \defgroup tcpip The Contiki/uIP interface
 * @{
 *
 * TCP/IP support in Contiki is implemented using the uIP TCP/IP
 * stack. For sending and receiving data, Contiki uses the functions
 * provided by the uIP module, but Contiki adds a set of functions for
 * connection management. The connection management functions make
 * sure that the uIP TCP/IP connections are connected to the correct
 * process.
 *
 * Contiki also includes an optional protosocket library that provides
 * an API similar to the BSD socket API.
 *
 * \sa \ref uip "The uIP TCP/IP stack"
 * \sa \ref psock "Protosockets library"
 *
 */

/**
 * \file
 *          Header for the Contiki/uIP interface.
 * \author  Adam Dunkels <adam@sics.se>
 * \author  Mathilde Durvy <mdurvy@cisco.com> (IPv6 related code)
 * \author  Julien Abeille <jabeille@cisco.com> (IPv6 related code)
 */

/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: tcpip.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
 */
#ifndef __TCPIP_H__
#define __TCPIP_H__

#include "../contiki.h"

struct uip_conn;

struct tcpip_uipstate {
  struct process *p;
  void *state;
};

#define UIP_APPCALL       tuip_appcall
#define UIP_UDP_APPCALL   tuip_udp_appcall
#define UIP_ICMP6_APPCALL tcpip_icmp6_call

/*#define UIP_APPSTATE_SIZE sizeof(struct tcpip_uipstate)*/

typedef struct tcpip_uipstate uip_udp_appstate_t;
typedef struct tcpip_uipstate uip_tcp_appstate_t;
typedef struct tcpip_uipstate uip_icmp6_appstate_t;


#endif /* __TCPIP_H__ */

/** @} */
/** @} */
