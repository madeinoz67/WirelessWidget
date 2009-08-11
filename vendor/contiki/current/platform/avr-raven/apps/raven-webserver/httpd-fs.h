/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
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
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: httpd-fs.h,v 1.2 2009/06/19 17:11:28 dak664 Exp $
 */
#ifndef __HTTPD_FS_H__
#define __HTTPD_FS_H__

#include "contiki-net.h"

//#define HTTPD_FS_STATISTICS 1   //Puts count in file system
#define HTTPD_FS_STATISTICS 2     //Puts count in RAM array

#if HTTPD_FS_STATISTICS==2
extern u16_t httpd_filecount[];
#endif /* HTTPD_FS_STATISTICS */

#include <avr/pgmspace.h>

struct httpd_fs_file {
  char *data;
  int len;
};

/* file must be allocated by caller and will be filled in
   by the function. If NULL, just file stats are returned.*/
u16_t httpd_fs_open(const char *name, struct httpd_fs_file *file);

/* Returns root of http pages in flash */
void * httpd_get_root();

void httpd_fs_init(void);

#define httpd_fs_cpy                    memcpy_P
#define httpd_fs_strchr                 strchr_P
#define httpd_fs_getchar(x)     pgm_read_byte(x)

#endif /* __HTTPD_FS_H__ */
