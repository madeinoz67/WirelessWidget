/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.
    4. This software is subject to the additional restrictions placed on the
       Zigbee Specification's Terms of Use.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
/*!
    \file buf.h
    \ingroup misc
    \brief Frame buffer header file

    This is the header file for buf.c.
*/
/**************************************************************************/
#ifndef BUF_H
#define BUF_H

#include <stdio.h>
#include "types.h"
#include "constants.h"

/*! Generic buffer allocate definition*/
#define BUF_ALLOC(name, txrx)                                   \
            do                                                  \
            {                                                   \
                if ((name = buf_get(txrx)) == NULL)             \
                {                                               \
                    printf("No Free Buffers...Hang...\n");   \
                    while (1);                                  \
                }                                               \
            } while (0);                                        \


/**************************************************************************/
/*!
    This is the frame buffer data structure that is used for TX and RX.
*/
/**************************************************************************/
typedef struct _buffer_t
{
    struct      _buffer_t *next;                ///< Next pointer
    bool        alloc;                          ///< Alloc flag
    uint8_t     *dptr;                          ///< Data pointer - points to current position in buffer array
    uint8_t     len;                            ///< Len of the data
    uint8_t     index;                          ///< Index used for buffer tracking and debugging
    uint8_t     buf[aMaxPHYPacketSize+1];       ///< Main data storage
} buffer_t;

void   buf_init();
buffer_t *buf_get(uint8_t tx_rx);
void buf_free(buffer_t *buf);
uint8_t buf_get_cnt();
#endif // BUF_H


