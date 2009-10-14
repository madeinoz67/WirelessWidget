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
 Created on: 14/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file rfm12_buf.c
 \ingroup

 \brief
 */
/**************************************************************************/

#include "rfm12_buf.h"
#include "rfm12_frame.h"

void
rfm12_buf_rx_init(buffer_t *b, uint8_t *dataptr)
{
  b->data = dataptr;
  b->put_ptr = 0;
  b->get_ptr = 0;
}

void
rfm12_buf_tx_init(buffer_t *b, uint8_t *dataptr)
{
  b->data = dataptr;
  b->put_ptr = PREAMBLE_SIZE + SYNCWORD_SIZE;       // we add new data after the preamble and sync bytes
  b->get_ptr = 0;                               // we read from the beginning
}

/*---------------------------------------------------------------------------*/
int
rfm12_buf_put(buffer_t *b, uint8_t c)
{
  /* Check if buffer is full. If it is full, return 0 to indicate that
     the element was not inserted into the buffer.
  */
  if(b->put_ptr > (sizeof(b->data)-1) ) {
    return 0;
  }
  b->data[b->put_ptr++] = c;
  return 1;
}

/*---------------------------------------------------------------------------*/
int
rfm12_buff_get(buffer_t *b)
{
  uint8_t c;

  /* Check if there are bytes in the buffer. If so, we return the
     first one and increase the pointer. If there are no bytes left, we
     return -1.
  */
  if (b->get_ptr > (sizeof(b->data)-1) ) {
    c = b->data[b->get_ptr++];
    return c;
  } else {
    return -1;
  }
}
/*---------------------------------------------------------------------------*/
int
rfm12_buf_size(buffer_t *b)
{
  return (sizeof(b->data));
}
/*---------------------------------------------------------------------------*/
int
rfm12_buf_elements(buffer_t *b)
{
  return (b->put_ptr - 1);
}
/*---------------------------------------------------------------------------*/
int
rfm12_buf_rewind(buffer_t *b)
{
  b->get_ptr = 0;
}
/*---------------------------------------------------------------------------*/
int
rfm12_buf_rx_clear(buffer_t *b)
{
  b->put_ptr = 0;
  b->get_ptr = 0;
}

/*---------------------------------------------------------------------------*/
int
rfm12_buf_tx_clear(buffer_t *b)
{
  b->put_ptr = PREAMBLE_SIZE + SYNCWORD_SIZE;       // we add new data after the preamble and sync bytes
  b->get_ptr = 0;
}
