#ifndef RFM12_BUF_H_
#define RFM12_BUF_H_
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

 \file rfm12_buf.h
 \ingroup

 \brief
 */
/**************************************************************************/
#include <stdint.h>
/**
 * \brief      Structure that holds the state of a buffer.
 *
 *             This structure holds the state of a buffer. The
 *             actual buffer needs to be defined separately. This
 *             struct is an opaque structure with no user-visible
 *             elements.
 *
 */
typedef struct _buffer_t {
  uint8_t *data;
  uint8_t put_ptr;
  uint8_t get_ptr;
} buffer_t;

/**
 * \brief      Initialize a Receive buffer
 * \param b    A pointer to a struct buf to hold the state of the buffer
 * \param a    A pointer to an array to hold the data in the buffer
 *
 *             This function initiates a buffer. The data in the
 *             buffer is stored in an external array, to which a
 *             pointer must be supplied.
 *
 */
void rfm12_buf_rx_init(buffer_t *b, uint8_t *a);

/**
 * \brief      Initialize a Transmit buffer
 * \param b    A pointer to a struct buf to hold the state of the buffer
 * \param a    A pointer to an array to hold the data in the buffer
 *
 *             This function initiates a buffer that is pre-filled with preamble
 *             and sync bytes.
 *
 *             The data in the buffer is stored in an external array, to which a
 *             pointer must be supplied.
 *
 */
void
rfm12_buf_tx_init(buffer_t *b, uint8_t *a);

int
rfm12_buf_put(buffer_t *b, uint8_t data);

/**
 * \brief
 */
int
rfm12_buf_get(buffer_t *b);

/**
 * \brief     rewinds the buffer so it can be re-read.
 * \param b   A Pointer to a struct buf to hold the state of the buffer
 */
int
rfm12_buf_rewind(buffer_t *b);

/**
 * \brief     clears the Tx buffer
 * \param b   A Pointer to a struct buf to hold the state of the buffer
 */
int
rfm12_buf_tx_clear(buffer_t *b);

/**
 * \brief     clears the Rx buffer
 * \param b   A Pointer to a struct buf to hold the state of the buffer
 */
int
rfm12_buf_rx_clear(buffer_t *b);


/**
 * \brief     gets the size of the buffer
 * \param b   A Pointer to a struct buf to hold the state of the buffer
 */
int
rfm12_buf_size(buffer_t *b);

/**
 * \brief     gets number of elements in currently in the buffer
 * \param b   A Pointer to a struct buf to hold the state of the buffer
 */
int
rfm12_buf_elements(buffer_t *b);

#endif /* RFM12_BUF_H_ */
