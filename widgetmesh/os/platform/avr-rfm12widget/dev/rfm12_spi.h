#ifndef RFM12_SPI_H_
#define RFM12_SPI_H_
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
 Created on: 12/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file rfm12_spi.h
 \ingroup

 \brief
 */
/**************************************************************************/

#include <avr/io.h>

#define RFM12_SPI_ENABLE()    do {RFM12_SPIPORT &= ~(_BV(RFM12_SSPIN));} while (0)
#define RFM12_SPI_DISABLE()   do {RFM12_SPIPORT |= _BV(RFM12_SSPIN);} while (0)

#define RFM12_SPIPORT     PORTB
#define RFM12_SSPIN       2                 // PB.2 - Output: Select Pin
#define RFM12_MOSI        3                 // PB.3 - Output: SPI Master out - slave in (MOSI)
#define RFM12_MISO        4                 // PB.4 - Input:  SPI Master in - slave out (MISO)
#define RFM12_SCK         5                 // PB.5 - Output: SPI Serial Clock (SCLK)


void rfm12_spi_init(void);
uint16_t rfm12_spi_xfer(uint16_t cmd);
uint8_t  rfm12_spi_xfer_byte(uint8_t data);
uint8_t  rfm12_spi_cmd0(void);

#endif /* RFM12_SPI_H_ */
