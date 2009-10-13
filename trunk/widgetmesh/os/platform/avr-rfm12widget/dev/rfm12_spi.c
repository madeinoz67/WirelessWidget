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

 \file rfm12_spi.c
 \ingroup

 \brief
 */
/**************************************************************************/
#include "rfm12_spi.h"

/**************************************************************************/
/*!
    \brief      This function initialiazes the SPI.

                SPI speed is limited to avoid exceeding the RFM12 SPI
                specs of 2.5Mhz
*/
/**************************************************************************/
void
rfm12_spi_init()
{
      // configure the SPI slave_select, spi clk, and mosi pins as output. the miso pin
      // is cleared since its an input.
        DDRB |= _BV(RFM12_SSPIN) | _BV(RFM12_SCK) | _BV(RFM12_MOSI);
        PORTB |= _BV(RFM12_SSPIN) | _BV(RFM12_SCK);

        #if F_CPU <= 10000000
        // clk/4 is ok for the RF12's SPI
                SPCR = _BV(SPE) | _BV(MSTR);
        #else
                // use clk/8 (2x 1/16th) to avoid exceeding RF12's SPI specs of 2.5 MHz
                SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
                SPSR |= _BV(SPI2X);
        #endif

        RFM12_SPI_DISABLE();
}

/**************************************************************************/
/*!
    \brief      This function both reads and writes a word of data data.
                For write operations, include data to be written as argument.
                For read ops, use dummy data as arg. Returned data is read byte val.

                The SS pin is automatically handled by this function whenever data
                is transfered to the RFM12
    \param data data to be transfered to the RFM12
    \return     result of spi command send back from RFM12
*/
/**************************************************************************/
uint16_t
rfm12_spi_xfer(uint16_t data)
{
    uint16_t reply;

    RFM12_SPI_ENABLE();
    SPDR = data >> 8;
    while (!(SPSR & _BV(SPIF)))
        ;
    reply = SPDR << 8;
    SPDR = data;
    while (!(SPSR & _BV(SPIF)))
        ;
    reply |= SPDR;
    RFM12_SPI_DISABLE();
    return reply;
}

/**************************************************************************/
/*!
    \brief      This function both reads and writes a byte of data to the SPI.
                For write operations, include data to be written as argument.
                For read ops, use dummy data as arg. Returned
                data is read byte val.

    \param data byte to be transfered to the RFM12

    \return     result from RFM12
*/
/**************************************************************************/
uint8_t
rfm12_spi_xfer_byte(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

/**************************************************************************/
/*!
    \brief      This function write the CMD 0x0 to the SPI which retrieves
                the byte of the RFM12.

    \return     Returns status byte
*/
/**************************************************************************/
uint8_t
rfm12_spi_cmd0(void)
{

    uint8_t reply;

    SPDR = 0x00;
    while (!(SPSR & _BV(SPIF)))
        ;
    reply = SPDR;

    return reply;
}
