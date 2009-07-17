/*****************************************************************************\
*              efs - General purpose Embedded Filesystem library              *
*          --------------------- -----------------------------------          *
*                                                                             *
* Filename : sam7x_spi.c                                                   *
* Description : This file contains the functions needed to use efs for        *
*               accessing files on an SD-card connected to an ATMega128.      *
*                                                                             *
* This program is free software; you can redistribute it and/or               *
* modify it under the terms of the GNU General Public License                 *
* as published by the Free Software Foundation; version 2                     *
* of the License.                                                             *
                                                                              *
* This program is distributed in the hope that it will be useful,             *
* but WITHOUT ANY WARRANTY; without even the implied warranty of              *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
* GNU General Public License for more details.                                *
*                                                                             *
* As a special exception, if other files instantiate templates or             *
* use macros or inline functions from this file, or you compile this          *
* file and link it with other works to produce a work based on this file,     *
* this file does not by itself cause the resulting work to be covered         *
* by the GNU General Public License. However the source code for this         *
* file must still be made available in accordance with section (3) of         *
* the GNU General Public License.                                             *
*                                                                             *
* This exception does not invalidate any other reasons why a work based       *
* on this file might be covered by the GNU General Public License.            *
*                                                                             *
*                                                    (c)2006 Lennart Yseboodt *
*                                                    (c)2006 Michael De Nil   *
\*****************************************************************************/

/*****************************************************************************/
#include "../inc/interfaces/sam7x_spi.h"
#include "../inc/interfaces/sd.h"
#include "config.h"
/*****************************************************************************/

esint8 if_initInterface(hwInterface* file, eint8* opts)
{
	if_spiInit(file);
	if(sd_Init(file)<0)	{
		DBG((TXT("Card failed to init, breaking up...\n")));
		return(-1);
	}
	if(sd_State(file)<0){
		DBG((TXT("Card didn't return the ready state, breaking up...\n")));
		return(-2);
	}
	file->sectorCount=4; /* FIXME ASAP!! */
	DBG((TXT("Init done...\n")));

    /* Set baud rate to a much faster value now */
    SPI_CSR0 = (SPI_CPOL | SPI_8_BITS | (((MASTER_CLOCK + 12000000/2)/12000000)<<8));

	return(0);
}
/*****************************************************************************/

esint8 if_readBuf(hwInterface* file,euint32 address,euint8* buf)
{
	return(sd_readSector(file,address,buf,512));
}
/*****************************************************************************/

esint8 if_writeBuf(hwInterface* file,euint32 address,euint8* buf)
{
	return(sd_writeSector(file,address, buf));
}
/*****************************************************************************/

esint8 if_setPos(hwInterface* file,euint32 address)
{
	return(0);
}
/*****************************************************************************/

void if_spiInit(hwInterface *iface)
{
	euint8 i;

	/* Unselect card */
	PIO_SODR_A = (SPI_CS1_0);

	/* Set as master, clock and chip select output */
	/* Configure hardware */
	POWER_UP(PIOA);
	PIO_ASR_A = (SPI_MOSI_0 | SPI_MISO_0 | SPI_CLK_0);
	PIO_PDR_A = (SPI_MOSI_0 | SPI_MISO_0 | SPI_CLK_0);
    PIO_PER_A = (SPI_CS1_0);
    PIO_SODR_A = (SPI_CS1_0);
    PIO_OER_A = (SPI_CS1_0);

    /* Power up */
    POWER_UP(SPI0 | PIOA);                // enable clocks to SPI0 in PMC and Port A due to CS line

    /* Setup registers */
    SPI_MR_0 = (SPI_MSTR | MODFDIS | SPI_CS_0);                            // master mode with no clock divider and control defined by CS0 configuration {27}

    /* Set baud rate */
    SPI_CSR0 = (SPI_CPOL | SPI_8_BITS | (((MASTER_CLOCK + 100000/2)/100000)<<8));

    /* Bring it online */
    SPI_CR_0 = SPIEN;

    euint8 incoming;

    /* Dummy read */
    incoming = SPI_RDR_0;

	/* Send 10 spi commands with card not selected */
	for(i=0;i<10;i++)
		if_spiSend(iface,0xff);

	/* Select card */
	PIO_CODR_A = (SPI_CS1_0);
}
/*****************************************************************************/

euint8 if_spiSend(hwInterface *iface, euint8 outgoing)
{
	euint8 incoming=0;

	PIO_CODR_A = (SPI_CS1_0);
	 SPI_TDR_0 = outgoing;
	 while ((SPI_SR_0 & SPI_RDRF) == 0);
	incoming = SPI_RDR_0;
	PIO_SODR_A = (SPI_CS1_0);

	return(incoming);
}
/*****************************************************************************/

