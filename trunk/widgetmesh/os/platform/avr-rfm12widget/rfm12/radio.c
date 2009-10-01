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
 Created on: 01/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file radio.c
 \ingroup rfm12widget

 \brief rfm12 driver implementation for AVR
 */
/**************************************************************************/
#include "contiki-rfm12widget.h"
#include "driver_avr_rfm12.h"
#include "hal.h"
#include "rand.h"


/**************************** Prototypes ***************************/
PROCESS(radio_driver_process, "RFM12 Driver Process");  ///< Main driver process for rfm12 driver
/*******************************************************************/
extern process_event_t event_driver_confirm;	  ///< Driver confirmation event
static radio_dcb_t dcb;                           ///< Driver control block instantiation

/**************************************************************************/
/*!
    Get the driver control block. When called, it will return a pointer
    the driver control block which holds information that is needed by other
    areas of the driver. Plus its nice to have all the flags and status in
    one place.
*/
/**************************************************************************/
radio_dcb_t *radio_dcb_get()
{
    return &dcb;
}
/**************************************************************************/
/*!
    This sets the handle of the frame in the driver control block. It will
    be used later when the confirmation gets sent up the stack to identify
    the frame that the confirmation pertains to.
*/
/**************************************************************************/
void radio_set_handle(uint8_t handle)
{
    dcb.handle = handle;
}

/**************************************************************************/
/*!
    Set the transmission status. This will also be used by the confirmation
    which will be sent up the stack from the MAC layer. The status will be
    included in the confirmation and will signal the higher layers whether the
    transmission was successful or not.
*/
/**************************************************************************/
void radio_set_status(radio_status_t status)
{
    dcb.status = status;
    dcb.status_avail = true;
    process_poll(&driver_process);
}

/**************************************************************************/
/*!
    Signal that data was received. This gets used in the driver process polling
    mechanism. If data is received, it will notify the MAC and begin to process
    the incoming frame.
*/
/**************************************************************************/
void radio_set_data_rx_flag(bool flag)
{
    dcb.data_rx = flag;
    if (flag)
    {
        process_poll(&driver_process);
    }
}

/**************************************************************************/
/*!
    Initialize the random seed. Random values are used a lot in the stack
    and this function will seed the pseudorandom number generator so that
    it won't always start at the same value. The seed will be taken from
    the TCNT0 timer which is free running.
*/
/**************************************************************************/
void radio_set_rand_seed()
{
    srand(TCNT0);
}


/**************************************************************************/
/*!
    Get a random number. This function returns a random 16-bit number
    to the function that calls it.
*/
/**************************************************************************/
uint16_t radio_get_rand()
{
    return (uint16_t)rand();
}

/**************************************************************************/
/*!
    Puts the RFM12 to sleep
      valid times to sleep are
*/
/**************************************************************************/
radio_status_t radio_sleep(uint8_t time)
{
	if ((state == RFM12_TX_BUSY) || (state == RFM12_RX_BUSY))
	    {
	         return RADIO_WRONG_STATE;
	    }

	//TODO: hal_sleep(time)

    return RADIO_SUCCESS;
}

/**************************************************************************/
/*!
    wake up the RFM12
*/
/**************************************************************************/
radio_status_t radio_wakeup()
{
	if (state != RFM12_SLEEPING)
	    {
	         return RADIO_WRONG_STATE;
	    }

	//TODO: hal_wakeup()

    return RADIO_SUCCESS;
}


/**************************************************************************/
/*!
    Sets the channel in the hardware. Changing a channel requires a certain
    amount of delay as the hardware and PLL sync up to the new channel.
*/
/**************************************************************************/
radio_status_t radio_set_channel(uint8_t channel)
{
    uint8_t state = driver_get_trx_state();

    if ((state == RFM12_TX_BUSY) || (state == RFM12_RX_BUSY))
    {
         return RADIO_WRONG_STATE;
    }

    //TODO: hal_set_channel(channel);

    // add a delay to allow the PLL to lock if in active mode.
    if ((state == RFM12_RX) || (state == RFM12_TX))
    {
        delay_us(TIME_PLL_LOCK);
    }

    return (uint8_t)RADIO_SUCCESS;
}

/**************************************************************************/
/*!
    Set the TX power setting in hardware.
    These are the settings and the corresponding TX power levels:
		0x0		 0
		0x1		-3
		0x2		-6
		0x3		-9
		0x4		-12
		0x5		-15
		0x6		-18
		0x7		-21
*/
/**************************************************************************/
radio_status_t radio_set_tx_pwr(uint8_t pwr_level)
{
    uint8_t state = driver_get_trx_state();

    if ((state == RFM12_TX_BUSY) || (state == RFM12_RX_BUSY))
    {
         return RADIO_WRONG_STATE;
    }

	//TODO: hal_spi_write(tx_pwr);

    return RADIO_SUCCESS;
}

/**************************************************************************/
/*!
    Get the Tx/Rx state. It will return the current state of the hal FSM.
*/
/**************************************************************************/
radio_state_t radio_get_trx_state()
{
    return hal_get_state();
}

/**************************************************************************/
/*!
        This is the main function in the driver which transmits the data.
*/
/**************************************************************************/
radio_status_t radio_send_data(const buffer_t *buf)
{
    radio_state_t state = driver_get_trx_state();

    if ((state == RFM12_TX_BUSY) || (state == RFM12_RX_BUSY))
    {
         return RADIO_WRONG_STATE;
    }

	//TODO: hal_tx_frame(tx_frame);

    return RADIO_SUCCESS;
}

/**************************************************************************/
/*!
        Initialize the driver. This function gets called when the Contiki OS is
        initialized.
*/
/**************************************************************************/
void radio_init()
{
    memset(&dcb, 0, sizeof(radio_dcb_t));

    delay_us(TIME_PWR_ON);
    hal_init();

    // set the default channel
    // TODO: define channels and frequencies
    radio_set_channel(11);

    // start the contiki driver process and register the event number
    process_start(&radio_driver_process, NULL);
    event_driver_confirm = process_alloc_event();
}

/**************************************************************************/
/*!
        The Contiki driver process will poll this function constantly. The poll
        function does two things:
        1) Check for received data
        2) Check for the status after a transmission

        The received data flag will trigger the MAC layer to pick up the frame
        from the queue and start to process it.

        If a transmit status is available, then it will generate a confirm that
        gets propagated up the stack.
*/
/**************************************************************************/
static void pollhandler()
{
    uint8_t status;

    if (dcb.data_rx)			// Data Received by driver
    {
        process_post(&mac_process, event_mac_rx, NULL);
        dcb.data_rx = false;
    }
    else if (dcb.status_avail) // driver status changed
    {
        switch (dcb.status)
        {
        case RADIO_SUCCESS:
            status = MAC_SUCCESS;
            break;
        default:
            status = MAC_UNSPECIFIED_FAILURE;
            break;
        }
        process_post(&mac_process, event_driver_confirm, &dcb);
        dcb.status_avail = false;
    }

    process_poll(&driver_process);
}

/**************************************************************************/
/*!
        The main process for the driver. Actually this process doesn't do anything
        other than call the polling function.

        According to the Contiki documentation, an IRQ cannot signal a process event
        directly, so a polling function needs to be used.
*/
/**************************************************************************/
PROCESS_THREAD(driver_process, ev, data)
{
  PROCESS_POLLHANDLER(pollhandler());

  PROCESS_BEGIN();

  // kick off the pollhandler
  process_poll(&driver_process);

  while (1)
  {
      PROCESS_YIELD();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


