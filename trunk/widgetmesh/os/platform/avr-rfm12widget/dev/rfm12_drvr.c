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

 \file rfm12_drvr.c
 \ingroup

 \brief
 */
/**************************************************************************/

#include "rfm12_drvr.h"
#include "radio.h"
#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

/*
 * This timer is used to keep track of when the last byte was received / sent
 * over the radio. If the inter-byte time is too large, the packet
 * currently being received is discarded and a new packet reception is
 * initiated.
 */
static struct timer rxtimer;

/**
 * RFM12 states
 */
static volatile enum radio_state radio_state = RADIO_SLEEP;
static volatile enum rfm12_internal_state rfm12_internal_state = RFM12_IDLE;

// Buffers
static volatile uint8_t rfm12_rxbuf[HDR_SIZE + RFM12_BUFFERSIZE];
static volatile uint8_t rfm12_txbuf[PREAMBLE_SIZE + SYNCWORD_SIZE + HDR_SIZE + RFM12_BUFFERSIZE + TAIL_SIZE];

/* number of bytes in receive and transmit buffers respectively. */
static uint8_t rfm12_rxlen = 0;
static uint8_t rfm12_txlen = 0;

/* CRC */
static uint16_t rxcrc, rxcrctmp;

static void (*receiver_callback)(const struct radio_driver *);

const struct radio_driver rfm12_driver =
  {
    rfm12_drvr_send,
    rfm12_drvr_read,
    rfm12_drvr_set_receiver,
    rfm12_drvr_wakeup,
    rfm12_drvr_sleep
  };


/*---------------------------------------------------------------------------*/

PT_THREAD(rfm12_rx_handler_pt(unsigned char c));
static struct pt rx_handler_pt;
/*---------------------------------------------------------------------------*/
static void load_config(const uint16_t *config);
static void dump_packet(int len);
static void delay_us(uint16_t delay);

/**
 * \brief               this function initializes the driver
 *
 * \param config        needs to be defined by setting the RFM12_BAND and RFM12_DATARATE
 */
void
rfm12_drvr_init(const uint16_t* config)
{


    PT_INIT(&rx_handler_pt);
    delay_us(TIME_PWR_ON);

    //NIRQ pullup resistor
    RFM12_IRQ_PORT |= _BV(RFM12_IRQ_PIN);

    rfm12_spi_init();
    load_config(config);

    /* pre-fill tx buffer with preamble + syncword */
    memset(rfm12_txbuf, PREAMBLE, PREAMBLE_SIZE);
    memcpy((char *)rfm12_txbuf + PREAMBLE_SIZE, &syncword, SYNCWORD_SIZE);

    process_start(&rfm12_driver_process, NULL);

    //enable interrupt
    RFM12_ENABLE_RADIO_INTERRUPT();


    timer_set(&rxtimer, CLOCK_SECOND / 4);

}

/**
 * \brief       changes the channel of the RFM12.
 *
 *              The rfm12 does not have a concept of channels but has the ability to change to a frequency
 *
 *              channels are configured in the rfm12_channel LUT.
 *
 *              This example is basedon the 802.15.4 900MHZ frequency allocation for channels 1-10
 *
 *              E.G. const uint16_t rfm12_channel[10] =
 *                     {
 *                      0xA320,     // 906.00
 *                      0xA42B,     // 908.00
 *                      0xA536,     // 910.00
 *                      0xA640,     // 912.00
 *                      0xA74B,     // 914.00
 *                      0xA856,     // 916.00
 *                      0xA960,     // 918.00
 *                      0xAA6B,     // 920.00
 *                      0xAB76,     // 922.00
 *                      0xAC80      // 924.00
 *                     };
 *
 *
 * \param channel               number of channel in LUT to change to.
 *
 * \return RADIO_SUCCESS        channel changed successfully
 * \return RADIO_INVALID_ARGUMENT incorrect channel number
 * \return RADIO_WRONG_STATE    RFM12 must be in idle state to change the channel
 *
 */
int
rfm12_drvr_set_channel(uint8_t channel)
{
        if(radio_state!= RADIO_IDLE)
          return RADIO_WRONG_STATE;

        if((channel < 0)||(channel > RFM12_MAX_CHANNELS-1))
          return RADIO_INVALID_ARGUMENT;

        rfm12_spi_xfer(rfm12_channel[channel]);            // change the channel
        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);                  // reset fifo to initiate PLL re-lock

        delay_us(TIME_PLL_LOCK);                     // wait for PLL to lock after freq change

        return RADIO_SUCCESS;
}

/**
 * \brief       sets the transmit power level
 *
 *              valid values are:
 *               RFM12_TXCONF_POWER_0  = 0x0  0db  Strongest Tx
 *               RFM12_TXCONF_POWER_3  = 0x1 -3db
 *               RFM12_TXCONF_POWER_6  = 0x2 -6db
 *               RFM12_TXCONF_POWER_9  = 0x3 -9dB
 *               RFM12_TXCONF_POWER_12 = 0x4 -12dB
 *               RFM12_TXCONF_POWER_15 = 0x5 -15dB
 *               RFM12_TXCONF_POWER_21 = 0x6 -21dB Weakest
 *
 * @param power                         new power setting to change to
 *
 * @return RADIO_SUCCESS                Tx power changed successfully
 * @return RADIO_WRONG_STATE            must be in idle state to change the power
 * @return RADIO_INVALID_ARGUMENT       incorrect power setting requested
 */
int
rfm12_drvr_set_power(uint8_t power)
{
    if (radio_state != RADIO_IDLE)
      return RADIO_WRONG_STATE;

    if ((power < RFM12_TXCONF_POWER_0) || (power > RFM12_TXCONF_POWER_17_5))
      return RADIO_INVALID_ARGUMENT;

    rfm12_spi_xfer(rfm12_config[8] & (0xfff0 | power));       // set TX power
    rfm12_spi_xfer(RFM12_CMD_FIFO_RST);                         // reset fifo to initiate PLL re-lock

    delay_us(TIME_PLL_LOCK);                            // wait for PLL to lock after freq change

    return RADIO_SUCCESS;
}

/**
 * \brief       puts the RFM12 into sleep mode
 *              note this does not power down the RFM12 but turns off the XTAL OSC to save power.
 *
 * \return RADIO_SUCCESS        RFM12 successfully put into sleep mode
 * \return RADIO_WRONG_STATE    RFM12 must be in the idle state first
 */
int
rfm12_drvr_sleep(void)
{
        if(radio_state!= RADIO_IDLE)
                return RADIO_WRONG_STATE;

        /* Discard the current read buffer when the radio is shutting down. */
        rfm12_rxlen = 0;

        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);               //  Reset the Fifo
        rfm12_spi_xfer(RFM12_CMD_SLEEP);                //  XTAL off, CLK off

        radio_state = RADIO_SLEEP;
        rfm12_internal_state = RFM12_SLEEP;

        return RADIO_SUCCESS;
}

/**
 *  \brief      wakes up the RFM12.
 *              OSC is turned on.
 *
 *  \return RADIO_SUCCESS       RFM12 successfully woken up
 *  \return RADIO_WRONG_STATE   not in sleep mode
 */
int
rfm12_drvr_wakeup(void)
{
        if(radio_state != RADIO_SLEEP)
          return RADIO_WRONG_STATE;

        //turn on RFM12 XTAL OSC
        rfm12_spi_xfer(RFM12_CMD_WKUP);

        radio_state = RADIO_WAKEUP;
        rfm12_internal_state = RFM12_WAKEUP;

        delay_us(TIME_SLEEP_TO_TRX_OFF);       // wait for device to wakeup

        radio_state = RADIO_IDLE;
        rfm12_internal_state = RFM12_IDLE;

        return RADIO_SUCCESS;
}

/**
 * \brief       puts the RFM12 into idle state
 *              the RFM12 must be in this state before it can:
 *
 *              transmit, receive or sleep
 *
 * \return RADIO_SUCCESS        state change successful
 * \return RADIO_WKUP_ERROR     sleep mode or unfinished wakeup
 * \return RADIO_TRX_BUSY       RFM12 is busy either transmitting or receiving
 */
int
rfm12_drvr_idle()
{
        if(radio_state == RADIO_IDLE)
                return RADIO_SUCCESS;         // already idle

        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        // receiving or transmission is in progress?
        if ((radio_state == RADIO_RX_RECEIVING) || (radio_state == RADIO_TX_TRANSMITTING))
            return RADIO_TRX_BUSY;
;
        //reset FIFO sync latch
        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);

        //turn on RFM12 XTAL OSC
        rfm12_spi_xfer(RFM12_CMD_IDLE);

        radio_state = RADIO_IDLE;
        rfm12_internal_state = RFM12_IDLE;

        return RADIO_SUCCESS;
}

/**
 * \brief       Starts transmitting the contents of the tx_buffer
 *
 * \return RADIO_WKUP_ERROR             still in sleep state or an unfinished wakeup
 * \return RADIO_WRONG_STATE            is not in the idel state
 * \return RADIO_INVALID_ARGUMENT       nothing to transmit tx buffer length is 0
 * \return RADIO_SUCCESS                transmission started successfully
 */
int
rfm12_drvr_transmit()
{
        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        if ((radio_state!=RADIO_IDLE) && (radio_state!=RADIO_TX_ERROR))
            return RADIO_WRONG_STATE;

        //check if we have something to transmit
        if ((rfm12_txlen > RFM12_BUFFERSIZE ) || (rfm12_txlen < 1 ))
            return RADIO_INVALID_ARGUMENT;

        //TODO: transmit sequence
        // listen before talk enabled?
            // spi_xfer(0x0000);
            // spi_xfer(rfm12_config[1] | 0x0080 );  // turn on receiver
            // radio_state = RADIO_TX_CHAN_CHK;
            // rfm12_internal_state = RFM12_TX_CHECK_CHANNEL;
            // csma_counter =
            // set timer for first DQD check
         // else

            rfm12_spi_xfer(0x0000);
            rfm12_spi_xfer(rfm12_config[1] | 0x0020 );  // turn on transmitter

            radio_state = RADIO_TX_TRANSMITTING;
            rfm12_internal_state = RFM12_TX_SEND_PREAMBLE;
        return RADIO_SUCCESS;
}

/**
 * \brief       turns on the receiver and waits for data to be received
 *
 * \return RADIO_SUCCESS        reception enabled successfully
 * \return RADIO_WKUP_ERROR     sleep mode or unfinished wakeup
 * \return RADIO_WRONG_STATE    must be in the idle state
 */
int
rfm12_drvr_receive()
{
        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        if(radio_state != RADIO_IDLE)
                return RADIO_WRONG_STATE;

        //TODO: receive sequence

        return RADIO_SUCCESS;
}

/**************************************************************************/
/*!
    Each nop takes 2 cycles so this should give us an 8 cycle delay or 1 usec.
    I'm just too lazy to implement the timer interrupt right now.
    Gimme a break, its almost midnight on a friday...

    TODO: replace this with a proper interrupt driven delay
*/
/**************************************************************************/
static void delay_us(uint16_t usec)
{
    do
    {
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    } while (--usec);
}
/*---------------------------------------------------------------------------*/
int
rfm12_drvr_send(const void *buf, unsigned short len)
{
        //TODO: send
        return RADIO_SUCCESS;
}

/*---------------------------------------------------------------------------*/
int
rfm12_drvr_read(const void *buf, unsigned short len)
{
        //TODO: read
        return RADIO_SUCCESS;
}

/*---------------------------------------------------------------------------*/
void
rfm12_drvr_set_receiver(void (*recv)(const struct radio_driver *))
{
  receiver_callback = recv;
}

/**
 * \brief       loads the config tothe RFM12 using an optimised configuration
 *              based on:
 *                 RFM12_BAND_XXX
 *                 RFM12_DATARATE_XXX
 *
 * \param config - rfm12_config
 */
static void
load_config(const uint16_t *config)
{
  int i;
  for(i = 0; i < 0x13; i++)
    rfm12_spi_xfer(config[i]);
}


/**
 * \brief       Prints a received packets contents to the debug console
 * \param len   Length of packet
 */
static void
dump_packet(int len)
{
  int i;
  for(i = 0; i < len; ++i) {
    LOG("%d: 0x%02x\n", i, rfm12_rxbuf[i]);
  }
}
/**
 * \brief Task that handles the incoming data from the receiver
 *        this task checks header address information.
 *
 *        Checks:
 *        Dest address not for this address then the packet is dropped
 *        CRC is checked if incorrect then the packet is dropped.
 *
 * \param incoming_byte from the interrupt handler
 * \return
 */
PT_THREAD(rfm12_rx_handler_pt(unsigned char incoming_byte))
{
  static unsigned char rxtmp, tmppos;

  if(timer_expired(&rxtimer)) {
    PT_INIT(&rx_handler_pt);
  }

  timer_restart(&rxtimer);

  PT_BEGIN(&rx_handler_pt);
  while(1) {

         // reset_receiver();

          /* Reset the CRC. */
          rxcrc = 0xffff;

          if(rxcrctmp == rxcrc) {
          /* A full packet has been received and the CRC checks out. We'll
                 request the driver to take care of the incoming data. */

                  //RADIOSTATS_ADD(rx);
                  process_poll(&rfm12_driver_process);

          /* We'll set the receive state flag to signal that a full frame
                 is present in the buffer, and we'll wait until the buffer has
                 been taken care of. */
               rfm12_internal_state = RFM12_RX_END;

               PT_WAIT_UNTIL(&rx_handler_pt, rfm12_internal_state != RFM12_RX_END);
             } else {
               LOG("Incorrect CRC\n");
               //RADIOSTATS_ADD(badcrc);
             }
  }
  PT_END(&rx_handler_pt);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rfm12_driver_process, ev, data)
{
  PROCESS_BEGIN();

  /* Reset reception state now that the process is ready to receive data. */
  //reset_receiver();

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    if(receiver_callback != NULL) {
      receiver_callback(&rfm12_driver);
    } else {
      LOG("rfm12 has no receive function\n");
      /* Perform a dummy read to drop the message. */
      rfm12_drvr_read(&data, 0);
    }
  }
  PROCESS_END();
}
/*----------------------------------------------------------------------------*/
/* This #if compile switch is used to provide a "standard" function body for the */
/* doxygen documentation. */
#if defined(DOXYGEN)
/** \brief ISR for the radio IRQ line, triggered by the IRQ0.
 *  This is the interrupt service routine for IRQ0.
 *  It is triggered of a falling edge on the radio transceivers NIRQ line.
 */
void RFM12_RADIO_IRQ{void);
#else  /* !DOXYGEN */
ISR(RFM12_RADIO_IRQ)
{
        RFM12_SPI_ENABLE();
        uint8_t source = rfm12_spi_cmd0();      //get interrupt source from RFM12
        RFM12_SPI_DISABLE();

        if(source & RFM12_IRQ_OVUR)             // overflow error
        {

        }

        if(source & RFM12_IRQ_FIFO)             // tx or rx event
        {

        }

        if(source & RFM12_IRQ_LOWBAT)           // battery voltage below threshold
        {

        }


}
#endif /* defined(DOXYGEN) */
