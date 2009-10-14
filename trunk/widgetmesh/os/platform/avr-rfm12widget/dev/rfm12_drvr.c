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

 \file          rfm12_drvr.c
 \ingroup

 \brief         Packet driver for the RFM12

 */
/**************************************************************************/

#include "rfm12_drvr.h"
#include "rfm12_frame.h"
#include "radio.h"
#include "contiki.h"

#include <string.h>

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#endif



/**
 *
 *  \brief  This enumeration defines the valid states for RFM12 radio transceiver.
 *
 *   register | eb or et | en  | ex |
 *   Active   |     X    |  1  |  1 |
 *   Idle     |     X    |  0  |  1 |
 *   Sleep    |     1    |  0  |  0 |
 *   Standby  |     0    |  0  |  0 |
 *
 */
enum rfm12_internal_state{
        RFM12_IDLE,
        RFM12_SLEEP,
        RFM12_WAKEUP,
        RFM12_RX_CHECK_PL,
        RFM12_RX_DATA_BYTE,
        RFM12_RX_CRC1,
        RFM12_RX_CRC2,
        RFM12_RX_END,
        RFM12_TX_CHECK_CHANNEL,
        RFM12_TX_ERROR,
        RFM12_TX_SEND_PREAMBLE,
        RFM12_TX_SEND_SYNC0,
        RFM12_TX_SEND_SYNC1,
        RFM12_TX_SEND_HEADER1,
        RFM12_TX_SEND_HEADER2,
        RFM12_TX_SEND_DATA,
        RFM12_TX_CALC_CRC1,
        RFM12_Tx_CALC_CRC2,
        RFM12_TX_SEND_EDC,
        RFM12_TX_END
};
/**
 * RFM12 states
 */
static volatile enum radio_state radio_state = RADIO_SLEEP;
static volatile enum rfm12_internal_state rfm12_internal_state = RFM12_IDLE;

// define some packet sizes
#define MIN_TX_PACKET_SIZE      PREAMBLE_SIZE + SYNCWORD_SIZE + HDR_SIZE + TAIL_SIZE
#define MAX_TX_PACKET_SIZE      PREAMBLE_SIZE + SYNCWORD_SIZE + HDR_SIZE + RFM12_MAX_PAYLOAD + TAIL_SIZE
#define MAX_RX_PACKET_SIZE      HDR_SIZE + RFM12_MAX_PAYLOAD

// Buffers
static volatile uint8_t rfm12_rxbuf[MAX_RX_PACKET_SIZE];
static volatile uint8_t rfm12_txbuf[MAX_TX_PACKET_SIZE];

/* number of bytes in receive and transmit buffers respectively. */
static uint8_t rfm12_rxlen = 0;
static uint8_t rfm12_txlen = 0;

/* CRC */
static uint16_t rxcrc, rxcrctmp;

static void (*receiver_callback)(const struct radio_driver *);  //< callback to upper layer on packet reception

const struct radio_driver rfm12_driver =
  {
    rfm12_drvr_send,
    rfm12_drvr_read,
    rfm12_drvr_set_receiver,
    rfm12_drvr_wakeup,
    rfm12_drvr_sleep
  };

/**
 * \brief valid RFM12 channel frequencies are set in this array.
 *
 *         The number of channels and frequency values can be set to anything but will depend on the datarate
 *         due to the Rx bandwidth setting.
 *
 *         These initial values have been set to 802.15.4 900Mhz defaults:
 *         channels 1-10 frequencies
 *
 *         These channels should be redefined to suit the regulatory requirements of your country.
 *
 */
const uint16_t rfm12_channel[RFM12_MAX_CHANNELS] =
{
    0xA320,     // 906.00
    0xA42B,     // 908.00
    0xA536,     // 910.00
    0xA640,     // 912.00
    0xA74B,     // 914.00
    0xA856,     // 916.00
    0xA960,     // 918.00
    0xAA6B,     // 920.00
    0xAB76,     // 922.00
    0xAC80      // 924.00
};

/**
 *  band and datarate optimised configurations defined here
 */
#ifdef RFM12_CONFIG_915_57600
const uint16_t rfm12_config[RFM12_MAX_CONFIG] =
{
                0x80F8,                         //< 915mhz EL,EF,12.0pF
                0x8209,                         //< enable crystal, disable CLK
                0xC606,                         //< 57600bps
                0x9460,                         //< VDI,FAST,270kHz,0dBm,-103dBm
                0xC2AC,                         //< AL,!ml,DIG,DQD4
                0xCA83,                         //< FIFO8,SYNC,!ff,DR
                0xCED4,                         //< 2nd Sync Byte D4
                0xC483,                         //< @PWR,NO RSTRIC,!st,!fi,OE,EN
                0x9880,                         //< mp,135kHz,MAX OUT
                0xCC77,                         //< OB1 OB0, LPX, ddy, DDIT, BW0
                0xE000,                         //< wakeup off
                0xC800,                         //< duty cycle - not used
                0xC040                          //< 1.66MHz, 2.2V
};

#elif RFM12_CONFIG_915_115200
const uint16_t rfm12_config[RFM12_CONFIG_COUNT] =
{
                0x80F8,                         //< 915mhz EL,EF,12.0pF
                0x8209,                         //< enable crystal, disable CLK
                0xC602,                         //< 115200bps
                0x9440,                         //< VDI,FAST,340kHz,0dBm,-103dBm
                0xC2AC,                         //< AL,!ml,DIG,DQD4
                0xCA83,                         //< FIFO8,SYNC,!ff,DR
                0xCED4,                         //< 2nd Sync Byte D4
                0xC483,                         //< @PWR,NO RSTRIC,!st,!fi,OE,EN
                0x98B0,                         //< mp,180kHz,MAX OUT
                0xCC77,                         //< OB1 OB0, LPX, ddy, DDIT, BW0
                0xE000,                         //< wakeup off
                0xC800,                         //< duty cycle - not used
                0xC040                          //< 1.66MHz, 2.2V
};

#else
#error "RFM12_CONFIG_XXX not defined"
#endif

/*---------------------------------------------------------------------------*/
PROCESS(rfm12_driver_process, "RFM12 driver");
PT_THREAD(rfm12_rx_handler_pt(unsigned char c));
PT_THREAD(rfm12_tx_handler_pt(void));
static struct pt tx_handler_pt;
static struct pt rx_handler_pt;
static struct timer trxtimer;                  //< interbyte transmission-reception timer
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


    timer_set(&trxtimer, CLOCK_SECOND / 4);

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
 * @param power                         power setting to change to
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
 *              XTAL OSC is turned on.
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
 * \return RADIO_WRONG_STATE            is not in the idle state
 * \return RADIO_INVALID_ARGUMENT       nothing to transmit
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
        if ((rfm12_txlen > MAX_TX_PACKET_SIZE ) || (rfm12_txlen < MIN_TX_PACKET_SIZE))
            return RADIO_INVALID_ARGUMENT;

        //TODO: reset transmit buffer read pointer

        //TODO: start transmit sequence
        // listen before talk enabled?
            // spi_xfer(0x0000);
            // spi_xfer(rfm12_config[1] | 0x0080 );  // turn on receiver
            // radio_state = RADIO_TX_CHAN_CHK;
            // rfm12_internal_state = RFM12_TX_CHECK_CHANNEL;
            // csma_counter =
            // set timer for first DQD check
         // else

            // load the first two bytes into RFM12 Tx buffer
            //rfm12_spi_xfer(RFM12_CMD_TX | TODO: );
            //rfm12_spi_xfer(RFM12_CMD_TX | TODO: );

            RFM12_SPI_ENABLE();
            rfm12_spi_cmd0();                           // clear onboard RFM12 interrupts
            RFM12_SPI_DISABLE();

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

/**
 *      /brief        usec timer using asm("nop") so is hardware dependant
 *
 *                    This function is blocking
 *
 *                    Each nop takes 2 cycles so this should give us an 10 cycle delay or 1 usec @ 10mhz.
 *
 *                      4 nop @ 8Mhz
 *                      5 nop @ 10Mhz
 *
 * TODO: implement using timer
 * @param usec
 */
static void delay_us(uint16_t usec)
{
    do
    {
        asm("nop");
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
 * \brief       loads the config in RFM12 using an optimised configuration
 *              for the band and datarate
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

/**************************************************************************/
/**
 * \brief Task that handles the incoming data from the receiver
 *        this task checks header address information.
 *
 *        Checks:
 *        DRSSI valid before receive first byte
 *        Dest address not for this address then the packet is dropped
 *        CRC is checked if incorrect then the packet is dropped.
 *
 *
 * \param incoming_byte from the interrupt handler
 * \return
 */
PT_THREAD(rfm12_rx_handler_pt(unsigned char incoming_byte))
{
  static unsigned char rxtmp, tmppos;

  if(timer_expired(&trxtimer)) {
    PT_INIT(&rx_handler_pt);   // restart the receive as we have not received data within given time
  }

  timer_restart(&trxtimer);

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


PT_THREAD(rfm12_tx_handler_pt(void)){

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
