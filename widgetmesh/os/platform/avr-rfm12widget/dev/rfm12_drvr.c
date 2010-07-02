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
#include "lib/crc16.h"
#include "contiki.h"
#include <avr/pgmspace.h>

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
        RFM12_SCAN,
        RFM12_RX_READY,
        RFM12_RX_END,
        RFM12_TX_CHECK_CHANNEL,
        RFM12_TX_ERROR,
        RFM12_TX_PREAMBLE,
        RFM12_TX_SYNC,
        RFM12_TX_HEADER,
        RFM12_TX_DATA,
        RFM12_TX_CRC1,
        RFM12_TX_CRC2,
        RFM12_TX_TAIL,
        RFM12_TX_END
};
/**
 * RFM12 states
 */
static volatile enum radio_state radio_state = RADIO_SLEEP;
static volatile enum rfm12_internal_state rfm12_internal_state = RFM12_IDLE;

// define syncword and packet sizes
const uint8_t syncword[SYNCWORD_SIZE] = {0x2D, 0xD4};
#define MIN_PACKET_SIZE      HDR_SIZE + CRC_SIZE
#define MAX_PACKET_SIZE      HDR_SIZE + MAX_PHY_PAYLOAD + CRC_SIZE

// Buffer
static volatile uint8_t rfm12_trxbuf[MAX_PACKET_SIZE];
static volatile uint8_t data_len = 0;   // length of data packet
static volatile uint8_t buf_pos = 0;    // used in IRQ
static volatile uint8_t irq_cnt = 0;    // used in IRQ

/* CRC */
volatile static uint16_t crc_expected, crc_actual;


static void (*receiver_callback)(const struct radio_driver *);  //< callback to upper layer on packet reception

const struct radio_driver rfm12_driver =
  {
    drvr_send,
    drvr_read,
    drvr_set_receiver,
    drvr_wakeup,
    drvr_sleep
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
const uint16_t rfm12_channel[DRVR_MAX_CHANNELS] =
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
                0x80F8,                         //< 915mhz EL,EF,12.5pF
                RFM12_CMD_SLEEP,                //< low power mode with CLK disabled
                0xC606,                         //< 57600bps
                0x9460,                         //< VDI,FAST,270kHz,0dBm,-103dBm
                0xC2AC,                         //< AL,!ml,DIG,DQD4
                0xCED4,                         //< 2nd Sync Byte D4
                0xC483,                         //< @PWR,NO RSTRIC,!st,!fi,OE,EN
                0x9880,                         //< mp,135kHz,MAX OUT
                0xCC77,                         //< OB1 OB0, LPX, ddy, DDIT, BW0
                0xE000,                         //< wakeup off
                0xC800,                         //< duty cycle - not used
                0xC000                          //< 1.00MHz, 2.2V
};

#elif RFM12_CONFIG_915_115200
const uint16_t rfm12_config[RFM12_CONFIG_COUNT] =
{
                0x80F8,                         //< 915mhz EL,EF,12.5pF
                RFM12_CMD_SLEEP,                //< low power mode with CLK disabled
                0xC602,                         //< 115200bps
                0x9440,                         //< VDI,FAST,340kHz,0dBm,-103dBm
                0xC2AC,                         //< AL,!ml,DIG,DQD4
                0xCED4,                         //< 2nd Sync Byte D4
                0xC483,                         //< @PWR,NO RSTRIC,!st,!fi,OE,EN
                0x98B0,                         //< mp,180kHz,MAX OUT
                0xCC77,                         //< OB1 OB0, LPX, ddy, DDIT, BW0
                0xE000,                         //< wakeup off
                0xC800,                         //< duty cycle - not used
                0xC000                          //< 1.00MHz, 2.2V
};

#else
#error "RFM12_CONFIG_XXX not defined"
#endif

/*---------------------------------------------------------------------------*/
PROCESS(driver_process, "RFM12 driver");
PT_THREAD(rfm12_rx_handler_pt(unsigned char c));
static struct pt rx_handler_pt;
static struct timer rxtimer;                  //< interbyte transmission-reception timer
/*---------------------------------------------------------------------------*/
static void load_config(const uint16_t *config);
static void dump_packet(int len);
static void delay_us(uint16_t delay);
static void rx_reset(void);

/**
 * \brief               this function initializes the driver
 *
 * \param config        needs to be defined by setting the RFM12_BAND and RFM12_DATARATE
 */
void
drvr_init(void)
{

    PT_INIT(&rx_handler_pt);
    delay_us(TIME_PWR_ON);

    //NIRQ pullup resistor
    RFM12_IRQ_PORT |= _BV(RFM12_IRQ_PIN);

    rfm12_spi_init();
    load_config(rfm12_config);

    process_start(&driver_process, NULL);

    //enable interrupt
    RFM12_ENABLE_RADIO_INTERRUPT();

    timer_set(&rxtimer, CLOCK_SECOND / 4);

}

/**
 * \brief       Changes the channel of the RFM12.
 *
 *              The rfm12 does not have a concept of channels but has the ability to change to a frequency
 *
 *              channels are configured in the rfm12_channel LUT.
 *
 *              This example is based on the 802.15.4 900MHZ frequency allocation for channels 1-10
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
drvr_set_channel(uint8_t channel)
{
        if(radio_state!= RADIO_IDLE)
          return RADIO_WRONG_STATE;

        if((channel < 0)||(channel > DRVR_MAX_CHANNELS-1))
          return RADIO_INVALID_ARGUMENT;

        rfm12_spi_xfer(rfm12_channel[channel]);         // change the channel
        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);                             // reset fifo to initiate PLL re-lock

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
drvr_set_power(uint8_t power)
{
    if (radio_state != RADIO_IDLE)
      return RADIO_WRONG_STATE;

    if ((power < PHY_TX_POWER_MIN) || (power > PHY_TX_POWER_MAX))
      return RADIO_INVALID_ARGUMENT;

    rfm12_spi_xfer(rfm12_config[8] & (0xfff0 | power));         // set TX power
    rfm12_spi_xfer(RFM12_CMD_FIFO_RST);                         // reset fifo to initiate PLL re-lock

    delay_us(TIME_PLL_LOCK);                                    // wait for PLL to lock after freq change

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
drvr_sleep(void)
{
        if(radio_state!= RADIO_IDLE)
                return RADIO_WRONG_STATE;

        /* Discard the current read buffer when the radio is shutting down. */
        data_len=0;

        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);             //  Reset the Fifo
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
drvr_wakeup(void)
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
static int
drvr_idle()
{
        if(radio_state == RADIO_IDLE)
                return RADIO_SUCCESS;         // already idle

        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        //reset FIFO sync latch
        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);

        //turn on RFM12 XTAL OSC
        rfm12_spi_xfer(RFM12_CMD_IDLE);

        radio_state = RADIO_IDLE;
        rfm12_internal_state = RFM12_IDLE;

        RFM12_DISABLE_RADIO_INTERRUPT();         // interrupts off

        return RADIO_SUCCESS;
}

/**
 * \brief       Starts transmitting the contents of the trx_buffer
 *
 *              To successfully start a transmission the RFM12 must be in idle and have
 *              a valid buffer length.
 *
 *              The buffer will already have header pre-filled with the packet length
 *              The correct number of preamble and sync bytes will be transmitted accordingly
 *              CRC is calculated and transmitted during the Tx process.
 *
 * \return RADIO_WKUP_ERROR             still in sleep state or an unfinished wakeup
 * \return RADIO_WRONG_STATE            is not in the idle state
 * \return RADIO_INVALID_ARGUMENT       packet size either too big or small to transmit
 * \return RADIO_SUCCESS                transmission started successfully
 */
static int
drvr_transmit()
{
        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        if ((radio_state!=RADIO_IDLE) && (radio_state!=RADIO_TX_ERROR))
            return RADIO_WRONG_STATE;

        //check length of data to transmit
        if ((data_len < MIN_PACKET_SIZE) || (data_len > MAX_PACKET_SIZE ))
            return RADIO_INVALID_ARGUMENT;

        //TODO: start transmit sequence
        // listen before talk enabled?
            // rfm12_spi_cmd0();
            // rfm12_spi_xfer(RFM12_CMD_RX_ON);         // turn on receiver
            // set timer for RSSI check time
            // radio_state = RADIO_TX_CHAN_CHK;
            // rfm12_internal_state = RFM12_TX_CHECK_CHANNEL;
            // csma_counter = x
            //
         // else

            crc_actual = 0xffff;        // Reset the CRC.
            buf_pos = 0;                // reset buffer position to start.
            irq_cnt = 0;                // reset preamble counter

            // load the first two preamble bytes into the RFM12 Tx buffer ready for Tx
            rfm12_spi_xfer(RFM12_CMD_TX | PREAMBLE);
            rfm12_spi_xfer(RFM12_CMD_TX | PREAMBLE);

            RFM12_SPI_ENABLE();
            rfm12_spi_cmd0();                           // clear any RFM12 interrupts
            RFM12_SPI_DISABLE();

            rfm12_spi_xfer(RFM12_CMD_TX_ON);            // turn on transmitter, rest is handled by RADIO_IRQ

            radio_state = RADIO_TX_TRANSMITTING;
            rfm12_internal_state = RFM12_TX_PREAMBLE;
            RFM12_ENABLE_RADIO_INTERRUPT();             // enable interrupts

        return RADIO_SUCCESS;
}

/**
 * \brief       turns on the receiver and waits for data to be received
 *
 * \return RADIO_SUCCESS        reception enabled successfully
 * \return RADIO_WKUP_ERROR     sleep mode or unfinished wakeup
 * \return RADIO_WRONG_STATE    must be in the idle state
 */
static int
drvr_receive()
{
        // sleep mode or unfinished wakeup
        if ((radio_state==RADIO_WAKEUP) || (radio_state==RADIO_SLEEP))
                return RADIO_WKUP_ERROR;

        if((radio_state != RADIO_IDLE) && (radio_state != RADIO_RX_PACKET_ERR))
                return RADIO_WRONG_STATE;

        data_len=0;
        buf_pos=0;

        RFM12_SPI_ENABLE();
        rfm12_spi_cmd0();                       // reset RFM12 interrupts
        RFM12_SPI_DISABLE();

        rfm12_spi_xfer(RFM12_CMD_RX_ON);        // receiver on

        rfm12_spi_xfer(RFM12_CMD_FIFO_RST);     // Reset FIFO
        rfm12_spi_xfer(RFM12_CMD_FIFO_EN);      // enable FIFO

        rfm12_internal_state = RFM12_RX_READY;
        radio_state = RADIO_RX_RECEIVING;

        RFM12_ENABLE_RADIO_INTERRUPT();         // enable interrupts, rest is handled by RADIO_IRQ

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
/**
 * \brief       This is where the upper layer initiates the sending of data
 *              via the Radio driver.
 * @param buf   buffer containing the data to send
 * @param len   length of data to send.
 * @return
 */
int
drvr_send(const void *buf, unsigned short len)
{

    if(len > MAX_PACKET_SIZE) {
             len = MAX_PACKET_SIZE;
    }

    LOG("rfm12_send: sending %d bytes\n", len);
    // copy incoming buffer to the Tx Buffer
    memcpy(&rfm12_trxbuf[HDR_SIZE], buf, len);

    return drvr_transmit();
}

/*---------------------------------------------------------------------------*/
int
drvr_read(const void *buf, unsigned short bufsize)
{
  uint8_t tmplen = 0;
  if(rfm12_internal_state == RFM12_RX_END) {
       dump_packet(data_len + 2);

       tmplen = data_len;

       if(tmplen > bufsize) {
         tmplen = bufsize;
       }

       memcpy(buf, &rfm12_trxbuf[HDR_SIZE], tmplen);

       /* header + content + CRC */
   /*     sstrength = (tmp_sstrength / (TR1001_HDRLEN + tr1001_rxlen + 2)) << 1; */
      // sstrength = (tmp_count ? ((tmp_sstrength / tmp_count) << 2) : 0);

       //TODO: add reception time stamp to packet

       rfm12_internal_state = RFM12_RX_READY;

       LOG("rfm12_read: got %d bytes\n", tmplen);
     }
  return tmplen;
}

/*---------------------------------------------------------------------------*/
void
drvr_set_receiver(void (*recv)(const struct radio_driver *))
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
 *  /brief      resets the receiver ready to start again.
 */
static void rx_reset(void)
{
    buf_pos = 0;
    data_len = 0;
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
    LOG("%d: 0x%02x\n", i, rfm12_trxbuf[i]);
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(driver_process, ev, data)
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
      drvr_read(&data, 0);
    }
  }
  PROCESS_END();
}

/**************************************************************************/
/**
 * \brief Task that handles the incoming data from the receiver
 *        this task checks header address information.
 *
 *        If a byte is not received within a given time then the thread is restarted;
 *        Checks:
 *        DRSSI valid before receive first byte
 *        CRC is checked if incorrect then the packet is dropped.
 *
 *
 * \param incoming_byte from the interrupt handler
 * \return
 */
PT_THREAD(rfm12_rx_handler_pt(unsigned char incoming_byte))
{
  static unsigned char tmppos;

  if(timer_expired(&rxtimer)) {
    PT_INIT(&rx_handler_pt);   // restart the receive thread as we've not received data within the given time
  }

  timer_restart(&rxtimer);

  PT_BEGIN(&rx_handler_pt);
  while(1) {

          rx_reset();    // reset the receiver

          /* Reset the CRC. */
          uint16_t crc_actual = 0xffff;
          uint16_t crc_expected;

          // TODO: start RSSI measurement

          // first add the sync bytes to CRC calculation as we have already received these (else we would'nt be getting through to here)
          crc_actual = crc16_add(SYNC1, crc_actual);
          crc_actual = crc16_add(SYNC2, crc_actual);

          /* packet header. */
          for(tmppos = 0 ; tmppos < HDR_SIZE ; ++tmppos)
            {
              PT_YIELD(&rx_handler_pt);              // wait until byte arrives
              rfm12_trxbuf[tmppos] = incoming_byte;
              /* Calculate the CRC. */
              crc_actual = crc16_add(rfm12_trxbuf[tmppos], crc_actual);
            }

          // we can now grab the length seeing we have the header
          data_len = ((PHY_header_t *)rfm12_trxbuf)->length;

          // check to see if we can handle the incoming packet length
          if (tmppos + data_len > sizeof(rfm12_trxbuf))
            PT_RESTART(&rx_handler_pt);                 // too long drop packet and restart from beginning

          // read packet data
           for(; tmppos < data_len + HDR_SIZE; ++tmppos) {
             PT_YIELD(&rx_handler_pt);
             rfm12_trxbuf[tmppos] = incoming_byte;
             crc_actual = crc16_add(rfm12_trxbuf[tmppos], crc_actual);
           }

           // read CRC
           for(tmppos = 0; tmppos < CRC_SIZE; ++tmppos) {
              PT_YIELD(&rx_handler_pt);
              crc_expected = (crc_expected << 8) | incoming_byte;   // will receive high byte first
           }

          if(crc_expected == crc_actual) {
              /* A full packet has been received and the CRC checks out. We'll
                 request the driver take care of the incoming data. */
                  //TODO: update successful Rx stats
                  process_poll(&driver_process);

              /* We'll set the receive state flag to signal that a full frame
                 is present in the buffer, and we'll wait until the buffer has
                 been taken care of. */
               rfm12_internal_state = RFM12_RX_END;

               PT_WAIT_UNTIL(&rx_handler_pt, rfm12_internal_state != RFM12_RX_END);

               drvr_idle();                       // finished turn off receiver

             } else {
               LOG("Incorrect CRC\n");
               //TODO: update bad crc stats
               radio_state = RADIO_RX_PACKET_ERR;
             }
  }
  PT_END(&rx_handler_pt);
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

        if(source & RFM12_IRQ_OVUR)             // FIFO error
        {
            // TODO: update fifo error stats
            // TODO: signal fifo error
            drvr_idle();                        // return to idle as error occurred
        }

        else if(source & RFM12_IRQ_FIFO)        // tx or rx event
        {
            if (radio_state == RADIO_TX_TRANSMITTING)
              {
                uint8_t out;
                switch(rfm12_internal_state)
                {
                  case RFM12_TX_PREAMBLE:

                      // check if we have sent the correct amount of preamble bytes
                      // note: we have already sent one if Tx has just started and we get here.
                      if(irq_cnt >= (PREAMBLE_SIZE-1))
                      {
                           irq_cnt = 0;               // reset the counter as we need for the next state
                           rfm12_internal_state++;
                      }
                      else irq_cnt++;                 // increment number of preamble sent

                      out = PREAMBLE;

                    break;

                  case RFM12_TX_SYNC:
                    if(irq_cnt >= SYNCWORD_SIZE)
                    {
                         irq_cnt = 0;               // reset the counter as we need for the next state
                         rfm12_internal_state++;
                    }
                    else
                    {
                        out = syncword[irq_cnt++];
                        crc_actual = crc16_add(out, crc_actual); // syncbyte included in CRC calc
                    }
                    break;

                  case RFM12_TX_HEADER:
                   if(irq_cnt >= SYNCWORD_SIZE)
                   {
                        irq_cnt = 0;                    // reset the counter as we need for the next state
                        rfm12_internal_state++;
                   }
                   else
                   {
                       out = syncword[irq_cnt++];
                       crc_actual = crc16_add(out, crc_actual);    // syncbyte included in CRC calc
                   }
                   break;

                  case RFM12_TX_DATA:
                    if(buf_pos >= (data_len-1))
                        rfm12_internal_state++;         // finished sending data
                    out = rfm12_trxbuf[buf_pos++];      // get next byte to send
                    crc_actual = crc16_add(out, crc_actual);
                    break;

                  case RFM12_TX_CRC1:
                    out = crc_actual >> 8;              // send crc high byte
                    rfm12_internal_state++;
                    break;

                  case RFM12_TX_CRC2:
                    out = crc_actual;                   // send crc low byte
                    rfm12_internal_state++;
                    break;

                  case RFM12_TX_TAIL:

                    if(irq_cnt >= (TAIL_SIZE-1))
                      rfm12_internal_state++;           // next state
                    else
                      irq_cnt++;                        // increment number of tail bytes sent
                    out=TAIL;
                    break;

                  case RFM12_TX_END:
                    drvr_idle();                        // finished and fall through
                  default:
                    out = TAIL;
                }
                  rfm12_spi_xfer(RFM12_CMD_TX | out);   // send next byte to fifo
                }
              }
            else // receiving
              {
                rfm12_rx_handler_pt(rfm12_spi_xfer(RFM12_CMD_TX)); //handle incoming byte
              }

        if (source & RFM12_IRQ_LOWBAT)           // battery voltage below threshold
        {
            // TODO: update low Battery stats
            // TODO: signal low battery status
        }


}
#endif /* defined(DOXYGEN) */
