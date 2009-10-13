#ifndef RFM12_DRVR_H_
#define RFM12_DRVR_H_
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

 \file rfm12_drvr.h
 \ingroup

 \brief
 */
/**************************************************************************/
//#include "contiki.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "rfm12_spi.h"

#define RFM12_MAX_PAYLOAD       127

//#define RFM12_CONFIG_433_57600        1
//#define RFM12_CONFIG_868_57600        1
#define RFM12_CONFIG_915_57600          1
//#define RFM12_CONFIG_915_115200         1

#define RFM12_CMD_WKUP          0x820D    //< EX, EB, DC
#define RFM12_CMD_IDLE          0x820D    //< EX, EB, DC
#define RFM12_CMD_SLEEP         0x8205    //< EB, DC, !EX
#define RFM12_CMD_FIFO_RST      0xCA81    //< Reset FIFO
#define RFM12_CMD_FIFO_EN       0xCA83    //< Enable FIFO Fill

// RFM12 hardware config
#define RFM12_IRQ_PORT          PORTD
#define RFM12_IRQ_DDR           DDRD
#define RFM12_IRQ_PIN           2
#define RFM12_RADIO_IRQ         INT0_vect // IRQ for RFM12 - INT0 falling trigger interrupt

// RFM12 Interrupt sources
#define RFM12_IRQ_FIFO          0X80      //< (NIRQ) FIFO Interrupt - Rx = byte received, Tx = ready for next byte
#define RFM12_IRQ_POR           0x40      //< (NIRQ) Power on Reset detected
#define RFM12_IRQ_OVUR          0X20      //< (NIRQ) Fifo error - Rx = overrun, Tx = underrun
#define RFM12_IRQ_WKUP          0x10      //< (NIRQ) Wakeup timer overflow
#define RFM12_IRQ_EXT           0x08      //< (NIRQ) external interrupt pin on RFM12.
#define RFM12_IRQ_LOWBAT        0x04      //< (NIRQ) Battery voltage below threshold.
#define RFM12_IRQ_COMM_OVUR     0XA0      //<< FIFO AND overrun/underrun

#define RFM12_ENTER_CRIT()      {uint8_t volatile saved_sreg = SREG; cli()
#define RFM12_LEAVE_CRIT()      SREG = saved_sreg;}
#define RFM12_ENABLE_RADIO_INTERRUPT( ) { ( EIMSK |= _BV(INT0) ) ; EICRA |= 0x02 ; RFM12_IRQ_PORT &= ~_BV(RFM12_IRQ_PIN);  RFM12_IRQ_DDR &= ~_BV(RFM12_IRQ_PIN); }
#define RFM12_DISABLE_RADIO_INTERRUPT( ) ( EIMSK &= ~ _BV(INT0)

// RFM12 Channels
#define RFM12_MAX_CHANNELS 10
extern const uint16_t rfm12_channel[RFM12_MAX_CHANNELS];

// config array
#define RFM12_CONFIG_COUNT 13
extern const uint16_t rfm12_config[RFM12_CONFIG_COUNT];
extern const struct radio_driver rfm12_driver;

enum{
  RFM12_TXCONF_POWER_0          = 0x00,  //  0db  (Strongest Tx)
  RFM12_TXCONF_POWER_2_5        = 0x01,  // -2.5db
  RFM12_TXCONF_POWER_5          = 0x02,  // -5.0db
  RFM12_TXCONF_POWER_7_5        = 0x03,  // -7.5dB
  RFM12_TXCONF_POWER_10         = 0x04,  // -10.0dB
  RFM12_TXCONF_POWER_12_5       = 0x05,  // -12.5dB
  RFM12_TXCONF_POWER_15         = 0x06,  // -15.0dB
  RFM12_TXCONF_POWER_17_5       = 0x07   // -17.5dB (Weakest)
};

/**************************************************************************/
/*!
    \brief  Radio status enumerations. These are the status values that can be
            returned by the radio.
*/
/**************************************************************************/
enum{
     RADIO_SUCCESS,                       /**< The requested service was performed successfully. */
     RADIO_WKUP_ERROR,
     RADIO_INVALID_ARGUMENT,                     /**< One or more of the supplied function arguments are invalid. */
     RADIO_INVALID,
     RADIO_CHANNEL_ACCESS_FAILURE,
     RADIO_WRONG_STATE,                          /**< radio is in the wrong state to perform requested function */
     RADIO_TIMED_OUT,                            /**< The requested service timed out. */
     RADIO_BUSY_STATE,                           /**< The radio transceiver is busy receiving or transmitting. */
     RADIO_CCA_IDLE,                             /**< Channel is clear, available to transmit a new frame. */
     RADIO_CCA_BUSY,                             /**< Channel busy. */
     RADIO_TRX_BUSY,                             /**< Transceiver is busy receiving or transmitting data. */
     RADIO_BAT_LOW,                              /**< Measured battery voltage is lower than voltage threshold. */
     RADIO_BAT_OK,                               /**< Measured battery voltage is above the voltage threshold. */
     RADIO_DRSSI_LOW,                            /**< The DRSSI is below the set threshold. */
     RADIO_DRSSI_OK,                             /**< The DRSSI is above the set threshold. */
     RADIO_TX_FIFO_UNDERRUN,                     /**< TX buffer empty when trying to transmit */
     RADIO_RX_FIFO_OVERRUN,                      /**< RX buffer overrun when receiving */
     RADIO_POR_DETECTED,                         /**< Power on Reset detected  */
     RADIO_TRX_WAKEUP,                           /**< Transceiver wake-up */
     RADIO_ANT_TUNE_DETECT,                      /**< Receiver Antenna tuning circuit detected signal */
     RADIO_DQD_OK,                               /**< Receiver data quality detect OK */
     RADIO_CLK_RECOVERY_OK,                      /**< Receiver FSK Clock recovery OK */
     RADIO_CRC_FAILED                            /**< The CRC failed for the actual frame. */
};

enum radio_state{
        RADIO_SLEEP            = 0x00,
        RADIO_WAKEUP           = 0x80,
        RADIO_IDLE             = 0x40,
        RADIO_RX_DQD_CHK       = 0x20,
        RADIO_RX_RECEIVING     = 0x22,
        RADIO_RX_PACKET_VALID  = 0x24,
        RADIO_RX_COMPLETE      = 0x28,
        RADIO_TX_CHAN_CHK      = 0x10,
        RADIO_TX_TRANSMITTING  = 0x12,
        RADIO_TX_ERROR         = 0x14,
        RADIO_TX_COMPLETE      = 0x18
};

/**
 *
 * \brief  This enumeration defines the necessary timing information for the
 *          RFM12 radio transceiver. All times are in microseconds (uSec).
 *
 *          These constants are extracted from the datasheet (where possible).
 *
 */
enum{
    TIME_PWR_ON                      = 5000,    /**<  Transition time from VCC/RESET is applied to PWR_ON. (max 100msec)*/
    TIME_PWR_ON_TO_TRX_OFF           = 510,     /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 1000,    /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_DQD                         = 6,       /**<  Time it takes to do a DQD measurement. */
    TIME_DRSSI                       = 500,     /**<  Time it takes for DRSSI to go high after signal is above the preprogrammed limit. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180,     /**<  Time for transition of Transceiver off to PLL being active */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,       /**<  Transition time from PLL active state to another. */
    TIME_PLL_LOCK                    = 30,      /**<  Maximum time it should take for the PLL to lock After 10MHz step, frequency error <10 kHz*/
    TIME_PLL_STARTUP                 = 200,     /**<  Initial calibration after power-up with running crystal oscillator        */
    TIME_OSC_STARTUP                 = 5000,    /**<  Max Crystal oscillator startup time (1msec typ  5msec Max)        */
    TIME_TX_RX_XTAL_ON               = 250,     /**<  Synthesizer off, crystal oscillator on during TX/RX change with 10 MHz step       */
    TIME_RX_TX_XTAL_ON               = 250,     /**<  Synthesizer off, crystal oscillator on during RX/TX change with 10 MHz step       */
    TIME_TX_RX_SYNTH_ON              = 150,     /**<  Synthesizer and crystal oscillator on during TX/RX change with 10 MHz step. */
    TIME_RX_TX_SYNTH_ON              = 150      /**<  Synthesizer and crystal oscillator on during RX/TX change with 10 MHz step.       */
};


void rfm12_drvr_init(const uint16_t* config);

int rfm12_drvr_set_power(uint8_t power);
int rfm12_drvr_set_channel(uint8_t channel);

int rfm12_drvr_sleep(void);
int rfm12_drvr_wakeup(void);
int rfm12_drvr_idle(void);
int rfm12_drvr_transmit(void);
int rfm12_drvr_receive(void);

int rfm12_drvr_send(const void *buf, unsigned short len);
int rfm12_drvr_read(const void *buf, unsigned short len);

void rfm12_drvr_set_receiver(void (*recv)(const struct radio_driver *));

// data transmit
int rfm12_drvr_tx(uint8_t *hdr, uint8_t *data, uint8_t len);

#endif /* RFM12_DRVR_H_ */
