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
#include "widgetmesh.h"

//#define RFM12_MAX_PAYLOAD       66      //< size of packet payload in bytes

// un-comment only ONE of the following reset configurations
//#define RFM12_CONFIG_433_57600        1
//#define RFM12_CONFIG_868_57600        1
#define RFM12_CONFIG_915_57600          1
//#define RFM12_CONFIG_915_115200       1

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
#define RFM12_DISABLE_RADIO_INTERRUPT( ) ( EIMSK &= ~ _BV(INT0))

// RFM12 Channels
#define DRVR_MAX_CHANNELS      10
extern const uint16_t rfm12_channel[DRVR_MAX_CHANNELS];


// rfm12 register configuration
#define RFM12_MAX_CONFIG        12
extern const uint16_t rfm12_config[RFM12_MAX_CONFIG];

extern const struct radio_driver rfm12_driver;

// common commands used in the driver
#define RFM12_CMD_WKUP          0x8209    //< EX, !EB, DC, !ew
#define RFM12_CMD_IDLE          0x8209    //< EX, !EB, DC, !ew
#define RFM12_CMD_SLEEP         0x8201    //< !EB, DC, !EX, !ew
#define RFM12_CMD_RX_ON         0x82D9    //< ER, ES, EX, !eb, !ew, DC
#define RFM12_CMD_TX_ON         0x8239    //< ET, ES, EX, !eb, !ew, DC
#define RFM12_CMD_FIFO_RST      0xCA81    //< Reset FIFO
#define RFM12_CMD_FIFO_EN       0xCA83    //< Enable FIFO Fill
#define RFM12_CMD_TX            0xB800    //< Tx buffer write
#define RFM12_CMD_READ          0xB000    //< Rx buffer read

enum{
  PHY_TX_POWER_MAX            = 0x00,
  PHY_TXCONF_POWER_0          = 0x00,  //  0db  (Strongest Tx)
  PHY_TXCONF_POWER_2_5        = 0x01,  // -2.5db
  PHY_TXCONF_POWER_5          = 0x02,  // -5.0db
  PHY_TXCONF_POWER_7_5        = 0x03,  // -7.5dB
  PHY_TXCONF_POWER_10         = 0x04,  // -10.0dB
  PHY_TXCONF_POWER_12_5       = 0x05,  // -12.5dB
  PHY_TXCONF_POWER_15         = 0x06,  // -15.0dB
  PHY_TXCONF_POWER_17_5       = 0x07,  // -17.5dB (Weakest)
  PHY_TX_POWER_MIN            = 0x07
};

/**************************************************************************/
/*!
    \brief  Radio status. These are the status values that can be
            returned by the radio driver.
*/
/**************************************************************************/
enum{
     RADIO_SUCCESS,                              /**< The requested service was performed successfully. */
     RADIO_WKUP_ERROR,                           /**< The radio did not wakeup within given time. */
     RADIO_INVALID_ARGUMENT,                     /**< One or more of the supplied function arguments are invalid. */
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

/**
 *  \brief      valid radio driver states
 */
enum radio_state{
        RADIO_SLEEP            = 0x00,//!< radio is in power saving mode
        RADIO_WAKEUP           = 0x80,//!< radio is waking up
        RADIO_IDLE             = 0x40,//!< radio is idle
        RADIO_RX_RSSI_CHK      = 0x20,//!< RSSI being performed by radio
        RADIO_RX_RECEIVING     = 0x22,//!< Receiving packet
        RADIO_RX_PACKET_ERR    = 0x24,//!< in valid packet CRC fail
        RADIO_RX_COMPLETE      = 0x28,//!< Rx has been completed
        RADIO_TX_CHAN_CHK      = 0x10,//!< Channel check before transmission
        RADIO_TX_TRANSMITTING  = 0x12,//!< transmitting
        RADIO_TX_ERROR         = 0x14,//!< error transmitting - channel not free
        RADIO_TX_COMPLETE      = 0x18 //!< Tx completed successfully
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
    TIME_PWR_ON                      = 5000,    //<  Transition time from VCC/RESET is applied to PWR_ON. (max 100msec)
    TIME_PWR_ON_TO_TRX_OFF           = 510,     //<  Transition time from P_ON to TRX_OFF.
    TIME_SLEEP_TO_TRX_OFF            = 1000,    //<  Transition time from SLEEP to TRX_OFF.
    TIME_DQD                         = 6,       //<  Time it takes to do a DQD measurement.
    TIME_DRSSI                       = 500,     //<  Time it takes for DRSSI to go high after signal is above the preprogrammed limit.
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180,     //<  Time for transition of Transceiver off to PLL being active
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,       //<  Transition time from PLL active state to another.
    TIME_PLL_LOCK                    = 30,      //<  Maximum time it should take for the PLL to lock After 10MHz step, frequency error <10 kHz
    TIME_PLL_STARTUP                 = 200,     //<  Initial calibration after power-up with running crystal oscillator
    TIME_OSC_STARTUP                 = 5000,    //<  Max Crystal oscillator startup time (1msec typ  5msec Max)
    TIME_TX_RX_XTAL_ON               = 250,     //<  Synthesizer off, crystal oscillator on during TX/RX change with 10 MHz step
    TIME_RX_TX_XTAL_ON               = 250,     //<  Synthesizer off, crystal oscillator on during RX/TX change with 10 MHz step
    TIME_TX_RX_SYNTH_ON              = 150,     //<  Synthesizer and crystal oscillator on during TX/RX change with 10 MHz step.
    TIME_RX_TX_SYNTH_ON              = 150,     //<  Synthesizer and crystal oscillator on during RX/TX change with 10 MHz step.
    TIME_TRX_INTERBYTE               = 500      //<  Inter-byte timing for Tx/Rx
};


void drvr_init(void);

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
int drvr_set_power(uint8_t power);

/**
 * \brief       function is called by upper layer to change channel of the RFM12.
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
int drvr_set_channel(uint8_t channel);

int drvr_scan(void);

/**
 * \brief       puts the RFM12 into sleep mode
 *              note this does not power down the RFM12 but turns off the XTAL OSC to save power.
 *
 * \return RADIO_SUCCESS        RFM12 successfully put into sleep mode
 * \return RADIO_WRONG_STATE    RFM12 must be in the idle state first
 */
int drvr_sleep(void);

/**
 *  \brief      wakes up the RFM12.
 *              XTAL OSC is turned on.
 *
 *  \return RADIO_SUCCESS       RFM12 successfully woken up
 *  \return RADIO_WRONG_STATE   not in sleep mode
 */
int drvr_wakeup(void);

/**
 * \brief       Function is called by upper the layer to transmit data
 *
 *              This function will calculate CRC and append correct length information
 * \param *buf  pointer to buffer containing data to transmit
 * \param len   length of data to transmit
 *
 * \return RADIO_SUCCESS        Data successfully transmitted
 * \return RADIO_
 *
 */
int drvr_send(const void *buf, unsigned short len);

int drvr_read(const void *buf, unsigned short bufsize);

void drvr_set_receiver(void (*recv)(const struct radio_driver *));

// data transmit
int drvr_tx(uint8_t *hdr, uint8_t *data, uint8_t len);

#endif /* RFM12_DRVR_H_ */
