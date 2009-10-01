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

 \file radio.h
 \ingroup rfm12widget

 \brief HEADER file for WidgetBoard RFM12 radio driver
 */
/**************************************************************************/

#ifndef RADIO_H_
#define RADIO_H_

#include <avr/io.h>
#include <util/delay_basic.h>
#include "contiki.h"
#include "types.h"
#include "buf.h"

#define delay_us( us )   ( _delay_loop_2( ( F_CPU / 4000000UL ) * ( us ) ) )  ///< 1 usec delay


#define RFM12_MAX_TX_FRAME_LENGTH               ( 127 ) /**< 127 Byte PSDU. */


/*============================ TYPEDEFS ======================================*/

/**
 *
 *  \brief  This enumeration defines the valid states for RFM12 radio transceiver.
 *
 */
typedef enum{
	RFM12_POWERING_UP,
	RFM12_IDLE,
	RFM12_TX_IDLE,
	RFM12_TX_BUSY,
	RFM12_RX_IDLE,
	RFM12_RX_BUSY,
	RFM12_SLEEP
}rfm12_radio_state_t;

/**
 *
 * \brief  This enumeration defines the necessary timing information for the
 *          RFM12 radio transceiver. All times are in microseconds (uSec).
 *
 *          These constants are extracted from the datasheet (where possible).
 *
 */
typedef enum{
    TIME_PWR_ON		                 = 5000,    /**<  Transition time from VCC/RESET is applied to PWR_ON. (max 100msec)*/
    TIME_PWR_ON_TO_TRX_OFF           = 510,     /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 880,     /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_DQD                         = 6,       /**<  Time it takes to do a DQD measurement. */
    TIME_DRSSI                       = 500,     /**<  Time it takes for DRSSI to go high after signal is above the preprogrammed limit. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 180,     /**<  Time for transition of Transceiver off to PLL being active */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,       /**<  Transition time from PLL active state to another. */
    TIME_PLL_LOCK					 = 30,      /**<  Maximum time it should take for the PLL to lock After 10MHz step, frequency error <10 kHz*/
    TIME_PLL_STARTUP				 = 200,		/**<  Initial calibration after power-up with running crystal oscillator	*/
    TIME_OSC_STARTUP				 = 1000, 	/**<  Max Crystal oscillator startup time (1msec typ  5msec Max)	*/
    TIME_TX_RX_XTAL_ON				 = 250,		/**<  Synthesizer off, crystal oscillator on during TX/RX change with 10 MHz step	*/
    TIME_RX_TX_XTAL_ON				 = 250,		/**<  Synthesizer off, crystal oscillator on during RX/TX change with 10 MHz step	*/
    TIME_TX_RX_SYNTH_ON				 = 150,		/**<  Synthesizer and crystal oscillator on during TX/RX change with 10 MHz step. */
    TIME_RX_TX_SYNTH_ON				 = 150		/**<  Synthesizer and crystal oscillator on during RX/TX change with 10 MHz step.	*/
} radio_timing_t;

/**************************************************************************/
/*!
    \brief  Radio status enumerations. These are the status values that can be
        returned by the radio.
*/
/**************************************************************************/
typedef enum{
	 RADIO_SUCCESS = 0x40,                       /**< The requested service was performed successfully. */
     RADIO_INVALID_ARGUMENT,                     /**< One or more of the supplied function arguments are invalid. */
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
} radio_status_t;


/**************************************************************************/
/*!
        Driver Control Block
        This structure contains flags and information that is used to control
        the behavior of the driver and allow it to respond to rx and tx events.
*/
/**************************************************************************/
typedef struct
{
    U8 handle;                  ///< Frame handle used to identify the frame and assoc it with a status.
    U8 status;                  ///< Transmit status after a transmission attempt.
    bool status_avail;  		///< Status available flag. Used to signal the driver that a new status is avail.
    bool data_rx;       		///< Data rx flag. Used to signal the driver that data has been received.
} radio_dcb_t;


typedef void (*radio_rx_callback) (uint16_t data);

// prototypes

/*********************************************************/
PROCESS_NAME(radio_driver_process);             		///< Main radio driver process
/*********************************************************/

void radio_init();
radio_status_t radio_set_channel(uint8_t channel);
radio_status_t radio_set_power(uint8_t pwr_level);
radio_status_t radio_send_data(const buffer_t *buf);
radio_status_t radio_sleep(uint8_t time);
radio_status_t radio_wakeup();
rfm_dcb_t *radio_dcb_get();
void radio_set_rand_seed();
uint16_t radio_get_rand();
radio_state_t radio_get_trx_state();

#endif /* RADIO_H_ */
