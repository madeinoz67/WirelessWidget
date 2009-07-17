/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file manages the CDC task for the virtual COM port.
 *
 * \par Application note:
 *      AVR280: USB Host CDC Demonstration
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name: RUM_REL_1_0_0_A $
 * $Revision: 1.1 $
 * $RCSfile: cdc_task.c,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: cdc_task.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
 ******************************************************************************/
/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

//_____  I N C L U D E S ___________________________________________________

#include "cdc_task.h"
#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_specific_request.h"
#include "uart_usb_lib.h"
#include "../mac.h"
#include <stdio.h>
#include "../serial.h"

/**
   @addtogroup usb
   @{
*/

//_____ M A C R O S ________________________________________________________





//_____ D E F I N I T I O N S ______________________________________________



//_____ D E C L A R A T I O N S ____________________________________________


volatile u8 cpt_sof;
extern u8    rx_counter;
extern u8    tx_counter;
S_line_coding line_coding;

uint8_t led0_timer, led1_timer, led2_timer, led3_timer;


//! @brief Initializes the hardware ressources required for CDC demo.
void cdc_task_init(void)
{
    Leds_init();
    Leds_off();
    Usb_enable_sof_interrupt();
}

//! @brief Entry point of the uart cdc management
//!
//! This function links the uart and the USB bus.
void cdc_task(void)
{
    // turn off LED's if necessary
    if (led0_timer) led0_timer--;
    else            Led0_off();
    if (led1_timer) led1_timer--;
    else            Led1_off();
    if (led2_timer) led2_timer--;
    else            Led2_off();
    if (led3_timer) led3_timer--;
    else            Led3_off();


    if(Is_device_enumerated() && DEBUG) //Enumeration processs OK ?
    {
        if(cpt_sof>=NB_MS_BEFORE_FLUSH && tx_counter!=0 )  //Flush buffer in Timeout
        {
            cpt_sof=0;
            uart_usb_flush();
        }

        while (tx_buf_has_char())    //Something from RES?
        {
            // send to USB port
            // don't send anything if USB can't accept chars
            Usb_select_endpoint(TX_EP);
            if (!uart_usb_tx_ready())
                return;
            // turn on LED
            Led2_on();
            led2_timer = 200;
            uart_usb_putchar(get_tx_char());   // Loop back, USART to USB
        }

        if (rx_buf_count() < BUFSIZE - 1)    //USART free ?
        {
            if (uart_usb_test_hit())   // Something received from the USB ?
            {
                while (rx_counter)
                {
                    // turn on LED
                    Led3_on();
                    led3_timer = 200;
                    add_char_to_rx(uart_usb_getchar());
                }
            }
        }
    }
}

//! @brief sof_action
//!
//! This function increments the cpt_sof counter each times
//! the USB Start Of Frame interrupt subroutine is executed (1ms)
//! Usefull to manage time delays
//!
//! @warning Code:?? bytes (function code length)
void sof_action(void)
{
   cpt_sof++;
}

/**
    @brief This will enable the RX_START LED for a period of 200 cycles.
*/
void rx_start_led(void)
{
    Led0_on();
    led0_timer = 200;
}

/**
    @brief This will enable the TRX_END LED for a period of 200 cycles.
*/
void tx_end_led(void)
{
    Led1_on();
    led1_timer = 200;
}
/** @}  */
