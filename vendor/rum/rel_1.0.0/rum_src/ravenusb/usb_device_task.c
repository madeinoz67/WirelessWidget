/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      The USB task checks the income of new requests from the USB Host.
 *      When a Setup request occurs, this task will launch the processing
 *      of this setup contained in the usb_standard_request.c file.
 *      Other class specific requests are also processed in this file.
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
 * $RCSfile: usb_device_task.c,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: usb_device_task.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
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

#include "config.h"
#include "usb_device_task.h"

#include "usb_drv.h"
#include "usb_descriptors.h"
#include "pll_drv.h"


/**
   @addtogroup usb
   @{
*/

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//!
//! Public : (bit) usb_connected
//! usb_connected is set to TRUE when VBUS has been detected
//! usb_connected is set to FALSE otherwise
//!/
bit   usb_connected;


//!
//! Public : (U8) usb_configuration_nb
//! Store the number of the USB configuration used by the USB device
//! when its value is different from zero, it means the device mode is enumerated
//! Used with USB_DEVICE_FEATURE == ENABLED only
//!/
extern U8  usb_configuration_nb;



//_____ D E C L A R A T I O N S ____________________________________________

//!
//! @brief This function initializes the USB device controller and system interrupt
//!
//! This function enables the USB controller and init the USB interrupts.
//! The aim is to allow the USB connection detection in order to send
//! the appropriate USB event to the operating mode manager.
//!/
void usb_device_task_init(void)
{
   Enable_interrupt();
   Usb_disable();
   Usb_enable();
   Usb_select_device();
#if (USB_LOW_SPEED_DEVICE==ENABLE)
   Usb_low_speed_mode();
#endif
   Usb_enable_vbus_interrupt();
   Enable_interrupt();

}

//!
//! @brief This function initializes the USB device controller
//!
//! This function enables the USB controller and init the USB interrupts.
//! The aim is to allow the USB connection detection in order to send
//! the appropriate USB event to the operating mode manager.
//! Start device function is executed once VBUS connection has been detected
//! either by the VBUS change interrupt either by the VBUS high level
void usb_start_device (void)
{
   Pll_start_auto();
   Wait_pll_ready();
   Usb_unfreeze_clock();
   Usb_enable_suspend_interrupt();
   Usb_enable_reset_interrupt();
   usb_init_device();         // configure the USB controller EP0
   Usb_attach();
}

//! @brief Entry point of the USB device mamagement
//!
//! This function is the entry point of the USB management. Each USB
//! event is checked here in order to launch the appropriate action.
//! If a Setup request occurs on the Default Control Endpoint,
//! the usb_process_request() function is call in the usb_standard_request.c file
void usb_device_task(void)
{
   if (Is_usb_vbus_high()&& usb_connected==FALSE)
   {
      usb_connected = TRUE;
      usb_start_device();
      Usb_vbus_on_action();
   }

   if(Is_usb_event(EVT_USB_RESET))
   {
      Usb_ack_event(EVT_USB_RESET);
      Usb_reset_endpoint(0);
      usb_configuration_nb=0;
   }

   // Here connection to the device enumeration process
   Usb_select_endpoint(EP_CONTROL);
   if (Is_usb_receive_setup())
   {
      usb_process_request();
   }
}
/** @}  */
