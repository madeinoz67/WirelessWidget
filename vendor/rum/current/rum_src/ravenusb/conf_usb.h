/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file contains the possible external configuration of the USB.
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
 * $RCSfile: conf_usb.h,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: conf_usb.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
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

#ifndef _CONF_USB_H_
#define _CONF_USB_H_



/**
   @addtogroup usb
   @{
*/


   // _________________ USB MODE CONFIGURATION ____________________________
   //
   //! @defgroup USB_op_mode USB operating modes configuration
   //! defines to enable device or host usb operating modes
   //! supported by the application
   //! @{

      //! @brief ENABLE to activate the host software library support
      //!
      //! Possible values ENABLE or DISABLE
      #define USB_HOST_FEATURE            DISABLED

      //! @brief ENABLE to activate the device software library support
      //!
      //! Possible values ENABLE or DISABLE
      #define USB_DEVICE_FEATURE          ENABLED

   //! @}

   // _________________ USB REGULATOR CONFIGURATION _______________________
   //
   //! @defgroup USB_reg_mode USB regulator configuration
   //! @{

   //! @brief Enable the internal regulator for USB pads
   //!
   //! When the application voltage is lower than 3.5V, to optimize power consumption
   //! the internal USB pads regulatr can be disabled.
#ifndef USE_USB_PADS_REGULATOR
   #define USE_USB_PADS_REGULATOR   ENABLE      // Possible values ENABLE or DISABLE
#endif
   //! @}

   // _________________ HOST MODE CONFIGURATION ____________________________
   //
   //! @defgroup USB_host_mode_cfg USB host operating mode configuration
   //!
   //! @{

   #define HID_CLASS          0x03
   #define MS_CLASS           0x08

   //!   @brief VID/PID supported table list
   //!
   //!   This table contains the VID/PID that are supported by the reduced host application
   //!   VID_PID_TABLE format definition:
   //!
   //!   VID_PID_TABLE      {VID1, number_of_pid_for_this_VID1, PID11_value,..., PID1X_Value \n
   //!                              ...\n
   //!                              ,VIDz, number_of_pid_for_this_VIDz, PIDz1_value,..., PIDzX_Value}
   #define VID_PID_TABLE            {0x03EB, 2, 0x201C, 0x2014 \
                                    ,0x0123, 3, 0x2000, 0x2100, 0x1258}

   //!   @brief CLASS/SUBCLASS_PROTOCOL supported table list
   //!
   //!   This table contains the CLASS/SUBCLASS/PROTOCOL that is supported by the reduced host application
   //!   This table definition allows to extended the reduced application device support to an entire Class/
   //!   /subclass/protocol instead of a simple VID/PID table list.
   //!
   //!   CLASS_SUBCLASS_PROTOCOL format definition: \n
   //!   CLASS_SUBCLASS_PROTOCOL  {CLASS1, SUB_CLASS1,PROTOCOL1, \n
   //!                                     ...\n
   //!                                     CLASSz, SUB_CLASSz,PROTOCOLz}
   #define CLASS_SUBCLASS_PROTOCOL     {0x0A, 0x00, 0x00, \
                                       0x00, 0x00, 0x00,\
                                       0xFF,0x00,0x00}

   //! The size of RAM buffer reserved of descriptors manipulation
   #define SIZEOF_DATA_STAGE        250

   //! The address that will be assigned to the connected device
   #define DEVICE_ADDRESS           0x05

   //! The host controller will be limited to the strict VID/PID list.
   //! When enabled, if the device PID/VID does not belongs  to the supported list,
   //! the host controller library will not go to deeper configuration, but to error state.
   #define HOST_STRICT_VID_PID_TABLE      DISABLE

   //! Try to configure the host pipe according to the device descriptors received
   #define HOST_AUTO_CFG_ENDPOINT         ENABLE

   //! Host start of frame interrupt always enable
   #define HOST_CONTINUOUS_SOF_INTERRUPT  DISABLE

   //! When Host error state detected, goto unattached state
   #define HOST_ERROR_RESTART             ENABLE

   //! Force WDT reset upon ID pin change
   #define ID_PIN_CHANGE_GENERATE_RESET   DISABLE

   //! NAK handshake in 1/4sec (250ms) before timeout
   #define NAK_TIMEOUT_DELAY              1

   #if (HOST_AUTO_CFG_ENDPOINT==FALSE)
      //! If no auto configuration of EP, map here user function
      #define        User_configure_endpoint()
   #endif

   //! @defgroup host_cst_actions USB host custom actions
   //!
   //! @{
   // write here the action to associate to each USB host event
   // be carefull not to waste time in order not disturbing the functions
   #define Usb_id_transition_action()
   #define Host_device_disconnection_action()
   #define Host_device_connection_action()
   #define Host_sof_action()
   #define Host_suspend_action()                      host_suspend_action();
   #define Host_hwup_action()
   #define Host_device_not_supported_action()
   #define Host_device_supported_action()
   #define Host_device_error_action()
   //! @}

extern void host_suspend_action(void);
   //! @}


// _________________ DEVICE MODE CONFIGURATION __________________________

   //! @defgroup USB_device_mode_cfg USB device operating mode configuration
   //!
   //! @{

#define NB_ENDPOINTS          4  //!  number of endpoints in the application including control endpoint
#define TX_EP					 0x01
#define RX_EP					 0x02
#define INT_EP              0x03


#define Usb_unicode(a)			((U16)(a))

   //! @defgroup device_cst_actions USB device custom actions
   //!
   //! @{
   // write here the action to associate to each USB event
   // be carefull not to waste time in order not disturbing the functions
#define Usb_sof_action()         sof_action();
#define Usb_wake_up_action()
#define Usb_resume_action()
#define Usb_suspend_action()     suspend_action();
#define Usb_reset_action()
#define Usb_vbus_on_action()
#define Usb_vbus_off_action()
#define Usb_set_configuration_action()
   //! @}

extern void sof_action(void);
extern void suspend_action(void);
   //! @}


/** @}  */

#endif // _CONF_USB_H_
