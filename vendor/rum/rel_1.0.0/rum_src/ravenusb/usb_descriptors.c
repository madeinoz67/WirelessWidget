/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file contains the usb parameters that uniquely identify the
 *      application through descriptor tables.
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
 * $RCSfile: usb_descriptors.c,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: usb_descriptors.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
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


//_____ I N C L U D E S ____________________________________________________

#include "config.h"

#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_specific_request.h"

/**
   @addtogroup usb
   @{
*/

//_____ M A C R O S ________________________________________________________




//_____ D E F I N I T I O N ________________________________________________

// usb_user_device_descriptor
code S_usb_device_descriptor usb_dev_desc =
{
  sizeof(usb_dev_desc)
, DEVICE_DESCRIPTOR
, Usb_write_word_enum_struc(USB_SPECIFICATION)
, DEVICE_CLASS
, DEVICE_SUB_CLASS
, DEVICE_PROTOCOL
, EP_CONTROL_LENGTH
, Usb_write_word_enum_struc(VENDOR_ID)
, Usb_write_word_enum_struc(PRODUCT_ID)
, Usb_write_word_enum_struc(RELEASE_NUMBER)
, MAN_INDEX
, PROD_INDEX
, SN_INDEX
, NB_CONFIGURATION
};

// usb_user_configuration_descriptor FS
code S_usb_user_configuration_descriptor usb_conf_desc = {
 { sizeof(S_usb_configuration_descriptor)
 , CONFIGURATION_DESCRIPTOR
 //, Usb_write_word_enum_struc(sizeof(usb_conf_desc_kbd))
 , 0x0043 //TODO: Change to generic codewith sizeof
 , NB_INTERFACE
 , CONF_NB
 , CONF_INDEX
 , CONF_ATTRIBUTES
 , MAX_POWER
 }
 ,
 { sizeof(S_usb_interface_descriptor)
 , INTERFACE_DESCRIPTOR
 , INTERFACE0_NB
 , ALTERNATE0
 , NB_ENDPOINT0
 , INTERFACE0_CLASS
 , INTERFACE0_SUB_CLASS
 , INTERFACE0_PROTOCOL
 , INTERFACE0_INDEX
 }
 ,
 { 0x05, 0x24, 0x00, 0x10, 0x01, 0x05, 0x24, 0x01, 0x03, 0x01, 0x04, 0x24, 0x02, 0x06,0x05, 0x24, 0x06, 0x00, 0x01 }
 ,
 { sizeof(S_usb_endpoint_descriptor)
 , ENDPOINT_DESCRIPTOR
 , ENDPOINT_NB_3
 , EP_ATTRIBUTES_3
 , Usb_write_word_enum_struc(EP_SIZE_3)
 , EP_INTERVAL_3
 }
 ,
 { sizeof(S_usb_interface_descriptor)
 , INTERFACE_DESCRIPTOR
 , INTERFACE1_NB
 , ALTERNATE1
 , NB_ENDPOINT1
 , INTERFACE1_CLASS
 , INTERFACE1_SUB_CLASS
 , INTERFACE1_PROTOCOL
 , INTERFACE1_INDEX
 }
 ,
 { sizeof(S_usb_endpoint_descriptor)
 , ENDPOINT_DESCRIPTOR
 , ENDPOINT_NB_1
 , EP_ATTRIBUTES_1
 , Usb_write_word_enum_struc(EP_SIZE_1)
 , EP_INTERVAL_1
 }
 ,
 { sizeof(S_usb_endpoint_descriptor)
 , ENDPOINT_DESCRIPTOR
 , ENDPOINT_NB_2
 , EP_ATTRIBUTES_2
 , Usb_write_word_enum_struc(EP_SIZE_2)
 , EP_INTERVAL_2
 }


};



                                      // usb_user_manufacturer_string_descriptor
code S_usb_manufacturer_string_descriptor usb_user_manufacturer_string_descriptor = {
  sizeof(usb_user_manufacturer_string_descriptor)
, STRING_DESCRIPTOR
, USB_MANUFACTURER_NAME
};


                                      // usb_user_product_string_descriptor

code S_usb_product_string_descriptor usb_user_product_string_descriptor = {
  sizeof(usb_user_product_string_descriptor)
, STRING_DESCRIPTOR
, USB_PRODUCT_NAME
};


                                      // usb_user_serial_number

code S_usb_serial_number usb_user_serial_number = {
  sizeof(usb_user_serial_number)
, STRING_DESCRIPTOR
, USB_SERIAL_NUMBER
};


                                      // usb_user_language_id

code S_usb_language_id usb_user_language_id = {
  sizeof(usb_user_language_id)
, STRING_DESCRIPTOR
, Usb_write_word_enum_struc(LANGUAGE_ID)
};


/** @}  */
