/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file contains the user callback functions corresponding to the
 *      application.
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
 * $RCSfile: usb_specific_request.c,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: usb_specific_request.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
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

//_____ P R I V A T E   D E C L A R A T I O N ______________________________

extern U8   code *pbuffer;
extern U8   data_to_transfer;
extern S_line_coding   line_coding;

//_____ D E C L A R A T I O N ______________________________________________

//! This function is called by the standard usb read request function when
//! the Usb request is not supported. This function returns TRUE when the
//! request is processed. This function returns FALSE if the request is not
//! supported. In this case, a STALL handshake will be automatically
//! sent by the standard usb read request function.
//!
//! @param type Not used
//! @param request Read request type
//!
//! @retval FALSE if unknown read type
//! @retval TRUE if request type is processed
//!
Bool usb_user_read_request(U8 type, U8 request)
{
    U8  descriptor_type ;
    U8  string_type     ;

    string_type     = Usb_read_byte();
	descriptor_type = Usb_read_byte();
	switch(request)
	{
		case GET_LINE_CODING:
			cdc_get_line_coding();
					return TRUE;
      			break;

  		case SET_LINE_CODING:
			cdc_set_line_coding();
					return TRUE;
      			break;

		case SET_CONTROL_LINE_STATE:
			cdc_set_control_line_state();
      return TRUE;
      break;

 		default:
			return FALSE;
			break;

	}
  	return FALSE;
}

//! usb_user_get_descriptor.
//!
//! @brief This function returns the size and the pointer on a user information
//! structure
//!
//! @param type descriptor type
//! @param string descriptor ID
//!
//! @retval FALSE
//!
Bool usb_user_get_descriptor(U8 type, U8 string)
{ /*
	switch(type)
	{
		case STRING_DESCRIPTOR:
      	switch (string)
      	{
        		case LANG_ID:
          		data_to_transfer = sizeof (usb_user_language_id);
          		pbuffer = &(usb_user_language_id.bLength);
					return TRUE;
          		break;
        		case MAN_INDEX:
         	 	data_to_transfer = sizeof (usb_user_manufacturer_string_descriptor);
         	 	pbuffer = &(usb_user_manufacturer_string_descriptor.bLength);
					return TRUE;
          		break;
        		case PROD_INDEX:
         		data_to_transfer = sizeof (usb_user_product_string_descriptor);
          		pbuffer = &(usb_user_product_string_descriptor.bLength);
					return TRUE;
          		break;
        		case SN_INDEX:
          		data_to_transfer = sizeof (usb_user_serial_number);
          		pbuffer = &(usb_user_serial_number.bLength);
					return TRUE;
          		break;
        		default:
          		return FALSE;
			}
		default:
			return FALSE;
	}*/

	return FALSE;
}

//! usb_user_endpoint_init.
//!
//! @brief This function configures the endpoints.
//!
//! @param conf_nb Not used
void usb_user_endpoint_init(U8 conf_nb)
{
  usb_configure_endpoint(INT_EP,      \
                         TYPE_INTERRUPT,     \
                         DIRECTION_IN,  \
                         SIZE_32,       \
                         ONE_BANK,     \
                         NYET_ENABLED);

  usb_configure_endpoint(TX_EP,      \
                         TYPE_BULK,  \
                         DIRECTION_IN,  \
                         SIZE_32,     \
                         ONE_BANK,     \
                         NYET_ENABLED);

  usb_configure_endpoint(RX_EP,      \
                         TYPE_BULK,     \
                         DIRECTION_OUT,  \
                         SIZE_32,       \
                         TWO_BANKS,     \
                         NYET_ENABLED);

  Usb_reset_endpoint(INT_EP);
  Usb_reset_endpoint(TX_EP);
  Usb_reset_endpoint(RX_EP);
}

//! cdc_get_line_coding.
//!
//! @brief This function manages reception of line coding parameters (baudrate...).
//!
void cdc_get_line_coding(void)
{
  	Usb_ack_receive_setup();
  	Usb_write_byte(LSB0(line_coding.dwDTERate));
  	Usb_write_byte(LSB1(line_coding.dwDTERate));
  	Usb_write_byte(LSB2(line_coding.dwDTERate));
  	Usb_write_byte(LSB3(line_coding.dwDTERate));
  	Usb_write_byte(line_coding.bCharFormat);
  	Usb_write_byte(line_coding.bParityType);
  	Usb_write_byte(line_coding.bDataBits);

  	Usb_send_control_in();
  	while(!(Is_usb_read_control_enabled()));
  	//Usb_clear_tx_complete();

   while(!Is_usb_receive_out());
   Usb_ack_receive_out();
}


//! cdc_set_line_coding.
//!
//! @brief This function manages reception of line coding parameters (baudrate...).
//!
void cdc_set_line_coding (void)
{
   Usb_ack_receive_setup();
   while (!(Is_usb_receive_out()));
	LSB0(line_coding.dwDTERate) = Usb_read_byte();
   LSB1(line_coding.dwDTERate) = Usb_read_byte();
   LSB2(line_coding.dwDTERate) = Usb_read_byte();
   LSB3(line_coding.dwDTERate) = Usb_read_byte();
   line_coding.bCharFormat = Usb_read_byte();
   line_coding.bParityType = Usb_read_byte();
   line_coding.bDataBits = Usb_read_byte();
  	Usb_ack_receive_out();

  	Usb_send_control_in();                // send a ZLP for STATUS phase
  	while(!(Is_usb_read_control_enabled()));
}

//! cdc_set_control_line_state.
//!
//! @brief This function manages the SET_CONTROL_LINE_LINE_STATE CDC request.
//!
//! Note: Can manage hardware flow control here...
//!
void cdc_set_control_line_state (void)
{
  	Usb_ack_receive_setup();
	Usb_send_control_in();
  	while(!(Is_usb_read_control_enabled()));
}
/** @}  */
