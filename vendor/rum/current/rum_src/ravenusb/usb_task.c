/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file manages the USB task either device/host or both.
 *
 *      The USB task selects the correct USB task (usb_device task or usb_host task
 *      to be executed depending on the current mode available.
 *
 *      According to USB_DEVICE_FEATURE and USB_HOST_FEATURE value (located in conf_usb.h file)
 *      The usb_task can be configured to support USB DEVICE mode or USB Host mode or both
 *      for a dual role device application.
 *
 *      This module also contains the general USB interrupt subroutine. This subroutine is used
 *      to detect asynchronous USB events.
 *
 *      Note:
 *        - The usb_task belongs to the scheduler, the usb_device_task and usb_host do not, they are called
 *          from the general usb_task
 *        - See conf_usb.h file for more details about the configuration of this module
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
 * $RCSfile: usb_task.c,v $
 * $Date: 2009/05/20 20:52:01 $  \n
 * $Id: usb_task.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
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
#include "conf_usb.h"
#include "usb_drv.h"
#if ((USB_DEVICE_FEATURE == ENABLED))
#include "usb_descriptors.h"
#endif
#include "pll_drv.h"

#if ((USB_HOST_FEATURE == ENABLED))
   #include "modules/usb/host_chap9/usb_host_task.h"
   #if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)
      extern U8 g_sav_int_sof_enable;
   #endif
#endif

#if ((USB_DEVICE_FEATURE == ENABLED))
   #include "usb_device_task.h"
#endif

#ifndef  USE_USB_PADS_REGULATOR
   #error "USE_USB_PADS_REGULATOR" should be defined as ENABLE or DISABLE in conf_usb.h file
#endif
#include <avr/sleep.h>

/**
   @addtogroup usb
   @{
*/

//_____ M A C R O S ________________________________________________________

#ifndef LOG_STR_CODE
#define LOG_STR_CODE(str)
#else
U8 code log_device_disconnect[]="Device Disconnected";
U8 code log_id_change[]="Pin Id Change";
#endif

//_____ D E F I N I T I O N S ______________________________________________

//!
//! Public : U16 g_usb_event
//! usb_connected is used to store USB events detected upon
//! USB general interrupt subroutine
//! Its value is managed by the following macros (See usb_task.h file)
//! Usb_send_event(x)
//! Usb_ack_event(x)
//! Usb_clear_all_event()
//! Is_usb_event(x)
//! Is_not_usb_event(x)
volatile U16 g_usb_event=0;


#if (USB_DEVICE_FEATURE == ENABLED)
//!
//! Public : (bit) usb_connected
//! usb_connected is set to TRUE when VBUS has been detected
//! usb_connected is set to FALSE otherwise
//! Used with USB_DEVICE_FEATURE == ENABLED only
//!/
extern bit   usb_connected;

//!
//! Public : (U8) usb_configuration_nb
//! Store the number of the USB configuration used by the USB device
//! when its value is different from zero, it means the device mode is enumerated
//! Used with USB_DEVICE_FEATURE == ENABLED only
//!/
extern U8    usb_configuration_nb;
#endif


#if (USB_HOST_FEATURE == ENABLED)
//!
//! Private : (U8) private_sof_counter
//! Incremented  by host SOF interrupt subroutime
//! This counter is used to detect timeout in host requests.
//! It must not be modified by the user application tasks.
//!/
volatile U8 private_sof_counter=0;

   #if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)
extern volatile S_pipe_int   it_pipe_str[MAX_EP_NB];
   #endif

#endif

#if ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == ENABLED))
//!
//! Public : (U8) g_usb_mode
//! Used in dual role application (both device/host) to store
//! the current mode the usb controller is operating
//!/
   U8 g_usb_mode=USB_MODE_UNDEFINED;
   U8 g_old_usb_mode;
#endif

//_____ D E C L A R A T I O N S ____________________________________________

/**
 * @brief This function initializes the USB proces.
 *
 *  Depending on the mode supported (HOST/DEVICE/DUAL_ROLE) the function
 *  calls the coresponding usb mode initialization function
 */
void usb_task_init(void)
{
   #if (USB_HOST_FEATURE == ENABLED && USB_DEVICE_FEATURE == ENABLED)
   U8 delay;
   #endif

   #if (USE_USB_PADS_REGULATOR==ENABLE)  // Otherwise assume USB PADs regulator is not used
   Usb_enable_regulator();
   #endif

// ---- DUAL ROLE DEVICE USB MODE ---------------------------------------------
#if ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == ENABLED))
   Usb_enable_uid_pin();
   delay=PORTA;
   g_usb_mode=USB_MODE_UNDEFINED;
   if(Is_usb_id_device())
   {
     g_usb_mode=USB_MODE_DEVICE;
     usb_device_task_init();
   }
   else
   {
     g_usb_mode=USB_MODE_HOST;
     Usb_ack_id_transition(); // REQUIRED !!! Startup with ID=0, Ack ID pin transistion (default hwd start up is device mode)
     Usb_enable_id_interrupt();
     Enable_interrupt();
     usb_host_task_init();
   }
   g_old_usb_mode=g_usb_mode;   // Store current usb mode, for mode change detection
// -----------------------------------------------------------------------------

// ---- DEVICE ONLY USB MODE ---------------------------------------------------
#elif ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == DISABLE))
   Usb_force_device_mode();
   usb_device_task_init();
// -----------------------------------------------------------------------------

// ---- REDUCED HOST ONLY USB MODE ---------------------------------------------
#elif ((USB_DEVICE_FEATURE == DISABLE)&& (USB_HOST_FEATURE == ENABLED))
   Usb_force_host_mode();
   usb_host_task_init();
#elif ((USB_DEVICE_FEATURE == DISABLE)&& (USB_HOST_FEATURE == DISABLE))
   #error  at least one of USB_DEVICE_FEATURE or USB_HOST_FEATURE should be enabled
#endif
// -----------------------------------------------------------------------------
}

/**
 *  @brief Entry point of the USB management
 *
 *  Depending on the USB mode supported (HOST/DEVICE/DUAL_ROLE) the function
 *  calls the corresponding usb management function.
*/
void usb_task(void)
{
// ---- DUAL ROLE DEVICE USB MODE ---------------------------------------------
#if ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == ENABLED))
   if(Is_usb_id_device())
   { g_usb_mode=USB_MODE_DEVICE;}
   else
   { g_usb_mode=USB_MODE_HOST;}
  // TODO !!! ID pin hot state change
  // Preliminary management: HARDWARE RESET !!!
   #if ( ID_PIN_CHANGE_GENERATE_RESET == ENABLE)
     // Hot ID transition generates wdt reset
      if((g_old_usb_mode!=g_usb_mode))
      #ifndef  AVRGCC
         {Wdt_change_16ms(); while(1);   LOG_STR_CODE(log_id_change);}
      #else
         {Wdt_change_enable(); while(1); LOG_STR_CODE(log_id_change);}
      #endif

   #endif
  g_old_usb_mode=g_usb_mode;   // Store current usb mode, for mode change detection
  // Depending on current usb mode, launch the correct usb task (device or host)
   switch(g_usb_mode)
   {
      case USB_MODE_DEVICE:
         usb_device_task();
         break;
      case USB_MODE_HOST:
         usb_host_task();
         break;
      case USB_MODE_UNDEFINED:  // No break !
      default:
         break;
  }
// -----------------------------------------------------------------------------

// ---- DEVICE ONLY USB MODE ---------------------------------------------------
#elif ((USB_DEVICE_FEATURE == ENABLED)&& (USB_HOST_FEATURE == DISABLE))
   usb_device_task();
// -----------------------------------------------------------------------------

// ---- REDUCED HOST ONLY USB MODE ---------------------------------------------
#elif ((USB_DEVICE_FEATURE == DISABLE)&& (USB_HOST_FEATURE == ENABLED))
   usb_host_task();
// -----------------------------------------------------------------------------

//! ---- ERROR, NO MODE ENABLED -------------------------------------------------
#elif ((USB_DEVICE_FEATURE == DISABLE)&& (USB_HOST_FEATURE == DISABLE))
   #error  at least one of USB_DEVICE_FEATURE or USB_HOST_FEATURE should be enabled
   #error  otherwise the usb task has nothing to do ...
#endif
// -----------------------------------------------------------------------------
}

//! @brief USB interrupt subroutine
//!
//! This function is called each time a USB interrupt occurs.
//! The following USB DEVICE events are taken in charge:
//! - VBus On / Off
//! - Start Of Frame
//! - Suspend
//! - Wake-Up
//! - Resume
//! - Reset
//! - Start of frame
//!
//! The following USB HOST events are taken in charge:
//! - Device connection
//! - Device Disconnection
//! - Start Of Frame
//! - ID pin change
//! - SOF (or Keep alive in low speed) sent
//! - Wake up on USB line detected
//!
//! For each event, the user can launch an action by completing
//! the associate define (See conf_usb.h file to add action upon events)
//!
//! Note: Only interrupts events that are enabled are processed
//!
#ifdef AVRGCC
 ISR(USB_GEN_vect)
#else
#pragma vector = USB_GENERAL_vect
__interrupt void usb_general_interrupt()
#endif
{
   #if (USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE)
   U8 i;
   U8 save_pipe_nb;
   #endif
// ---------- DEVICE events management -----------------------------------
#if (USB_DEVICE_FEATURE == ENABLED)
  //- VBUS state detection
   if (Is_usb_vbus_transition() && Is_usb_vbus_interrupt_enabled())
   {
      Usb_ack_vbus_transition();
      if (Is_usb_vbus_high())
      {
         usb_connected = TRUE;
         Usb_vbus_on_action();
         Usb_send_event(EVT_USB_POWERED);
			Usb_enable_reset_interrupt();
         usb_start_device();
			Usb_attach();
      }
      else
      {
         Usb_vbus_off_action();
         usb_connected = FALSE;
         usb_configuration_nb = 0;
         Usb_send_event(EVT_USB_UNPOWERED);
      }
   }
  // - Device start of frame received
   if (Is_usb_sof() && Is_sof_interrupt_enabled())
   {
      Usb_ack_sof();
      Usb_sof_action();
   }
  // - Device Suspend event (no more USB activity detected)
   if (Is_usb_suspend() && Is_suspend_interrupt_enabled())
   {
      Usb_ack_suspend();
      Usb_enable_wake_up_interrupt();
      Usb_ack_wake_up();                 // clear wake up to detect next event
      Usb_freeze_clock();
      Usb_send_event(EVT_USB_SUSPEND);
      Usb_suspend_action();
   }
  // - Wake up event (USB activity detected): Used to resume
   if (Is_usb_wake_up() && Is_swake_up_interrupt_enabled())
   {
      Usb_unfreeze_clock();
      Usb_ack_wake_up();
      Usb_disable_wake_up_interrupt();
      Usb_wake_up_action();
      Usb_send_event(EVT_USB_WAKE_UP);
   }
  // - Resume state bus detection
   if (Is_usb_resume() && Is_resume_interrupt_enabled())
   {
      Usb_disable_wake_up_interrupt();
      Usb_ack_resume();
      Usb_disable_resume_interrupt();
      Usb_resume_action();
      Usb_send_event(EVT_USB_RESUME);
   }
  // - USB bus reset detection
   if (Is_usb_reset()&& Is_reset_interrupt_enabled())
   {
      Usb_ack_reset();
      usb_init_device();
      Usb_reset_action();
      Usb_send_event(EVT_USB_RESET);
   }
#endif// End DEVICE FEATURE MODE

// ---------- HOST events management -----------------------------------
#if (USB_HOST_FEATURE == ENABLED && USB_DEVICE_FEATURE == ENABLED)
  // - ID pin change detection
   if(Is_usb_id_transition()&&Is_usb_id_interrupt_enabled())
   {
      if(Is_usb_id_device())
      { g_usb_mode=USB_MODE_DEVICE;}
      else
      { g_usb_mode=USB_MODE_HOST;}
      Usb_ack_id_transition();
      if( g_usb_mode != g_old_usb_mode) // Basic Debounce
      {
         if(Is_usb_id_device()) // Going to device mode
         {
            Usb_send_event(EVT_USB_DEVICE_FUNCTION);
         }
         else                   // Going to host mode
         {
            Usb_send_event(EVT_USB_HOST_FUNCTION);
         }
         Usb_id_transition_action();
         LOG_STR_CODE(log_id_change);
         #if ( ID_PIN_CHANGE_GENERATE_RESET == ENABLE)
        // Hot ID transition generates wdt reset
            #ifndef  AVRGCC
               Wdt_change_16ms(); while(1);
            #else
               Wdt_change_enable(); while(1);
            #endif
         #endif
      }
   }
#endif
#if (USB_HOST_FEATURE == ENABLED)
  // - The device has been disconnected
   if(Is_device_disconnection() && Is_host_device_disconnection_interrupt_enabled())
   {
      host_disable_all_pipe();
      Host_ack_device_disconnection();
      device_state=DEVICE_DISCONNECTED;
      Usb_send_event(EVT_HOST_DISCONNECTION);
      LOG_STR_CODE(log_device_disconnect);
      Host_device_disconnection_action();
   }
  // - Device connection
   if(Is_device_connection() && Is_host_device_connection_interrupt_enabled())
   {
      Host_ack_device_connection();
      host_disable_all_pipe();
      Host_device_connection_action();
   }
  // - Host Start of frame has been sent
   if (Is_host_sof() && Is_host_sof_interrupt_enabled())
   {
      Host_ack_sof();
      Usb_send_event(EVT_HOST_SOF);
      private_sof_counter++;

      // delay timeout management for interrupt tranfer mode in host mode
      #if ((USB_HOST_PIPE_INTERRUPT_TRANSFER==ENABLE) && (TIMEOUT_DELAY_ENABLE==ENABLE))
      if (private_sof_counter>=250)   // Count 1/4 sec
      {
         private_sof_counter=0;
         for(i=0;i<MAX_EP_NB;i++)
         {
            if(it_pipe_str[i].enable==ENABLE)
            {
               save_pipe_nb=Host_get_selected_pipe();
               Host_select_pipe(i);
               if((++it_pipe_str[i].timeout>TIMEOUT_DELAY) && (Host_get_pipe_type()!=TYPE_INTERRUPT))
               {
                  it_pipe_str[i].enable=DISABLE;
                  it_pipe_str[i].status=PIPE_DELAY_TIMEOUT;
                  Host_stop_pipe_interrupt(i);
                  if (is_any_interrupt_pipe_active()==FALSE)    // If no more transfer is armed
                  {
                     if (g_sav_int_sof_enable==FALSE)
                     {
                        Host_disable_sof_interrupt();
                     }
                  }
                  it_pipe_str[i].handle(PIPE_DELAY_TIMEOUT,it_pipe_str[i].nb_byte_processed);
               }
               Host_select_pipe(save_pipe_nb);
            }
         }
      }
      #endif  // (USB_HOST_PIPE_INTERRUPT_TRANSFER==ENABLE) && (TIMEOUT_DELAY_ENABLE==ENABLE))
      Host_sof_action();
   }
  // - Host Wake-up has been received
   if (Is_host_hwup() && Is_host_hwup_interrupt_enabled())
   {
      Host_disable_hwup_interrupt();  // Wake up interrupt should be disable host is now wake up !
      // CAUTION HWUP can be cleared only when USB clock is active (not frozen)!
      Pll_start_auto();               // First Restart the PLL for USB operation
      Wait_pll_ready();               // Get sure pll is lock
      Usb_unfreeze_clock();           // Enable clock on USB interface
      Host_ack_hwup();                // Clear HWUP interrupt flag
      Usb_send_event(EVT_HOST_HWUP);  // Send software event
      Host_hwup_action();             // Map custom action
   }
#endif // End HOST FEATURE MODE
}

/**
    @brief Spare function to handle sleep mode.
*/
extern void suspend_action(void)
{
   Enable_interrupt();
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
   sleep_mode();
}
/**
    @brief Spare function to handle power down mode.
*/
extern void host_suspend_action(void)
{
   //Enter_power_down_mode();  //For example...
}
/** @}  */
