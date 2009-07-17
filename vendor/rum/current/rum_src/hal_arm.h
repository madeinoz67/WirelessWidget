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
/*
  $Id: hal_arm.h,v 1.2 2009/05/21 20:13:24 mvidales Exp $
*/

#ifndef HAL_ARM_H
#define HAL_ARM_H

// Utasker
#include "config.h"

#include "rum_types.h"

#define NODETYPE COORD

// Define the channel that the PAN coordinator uses
#define BAND900    1
#define BAND2400   2

#ifndef BAND
  #define BAND        BAND2400
#endif // BAND

#if BAND == BAND2400
#include "at86rf23x_registermap.h"
#else
#include "at86rf212_registermap.h"
#endif

// Define platforms so that we can compile out the non-ARM platforms
#define RCB231   1
#define RCB230   2
#define RCB212   3
#define RCBSIP   4
#define RAVEN    5
#define SPITFIRE 6
#define RAVENUSB 7
#define ZIGBIT9  8
#define ZIGBIT24 9
#define DSK001   10

#define PLATFORM  0  // This eliminates any non-ARM platforms from being used.

/// @name These macros convert the platform-specific pin defines into names and functions
///       that the source code can directly use.
/// @{
    #   define RSTPIN     (PA09)    ///< Pin number that corresponds to the RST pin.
    #   define IRQPIN     (PA30)    ///< Pin number that corresponds to the IRQ pin.
    #   define SLPTRPIN   (PA08)    ///< Pin number(PA8) that corresponds to the SLP_TR pin.
/// @}

/// @name Macros for radio operation.
/// @{
#define HAL_PLL_LOCK_MASK      ( 0x01 ) //!< Mask for the PLL_LOCK interrupt.
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) //!< Mask for the PLL_UNLOCK interrupt.
#define HAL_RX_START_MASK      ( 0x04 ) //!< Mask for the RX_START interrupt.
#define HAL_TRX_END_MASK       ( 0x08 ) //!< Mask for the TRX_END interrupt.
#define HAL_ED_READY_MASK      ( 0x10 ) //!< Mask for the ED_READY interrupt.
#define HAL_TRX_UR_MASK        ( 0x40 ) //!< Mask for the TRX_UR interrupt.
#define HAL_BAT_LOW_MASK       ( 0x80 ) //!< Mask for the BAT_LOW interrupt.

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) //!< A frame should be at least 3 bytes.
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) //!< A frame should no more than 127 bytes.
/// @}

// Equivalent delay function built into uTasker OS. Measured accuracy is not
// too good. a 1 us entry gives about a 5 us actual delay. Higher delay's
// (>10 us) appear to be off by +10%.
void delay_us(int us);

// ------------ FOR UAB/UART COnnections ---------------
//extern QUEUE_HANDLE  current_interface_handle;  // Defined in application.c

/*============================ MACROS & DEFINES =============================*/

#define macSetLongAlarm(x,y) 0

//---------- Macros ---------------

void ledOn(u8 val);
void ledOff(u8 val);
#define LED1ON()                PIO_OER_B = PB19,    PIO_CODR_B = PB19
#define LED1OFF()               PIO_OER_B = PB19,    PIO_SODR_B = PB19
#define LED2ON()                PIO_OER_B = PB20,    PIO_CODR_B = PB20
#define LED2OFF()               PIO_OER_B = PB20,    PIO_SODR_B = PB20
#define LED3ON()                PIO_OER_B = PB21,    PIO_CODR_B = PB21
#define LED3OFF()               PIO_OER_B = PB21,    PIO_SODR_B = PB21
#define LED4ON()                PIO_OER_B = PB22,    PIO_CODR_B = PB22
#define LED4OFF()               PIO_OER_B = PB22,    PIO_SODR_B = PB22
#define LED_ON(led)             ledOn(led + 2)
#define LED_OFF(led)            ledOff(led + 2)
#define BUTTON_SETUP()          // Don't use Joystick for button. Shares same GPIO(SPI1) as radio.
#define BUTTON_PRESSED()        0


// ---------- RS232 Macros -----------
#define serial_putspaces(n)
#define debugMsgStr(str)        fnDebugMsg(str)
#define debugMsgInt(i)          fnDebugDec(i,0,0)
#define debugMsgFlt(x)
#define debugMsgHex(x)          fnDebugHex(x, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(x)))
#define debugMsgChr(c)          debugStr[0] = c, debugStr[1] = 0, fnDebugMsg(debugStr)
#define debugMsgCrLf()          fnDebugMsg("\r\n")

#define TIMER_INIT()            timerInit()
#define TIMER_CLEAR()
#define TIMER_ENABLE()
#define TIMER_STOP()            timerStop()

#define HAL_STOP_ADC()

// ---------- Handy SPI Macros ----------
// Both SPI0 and SPI1 are on PIOA.
#define SPI_PCS_LSB             16  /*!< \brief Least significant bit of peripheral chip select. */
#define SPI_SCBR_LSB            8   /*!< \brief Least significant bit of serial clock baud rate. */
#define SPI_RF_PINS             ((SPI_MISO_1B) | (SPI_MOSI_1B) | (SPI_CLK_1B))
#define SPI_RF_CS_MR            (0xE)
#define AT91C_AIC_SRCTYPE_EXT_POSITIVE_EDGE    (0x3 <<  5) // (AIC) External Sources Code Label Positive Edge triggered

// ---------- SPI Macros ----------
#define hal_set_slptr_high( )   (PIO_SODR_A = SLPTRPIN) /// < This macro pulls the SLP_TR pin high.
#define hal_set_slptr_low( )    (PIO_CODR_A = SLPTRPIN) ///< This macro pulls the SLP_TR pin low.
#define hal_get_slptr( )        (PIO_PDSR_A & SLPTRPIN) //!PROBLEM - SLPTR IS AN OUTPUT!               ///< Read current state of the SLP_TR pin (High/Low).
#define hal_set_rst_high( )     (PIO_SODR_A = RSTPIN)   ///< This macro pulls the RST pin high.
#define hal_set_rst_low( )      (PIO_CODR_A = RSTPIN)   ///< This macro pulls the RST pin low.


//---------- Prototypes ---------------

// Support HTML ping request.
volatile extern u8 gotPanCoordPing;

// ---------- Low level SAM7X SPI functions ----------
// For radio interface and initialization.
void hal_init(void);
u8 spiReadWrite(u8 val);
u8 hal_register_read(u8 address);
void hal_register_write(u8 address, u8 value);
u8 hal_subregister_read(u8 address, u8 mask, u8 position);
void hal_subregister_write(u8 address, u8 mask, u8 position, u8 value);
void hal_frame_read(void);
void hal_frame_write(u8 *write_buffer, u8 length);

void halGetMacAddr(u8 *p);

// Macros used to ensure compatibility with ARM code.
// ARM is a 32-bit machine, so it needs to declare a
// temporary variable, which is guaranted to be on a
// 32-bit boundary.  The AVR does not need to declare the
// temporary variable, which saves flash space in the
// AVR platforms.
#define  DECLARE64(x) u64 ad = (x)
#define  USE64(x)   (&ad)

// AVR functions that are nulled here.
#define halPutEeprom(offset, size, srcAddr)
#define halGetEeprom(offset, size, dstAddr)
#define calibrate_rc_osc(x)
#define LED_INIT()
#define HAL_INIT_ADC()
#define HAL_SAMPLE_ADC()
#define HAL_WAIT_ADC()
#define HAL_READ_ADC() 0

#endif //HAL_ARM_H
