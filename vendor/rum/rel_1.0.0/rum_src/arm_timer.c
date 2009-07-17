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
  $Id: arm_timer.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#include "arm_timer.h"
#include "hal_arm.h"
#include "mac_event.h"
#include "arm_timer_event.h"

/**
    @addtogroup arm

    @{
 */

/*============================ MACROS & DEFINES =============================*/

typedef struct
{
    u16 time;                  ///< Ticks left before timer expires.
    u8 ID;
    void (*callback) (void);   ///< Function to call when timer expires.
} timerEventT;

#define TIMER_EVENTS_MAX 30
#define LONG_TIMER_EVENTS_MAX 3

static timerEventT timerEvents[TIMER_EVENTS_MAX];

#define MS_PER_TICK (1)   ///< Milliseconds per tick of the timer.

static u8 getUniqueID(void);

/*============================ IMPLEMENTATION ===============================*/

/**
   @name ARM Timer Functions
   @{
*/

/**
   @brief Sets a general purpose timer.  Can be called by application.
   The callback will not be called from the timer ISR, so it is
   thread-safe.

   @param time Number of milli-seconds to wait before calling the
   callback function.

   @param callback Pointer to a function to call back after the specified time.

   @return timerEvents.ID The newly created timer ID.
*/
u8 macSetAlarm_arm(u16 time, void(*callback)(void))
{
    u8 i;
    u32 ticks;
    volatile u8 empty_timer = false;

    // Search for free event structure
    for (i=0;i<TIMER_EVENTS_MAX;i++)
    {
        if (!timerEvents[i].time)
        {
            // Prevent a default re-use of last timer slot.
            empty_timer = true;

            // Found a free one, use this one
            break;
        }
    }


    if (!empty_timer)
    {
        // Out of timers to use, just quit
        return 0;
    }

    // Store the timer details in the array
    ticks = ((u32)time)/MS_PER_TICK;

    if (!ticks)
        // make sure we have at least one tick
        ticks++;
    timerEvents[i].time = ticks;
    timerEvents[i].callback = callback;
    // Don't return zero as a timer ID
    timerEvents[i].ID = getUniqueID();

    // Leave timer on all the time. This also gives a heartbeat on the SAM7X-EK board.

    return timerEvents[i].ID;
}

/**
   @brief Returns a unique timer ID.

   @return The unique ID.
*/
static u8 getUniqueID(void)
{
    static u8 currentTimerID;
    u8 i,goodID;

    currentTimerID++;
    for(;;)
    {
        // Don't return zero as an ID, macSetAlarm returns zero on error
        if (!currentTimerID)
            currentTimerID++;

        // See if this ID is used by any active timer
        goodID = 1;
        for (i=0;i<TIMER_EVENTS_MAX;i++)
            if (timerEvents[i].ID == currentTimerID)
                goodID = 0;

        if (goodID)
            // Unused ID, use this one
            return currentTimerID;
        // ID already exists, try again skippy
        currentTimerID++;
    }
}

/**
   @brief End the timer specified by ID, before it is expired.
   Calling after expiration will not cause any problem.

   @param ID The value returned from macSetAlarm when the alarm was set.
*/
void macTimerEnd_arm(u8 ID)
{
    u8 i;

    // Search for timer with ID
    for (i=0;i<TIMER_EVENTS_MAX;i++)
    {
        if (timerEvents[i].ID == ID)
        {
            // Stop this timer
            timerEvents[i].time = 0;
            return;
        }
    }
}

/**
    @brief  The ARM timer initialization procedure requires that the master
    clock be turned on then the PIO set up then the timer setup. Timer
    setup requires the "frequency" of the interrupt to be set, the
    interrupt enabled and, finally, the AIC setup - The AIC setup
    includes loading the ISR function pointer (vector).
*/
void timerInit_arm(void)
{
    u32 val = 0;

    /*********** SETUP PMC **********/

    /* Turn on the Master Clock for the peripheral (TC1 = 13). */
    PMC_PCER = TC1;

    /*********** SETUP DEBUG PIO PIN TOGGLE - Interferes wit uTasker **********/

    // TIOB1 is connected to PB26, turn it on for output to see if we can
    // see the toggling.
    //    PIO_OER_B = PB26;

    // Disable PIO control of the line and allow peripheral control.
    //    PIO_PDR_B = PB26;

    // Assign IO control to peripheral A function of the PIOA Controller.
    //    PIO_ASR_B = PB26;

    /*********** SETUP TC1 **********/

    // Disable the timer/counter by setting the TC_CLKDIS bit in the
    // channel control register.
    TC1_CCR = TC_CLKDIS;

    // Reading the status register will clear any pending interrupt.
    val = TC1_SR;

    // In the channel mode register we select the main clock divided by
    // 2 as our clock source. The value of the counter will be
    // incremented at a positive edge from the clock source. Furthermore
    // we enable the compare mode trigger, which will reset the counter
    // when it reaches the value in the compare register.
    // Finally we select the waveform mode and toggle the output.
    //TC1_CMR = (TC_CLKS_MCK2 | TC_CPCTRG | TC_WAVE | TC_EEVT_XC1 | TC_BCPC_TOGGLE_OUTPUT);
    TC1_CMR = (TC_CLKS_MCK2 | TC_CPCTRG |TC_WAVE);

    // Implement a 1 ms interval. Note: uTasker MASTER_CLOCK has value
    // (in Hz) of the PLL output.
    TC1_RC = 0x5DE8; // decimal 24040 - empirical...

    // Enable RC compare interrupts by setting the related bit in the
    // interrupt enable register.
    TC1_IER = TC_CPCS;

    /*********** SETUP AIC **********/

    // Set the vector. Cast to int to eliminate compiler warning.
    (AIC_SVR(TC1_ID)) = (u32)ApiTimerIntr;

    // Initialize to edge triggered with defined priority.
    (AIC_SMR(TC1_ID)) = (AIC_SRCTYPE_INT_EDGE_TRIGGERED | 3);

    return;
}


/**
    @brief  After timer initialization this function will start/restart the
    timer and clear it...
*/
void timerStart_arm(void)
{
    u32 val = 0;

    // Reading the status register will clear any pending interrupt.
    val = TC1_SR;

    // Clear interrupt.
    AIC_ICCR = PID13;

    // Disable the timer/counter by setting the TC_CLKDIS bit in the
    // channel control register.
    TC1_CCR = TC_CLKDIS;

    // Enable the timer.
    TC1_CCR = TC_CLKEN;

    // Enable the interrupt.
    AIC_IECR =  PID13;

    // Start the timer.
    TC1_CCR = TC_SWTRG;
}

volatile int val;
u8 i;
timer_event_object_t event;
/**
    @brief  This is the timer ISR. When the timer interrupt occurs this
            function is called via the function pointer (vector) previously
            loaded.
*/
void ApiTimerIntr(void)
{
    // Reading the status register will clear any pending interrupt.
    val = TC1_SR;

    // Disable timer isr's.
    TC1_IDR = TC_CPCS;

    // Send the address to read.
    fnInUserInterrupt(1); // entering;

    // check for pending events
    for (i=0;i<TIMER_EVENTS_MAX;i++)
    {
        if (timerEvents[i].time)
        {
            // this timer event is live, check for expiration
            if (!--timerEvents[i].time)
            {
                // Timer expired, queue the associated callback
                event.event = TIMER_EVENT_TIMER;  // Event type, see event_t for details.
                event.data = (u8*)timerEvents[i].callback;
                timer_put_event(&event);
            }
        }
    }

    fnInUserInterrupt(0); // leaving

    // Enable timer isr's.
    TC1_IER = TC_CPCS;
}

/** @} */
/** @} */
