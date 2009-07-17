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
  $Id: arm_timer_event.c,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/


// Includes
#include <stdlib.h>
#include "mac.h"
#include "mac_data.h"
#include "mac_scan.h"
#include "mac_associate.h"
#include "radio.h"
#include "stdio.h"
#include "system.h"
#include "arm_timer_event.h"

// Globals

#define MAX_TIMER_EVENTS 30
struct {
    volatile u8 head;
    volatile u8 tail;
    timer_event_object_t timer_event_object[MAX_TIMER_EVENTS];
} /*__attribute__((packed))*/ timer_event_queue;

// Implementation

/**
    @brief Checks for any pending events in the queue.

    @return True if there is a pending event, else false.
*/
u8 timer_event_pending(void)
{
    return (timer_event_queue.head != timer_event_queue.tail);
}


/**
    @brief Puts an event into the queue of events.

    @param object is a pointer to the event to add to queue.

    @return flag Used to see if timer queue is full.
*/
u8 timer_put_event(timer_event_object_t *object)
{
    u8 newhead;

    if ((timer_event_queue.head + 1) % MAX_TIMER_EVENTS == timer_event_queue.tail)
    {
        fnDebugMsg("The timer queue is full, bailing out!");
        // Queue full, get outta here
        return 0;
    }

    newhead = timer_event_queue.head;

    // Store in queue
    timer_event_queue.timer_event_object[newhead] = *object;

    // Calculate new head index
    newhead++;
    if (newhead >= MAX_TIMER_EVENTS)
        newhead = 0;
    timer_event_queue.head = newhead;

    return 1;
}

/**
    @brief Pulls an event from the event queue.
    Assumes that there is an event in the queue.

    @return Pointer to the event object, or NULL in the event of empty queue.
*/
timer_event_object_t *timer_get_event(void)
{
    timer_event_object_t *object = NULL;
    volatile u8 newtail;

    newtail = timer_event_queue.tail;

    object = &(timer_event_queue.timer_event_object[newtail]);

    // Calculate new tail
    newtail++;
    if (newtail >= MAX_TIMER_EVENTS)
        newtail = 0;

    timer_event_queue.tail = newtail;

    return(object);
}

/**
   @brief This is the main loop task for the timer (TC1).  Called by the
   mac nut thread "Mac_Timer_Thread".
*/
void timerTask(void)
{
    timer_event_object_t *event;

    while(timer_event_pending())
    {
        event = timer_get_event();
            // Timer has expired, just callback the stored function pointer
            ((void (*)(void))event->data)();  // cast data to function pointer
    }
}
