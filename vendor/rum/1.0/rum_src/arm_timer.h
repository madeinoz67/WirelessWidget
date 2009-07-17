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
  $Id: arm_timer.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
*/

#ifndef ARM_TIMER_H
#define ARM_TIMER_H

#include "rum_types.h"

/**
 	@addtogroup arm

 	@{
*/

//------------------------ SAM7X ----------------------------------------------
#define TC_BASE         0xFFFA0000          /**< \brief Timer/counter base address. */
#define TC1_ID          13                  /*!< \brief Timer 1 ID. */
#define PID13          	0x00002000          /*!< \brief Timer 1 ID. */
#define TC1_CCR         *(volatile unsigned long*)(TC_BASE + 0x40)    /*!< \brief Channel 1 control register address. */
#define TC_CLKDIS       0x00000002          /*!< \brief Clock disable command. */
#define TC1_CMR         *(volatile unsigned long*)(TC_BASE + 0x44)    /*!< \brief Channel 1 mode register address. */
#define TC_CLKS         0x00000007          /*!< \brief Clock selection mask. */
#define TC_CLKS_MCK2    0x00000000          /*!< \brief Selects MCK / 2. */
//#define TC_CLKS_MCK8    0x00000001          /*!< \brief Selects MCK / 8. */
//#define TC_CLKS_MCK32   0x00000002          /*!< \brief Selects MCK / 32. */
//#define TC_CLKS_MCK128  0x00000003          /*!< \brief Selects MCK / 128. */
//#define TC_CLKS_MCK1024 0x00000004          /*!< \brief Selects MCK / 1024. */
//#define TC_CLKS_XC0     0x00000005          /*!< \brief Selects external clock 0. */
//#define TC_CLKS_XC1     0x00000006          /*!< \brief Selects external clock 1. */
//#define TC_CLKS_XC2     0x00000007          /*!< \brief Selects external clock 2. */
//#define TC_CLKI         0x00000008          /*!< \brief Increments on falling edge. */
//#define TC_BURST        0x00000030          /*!< \brief Burst signal selection mask. */
//#define TC_BURST_NONE   0x00000000          /*!< \brief Clock is not gated by an external signal. */
//#define TC_BUSRT_XC0    0x00000010          /*!< \brief ANDed with external clock 0. */
//#define TC_BURST_XC1    0x00000020          /*!< \brief ANDed with external clock 1. */
//#define TC_BURST_XC2    0x00000030          /*!< \brief ANDed with external clock 2. */
#define TC_CPCTRG       0x00004000          /*!< \brief RC Compare Enable Trigger Enable. */
#define TC_WAVE         0x00008000          /*!< \brief Selects waveform mode. */
//#define TC_CAPT         0x00000000          /*!< \brief Selects capture mode. */
//#define TC_EEVT         0x00000C00          /*!< \brief External event selection mask. */
//#define TC_EEVT_TIOB    0x00000000          /*!< \brief TIOB selected as external event. */
//#define TC_EEVT_XC0     0x00000400          /*!< \brief XC0 selected as external event. */
#define TC_EEVT_XC1     0x00000800          /*!< \brief XC1 selected as external event. */
//#define TC_EEVT_XC2     0x00000C00          /*!< \brief XC2 selected as external event. */
#define TC_BCPC_TOGGLE_OUTPUT   0x0C000000      /*!< \brief RC compare toggles TIOB. */
#define TC1_IER         *(volatile unsigned long*)(TC_BASE + 0x64)    /*!< \brief Channel 1 interrupt enable register address. */
#define TC1_IDR         *(volatile unsigned long*)(TC_BASE + 0x68)    /*!< \brief Channel 1 interrupt disable register address. */
#define TC_CPCS         0x00000010          /*!< \brief RC compare flag. */
#define TC1_SR          *(volatile unsigned long*)(TC_BASE + 0x60)        /*!< \brief Status register address. */
#define TC1_RC          *(volatile unsigned long*)(TC_BASE + 0x5C)        /*!< \brief Channel 1 register C. */
#define TC_CLKEN        0x00000001      	/*!< \brief Clock enable command. */
#define TC_SWTRG        0x00000004      	/*!< \brief Software trigger command. */


#define AIC_BASE        0xFFFFF000          /*!< \brief AIC base address. */
#define AIC_SVR(i)      (*(volatile unsigned long*)(AIC_BASE + 0x80 + i * 4))
#define AIC_SMR(i)      (*(volatile unsigned long*)(AIC_BASE + i * 4))

#define AIC_SRCTYPE_INT_EDGE_TRIGGERED  0x00000020      /*!< \brief Internal edge triggered. */
/**  @} */

void ApiTimerIntr(void);
void timerInit_arm(void);
void timerStart_arm(void);
u8 macSetAlarm_arm(u16 microseconds, void(*callback)(void));
void macTimerEnd_arm(u8 ID);

// Definitions for ARM target to compile (and for Doxygen to not have conflicts)
#define timerInit   timerInit_arm
#define timerStart  timerStart_arm
#define macSetAlarm macSetAlarm_arm
#define macTimerEnd macTimerEnd_arm

#endif //ARM_TIMER_H
