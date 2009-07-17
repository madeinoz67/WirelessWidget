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

/*!
 * $Id: arm_app.h,v 1.1 2009/05/20 20:52:01 mvidales Exp $
 *
 */

#ifndef ARM_APP_H
#define ARM_APP_H

#include "rum_types.h"
#include "sensors.h"
// EFSL include
#include "efs.h"

// ---------- Telnet Support -------
typedef enum {TEL_FILE_NAME = 0x01,
              TEL_PING_NODE,
              TEL_CHANNEL,
              TEL_REQUEST_DATA_FROM_NODE,
              TEL_REPORT_INTERVAL,
              TEL_CAL_WHICH_NODE,
              TEL_CAL_POINT_1,
              TEL_CAL_POINT_2,
              TEL_NAME_WHICH_NODE,
              TEL_NEW_NODE_NAME,
              TEL_WAKE_NODE,
              TEL_NEW_IP,
              TEL_TOUCH,
              TEL_TOUCH_INTERVAL
} __attribute__((packed)) telType_t;

extern u16 ping_address;
extern telType_t telState;

#define LF  0x0A
#define CR  0x0D

// Turn on and off reading print out.
extern u8 telPrintReading;

// ---------- TCP Support ----------
#define TCP_PORT 6464
volatile USOCKET host_socket;
volatile USOCKET host_socket2;

#define TCP_BUFFER_LENGTH 10
typedef struct stTCP_MESSAGE
{
    TCP_HEADER      tTCP_Header;     // reserve header space
    unsigned char   ucTCP_Message[TCP_BUFFER_LENGTH];
} TCP_MESSAGE;

// ---------- SD Card Support ----------
extern File sd_file;
extern eint8 sd_filename[20];
extern u8 files_sys_active;

#define MAX_STORED_READINGS      (1)
typedef struct{
    u16   addr;          ///< Short address of node sending reading
    u8    c1;
    char  reading[7];    ///< Calculated sensor reading, as an ASCII string
    u8    c2;
    char  units[6];      ///< Units of sensor reading, as an ASCII string
    u8    c3;
    char  name[9];
    u8    c4;
    char  time[30];
    u8    c5;
} __attribute__((packed)) stored_readings_t;

void printPrompt(void);
void armAppInit(void);
void armCalReset(void);
CHAR *armAppBuildInfo(unsigned short *usLengthToSend);
void armAppPingReq(CHAR *ptrData);
void armAppPingHandler(unsigned char *ptrBuffer, unsigned short *usLengthToSend, CHAR *cValue, CHAR *cPtr);
CHAR *armAppNetworkTable(LENGTH_CHUNK_COUNT TxLength, unsigned short *usLengthToSend, CHAR *cValue, CHAR *cPtr, HTTP *http_session);
void rf2xxregdump(void);
void armAppRcvData(sftSensorReading *frame);
void get_nwk_table(USOCKET Socket);
int fnHandleTelnetInput(unsigned char *telBuf, int telBufLen, unsigned short  usTelnet_state);
int tcpListener(USOCKET Socket, unsigned char ucEvent, unsigned char *ucIp_Data, unsigned short usPortLen);
void tcp_parse_command(USOCKET Socket, u8 * data, int len);

#endif // ARM_APP_H
