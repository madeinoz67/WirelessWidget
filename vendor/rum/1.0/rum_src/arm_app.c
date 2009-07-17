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
 *  $Id: arm_app.c,v 1.2 2009/05/28 23:27:07 mvidales Exp $
 *
 */

#include <stdlib.h>
// Utasker include
#include "config.h"

#include "mac_event.h"
#include "arm_app.h"
#include "system.h"
#include "arm_timer.h"
#include "sensors.h"
#include "rum_types.h"
#include "mac_associate.h"
#include "radio.h"
// UIP include
#include "net/uip-netif.h"
// EFSL includes
#include "file.h"
#include "interfaces/sd.h"


// ---------- Local Macros and Globals ----------
File sd_file;
eint8 sd_filename[20];
u8 files_sys_active;

// Global stored ping address.
u16 ping_address = 0;
volatile unsigned char table_count;
unsigned char * nt;
volatile unsigned long previous_TxLength = 0;
const CHAR            cBuildInformation[] =
                                    {'B','A','N','D','=',
#if (BAND == BAND2400)
                                     '2','.','4','G','H','z',
#elif (BAND == BAND900)
                                     '9','0','0','M','H','z',
#else
                                     '?',
#endif
                                     ' ','S','L','E','E','P','=',
#if (RUMSLEEP)
                                     '1',
#else
                                     '0',
#endif
                                    ' ', 'H','W','=',
#ifdef SAM7X_EVAL
                                    'A','T','9','1','S','A','M','7','X','-','E','K',
#else
                                    '?',
#endif
                                    0};

// Turn on and off reading print out.
u8 telPrintReading = 0;

stored_readings_t stored_data[MAX_STORED_READINGS];

/**
    @brief This will show the RUM build information on the webserver screen. Various
    parameters will be displayed for development evaluation.

    @param usLengthToSend Pointer to length of buffered data to be sent.

    @return cBuildInformation Pointer to build data.
*/
CHAR *armAppBuildInfo(unsigned short *usLengthToSend)
{
    *usLengthToSend = (sizeof(cBuildInformation)-1);
    return (CHAR *)cBuildInformation;
}

/**
    @brief When the user requests a PING from the webserver, this function will excute
    the PING operation.

    @param ptrData Pointer to data received from HTTP request (PING address).
*/
void armAppPingReq(CHAR *ptrData)
{
    // This is a ping command.
    if(*ptrData == '1')
    {
        // Check that the field is not empty.
        if(*(ptrData + 2) != ' ')
        {
            ping_address = (unsigned short)fnHexStrHex(ptrData + 2);
            macPing(PING_REQ_FRAME, ping_address);
        }
    }
}

/**
    @brief The webserver network page needs updated before/after PING attempts. This
    function will dynamically update the PING response field.

    @param ptrBuffer Pointer to dynamic data positioning (html parsing character + value)
    @param usLengthToSend Length of data to return to html page.
    @param cValue Pointer to previous appended data.
    @param cPtr New data to be appended to previous http response frame.
*/
void armAppPingHandler(unsigned char *ptrBuffer, unsigned short *usLengthToSend, CHAR *cValue, CHAR *cPtr)
{
    switch (*ptrBuffer)
    {
        case '1':
            if(ping_address == 0)
                cPtr = (uStrcpy(cValue, "") - 1);
            else
                cPtr = (fnBufferHex(ping_address, (2 | NO_LEADIN | CODE_CAPITALS), cValue) - 1);

            *usLengthToSend = (cPtr - cValue);
        break;

        case '2':
            if(ping_address == 0)
                cPtr = (uStrcpy(cValue, "") - 1);
            else
                cPtr = (fnBufferHex(ping_address, (2 | WITH_LEADIN | CODE_CAPITALS), cValue) - 1);

            *usLengthToSend = (cPtr - cValue);
        break;

        case '3':
            macTask();
            if(ping_address == 0)
                cPtr = (uStrcpy(cValue, "") - 1);
            else if(gotPanCoordPing == 1)
            {
                cPtr = (uStrcpy(cValue, "Ping Success") - 1);
                gotPanCoordPing = 0;
            }
            else if(gotPanCoordPing == 0)
                cPtr = (uStrcpy(cValue, "Ping Failed") - 1);

            *usLengthToSend = (cPtr - cValue);
        break;

        default:
        break;
    }
}

/**
    @brief This function will update the RUM network table whenever a webpage is refreshed. Dynamic table
    data content is based on the Coordinator association table.

    @param TxLength The current length of the HTTP response frame
    @param usLengthToSend Length of data to return to the webpage.
    @param cValue Pointer to previous appended data.
    @param cPtr New data to be appended to previous http response frame.
    @param http_session Contains current HTTP status flags.

    @return cValue The address of the newly created table data.
*/
CHAR *armAppNetworkTable(LENGTH_CHUNK_COUNT TxLength, unsigned short *usLengthToSend, CHAR *cValue, CHAR *cPtr, HTTP *http_session)
{
    associatedNodes_t *node;
#if IPV6LOWPAN
    char ipv6_addr[60]; /* Store IPv6 Address: worst-case size */
#endif // IPV6LOWPAN

    // Check if we need to regenerate un-transmitted data.
    if(previous_TxLength == TxLength)
    {
        table_count--;
    }

    // Store last chunk TXLength.
    previous_TxLength = TxLength;

    if(table_count == 0)
    {
        http_session->ucDynamicFlags |= GENERATING_DYNAMIC_BINARY;   // ensure the HTTP server knows that we are generating binary content
        cPtr = (uStrcpy(cValue, "<TABLE cellspacing=\"5\" bgcolor=\"#FFFFFF\" border=\"border\" align=\"center\"> \
        <TR><TD align=\"center\" style=\"width: 150px\">Node Short Address</TD><TD \
        align=\"center\" style=\"width: 114px\">Node Type</TD><TD align=\"center\" style=\"width: 220px\" \
        >Node Long Address</TD><TD align=\"center\" style=\"width: 163px\">Parent Short Address</TD><TD  \
        align=\"center\" style=\"width: 162px\">Last Routed Address</TD><TD align=\"center\" \
        style=\"width: 220px\">IP Address</TD><TD align=\"center\" style=\"width: 111px\">Node Data</TD></TR>") - 1);
        *usLengthToSend = (cPtr - cValue);
        table_count++;
    }
    else if(table_count == 1)
    {
        http_session->ucDynamicFlags |= GENERATING_DYNAMIC_BINARY;   // ensure the HTTP server knows that we are generating binary content
        cPtr = (uStrcpy(cValue, "<TR><TD align=\"center\" style=\"width: 150px\">") - 1);
        cPtr = (fnBufferHex((table_count-1), (2 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
        cPtr = (uStrcpy(cPtr, "</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 114px\">COORD</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 220px\">") - 1);
        u32 low = (u32)DEFAULT_ARM_COORD_LONG;
        u32 high = (u32)(DEFAULT_ARM_COORD_LONG >> 32);
        cPtr = (fnBufferHex(high, (4 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
        cPtr = (fnBufferHex(low, (4 | CODE_CAPITALS), cPtr) - 1);
        cPtr = (uStrcpy(cPtr, "</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 163px\">0xFFFF</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 162px\">0xFFFF</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 220px\">") -1);
        cPtr = fnIPStr(&network.ucOurIP[0], cPtr);
#if IPV6LOWPAN
        if(uip_netif_physical_if[INTERFACE_ETHERNET].addresses[1].state != NOT_USED)
        {
            sprint_ip(ipv6_addr, uip_netif_physical_if[INTERFACE_ETHERNET].addresses[1].ipaddr.u16);
            cPtr = (uStrcpy(cPtr, "<br>IPv6 Ethernet IF<br>") - 1);
            cPtr = (uStrcpy(cPtr, ipv6_addr) - 1);
            sprint_ip(ipv6_addr, uip_netif_physical_if[INTERFACE_802154].addresses[1].ipaddr.u16);
            cPtr = (uStrcpy(cPtr, "<br>IPv6 6LoWPAN IF<br>") - 1);
            cPtr = (uStrcpy(cPtr, ipv6_addr) - 1);
        }
#endif // IPV6LOWPAN
        cPtr = (uStrcpy(cPtr, "</TD>") - 1);
        cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 111px\">N/A</TD></TR>") - 1);
        *usLengthToSend = (cPtr - cValue);
        table_count++;
    }
    else if((table_count-1) < MAXNODES)
    {
        http_session->ucDynamicFlags |= GENERATING_DYNAMIC_BINARY;   // ensure the HTTP server knows that we are generating binary content
        // Start at index 1.
        node = macGetNode(table_count-1);
        if (node->nodeType)
        {
            cPtr = (uStrcpy(cValue, "<TR><TD align=\"center\" style=\"width: 150px\">") - 1);
            cPtr = (fnBufferHex((table_count-1), (2 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
            cPtr = (uStrcpy(cPtr, "</TD>") - 1);
            if(node->nodeType == ENDDEVICE)
                cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 114px\">ENDNODE</TD>") - 1);
            else
                cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 114px\">ROUTER</TD>") - 1);
            cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 220px\">") - 1);
            u32 low = (u32)node->nodeLongAddress;
            u32 high = (u32)(node->nodeLongAddress >> 32);
            cPtr = (fnBufferHex(high, (4 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
            cPtr = (fnBufferHex(low, (4 | CODE_CAPITALS), cPtr) - 1);
            cPtr = (uStrcpy(cPtr, "</TD>") - 1);
            cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 163px\">") - 1);
            cPtr = (fnBufferHex((node->parentShortAddress), (2 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
            cPtr = (uStrcpy(cPtr, "</TD>") - 1);
            cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 162px\">") - 1);
            cPtr = (fnBufferHex((node->lastRoutedAddress), (2 | WITH_LEADIN | CODE_CAPITALS), cPtr) - 1);
            cPtr = (uStrcpy(cPtr, "</TD>") - 1);
            cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 220px\">") -1);
#if IPV6LOWPAN
            if (ipv6_lookup_addr(ipv6_addr, (table_count-1)))
            {
                cPtr = (uStrcpy(cPtr, ipv6_addr) - 1);
            }
            else // Copy new IPv6 address into buffer.
#endif // IPV6LOWPAN
            {
                cPtr = (uStrcpy(cPtr, "Unknown") - 1);
            }
            cPtr = (uStrcpy(cPtr, "</TD>") - 1);
            cPtr = (uStrcpy(cPtr, "<TD align=\"center\" style=\"width: 111px\">N/A</TD></TR>") - 1);

            *usLengthToSend = (cPtr - cValue);
            table_count++;
        }
        else
        {
            *usLengthToSend = 0;
            table_count++;
        }

    }
    else if((table_count-1) == MAXNODES)
    {
        http_session->ucDynamicFlags |= LAST_DYNAMIC_CONTENT_DATA;
        cPtr = uStrcpy(cValue, "</TABLE>");
        *usLengthToSend = (cPtr - cValue);
        table_count++;
    }
    else
    {
        http_session->ucDynamicFlags = NO_DYNAMIC_CONTENT_TO_ADD;// inform that we don't want to generate anything this time
        previous_TxLength = 0;
        table_count = 0;
        return 0;
    }

    return cValue;
}

/**
    @brief This is used to establish a TCP socket with a host server. Data can be transferred
    to the host server for external analysis if needed.

    @param Socket Current TCP socket to use.
    @param ucEvent Current TCP event status.
    @param ucIp_Data Pointer to data.
    @param usPortLen Destination Port data length.

    @return iReturn TCP status message.
*/
int tcpListener(USOCKET Socket, unsigned char ucEvent, unsigned char *ucIp_Data, unsigned short usPortLen)
{
    volatile s16 queue;
    volatile u8 local_evt;
    volatile u8 buf[100];
    volatile int iReturn = APP_ACCEPT;

    local_evt = ucEvent;

    switch (local_evt)
    {
    case TCP_EVENT_CONREQ:
    case TCP_EVENT_CONNECTED:
        uStrcpy((CHAR *)buf, "Hi!!\r\n");
        queue = fnSendBufTCP(Socket, (u8 *)buf, 6, (TCP_BUF_SEND | TCP_BUF_SEND_REPORT_COPY));
        iReturn =  APP_SENT_DATA;
        break;
    case TCP_EVENT_CLOSE:
    case TCP_EVENT_ACK:
        // Send next buffered (if waiting)
        if (fnSendBufTCP(Socket, 0, 0, TCP_BUF_NEXT))
        {
            iReturn = APP_SENT_DATA;    // Mark that data has been transmitted
        }
        break;
    case TCP_EVENT_ARP_RESOLUTION_FAILED:
    #ifdef SUPPORT_PEER_WINDOW
    case TCP_EVENT_PARTIAL_ACK:
        // Possible ack to a part of a transmission received
        // Send next buffered (if waiting)
        if (fnSendBufTCP(Socket, 0, usPortLen, TCP_BUF_NEXT))
        {
            iReturn =  APP_SENT_DATA;
        }
        break;
    #endif
    case TCP_EVENT_REGENERATE:
        if (fnSendBufTCP(Socket, 0, 0, TCP_BUF_REP) != 0) // repeat send buffered
        {
          #ifdef SUPPORT_PEER_WINDOW
            // Kick off any following data as long as windowing allows it
            fnSendBufTCP(Socket, 0, 0, (TCP_BUF_NEXT | TCP_BUF_KICK_NEXT));
          #endif
            iReturn =  APP_SENT_DATA;
        }
        break;
    case TCP_EVENT_DATA:
        memset((CHAR *)buf, 0, 100);
        uStrcpy((CHAR *)buf, "Eat at Joe's\r\n");
        queue = fnSendBufTCP(Socket, (u8 *)buf, 14, (TCP_BUF_SEND | TCP_BUF_SEND_REPORT_COPY));
        tcp_parse_command(Socket, ucIp_Data, usPortLen);
        iReturn =  APP_SENT_DATA;
        break;
    case TCP_EVENT_ABORT:
    case TCP_EVENT_CLOSED:
        // Go back to listening state on the same port number
        fnTCP_Listen(Socket, TCP_PORT, 0);
        break;
    }
    return iReturn;
}

/**
    @brief This will parse the received TCP requests from the host server.

    @param Socket Current TCP socket.
    @param data Pointer to received data.
    @param len Length of received data.
*/
void tcp_parse_command(USOCKET Socket, u8 * data, int len)
{
    volatile u16 addr;

    switch(data[0])
    {
        // Ping Request
        case 0x01:
            // Send ping request to node address.
            addr = data[2];
            addr <<= 8;
            addr |= data[1];
            macPing(PING_REQ_FRAME, (u16)addr);
            fnDebugMsg("\r\nTCP Ping to");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg("\r\n");
            break;

        // Remove Node
        case 0x02:
            // STUB FUNCTION
            break;

#if APP == SENSOR
        u8 calpoint;
        // Send Cal Data
        case 0x03:
            // Send request for cal info to node address.
            calpoint = data[1];
            char string[10];
            strncpy(string, (char *)&data[2], 8);
            string[8] = 0;
            sensorSendCalPoint(calpoint, string);
            fnDebugMsg("\r\nTCP Set Cal Data to");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg(" with data ");
            fnDebugMsg(string);
            fnDebugMsg("\r\n");
            break;

        // Get Cal Data
        case 0x04:
            // Send request for cal info to node address.
            addr = data[2];
            addr <<= 8;
            addr |= data[1];
            sensorRequestCalInfo((u16)addr);
            fnDebugMsg("\r\nTCP Get Cal Data from");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg("\r\n");
            break;

        // Set Name
        case 0x05:
            // Send set name request to node address.
            addr = data[2];
            addr <<= 8;
            addr |= data[1];
            strncpy(string, (char *)&data[3], 8);
            string[8] = 0;
            sensorSendSetNodeName((u16)addr, string);
            fnDebugMsg("\r\nTCP Set Name of");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg("  to  ");
            fnDebugMsg(string);
            fnDebugMsg("\r\n");
            break;

        // Get Name
        case 0x06:
            // Send get name request to node address.
            addr = data[2];
            addr <<= 8;
            addr |= data[1];
            sensorRequestReading((u16)addr, 20);
            fnDebugMsg("\r\nTCP Get Name from");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg("\r\n");
            break;

        // Send Data
        case 0x07:
            // Tell node to send data on periodic interval.
            addr = data[2];
            addr <<= 8;
            addr |= data[1];
            u16 time = data[3];
            sensorRequestReading((u16)addr, time);
            fnDebugMsg("\r\nTCP Send Data Cmd to");
            fnDebugHex(addr, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg(" with interval ");
            fnDebugHex(time, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 2));
            fnDebugMsg("\r\n");
            break;
#endif // APP == SENSOR

        // Send network table info.
        case 0x08:
            // Send all the data in the network table.
            get_nwk_table(host_socket);
            fnDebugMsg("\r\nTCP Send Network Table from coord");
            fnDebugMsg("\r\n");
            break;

        default:
            break;
    }
}

/**
    @brief This will send the current network table to the host server when requested.

    @param Socket Current TCP socket.
*/
void get_nwk_table(USOCKET Socket)
{
    u16 i;
    associatedNodes_t *node;
    u8 hostbuf[50];
    u8 temp = 0;
    s16 queue;

    queue = fnSendBufTCP(host_socket, (u8 *)"WIRELESST", 9, TCP_BUF_SEND);

    if(telPrintReading && (queue > 0))
        fnDebugMsg("\r\nWIRELESST buffered for TCP\r\n");
    else
        fnDebugMsg("\r\nWIRELESST buffer problem\r\n");

    for(i=0; i<MAXNODES; i++)
    {
        node = macGetNode(i);
        if (node->nodeType)
        {
            memset(hostbuf, 0, 50);
            memcpy(hostbuf, node, sizeof(associatedNodes_t));
//            ipv6_lookup_addr((char *)&hostbuf[sizeof(associatedNodes_t)], i);
            queue = fnSendBufTCP(host_socket, hostbuf, 50, TCP_BUF_SEND);
            if(telPrintReading && (queue <= 0))
                fnDebugMsg("\r\nTable buffer problem");
            temp++;
        }
    }
    hostbuf[0]=0xFF;
    fnSendBufTCP(host_socket, hostbuf, 1, 0);
    fnDebugMsg("node count = ");
    fnDebugDec(temp, 0, 0);
}

/**
    @brief This is an array of radio register names for serial output
           used when rf2xx_reg_dump() is called.  See the radio
           datasheet for details.
*/
u8 rf2xxregnames[][16] =
    {"TRX_STATUS", "TRX_STATE", "TRX_CTRL_0", "TRX_CTRL_1", "PHY_TX_PWR",
     "PHY_RSSI", "PHY_ED_LEVEL", "PHY_CC_CCA", "CCA_THRES", "IRQ_MASK",
     "IRQ_STATUS", "VREG_CTRL", "BATMON", "XOSC_CTRL", "RX_SYN", "XAH_CTRL_1",
     "FTN_CTRL", "PLL_CF", "PLL_DCU", "PART_NUM", "VERSION_NUM", "MAN_ID_0",
     "MAN_ID_1", "SHORT_ADDR_0", "SHORT_ADDR_1", "PAN_ID_0", "PAN_ID_1",
     "IEEE_ADDR_0", "IEEE_ADDR_1", "IEEE_ADDR_2", "IEEE_ADDR_3", "IEEE_ADDR_4",
     "IEEE_ADDR_5", "IEEE_ADDR_6", "IEEE_ADDR_7", "XAH_CTRL_0", "CSMA_SEED_0",
     "CSMA_SEED_1", "CSMA_BE"};

/**
    @brief This is an array of radio register values to be used when
           rf2xx_reg_dump() is called.
*/
u8 rf2xxregenum[] =
    {RG_TRX_STATUS, RG_TRX_STATE, RG_TRX_CTRL_0, RG_TRX_CTRL_1, RG_PHY_TX_PWR,
     RG_PHY_RSSI, RG_PHY_ED_LEVEL, RG_PHY_CC_CCA, RG_CCA_THRES, RG_IRQ_MASK,
     RG_IRQ_STATUS, RG_VREG_CTRL, RG_BATMON, RG_XOSC_CTRL, RG_RX_SYN, RG_XAH_CTRL_1,
     RG_FTN_CTRL, RG_PLL_CF, RG_PLL_DCU, RG_PART_NUM, RG_VERSION_NUM, RG_MAN_ID_0,
     RG_MAN_ID_1, RG_SHORT_ADDR_0, RG_SHORT_ADDR_1, RG_PAN_ID_0, RG_PAN_ID_1,
     RG_IEEE_ADDR_0, RG_IEEE_ADDR_1, RG_IEEE_ADDR_2, RG_IEEE_ADDR_3, RG_IEEE_ADDR_4,
     RG_IEEE_ADDR_5, RG_IEEE_ADDR_6, RG_IEEE_ADDR_7, RG_XAH_CTRL_0, RG_CSMA_SEED_0,
     RG_CSMA_SEED_1, RG_CSMA_BE};

/**
    @brief Dumps the RF2xx register contents to serial port.

    Note: The serial output will only be available if the @ref DEBUG
    macro is defined as non-zero.
*/
void rf2xxregdump(void)
{
    {
        u8 i,j,k,val, x, z;

        fnDebugMsg("\r\n\r\nREG DUMP\r\n");

        k = sizeof(rf2xxregenum);
        for(i=0;i<k;i++)
        {
            val =  hal_register_read(rf2xxregenum[i]);
            fnDebugMsg((char *)rf2xxregnames[i]);
            x = 12 - strlen((char *)rf2xxregnames[i] );
            for(z=0; z<x; z++)
            {
                fnDebugMsg(" ");
            }
            fnDebugHex(val, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(val)));
            fnDebugMsg(" ");
            for (j=7;j<8;j--)
                // Print a bit
                if((val >> j) & 1)
                fnDebugMsg("1");
                else
                fnDebugMsg("0");
            fnDebugMsg("\r\n");
        }
        fnDebugMsg("\r\n");
    }

}

/**
    @brief Initialize the ARM based RUM application.
*/
void armAppInit(void)
{
    volatile s8 val;

    // Open up a file for writing.
    val = file_fopen(&sd_file, &(efs.myFs), sd_filename, MODE_APPEND);
    if(val == -1)
        fnDebugMsg("\r\nfopen failed!\r\n");
    else
    {
        // Create a directory.
        mkdir(&(efs.myFs), "/data");

        // concatenate the directory and name
        uStrcpy((CHAR *)sd_filename, "/data/log.txt");
    }

    // Global to use in terminalArmAppTask in case a user enters a new ile name.
    files_sys_active = true;
}

/**
    @brief When sensor data is received and an SD card is available, it will parse
    the sensor frame data and append it to the current log file.

    @param frame Pointer to data received.
*/
void armAppRcvData(sftSensorReading *frame)
{
    // SD card is good if "1" returned. Error if "-1".
    if((sd_State(&(efs.myCard)) == 1) && (files_sys_active))
    {
        file_fopen(&sd_file, &(efs.myFs), sd_filename, MODE_APPEND);
        // NULL terminate the string buffers.
        stored_data[0].c1 = ',';
        stored_data[0].c2 = ',';
        stored_data[0].c3 = ',';
        stored_data[0].c4 = ',';
        stored_data[0].c5 = '\n';

        fnDebugDec(frame->addr, 0, (CHAR *)&stored_data[0].addr);
        strncpy((char *)stored_data[0].reading, (char *)frame->reading, 6);
        strncpy((char *)stored_data[0].units, (char *)frame->units, 5);
        strncpy((char *)stored_data[0].name, (char *)frame->name, 8);
        //strcpy((char *)stored_data[0].time, (char *)time);
        //    data_i++;

        file_write(&sd_file, sizeof(stored_readings_t), (euint8 *)stored_data);
        file_fclose(&sd_file);
        fs_flushFs(&(efs.myFs));
    }
}

/**
    @brief This will reset the telnet state if calibration takes too long.
*/
void armCalReset(void)
{
    fnDebugMsg("\r\nCalibration timed out.\r\n");
    telState = 0;
}

/**
   @brief Sample application task loop function.

   This function is meant to be called periodically.  It uses the serial port and button status
   for input, and implements a terminal interface for debugging use.

   @param telBuf Current telnet data buffer.
   @param telBufLen The telnet data length.
   @param usTelnet_state The current telnet state used to manage user interaction.

   @return usTelnet_state Returns the current telnet state after user action.
*/
telType_t telState = 0;
int fnHandleTelnetInput(unsigned char *telBuf, int telBufLen, unsigned short usTelnet_state)
{
    static u16 addr=0;
    static u16 channel;
    static char str[102];
    static CHAR cBuf[16];
    static u8 timer_id;
    CHAR new_rum_ip[15];
#if IPV6LOWPAN
    char ipv6_addr[60]; /* Store IPv6 Address: worst-case size */
#endif // IPV6LOWPAN

    if (macConfig.busy)
    {
        // try again when mac is not busy
        return usTelnet_state;
    }

    // We were in the middle of things and had to grab data from the user.
    // Data is available to process now.
    if(telState != 0)
    {
        switch(telState)
        {
            case TEL_FILE_NAME:
                memset(sd_filename, 0, 20);
                // Always use the same directory.
                uStrcpy((CHAR *)sd_filename, "/data/");
                // A file name can be max 8.3 format

                // The file system was closed in the initial call.
                if(!files_sys_active)
                {
                  // Has a char been received?
                  if(telBufLen)
                  {
                      uMemcpy((CHAR *)&sd_filename[6], telBuf, telBufLen);
                  }
                  // Write the fle name.
                  uMemcpy((CHAR *)&sd_filename[6+telBufLen], ".txt", 4);
                  efs_init(&efs,0);
                  file_fopen(&sd_file, &(efs.myFs), sd_filename, MODE_APPEND);
                }
                telState = 0;
            break;

            case TEL_PING_NODE:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                addr = atoi((char *)telBuf);
                macPing(PING_REQ_FRAME, addr);
                telState = 0;
            break;

            case TEL_CHANNEL:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                channel = atoi((char *)telBuf);

                // Re-do the init stuff.
                macInit(channel);
                macStartCoord();
                fnDebugMsg("\r\nNew channel selected.\r\n");
                macConfig.associated = true;
                telState = 0;
            break;

#if APP == SENSOR
            case TEL_REQUEST_DATA_FROM_NODE:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                addr = atoi((char *)telBuf);
                u16 time;
                fnDebugMsg("\r\nReport time (100mS intervals):");
                telState = TEL_REPORT_INTERVAL;
            break;

            case TEL_REPORT_INTERVAL:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                time = atoi((char *)telBuf);
                sensorRequestReading(addr, time);
                telState = 0;
            break;

            case TEL_CAL_WHICH_NODE:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                addr = atoi((char *)telBuf);

                // Get cal info from node
                sensorRequestCalInfo(addr);

                fnDebugMsg("\r\nEnter Cal point 1:");
                telState = TEL_CAL_POINT_1;

                // Set a time out.
                timer_id = macSetAlarm(5000, armCalReset);
            break;

            case TEL_CAL_POINT_1:
                telBuf[telBufLen] = 0;

                macTimerEnd(timer_id);

                sensorSendCalPoint(0, (char *)telBuf);

                if(coordCalInfo->calType == 2)
                {
                  fnDebugMsg("\r\nEnter Cal point 2:");
                  telState = TEL_CAL_POINT_2;
                }
                else
                  telState = 0;
            break;

            case TEL_CAL_POINT_2:
                telBuf[telBufLen] = 0;

                sensorSendCalPoint(1, (char *)telBuf);

                telState = 0;
            break;

            case TEL_NAME_WHICH_NODE:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                addr = atoi((char *)telBuf);
                fnDebugMsg("\r\nEnter name:");

                telState = TEL_NEW_NODE_NAME;
            break;

            case TEL_NEW_NODE_NAME:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                sensorSendSetNodeName(addr, (char *)telBuf);
                telState = 0;
            break;
#endif // APP == SENSOR

            case TEL_WAKE_NODE:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                addr = atoi((char *)telBuf);
                macWakeChildNode(addr);
                telState = 0;
            break;

            case TEL_NEW_IP:
                // The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                uMemset(new_rum_ip, 0, 15);
                uMemcpy(new_rum_ip, telBuf, telBufLen);

                /**
                  @brief    This code will take the new IP the user has entered write it to FLASH
                            and immediately start using the new IP address.
                */
                fnStrIP(new_rum_ip, &network.ucOurIP[0]);   // set new address for immediate use
                fnDeleteArp();  // delete all ARP entries since they may no longer be valid
                uMemcpy(&temp_pars->temp_network.ucOurIP[0], &network.ucOurIP[0], IPV4_LENGTH); // copy also to temp parameters since these are the ones saved by next command
                fnSaveNewPars(SAVE_NEW_PARAMETERS); // save these to FLASH with immediate validity

                telState = 0;
            break;

            case TEL_TOUCH:
            		// The last element in the buffer should be a CR, remove it.
                telBuf[telBufLen] = 0;

                // Do the function
                if (telBuf[0] == 'p')
                {
                    allNodes(PING_ALL,0);
                    telState = 0;
                }

                if (telBuf[0] == 'r' && APP == SENSOR)
                {
                    fnDebugMsg("\r\nReport time (100mS intervals):");
                    telState = TEL_TOUCH_INTERVAL;
                }
            break;

            case TEL_TOUCH_INTERVAL:
              // The last element in the buffer should be a CR, remove it.
              telBuf[telBufLen] = 0;

              allNodes(REPORT_ALL, atoi((char *)telBuf));

              telState = 0;
            break;

            default:
            break;
        }
    }
    else
    {
        fnDebugMsg("\r\n");
        switch (telBuf[0])
        {
            case 'o':
                // Toggle reading print out.
                telPrintReading ^= 1;
            break;

            case 'd':
                // reg dump
                rf2xxregdump();
                break;
            case 'f':
                // Turn on data file recording to SD card.
                // Close up the file sys if there is a card.

                if((sd_State(&(efs.myCard)) != 1))
                {
                    fnDebugMsg("\r\nSD card not installed!\r\n");
                    break;
                }
                files_sys_active = false;
                file_fclose(&sd_file);
                fs_flushFs(&(efs.myFs));

                fnDebugMsg("\r\nEnter a file name (8 characters max): ");

                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_FILE_NAME;
            break;

            case 'b':
                if((sd_State(&(efs.myCard)) != 1))
                {
                    fnDebugMsg("\r\n SD card not installed!\r\n");
                    break;
                }
                // Stop logging data to file.
                files_sys_active = false;
                file_fclose(&sd_file);
                fs_flushFs(&(efs.myFs));
            break;

            case 'l':
                // Start logging data to file.
                if(efs_init(&efs,0)!=0)
                {
                        fnDebugMsg("No SD Card!!\r\n");
                        break;
                }
                file_fopen(&sd_file, &(efs.myFs), sd_filename, MODE_APPEND);
                files_sys_active = true;
            break;

            case 't':
                // print table
                arm_macPrintTree();
            break;

            case 'T':
        	      // Do something to all nodes
                fnDebugMsg("\r\nAll nodes - (r)eading, (p)ping:");
            		telState = TEL_TOUCH;
            break;

            case 'I':
              // Get new IP address.
              fnDebugMsg("\r\nNew IP = ");

              // We have to leave and let the user enter data via the telnet session.
               telState = TEL_NEW_IP;
            break;

            case 'i':
                // print info
                fnDebugMsg("\r\nshort =");
                fnDebugHex(macConfig.shortAddress, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(macConfig.shortAddress)));
                fnDebugMsg("\r\nparent =");
                fnDebugHex(macConfig.parentShortAddress, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(macConfig.parentShortAddress)));
                fnDebugMsg("\r\nroute=");
                fnDebugHex(macConfig.lastRoute, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(macConfig.lastRoute)));
                fnDebugMsg("\r\nchannel = ");
                fnDebugDec(macConfig.currentChannel, 0, 0);
                fnDebugMsg("\r\nPAN ID =");
                fnDebugHex(macConfig.panId, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | sizeof(macConfig.panId)));
                u32 low = macConfig.longAddr;
                u32 high = macConfig.longAddr >> 32;
                fnDebugMsg("\r\nlong =");
                fnDebugHex(high, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 4));
                fnDebugHex(low, (CODE_CAPITALS | 4));
                if(macConfig.associated)
                    fnDebugMsg("\r\nassoc = true");
                else
                    fnDebugMsg("\r\nassoc = false");
                fnDebugMsg("\r\nrand =");
                u8 rnum = radioRandom(8);
                fnDebugHex(rnum, (WITH_SPACE | WITH_LEADIN | CODE_CAPITALS | 1));
                fnDebugMsg("\r\n");
                // Radio part number
                u16 pn = radioGetPartnum();
                switch (pn)
                {
                  case RF230:
                      pn = 230;
                      break;
                  case RF231:
                      pn = 231;
                      break;
                  case RF212:
                      pn = 212;
                      break;
                  default:
                      // Just report whatever number the radio chip gives.
                      break;
                }
                fnDebugMsg("part = RF");
                fnDebugDec(pn, 0, 0);
                fnDebugMsg("\r\n");
                u8 radnum = hal_register_read(RG_VERSION_NUM);
                fnDebugMsg("version = ");
                fnDebugDec(radnum, 0, 0);
                fnDebugMsg("\r\n");
                if (NODETYPE != COORD && APP == SENSOR)
                {
                    fnDebugMsg("Name= ");
                    fnDebugMsg(sensorGetName());
                    fnDebugMsg("\r\n");
                }
            break;

            case 'p':
                // ping
                memset(str, 0, 102);
                fnDebugMsg("\r\nEnter short addr: ");

                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_PING_NODE;
            break;

            case 'c':
                // change coordinator channel
                fnDebugMsg("\r\nEnter new chan: ");

                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_CHANNEL;
              break;

#if APP == SENSOR
            case 'r':
                // get address and time
                fnDebugMsg("\r\nRequest data from node: ");

                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_REQUEST_DATA_FROM_NODE;
            break;

            case 'C':
                // calibrate an end node
                // get address of node
                fnDebugMsg("\r\nCal which node: ");

                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_CAL_WHICH_NODE;
            break;

            case 'n':
                // Name a node
                fnDebugMsg("\r\nName which node: ");
                // We have to leave and let the user enter data via the telnet session.
                telState = TEL_NAME_WHICH_NODE;
            break;
#endif // APP == SENSOR

            case 'w':
                // wake an end node
                fnDebugMsg("\r\nWake which node: ");

                telState = TEL_WAKE_NODE;
            break;

            case 'a':
                // Print out our IP address.
                fnDebugMsg("IP address = ");
                fnIPStr(&network.ucOurIP[0], cBuf);
                fnDebugMsg(cBuf);
            break;

#if IPV6LOWPAN
            case 'A':
                // Print out the IPv6 addr info.
                if(uip_netif_physical_if[INTERFACE_ETHERNET].addresses[1].state != NOT_USED)
                {
                    sprint_ip(ipv6_addr, uip_netif_physical_if[INTERFACE_ETHERNET].addresses[1].ipaddr.u16);
                    fnDebugMsg("\r\nIPv6 Ethernet IF = ");
                    fnDebugMsg(ipv6_addr);
                    fnDebugMsg("\r\n");
                    sprint_ip(ipv6_addr, uip_netif_physical_if[INTERFACE_802154].addresses[1].ipaddr.u16);
                    fnDebugMsg("IPv6 6LoWPAN IF = ");
                    fnDebugMsg(ipv6_addr);
                    fnDebugMsg("\r\n");
                }
                else
                    fnDebugMsg("\r\nNo IPv6 Addresses Acquired\r\n");
            break;
#endif // IPV6LOWPAN

            case 'Q':
                if(usTelnet_state == ES_NETWORK_COMMAND_MODE)
                {
                    fnDebugMsg("\n\rBye Bye\n\r");
                    usTelnet_state = ES_TERMINATE_CONNECTION;                  // now quit the Telnet session
                    return usTelnet_state;
                }
            break;
            case 'X':
                // Crank up transmit power to ~3dB
                radioSetTxPowerLevel(TX_PWR_3DBM);
                break;

            default:
            break;
        }
    }

    // Delay a bit to allow for other messages (ping resp) to print.
    if(!telState)
        macSetAlarm(250,printPrompt);
    // Clear the buffer.
    uMemset(telBuf, 0, 32);

    return usTelnet_state;
}
