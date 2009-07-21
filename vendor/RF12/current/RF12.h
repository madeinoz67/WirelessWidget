// RFM12B driver definitions
// 2009-02-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12.h 2 2009-07-05 09:25:22Z jcw@equi4.com $

#ifndef RF12_h
#define RF12_h

#include <stdint.h>

// version 1 did not include the group code in the crc
// version 2 does include the group code in the crc
#define RF12_VERSION    2

#define rf12_hdr        rf12_buf[0]
#define rf12_len        rf12_buf[1]
#define rf12_data       (rf12_buf + 2)

#define RF12_HDR_CTL    0x80
#define RF12_HDR_DST    0x40
#define RF12_HDR_ACK    0x20
#define RF12_HDR_MASK   0x1F

#define RF12_MAXDATA    66

#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

// EEPROM address range used by the rf12_config() code
#define RF12_EEPROM_ADDR ((uint8_t*) 0x20)
#define RF12_EEPROM_SIZE 32

extern volatile uint16_t rf12_crc;  // running crc value, should be zero at end
extern volatile uint8_t rf12_buf[]; // recv/xmit buf including hdr & crc bytes

// call this once with the node ID, frequency band, and optional group
void (rf12_initialize) (uint8_t, uint8_t, uint8_t =0xD4);

// initialize the RF12 module from settings stored in EEPROM by "RF12demo"
// don't call rf12_initialize() if you init the hardware with rf12_config()
// returns the node ID as 1..31 value (1..26 correspond to nodes 'A'..'Z')
uint8_t (rf12_config) (void);

// call this frequently, returns true if a packet has been received
uint8_t (rf12_recvDone) (void);

// call this to check whether a new transmission can be started
// returns true when a new transmission may be started with rf12_sendStart()
uint8_t (rf12_canSend) (void);

// call this only when rf12_recvDone() or rf12_canSend() return true
void (rf12_sendStart) (uint8_t, const void*, uint8_t);

// this simulates OOK by turning the transmitter on and off via SPI commands
// use this only when the radio was initialized with a fake zero node ID
void (rf12_onOff) (uint8_t);

// power off the RF12, ms > 0 sets watchdog to wake up again after N * 32 ms
// note: once off, calling this with -1 can be used to bring the RF12 back up
void (rf12_sleep) (char);

#endif
