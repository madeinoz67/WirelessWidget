// RFM12B driver implementation
// 2009-02-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12.cpp 2 2009-07-05 09:25:22Z jcw@equi4.com $

#include "RF12.h"
#include <avr/io.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <WProgram.h>

// maximum transmit / receive buffer: 2 header + data + 2 crc bytes
#define RF_MAX   (RF12_MAXDATA + 4)

// pins used for the RFM12B interface
#define RFM_IRQ  2
#define SPI_SS   10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK  13

// RF12 command codes
#define RF_RECEIVER_ON  0x82D9
#define RF_XMITTER_ON   0x8239
#define RF_IDLE_MODE    0x8209
#define RF_SLEEP_MODE   0x8201
#define RF_WAKEUP_MODE  0x8203

// bits in the node id configuration byte
#define NODE_BAND       0xC0        // frequency band
#define NODE_ACKANY     0x20        // ack on broadcast packets if set
#define NODE_ID         0x1F        // id of this node, as A..Z or 1..31

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
    TXRECV,
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

static uint8_t nodeid;              // address of this node
static uint8_t group;               // network group
static uint16_t crcInit;            // initial crc value
static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state

volatile uint16_t rf12_crc;         // running crc value
volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes

static void spi_initialize () {
    digitalWrite(SPI_SS, 1);
    pinMode(SPI_SS, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_SCK, OUTPUT);
    
#if F_CPU <= 10000000
    // clk/4 is ok for the RF12's SPI
    SPCR = _BV(SPE) | _BV(MSTR);
#else
    // use clk/8 (2x 1/16th) to avoid exceeding RF12's SPI specs of 2.5 MHz
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
    SPSR |= _BV(SPI2X);
#endif
}

static uint16_t rf12_xfer (uint16_t cmd) {
    // the 2 loops below each spin 4 usec with a 2 MHz SPI clock
    uint16_t reply;
    digitalWrite(SPI_SS, 0);
    SPDR = cmd >> 8;
    while (!(SPSR & _BV(SPIF)))
        ;
    reply = SPDR << 8;
    SPDR = cmd;
    while (!(SPSR & _BV(SPIF)))
        ;
    reply |= SPDR;
    digitalWrite(SPI_SS, 1);
    return reply;
}

static void rf12_interrupt() {
    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
    rf12_xfer(0x0000);
    
    if (rxstate == TXRECV) {
        uint8_t in = rf12_xfer(0xB000);

        rf12_buf[rxfill++] = in;
        rf12_crc = _crc16_update(rf12_crc, in);

        if (rxfill >= rf12_len + 4 || rxfill >= RF_MAX)
            rf12_xfer(RF_IDLE_MODE);
    } else {
        uint8_t out;

        if (rxstate < 0) {
            uint8_t pos = 2 + rf12_len + rxstate++;
            out = rf12_buf[pos];
            rf12_crc = _crc16_update(rf12_crc, out);
        } else
            switch (rxstate++) {
                case TXSYN1: out = 0x2D; break;
                case TXSYN2: out = group; rxstate = - (2 + rf12_len); break;
                case TXCRC1: out = rf12_crc; break;
                case TXCRC2: out = rf12_crc >> 8; break;
                case TXDONE: rf12_xfer(RF_IDLE_MODE); // fall through
                default:     out = 0xAA;
            }
            
        rf12_xfer(0xB800 + out);
    }
}

static void rf12_recvStart () {
    rxfill = rf12_len = 0;
    rf12_crc = crcInit;
    rxstate = TXRECV;    
    rf12_xfer(RF_RECEIVER_ON);
}

uint8_t rf12_recvDone () {
    if (rxstate == TXRECV && (rxfill >= rf12_len + 4 || rxfill >= RF_MAX)) {
        rxstate = TXIDLE;
        if (rf12_len > RF12_MAXDATA)
            rf12_crc = 1; // force bad crc if packet length is invalid
        if (!(rf12_hdr & RF12_HDR_DST) ||
                (rf12_hdr & RF12_HDR_MASK) == (nodeid & NODE_ID))
            return 1; // it's a broadcast packet or it's addressed to this node
    }
    if (rxstate == TXIDLE)
        rf12_recvStart();
    return 0;
}

uint8_t rf12_canSend () {
    // no need to test with interrupts disabled: state TXRECV is only reached
    // outside of ISR and we don't care if rxfill jumps from 0 to 1 here
    if (rxstate == TXRECV && rxfill == 0) {
        cli(); // start critical section so we can call rf12_xfer() safely
        rf12_xfer(RF_IDLE_MODE); // stop receiver
        //XXX just in case, don't know whether these RF21 reads are needed!
        rf12_xfer(0x0000); // status register
        rf12_xfer(0xB000); // fifo read
        rxstate = TXIDLE;
        sei(); // end critical section
        return 1;
    }
    return 0;
}

void rf12_sendStart (uint8_t hdr, const void* ptr, uint8_t len) {
    rf12_hdr = hdr & RF12_HDR_DST ? hdr :
                (hdr & ~RF12_HDR_MASK) + (nodeid & NODE_ID);
    rf12_len = len;
    memcpy((void*) (2 + rf12_buf), ptr, len);
    
    rf12_crc = crcInit;
    rxstate = TXPRE1;
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void rf12_initialize (uint8_t id, uint8_t band, uint8_t g) {
    nodeid = id;
    group = g;
    
    spi_initialize();
    
    pinMode(RFM_IRQ, INPUT);
    digitalWrite(RFM_IRQ, 1); // pull-up
    
    rf12_xfer(0x80C7 | (band << 4)); // EL,EF,12.0pF 
    rf12_xfer(0x8209); // enable xtal, disable clk pin
    rf12_xfer(0xA640); // 868MHz 
    rf12_xfer(0xC606); // 57.6Kbps (38.4: 8, 19.2: 11, 9.6: 23, 4.8: 47)
    rf12_xfer(0x94A0); // VDI,FAST,134kHz,0dBm,-103dBm 
    rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4 
    rf12_xfer(0xCA83); // FIFO8,SYNC,!ff,DR 
    rf12_xfer(0xCE00 | group); // SYNC=2DXX； 
    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
    rf12_xfer(0x9850); // !mp,90kHz,MAX OUT 
    rf12_xfer(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0 
    rf12_xfer(0xE000); // NOT USE 
    rf12_xfer(0xC800); // NOT USE 
    rf12_xfer(0xC040); // 1.66MHz,2.2V 

#if RF12_VERSION == 1
    crcInit = ~0;
#else
    crcInit = _crc16_update(~0, group);
#endif

    rxstate = TXIDLE;
    if ((nodeid & NODE_ID) != 0)
        attachInterrupt(0, rf12_interrupt, LOW);
}

void rf12_onOff(uint8_t value) {
    rf12_xfer(value ? RF_XMITTER_ON : RF_IDLE_MODE);
}

uint8_t rf12_config() {
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + i));
    if (crc != 0)
        return 0;
        
    uint8_t nodeId = 0, group = 0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE - 2; ++i) {
        uint8_t b = eeprom_read_byte(RF12_EEPROM_ADDR + i);
        if (i == 0)
            nodeId = b;
        else if (i == 1)
            group = b;
        else if (b == 0)
            break;
        else
            Serial.print(b);
    }
    Serial.println();
    
    rf12_initialize(nodeId, nodeId >> 6, group);
    return nodeId & 0x1F;
}

void rf12_sleep (char n) {
    if (n < 0)
        rf12_xfer(RF_IDLE_MODE);
    else {
        rf12_xfer(0xE500 | n);
        rf12_xfer(RF_SLEEP_MODE);
        if (n > 0)
            rf12_xfer(RF_WAKEUP_MODE);
    }
    rxstate = TXIDLE;
}
