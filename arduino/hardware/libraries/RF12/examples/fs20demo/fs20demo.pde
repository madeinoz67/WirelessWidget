// This example sends commands to the Conrad/ELV 868 MHz FS20 units via OOK.
// 2009-02-21 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: fs20demo.pde 2 2009-07-05 09:25:22Z jcw@equi4.com $

// Note thar RFM12B radios are not really designed for OOK (on-off keying),
// but this can be simulated anyway by simply turning the transmitter on and 
// off via the SPI interface. Doing so takes about 25 usecs, so the delays
// used for encoding simple bit patterns need to be adjusted accordingly.

#include "RF12.h"
#include <util/parity.h>

void sendBits(uint16_t data, uint8_t bits) {
    if (bits == 8) {
        ++bits;
        data = (data << 1) | parity_even_bit(data);
    }
    for (uint16_t mask = bit(bits-1); mask != 0; mask >>= 1) {
        int width = data & mask ? 575 : 375; // 25 usec is approx overhead
        rf12_onOff(1);
        delayMicroseconds(width);
        rf12_onOff(0);
        delayMicroseconds(width);
    }
}

void fs20cmd(uint16_t house, uint8_t addr, uint8_t cmd) {
	uint8_t sum = 6 + (house >> 8) + house + addr + cmd;
	for (uint8_t i = 0; i < 3; ++i) {
		sendBits(1, 13);
		sendBits(house >> 8, 8);
		sendBits(house, 8);
		sendBits(addr, 8);
		sendBits(cmd, 8);
		sendBits(sum, 8);
		sendBits(0, 1);
		delay(10);
	}
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[fs20demo]");
    
    rf12_initialize(0, RF12_868MHZ);
}

void loop() {  
    Serial.println("on");
	fs20cmd(0x1234, 1, 17);
	delay(2000);
	
    Serial.println("off");
	fs20cmd(0x1234, 1, 0);
	delay(5000);
}
