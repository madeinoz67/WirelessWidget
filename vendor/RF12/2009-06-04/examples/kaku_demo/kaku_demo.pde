// This example sends commands to the KlikAanKlikUit units via OOK at 433 Mhz.
// 2009-02-21 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: kaku_demo.pde 2 2009-07-05 09:25:22Z jcw@equi4.com $

// Note that 868 MHz RFM12B's can send 433 MHz just fine, even though the RF
// circuitry is presumably not optimized for that band. Maybe the range will
// be limited, or maybe it's just because 868 is nearly a multiple of 433 ?

#include "RF12.h"
#include <util/parity.h>

void kakuSend(char addr, uint8_t device, uint8_t on) {
    int cmd = 0x600 | ((device - 1) << 4) | ((addr - 1) & 0xF);
    if (on)
        cmd |= 0x800;
    for (uint8_t repeat = 0; repeat < 4; ++repeat) {
        for (uint8_t bit = 0; bit < 12; ++bit) {
            uint8_t pattern = cmd & _BV(bit) ? 0x8E : 0x88;
            for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    			rf12_onOff(pattern & mask);
    			delayMicroseconds(350);
			}
        }
		rf12_onOff(1);
		delayMicroseconds(350);
		rf12_onOff(0);
		delayMicroseconds(375*31);
    }
}

void setup() {
    Serial.begin(57600);
    Serial.println("\n[kaku_demo]");
    
    rf12_initialize(0, RF12_433MHZ);
}

void loop() {  
    Serial.println("off");
    kakuSend('A', 3, 0);
    delay(2000);
    
    Serial.println("on");
    kakuSend('A', 3, 1);
    delay(5000);
}
