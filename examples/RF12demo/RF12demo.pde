// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// $Id: $
//
// 2009-07-21 (seaton@gateway.net.au) Added support for RFM12Widget, GREEN LED Blinks on successful CRC Receipt

#include "RF12.h"
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#define ledPin 9

typedef struct {
  uint8_t nodeId;
  uint8_t group;
  char msg[RF12_EEPROM_SIZE-4];
  uint16_t crc;
}
RF12Config;

static RF12Config config;

char cmd;
uint8_t value, arg;
uint8_t testbuf[RF12_MAXDATA];

static void addCh(char* msg, char c) {
  uint8_t n = strlen(msg);
  msg[n] = c;
}

static void addInt(char* msg, uint16_t v) {
  if (v > 10)
    addInt(msg, v / 10);
  addCh(msg, '0' + v % 10);
}

static void saveConfig() {
  // set up a nice config string to be shown on startup
  memset(config.msg, 0, sizeof config.msg);
  strcpy(config.msg, " ");

  uint8_t id = config.nodeId & 0x1F;
  addCh(config.msg, '@' + id);
  strcat(config.msg, " i");
  addInt(config.msg, id);
  if (config.nodeId & 0x20)
    addCh(config.msg, '*');

  strcat(config.msg, " g");
  addInt(config.msg, config.group);

  strcat(config.msg, " @ ");
  static uint16_t bands[4] = {
    315, 433, 868, 915   };
  uint16_t band = config.nodeId >> 6;
  addInt(config.msg, bands[band]);
  strcat(config.msg, " MHz ");

  config.crc = ~0;
  for (uint8_t i = 0; i < sizeof config - 2; ++i)
    config.crc = _crc16_update(config.crc, ((uint8_t*) &config)[i]);

  // save to EEPROM
  for (uint8_t i = 0; i < sizeof config; ++i) {
    uint8_t b = ((uint8_t*) &config)[i];
    eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
  }

  if (!rf12_config())
    Serial.println("config save failed");
}

static void showHelp() {
  Serial.println();
  Serial.println("Available commands:");
  Serial.println("  <nn> i    - set node ID (standard node ids are 1..26)");
  Serial.println("              (or enter an upper case 'A'..'Z' to set id)");
  Serial.println("  <n> b     - set MHz band (4 = 433, 8 = 868, 9 = 915)");
  Serial.println("  <nnn> g   - set network group (RFM12 only supports 212)");
  Serial.println("  <n> c     - set collect mode (advanced use, normally 0)");
  Serial.println("  <n> a     - send test packet of 5 x <n> bytes, with ack");
  Serial.println("  <n> s     - send test packet of 5 x <n> bytes, no ack");
  Serial.println("Current configuration:");
  rf12_config();
}

static void blinkLed(int times, int lengthms) {
    long limit;
    for (int i = 0; i < times; i++){
        digitalWrite(ledPin, LOW);  // on
        limit = millis() + lengthms;
        while(millis() < limit) {
          set_sleep_mode(SLEEP_MODE_IDLE);
          sleep_mode();
        }
        digitalWrite(ledPin, HIGH); // off
        limit = millis() + lengthms;
        while(millis() < limit) {
          set_sleep_mode(SLEEP_MODE_IDLE);
          sleep_mode();
        }
    }
}


void setup() {
  Serial.begin(57600);
  Serial.print("\n[RF12DEMO]");

  pinMode(ledPin, OUTPUT);

  if (rf12_config()) {
    config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
    config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
  }
  else {
    config.nodeId = 0x41; // node A1 @ 433 MHz
    config.group = 0xD4;
    saveConfig();
  }

  for (uint8_t i = 0; i < sizeof testbuf; ++i)
    testbuf[i] = i;

   blinkLed(3,300);

  showHelp();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if ('0' <= c && c <= '9')
      value = 10 * value + c - '0';
    else if ('a' <= c && c <='z') {
      Serial.print("> ");
      Serial.print((int) value);
      Serial.println(c);
      switch (c) {
      default:
        showHelp();
        break;
      case 'i': // set node id
        config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
        saveConfig();
        break;
      case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
        value = value == 8 ? RF12_868MHZ :
        value == 9 ? RF12_915MHZ : RF12_433MHZ;
        config.nodeId = (value << 6) + (config.nodeId & 0x3F);
        saveConfig();
        break;
      case 'g': // set network group
        config.group = value;
        saveConfig();
        break;
      case 'c': // set collect mode (off = 0, on = 1)
        if (value)
          config.nodeId |= 0x20;
        else
          config.nodeId &= ~0x20;
        saveConfig();
        break;
      case 'a': // send test packet of 5 x N bytes, request an ack
      case 's': // send test packet of 5 x N bytes, no ack
        cmd = c;
        arg = value < 9 ? 5 * value : sizeof testbuf;
        break;
      }
      value = 0;
    }
    else if ('A' <= c && c <= 'Z') {
      config.nodeId = (config.nodeId & 0xE0) + (c & 0x1F);
      saveConfig();
    }
    else if (c > ' ')
      showHelp();
  }

  if (rf12_recvDone()) {
    if(rf12_crc == 0){
      Serial.print("OK");
      blinkLed(1,300);
    }
     else
       Serial.print(" ?");
    //Serial.print( (rf12_crc == 0)? "OK" : " ?");
    for (uint8_t i = 0; i < rf12_len + 2 && i < 20; ++i) {
      Serial.print(' ');
      Serial.print((int) rf12_buf[i]);
    }
    Serial.println();

    if (rf12_crc == 0 && (rf12_hdr & ~RF12_HDR_MASK) == RF12_HDR_ACK) {
      Serial.println(" -> ack");
      uint8_t addr = rf12_hdr & RF12_HDR_MASK;
      rf12_sendStart(RF12_HDR_CTL | RF12_HDR_DST | addr, 0, 0);
    }

  }

  if (cmd && rf12_canSend()) {
    Serial.print(" -> ");
    Serial.print((int) arg);
    Serial.println(" b");
    rf12_sendStart(cmd == 'a' ? RF12_HDR_ACK : 0, testbuf, arg);
    cmd = 0;
  }
}

