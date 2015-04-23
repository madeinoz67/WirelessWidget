# Introduction #

To make a good wireless protocol for remote flash triggering we need to consider the following:

  * There will be only one master broadcasting. This means that a master unit can assume the channel is free and start transmission whenever it is necessary. The slave must listen to messages from master and only respond when it is required by the message.

  * Messages need to be of different type. Message length depends on message type, therefore type must be transmitted at the beginning of the message. One obvious message type would be "trigger" that would force the slave units to fire their connected flashes. I suggest leaving reserved bits for adding more message types in the future.

  * We should make a distinction between each unit's unique identifier and its master/slave/group function. A unique ID should be rather long, like 8 bytes. There should never be two units with the same ID. ID should be used to target a message to a specific unit, most likely for configuration purposes.

  * A function signature can be made 3 bit long. Say, 0 for master and 1-7 for slave groups. Each slave would know his group. Then a master could transmit a trigger message and have a mask. Slaves would then see if their group is masked or not and trigger or not trigger the flash.

_For future compatibility, all reserved bits should be written as 0._

# Details #

## Message structure ##

| **Type definition** | **Message payload** | **CRC** |
|:--------------------|:--------------------|:--------|
| 1 byte | 0 or more bytes | 1 byte|

Type definition byte defines message type. Message payload varies and depends on message type. accordingly.CRC is 8-bit cyclic redundancy code (CRC-8). This code is good for up to 15 bytes of data and detects single, double and all odd errors.
4 bits in type definition byte can indicate message length in bytes.

Messages with bad CRC are ignored (possibly with beep from buzzer).

### CRC-8 calculation ###

C code example from Wikipedia:

```
/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 bytes(127 bit)
*/
unsigned char Crc8(unsigned char *pcBlock, unsigned char len)
{
    unsigned char crc = 0xFF;
    unsigned char i;
 
    while (len--)
    {
        crc ^= *pcBlock++;
 
        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }
 
    return crc;
}
```

My asm code for fast (table) CRC-8 calculation - tested

```

/* ====================================== */
//	Calculate CRC-8
//	X must contain string address in SRAM
//	TEMP must contain string length
//	Result is returned via TEMP register
//	Uses registers R0, R1, Z, X
/* ====================================== */
calc_CRC:
	;R0 is counter
	mov		R0,	TEMP

	;0xFF is the right starting value for CRC
	ldi		TEMP,	0xFF

calc_next_byte:
	ld		R1,	X+			;R1 now contains next char
	eor		TEMP,	R1		;CRC = CRC xor (next char)

	; load table addres into Y
	ldi		ZH,		high(crc_table*2)
	ldi		ZL,		low(crc_table*2)

	add		ZL,		TEMP	;mind carry!
	brcc	nocarry
	inc		ZH

nocarry:
	lpm		TEMP,	Z		;TEMP contains CRC

	dec		R0
	brne	calc_next_byte

	ret

crc_table:
.db	0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97;
.db	0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E;
.db	0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4;
.db	0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D;
.db	0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11;
.db	0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8;
.db	0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52;
.db	0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB;
.db	0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA;
.db	0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13;
.db	0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9;
.db	0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50;
.db	0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C;
.db	0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95;
.db	0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F;
.db	0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6;
.db	0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED;
.db	0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54;
.db	0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE;
.db	0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17;
.db	0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B;
.db	0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2;
.db	0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28;
.db	0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91;
.db	0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0;
.db	0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69;
.db	0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93;
.db	0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A;
.db	0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56;
.db	0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF;
.db	0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15;
.db	0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC;


```

### Type definition ###
| **bits 7-4** | **bits 3-0** |
|:-------------|:-------------|
| Message type | Payload length |

Payload length doesn't include the type definition byte and the CRC byte. Total message length is payload length + 2.

| **Code** | **Message type** |
|:---------|:-----------------|
| 0000 | Announce master presence |
| 0001 | Trigger |


Announcing master presence should probably trigger some kind of response from slaves but this command requires more consideration and will be updated later.

Trigger command should perform the actual triggering. Payload is one byte where bits 7-1 determine which flash groups fire. Bit 0 is reserved. Therefore, payload 0xFE will trigger all available flash groups. Trigger command is never acknowledged.