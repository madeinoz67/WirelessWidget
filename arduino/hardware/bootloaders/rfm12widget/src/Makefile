# Makefile for WidgetBoot
#
# 20090925:'Widgitised' the origional bootloader makefile
#          Stephen Eaton <seaton@gateway.net.au>
#
# Instructions
#
# To make bootloader .hex file:
# make rfm12widget168_10mhz
# make rfm12widget168_8mhz
# make rfm12widget328_10mhz
# etc...
#
# To burn bootloader .hex file:
# make rfm12widget168_10mhz_isp
# make rfm12widget168_8mhz_isp
# make rfm12widget328_10mhz_isp
# etc...

# program name should not be changed...
PROGRAM    = BOOT

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPPORT	   = usb
ISPSPEED   = -b 115200

MCU_TARGET = atmega168
LDSECTION  = --section-start=.text=0x3800

# the efuse should really be 0xf8; since, however, only the lower
# three bits of that byte are used on the atmega168, avrdude gets
# confused if you specify 1's for the higher bits, see:
# http://tinker.it/now/2007/02/24/the-tale-of-avrdude-atmega168-and-extended-bits-fuses/
#
# similarly, the lock bits should be 0xff instead of 0x3f (to
# unlock the bootloader section) and 0xcf instead of 0x0f (to
# lock it), but since the high two bits of the lock byte are
# unused, avrdude would get confused.

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-e -u -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
-U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x0f:m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2

DEFS       =
LIBS       =

CC         = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION)
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all:

rfm12widget168_10mhz: TARGET = rfm12widget168_10mhz
rfm12widget168_10mhz: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3'
rfm12widget168_10mhz: AVR_FREQ = 10000000L
rfm12widget168_10mhz: $(PROGRAM)_rfm12widget168_10mhz.hex

rfm12widget168_10mhz_isp: rfm12widget168_10mhz
rfm12widget168_10mhz_isp: TARGET = atmega168
rfm12widget168_10mhz_isp: HFUSE = DF
rfm12widget168_10mhz_isp: LFUSE = F7
rfm12widget168_10mhz_isp: EFUSE = 00
rfm12widget168_10mhz_isp: isp

rfm12widget168_8mhz: TARGET = rfm12widget168_8mhz
rfm12widget168_8mhz: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=1'
rfm12widget168_8mhz: AVR_FREQ = 8000000L
rfm12widget168_8mhz: $(PROGRAM)_rfm12widget168_8mhz.hex

rfm12widget168_8mhz_isp: rfm12widget_8mhz
rfm12widget168_8mhz_isp: TARGET = atmega168
rfm12widget168_8mhz_isp: HFUSE = DF
rfm12widget168_8mhz_isp: LFUSE = E2
rfm12widget168_8mhz_isp: EFUSE = 00
rfm12widget168_8mhz_isp: isp


rfm12widget328_16mhz: TARGET = rfm12widget328_16mhz
rfm12widget328_16mhz: MCU_TARGET = atmega328p
rfm12widget328_16mhz: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
rfm12widget328_16mhz: AVR_FREQ = 16000000L
rfm12widget328_16mhz: LDSECTION  = --section-start=.text=0x7800
rfm12widget328_16mhz: $(PROGRAM)_rfm12widget328_16mhz.hex

rfm12widget328_16mhz_isp: rfm12widget328_16mhz
rfm12widget328_16mhz_isp: TARGET = rfm12widget328_16mhz
rfm12widget328_16mhz_isp: MCU_TARGET = atmega328p
rfm12widget328_16mhz_isp: HFUSE = DD
rfm12widget328_16mhz_isp: LFUSE = F7
rfm12widget328_16mhz_isp: EFUSE = 05
rfm12widget328_16mhz_isp: isp

rfm12widget328_10mhz: TARGET = rfm12widget328_10mhz
rfm12widget328_10mhz: MCU_TARGET = atmega328p
rfm12widget328_10mhz: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=1'
rfm12widget328_10mhz: AVR_FREQ = 10000000L
rfm12widget328_10mhz: LDSECTION  = --section-start=.text=0x7800
rfm12widget328_10mhz: $(PROGRAM)_rfm12widget328_10mhz.hex

rfm12widget328_10mhz_isp: rfm12widget328_10mhz
rfm12widget328_10mhz_isp: TARGET = rfm12widget328_10mhz
rfm12widget328_10mhz_isp: MCU_TARGET = atmega328p
rfm12widget328_10mhz_isp: HFUSE = DD
rfm12widget328_10mhz_isp: LFUSE = F7
rfm12widget328_10mhz_isp: EFUSE = 05
rfm12widget328_10mhz_isp: isp

rfm12widget328_8mhz: TARGET = rfm12widget328_8mhz
rfm12widget328_8mhz: MCU_TARGET = atmega328p
rfm12widget328_8mhz: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4' '-DNUM_LED_FLASHES=2'
rfm12widget328_8mhz: AVR_FREQ = 8000000L
rfm12widget328_8mhz: LDSECTION  = --section-start=.text=0x7800
rfm12widget328_8mhz: $(PROGRAM)_rfm12widget328_8mhz.hex

rfm12widget328_8mhz_isp: rfm12widget328_8mhz
rfm12widget328_8mhz_isp: TARGET = rfm12widget328_8mhz
rfm12widget328_8mhz_isp: MCU_TARGET = atmega328p
rfm12widget328_8mhz_isp: HFUSE = DD
rfm12widget328_8mhz_isp: LFUSE = FF
rfm12widget328_8mhz_isp: EFUSE = 05
rfm12widget328_8mhz_isp: isp


isp: $(TARGET)
	$(ISPFUSES)
	$(ISPFLASH)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

