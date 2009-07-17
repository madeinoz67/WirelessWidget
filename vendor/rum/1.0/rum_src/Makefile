# Hey Emacs, this is a -*- makefile -*-
###############################################################################
# Makefile for the CDC project
###############################################################################

## General Flags
PROJECT = rum
TARGET = $(PROJECT).elf
CC = avr-gcc

## Define PLATFORM on command line to override makefile default
ifndef PLATFORM
PLATFORM=RCB231
endif

## Figure out which CPU to use for the given platform
ifeq ($(PLATFORM), RAVEN)
MCU = atmega1284p
else
ifeq ($(PLATFORM), RCBSIP)
MCU = atmega1284p
else
ifeq ($(PLATFORM), SPITFIRE)
#MCU = atmega328p
MCU = atmega168p
else
ifeq ($(PLATFORM), RAVENUSB)
MCU = at90usb1287
else
ifeq ($(PLATFORM), DSK001)
MCU = atmega328p
else
MCU = atmega1281
endif
endif
endif
endif
endif

## Figure out bootloader start address
ifeq ($(MCU), atmega1284p)
BOOTLOADER_START=0x1FC00
else
ifeq ($(MCU), atmega328p)
BOOTLOADER_START=0x7E00
else
ifeq ($(MCU), atmega1281)
BOOTLOADER_START=0x1FC00
else
BOOTLOADER_START=0x1FC00
endif
endif
endif


## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

ifndef F_CPU
COMMON += -DF_CPU=8000000UL
else
COMMON += -DF_CPU=$(F_CPU)UL
endif


## Compile options common for all C compilation units.
CFLAGS = $(COMMON) $(CEXTRA)
CFLAGS += -D AVRGCC -Wall -gdwarf-2  -Os -fsigned-char -DPLATFORM=$(PLATFORM)
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d
CFLAGS += -std=gnu99
CFLAGS += -fshort-enums
CFLAGS += -fno-inline
CFLAGS += -D$(TYPE)NODE
## Define DEBUG on command line to override code default
ifdef DEBUG
CFLAGS += -DDEBUG=$(DEBUG)
endif
## Define DEMO on command line to override code default
ifdef DEMO
CFLAGS += -DDEMO=$(DEMO)
endif
## Define APP on command line to override code default
ifdef APP
CFLAGS += -DAPP=$(APP)
endif
## Define RUMSLEEP on command line to override code default
ifdef RUMSLEEP
CFLAGS += -DRUMSLEEP=$(RUMSLEEP)
endif
## Define IPV6LOWPAN on command line to override code default
ifdef IPV6LOWPAN
CFLAGS += -DIPV6LOWPAN=$(IPV6LOWPAN)
endif
## Define SENSOR_TYPE on command line
ifdef SENSOR_TYPE
CFLAGS += -DSENSOR_TYPE=$(SENSOR_TYPE)
endif
## Define PAN_CHANNEL on command line
ifdef PAN_CHANNEL
CFLAGS += -DPAN_CHANNEL=$(PAN_CHANNEL)
endif
## Define PAN_ID on command line
ifdef PAN_ID
CFLAGS += -DPAN_ID=$(PAN_ID)
endif
## Define SERIAL on command line
ifdef SERIAL
CFLAGS += -DSERIAL=$(SERIAL)
endif
## Define OTA_DEBUG on command line
ifdef OTA_DEBUG
CFLAGS += -DOTA_DEBUG=$(OTA_DEBUG)
endif
## Define VLP on command line
ifdef VLP
CFLAGS += -DVLP=$(VLP)
endif

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,-gstabs

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-Map=$(TYPE)$(PROJECT).map,--cref
LDFLAGS += -Wl,--section-start=.bootloader=$(BOOTLOADER_START)

## Make sure DEBUG is defined
ifndef DEBUG
DEBUG = 1
endif

## For debugging, we need printf linked in
ifeq ($(DEBUG), 1)
LDFLAGS += -Wl,-u,vfprintf
endif

## And any APP requires floating point printf
ifeq ($(APP), 0)
LIBS += -lprintf_min -lm
else
LIBS += -Wl,-u,vfprintf -lprintf_flt -lm
endif

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0

## Include Directories
INCLUDES =

## Objects that must be built in order to link
SRC = radio.c main.c hal_avr.c serial.c mac_event.c mac.c \
	mac_scan.c avr_timer.c mac_data.c mac_beacon.c \
	mac_start.c rum_application.c mac_associate.c mac_route.c \
	sensors.c sleep.c avr_sixlowpan.c avr_sixlowpan_application.c \
	sixlowpan_wake.c bootloader.c

## Source files for specific platforms
ifeq ($(PLATFORM), RAVENUSB)
SRC += ravenusb/usb_task.c ravenusb/cdc_task.c ravenusb/uart_usb_lib.c \
        ravenusb/usb_device_task.c ravenusb/usb_standard_request.c \
        ravenusb/usb_drv.c ravenusb/usb_descriptors.c \
        ravenusb/usb_specific_request.c
endif

OBJECTS = $(SRC:.c=.o)

## Objects explicitly added by the user
LINKONLYOBJECTS =

## Build
all: $(TARGET) $(PROJECT).hex $(PROJECT).eep $(PROJECT).lss $(PROJECT).noboot.bin size

## Compile: create object files from C source files.
.c.o:
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS:ravenusb/%=%) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TYPE)$(TARGET)
	@echo
	@echo Built $(TYPE)$(TARGET) with the following options:
	@echo PLATFORM=$(PLATFORM)
	@echo DEBUG=$(DEBUG)
	@echo DEMO=$(DEMO)
	@echo APP=$(APP)
	@echo RUMSLEEP=$(RUMSLEEP)
	@echo IPV6LOWPAN=$(IPV6LOWPAN)
	@echo SENSOR_TYPE=$(SENSOR_TYPE)
	@echo


%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $(TYPE)$< $(TYPE)$@

%.noboot.bin: $(TARGET)
	avr-objcopy -O binary $(HEX_FLASH_FLAGS) -R .bootloader $(TYPE)$< $(TYPE)$@

%.eep: $(TARGET)
	avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $(TYPE)$< $(TYPE)$@  || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $(TYPE)$< > $(TYPE)$@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} $(TYPE)${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) $(TYPE)$(PROJECT).elf dep/* dep $(TYPE)$(PROJECT).hex $(TYPE)$(PROJECT).eep $(TYPE)$(PROJECT).lss $(TYPE)$(PROJECT).map

## Help target
help:
	@echo
	@echo "There are several makefile options of interest:"
	@echo "DEBUG=[0 or 1] To turn debugging (serial support) on/off"
	@echo "DEMO=[0 or 1] To turn demo mode on/off"


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

