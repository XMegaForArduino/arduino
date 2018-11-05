# Makefile for xmega bootloader for ATxmega64d4 processor
#
# (some comments from original retained for reference)
#
# Makefile for ATmegaBOOT
# E.Lins, 18.7.2005
# $Id: Makefile,v 1.4 2008/10/05 17:41:43 cvs Exp $
#

# program name should not be changed...
PROGRAM    = xmegaBOOT

# enter the parameters for the avrdude isp tool
ISPTOOL ?= avrispmkii
ISPPORT	?= usb
ISPSPEED ?= 

# the conf file is 'special' for the xmega and has more definitions in it
ISPCONF ?= ./avrdude.conf


# TARGET definition - sets up a bunch of stuff

TARGET ?=
BOOTLOADER_SUFFIX ?=
USE_STK500V2 ?=
GREP_LINE ?=

# some defaults
# AVR frequency - normally 32Mhz
AVR_FREQ ?= 32000000L

# max time count - how long to wait after blinking - for 'arduino' protocol
# the 'wiring' protocol will use a different default timeout
MAX_TIME_COUNT ?= F_CPU>>3

# LED flashes - default is 3
NUM_LED_FLASHES ?= 3


# known targets
# standard targets:
#    xmega16d4 xmega32d4 xmega64d4 xmega128d4
#    xmega16a1 xmega16a1u xmega32a1 xmega32a1u
#    xmega64a1 xmega64a1u xmega128a1 xmega128a1u
#    xmega64a4 xmega64a4u xmega128a4 xmega128a4u
#    xmega8e5 xmega16e5 xmega32e5
#  or one of the following custom targets
#    cc0_atxmega_e5 RockSolid128A

ifneq ($(strip $(TARGET)),)

# if the target name begins with 'xmega' it's a standard target

ifeq ($(shell echo $(TARGET) | awk '{ print substr($$0,1,5); }'),xmega)

TRGSFX:= $(shell echo $(TARGET) | awk '{ print substr($$0,6,99); }')
TRGMEM:= $(shell echo $(TRGSFX) | awk '{ A=substr($$0,length($$0),1); if(A=="u") print substr($$0,1,length($$0)-3); else print substr($$0,1,length($$0)-2);}')

# use TRGMEM to determine how to set up the start address
ifeq ($(strip $(TRGMEM)),8)
START_ADDR := 0x2000
endif
ifeq ($(strip $(TRGMEM)),16)
START_ADDR := 0x4000
endif
ifeq ($(strip $(TRGMEM)),32)
START_ADDR := 0x8000
endif
ifeq ($(strip $(TRGMEM)),64)
START_ADDR := 0x10000
GREP_LINE := ":020000021000EC"
endif
ifeq ($(strip $(TRGMEM)),128)
START_ADDR := 0x20000
USE_STK500V2 := 1
GREP_LINE := ":020000022000DC"
# decrease the maximum timeout
MAX_TIME_COUNT := F_CPU>>4
endif
ifeq ($(strip $(TRGMEM)),192)
START_ADDR := 0x30000
USE_STK500V2 := 1
# decrease the maximum timeout
MAX_TIME_COUNT := F_CPU>>4
endif
ifeq ($(strip $(TRGMEM)),256)
START_ADDR := 0x40000
USE_STK500V2 := 1
# decrease the maximum timeout
MAX_TIME_COUNT := F_CPU>>4
endif
ifeq ($(strip $(TRGMEM)),384)
START_ADDR := 0x60000
USE_STK500V2 := 1
# decrease the maximum timeout
MAX_TIME_COUNT := F_CPU>>4
endif

# others are pretty much the same for everything
MCU_TARGET := at$(TARGET)
SERIAL_PORT := PORTD
SERIAL_USART := USARTD0
SERIAL_PORT_REMAP := no
LED_BUILTIN_PORT := PORTR
LED_BUILTIN_PIN := 1

# END OF STANDARD TARGETS

else

# CUSTOM TARGETS supported by this project, not starting with 'xmega'

ifeq ($(TARGET),cc0_atxmega_e5)

MCU_TARGET := atxmega32e5
START_ADDR := 0x8000
SERIAL_PORT := PORTD
SERIAL_USART := USARTD0
SERIAL_PORT_REMAP := yes
LED_BUILTIN_PORT := PORTR
LED_BUILTIN_PIN := 1

#BOOTLOADER_SUFFIX=$(TARGET)

endif

ifeq ($(TARGET),RockSolid128A)

MCU_TARGET := atxmega128a1u
START_ADDR := 0x20000
SERIAL_PORT := PORTC
SERIAL_USART := USARTC0
SERIAL_PORT_REMAP := no
LED_BUILTIN_PORT := PORTQ
LED_BUILTIN_PIN := 3
# this one has more than 64k, so it needs STK500V2
USE_STK500V2 := 1
GREP_LINE := ":020000022000DC"
# increase the maximum timeout
MAX_TIME_COUNT := F_CPU>>2

#BOOTLOADER_SUFFIX=$(TARGET)

endif

ifeq ($(TARGET),Test128)

MCU_TARGET := atxmega128a1u
START_ADDR := 0x20000
SERIAL_PORT := PORTD
SERIAL_USART := USARTD0
SERIAL_PORT_REMAP := no
LED_BUILTIN_PORT := PORTR
LED_BUILTIN_PIN := 1
# note, NOT using STK500V2 in this one
GREP_LINE := ":020000022000DC"

endif

ifeq ($(TARGET),Test64)

MCU_TARGET := atxmega64d4
START_ADDR := 0x10000
SERIAL_PORT := PORTD
SERIAL_USART := USARTD0
SERIAL_PORT_REMAP := no
LED_BUILTIN_PORT := PORTR
LED_BUILTIN_PIN := 1
# testing STK500V2 with this one
USE_STK500V2 := 1
GREP_LINE := ":020000021000EC"

# increase the maximum timeout
MAX_TIME_COUNT := F_CPU>>2

endif

# anything else besides those 2 targets above will generate an error later

endif
# END OF BUILTIN STANDARD TARGETS AND CUSTOM TARGETS CHECK

else
# at this point 'TARGET' is blank, but user can specify other things directly

# default parameters.  specify non-defaults on command line
# certain ones are required.  

MCU_TARGET ?= help
START_ADDR ?=
SERIAL_PORT ?= PORTD
SERIAL_USART ?=
SERIAL_PORT_REMAP ?= no
LED_BUILTIN_PORT ?= PORTR
LED_BUILTIN_PIN ?= 1

endif


ifeq ($(strip $(SERIAL_USART)),)
# default to the 'n' in 'PORTn' as 'USARTn0'
ifneq ($(strip $(SERIAL_PORT)),)
SERIAL_USART := $(shell echo $(SERIAL_PORT) | awk '{ print "USART" substr($$0,5,1) "0"; }')
endif
endif

# suffix for boot image file name assigned here

ifeq ($(strip $(BOOTLOADER_SUFFIX)),)
ifeq ($(strip $(TARGET)),)
BOOTLOADER_SUFFIX := $(MCU_TARGET)_$(SERIAL_PORT)_$(SERIAL_PORT_REMAP)_$(LED_BUILTIN_PORT)_$(LED_BUILTIN_PIN)
else
BOOTLOADER_SUFFIX := $(TARGET)
endif
endif


# xmega64 bootloader starts at 10000H.  Others depend on CPU
LDSECTION  = --section-start=.text=$(START_ADDR)


#######################
# FUSES FUSES FUSES
#######################

# fuses are a little different for the xmega.  these values are derived from
# experimentation.  They do NOT lock any of the NVRAM sections after flashing

# xmega fuse assignments

# lockbits - must have BOTH bits 0 and 1 SET or you'll have to chip erase to get them back
# bit assignments:  11 = NOLOCK, 10 = WLOCK, 01 = RLOCK, 00 = RWLOCK (typically)
# bit 7:6 bootloader   bit 5:4 application   bit 3:2 app table   bit 1:0  programming interface
#
# to ONLY lock the bootloader, set LOCKBITS to 0xBF (bit 6 clear, all others set)
#LOCKBITS_LOCK=0xFF
LOCKBITS_LOCK=0xBF
LOCKBITS_UNLOCK=0xFF

# 4.13.1
# FUSEBYTE1 - watchdog timer bits - set to FF for now - see 9.7.1 and 9.7.2
FUSE1=0xFF

# 4.13.2
# FUSEBYTE2 - bit 6 = STARTUP (0 for bootloader, 1 for app address 0000)
#             bit 5 = TOSCSEL (set to 1, always - implies 'shared' when applicable)
#             bits 1:0 = sleep BOD mode/disable  11 to disable, 10 or 01 otherwise (see manual)
#11111111 = FF (bit 6 set)
#10111111 = BF (bit 6 clear)
FUSE2=0xBF


# 4.13.3
# FUSEBYTE4  bit 4 = reset disable (0 to disable, 1 = normal)
#            bits 3:2 = startup time delay (00 = 64, 01 = 4, 11 = 0)
#            bit 1 = watchdog timer config lock (0 to lock, 1 = unlocked)
#            all other bits should be 1 (note bit 0 may be read as a 0!)
FUSE4=0xFF


# 4.13.4
# FUSEBYTE5 bits 5:4 = (running) BOD mode/disable  11 to disable, 10 or 01 otherwise (see manual)
#           bit 3 = 'preserve EEPROM during chip erase' - 0 to NOT erase EEPROM, 1 to erase
#           bits 2:0 = BOD level - see table 8-2 - 000 = 1.6, 111 = 3.0 at 0.2v increments
#FUSE5=0xF8
#FUSE5=0xF0
FUSE5=0xEA
# EA enables undervolts with mode 10 (continuous) and setting at 2.6v
# TODO:  determine if this is the best setting to use


#/usr/local/bin/avrdude -C/home/zelgadis/boardformula/avrdude.conf -patxmega64d4 -cavrispmkII -Pusb -e \
#  -Ulockbits:w:${LOCKBITS_UNLOCK}:m -Ufuse1:w:${FUSE1}:m -Ufuse2:w:${FUSE2}:m \
#  -Ufuse4:w:${FUSE4}:m -Ufuse5:w:${FUSE5}:m

#/usr/local/bin/avrdude -C/home/zelgadis/boardformula/avrdude.conf -patxmega64d4 -cavrispmkII -Pusb \
#  -Uflash:w:${hex_file}:i -Ulockbits:w:${LOCKBITS_LOCK}:m


OBJ        = $(PROGRAM)_$(BOOTLOADER_SUFFIX).o
OPTIMIZE   = -O2

DEFS       = 
LIBS       =

CC         = avr-gcc

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
LDFLAGS       = -Wl,$(LDSECTION)
# Override is only needed by avr-lib build system.
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)



# this next part should be modified as needed
# I set the max time count to 1/8 the CPU frequency, 3 flashes on the LED,
# and flash baud rate fixed at 115200 (just like the Uno, actually)

CFLAGS += '-DMAX_TIME_COUNT=$(MAX_TIME_COUNT)' '-DNUM_LED_FLASHES=$(NUM_LED_FLASHES)' \
          '-DWATCHDOG_MODS' '-DDOUBLE_SPEED'

# stk500v2 protocol
ifneq ($(strip $(USE_STK500V2)),)
CFLAGS += '-DUSE_STK500V2'
endif

# change this next part for a different baud rate (but why would you?)
CFLAGS += '-DBAUD_RATE=115200'

# this next part incorporates the definitions from earlier

CFLAGS += '-DSERIAL_PORT=$(SERIAL_PORT)' '-DSERIAL_USART=$(SERIAL_USART)'

# do the 'negative' responses here
ifneq ($(strip $(SERIAL_PORT_REMAP)),)
ifeq ($(strip $(SERIAL_PORT_REMAP)),0)
else
ifeq ($(strip $(SERIAL_PORT_REMAP)),n)
else
ifeq ($(strip $(SERIAL_PORT_REMAP)),no)
else

ifeq ($(strip $(SERIAL_PORT_REMAP)),yes)
CFLAGS += '-DSERIAL_PORT_REMAP'
else
$(error you must specify SERIAL_PORT_REMAP correctly, was '"' $(SERIAL_PORT_REMAP) '"')
endif
endif
endif
endif
endif

ifneq ($(strip $(LED_BUILTIN_PORT)),)
CFLAGS += '-DLED_BUILTIN_PORT=$(LED_BUILTIN_PORT)' '-DLED_BUILTIN_PIN=$(LED_BUILTIN_PIN)'
endif

# this defines SERIAL_PORT SERIAL_PORT_REMAP LED_BUILTIN_PORT LED_BUILTIN_PIN
# with 'SERIAL_PORT_REMAP' being optional - test with #ifdef in code

# now do the sig bytes - that's the last of it

# OPTIONAL:  getting sig bytes from CPU type

SIG_BYTES := $(shell ./get_sig.sh $(MCU_TARGET))

ifneq ($(trim $(SIG_BYTES)),)

CFLAGS += "-DSIG1=$(shell echo $(SIG_BYTES) | awk '{ print $1; }')"
CFLAGS += "-DSIG2=$(shell echo $(SIG_BYTES) | awk '{ print $2; }')"
CFLAGS += "-DSIG3=$(shell echo $(SIG_BYTES) | awk '{ print $3; }')"

endif



all:
	echo $(CFLAGS)

xmega: $(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex
	@echo building xmega image for $(TARGET)


$(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex: $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@


$(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)


$(OBJ): $(PROGRAM).c
	$(CC) $(CFLAGS) -c -o $@ $^


clean_$(BOOTLOADER_SUFFIX): clean_fixhex
	-rm $(PROGRAM)_$(BOOTLOADER_SUFFIX).o
	-rm $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).elf
	-rm $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex
	-rm $(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex

#flashall: $(PROGRAM)_xmega_plus_image.hex fusesxmega
#	avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
#                -U flash:w:$(PROGRAM)_xmega_plus_image.hex -U lockbits:w:$(LOCKBITS_LOCK):m


ifneq ($(strip $(GREP_LINE)),)

$(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex: $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex
	grep -v $(GREP_LINE) $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex \
                >$(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex

clean_fixhex:


else

fixhex: fixhex.c
	cc -o fixhex fixhex.c

$(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex: $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex fixhex
	./fixhex $(PROGRAM)_xmega_$(BOOTLOADER_SUFFIX).hex $(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex

clean_fixhex:
	-rm fixhex

endif


flashboot: xmega fusesxmega
# in order to flash the boot loader I have to flash it as the "boot" section (no choice)
# since the hex file begins with an extended address record, if I were to simply TRIM THAT OFF
# then the atxmega64d4 will flash the bootloader correctly.
# TODO:  write a program to correctly manage this out of a combined file
	avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
                -Uboot:w:$(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex:i \
                -Ulock:w:$(LOCKBITS_LOCK):m

#                -Ulockbits:w:$(LOCKBITS_LOCK):m

flashboot_cmd:
	@echo avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
                -Uboot:w:$(PROGRAM)_xmega0_$(BOOTLOADER_SUFFIX).hex:i \
                -Ulock:w:$(LOCKBITS_LOCK):m

#                -Ulockbits:w:$(LOCKBITS_LOCK):m


# this next target programs the fuses and leaves lock bits 'unlocked' for flashing
# it also performs a chip erase which normally includes the EEPROM as well
fusesxmega:
	while ! avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) -e \
                        -Ulock:w:${LOCKBITS_UNLOCK}:m -Ufuse1:w:${FUSE1}:m -Ufuse2:w:${FUSE2}:m \
                        -Ufuse4:w:${FUSE4}:m -Ufuse5:w:${FUSE5}:m ; do sleep 1 ; done

#                        -Ulockbits:w:${LOCKBITS_UNLOCK}:m -Ufuse1:w:${FUSE1}:m -Ufuse2:w:${FUSE2}:m \

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex
	-rm -f fixhex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@


