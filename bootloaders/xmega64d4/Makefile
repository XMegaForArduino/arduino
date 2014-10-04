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
ISPTOOL = avrispmkii
ISPPORT	= usb
ISPSPEED = 

# added by me - conf file is 'special' for the xmega
ISPCONF = ./avrdude.conf


# TODO:  have multiple CPU-based targets that build the 'xmega' target but for a specific CPU
MCU_TARGET = atxmega64d4

# xmega64 bootloader starts at 10000H
LDSECTION  = --section-start=.text=0x10000


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


OBJ        = $(PROGRAM).o
#OBJ        += sp_driver.o
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


TARGET = xmega
AVR_FREQ = 32000000L

# this next part should be modified as needed
# I set the max time count to 1/8 the CPU frequency, 3 flashes on the LED,
# and flash baud rate fixed at 115200 (just like the Uno, actually)

CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>3' '-DNUM_LED_FLASHES=3' '-DWATCHDOG_MODS' '-DDOUBLE_SPEED'

# change this next part for a different baud rate (but why would you?)
CFLAGS += '-DBAUD_RATE=115200'


all:


xmega: $(PROGRAM)_xmega0.hex
	@echo building xmega


$(PROGRAM)_xmega.hex: $(PROGRAM)_xmega.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@


$(PROGRAM)_xmega.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)


#flashall: $(PROGRAM)_xmega_plus_image.hex fusesxmega
#	avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
#                -U flash:w:$(PROGRAM)_xmega_plus_image.hex -U lockbits:w:$(LOCKBITS_LOCK):m


$(PROGRAM)_xmega0.hex: $(PROGRAM)_xmega.hex
	grep -v ":020000021000EC" $(PROGRAM)_xmega.hex >$(PROGRAM)_xmega0.hex


flashboot: xmega fusesxmega
# in order to flash the boot loader I have to flash it as the "boot" section (no choice)
# since the hex file begins with an extended address record, if I were to simply TRIM THAT OFF
# then the atxmega64d4 will flash the bootloader correctly.
# TODO:  write a program to correctly manage this out of a combined file
	avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
                -Uboot:w:$(PROGRAM)_xmega0.hex:i -Ulockbits:w:$(LOCKBITS_LOCK):m


# this next target programs the fuses and leaves lock bits 'unlocked' for flashing
# it also performs a chip erase which normally includes the EEPROM as well
fusesxmega:
	while ! avrdude -C $(ISPCONF) -c $(ISPTOOL) -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) -e \
                        -Ulockbits:w:${LOCKBITS_UNLOCK}:m -Ufuse1:w:${FUSE1}:m -Ufuse2:w:${FUSE2}:m \
                        -Ufuse4:w:${FUSE4}:m -Ufuse5:w:${FUSE5}:m ; do sleep 1 ; done

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


