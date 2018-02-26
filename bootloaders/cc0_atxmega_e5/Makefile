# Makefile for atxmega32e5 - invokes 'universal' Makefile in ../xmega
#
#

USE_TARGET=cc0_atxmega_e5
CPU_TARGET=atxmega32e5

# this board re-maps the serial port pins
CFLAGS += -DSERIAL_PORT_REMAP


xmega:
	$(MAKE) -C ../xmega xmega TARGET=$(USE_TARGET)
	cp -p ../xmega/xmegaBOOT_xmega_$(USE_TARGET).hex .
	cp -p ../xmega/xmegaBOOT_xmega0_$(USE_TARGET).hex .

flashboot:
	$(MAKE) -C ../xmega fusesxmega TARGET=$(USE_TARGET)
	avrdude -C ../xmega/avrdude.conf -c avrispmkii -p $(CPU_TARGET) -P usb -Uboot:w:xmegaBOOT_xmega0_$(USE_TARGET).hex:i -Ulockbits:w:0xBF:m



clean:
	$(MAKE) -C ../xmega clean_$(USE_TARGET) TARGET=$(USE_TARGET)


