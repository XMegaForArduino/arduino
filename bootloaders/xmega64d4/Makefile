# Makefile for atxmega64d4 - invokes 'universal' Makefile in ../xmega
#
#

#USE_TARGET=Test64
#CPU_TARGET=atxmega64d4
USE_TARGET=xmega64d4
CPU_TARGET=at$(USE_TARGET)


xmega:
	$(MAKE) -C ../xmega xmega TARGET=$(USE_TARGET)
	cp -p ../xmega/xmegaBOOT_xmega_$(USE_TARGET).hex .
	cp -p ../xmega/xmegaBOOT_xmega0_$(USE_TARGET).hex .

flashboot:
	$(MAKE) -C ../xmega fusesxmega TARGET=$(USE_TARGET)
	avrdude -C ../xmega/avrdude.conf -c avrispmkii -p $(CPU_TARGET) -P usb -Uboot:w:xmegaBOOT_xmega0_$(USE_TARGET).hex:i -Ulockbits:w:0xBF:m

clean:
	$(MAKE) -C ../xmega clean_$(USE_TARGET) TARGET=$(USE_TARGET)


