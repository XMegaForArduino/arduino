# Makefile for atxmega128a1 - invokes 'universal' Makefile in ../xmega
#
#

USE_TARGET=Test128
CPU_TARGET=atxmega128a1u
#USE_TARGET=xmega128a1u
#CPU_TARGET=at$(USE_TARGET)

xmega:
	$(MAKE) -C ../xmega xmega TARGET=$(USE_TARGET)
	cp -p ../xmega/xmegaBOOT_xmega_$(USE_TARGET).hex .
	cp -p ../xmega/xmegaBOOT_xmega0_$(USE_TARGET).hex .

flashboot:
	$(MAKE) -C ../xmega fusesxmega TARGET=$(USE_TARGET)
	avrdude -C ../xmega/avrdude.conf -c avrispmkii -p $(CPU_TARGET) -P usb -Uboot:w:xmegaBOOT_xmega0_$(USE_TARGET).hex:i -Ulockbits:w:0xBF:m


