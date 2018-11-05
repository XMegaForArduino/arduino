# Makefile for atxmega128a4u - invokes 'universal' Makefile in ../xmega
#
#

USE_TARGET=xmega128a4u
CPU_TARGET=at$(USE_TARGET)
LED_BUILTIN_PORT=PORTC
LED_BUILTIN_PIN=7

xmega:
	$(MAKE) -C ../xmega xmega TARGET=$(USE_TARGET) LED_BUILTIN_PORT=$(LED_BUILTIN_PORT) LED_BUILTIN_PIN=$(LED_BUILTIN_PIN)
	cp -p ../xmega/xmegaBOOT_xmega_$(USE_TARGET).hex .
	cp -p ../xmega/xmegaBOOT_xmega0_$(USE_TARGET).hex .

flashboot:
	$(MAKE) -C ../xmega fusesxmega TARGET=$(USE_TARGET)
	avrdude -C ../xmega/avrdude.conf -c avrispmkii -p $(CPU_TARGET) -P usb -Uboot:w:xmegaBOOT_xmega0_$(USE_TARGET).hex:i -Ulock:w:0xBF:m

#	avrdude -C ../xmega/avrdude.conf -c avrispmkii -p $(CPU_TARGET) -P usb -Uboot:w:xmegaBOOT_xmega0_$(USE_TARGET).hex:i -Ulockbits:w:0xBF:m



clean:
	$(MAKE) -C ../xmega clean_$(USE_TARGET) TARGET=$(USE_TARGET)


