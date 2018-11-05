#!/bin/sh

if test -z "$1" ; then
  echo usage:  flash.xmega.sh [hexfile]
  exit
fi

/usr/local/bin/avrdude -C../xmega/avrdude.conf -v -v -v -v -v \
   -patxmega128a4u -cwiring -P/dev/ttyU0 -b115200 -D -Uflash:w:$1:i


