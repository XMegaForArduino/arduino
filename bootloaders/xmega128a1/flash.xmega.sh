#!/bin/sh

if test -z "$1" ; then
  echo usage:  flash.xmega.sh [hexfile]
  exit
fi

/usr/local/bin/avrdude -C./avrdude.conf -v -v -v -v -v \
   -patxmega128a1 -cwiring -P/dev/ttyU0 -b115200 -D -Uflash:w:$1:i


