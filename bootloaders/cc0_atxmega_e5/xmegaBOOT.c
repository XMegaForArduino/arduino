/////////////////////////////////////////////////////////////////////////////////
// XMEGA BOOT LOADER - modified from the Arduino 1.05 bootloader ATmegaBOOT_xx8.c
//                     by Bob Frazier, S.F.T. Inc..  Some xmega-specific code was
//                     originally developed under contract for Boardformula, Inc.
//                     for their xmega-based device, with similar GPL licensing.
//    This program must be licensed under GPLv2 or later (see comments below)
/////////////////////////////////////////////////////////////////////////////////
// This bootloader has been tested with the following processors:
//
//   ATxmega64D4
//
/////////////////////////////////////////////////////////////////////////////////


// NOTE:  bootloader for ATxmega64D4 will be compiled for 10000H start address
//        this is 'the boot section' and MUST be flashed as 'boot' or it won't work
//        The easiest way to do this is to strip out the first line of the bootloader
//        hex file which will read as ":020000021000EC".
//        As far as I can tell, it only works for the 64D4.  A HEX bootloader for the
//        16D4, 32D4 or 128D4 may need to be 'relocated' programatically by modifying
//        the address entries themselves.  This is a problem with avrdude, and not
//        gcc nor this bootloader.  The boot section simply will NOT flash unless you
//        specify the section name as 'boot', and start at address 0000.


// XMEGA CHANGES:
//
// - apply equivalent fixes as the Adafruit bootloader
//   (blink, flash bypass, WATCHDOG_MODS)
// - special '115k baud only' baud rate code (only rate supported at this time)
// - built-in LED is on PORTR pin 1, optional LED on PORTR pin 0
// - serial port on PORTD (pins 2,3) only for flash
// - XMEGA-specific NVRAM write functions

// THIS file is 'special' since it maps PD6/7 to the serial port using PORTD_REMAP

//----------------------------------------------------------
// ORIGINAL COMMENT BLOCK FROM Arduino bootloader
// this is being retained for licensing reasons
/**********************************************************/
/* Serial Bootloader for Atmel megaAVR Controllers        */
/*                                                        */
/* tested with ATmega8, ATmega128 and ATmega168           */
/* should work with other mega's, see code for details    */
/*                                                        */
/* ATmegaBOOT.c                                           */
/*                                                        */
/*                                                        */
/* 20090308: integrated Mega changes into main bootloader */
/*           source by D. Mellis                          */
/* 20080930: hacked for Arduino Mega (with the 1280       */
/*           processor, backwards compatible)             */
/*           by D. Cuartielles                            */
/* 20070626: hacked for Arduino Diecimila (which auto-    */
/*           resets when a USB connection is made to it)  */
/*           by D. Mellis                                 */
/* 20060802: hacked for Arduino by D. Cuartielles         */
/*           based on a previous hack by D. Mellis        */
/*           and D. Cuartielles                           */
/*                                                        */
/* Monitor and debug functions were added to the original */
/* code by Dr. Erik Lins, chip45.com. (See below)         */
/*                                                        */
/* Thanks to Karl Pitrich for fixing a bootloader pin     */
/* problem and more informative LED blinking!             */
/*                                                        */
/* For the latest version see:                            */
/* http://www.chip45.com/                                 */
/*                                                        */
/* ------------------------------------------------------ */
/*                                                        */
/* based on stk500boot.c                                  */
/* Copyright (c) 2003, Jason P. Kyle                      */
/* All rights reserved.                                   */
/* see avr1.org for original file and information         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/* Target = Atmel AVR m128,m64,m32,m16,m8,m162,m163,m169, */
/* m8515,m8535. ATmega161 has a very small boot block so  */
/* isn't supported.                                       */
/*                                                        */
/* Tested with m168                                       */
/**********************************************************/

// some of this code was adapted from ATmel sample source and
// requires the following license information for distribution:
/*
Copyright (c) 2009 Atmel Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. The name of Atmel may not be used to endorse or promote products derived
from this software without specific prior written permission.

4. This software may only be redistributed and used in connection with an Atmel
AVR product.

THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/



// ---------------------------------------------------------------------------------
// NOTICE - No K&R style coding has been retained.  Allman style is used throughout
// ---------------------------------------------------------------------------------



/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// These function names were derived from equivalent functions supplied
// by ATmel in the 'SPM driver' sample for flashing the xmega
void SP_WaitForSPM(void);
void SP_EraseFlashBuffer(void);
void SP_LoadFlashPage(const uint8_t * data, uint16_t length);
void SP_WriteApplicationPage(uint32_t address);



/* Use the F_CPU and MAX_TIME_COUNT as defined in Makefile */

/* 20070707: hacked by David A. Mellis - after this many errors give up and launch application */
#define MAX_ERROR_COUNT 5

/* set the UART baud rate, default 115200 */
#ifndef BAUD_RATE
#define BAUD_RATE   115200
#endif


/* SW_MAJOR and MINOR needs to be updated from time to time to avoid warning message from AVR Studio */
/* never allow AVR Studio to do an update !!!! */
#define HW_VER   0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x10


// for these bit values see D manual section 11.12.15
// do NOT use 'INPUT_DISABLED' since it won't detect input level changes that way (it's for ADC pins)
#define PINCTRL_DEFAULT        (0)                            /* totem pole mode. trigger on 'both' */
#define PINCTRL_INPUT_PULLUP   (_BV(4) | _BV(3))              /* input pullup mode */
#define PINCTRL_AND_PULLUP     (_BV(5) | _BV(4) | _BV(3))     /* 'wired and' input pullup mode */


/* onboard LED is used to indicate, that the bootloader was entered (3x flashing) */
/* if monitor functions are included, LED goes on after monitor was entered */

// see also:  hardware/arduino/variants/standard/pins_arduino.h
//
// LED on PR1
#define LED_DDR  PORTR_DIR         /* D manual section 11.12.1 */
#define LED_PORT PORTR_OUT         /* D manual section 11.12.5 */
#define LED_PIN  PORTR_IN          /* D manual section 11.12.9 */
#define LED_CTRL PORTR_PIN1CTRL    /* D manual section 11.12.15 */
#define LED      1


// THIS SECTION LEFT FOR FUTURE REFERENCE - do we want 'monitor' functions on the xmega?
///* monitor functions will only be compiled when using ATmega128, due to bootblock size constraints */
//#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
//#define MONITOR 1
//#endif


/* define various device id's */
/* manufacturer byte is always the same */
// DEFAULT SIGNATURES (for now, hard-coded for the CPU, later use -D)
#define SIG1  0x1E  // Yep, Atmel is the only manufacturer of AVR micros.  Single source :(  [hey why is this a surprise? - BF]
#define SIG2  0x95
#define SIG3  0x4c  /* this is the sig byte that the Arduino environment looks for */
#define PAGE_SIZE 0x40U /* 64 words (128 bytes) for the ATXMega32E5 - see sect 8.12 in the ATXMega32E5 (etc) manual */



/* function prototypes */
void putch(char);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);
char gethex(void);
void puthex(char);
void soft_boot(void);
void smart_delay_ms(uint16_t ms);
void flash_led(uint8_t);

// TEMPORARY - until I have proper defintions for E5 I need this to make serial work

#ifndef __AVR_ATxmega32E5__

typedef struct E5USART_struct
{
    register8_t DATA;  /* Data Register */
    register8_t STATUS;  /* Status Register */
    register8_t CTRLA;  /* Control Register A */
    register8_t CTRLB;  /* Control Register B */
    register8_t CTRLC;  /* Control Register C */
    register8_t CTRLD;  /* Control Register D */
    register8_t BAUDCTRLA;  /* Baud Rate Control Register A */
    register8_t BAUDCTRLB;  /* Baud Rate Control Register B */
} E5USART_t;

#undef USARTD0
#define USARTD0    (*(E5USART_t *) 0x09C0)  /* Universal Synchronous/Asynchronous Receiver/Transmitter */

#endif // __AVR_ATxmega32E5__


/* some variables */
volatile union address_union
{
  uint16_t word;
  uint8_t  byte[2];
} address;

volatile union length_union
{
  uint16_t word;
  uint8_t  byte[2];
} length;

volatile struct flags_struct
{
  unsigned eeprom : 1;
  unsigned rampz  : 1;
} flags;

#ifdef __AVR_XMEGA__
uint8_t buff[512]; // page size 0x200 on XMEGA
#else // __AVR_XMEGA__
#error this is an xmega bootloader, you should only use it for XMEGA
uint8_t buff[256]; // old code (for reference)
#endif // __AVR_XMEGA__

uint8_t address_high;

uint8_t i;
uint8_t bootuart = 0;

uint8_t error_count = 0;


void (*app_start)(void) = 0x0000;


/* main program starts here */
int main(void)
{
  uint8_t ch,ch2,bod;
  uint16_t w;
  uint8_t firstchar = 0; // make sure we dont start bootloader by accident


#if defined(WATCHDOG_MODS)
  // this part needs to be done right away

  bod = RST_STATUS & 0x3f; // bits 6 and 7 aren't used
  RST_STATUS = 0x3f; // write 1 bits to clear everything (so I don't loop)

  // disable watchdog timer NOW

  CCP = CCP_IOREG_gc;// 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  WDT_CTRL = 1; // sets watchdog timer "enable" bit to zero - bit 0 must be set to change bit 1 - section 9.7.1
  CCP = CCP_IOREG_gc; // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  WDT_WINCTRL = 1; // sets watchdog 'window' timer "enable" bit to zero - bit 0 must be set to change bit 1 - section 9.7.2

  // the 'bod' value will be non-zero if I did a normal reboot, or had the WDT time out,
  // or had a brownout, or soft reboot, or hard reboot.  But if I merely jump here, it's
  // going to be zero, so I'll execute a soft reboot to make sure I'm not boot-looping
  // due to jumping into erased NVRAM and executing NOP instructions until I get here.
  // (that generally doesn't work very well since the hardware doesn't properly reset)

  if(!bod) // if 'bod' is zero, I executed this directly.  So to prevent boot-loops after a bad flash, do this
  {        // this forces a 'soft boot' which re-enters the bootloader normally, preventing a boot-loop
    soft_boot();
  }


  // NOW assign 'ch' to the watchdog bit (3) and bod to the B.O.D. bit (2)
  // TODO:  other flags?

  ch = bod & _BV(3); // watchdog bit, D manual sect 8.5.1
  bod &= _BV(2); // brown out detector (8.5.1)

  // TODO:  add other "skip the bootloader flash check" flags to this?  like soft boot?
  //        if I soft boot here, it might be an INTERNAL call to 'soft_boot' so another
  //        method will be needed to detect 'boot directly into the flash code'.
  // NOTE:  one candidate is a special EEPROM memory location, assigned to 'a function'
  //        this would deal with watchdog timers as well as soft booting into the bootloader
  //        to do something specific like writing the NVRAM.


  // ----------------------------
  // HANDLING BROWN-OUT DETECTION
  // ----------------------------

  // if the brown-out detector booted up, do NOT do the bluetooth reset, and make sure BT is de-energized.
  // this will help allow a 'very dead' battery to charge.  Problem is that when battery voltage is at
  // the minimum supported by the buck/boost converter, the current draw will be DOUBLE, which doesn't leave
  // a whole lot left for charging batteries.  In fact, doing a bluetooth data transfer can DRAIN the battery
  // completely.  As such, it might be necessary to check battery voltage before allowing an image to be flashed.

  // NEXT, set up the interrupt controller to disable interrupts
  // Important.  See 10.8.3 in D manual.  The startup code will need
  // to re-enable them in the application section for 'app IVT.

  CCP = CCP_IOREG_gc; // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  PMIC_CTRL = 0xc0; // binary 11000000  all 3 int levels disabled, round-robin enabled, IVT starts at 10002H

#else // old code

  asm volatile("nop\n\t");

#endif // WATCHDOG_MODS

//  if(bod)
//  {
//    goto skip_clock; // if I did the B.O.D. skip the clock stuff
//  }

  // -------------------------------------------------
  // The CLOCK section (all xmegas need this, really)
  // -------------------------------------------------

  // enable BOTH the 32Mhz and 32.768KHz internal clocks [ignore what else might be set for now]

  OSC_CTRL |= _BV(2) | _BV(1); // sect 6.10.1 - enable 32.768KHz (2), 32Mhz (1).  2Mhz is _BV(0)
                               //               _BV(3) and _BV(4) are for PLL and external - will set to 0 later

  if(!(CLK_LOCK & 1)) // clock lock bit NOT set, so I can muck with the clock
  {
    if((CLK_CTRL & 0x7) != 1) // it's not already 32 Mhz (D manual 6.9.1)
    {
      // wait until 32mhz clock is 'stable'

      for(w=32767; w > 0; w--) // TODO:  remove counter?
      {
        // spin on oscillator status bit for 32Mhz oscillator

        if(OSC_STATUS & _BV(1)) // 32Mhz oscillator is 'ready' (6.10.2)
        {
          break;
        }
      }

      // for now, I can allow the clock to NOT be changed if it's
      // not ready.  This prevents infinite loop inside startup code

      if(!(OSC_STATUS & _BV(1))) // is my oscillator 'ready' ?
      {
        goto skip_clock; // exit - don't change anything
      }

      // switch to 32Mhz clock using internal source

      CCP = CCP_IOREG_gc;// 0xd8 - see D manual, sect 3.14.1 (protected I/O)
      CLK_CTRL = 1; // set the clock to 32Mhz (6.9.1)
    }

    if(CLK_PSCTRL != 0)
    {
      CCP = CCP_IOREG_gc;// 0xd8 - see D manual, sect 3.14.1 (protected I/O)
      CLK_PSCTRL = 0; // set the clock divider(s) to 1:1 (6.9.2)
    }


    // now that I've changed the clock, disable 2Mhz, PLL, and external clocks
    // 32.768KHz should remain active, but I need to make sure it's stable
    OSC_CTRL &= ~(_BV(4) | _BV(3) | _BV(0)); // sect 6.10.1 - disable PLL, external, 2Mhz clocks

    // wait until 32.768KHz clock is 'stable'.  this one goes for a while
    // in case it doesn't stabilize in a reasonable time.  I figure about
    // 64*255 clock cycles should be enough, ya think?
    for(w=65535; w > 0; w--)
    {
      for(ch2=255; ch2 > 0; ch2--)
      {
        if(OSC_STATUS & _BV(2)) // 32.768KHz oscillator is 'ready' (6.10.2)
        {
          w = 1; // this will bail out of the outer loop
          break;
        }
      }
    }

    // enable DFLL auto-calibration of the 32Mhz internal oscillator
    // (it uses the reasonably precise 32.768KHz clock to do it)

    OSC_DFLLCTRL = 0; // sect 6.10.7 - select 32.768KHz osc for everything, basically
    DFLLRC32M_CTRL = 1; // set the bit to enable DFLL calibration - section 6.11.1
  }

  // I'll be using the 1.024khz clock (from the 32.768KHz clock) for the real-time counter
  // this will give me a reasonable "about 1 millisecond" accuracy on the RTC

  // NOTE:  I may not have checked for this if I skipped the previous section,
  //        so now I check again, just in case, to make sure the 32.768KHz osc is stable
  for(w=65535; w > 0; w--)
  {
    for(ch2=255; ch2 > 0; ch2--)
    {
      if(OSC_STATUS & _BV(2)) // 32.768KHz oscillator is 'ready' (6.10.2)
      {
        w = 1; // this will bail out of the outer loop
        break;
      }
    }
  }

  if(!(OSC_STATUS & _BV(2))) // is my oscillator 'ready' ?
  {
    goto skip_clock; // exit - don't change anything else
  }


  // RUN-TIME clock - use internal 1.024 khz source.  32khz needed for this
  CLK_RTCCTRL = 2; // section 6.9.4

skip_clock:  // go here if clock cannot be assigned for some reason or is already assigned


  // regular WATCHDOG_MODS code starts back up here

  // Check if the WDT was used to reset, in which case we dont bootload and skip straight to the code. woot.
  if (ch)   // if it's not an external reset... (in this case, JUST checking for watchdog, later BOD as well?)
  {
    app_start();  // skip bootloader (NOTE:  function call WAY smaller than goto)
  }

  /* check if flash is programmed already, if not start bootloader anyway */
  if(pgm_read_byte_near(0x0000) != 0xFF)
  {
    // TODO:  handle differently if programmed?
  }

  //------------------------------
  // UARTD0 config using HIGH pins
  //------------------------------

  // make sure that the HIGH pins are configured for the USART
#ifndef PORTD_REMAP // some headers may not have it
#define PORTD_REMAP  _SFR_MEM8(0x066E)
#endif // PORTD_REMAP

#if 1 // REMAP
  PORTD_REMAP = _BV(4); // see 12.13.13 in 'E' manual

  // PD6 (GPIO 6) must have input pullup resistor (for serial I/O)
  PORTD_PIN6CTRL = PINCTRL_INPUT_PULLUP;
  PORTD_OUT |= _BV(6);
  PORTD_DIR &= ~_BV(6);

  // PD7 (GPIO 7) must be configured as an output
  PORTD_PIN7CTRL = PINCTRL_DEFAULT;
  PORTD_OUT |= _BV(7);
  PORTD_DIR |= _BV(7);

#else  // REMAP

  PORTD_REMAP = 0; // see 12.13.13 in 'E' manual

  // PD2 (GPIO 2) must have input pullup resistor (for serial I/O)
  PORTD_PIN2CTRL = PINCTRL_INPUT_PULLUP;
  PORTD_OUT |= _BV(2);
  PORTD_DIR &= ~_BV(2);

  // PD7 (GPIO 7) must be configured as an output
  PORTD_PIN3CTRL = PINCTRL_DEFAULT;
  PORTD_OUT |= _BV(3);
  PORTD_DIR |= _BV(3);

#endif // REMAP


// set the CORRECT baud rate NOW.

#if BAUD_RATE == 115200

// section 19.4.4 - 'double speed' flag
#ifdef DOUBLE_SPEED
#define BAUD_SETTING  ((-2 << 12) | 135)
  // section 19.4.4
  (&USARTD0)->CTRLB = _BV(2); // enable clock 2x (everything else disabled)
#else // DOUBLE_SPEED
#define BAUD_SETTING ((-3 << 12) | 131)
  (&USARTD0)->CTRLB = 0; // DISable clock 2x
#endif // DOUBLE_SPEED


#else // BAUD_RATE != 115200

  // section 19.4.4 - 'double speed' flag
#ifdef DOUBLE_SPEED
  // section 19.4.4
  (&USARTD0)->CTRLB = _BV(2); // enable clock 2x (everything else disabled)
#else   // DOUBLE_SPEED
  (&USARTD0)->CTRLB = 0; // DISable clock 2x
#endif  // DOUBLE_SPEED



// for now only 115k supported, and error if I try to compile this
#error baud rate BAUD_RATE is NOT supported (for now)


#endif // BAUD_RATE == 115200

  // section 19.14.5 - USART mode, parity, bits
  // CMODE 7:6 = 00 [async]  PMODE 5:4 = 00 [none]  SBMODE 3 = 0 [1 bit]   CHSIZE 2:0 = 3 (8-bit)
  (&USARTD0)->CTRLC = 0x03; // SERIAL_8N1 - see HardwareSerial.h
  (&USARTD0)->CTRLD = 0; // always set to zero (E5 special)

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)

  (&USARTD0)->BAUDCTRLA = ((uint8_t)BAUD_SETTING) & 0xff;
  (&USARTD0)->BAUDCTRLB = (uint8_t)(BAUD_SETTING >> 8);

  // section 19.4.4
#ifdef DOUBLE_SPEED
  (&USARTD0)->CTRLB = _BV(2) | _BV(4) | _BV(3); // enable double-speed, RX, TX, and disable other stuff
#else   // DOUBLE_SPEED
  (&USARTD0)->CTRLB = _BV(4) | _BV(3); // enable RX, TX, disable other stuff
#endif  // DOUBLE_SPEED
  (&USARTD0)->CTRLA = 0; // NO! SERIAL! PORT! INTERRUPTS!


  sei();  // ok to enable global interrupts now


  // assign pin mode to LED output pin
  LED_CTRL = PINCTRL_DEFAULT;
  LED_DDR |= _BV(LED);

  /* flash onboard LED to signal entering of bootloader */
#if NUM_LED_FLASHES == 0
  ch2 = 3;
#else
  ch2 = NUM_LED_FLASHES;
#endif // NUM_LED_FLASHES

  while (ch2--) // inlined, may be smaller this way
  {
    LED_PORT |= _BV(LED);
    smart_delay_ms(100);
    LED_PORT &= ~_BV(LED);
    smart_delay_ms(100);
  }


  /* 20050803: by DojoCorp, this is one of the parts provoking the
     system to stop listening, cancelled from the original */
  //putch('\0');

  /* forever loop */
  for (;;)
  {

    /* get character from UART */
    ch = getch();

    /* A bunch of if...else if... gives smaller code than switch...case ! */

    /* Hello is anyone home ? */
    if(ch=='0')
    {
      firstchar = 1; // we got an appropriate bootloader instruction
      nothing_response();
    }
    else if (firstchar == 0)
    {
      // the first character we got is not '0', lets bail!
      // autoreset via watchdog (sneaky!)
      app_start(); // start the app (NOTE:  this is WAY smaller than using a goto)
      soft_boot();
    }


    /* Request programmer ID */
    /* Not using PROGMEM string due to boot block in m128 being beyond 64kB boundry  */
    /* Would need to selectively manipulate RAMPZ, and it's only 9 characters anyway so who cares.  */
    else if(ch=='1')
    {
      if (getch() == ' ')
      {
        putch(0x14);
        putch('A');
        putch('V');
        putch('R');
        putch(' ');
        putch('I');
        putch('S');
        putch('P');
        putch(0x10);
      }
      else
      {
        if (++error_count == MAX_ERROR_COUNT)
        {
          app_start();
          soft_boot();
        }
      }
    }


    /* AVR ISP/STK500 board commands  DON'T CARE so default nothing_response */
    else if(ch=='@')
    {
      ch2 = getch();

      if (ch2>0x85)
        getch();

      nothing_response();
    }


    /* AVR ISP/STK500 board requests */
    else if(ch=='A')
    {
      ch2 = getch();
      if(ch2==0x80)
      {
        byte_response(HW_VER);    // Hardware version
      }
      else if(ch2==0x81)
      {
        byte_response(SW_MAJOR);  // Software major version
      }
      else if(ch2==0x82)
      {
        byte_response(SW_MINOR);  // Software minor version
      }
      else if(ch2==0x98)
      {
        byte_response(0x03);      // Unknown but seems to be required by avr studio 3.56
      }
      else
      {
        byte_response(0x00);      // Covers various unnecessary responses we don't care about
      }
    }


    /* Device Parameters  DON'T CARE, DEVICE IS FIXED  */
    else if(ch=='B')
    {
      getNch(20);
      nothing_response();
    }


    /* Parallel programming stuff  DON'T CARE  */
    else if(ch=='E')
    {
      getNch(5);
      nothing_response();
    }


    /* P: Enter programming mode  */
    /* R: Erase device, don't care as we will erase one page at a time anyway.  */
    else if(ch=='P' || ch=='R')
    {
      nothing_response();
    }


    /* Leave programming mode  */
    else if(ch=='Q')
    {
      nothing_response();

      // CPU is too fast, and last byte gets corrupted
      // so wait for UART to actually send it before starting the app

      smart_delay_ms(50); // 50 msec should be enough

      app_start();
      soft_boot();
    }


    /* Set address, little endian. EEPROM in bytes, FLASH in words  */
    /* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
    /* This might explain why little endian was used here, big endian used everywhere else.  */
    else if(ch=='U')
    {
      address.byte[0] = getch();
      address.byte[1] = getch();
      nothing_response();
    }


    /* Universal SPI programming command, disabled.  Would be used for fuses and lock bits.  */
    else if(ch=='V')
    {
      if (getch() == 0x30)
      {
        getch();

        ch = getch();

        getch();

        if (ch == 0)
        {
          byte_response(SIG1);
        }
        else if (ch == 1)
        {
          byte_response(SIG2);
        }
        else
        {
          byte_response(SIG3);
        }
      }
      else
      {
        getNch(3);
        byte_response(0x00);
      }
    }


    /* Write memory, length is big endian and is in bytes  */
    else if(ch=='d')
    {
      length.byte[1] = getch();
      length.byte[0] = getch();
      flags.eeprom = 0;

      if (getch() == 'E')
      {
        flags.eeprom = 1;
      }

      for (w=0;w<length.word;w++)
      {
        buff[w] = getch();                          // Store data in buffer, can't keep up with serial data stream whilst programming pages
      }

      if (getch() == ' ')
      {
        if (flags.eeprom)
        {                       // Write to EEPROM one byte at a time
//          address.word <<= 1; // TODO:  is this right for EEPROM ?
          for(w=0;w<length.word;w++)
          {
            eeprom_write_byte((void *)address.word,buff[w]);
            address.word++;
          }
        }
        else
        {                   // Write to FLASH one page at a time
          if (length.byte[0] & 0x01)
            length.word++;  // Even up an odd number of bytes

          // XMEGA-ONLY CODE HERE

          SP_WaitForSPM();
          SP_EraseFlashBuffer();
          SP_WaitForSPM();
          SP_LoadFlashPage(&(buff[0]), length.word);
          SP_WaitForSPM();
          SP_WriteApplicationPage((uint32_t)address.word);
          SP_WaitForSPM();

          address.word += length.word; // compatibility
        }

        putch(0x14);
        putch(0x10);
      }
      else
      {
        if (++error_count == MAX_ERROR_COUNT)
        {
          app_start();
          soft_boot();
        }
      }
    }


    /* Read memory block mode, length is big endian.  */
    else if(ch=='t')
    {
      uint32_t addr;

      length.byte[1] = getch();
      length.byte[0] = getch();

      addr = ((uint32_t)address.word);

      if (getch() == 'E')
      {
        flags.eeprom = 1;
      }
      else
      {
        flags.eeprom = 0;
      }

      if (getch() == ' ')
      {                     // Command terminator
        putch(0x14);

        for (w=0;w < length.word;w++)
        {                                             // Can handle odd and even lengths okay
          if (flags.eeprom)
          {                                           // Byte access EEPROM read
            // TODO:  is this right for EEPROM ?
            putch(eeprom_read_byte((void *)address.word));

            address.word++;
          }
          else
          {
            SP_WaitForSPM();

            putch(pgm_read_byte_far(addr));

            // Hmmmm, yuck  FIXME when m256 arrvies
            addr++;
          }
        }

        if (!flags.eeprom)
        {
          address.word = addr; // for compatibility (do I need it?  is it right?)
        }

        putch(0x10);
      }
    }


    /* Get device signature bytes  */
    else if(ch=='u')
    {
      if (getch() == ' ')
      {
        putch(0x14);
        putch(SIG1);
        putch(SIG2);
        putch(SIG3);
        putch(0x10);
      }
      else
      {
        if (++error_count == MAX_ERROR_COUNT)
        {
          app_start();
          soft_boot();
        }
      }
    }


    /* Read oscillator calibration byte */
    else if(ch=='v')
    {
      byte_response(0x00);
    }


    else if (++error_count == MAX_ERROR_COUNT)
    {
      app_start();
      soft_boot();
    }
  } /* end of forever loop */

}

void soft_boot(void)
{
  CCP = CCP_IOREG_gc; // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  RST_CTRL = 0x1; // D manual section 8.5.2 - set the 'reset me' bit in the reset control reg

  while(1)
  { } // do nothing spinner
}

extern void __builtin_avr_delay_cycles(unsigned long); // see util/delay.h

void smart_delay_ms(uint16_t ms)
{
  while(ms)
  {
    ms--;

    __builtin_avr_delay_cycles(F_CPU / 1000); // delay approximately 1 msec
  }
}

char gethexnib(void)
{
  char a;

  a = getch(); putch(a);

  if(a >= 'a')
  {
    return (a - 'a' + 0x0a);
  }
  else if(a >= '0')
  {
    return(a - '0');
  }

  return a;
}


char gethex(void)
{
  return (gethexnib() << 4) + gethexnib();
}


void puthex(char ch)
{
  char ah;

  ah = ch >> 4;
  if(ah >= 0x0a)
  {
    ah = ah - 0x0a + 'a';
  }
  else
  {
    ah += '0';
  }

  ch &= 0x0f;
  if(ch >= 0x0a)
  {
    ch = ch - 0x0a + 'a';
  }
  else
  {
    ch += '0';
  }

  putch(ah);
  putch(ch);
}


void putch(char ch)
{

  while (!((&USARTD0)->STATUS & _BV(5))) // bit 5 is the DRE bit (6 is the 'transmitted' bit)
  { } // wait for DRE flag

  (&USARTD0)->DATA = ch;
}


char getch(void)
{
  uint32_t count = 0;

  LED_PORT &= ~_BV(LED);          // turn off the LED to indicate receiving data

  while(!((&USARTD0)->STATUS & _BV(7))) // wait for RX data
  {
    /* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/
    /* HACKME:: here is a good place to count times*/
    count++;

    if (count > (uint32_t)(MAX_TIME_COUNT)) // delay period for flashing
    {
      app_start();
      soft_boot();
    }
  }

  LED_PORT |= _BV(LED);          // turn on the LED to indicate receiving data

  return (&USARTD0)->DATA;
}


void getNch(uint8_t count)
{
  while(count--)
  {
    getch();
  }
}


void byte_response(uint8_t val)
{
  if (getch() == ' ')
  {
    putch(0x14);
    putch(val);
    putch(0x10);
  }
  else
  {
    if (++error_count == MAX_ERROR_COUNT)
    {
      app_start();
      soft_boot();
    }
  }
}


void nothing_response(void)
{
  if (getch() == ' ')
  {
    putch(0x14);
    putch(0x10);
  }
  else
  {
    if (++error_count == MAX_ERROR_COUNT)
    {
      app_start();
      soft_boot();
    }
  }
}

void SP_WaitForSPM(void)
{
//  lds r16, NVM_STATUS     ; Load the NVM Status register.
//  sbrc  r16, NVM_NVMBUSY_bp ; Check if bit is cleared.
//  rjmp  SP_WaitForSPM       ; Repeat check if bit is not cleared.
//  clr r16
//  sts NVM_CMD, r16        ; Clear up command register to NO_OPERATION.
//  ret

  while(NVM_STATUS & _BV(NVM_NVMBUSY_bp))
  { } // wait for NVM status to NOT be busy

  NVM_CMD = 0; // clear command register to NO_OPERATION
}

// NOTE:  for registers
// X=R27:R26, Y=R29:R28 and Z=R31:R30
// RAMPX RAMPY and RAMPZ extend X, Y, and Z

// 'SP_CommonSPM' - a jump point in the ATmel code, implemented
//                  as inline assembler here

// see gcc documentation "Assembler Instructions with C Expression Operands"
// self-notes, '%C1' is byte offset 2 for '%1' ('%A1' is byte offset 0, first byte)

#define SP_CommonSPM(cmd,addr)               \
  (__extension__(                            \
    {                                        \
      uint32_t __addr32 = (uint32_t)(addr);  \
      uint8_t __cmd = (uint8_t)(cmd);        \
      __asm__                                \
      (                                      \
        "push r31" "\n\t"                    \
        "push r30" "\n\t"                    \
        "push r16" "\n\t"                    \
        "push r18" "\n\t"                    \
        "in r18, %2" "\n\t"                  \
        "out %2, %C1" "\n\t"                 \
        "movw r30, %1" "\n\t"                \
        "sts %3, %0" "\n\t"                  \
        "ldi r16, %5" "\n\t"                 \
        "sts %4, r16" "\n\t"                 \
        "spm" "\n\t"                         \
        "out %2, r18" "\n\t"                 \
        "pop r18" "\n\t"                     \
        "pop r16" "\n\t"                     \
        "pop r30" "\n\t"                     \
        "pop r31" "\n\t"                     \
        : /* no output */                    \
        : "r" (__cmd),                       \
          "r" (__addr32),                    \
          "I" (_SFR_IO_ADDR(RAMPZ)),         \
          "m" (NVM_CMD),                     \
          "m" (CCP),                         \
          "M" (CCP_SPM_gc)                   \
        : "r16", "r18", "r30", "r31"         \
      );                                     \
    }))

//SP_CommonSPM:
//  movw  ZH:ZL, r17:r16   ; Load R17:R16 into Z.
//  sts NVM_CMD, r20     ; Load prepared command into NVM Command register.
//  ldi r16, CCP_SPM_gc  ; Prepare Protect SPM signature in R16.
//  sts CCP, r16         ; Enable SPM operation (this disables interrupts for 4 cycles).
//  spm                      ; Self-program.
//  out RAMPZ, r23
//  ret



void SP_EraseFlashBuffer(void)
{
  NVM_INTCTRL = 0; // always do first (disable interrupts)
  SP_CommonSPM(NVM_CMD_ERASE_FLASH_BUFFER_gc, 0);
}

void do_one_load_flash_page_loop(uint16_t iCtr0, uint8_t bValL0, uint8_t bValH0)
{
register uint8_t bValL = bValL0, bValH = bValH0;
register uint16_t iCtr = iCtr0;

  (__extension__(
  {
    __asm__
    (
      "push r31" "\r\n"
      "push r30" "\r\n"
      "push r1" "\r\n"
      "push r0" "\r\n"
      "push r18" "\r\n"
      "in r18, %0" "\n\t"      // preserve RAMPZ
      "push r18" "\n\t"
      "eor r18,r18" "\n\t"
      "out %0, r18" "\n\t"    // RAMPZ = 0
      "movw r30, %3" "\n\t"   // iCtr --> Z
      "mov r0, %1" "\n\t"     // pD[0] --> r0
      "mov r1, %2" "\n\t"     // pD[1] --> r1
      "ldi r18, %5" "\n\t"    // NVM_CMD_LOAD_FLASH_BUFFER_gc
      "sts %4, r18" "\n\t"    // NVM_CMD 'load flash buffer'
      "ldi r18, %7" "\n\t"    // CCP_SPM_gc
      "sts %6, r18" "\n\t"    // CCP = CCP_SPM_gc
      "spm" "\n\t"            // SPM function
      "pop r18" "\n\t"
      "out %0, r18" "\n\t"    // restore RAMPZ
      "pop r18" "\n\t"
      "pop r0" "\n\t"
      "pop r1" "\n\t"
      "pop r30" "\n\t"
      "pop r31" "\n\t"
      : /* no return */
      :   "I" (_SFR_IO_ADDR(RAMPZ)),
          "r" (bValL),  // word value, goes into R1:R0
          "r" (bValH),
          "r" (iCtr),  // counter/offset, goes into R31:R30
          "m" (NVM_CMD),
          "M" (NVM_CMD_LOAD_FLASH_BUFFER_gc),
          "m" (CCP),
          "M" (CCP_SPM_gc)
      : "r0","r1","r18","r30","r31"
    );
  }));

}

void SP_LoadFlashPage(const uint8_t * data, uint16_t length)
{
register uint16_t iCtr;
register const uint8_t *pD = data;

  NVM_INTCTRL = 0; // always do first (disable interrupts)

  // written for minimal assembly and as much C code as possible

  if(length & 1)  // odd # of bytes?
  {
    length++;
  }

  if(!length)
  {
    return; // don't do anything (zero length)
  }

  for(iCtr = 0; iCtr < length; iCtr += 2)
  {
    uint8_t bValL = *(pD++);
    uint8_t bValH = *(pD++);

    do_one_load_flash_page_loop(iCtr, bValL, bValH);
  }


// original assembly code from ATmel

//SP_LoadFlashPage:
//  clr ZL              ; Clear low byte of Z, to indicate start of page.
//  clr ZH              ; Clear high byte of Z, to indicate start of page.
//  movw  r3:r2, XH:XL    ; Save X to R3:R2 for later restore.
//
//  ldi r18, 0x00
//  out RAMPX, r18      ; Clear RAMPX pointer.
//  movw  XH:XL, r17:r16  ; Load X with data buffer address.
//
//  ldi   r20, NVM_CMD_LOAD_FLASH_BUFFER_gc  ; Prepare NVM command code in R20.
//  sts NVM_CMD, r20                       ; Load it into NVM command register.
//
//#if FLASH_PAGE_SIZE > 512
//  ldi r22, ((FLASH_PAGE_SIZE/2) >> 8)
//#endif
//  ldi r21, ((FLASH_PAGE_SIZE/2)&0xFF)    ; Load R21 with page word count.
//  ldi r16, CCP_SPM_gc                    ; Prepare Protect SPM signature in R16.
//
//SP_LoadFlashPage_1:
//  ld  r0, X+         ; Load low byte from buffer into R0.
//  ld  r1, X+         ; Load high byte from buffer into R1.
//  sts CCP, r16       ; Enable SPM operation (this disables interrupts for 4 cycles).
//  spm                    ; Self-program.
//  adiw  ZH:ZL, 2       ; Move Z to next Flash word.
//
//#if FLASH_PAGE_SIZE > 512
//  subi  r21, 1         ; Decrement word count.
//  sbci  r22, 0
//#else
//  dec r21            ; Decrement word count.
//#endif
//
//  brne  SP_LoadFlashPage_1   ; Repeat until word cont is zero.
//
//  movw  XH:XL, r3:r2         ; Restore old X from R3:R2.
//  ret


}

void SP_WriteApplicationPage(uint32_t address)
{
  NVM_INTCTRL = 0; // always do first (disable interrupts)

//  SP_CommonSPM(NVM_CMD_WRITE_APP_PAGE_gc, address);
  SP_CommonSPM(NVM_CMD_ERASE_WRITE_APP_PAGE_gc, address);
}



/* end of file ATmegaBOOT.c */

