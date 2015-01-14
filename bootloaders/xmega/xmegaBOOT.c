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


// XMEGA CHANGES:
//
// - apply equivalent fixes as the Adafruit bootloader does
//   (blink, flash bypass, WATCHDOG_MODS)
// - special '115k baud only' baud rate code (only rate supported at this time)
// - by default, built-in LED is on PORTR pin 1
// - also by default, the serial port is on PORTD (pins 2,3) only for flashing
// - XMEGA-specific NVRAM write functions

// THIS is the _GENERIC_ version.  It requires some definitions to work properly

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

// for reading NVM data directly (a convenience function)
uint8_t readNVMData(uint8_t cmd, uint16_t iIndex);


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
#ifdef LED_BUILTIN_PORT

#define LED_DDR ((&(LED_BUILTIN_PORT))->DIR)
// NOTE:  if I don't cast the address as 'volatile uint8_t *' some processors won't work properly
#define LED_PORT (*((volatile uint8_t *) &((&(LED_BUILTIN_PORT))->OUT) ))
#define LED_PIN (*((volatile uint8_t *) &((&(LED_BUILTIN_PORT))->IN) ))
#define LED_CTRL (*((volatile uint8_t *) &(*(&((&(LED_BUILTIN_PORT))->PIN0CTRL) + LED_BUILTIN_PIN)) ))
#define LED_PIN_BIT _BV(LED_BUILTIN_PIN)
#define LED_INTCTRL (*((volatile uint8_t *) &((&(LED_BUILTIN_PORT))->INTCTRL) ))

#else // LED_BUILTIN_PORT

// LED on PR1
#define LED_DDR  PORTR_DIR         /* D manual section 11.12.1 */
#define LED_PORT PORTR_OUT         /* D manual section 11.12.5 */
#define LED_PIN  PORTR_IN          /* D manual section 11.12.9 */
#define LED_CTRL PORTR_PIN1CTRL    /* D manual section 11.12.15 */
#define LED_PIN_BIT _BV(1)
#define LED_INTCTRL PORTR_INTCTRL

#endif // LED_BUILTIN_PORT


// SERIAL PORT AND REMAP REGISTER

#ifdef PORTC_REMAP /* does it exist? */
#define SERIAL_PORT_REMAP_REG ((&(SERIAL_PORT))->REMAP)
#endif // PORTC_REMAP

#ifdef USARTC0_CTRLD
#define SERIAL_USART_CTRLD  (*((volatile uint8_t *) &((&(SERIAL_USART))->CTRLD) ))
#endif // USARTC0_CTRLD


#if 0
// legacy testing code, currently commented out
// NOTE:  if I don't cast the address as 'volatile uint8_t *' some processors won't work properly
#define SERIAL_PORT_PIN2CTRL (*((volatile uint8_t *) &((&(SERIAL_PORT))->PIN2CTRL) ))
#define SERIAL_PORT_PIN3CTRL (*((volatile uint8_t *) &((&(SERIAL_PORT))->PIN3CTRL) ))
#define SERIAL_PORT_PIN6CTRL (*((volatile uint8_t *) &((&(SERIAL_PORT))->PIN6CTRL) ))
#define SERIAL_PORT_PIN7CTRL (*((volatile uint8_t *) &((&(SERIAL_PORT))->PIN7CTRL) ))
#define SERIAL_PORT_OUT      (*((volatile uint8_t *) &((&(SERIAL_PORT))->OUT) ))
#define SERIAL_PORT_DIR      (*((volatile uint8_t *) &((&(SERIAL_PORT))->DIR) ))

#define SERIAL_USART_STATUS (*((volatile uint8_t *) &((&(SERIAL_USART))->STATUS) ))
#define SERIAL_USART_DATA   (*((volatile uint8_t *) &((&(SERIAL_USART))->DATA) ))
#define SERIAL_USART_CTRLA  (*((volatile uint8_t *) &((&(SERIAL_USART))->CTRLA) ))
#define SERIAL_USART_CTRLB  (*((volatile uint8_t *) &((&(SERIAL_USART))->CTRLB) ))
#define SERIAL_USART_CTRLC  (*((volatile uint8_t *) &((&(SERIAL_USART))->CTRLC) ))
#define SERIAL_USART_BAUDCTRLA (*((volatile uint8_t *) &((&(SERIAL_USART))->BAUDCTRLA) ))
#define SERIAL_USART_BAUDCTRLB (*((volatile uint8_t *) &((&(SERIAL_USART))->BAUDCTRLB) ))

#else

#define SERIAL_PORT_PIN2CTRL PORTD_PIN2CTRL
#define SERIAL_PORT_PIN3CTRL PORTD_PIN3CTRL
#define SERIAL_PORT_PIN6CTRL PORTD_PIN6CTRL
#define SERIAL_PORT_PIN7CTRL PORTD_PIN7CTRL
#define SERIAL_PORT_DIR PORTD_DIR
#define SERIAL_PORT_OUT PORTD_OUT


#define SERIAL_USART_STATUS USARTD0_STATUS
#define SERIAL_USART_DATA USARTD0_DATA
#define SERIAL_USART_CTRLA USARTD0_CTRLA
#define SERIAL_USART_CTRLB USARTD0_CTRLB
#define SERIAL_USART_CTRLC USARTD0_CTRLC
#define SERIAL_USART_BAUDCTRLA USARTD0_BAUDCTRLA
#define SERIAL_USART_BAUDCTRLB USARTD0_BAUDCTRLB

#endif 



// THIS SECTION LEFT FOR FUTURE REFERENCE - do we want 'monitor' functions on the xmega?
///* monitor functions will only be compiled when using ATmega128, due to bootblock size constraints */
//#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
//#define MONITOR 1
//#endif


/* define various device id's */
/* manufacturer byte is always the same */
// DEFAULT SIGNATURES (for now, hard-coded for the CPU, later use -D)
//#define SIG1  0x1E  // Yep, Atmel is the only manufacturer of AVR micros.  Single source :(  [hey why is this a surprise? - BF]
//#define SIG2  0x95
//#define SIG3  0x4c  /* this is the sig byte that the Arduino environment looks for */

#ifndef SIG1
#define SIG1 SIGNATURE_0 /* from the header file */
#endif // SIG1
#ifndef SIG2
#define SIG2 SIGNATURE_1
#endif // SIG2
#ifndef SIG3
#define SIG3 SIGNATURE_2
#endif // SIG3

#define SIGNATURE_BYTES ( ((unsigned long)SIG1 << 16) | ((unsigned long)SIG2 << 8) | (unsigned long)SIG3 )


// use APP_SECTION_PAGE_SIZE as-is instead
// use conditional so I can assign it with a '-D' in the compiler command
#ifndef PAGE_SIZE
#define PAGE_SIZE APP_SECTION_PAGE_SIZE
#endif // PAGE_SIZE

// OLD CODE FOR REFERENCE
//// NOTE that page size is expressed in WORDS, not bytes, so must divide byte size by 2
//#ifndef PAGE_SIZE
//#define PAGE_SIZE (APP_SECTION_PAGE_SIZE / 2) /*0x40U*/ /* 64 words (128 bytes) for the ATXMega32E5 - see sect 8.12 in the ATXMega32E5 (etc) manual */
//#endif // PAGE_SIZE


/* function prototypes */
void putch(char);
void putstr(unsigned char *pBuf, char nBytes);
void flush(void);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);
char gethex(void);
void puthex(char);
void soft_boot(void);
char smart_delay_ms(uint16_t ms);

#ifdef USE_STK500V2
void flushin(void);
uint8_t do_getch_and_checksum(void);
void do_putch_and_checksum(uint8_t chPut);
#endif USE_STK500V2


// DEBUG STUFF
//#define ENABLE_BANG /* for STK500v2, enable '!' command to test serial port */




/* some variables */
#ifdef USE_STK500V2

#if defined(RAMPZ)
typedef uint32_t address_t;
#else
typedef uint16_t address_t;
#endif

address_t address = 0;
static uint8_t checksum; // used by 'do_XXXch_and_checksum'

#else // USE_STK500V2

volatile union address_union
{
#ifdef RAMPZ
  uint32_t dword;
#endif // RAMPZ
  uint16_t word;
#ifdef RAMPZ
  uint8_t  byte[4];
#else // RAMPZ
  uint8_t  byte[2];
#endif // RAMPZ
} address;

#endif // USE_STK500V2

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

#ifndef __AVR_XMEGA__
#error this is an xmega bootloader, you should only use it for XMEGA
#endif // __AVR_XMEGA__


#ifdef USE_STK500V2

#if PAGE_SIZE < 256
static unsigned char msgBuffer[288]; // the stk500boot.c defined it as 285
#else // PAGE_SIZE >= 256
static unsigned char msgBuffer[PAGE_SIZE + 32]; // 32 extra bytes
#endif // PAGE_SIZE <, >= 256

// other vars used solely for STK500V2
static unsigned char seqNum;// = 0;
static unsigned int msgLength;// = 0;

#else // USE_STK500V2

// use PAGE_SIZE for buffer size - usually assigned to APP_SECTION_PAGE_SIZE
#if PAGE_SIZE < 256
uint8_t buff[258]; // min size
#else // PAGE_SIZE >= 256
uint8_t buff[PAGE_SIZE + 2]; // was 512 - largest page size 0x200 on XMEGA
#endif // PAGE SIZE <, >= 256

#endif // USE_STK500V2

uint8_t error_count;


/////////////////////////////////////////////////////////////////////////////////////////////////////
// APP START - different definitions for >64K NVRAM and <=64K

#if defined(EIND) && defined(__AVR_3_BYTE_PC__) // need to re-do the way we call the application

void app_start(void)
{
  // first, assign 0 to EIND (I'm jumping to 128k-page 0)
  __asm volatile ("ldi r24,0\n\t"
                  "out %i0,r24" :: "n" (&EIND) : "memory");

  // next, push 3 zeros onto the stack and do a return (it's a 3-byte PC so this should work)
  // I check for '__AVR_3_BYTE_PC__' earlier
  __asm volatile ("ldi r30,0\n\t"
                  "push r30\n\t"
                  "push r30\n\t"
                  "push r30\n\t"
                  "ret\r\n" ::: "memory");

  // NOTE:  the ret address for the function that called THIS one should be ok on the stack
}

// NOTE:  this is in main.c now, in the core
//// for info on THIS thing, see http://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
//static void __attribute__((section(".init3"),naked,used,no_instrument_function)) init3_set_eind (void)
//{
//  __asm volatile ("ldi r24,pm_hh8(__trampolines_start)\n\t"
//                  "out %i0,r24" :: "n" (&EIND) : "r24","memory");
//}

#else // no EIND or 3-byte PC, we're fine

void (*app_start)(void) = 0x0000; // here it's a function pointer with an address of 0000H

#endif // EIND+3-byte PC vs "not"

/////////////////////////////////////////////////////////////////////////////////////////////////////


/* main program starts here */
int main(void)
{
uint8_t ch, ch2, bod;
uint16_t w1;
uint8_t firstchar;
#ifdef USE_STK500V2
unsigned char *p1;
#endif // USE_STK500V2


//#if defined(WATCHDOG_MODS)  ALWAYS, now

  // TODO:  create a stack frame?  some CPUs may not do this right, from what I have seen

  if(SPL != (uint8_t)((INTERNAL_SRAM_START + INTERNAL_SRAM_SIZE) & 0xff) ||
     SPH != (uint8_t)(((INTERNAL_SRAM_START + INTERNAL_SRAM_SIZE) >> 8) & 0xff))
  {
    // TODO:  I need to set the stack up...
  }

  // this part needs to be done right away

  bod = RST_STATUS & 0x3f; // bits 6 and 7 aren't used according to the manual (note bit 6 is defined as RST_SDRF_bm in nearly every header)
#ifdef RST_SDRF_bm
  RST_STATUS = 0x7f; // bit 6 also
#else // RST_SDRF_bm
  RST_STATUS = 0x3f; // write 1 bits to clear everything (so I don't loop)
#endif // RST_SDRF_bm

  // disable watchdog timer NOW

  CCP = CCP_IOREG_gc;        // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  WDT_CTRL = WDT_CEN_bm;     // sets watchdog timer "enable" bit to zero - bit 0 must be set to change bit 1 - section 9.7.1
  CCP = CCP_IOREG_gc;        // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  WDT_WINCTRL = WDT_WCEN_bm; // sets watchdog 'window' timer "enable" bit to zero - bit 0 must be set to change bit 1 - section 9.7.2

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
  // TODO:  other flags?  I might want to do a soft boot into a special feature, like bootloader flashing

  ch = bod & RST_WDRF_bm /*_BV(3)*/; // watchdog bit, D manual sect 8.5.1
  bod &= RST_BORF_bm /*_BV(2)*/; // brown out detector (8.5.1)

  // TODO:  add other "skip the bootloader flash check" flags to this?  like soft boot?
  //        if I soft boot here, it might be an INTERNAL call to 'soft_boot' so another
  //        method will be needed to detect 'boot directly into the flash code'.
  // NOTE:  one candidate is a special EEPROM memory location, assigned to 'a function'
  //        this would deal with watchdog timers as well as soft booting into the bootloader
  //        to do something specific like writing the NVRAM.


  // ----------------------------
  // HANDLING BROWN-OUT DETECTION
  // ----------------------------

  // TODO: special brown-out detect code

  // NEXT, set up the interrupt controller to disable interrupts
  // Important.  See 10.8.3 in D manual.  The startup code will need
  // to re-enable them in the application section for 'app IVT.

  CCP = CCP_IOREG_gc; // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
  PMIC_CTRL = 0xc0; // binary 11000000  all 3 int levels disabled, round-robin enabled, IVT starts at 10002H

//#else // old code
//
//  asm volatile("nop\n\t");
//
//#endif // WATCHDOG_MODS

  if(bod)
  {
//  do something special if brown-out detected, like keeping the clock slow
//  goto skip_clock; // if I did the B.O.D. skip the clock stuff (this is a battery system strategy)
  }

  // -------------------------------------------------
  // The CLOCK section (all xmegas need this, really)
  // -------------------------------------------------

  // enable BOTH the 32Mhz and 32.768KHz internal clocks [ignore what else might be set for now]

//  OSC_CTRL |= _BV(2) | _BV(1); // sect 6.10.1 - enable 32.768KHz (2), 32Mhz (1).  2Mhz is _BV(0)
//                               //               _BV(3) and _BV(4) are for PLL and external - will set to 0 later

  OSC_CTRL |= OSC_RC32KEN_bm | OSC_RC32MEN_bm; // enable 32khz osc, 32mhz osc - 'D4' manual sect 6.10.1

  if(!(CLK_LOCK & CLK_LOCK_bm/*1*/)) // clock lock bit NOT set, so I can muck with the clock
  {                   // note that in the bootloader this should NEVER happen.  check anyway.

    if((CLK_CTRL & CLK_SCLKSEL_gm/*7*/) != CLK_SCLKSEL_RC32M_gc/*1*/) // it's not already 32 Mhz (D manual 6.9.1)
    {
      // wait until 32mhz clock is 'stable'

      for(w1=32767; w1 > 0; w1--) // TODO:  remove counter?
      {
        // spin on oscillator status bit for 32Mhz oscillator

        if(OSC_STATUS & OSC_RC32MRDY_bm/*_BV(1)*/) // 32Mhz oscillator is 'ready' (6.10.2)
        {
          break;
        }
      }

      // for now, I can allow the clock to NOT be changed if it's
      // not ready.  This prevents infinite loop inside startup code

      if(!(OSC_STATUS & OSC_RC32MRDY_bm/*_BV(1)*/)) // is my oscillator 'ready' ?
      {
        goto skip_clock; // exit - don't change anything
      }

      // switch to 32Mhz clock using internal source

      CCP = CCP_IOREG_gc;// 0xd8 - see D manual, sect 3.14.1 (protected I/O)
      CLK_CTRL = CLK_SCLKSEL_RC32M_gc/*1*/; // set the clock to 32Mhz (6.9.1)
    }

    if(CLK_PSCTRL != 0)
    {
      CCP = CCP_IOREG_gc;// 0xd8 - see D manual, sect 3.14.1 (protected I/O)
      CLK_PSCTRL = CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc/*0*/; // set the clock divider(s) to 1:1 (6.9.2)
    }


    // now that I've changed the clock, disable 2Mhz, PLL, and external clocks
    // 32.768KHz should remain active, but I need to make sure it's stable
//    OSC_CTRL &= ~(_BV(4) | _BV(3) | _BV(0)); // sect 6.10.1 - disable PLL, external, 2Mhz clocks
    OSC_CTRL &= ~(OSC_PLLEN_bm | OSC_XOSCEN_bm // | OSC_RC2MEN_bm (don't disable 2M oscillator in bootloader)
#ifdef OSC_RC8MCAL // only present in 'E' series
                  | OSC_RC8MEN_bm
#endif // OSC_RC8MCAL
                  );

    // wait until 32.768KHz clock is 'stable'.  this one goes for a while
    // in case it doesn't stabilize in a reasonable time.  I figure about
    // 64*255 clock cycles should be enough, ya think?
    for(w1=65535; w1 > 0; w1--)
    {
      for(ch2=255; ch2 > 0; ch2--) // this waits up to 256 times longer than just the outer loop
      {
        if(OSC_STATUS & OSC_RC32KRDY_bm/*_BV(2)*/) // 32.768KHz oscillator is 'ready' (6.10.2)
        {
          goto done_waiting_for_osc_status;
        }
      }
    }

done_waiting_for_osc_status:

    // enable DFLL auto-calibration of the 32Mhz internal oscillator
    // (it uses the reasonably precise 32.768KHz clock to do it)
    // This is the BEST clock strategy to use for 32Mhz.

    OSC_DFLLCTRL = OSC_RC32MCREF_RC32K_gc/*0*/; // sect 6.10.7 - select 32.768KHz osc for everything, basically
    DFLLRC32M_CTRL = DFLL_ENABLE_bm/*1*/; // set the bit to enable DFLL calibration - section 6.11.1
  }

  // I'll be using the 1.024khz clock (from the 32.768KHz clock) for the real-time counter
  // this will give me a reasonable "about 1 millisecond" accuracy on the RTC

  // NOTE:  I may not have checked for this if I skipped the previous section,
  //        so now I check again, just in case, to make sure the 32.768KHz osc is stable
  for(w1=65535; w1 > 0; w1--)
  {
    for(ch2=255; ch2 > 0; ch2--)
    {
      if(OSC_STATUS & OSC_RC32KRDY_bm/*_BV(2)*/) // 32.768KHz oscillator is 'ready' (6.10.2)
      {
        goto done_waiting_again_osc_status;
      }
    }
  }

done_waiting_again_osc_status:

  if(!(OSC_STATUS & OSC_RC32KRDY_bm/*_BV(2)*/)) // is my oscillator 'ready' ?
  {
    goto skip_clock; // exit - don't change anything else
  }


  // RUN-TIME clock - use internal 1.024 khz source.  32khz needed for this
  // (the run-time clock works pretty well this way, scheduling events, CPU wakeup, etc.)
  CLK_RTCCTRL = 2; // section 6.9.4

skip_clock:  // go here if clock cannot be assigned for some reason or is already assigned

  // regular 'WATCHDOG_MODS' code starts back up here (NOTE: 'WATCHDOG_MODS' are always active now)

  // Check if the WDT was used to reset, in which case we don't bootload and skip straight to the code. w00t.
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

#ifdef SERIAL_PORT_REMAP

  // TODO:  allow this to be used for the A1 series with SERIAL1 ?

#ifdef SERIAL_PORT_REMAP_REG
  SERIAL_PORT_REMAP_REG = PORT_USART0_bm/*_BV(4)*/; // see 12.13.13 in 'E' manual
#endif // SERIAL_PORT_REMAP_REG

  // PD6 (GPIO 6) must have input pullup resistor (for serial I/O)
  SERIAL_PORT_PIN6CTRL = PINCTRL_INPUT_PULLUP;
  SERIAL_PORT_OUT |= _BV(6);
  SERIAL_PORT_DIR &= ~_BV(6);

  // PD7 (GPIO 7) must be configured as an output
  SERIAL_PORT_PIN7CTRL = PINCTRL_DEFAULT;
  SERIAL_PORT_OUT |= _BV(7);
  SERIAL_PORT_DIR |= _BV(7);

#else  // REMAP

#ifdef SERIAL_PORT_REMAP_REG
  SERIAL_PORT_REMAP_REG = 0; // see 12.13.13 in 'E' manual
#endif // SERIAL_PORT_REMAP_REG

  // PD2 (GPIO 2) must have input pullup resistor (for serial I/O)
  SERIAL_PORT_PIN2CTRL = PINCTRL_INPUT_PULLUP;
  SERIAL_PORT_OUT |= _BV(2);
  SERIAL_PORT_DIR &= ~_BV(2);

  // PD7 (GPIO 7) must be configured as an output
  SERIAL_PORT_PIN3CTRL = PINCTRL_DEFAULT;
  SERIAL_PORT_OUT |= _BV(3);
  SERIAL_PORT_DIR |= _BV(3);

#endif // REMAP


// set the CORRECT baud rate NOW.

#if BAUD_RATE == 115200

// section 19.4.4 - 'double speed' flag
// since this is set below, maybe it's not needed here?
#ifdef DOUBLE_SPEED
#define BAUD_SETTING  ((-2 << 12) | 135)
//  // section 19.4.4
//  SERIAL_USART_CTRLB = USART_CLK2X_bm/*_BV(2)*/; // enable clock 2x (everything else disabled)
#else // DOUBLE_SPEED
#define BAUD_SETTING ((-3 << 12) | 131)
//  SERIAL_USART_CTRLB = 0; // DISable clock 2x
#endif // DOUBLE_SPEED


#else // BAUD_RATE != 115200

// section 19.4.4 - 'double speed' flag
// since this is set below, maybe it's not needed here?
#ifdef DOUBLE_SPEED
//  // section 19.4.4
//  SERIAL_USART_CTRLB = USART_CLK2X_bm/*_BV(2)*/; // enable clock 2x (everything else disabled)
#else   // DOUBLE_SPEED
//  SERIAL_USART_CTRLB = 0; // DISable clock 2x
#endif  // DOUBLE_SPEED

// for now only 115k supported, so it's an error if I try to compile this for different baud rates
#error baud rate BAUD_RATE is NOT supported (for now)

#endif // BAUD_RATE == 115200


  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)

  SERIAL_USART_BAUDCTRLA = ((uint8_t)BAUD_SETTING) & 0xff;
  SERIAL_USART_BAUDCTRLB = (uint8_t)(BAUD_SETTING >> 8);

  // CTRLA settings - no interrupts
  SERIAL_USART_CTRLA = 0; // NO! SERIAL! PORT! INTERRUPTS!

  // section 19.14.5 - USART mode, parity, bits
  // CMODE 7:6 = 00 [async]  PMODE 5:4 = 00 [none]  SBMODE 3 = 0 [1 bit]   CHSIZE 2:0 = 3 (8-bit)
  SERIAL_USART_CTRLC = USART_CHSIZE_8BIT_gc/*0x03*/; // SERIAL_8N1 - see HardwareSerial.h

#ifdef USARTD0_CTRLD // which means we have a CTRLD - E5's do
  SERIAL_USART_CTRLD = 0; // always set to zero (E5 special)
#endif // USARTD0_CTRLD

  // last of all, assign CTRLB which switches on the right pins

#ifdef USE_STK500V2
  __builtin_avr_delay_cycles(F_CPU / 200); // delay approximately 5 msec before enabling serial port for STK500V2
                                           // this is to allow things to settle a bit beforehand
#endif // USE_STK500V2

  // section 19.4.4
#ifdef DOUBLE_SPEED
  SERIAL_USART_CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm; // enable double-speed, RX, TX, and disable other stuff
#else   // DOUBLE_SPEED
  SERIAL_USART_CTRLB = USART_RXEN_bm | USART_TXEN_bm;  // enable RX, TX, disable other stuff
#endif  // DOUBLE_SPEED


  // before globally enabling interrupts, make sure that 'certain hardware' isn't possibly messing things up

#ifdef USB_INTCTRLA
  USB_INTCTRLA = 0;
  USB_INTCTRLB = 0;
#endif // USB_INTCTRLA


  sei();  // ok to globally enable interrupts now


  // assign pin mode to LED output pin
  LED_CTRL = PINCTRL_DEFAULT;
  LED_DDR |= LED_PIN_BIT;

  /* flash onboard LED to signal entering of bootloader */
#if NUM_LED_FLASHES == 0
  ch2 = 6; //3;
#else
  ch2 = NUM_LED_FLASHES << 1;
#endif // NUM_LED_FLASHES

  while (ch2--) // inlined, may be smaller this way
  {
    // this requires ch2 to be twice the loop count
    if(!(ch2 & 1))
    {
      LED_PORT |= LED_PIN_BIT;
    }
    else
    {
      LED_PORT &= ~LED_PIN_BIT;
    }

    if(smart_delay_ms(100))
    {
      break;
    }
  }

  // NOTE:  call to 'getch' turns off the LED

  firstchar = 0; // to make sure we don't start the app by accident
                 // detect whether the first character is being received or not

  // and before anything else, zero out the error counter
  error_count = 0;


  /* forever loop */
  for (;;)
  {
#ifndef USE_STK500V2
    /* get character from UART */
    ch = getch();


    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                                 _         _                              //
    //                  __ _  _ __  __| | _   _ (_) _ __    ___                 //
    //                 / _` || '__|/ _` || | | || || '_ \  / _ \                //
    //                | (_| || |  | (_| || |_| || || | | || (_) |               //
    //                 \__,_||_|   \__,_| \__,_||_||_| |_| \___/                //
    //                                                                          //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////

    // 'arduino' flash protocol, aka stk500

    /* NOTE:  A bunch of if...else if... gives smaller code than switch...case ! */

    /* Hello is anyone home ? */
    if(ch=='0')
    {
      // KNOCK, KNOCK, Neo!
      firstchar = 1; // we got an appropriate bootloader instruction
      nothing_response();
    }
    else if (firstchar == 0) // 1st char is NOT a '0'
    {
      // the first character we got is not '0', lets bail!

      app_start(); // start the app (NOTE:  this is WAY smaller than using a goto)
      soft_boot(); // it's what I do when/if the application returns - soft boot
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
          soft_boot(); // it's what I do when the application returns - soft boot
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
      soft_boot(); // it's what I do when the application returns - soft boot
    }


    /* Set address, little endian. EEPROM in bytes, FLASH in words  */
    /* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
    /* This might explain why little endian was used here, big endian used everywhere else.  */
    else if(ch=='U')
    {
#if PAGE_SIZE <= 256
      address.byte[0] = getch();
      address.byte[1] = getch();
#ifdef RAMPZ
      address.byte[2] = 0;
      address.byte[3] = 0;
#endif // RAMPZ
#else // PAGE_SIZE > 256
// this is a hack for 512 byte pages, which appear to use word addressing...
      address.byte[0] = getch();
      address.byte[1] = getch();
#ifdef RAMPZ
      address.byte[2] = 0;
      address.byte[3] = 0;

      address.dword <<= 1;
#else // RAMPZ
      address.word <<= 1;
#endif // RAMPZ
#endif // PAGE_SIZE > 256

      nothing_response();
    }

    /* Universal SPI programming command, disabled.  Would be used for fuses and lock bits.  */
    else if(ch=='V')
    {
      if (getch() == 0x30)
      {
        // this command gets the chip signature bytes

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
      unsigned short w2 = address.word % PAGE_SIZE;

      length.byte[1] = getch();
      length.byte[0] = getch();
      flags.eeprom = 0;

      if(length.word > PAGE_SIZE)
      {
        length.word = PAGE_SIZE; // no buffer overruns (will cause error later, probably)
      }

      if (getch() == 'E')
      {
        flags.eeprom = 1;
      }

      if(w2 && !flags.eeprom) // address NOT on a page boundary, NOT EEPROM
      {
        for(w1=0; w1 < w2; w1++)
        {
#ifdef RAMPZ
          buff[w1] = pgm_read_byte_far(address.dword - w2 + w1);
#else // RAMPZ
          buff[w1] = pgm_read_byte((unsigned char *)address.word - w2 + w1);
#endif // RAMPZ
        }
      }
      else
      {
        w2 = 0; // for EEPROM
      }

      for (w1=0; w1<length.word; w1++)
      {
        buff[w1 + w2] = getch();                          // Store data in buffer, can't keep up with serial data stream whilst programming pages
      }

      if (getch() == ' ')
      {
        if (flags.eeprom)
        {                       // Write to EEPROM one byte at a time
//          address.word <<= 1; // TODO:  is this right for EEPROM ?
          for(w1=0;w1<length.word;w1++)
          {
            eeprom_write_byte((void *)address.word,buff[w1]);
            address.word++;
          }
        }
        else
        {                   // Write to FLASH one page at a time
          if (length.byte[0] & 0x01)
            length.word++;  // Even up an odd number of bytes

          // TODO:  compensate for page size too short or too long
          //        for now, compensate for 'too short' only

          // XMEGA-ONLY CODE HERE

          SP_WaitForSPM();
          SP_EraseFlashBuffer();
          SP_WaitForSPM();
          SP_LoadFlashPage(&(buff[0]), length.word + w2);
          SP_WaitForSPM();
#ifdef RAMPZ
          SP_WriteApplicationPage((uint32_t)address.dword - w2);
#else // RAMPZ
          SP_WriteApplicationPage((uint32_t)address.word - w2);
#endif // RAMPZ
          SP_WaitForSPM();

#ifdef RAMPZ
          address.dword += length.word;
#else // RAMPZ
          address.word += length.word; // compatibility
#endif // RAMPZ
        }

        putch(0x14);
        putch(0x10);
      }
      else
      {
        if (++error_count == MAX_ERROR_COUNT)
        {
          app_start();
          soft_boot(); // it's what I do when the application returns - soft boot
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

        // NOTE:  the length can be up to a page size, but the address should be
        //        on a page boundary.  the programmer should respect the page size

        for (w1=0;w1 < length.word;w1++)
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
          soft_boot(); // it's what I do when the application returns - soft boot
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
      soft_boot(); // it's what I do when the application returns - soft boot
    }

#else // USE_STK500V2


    //////////////////////////////////////////////////////////////////////////////
    //                                                                          //
    //                  _    _     ____    ___    ___          ____             //
    //             ___ | |_ | | __| ___|  / _ \  / _ \ __   __|___ \            //
    //            / __|| __|| |/ /|___ \ | | | || | | |\ \ / /  __) |           //
    //            \__ \| |_ |   <  ___) || |_| || |_| | \ V /  / __/            //
    //            |___/ \__||_|\_\|____/  \___/  \___/   \_/  |_____|           //
    //                                                                          //
    //                                                                          //
    //////////////////////////////////////////////////////////////////////////////

/*
 * States used in the receive state machine (from stk500boot.c)
 */
#define	ST_START		0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA		5
#define	ST_GET_CHECK	6
#define	ST_PROCESS		7


// definitions from 'command.h' (an ATMel file describing stk500v2 protocol commands)
#define MESSAGE_START                       0x1B        //= ESC = 27 decimal
#define TOKEN                               0x0E
#define CMD_SIGN_ON                         0x01
#define CMD_SET_PARAMETER                   0x02
#define CMD_GET_PARAMETER                   0x03
#define CMD_LOAD_ADDRESS                    0x06
#define CMD_ENTER_PROGMODE_ISP              0x10
#define CMD_LEAVE_PROGMODE_ISP              0x11
#define CMD_CHIP_ERASE_ISP                  0x12
#define CMD_PROGRAM_FLASH_ISP               0x13
#define CMD_READ_FLASH_ISP                  0x14
#define CMD_PROGRAM_EEPROM_ISP              0x15
#define CMD_READ_EEPROM_ISP                 0x16
#define CMD_READ_FUSE_ISP                   0x18
#define CMD_PROGRAM_LOCK_ISP                0x19
#define CMD_READ_LOCK_ISP                   0x1A
#define CMD_READ_SIGNATURE_ISP              0x1B
#define CMD_SPI_MULTI                       0x1D
#define STATUS_CMD_OK                       0x00
#define STATUS_CMD_FAILED                   0xC0
#define STATUS_CKSUM_ERROR                  0xC1
#define PARAM_BUILD_NUMBER_LOW              0x80
#define PARAM_BUILD_NUMBER_HIGH             0x81
#define PARAM_HW_VER                        0x90
#define PARAM_SW_MAJOR                      0x91
#define PARAM_SW_MINOR                      0x92

// from stk500boot.c, says these must match the version of AVRStudio
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A



    checksum = 0;
    ch = do_getch_and_checksum(); // all messages start here

    // this code is borrowed (and adapted) from 'stk500boot.c', under GPL license, part of Arduino IDE
    // use protocol 'wiring' in boards.txt (same as mega2560)

    // main loop  (executed when bootstate == 1 in original code)
    // NOTE:  ch contains the last read-in character, and 'getch()' handles startup delay and blinking  

#ifdef ENABLE_BANG
    if(ch == '!' && !firstchar) // TEMPORARY, for testing
    {
      firstchar = 1;

      msgBuffer[0] = '*';
      msgBuffer[1] = '@';
      msgBuffer[2] = '$';
      msgBuffer[3] = 'A';
      msgBuffer[4] = 'V';
      msgBuffer[5] = 'R';
      msgBuffer[6] = 'I';
      msgBuffer[7] = 'S';
      msgBuffer[8] = 'P';
      msgBuffer[9] = '_';
      msgBuffer[10] = '2';
      msgBuffer[11] = ' ';
      msgBuffer[12] = 'X';
      msgBuffer[13] = 'M';
      msgBuffer[14] = 'e';
      msgBuffer[15] = 'g';
      msgBuffer[16] = 'a';
      msgBuffer[17] = '\r';
      msgBuffer[18] = '\n';

      msgLength = 19;

      LED_PORT &= ~LED_PIN_BIT;          // turn OFF the led

      error_count = MAX_ERROR_COUNT; // always bail out

      goto send_the_reply;
    }    
#endif // ENABLE_BANG

    {

      /*
       * Collect received bytes to a complete message
       */

      if(ch != MESSAGE_START)
      {
        // TODO:  flush the input?
        msgLength = 2;
        msgBuffer[1] = STATUS_CKSUM_ERROR;

error_flush_send_reply:
        flushin();

        error_count++;

        if(firstchar)
        {
          goto send_the_reply;
        }

        firstchar = 1;

        // THIS code *REALLY* only applies the *VERY* first time
        // wait up to 6 seconds for more serial input.  this is to accomodate AVRDUDE 'wiring' protocol behavior

        smart_delay_ms(6000); // smart_delay_ms waits for 'n' msecs OR serial input (whichever)

        continue; // continues with the 'forever loop'
      }

      firstchar = 1; // mark first character was received

      ch = do_getch_and_checksum();  // seq number
      seqNum = ch;

      ch = do_getch_and_checksum();  // high byte of length
      msgLength = ((uint16_t)ch) << 8;

      ch = do_getch_and_checksum();  // low byte of length
      msgLength |= ch;

      ch = do_getch_and_checksum();  // token

      if(ch != TOKEN || msgLength >= sizeof(msgBuffer)) // this is my buffer check
      {
        msgLength = 2;
        msgBuffer[1] = STATUS_CKSUM_ERROR;

        goto error_flush_send_reply;
      }

      // 'msgLength' bytes up to PAGE_SIZE
      p1 = &(msgBuffer[0]);
      for(w1 = 0; w1 < msgLength; w1++)
      {
        *(p1++) = do_getch_and_checksum();  // individual data bytes
      }

      ch = getch(); // checksum byte

      if(ch != checksum) // mismatch?
      {
        msgLength = 2;
        msgBuffer[1] = STATUS_CKSUM_ERROR;

        goto error_flush_send_reply;
      }

      /*
       * Now process the STK500 commands, see Atmel Appnote AVR068
       */

      ch = msgBuffer[0]; // cache it, probably faster

#ifndef REMOVE_CMD_SPI_MULTI
      if(ch == CMD_SPI_MULTI) // 0x1d
      {
        // the implementation in stk500boot.c is WRONG FREAKING WRONG!  OK it was a pretty
        // good HACK, and 'works', but it's still WRONG.
        //
        // So I read the documentation (STK500v2 Manual aka 'doc2591.pdf'), and also the ATmega328 manual section 28.8.3
        //
        // These are the correct message pseudo-structures for command and reply:
        //
        // struct cmd_spi_multi_thing
        // {
        //   uint8_t command; // 1dH
        //   uint8_t numTx;   // # bytes to transmit
        //   uint8_t numRx:   // # bytes to receive startign at offset 'rxStartAddr'
        //   uint8_t rxStartAddr; // start offset within returned data to place into rxData[]
        //   uint8_t txData[0];   // actual data sent to the SPI bus (numTx bytes)
        // };
        // struct cmd_spi_multi_thing_reply
        // {
        //   uint8_t command;   // 1dH again
        //   uint8_t Status1;   // STATUS_CMD_OK (0)
        //   uint8_t rxData[n]; // 'n' bytes of data padded with 0 bytes if fewer received
        //   uint8_t Status2;   // STATUS_CMD_OK (0)
        // };
        //
        // This pseudo structure is the command sent in 'msgBuffer'.  the returned data is
        // sent BACK after doing the SPI command, using the bytes transmitted by the SPI command.
        //
        // The SPI commands are for the ATMega processors, and NOT the xmega.  They can still be translated
        // into the equivalent functions. I'll do a small number, enough to make things work, and verify the
        // existing 'hack' to make sure it's 'correct enough'.
        //
        // Just to complicate things, it uses WORD addresses for SPI instructions.  'low byte' and
        // 'high byte' access handle addresses ending in 0 or 1, respectively.

        {
          register unsigned char answerByte=0;
          register unsigned char flag=0;

          if ( msgBuffer[4]== 0x30 ) // read signature byte (this turns out to be ok, actually)
          {
            register unsigned char signatureIndex = msgBuffer[6];

            if ( signatureIndex == 0 )
            {
              answerByte = (SIGNATURE_BYTES >> 16) & 0x000000FF;
            }
            else if ( signatureIndex == 1 )
            {
              answerByte = (SIGNATURE_BYTES >> 8) & 0x000000FF;
            }
            else
            {
              answerByte = SIGNATURE_BYTES & 0x000000FF;
            }
          }
          else if ( msgBuffer[4] == 0x20 || msgBuffer[4] == 0x28 ) // read memory location
          {
            register uint32_t addr = (uint32_t)address * PAGE_SIZE;

            // 'address' contains the page address?  msgBuffer[7] is the rxStartAddr?
            // and it expects msgBuffer[6] bytes in response

            flag = 1; // meaning I don't populate this
            w1 = msgBuffer[2]; // number of bytes to receive

            addr += ((uint16_t)msgBuffer[5] << 9)  // MSB from SPI command, WORD address
                  + ((uint16_t)msgBuffer[6] << 1); // LSB from SPI command, WORD address

            if(msgBuffer[4] == 0x28)
            {
              addr++; // high byte
            }

            // TODO:  do I add msgBuffer[3] (the original offset) to this?

            msgLength = 3 + w1;
            msgBuffer[1] = STATUS_CMD_OK;
            msgBuffer[2 + w1] = STATUS_CMD_OK;

#ifdef RAMPZ
            msgBuffer[1 + w1] = pgm_read_byte_far(addr);  // last byte in sequence
#else // RAMPZ
            msgBuffer[1 + w1] = pgm_read_byte(addr);
#endif // RAMPZ

            while(w1 > 0)
            {
              w1--;
              msgBuffer[1 + w1] = 0; // zero everything else
            }
          }
          else if ( msgBuffer[4] & 0x50 ) // that would be 4xH or 5xH or 1xH - 50 read fuse bits, 58 read lock bits 
          {
          //*  Issue 544:   stk500v2 bootloader doesn't support reading fuses
          //*  I cant find the docs that say what these are supposed to be but this was figured out by trial and error
          //  answerByte = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
          //  answerByte = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
          //  answerByte = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);

            // NOTE:  these are really mega2560 features, NOT XMEGA
            // TODO:  come up with an XMEGA equivalent?

            // the docs say it's sending SPI commands.  So a typical sequence (that fails) might be:
            // 1d 04 04 00 4d 00 00 00 12  [this replies 1d 00 00 4d 00 00 00 1d 1b]
            // 1d 04 04 00 20 00 55 00 15  [this fails]

            if (msgBuffer[4] == 0x50)
            {
              answerByte = readNVMData(NVM_CMD_READ_FUSES_gc, 0);
//                answerByte = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
            }
            else if (msgBuffer[4] == 0x58)
            {
              answerByte = readNVMData(NVM_CMD_READ_FUSES_gc, 1); // low/high fuse byte is wrong; xmega has up to 6 of them
//                answerByte = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
            }
            else if (msgBuffer[4] == 0x40) // load memory page low byte
            {
              address &= ~(address_t)0xff00;
              address |= msgBuffer[7];
              answerByte = 0;
            }
            else if (msgBuffer[4] == 0x48) // load memory page high byte
            {
              address &= ~(address_t)0xff00;
              address |= ((address_t)(msgBuffer[7])<<8);
              answerByte = 0;
            }
#if defined(RAMPZ)
            else if (msgBuffer[4] == 0x4d) // load memory page extended address
            {
              address &= ~(address_t)0xff0000;
              address |= ((address_t)(msgBuffer[7])<<16);
              answerByte = 0;
            }
#endif
            else
            {
              answerByte = 0;
            }
          }
          else
          {
            // NOTE:  this is a hack and should be properly implemented.

            answerByte = 0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
          }

          if ( !flag )
          {
            // these are simple 1-byte returns
            msgLength = 7;
            msgBuffer[1] = STATUS_CMD_OK;
            msgBuffer[2] = 0;
            msgBuffer[3] = msgBuffer[4]; // not sure why, but I think it should be zero - I'll leave it alone just in case
            msgBuffer[4] = 0;
            msgBuffer[5] = answerByte;
            msgBuffer[6] = STATUS_CMD_OK;
          }
        }

        goto send_the_reply;
      }

#endif
      if(ch == CMD_SIGN_ON)
      {
        msgLength = 17; // was 11, I added ' XMega' to it just for grins

        msgBuffer[1] = STATUS_CMD_OK;
        msgBuffer[2] = 8;
        msgBuffer[3] = 'A';
        msgBuffer[4] = 'V';
        msgBuffer[5] = 'R';
        msgBuffer[6] = 'I';
        msgBuffer[7] = 'S';
        msgBuffer[8] = 'P';
        msgBuffer[9] = '_';
        msgBuffer[10] = '2';
        msgBuffer[11] = ' ';
        msgBuffer[12] = 'X';
        msgBuffer[13] = 'M';
        msgBuffer[14] = 'e';
        msgBuffer[15] = 'g';
        msgBuffer[16] = 'a';
      }
      else if(ch == CMD_GET_PARAMETER)
      {
        register unsigned char value;

        if(msgBuffer[1] == PARAM_BUILD_NUMBER_LOW)
        {
          value = CONFIG_PARAM_BUILD_NUMBER_LOW;
        }
        else if(msgBuffer[1] == PARAM_BUILD_NUMBER_HIGH)
        {
          value = CONFIG_PARAM_BUILD_NUMBER_HIGH;
        }
        else if(msgBuffer[1] == PARAM_HW_VER)
        {
          value = CONFIG_PARAM_HW_VER;
        }
        else if(msgBuffer[1] == PARAM_SW_MAJOR)
        {
          value = CONFIG_PARAM_SW_MAJOR;
        }
        else if(msgBuffer[1] == PARAM_SW_MINOR)
        {
          value = CONFIG_PARAM_SW_MINOR;
        }
        else
        {
          value = 0;
        }

        msgLength = 3;
        msgBuffer[1] = STATUS_CMD_OK;
        msgBuffer[2] = value;
      }
      else if(ch == CMD_LEAVE_PROGMODE_ISP) // I call this at various times, not just at the end
      {
        msgLength = 2;
        msgBuffer[1] = STATUS_CMD_OK;
      }
      else if(ch ==CMD_SET_PARAMETER ||
              ch == CMD_ENTER_PROGMODE_ISP)
      {
        msgLength = 2;
        msgBuffer[1] = STATUS_CMD_OK;
      }
      else if(ch == CMD_READ_SIGNATURE_ISP)
      {
        register unsigned char signatureIndex = msgBuffer[4];
        register unsigned char signature;

        if ( signatureIndex == 0 )
        {
          signature = (SIGNATURE_BYTES >>16) & 0x000000FF;
        }
        else if ( signatureIndex == 1 )
        {
          signature = (SIGNATURE_BYTES >> 8) & 0x000000FF;
        }
        else
        {
          signature = SIGNATURE_BYTES & 0x000000FF;
        }

        msgLength = 4;
        msgBuffer[1] = STATUS_CMD_OK;
        msgBuffer[2] = signature;
        msgBuffer[3] = STATUS_CMD_OK;
      }
      else if(ch ==  CMD_READ_LOCK_ISP)
      {
        msgLength = 4;
        msgBuffer[1] = STATUS_CMD_OK;
#ifdef LOCKBIT_LOCKBITS
        msgBuffer[2] = LOCKBIT_LOCKBITS; // boot_lock_fuse_bits_get( GET_LOCK_BITS );
#else // no LOCKBIT_LOCKBITS
        msgBuffer[2] = NVM_LOCKBITS; // this is how it's defined in the A1 files
#endif // LOCKBIT_LOCKBITS
        msgBuffer[3] = STATUS_CMD_OK;
      }
      else if(ch == CMD_READ_FUSE_ISP)
      {
        register unsigned char fuseBits;

        if ( msgBuffer[2] == 0x50 )
        {
          if ( msgBuffer[3] == 0x08 )
          {
            fuseBits = readNVMData(NVM_CMD_READ_FUSES_gc, 2); // extended fuse byte is wrong; xmega has up to 6 of them
          }
          else
          {
            fuseBits = readNVMData(NVM_CMD_READ_FUSES_gc, 0); // low fuse byte is wrong; xmega has up to 6 of them
          }
        }
        else
        {
          fuseBits = readNVMData(NVM_CMD_READ_FUSES_gc, 1); // high fuse byte is wrong; xmega has up to 6 of them
        }
        msgLength = 4;
        msgBuffer[1] = STATUS_CMD_OK;
        msgBuffer[2] = fuseBits;
        msgBuffer[3] = STATUS_CMD_OK;
      }
#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
      else if(ch == CMD_PROGRAM_LOCK_ISP)
      {
          // NOTE:  this is the old 'lock bits' code.  activating lock bits on an xmega is a one-way operation
          //        that requires using the PDI interface to change them back, once locked.  So don't do it here.
//            unsigned char lockBits = msgBuffer[4];
//
//            lockBits = (~lockBits) & 0x3C;  // mask BLBxx bits
//            boot_lock_bits_set(lockBits);    // and program it
//            boot_spm_busy_wait();

          // fake like it worked, though I didn't do anything.  this is probably NOT compatible but avrdude didn't gripe

          msgLength = 3;
          msgBuffer[1] = STATUS_CMD_OK;
          msgBuffer[2] = STATUS_CMD_OK;
      }
#endif
      else if(ch == CMD_CHIP_ERASE_ISP)
      {
        msgLength = 2;

        // performing a 'chip erase' is probably NOT a good idea on the xmega.
        // an alternate way would be to write blocks of 'FF' into the application area, and optionally
        // into the EEPROM area (based on fuse settings), excluding the bootloader [of course].
        // Since I don't want to write the code, I'll just indicate 'failed' as before.

        // msgBuffer[1] = STATUS_CMD_OK;
        msgBuffer[1] = STATUS_CMD_FAILED;  //*  isue 543, return FAILED instead of OK
      }
      else if(ch == CMD_LOAD_ADDRESS)
      {
#if defined(RAMPZ)
        address = ( ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;
#else
        address = ( ((msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;    //convert word to byte address
#endif
        msgLength = 2;
        msgBuffer[1] = STATUS_CMD_OK;
      }
      else if(ch == CMD_PROGRAM_FLASH_ISP ||
              ch == CMD_PROGRAM_EEPROM_ISP)
      {
        address_t tempaddress;

        w1 = ((msgBuffer[1])<<8) | msgBuffer[2]; // the size

        p1 = &(msgBuffer[10]);

        tempaddress = address;

#ifdef RAMPZ
        if(tempaddress >= 0x0800000)
        {
          tempaddress -= 0x0800000; // the absolute address of the NVRAM in 'programmer' space
                                    // if the address was based from THIS, must subtract it
                                    // this is documented in the D manual Figure 25-3 and elsewhere
        }
#endif // RAMPZ

        if ( ch == CMD_PROGRAM_FLASH_ISP )
        {
          if(w1 & 1)
          {
            w1++; // make sure it's even (but it should be)
          }

          if(tempaddress < BOOT_SECTION_START) // do NOT overwrite the bootloader!
          {
            SP_WaitForSPM();
            SP_EraseFlashBuffer();
            SP_WaitForSPM();
            SP_LoadFlashPage(p1, w1);
            SP_WaitForSPM();
            SP_WriteApplicationPage(tempaddress);
            SP_WaitForSPM();
          }

          address += w1; // increment address by the size (for next loop)
        }
        else
        {
          //*  issue 543, this should work, It has not been tested.

          // NOTE:  why are we writing the EEPROM as WORD values?  I need to fix this.  Again.

          uint16_t w2 = address >> 1;
          /* write EEPROM */
          while (w1) // count down the size
          {
            eeprom_write_byte((uint8_t*)w2, *p1++);
            address+=2;            // Select next EEPROM byte
            w2++;
            w1--;
          }
        }

        msgLength = 2;
        msgBuffer[1] = STATUS_CMD_OK;
      }
      else if(ch == CMD_READ_FLASH_ISP ||
              ch == CMD_READ_EEPROM_ISP)
      {
        address_t tempaddress;

        w1 = ((msgBuffer[1])<<8) | msgBuffer[2]; // the size
        p1 = &(msgBuffer[1]);

        msgLength = w1 + 3;

        tempaddress = address; // so I can do maths with it

#ifdef RAMPZ
        if(tempaddress >= 0x0800000)
        {
          tempaddress -= 0x0800000; // the absolute address of the NVRAM in 'programmer' space
                                    // if the address was based from THIS, must subtract it
                                    // this is documented in the D manual Figure 25-3 and elsewhere
        }
#endif // RAMPZ

        *(p1++) = STATUS_CMD_OK;

        if (ch == CMD_READ_FLASH_ISP )
        {
          register unsigned int data;

          address += w1; // pre-add size - will mess with 'tempaddress' later

          // Read FLASH
          do
          {
//#if defined(RAMPZ)
#ifdef RAMPZ // (FLASHEND > 0x10000)
            if(tempaddress >= BOOT_SECTION_START)
            {
              data = 0; // assume boot section is all 0's for the sake of flash verification
            }
            else
            {
              data = pgm_read_word_far(tempaddress);
            }
#else // FLASHEND > 0x10000
            if(address >= BOOT_SECTION_START)
            {
              data = 0; // assume boot section is all 0's for the sake of flash verification
            }
            else
            {
              data = pgm_read_word_near(tempaddress);
            }
#endif // FLASHEND > 0x10000
            *(p1++) = (unsigned char)data;    //LSB
            *(p1++) = (unsigned char)(data >> 8);  //MSB

            tempaddress += 2; // increment address by 2 bytes
            w1 -= 2;          // decrement size by 2
          } while (w1); // while size > 0
        }
        else
        {
          /* Read EEPROM */
          register uint16_t w2 = address >> 1;
          do
          {
            *(p1++) = eeprom_read_byte((uint8_t*)w2); // VERIFY THIS IS CORRECT (it is likely NOT to be)
            address+=2;            // Select next EEPROM byte (add 2 to the address?  really?)
            w2++;
            w1--;                // MAKE SURE THIS WORKS
          } while (w1); // while size > 0
        }

        *(p1++) = STATUS_CMD_OK;
      }
      else if(!ch) // this is my "error trap" for a strange problem
      {
        msgLength = 4;
        msgBuffer[1] = STATUS_CMD_FAILED;
        msgBuffer[2] = ch;
        msgBuffer[3] = STATUS_CMD_FAILED;

        error_count++;
      }
      else
      {
        msgLength = 2;
        msgBuffer[1] = STATUS_CMD_FAILED;

        error_count++;
      }

send_the_reply:
      /*
       * Now send answer message back
       */

      checksum = 0;

      do_putch_and_checksum(MESSAGE_START);
      do_putch_and_checksum(seqNum);

      ch = ((msgLength>>8)&0xFF);
      do_putch_and_checksum(ch);

      ch = msgLength&0x00FF;
      do_putch_and_checksum(ch);

      do_putch_and_checksum(TOKEN);

      flush();

      {
        p1 = &(msgBuffer[0]);

        while ( msgLength )
        {
          do_putch_and_checksum(*(p1++));
          msgLength--;
        }
      }

      putch(checksum);

      flush();

      seqNum++;
    }

    // if I get here, I guess I'm done doing the stk500v2 protocol stuff

    if(error_count >= MAX_ERROR_COUNT)
    {
#if 1 /* testing only */
      ch2 = 9;

      while (ch2--) // inlined, may be smaller this way
      {
        LED_PORT |= LED_PIN_BIT;
        smart_delay_ms(100);

        LED_PORT &= ~LED_PIN_BIT;
        smart_delay_ms(100);
      }
#endif // 0,1

      if(pgm_read_byte(0) != 0xff)
      {
        app_start();
      }

      soft_boot();
    }

    // if no error, just loop again using original 'main loop' code

#endif // USE_STK500V2

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

char smart_delay_ms(uint16_t ms)
{
  while(ms)
  {
#ifdef USE_STK500V2
    if((SERIAL_USART_STATUS) & USART_RXCIF_bm) // is there RX data ??
    {
      return 1; // exit now (this avoids delays in startup for STK500V2 protocol)
                // it's basically a workaround for intolerance by the protocol
    }
#endif // USE_STK500V2
    ms--;

    __builtin_avr_delay_cycles(F_CPU / 1000); // delay approximately 1 msec
  }

  return 0; // was not early-terminated
}

// 'getch()' and 'putch()' - basic serial I/O functions
char getch(void)
{
  uint32_t count = 0;
  register char rVal;

  LED_PORT &= ~LED_PIN_BIT;          // turn off the LED to indicate receiving data

  while(!((SERIAL_USART_STATUS) & USART_RXCIF_bm)) // wait for RX data
  {
    /* 20060803 DojoCorp:: Addon coming from the previous Bootloader*/
    /* HACKME:: here is a good place to count times*/
    count++;

    if (count > (uint32_t)(MAX_TIME_COUNT)) // delay period for firmware flashing
    {
      app_start();
      soft_boot();
    }
  }

  SERIAL_USART_STATUS = USART_RXCIF_bm; // forcibly clear the bit
  rVal = SERIAL_USART_DATA;

  LED_PORT |= LED_PIN_BIT;          // turn on the LED to indicate receiving data

  return rVal;
}

void putch(char ch)
{
  while (!( (SERIAL_USART_STATUS) & USART_DREIF_bm)) // bit 5 is the DRE bit (6 is the 'transmitted' bit)
  {  } // wait forever for DRE flag

  SERIAL_USART_STATUS = USART_DREIF_bm; // set the bit to clear it

  (SERIAL_USART_DATA) = ch; // and assign the data
}


#ifdef USE_STK500V2
uint8_t do_getch_and_checksum(void)
{
  register uint8_t chRval = getch();
  checksum ^= chRval;

  return chRval;
}

void do_putch_and_checksum(uint8_t chPut)
{
  putch(chPut);
  checksum ^= chPut;
}
#endif // USE_STK500V2


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


void putstr(unsigned char *pBuf, char nBytes)
{
  while(nBytes--)
  {
    putch(*(pBuf++));
  }
}


void flush(void)
{
  uint32_t count = 0;

  while (!( (SERIAL_USART_STATUS) & USART_DREIF_bm)) // bit 5 is the DRE bit (6 is the 'transmitted' bit)
  {
    count++;
    if(count > (uint32_t)(F_CPU / 256)) // this works out to about 1/256'th of a second times the loop cycles
    {
      return; // safety
    }
  } // wait forever for DRE flag

  while (!( (SERIAL_USART_STATUS) & USART_TXCIF_bm)) // and wait for 'transmitted' bit also
  {
    count++;
    if(count > (uint32_t)(F_CPU / 256)) // this count is cumulative from the last loop
    {
      return; // safety
    }
  }

  SERIAL_USART_STATUS = USART_TXCIF_bm | USART_DREIF_bm; // set the bits to clear them (oxymoronic, but true)
}


#ifdef USE_STK500V2
void flushin(void)
{
  uint32_t count = 0;
  volatile uint8_t x1;

  LED_PORT &= ~LED_PIN_BIT;          // turn off the LED to indicate receiving data

  while(1)
  {
    while(!((SERIAL_USART_STATUS) & USART_RXCIF_bm)) // wait for RX data
    {
      count++;

      if (count > (uint32_t)(F_CPU / 128)) // delay period for flushing - about 0.05 sec (based on 8 cycles or so per loop)
      {                                    // that of course would be 0.05 seconds of *SILENCE*, not total accumulation
        return; // nothing left after timeout
      }
    }

    SERIAL_USART_STATUS = USART_RXCIF_bm; // clear the bit
    x1 = SERIAL_USART_DATA; // burn the data

    count = 0; // restart counter
  }
  
}
#endif // USE_STK500V2



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

uint8_t readNVMData(uint8_t cmd, uint16_t iIndex)
{
  uint8_t rVal;

  /* Load the NVM Command register to read the correct data. */
  NVM_CMD = cmd; // example, NVM_CMD_READ_CALIB_ROW_gc;

  // others of interest:  NVM_CMD_WRITE_LOCK_BITS_gc, NVM_CMD_READ_FUSES_gc
  // writing lock bits requires the CCP protection sequence
  // reading lock bits can be done with the LOCKBITS register - 'D' manual 4.13.5

  // these commands are triggered by the lpm or spm instructions
  // see 'D' manual section 25.11.3, table 25-3

////  rVal = pgm_read_byte_near(iIndex); // this should work correctly
  __asm__ ("lpm %0, Z\n" : "=r" (rVal) : "z" (iIndex)); // this works better

  /* Clean up NVM Command register. */
  NVM_CMD = NVM_CMD_NO_OPERATION_gc;

  return(rVal);
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

// for reference, a command to test the flash process
// /usr/local/arduino/hardware/tools/avr/bin/avrdude -C/usr/local/arduino/hardware/tools/avr/etc/avrdude.conf -v -v -v -v -patxmega64d4 -cwiring -P/dev/cuaU0 -b115200 -D -Uflash:w:/var/tmp//build1587155088328648934.tmp/Blink.cpp.hex:i 


