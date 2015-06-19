/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  Updated for 'xmega' core by bob frazier, S.F.T. Inc. - http://mrp3.com/

  X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A

  The xmega code mods make a considerable number of assumptions
  about the pin number assignments (as defined here):

  DEFAULT MAPPING ('DIGITAL_IO_PIN_SHIFT' NOT DEFINED)
  ----------------------------------------------------
  PORTD - digital 0-7
  PORTC - digital 8-15
  PORTE - digital 16-23
  PORTF - digital 24-31
  PORTH - digital 32-39
  PORTJ - digital 40-47
  PORTK - digital 48-55
  PORTQ - digital 56-59
  PORTR - digital 60-61 (built-in LED on PORTR pin 1, aka '61')
  PORTA - analog 0-7, digital 62-69
  PORTB - analog 8-15, digital 70-77

  SPI is assumed to be on PORTC (pins 4-7)
  Serial is implemented on PORTD, Serial2 on PORTC, both using pins 2,3 (no flow control)
  PORTR pin 1 is assumed to be connected to an LED.  Pin 1 is the 'built-in' LED, defined
  as 'LED_BUILTIN', and is active HIGH.

  Your Mileage May Vary, depending on your board layout.  Some boards shift the
  digital pin assignments by 2 so that digital I/O pin 0 is PORTD Rx, pin 13 is PORTC SCK, just
  like the Arduino ATmega board.  Then they align the physical pins so that a regular Arduino shield
  will work, and assign PORTD 0-1 to 2 different digital I/O pin numbers (such as 20 and 21).

  To facilitate that specific change, uncomment the #define for 'DIGITAL_IO_PIN_SHIFT', below.
  Alternately you can create a separate version of this file with a different variant name,
  such as 'xmega-compat', with the #define uncommented, stored in an appropriately named directory.

  ============================
  HARDWARE SERIAL FLOW CONTROL
  ============================

  This version of the xmega Arduino startup+support files supports HARDWARE FLOW CONTROL on ALL ROUR serial ports via
  RTS (output) and CTS (input).  CTS is implemented as input from remote device's DTR.  RTS is implemented as DTR output.

  To implement RTS/CTS, use definitions similar to the following in your version of this header file

  NOTE: RTS(DTR) will be configured as an output, active low (high tells sender to stop sending data to the UART)
        CTS will be configured as an input, active low (high stops data from being sent out via UART)

  CTS high to low transition causes an interrupt that may result in serial I/O (for faster response time).

  // RTS(DTR) as GPIO 0 (port D pin 0)
  #define SERIAL_0_RTS_PORT_NAME PORTD
  #define SERIAL_0_RTS_PIN_INDEX 0

  // CTS as GPIO 1 (port D pin 1)
  #define SERIAL_0_CTS_PORT_NAME PORTD
  #define SERIAL_0_CTS_PIN_INDEX 1

  use similar definitions for serial 1, aka 'SERIAL_1_CTS_PORT'

  NOTE:  you can even use PORTA or PORTB pins for this, if you don't need to measure analog volts on those pins

  X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A

*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// for now, the default xmega uses a simple assignment of digital pin numbers, beginning with port D
// to accomodate a useful "compatibility" shield design, these pins can be shifted so that the pin
// that maps to 'digitalRead(0)' would be D2 rather than D0.  This also puts 'Serial' on pins 0 and 1
// exactly like the Arduino UNO.  For any other mapping, create your own 'pins_arduino.h' file.
//
#define DIGITAL_IO_PIN_SHIFT /* UNCOMMENT THIS to shift digital pin assignments for Arduino shield compatibility */

//#define USE_AREF 0x2 /* see 28.16.3 in 'AU' manual - this is the REFCTRL bits for the reference select, AREF on PORTA (PA0) */

#define NUM_DIGITAL_PINS            62

#ifdef USE_AREF
#define NUM_ANALOG_INPUTS           15
#define analogInputToAnalogPin(p) ((p)-A0 + 1)
#else // USE_AREF
#define NUM_ANALOG_INPUTS           16
#define analogInputToAnalogPin(p) ((p)-A0)
#endif // USE_AREF

#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 62 : -1)

#ifdef DIGITAL_IO_PIN_SHIFT
#define digitalPinHasPWM(p)         ((p) < 30 || (p) == 60 || (p) == 61) /* PORTD pins 0 and 1 are 20 and 21, respectively */
#else // no digital I/O pin shift
#define digitalPinHasPWM(p)         ((p) < 32) /* port F pin 7 is the highest one that has PWM */
#endif // DIGITAL_IO_PIN_SHIFT



// TODO:  find out how to make this one work
//#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))


// xmega-specific - Interrupt 'vector number' assignments:

// Interrupts are PORT-SPECIFIC, not pin specific.
// pin 2 on any port is always asynchronous (except for 'R' which doesn't have a pin 2)
// all other pins can manage synchronous interrupts.  'wakeup' from sleep mode
// and other async interrupts must be on a 'pin 2', on ports A through E
//
// Each port has 2 separate interrupt vectors.  They can be assigned different pins.
// The same pin can also be assigned to both vectors on the same port, if desired.

#define PORTD_INT0  0
#define PORTD_INT1  1
#define PORTC_INT0  2
#define PORTC_INT1  3
#define PORTE_INT0  4
#define PORTE_INT1  5
#define PORTA_INT0  6
#define PORTA_INT1  7
#define PORTB_INT0  8
#define PORTB_INT1  9
#define PORTR_INT0  10
#define PORTR_INT1  11
#define PORTF_INT0  12
#define PORTF_INT1  13
#define PORTH_INT0  14
#define PORTH_INT1  15
#define PORTJ_INT0  16
#define PORTJ_INT1  17
#define PORTK_INT0  18
#define PORTK_INT1  19
#define PORTQ_INT0  20
#define PORTQ_INT1  21

#define EXTERNAL_NUM_INTERRUPTS 22 /* defined here instead of wiring_private.h */

// was in wiring_external.h, moved here
#define EXTERNAL_INT_0  0
#define EXTERNAL_INT_1  1
#define EXTERNAL_INT_2  2
#define EXTERNAL_INT_3  3
#define EXTERNAL_INT_4  4
#define EXTERNAL_INT_5  5
#define EXTERNAL_INT_6  6
#define EXTERNAL_INT_7  7
#define EXTERNAL_INT_8  8
#define EXTERNAL_INT_9  9
#define EXTERNAL_INT_10 10
#define EXTERNAL_INT_11 11
#define EXTERNAL_INT_12 12
#define EXTERNAL_INT_13 13
#define EXTERNAL_INT_14 14
#define EXTERNAL_INT_15 15
#define EXTERNAL_INT_16 16
#define EXTERNAL_INT_17 17
#define EXTERNAL_INT_18 18
#define EXTERNAL_INT_19 19
#define EXTERNAL_INT_20 20
#define EXTERNAL_INT_21 21



// xmega has 2 sets of UART and SPI.
// The default UART is assigned on Port D, pins PD2-3
// The default SPI is assigned on Port C, pins PC4-7
//
// Also there are multiple 2-wire ports, the default being assigned to PE0-1
// TODO:  assign to PC0-1 (TWIC) since TWIE appears to be broke-dick
//
// Standard GPIO pins are assigned as follows:
// PD0-7 Digital 0-7
// PC0-7 Digital 8-15
// PE0-7 digital 16-23
// PF0-7 digital 24-31
// PH0-7 digital 32-39
// PJ0-7 digital 40-47
// PK0-7 digital 48-55
// PQ0-3 digital 56-59
// PR0-1 digital 60-61
// PA0-7 analog A0-A7
// PB0-3 analog A8-A11
//
// '#define'ing DIGITAL_IO_PIN_SHIFT shifts this down by 2, and places PD0-1 on 20-21
// This is for Arduino 'atmega' compatibility with respect to existing shields, so that
// you don't have to re-map pin numbers with #defines in existing software that hard-codes them
// or makes assumptions about pin numbers vs functionality [except TWI won't ever match up]
//
// ALL PORT REMAP registers must be assigned to 0 (default mappings for pins)
// this puts PWM output on pins 0-3 for PORT E (the timers are split for C and D)
// Additionally, CLKOUT should be 0 (no clock outputs on any port/pin).
//
// TIMERS
// Timer 0 should be configured as 'Tx2' (for 8 PWM outputs) by default, essentially
// as a dual 8-bit timer, more or less compatible with the Arduino's 3 timers and
// supporting all 8 pins on ports C and D for PWM output.  Port C's timer supports
// the system clock.
//
// See 'D' manual chapter 13 for more on this

// --------------------------------------------
// DEFINITIONS FOR SERIAL PORTS AND DEFAULT TWI
// --------------------------------------------


#define DEFAULT_TWI TWIC /* for now, later maybe TWIE? */
#define TWIC_VECT_ENABLE /* use this to select the correct interrupt vectors */

// serial port 0
#define SERIAL_0_PORT_NAME PORTD
#define SERIAL_0_USART_NAME USARTD0
#define SERIAL_0_USART_DATA USARTD0_DATA
#define SERIAL_0_RXC_ISR ISR(USARTD0_RXC_vect)
#define SERIAL_0_DRE_ISR ISR(USARTD0_DRE_vect)
//#define SERIAL_0_REMAP PORTD_REMAP /* define THIS to re-map the pins from 0-3 to 4-7 on serial port 0 */
#define SERIAL_0_REMAP_BIT 4    /* the bit needed to remap the port if SERIAL_0_REMAP is defined */
#define SERIAL_0_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_0_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTD0_VECTOR_EXISTS

// serial port 1
#define SERIAL_1_PORT_NAME PORTC
#define SERIAL_1_USART_NAME USARTC0
#define SERIAL_1_USART_DATA USARTC0_DATA
#define SERIAL_1_RXC_ISR ISR(USARTC0_RXC_vect)
#define SERIAL_1_DRE_ISR ISR(USARTC0_DRE_vect)
//#define SERIAL_1_REMAP PORTC_REMAP /* define THIS to re-map the pins from 0-3 to 4-7 on serial port 1 */
#define SERIAL_1_REMAP_BIT 4    /* the bit needed to remap the port if SERIAL_1_REMAP is defined */
#define SERIAL_1_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_1_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTC0_VECTOR_EXISTS

// serial port 2
#define SERIAL_2_PORT_NAME PORTE
#define SERIAL_2_USART_NAME USARTE0
#define SERIAL_2_USART_DATA USARTE0_DATA
#define SERIAL_2_RXC_ISR ISR(USARTE0_RXC_vect)
#define SERIAL_2_DRE_ISR ISR(USARTE0_DRE_vect)
//#define SERIAL_2_REMAP PORTE_REMAP /* define THIS to re-map the pins from 0-3 to 4-7 on serial port 2 */
#define SERIAL_2_REMAP_BIT 4    /* the bit needed to remap the port if SERIAL_1_REMAP is defined */
#define SERIAL_2_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_2_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTE0_VECTOR_EXISTS

// serial port 3
#define SERIAL_3_PORT_NAME PORTF
#define SERIAL_3_USART_NAME USARTF0
#define SERIAL_3_USART_DATA USARTF0_DATA
#define SERIAL_3_RXC_ISR ISR(USARTF0_RXC_vect)
#define SERIAL_3_DRE_ISR ISR(USARTF0_DRE_vect)
//#define SERIAL_3_REMAP PORTF_REMAP /* define THIS to re-map the pins from 0-3 to 4-7 on serial port 3 */
#define SERIAL_3_REMAP_BIT 4    /* the bit needed to remap the port if SERIAL_1_REMAP is defined */
#define SERIAL_3_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_3_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTF0_VECTOR_EXISTS



// For atmega/Arduino shield compatibility, with DIGITAL_IO_PIN_SHIFT defined
// typical board/pin layout might be like this:
//
//      
//                                                                 R I
//                                                                 E O N
//                    A A A A A A                        V G G 3   S R .
//                    1 1 1 1 1 1 A A   A A A A A A A A  I N N V 5 E E C
//                    5 4 3 2 1 0 9 8   7 6 5 4 3 2 1 0  n D D 3 V T F .
//              {  } -o-o-o-o-o-o-o-o---o-o-o-o-o-o-o-o--o-o-o-o-o-o-o-o-----------
//                    
//      55 -o-o       
//      53 -o-o       
//      51 -o-o
//      49 -o-o 48
//      47 -o-o 46
//      45 -o-o 44
//      43 -o-o 42
//      41 -o-o 40
//      39 -o-o 38
//      37 -o-o 36                     T O P   V I E W
//      35 -o-o 34
//      33 -o-o 32
//      31 -o-o 30
//      29 -o-o 28
//      27 -o-o
//      25 -o-o
//      23 -o-o
//         -o-o {  } -----o-o-o-o-o-o-o-o---o-o-o-o-o-o---o-o-o-o-o-o-o-o-o-o-o-o---
//          5 5           2 2 1 1 1 1 1 1                         1 1 1 1 G A S S
//          V V           1 0 9 8 7 6 5 4   0 1 2 3 4 5   6 7 8 9 0 1 2 3 N R C D
//                        S S R T R T R T   R T                   S M M S D E L A
//                        C D x x x x x x   x x                   S O I C   F
//                        L A 1 1 2 2 3 3   0 0                     S S K
//                                                                  I O
//
// As with other 'mega' Arduino boards, additional connectors would
// break out the additional pins, with appropriate labeling.
//
// This layout is based on the 'Rev 3' Arduino mega 2560.
//
// NOTE - NO AREF:  AREF is not connected.  AREF is a bit of an issue on xmega because
// it DOES! NOT! WORK! THE! SAME! as it does on the ATmegaXXX and so you would need to
// (literally) steal one of the additional analog input pins to implement it. It's not
// impossible, or even THAT difficult.  I'm just not doing it here.



#ifdef DIGITAL_IO_PIN_SHIFT // aka digital I/O pin 0 is PORTD pin 2

// SHIFTED I/O pins (atmega compatibility) - see xmega mod description in comment at top

// default SPI
static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

// primary SPI on PC4-7
static const uint8_t SS0   = 10;
static const uint8_t MOSI0 = 11;
static const uint8_t MISO0 = 12;
static const uint8_t SCK0  = 13;

// secondary SPI on PD4-7
static const uint8_t SS1   = 2;
static const uint8_t MOSI1 = 3;
static const uint8_t MISO1 = 4;
static const uint8_t SCK1  = 5;

// default 2-wire on PE0,PE1 - TWIE  (for TWIC, you're on your own)
// NOTE:  TWIE appears it may be broken, so switch to TWIC?
static const uint8_t SDA = 14;
static const uint8_t SCL = 15;

// TODO:  alternate 2-wire ports

// keep track of the indices for port R since its control register
// settings may be slightly different
#define PR0 58
#define PR1 59

#else // no digital I/O pin shifting, PORTD pin 0 is digital I/O pin 0 (as it should be)

// default "no shift" pin assignments

// default SPI
static const uint8_t SS   = 12;
static const uint8_t MOSI = 13;
static const uint8_t MISO = 14;
static const uint8_t SCK  = 15;

// primary SPI on PC4-7
static const uint8_t SS0   = 12;
static const uint8_t MOSI0 = 13;
static const uint8_t MISO0 = 14;
static const uint8_t SCK0  = 15;

// secondary SPI on PD4-7
static const uint8_t SS1   = 4;
static const uint8_t MOSI1 = 5;
static const uint8_t MISO1 = 6;
static const uint8_t SCK1  = 7;

// default 2-wire on PE0,PE1 - TWIE  (for TWIC, you're on your own)
// NOTE:  TWIE appears it may be broken, so switch to TWIC?
// NOTE:  this does NOT correspond to the mega2560, which uses 20 and 21 (need more remap work)
static const uint8_t SDA = 16;
static const uint8_t SCL = 17;

// TODO:  alternate 2-wire ports TWIC?

// keep track of the indices for port R since its control register
// settings should be slightly different - D manual table 11-6
#define PR0 60
#define PR1 61

#endif // DIGITAL_IO_PIN_SHIFT


// default 'status' LED on PR1
//static const uint8_t LED_BUILTIN = PR1;
#define LED_BUILTIN PR1 /* Arduino 1.06 uses #define, not a const uint8_t */


static const uint8_t A0 = 62;
static const uint8_t A1 = 63;
static const uint8_t A2 = 64;
static const uint8_t A3 = 65;
static const uint8_t A4 = 66;
static const uint8_t A5 = 67;
static const uint8_t A6 = 68;
static const uint8_t A7 = 69;
static const uint8_t A8 = 70;
static const uint8_t A9 = 71;
static const uint8_t A10 = 72;
static const uint8_t A11 = 73;
static const uint8_t A12 = 74;
static const uint8_t A13 = 75;
static const uint8_t A14 = 76;
static const uint8_t A15 = 77;

// on the xmega, PA2, PB2, PC2, PD2, and PE2 are asynchronous ints.  Others are 'synchronous' which means
// that they must be held in their 'interrupt state' long enough for the system to detect them.  In any case
// all digital input pins can be use as interrupts, synchronous or otherwise.


// this is the megaxxxx code
//// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
//// Only pins available for RECEIVE (TRANSMIT can be on any pin):
//// (I've deliberately left out pin mapping to the Hardware USARTs - seems senseless to me)
//// Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69
//
//// NOTE:  these have to be completely re-written to work on the xmega
//

// TODO:  write this?  for now, leave it out

// old mega code for reference
#if 0
#define digitalPinToPCICR(p)    ( (((p) >= 10) && ((p) <= 13)) || \
                                  (((p) >= 50) && ((p) <= 53)) || \
                                  (((p) >= 62) && ((p) <= 69)) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? 2 : \
                                0 ) )

#define digitalPinToPCMSK(p)    ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? (&PCMSK0) : \
                                ( (((p) >= 62) && ((p) <= 69)) ? (&PCMSK2) : \
                                ((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) ( (((p) >= 10) && ((p) <= 13)) ? ((p) - 6) : \
                                ( ((p) == 50) ? 3 : \
                                ( ((p) == 51) ? 2 : \
                                ( ((p) == 52) ? 1 : \
                                ( ((p) == 53) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? ((p) - 62) : \
                                0 ) ) ) ) ) )
#endif // 0



#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
  NOT_A_PORT,                  // 0
  (uint16_t) &PORTA_DIR,       // PA
  (uint16_t) &PORTB_DIR,       // PB
  (uint16_t) &PORTC_DIR,       // PC
  (uint16_t) &PORTD_DIR,       // PD
  (uint16_t) &PORTE_DIR,       // PE
  (uint16_t) &PORTR_DIR,       // PR
  (uint16_t) &PORTF_DIR,       // PF
  (uint16_t) &PORTH_DIR,       // PH
  (uint16_t) &PORTJ_DIR,       // PJ
  (uint16_t) &PORTK_DIR,       // PK
  (uint16_t) &PORTQ_DIR,       // PQ
};

const uint16_t PROGMEM port_to_output_PGM[] = {
  NOT_A_PORT,                  // 0
  (uint16_t) &PORTA_OUT,       // PA
  (uint16_t) &PORTB_OUT,       // PB
  (uint16_t) &PORTC_OUT,       // PC
  (uint16_t) &PORTD_OUT,       // PD
  (uint16_t) &PORTE_OUT,       // PE
  (uint16_t) &PORTR_OUT,       // PR
  (uint16_t) &PORTF_OUT,       // PF
  (uint16_t) &PORTH_OUT,       // PH
  (uint16_t) &PORTJ_OUT,       // PJ
  (uint16_t) &PORTK_OUT,       // PK
  (uint16_t) &PORTQ_OUT,       // PQ
};

const uint16_t PROGMEM port_to_input_PGM[] = {
  NOT_A_PORT,                 // 0
  (uint16_t) &PORTA_IN,       // PA
  (uint16_t) &PORTB_IN,       // PB
  (uint16_t) &PORTC_IN,       // PC
  (uint16_t) &PORTD_IN,       // PD
  (uint16_t) &PORTE_IN,       // PE
  (uint16_t) &PORTR_IN,       // PR
  (uint16_t) &PORTF_IN,       // PF
  (uint16_t) &PORTH_IN,       // PH
  (uint16_t) &PORTJ_IN,       // PJ
  (uint16_t) &PORTK_IN,       // PK
  (uint16_t) &PORTQ_IN,       // PQ
};

// xmega has a per-pin config register as well.  Normally these will be 00000111 for analog, 00000000 for digital 'totem pole'
// for 'INPUT_PULLUP' these will be 00011111
//   bits 2:0 (trigger)  000 both edges  001 rising  010 falling  011 level  111 input buffer disabled
//            note:  'input buffer disabled' required to use the 'IN' register (so default here)
//                   also port R does not support 'INTPUT_DISABLED' (sic) so use BOTHEDGES [0] instead
//   bits 5:3 (out/pull) 000 TOTEM [normal], 001 bus keeper [sticky], 010 pulldown, 011 pullup,
//                       100 wired 'or', 101 wired 'and', 110 wired 'or' pulldown, 111 wired 'and' pullup
//   bit 6:  "invert logic" (0 = normal, 1 = inverted)
//   bit 7:  unused, must be zero
// NOTE:  PORTA through PORTQ support 'input buffer disabled' and this setting is recommended
//        for analog inputs.  PORTR apparently does NOT support this (set to zero?)

const uint16_t PROGMEM digital_pin_to_control_PGM[] = {
#ifndef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN0CTRL,  // PD 0 ** 0 **
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN2CTRL,  // PD 2 ** 2 ** USARTD_RX     ASYNC
  (uint16_t) &PORTD_PIN3CTRL,  // PD 3 ** 3 ** USARTD_TX
  (uint16_t) &PORTD_PIN4CTRL,  // PD 4 ** 4 **
  (uint16_t) &PORTD_PIN5CTRL,  // PD 5 ** 5 **
  (uint16_t) &PORTD_PIN6CTRL,  // PD 6 ** 6 **
  (uint16_t) &PORTD_PIN7CTRL,  // PD 7 ** 7 **
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0 ** 8 **
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1 ** 9 **
  (uint16_t) &PORTC_PIN2CTRL,  // PC 2 ** 10 **              ASYNC
  (uint16_t) &PORTC_PIN3CTRL,  // PC 3 ** 11 **
  (uint16_t) &PORTC_PIN4CTRL,  // PC 4 ** 12 ** SPI_SS
  (uint16_t) &PORTC_PIN5CTRL,  // PC 5 ** 13 ** SPI_MOSI
  (uint16_t) &PORTC_PIN6CTRL,  // PC 6 ** 14 ** SPI_MISO
  (uint16_t) &PORTC_PIN7CTRL,  // PC 7 ** 15 ** SPI_SCK
  (uint16_t) &PORTE_PIN0CTRL,  // PE 0 ** 16 ** SDA
  (uint16_t) &PORTE_PIN1CTRL,  // PE 1 ** 17 ** SCL
  (uint16_t) &PORTE_PIN2CTRL,  // PE 2 ** 18 **              ASYNC
  (uint16_t) &PORTE_PIN3CTRL,  // PE 3 ** 19 **

  (uint16_t) &PORTE_PIN4CTRL,  // PE 3 ** 20 **
  (uint16_t) &PORTE_PIN5CTRL,  // PE 3 ** 21 **
  (uint16_t) &PORTE_PIN6CTRL,  // PE 3 ** 22 **
  (uint16_t) &PORTE_PIN7CTRL,  // PE 3 ** 23 **

  (uint16_t) &PORTF_PIN0CTRL,  // PF 0 ** 24 **
  (uint16_t) &PORTF_PIN1CTRL,  // PF 1 ** 25 **
  (uint16_t) &PORTF_PIN2CTRL,  // PF 2 ** 26 **              ASYNC
  (uint16_t) &PORTF_PIN3CTRL,  // PF 3 ** 27 **
  (uint16_t) &PORTF_PIN4CTRL,  // PF 4 ** 28 **
  (uint16_t) &PORTF_PIN5CTRL,  // PF 5 ** 29 **
  (uint16_t) &PORTF_PIN6CTRL,  // PF 6 ** 30 **
  (uint16_t) &PORTF_PIN7CTRL,  // PF 7 ** 31 **

  (uint16_t) &PORTH_PIN0CTRL,  // PH 0 ** 32 **
  (uint16_t) &PORTH_PIN1CTRL,  // PH 1 ** 33 **
  (uint16_t) &PORTH_PIN2CTRL,  // PH 2 ** 34 **              ASYNC
  (uint16_t) &PORTH_PIN3CTRL,  // PH 3 ** 35 **
  (uint16_t) &PORTH_PIN4CTRL,  // PH 4 ** 36 **
  (uint16_t) &PORTH_PIN5CTRL,  // PH 5 ** 37 **
  (uint16_t) &PORTH_PIN6CTRL,  // PH 6 ** 38 **
  (uint16_t) &PORTH_PIN7CTRL,  // PH 7 ** 39 **

  (uint16_t) &PORTJ_PIN0CTRL,  // PJ 0 ** 40 **
  (uint16_t) &PORTJ_PIN1CTRL,  // PJ 1 ** 41 **
  (uint16_t) &PORTJ_PIN2CTRL,  // PJ 2 ** 42 **              ASYNC
  (uint16_t) &PORTJ_PIN3CTRL,  // PJ 3 ** 43 **
  (uint16_t) &PORTJ_PIN4CTRL,  // PJ 4 ** 44 **
  (uint16_t) &PORTJ_PIN5CTRL,  // PJ 5 ** 45 **
  (uint16_t) &PORTJ_PIN6CTRL,  // PJ 6 ** 46 **
  (uint16_t) &PORTJ_PIN7CTRL,  // PJ 7 ** 47 **

  (uint16_t) &PORTK_PIN0CTRL,  // PK 0 ** 48 **
  (uint16_t) &PORTK_PIN1CTRL,  // PK 1 ** 49 **
  (uint16_t) &PORTK_PIN2CTRL,  // PK 2 ** 50 **              ASYNC
  (uint16_t) &PORTK_PIN3CTRL,  // PK 3 ** 51 **
  (uint16_t) &PORTK_PIN4CTRL,  // PK 4 ** 52 **
  (uint16_t) &PORTK_PIN5CTRL,  // PK 5 ** 53 **
  (uint16_t) &PORTK_PIN6CTRL,  // PK 6 ** 54 **
  (uint16_t) &PORTK_PIN7CTRL,  // PK 7 ** 55 **

  (uint16_t) &PORTQ_PIN0CTRL,  // PQ 0 ** 56 **
  (uint16_t) &PORTQ_PIN1CTRL,  // PQ 1 ** 57 **
  (uint16_t) &PORTQ_PIN2CTRL,  // PQ 2 ** 58 **              ASYNC
  (uint16_t) &PORTQ_PIN3CTRL,  // PQ 3 ** 59 **

  (uint16_t) &PORTR_PIN0CTRL,  // PR 0 ** 60 **
  (uint16_t) &PORTR_PIN1CTRL,  // PR 1 ** 61 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN0CTRL,  // PD 0 ** the new 60 **
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1 ** the new 61 **
#endif // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTA_PIN0CTRL,  // PA 0 ** 62 ** A0
  (uint16_t) &PORTA_PIN1CTRL,  // PA 1 ** 63 ** A1
  (uint16_t) &PORTA_PIN2CTRL,  // PA 2 ** 64 ** A2           ASYNC
  (uint16_t) &PORTA_PIN3CTRL,  // PA 3 ** 65 ** A3
  (uint16_t) &PORTA_PIN4CTRL,  // PA 4 ** 66 ** A4
  (uint16_t) &PORTA_PIN5CTRL,  // PA 5 ** 67 ** A5
  (uint16_t) &PORTA_PIN6CTRL,  // PA 6 ** 68 ** A6
  (uint16_t) &PORTA_PIN7CTRL,  // PA 7 ** 69 ** A7
  (uint16_t) &PORTB_PIN0CTRL,  // PB 0 ** 70 ** A8
  (uint16_t) &PORTB_PIN1CTRL,  // PB 1 ** 71 ** A9
  (uint16_t) &PORTB_PIN2CTRL,  // PB 2 ** 72 ** A10         ASYNC
  (uint16_t) &PORTB_PIN3CTRL,  // PB 3 ** 73 ** A11
  (uint16_t) &PORTB_PIN4CTRL,  // PB 4 ** 74 ** A12
  (uint16_t) &PORTB_PIN5CTRL,  // PB 5 ** 75 ** A13
  (uint16_t) &PORTB_PIN6CTRL,  // PB 6 ** 76 ** A14
  (uint16_t) &PORTB_PIN7CTRL,  // PB 7 ** 77 ** A15
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  // PORTLIST
  // -------------------------------------------
#ifndef DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 0 ** 0 **
  _PD,  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 2 ** 2 ** USARTD_RX
  _PD,  // PD 3 ** 3 ** USARTD_TX
  _PD,  // PD 4 ** 4 **
  _PD,  // PD 5 ** 5 **
  _PD,  // PD 6 ** 6 **
  _PD,  // PD 7 ** 7 **
  _PC,  // PC 0 ** 8 **
  _PC,  // PC 1 ** 9 **
  _PC,  // PC 2 ** 10 **
  _PC,  // PC 3 ** 11 **
  _PC,  // PC 4 ** 12 ** SPI_SS
  _PC,  // PC 5 ** 13 ** SPI_MOSI
  _PC,  // PC 6 ** 14 ** SPI_MISO
  _PC,  // PC 7 ** 15 ** SPI_SCK
  _PE,  // PE 0 ** 16 ** SDA
  _PE,  // PE 1 ** 17 ** SCL
  _PE,  // PE 2 ** 18 **
  _PE,  // PE 3 ** 19 **

  _PE,  // PE 4 ** 20 **
  _PE,  // PE 5 ** 21 **
  _PE,  // PE 6 ** 22 **
  _PE,  // PE 7 ** 23 **

  _PF,  // PF 0 ** 24 ** SDA
  _PF,  // PF 1 ** 25 ** SCL
  _PF,  // PF 2 ** 26 **
  _PF,  // PF 3 ** 27 **
  _PF,  // PF 4 ** 28 **
  _PF,  // PF 5 ** 29 **
  _PF,  // PF 6 ** 30 **
  _PF,  // PF 7 ** 31 **

  _PH,  // PH 0 ** 32 ** SDA
  _PH,  // PH 1 ** 33 ** SCL
  _PH,  // PH 2 ** 34 **
  _PH,  // PH 3 ** 35 **
  _PH,  // PH 4 ** 36 **
  _PH,  // PH 5 ** 37 **
  _PH,  // PH 6 ** 38 **
  _PH,  // PH 7 ** 39 **

  _PJ,  // PJ 0 ** 40 ** SDA
  _PJ,  // PJ 1 ** 41 ** SCL
  _PJ,  // PJ 2 ** 42 **
  _PJ,  // PJ 3 ** 43 **
  _PJ,  // PJ 4 ** 44 **
  _PJ,  // PJ 5 ** 45 **
  _PJ,  // PJ 6 ** 46 **
  _PJ,  // PJ 7 ** 47 **

  _PK,  // PK 0 ** 48 ** SDA
  _PK,  // PK 1 ** 49 ** SCL
  _PK,  // PK 2 ** 50 **
  _PK,  // PK 3 ** 51 **
  _PK,  // PK 4 ** 52 **
  _PK,  // PK 5 ** 53 **
  _PK,  // PK 6 ** 54 **
  _PK,  // PK 7 ** 55 **

  _PQ,  // PE 0 ** 56 ** SDA
  _PQ,  // PE 1 ** 57 ** SCL
  _PQ,  // PE 2 ** 58 **
  _PQ,  // PE 3 ** 59 **

  _PR,  // PR 0 ** 60 **
  _PR,  // PR 1 ** 61 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 0 ** the new 60 **
  _PD,  // PD 1 ** the new 61 **
#endif // DIGITAL_IO_PIN_SHIFT
  _PA,  // PA 0 ** 62 ** A0
  _PA,  // PA 1 ** 63 ** A1
  _PA,  // PA 2 ** 64 ** A2
  _PA,  // PA 3 ** 65 ** A3
  _PA,  // PA 4 ** 66 ** A4
  _PA,  // PA 5 ** 67 ** A5
  _PA,  // PA 6 ** 68 ** A6
  _PA,  // PA 7 ** 69 ** A7
  _PB,  // PB 0 ** 70 ** A8
  _PB,  // PB 1 ** 71 ** A9
  _PB,  // PB 2 ** 72 ** A10
  _PB,  // PB 3 ** 73 ** A11
  _PB,  // PB 4 ** 74 ** A12
  _PB,  // PB 5 ** 75 ** A13
  _PB,  // PB 6 ** 76 ** A14
  _PB,  // PB 7 ** 77 ** A15
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  // PIN IN PORT
  // -------------------------------------------
#ifndef DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PD 0 ** 0 **
  _BV( 1 ),  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 2 ),  // PD 2 ** 2 ** USARTD_RX
  _BV( 3 ),  // PD 3 ** 3 ** USARTD_TX
  _BV( 4 ),  // PD 4 ** 4 **
  _BV( 5 ),  // PD 5 ** 5 **
  _BV( 6 ),  // PD 6 ** 6 **
  _BV( 7 ),  // PD 7 ** 7 **
  _BV( 0 ),  // PC 0 ** 8 ** SDA TWIC
  _BV( 1 ),  // PC 1 ** 9 ** SCL TWIC
  _BV( 2 ),  // PC 2 ** 10 ** USARTC_RX
  _BV( 3 ),  // PC 3 ** 11 ** USARTC_TX
  _BV( 4 ),  // PC 4 ** 12 ** SPI_SS
  _BV( 5 ),  // PC 5 ** 13 ** SPI_MOSI
  _BV( 6 ),  // PC 6 ** 14 ** SPI_MISO
  _BV( 7 ),  // PC 7 ** 15 ** SPI_SCK
  _BV( 0 ),  // PE 0 ** 16 ** SDA TWIE
  _BV( 1 ),  // PE 1 ** 17 ** SCL TWIE
  _BV( 2 ),  // PE 2 ** 18 **
  _BV( 3 ),  // PE 3 ** 19 **
  _BV( 4 ),  // PE 4 ** 20 **
  _BV( 5 ),  // PE 5 ** 21 **
  _BV( 6 ),  // PE 6 ** 22 **
  _BV( 7 ),  // PE 7 ** 23 **

  _BV( 0 ),  // PF 0 ** 24 **
  _BV( 1 ),  // PF 1 ** 25 **
  _BV( 2 ),  // PF 2 ** 26 **
  _BV( 3 ),  // PF 3 ** 27 **
  _BV( 4 ),  // PF 4 ** 28 **
  _BV( 5 ),  // PF 5 ** 29 **
  _BV( 6 ),  // PF 6 ** 30 **
  _BV( 7 ),  // PF 7 ** 31 **

  _BV( 0 ),  // PH 0 ** 32 **
  _BV( 1 ),  // PH 1 ** 33 **
  _BV( 2 ),  // PH 2 ** 34 **
  _BV( 3 ),  // PH 3 ** 35 **
  _BV( 4 ),  // PH 4 ** 36 **
  _BV( 5 ),  // PH 5 ** 37 **
  _BV( 6 ),  // PH 6 ** 38 **
  _BV( 7 ),  // PH 7 ** 39 **

  _BV( 0 ),  // PJ 0 ** 40 **
  _BV( 1 ),  // PJ 1 ** 41 **
  _BV( 2 ),  // PJ 2 ** 42 **
  _BV( 3 ),  // PJ 3 ** 43 **
  _BV( 4 ),  // PJ 4 ** 44 **
  _BV( 5 ),  // PJ 5 ** 45 **
  _BV( 6 ),  // PJ 6 ** 46 **
  _BV( 7 ),  // PJ 7 ** 47 **

  _BV( 0 ),  // PK 0 ** 48 **
  _BV( 1 ),  // PK 1 ** 49 **
  _BV( 2 ),  // PK 2 ** 50 **
  _BV( 3 ),  // PK 3 ** 51 **
  _BV( 4 ),  // PK 4 ** 52 **
  _BV( 5 ),  // PK 5 ** 53 **
  _BV( 6 ),  // PK 6 ** 54 **
  _BV( 7 ),  // PK 7 ** 55 **

  _BV( 0 ),  // PQ 0 ** 56 **
  _BV( 1 ),  // PQ 1 ** 57 **
  _BV( 2 ),  // PQ 2 ** 58 **
  _BV( 3 ),  // PQ 3 ** 59 **

  _BV( 0 ),  // PR 0 ** 60 **
  _BV( 1 ),  // PR 1 ** 61 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PD 0 ** the new 60 **
  _BV( 1 ),  // PD 1 ** the new 61 **
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PA 0 ** 22 ** A0
  _BV( 1 ),  // PA 1 ** 23 ** A1
  _BV( 2 ),  // PA 2 ** 24 ** A2
  _BV( 3 ),  // PA 3 ** 25 ** A3
  _BV( 4 ),  // PA 4 ** 26 ** A4
  _BV( 5 ),  // PA 5 ** 27 ** A5
  _BV( 6 ),  // PA 6 ** 28 ** A6
  _BV( 7 ),  // PA 7 ** 29 ** A7
  _BV( 0 ),  // PB 0 ** 30 ** A8
  _BV( 1 ),  // PB 1 ** 31 ** A9
  _BV( 2 ),  // PB 2 ** 32 ** A10
  _BV( 3 ),  // PB 3 ** 33 ** A11
  _BV( 4 ),  // PB 4 ** 26 ** A12
  _BV( 5 ),  // PB 5 ** 27 ** A13
  _BV( 6 ),  // PB 6 ** 28 ** A14
  _BV( 7 ),  // PB 7 ** 29 ** A15
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  // TIMERS
  // -------------------------------------------
  // for now 'NOT_ON_TIMER' for all - later, assign timers based
  // on pins 0-3 being enabled as PWM out for ports A through E
  // corresponding to timers A through D (see D manual sections 11.12.14,
  // also see D manual sect 13.6 for using the 'compare' channel on 'TCx2' to generate
  // a PWM output.  Must select pin as output, _AND_ enable the 'compare' output
  // for the appropriate pin.  LCMPENx/HCMPENx registers to enable it.

  // TODO:  correctly implement for xmega - PORTC, PORTD, and PORTE are all PWM capable!

#ifndef DIGITAL_IO_PIN_SHIFT
  TIMERD2,       // PD 0 ** 0 **
  TIMERD2,       // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
  TIMERD2,       // PD 2 ** 2 ** USARTD_RX
  TIMERD2,       // PD 3 ** 3 ** USARTD_TX
  TIMERD2,       // PD 4 ** 4 **
  TIMERD2,       // PD 5 ** 5 **
  TIMERD2,       // PD 6 ** 6 **
  TIMERD2,       // PD 7 ** 7 **
  TIMERC2,       // PC 0 ** 8 ** SDA TWIC
  TIMERC2,       // PC 1 ** 9 ** SCL TWIC
  TIMERC2,       // PC 2 ** 10 ** USARTD_RX
  TIMERC2,       // PC 3 ** 11 ** USARTD_TX
  TIMERC2,       // PC 4 ** 12 ** SPI_SS
  TIMERC2,       // PC 5 ** 13 ** SPI_MOSI
  TIMERC2,       // PC 6 ** 14 ** SPI_MISO
  TIMERC2,       // PC 7 ** 15 ** SPI_SCK
  TIMERE2,       // PE 0 ** 16 ** SDA TWIE
  TIMERE2,       // PE 1 ** 17 ** SCL TWIE
  TIMERE2,       // PE 2 ** 18 **
  TIMERE2,       // PE 3 ** 19 **
  TIMERE2,       // PE 4 ** 20 **
  TIMERE2,       // PE 5 ** 21 **
  TIMERE2,       // PE 6 ** 22 **
  TIMERE2,       // PE 7 ** 23 **

  TIMERF2,       // PF 0 ** 24 **
  TIMERF2,       // PF 1 ** 25 **
  TIMERF2,       // PF 2 ** 26 **
  TIMERF2,       // PF 3 ** 27 **
  TIMERF2,       // PF 4 ** 28 **
  TIMERF2,       // PF 5 ** 29 **
  TIMERF2,       // PF 6 ** 30 **
  TIMERF2,       // PF 7 ** 31 **

  NOT_ON_TIMER,  // PH 0 ** 32 **
  NOT_ON_TIMER,  // PH 1 ** 33 **
  NOT_ON_TIMER,  // PH 2 ** 34 **
  NOT_ON_TIMER,  // PH 3 ** 35 **
  NOT_ON_TIMER,  // PH 4 ** 36 **
  NOT_ON_TIMER,  // PH 5 ** 37 **
  NOT_ON_TIMER,  // PH 6 ** 38 **
  NOT_ON_TIMER,  // PH 7 ** 39 **

  NOT_ON_TIMER,  // PJ 0 ** 40 **
  NOT_ON_TIMER,  // PJ 1 ** 41 **
  NOT_ON_TIMER,  // PJ 2 ** 42 **
  NOT_ON_TIMER,  // PJ 3 ** 43 **
  NOT_ON_TIMER,  // PJ 4 ** 44 **
  NOT_ON_TIMER,  // PJ 5 ** 45 **
  NOT_ON_TIMER,  // PJ 6 ** 46 **
  NOT_ON_TIMER,  // PJ 7 ** 47 **

  NOT_ON_TIMER,  // PK 0 ** 48 **
  NOT_ON_TIMER,  // PK 1 ** 49 **
  NOT_ON_TIMER,  // PK 2 ** 50 **
  NOT_ON_TIMER,  // PK 3 ** 51 **
  NOT_ON_TIMER,  // PK 4 ** 52 **
  NOT_ON_TIMER,  // PK 5 ** 53 **
  NOT_ON_TIMER,  // PK 6 ** 54 **
  NOT_ON_TIMER,  // PK 7 ** 55 **

  NOT_ON_TIMER,  // PQ 0 ** 56 **
  NOT_ON_TIMER,  // PQ 1 ** 57 **
  NOT_ON_TIMER,  // PQ 2 ** 58 **
  NOT_ON_TIMER,  // PQ 3 ** 59 **

  NOT_ON_TIMER,  // PR 0 ** 60 **
  NOT_ON_TIMER,  // PR 1 ** 61 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  TIMERD2,       // PD 0 ** the new 60 **
  TIMERD2,       // PD 1 ** the new 61 **
#endif // DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  // PA 0 ** 62 ** A0
  NOT_ON_TIMER,  // PA 1 ** 63 ** A1
  NOT_ON_TIMER,  // PA 2 ** 64 ** A2
  NOT_ON_TIMER,  // PA 3 ** 65 ** A3
  NOT_ON_TIMER,  // PA 4 ** 66 ** A4
  NOT_ON_TIMER,  // PA 5 ** 67 ** A5
  NOT_ON_TIMER,  // PA 6 ** 68 ** A6
  NOT_ON_TIMER,  // PA 7 ** 69 ** A7
  NOT_ON_TIMER,  // PB 0 ** 70 ** A8
  NOT_ON_TIMER,  // PB 1 ** 71 ** A9
  NOT_ON_TIMER,  // PB 2 ** 72 ** A10
  NOT_ON_TIMER,  // PB 3 ** 73 ** A11
  NOT_ON_TIMER,  // PB 4 ** 74 ** A12
  NOT_ON_TIMER,  // PB 5 ** 75 ** A13
  NOT_ON_TIMER,  // PB 6 ** 76 ** A14
  NOT_ON_TIMER,  // PB 7 ** 77 ** A15

};

#endif // ARDUINO_MAIN


// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial
#define SERIAL_HARDWARE_OPEN  Serial2

#endif // Pins_Arduino_h


