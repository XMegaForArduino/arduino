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

  **************************************************************************
  This is the variant header file for Mike Nikolaiev's Rock Solid XMega 128A
  **************************************************************************

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

#define USE_AREF analogReference_PORTA0 /* see 28.16.3 in 'AU' manual - this is the REFCTRL bits for the reference select, AREF on PORTA (PA0) */


#define NUM_DIGITAL_PINS            62
#define NUM_ANALOG_INPUTS           16
#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 62 : -1)
#define digitalPinHasPWM(p)         ((p) < 22 || ((p) >= 46 && (p) <= 55)) /* < 22 or 46 through 55 which are PORTC through PORTF */

// for now mapped PA0-7 PB0-7 as-is; later may need to change this?
#define analogInputToAnalogPin(p) ((p)-A0)



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


// PORT MAPPING
// The various ports and pins are mapped as follows:



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


#define DEFAULT_TWI TWID /* this is the TWI that is mapped to 20/21 */
#define TWID_VECT_ENABLE /* use this to select the correct interrupt vectors */

// serial port 0
#define SERIAL_0_PORT_NAME PORTC
#define SERIAL_0_USART_NAME USARTC0
#define SERIAL_0_USART_DATA USARTC0_DATA
#define SERIAL_0_RXC_ISR ISR(USARTC0_RXC_vect)
#define SERIAL_0_DRE_ISR ISR(USARTC0_DRE_vect)
#define SERIAL_0_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_0_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTC0_VECTOR_EXISTS

// serial port 1
#define SERIAL_1_PORT_NAME PORTD
#define SERIAL_1_USART_NAME USARTD0
#define SERIAL_1_USART_DATA USARTD0_DATA
#define SERIAL_1_RXC_ISR ISR(USARTD0_RXC_vect)
#define SERIAL_1_DRE_ISR ISR(USARTD0_DRE_vect)
#define SERIAL_1_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_1_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTD0_VECTOR_EXISTS

// serial port 2
#define SERIAL_2_PORT_NAME PORTE
#define SERIAL_2_USART_NAME USARTE1
#define SERIAL_2_USART_DATA USARTE1_DATA
#define SERIAL_2_RXC_ISR ISR(USARTE1_RXC_vect)
#define SERIAL_2_DRE_ISR ISR(USARTE1_DRE_vect)
#define SERIAL_2_RX_PIN_INDEX 6 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_2_TX_PIN_INDEX 7 /* the pin number on the port, not the mapped digital pin number */
#define USARTE1_VECTOR_EXISTS

// serial port 3
#define SERIAL_3_PORT_NAME PORTF
#define SERIAL_3_USART_NAME USARTF0
#define SERIAL_3_USART_DATA USARTF0_DATA
#define SERIAL_3_RXC_ISR ISR(USARTF0_RXC_vect)
#define SERIAL_3_DRE_ISR ISR(USARTF0_DRE_vect)
#define SERIAL_3_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_3_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTF0_VECTOR_EXISTS



// For atmega/Arduino shield compatibility, with DIGITAL_IO_PIN_SHIFT defined
// typical board/pin layout might be like this (for shield pins):
//

// NOTE:  re-do this ASCII art for the MEGA2560 layout


//      
//                                                                 R I
//                                                                 E O N
//                    A A A A A A                        V G G 3   S R .
//                    1 1 1 1 1 1 A A   A A A A A A A A  I N N V 5 E E C
//                    5 4 3 2 1 0 9 8   7 6 5 4 3 2 1 0  n D D 3 V T F .
//              {  } -o-o-o-o-o-o-o-o---o-o-o-o-o-o-o-o--o-o-o-o-o-o-o-o-----------
//                    P P P P P P P P   P P P P P P P P
//  PF0 55 -o-o PF1   B B B B B B B B   A A A A A A A A
//  PF4 53 -o-o PF7   7 6 5 4 3 2 1 0   7 6 5 4 3 2 1 0
//  PF5 51 -o-o PF6                                                 EXTRA-CONNECTOR
//  PC0 49 -o-o 48 PC1                                                 56 -o- PQ0
//  PD6 47 -o-o 46 PD7                                                 57 -o- PQ1
//  PH4 45 -o-o 44 PH5                                                 58 -o- PQ2 - ASYNC
//  PH6 43 -o-o 42 PH7                                                 59 -o- PQ3 - LED_DEFAULT
//  PH0 41 -o-o 40 PH1                                                 60 -o- PR0 - XTAL1
//  PH2 39 -o-o 38 PH3                                                 61 -o- PR1 - XTAL2
//  PK0 37 -o-o 36 PK1                 T O P   V I E W                    -o- GND
//  PK2 35 -o-o 34 PK3                                                    -o- N/C
//  PK4 33 -o-o 32 PJ5
//  PK6 31 -o-o 30 PK7
//  PJ7 29 -o-o 28 PJ6
//  PJ5 27 -o-o PJ4       P P P P P P P P   P P P P P P   P P P P P P P P
//  PJ3 25 -o-o PJ2       D D D D E E F F   C C E E E E   E E D D C C C C
//  PJ1 23 -o-o PJ0       1 0 2 3 6 7 2 3   2 3 5 4 3 2   1 0 5 4 4 5 6 7
//         -o-o {  } -----o-o-o-o-o-o-o-o---o-o-o-o-o-o---o-o-o-o-o-o-o-o-o-o-o-o---
//          5 5           2 2 1 1 1 1 1 1                         1 1 1 1 G A S S
//          V V           1 0 9 8 7 6 5 4   0 1 2 3 4 5   6 7 8 9 0 1 2 3 N R C D
//                        S S R T R T R T   R T                   S M M S D E L A
//                        C D x x x x x x   x x                   S O I C   F
//                        L A 1 1 2 2 3 3   0 0                     S S K
//                                                                  I O


// As with other 'mega' Arduino boards, additional connectors would
// break out the additional pins, with appropriate labeling.
//
// This layout is based on the 'Rev 3' Arduino mega 2560
//
// NOTE - NO AREF:  AREF is not connected.  AREF is a bit of an issue on xmega because
// it DOES! NOT! WORK! THE! SAME! as it does on the ATmegaXXX and so you would need to
// (literally) steal one of the additional analog input pins to implement it. It's not
// impossible, or even THAT difficult.  I'm just not doing it here.



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

// keep track of the indices for ports Q and R since their control register
// settings may be slightly different
#define PQ0 56
#define PQ1 57
#define PQ2 58
#define PQ3 59
#define PR0 60
#define PR1 61


// default 'status' LED on PR1
//static const uint8_t LED_BUILTIN = PR1;
#define LED_BUILTIN PQ3 /* Arduino 1.06 uses #define, not a const uint8_t */
//#define LED_BUILTIN 13 /* this is the arduino Uno default, here for reference */


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


// on the xmega, all 'pin 2' inputs on any port are asynchronous ints.  Others are 'synchronous' which means
// that they must be held in their 'interrupt state' long enough for the system to detect them.  In any case
// all digital input pins can be use as interrupts, synchronous or otherwise.


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


// PIN DEFINITION ARRAYS - generated using formulas in a spreadsheet

const uint16_t PROGMEM digital_pin_to_control_PGM[] = {
  (uint16_t) &PORTC_PIN2CTRL,  // PC 2  ** 00 **
  (uint16_t) &PORTC_PIN3CTRL,  // PC 3  ** 01 **
  (uint16_t) &PORTE_PIN5CTRL,  // PE 5  ** 02 **
  (uint16_t) &PORTE_PIN4CTRL,  // PE 4  ** 03 **
  (uint16_t) &PORTE_PIN3CTRL,  // PE 3  ** 04 **
  (uint16_t) &PORTE_PIN2CTRL,  // PE 2  ** 05 **
  (uint16_t) &PORTE_PIN1CTRL,  // PE 1  ** 06 **
  (uint16_t) &PORTE_PIN0CTRL,  // PE 0  ** 07 **
  (uint16_t) &PORTD_PIN5CTRL,  // PD 5  ** 08 **
  (uint16_t) &PORTD_PIN4CTRL,  // PD 4  ** 09 **
  (uint16_t) &PORTC_PIN4CTRL,  // PC 4  ** 10 **
  (uint16_t) &PORTC_PIN5CTRL,  // PC 5  ** 11 **
  (uint16_t) &PORTC_PIN6CTRL,  // PC 6  ** 12 **
  (uint16_t) &PORTC_PIN7CTRL,  // PC 7  ** 13 **
  (uint16_t) &PORTF_PIN3CTRL,  // PF 3  ** 14 **
  (uint16_t) &PORTF_PIN2CTRL,  // PF 2  ** 15 **
  (uint16_t) &PORTE_PIN7CTRL,  // PE 7  ** 16 **
  (uint16_t) &PORTE_PIN6CTRL,  // PE 6  ** 17 **
  (uint16_t) &PORTD_PIN3CTRL,  // PD 3  ** 18 **
  (uint16_t) &PORTD_PIN2CTRL,  // PD 2  ** 19 **
  (uint16_t) &PORTD_PIN0CTRL,  // PD 0  ** 20 **
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1  ** 21 **
  (uint16_t) &PORTJ_PIN0CTRL,  // PJ 0  ** 22 **
  (uint16_t) &PORTJ_PIN1CTRL,  // PJ 1  ** 23 **
  (uint16_t) &PORTJ_PIN2CTRL,  // PJ 2  ** 24 **
  (uint16_t) &PORTJ_PIN3CTRL,  // PJ 3  ** 25 **
  (uint16_t) &PORTJ_PIN4CTRL,  // PJ 4  ** 26 **
  (uint16_t) &PORTJ_PIN5CTRL,  // PJ 5  ** 27 **
  (uint16_t) &PORTJ_PIN6CTRL,  // PJ 6  ** 28 **
  (uint16_t) &PORTJ_PIN7CTRL,  // PJ 7  ** 29 **
  (uint16_t) &PORTK_PIN7CTRL,  // PK 7  ** 30 **
  (uint16_t) &PORTK_PIN6CTRL,  // PK 6  ** 31 **
  (uint16_t) &PORTK_PIN5CTRL,  // PK 5  ** 32 **
  (uint16_t) &PORTK_PIN4CTRL,  // PK 4  ** 33 **
  (uint16_t) &PORTK_PIN3CTRL,  // PK 3  ** 34 **
  (uint16_t) &PORTK_PIN2CTRL,  // PK 2  ** 35 **
  (uint16_t) &PORTK_PIN1CTRL,  // PK 1  ** 36 **
  (uint16_t) &PORTK_PIN0CTRL,  // PK 0  ** 37 **
  (uint16_t) &PORTH_PIN3CTRL,  // PH 3  ** 38 **
  (uint16_t) &PORTH_PIN2CTRL,  // PH 2  ** 39 **
  (uint16_t) &PORTH_PIN1CTRL,  // PH 1  ** 40 **
  (uint16_t) &PORTH_PIN0CTRL,  // PH 0  ** 41 **
  (uint16_t) &PORTH_PIN7CTRL,  // PH 7  ** 42 **
  (uint16_t) &PORTH_PIN6CTRL,  // PH 6  ** 43 **
  (uint16_t) &PORTH_PIN5CTRL,  // PH 5  ** 44 **
  (uint16_t) &PORTH_PIN4CTRL,  // PH 4  ** 45 **
  (uint16_t) &PORTD_PIN7CTRL,  // PD 7  ** 46 **
  (uint16_t) &PORTD_PIN6CTRL,  // PD 6  ** 47 **
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1  ** 48 **
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0  ** 49 **
  (uint16_t) &PORTF_PIN6CTRL,  // PF 6  ** 50 **
  (uint16_t) &PORTF_PIN5CTRL,  // PF 5  ** 51 **
  (uint16_t) &PORTF_PIN7CTRL,  // PF 7  ** 52 **
  (uint16_t) &PORTF_PIN4CTRL,  // PF 4  ** 53 **
  (uint16_t) &PORTF_PIN1CTRL,  // PF 1  ** 54 **
  (uint16_t) &PORTF_PIN0CTRL,  // PF 0  ** 55 **
  (uint16_t) &PORTQ_PIN0CTRL,  // PQ 0  ** 56 **
  (uint16_t) &PORTQ_PIN1CTRL,  // PQ 1  ** 57 **
  (uint16_t) &PORTQ_PIN2CTRL,  // PQ 2  ** 58 **
  (uint16_t) &PORTQ_PIN3CTRL,  // PQ 3  ** 59 **
  (uint16_t) &PORTR_PIN0CTRL,  // PR 0  ** 60 **
  (uint16_t) &PORTR_PIN1CTRL,  // PR 1  ** 61 **
  (uint16_t) &PORTA_PIN0CTRL,  // PA 0  ** 62 **
  (uint16_t) &PORTA_PIN1CTRL,  // PA 1  ** 63 **
  (uint16_t) &PORTA_PIN2CTRL,  // PA 2  ** 64 **
  (uint16_t) &PORTA_PIN3CTRL,  // PA 3  ** 65 **
  (uint16_t) &PORTA_PIN4CTRL,  // PA 4  ** 66 **
  (uint16_t) &PORTA_PIN5CTRL,  // PA 5  ** 67 **
  (uint16_t) &PORTA_PIN6CTRL,  // PA 6  ** 68 **
  (uint16_t) &PORTA_PIN7CTRL,  // PA 7  ** 69 **
  (uint16_t) &PORTB_PIN0CTRL,  // PB 0  ** 70 **
  (uint16_t) &PORTB_PIN1CTRL,  // PB 1  ** 71 **
  (uint16_t) &PORTB_PIN2CTRL,  // PB 2  ** 72 **
  (uint16_t) &PORTB_PIN3CTRL,  // PB 3  ** 73 **
  (uint16_t) &PORTB_PIN4CTRL,  // PB 4  ** 74 **
  (uint16_t) &PORTB_PIN5CTRL,  // PB 5  ** 75 **
  (uint16_t) &PORTB_PIN6CTRL,  // PB 6  ** 76 **
  (uint16_t) &PORTB_PIN7CTRL,  // PB 7  ** 77 **
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  _PC,    // PC 2  ** 00 **
  _PC,    // PC 3  ** 01 **
  _PE,    // PE 5  ** 02 **
  _PE,    // PE 4  ** 03 **
  _PE,    // PE 3  ** 04 **
  _PE,    // PE 2  ** 05 **
  _PE,    // PE 1  ** 06 **
  _PE,    // PE 0  ** 07 **
  _PD,    // PD 5  ** 08 **
  _PD,    // PD 4  ** 09 **
  _PC,    // PC 4  ** 10 **
  _PC,    // PC 5  ** 11 **
  _PC,    // PC 6  ** 12 **
  _PC,    // PC 7  ** 13 **
  _PF,    // PF 3  ** 14 **
  _PF,    // PF 2  ** 15 **
  _PE,    // PE 7  ** 16 **
  _PE,    // PE 6  ** 17 **
  _PD,    // PD 3  ** 18 **
  _PD,    // PD 2  ** 19 **
  _PD,    // PD 0  ** 20 **
  _PD,    // PD 1  ** 21 **
  _PJ,    // PJ 0  ** 22 **
  _PJ,    // PJ 1  ** 23 **
  _PJ,    // PJ 2  ** 24 **
  _PJ,    // PJ 3  ** 25 **
  _PJ,    // PJ 4  ** 26 **
  _PJ,    // PJ 5  ** 27 **
  _PJ,    // PJ 6  ** 28 **
  _PJ,    // PJ 7  ** 29 **
  _PK,    // PK 7  ** 30 **
  _PK,    // PK 6  ** 31 **
  _PK,    // PK 5  ** 32 **
  _PK,    // PK 4  ** 33 **
  _PK,    // PK 3  ** 34 **
  _PK,    // PK 2  ** 35 **
  _PK,    // PK 1  ** 36 **
  _PK,    // PK 0  ** 37 **
  _PH,    // PH 3  ** 38 **
  _PH,    // PH 2  ** 39 **
  _PH,    // PH 1  ** 40 **
  _PH,    // PH 0  ** 41 **
  _PH,    // PH 7  ** 42 **
  _PH,    // PH 6  ** 43 **
  _PH,    // PH 5  ** 44 **
  _PH,    // PH 4  ** 45 **
  _PD,    // PD 7  ** 46 **
  _PD,    // PD 6  ** 47 **
  _PC,    // PC 1  ** 48 **
  _PC,    // PC 0  ** 49 **
  _PF,    // PF 6  ** 50 **
  _PF,    // PF 5  ** 51 **
  _PF,    // PF 7  ** 52 **
  _PF,    // PF 4  ** 53 **
  _PF,    // PF 1  ** 54 **
  _PF,    // PF 0  ** 55 **
  _PQ,    // PQ 0  ** 56 **
  _PQ,    // PQ 1  ** 57 **
  _PQ,    // PQ 2  ** 58 **
  _PQ,    // PQ 3  ** 59 **
  _PR,    // PR 0  ** 60 **
  _PR,    // PR 1  ** 61 **
  _PA,    // PA 0  ** 62 **
  _PA,    // PA 1  ** 63 **
  _PA,    // PA 2  ** 64 **
  _PA,    // PA 3  ** 65 **
  _PA,    // PA 4  ** 66 **
  _PA,    // PA 5  ** 67 **
  _PA,    // PA 6  ** 68 **
  _PA,    // PA 7  ** 69 **
  _PB,    // PB 0  ** 70 **
  _PB,    // PB 1  ** 71 **
  _PB,    // PB 2  ** 72 **
  _PB,    // PB 3  ** 73 **
  _PB,    // PB 4  ** 74 **
  _PB,    // PB 5  ** 75 **
  _PB,    // PB 6  ** 76 **
  _PB,    // PB 7  ** 77 **
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  _BV(2),    // PC 2  ** 00 **
  _BV(3),    // PC 3  ** 01 **
  _BV(5),    // PE 5  ** 02 **
  _BV(4),    // PE 4  ** 03 **
  _BV(3),    // PE 3  ** 04 **
  _BV(2),    // PE 2  ** 05 **
  _BV(1),    // PE 1  ** 06 **
  _BV(0),    // PE 0  ** 07 **
  _BV(5),    // PD 5  ** 08 **
  _BV(4),    // PD 4  ** 09 **
  _BV(4),    // PC 4  ** 10 **
  _BV(5),    // PC 5  ** 11 **
  _BV(6),    // PC 6  ** 12 **
  _BV(7),    // PC 7  ** 13 **
  _BV(3),    // PF 3  ** 14 **
  _BV(2),    // PF 2  ** 15 **
  _BV(7),    // PE 7  ** 16 **
  _BV(6),    // PE 6  ** 17 **
  _BV(3),    // PD 3  ** 18 **
  _BV(2),    // PD 2  ** 19 **
  _BV(0),    // PD 0  ** 20 **
  _BV(1),    // PD 1  ** 21 **
  _BV(0),    // PJ 0  ** 22 **
  _BV(1),    // PJ 1  ** 23 **
  _BV(2),    // PJ 2  ** 24 **
  _BV(3),    // PJ 3  ** 25 **
  _BV(4),    // PJ 4  ** 26 **
  _BV(5),    // PJ 5  ** 27 **
  _BV(6),    // PJ 6  ** 28 **
  _BV(7),    // PJ 7  ** 29 **
  _BV(7),    // PK 7  ** 30 **
  _BV(6),    // PK 6  ** 31 **
  _BV(5),    // PK 5  ** 32 **
  _BV(4),    // PK 4  ** 33 **
  _BV(3),    // PK 3  ** 34 **
  _BV(2),    // PK 2  ** 35 **
  _BV(1),    // PK 1  ** 36 **
  _BV(0),    // PK 0  ** 37 **
  _BV(3),    // PH 3  ** 38 **
  _BV(2),    // PH 2  ** 39 **
  _BV(1),    // PH 1  ** 40 **
  _BV(0),    // PH 0  ** 41 **
  _BV(7),    // PH 7  ** 42 **
  _BV(6),    // PH 6  ** 43 **
  _BV(5),    // PH 5  ** 44 **
  _BV(4),    // PH 4  ** 45 **
  _BV(7),    // PD 7  ** 46 **
  _BV(6),    // PD 6  ** 47 **
  _BV(1),    // PC 1  ** 48 **
  _BV(0),    // PC 0  ** 49 **
  _BV(6),    // PF 6  ** 50 **
  _BV(5),    // PF 5  ** 51 **
  _BV(7),    // PF 7  ** 52 **
  _BV(4),    // PF 4  ** 53 **
  _BV(1),    // PF 1  ** 54 **
  _BV(0),    // PF 0  ** 55 **
  _BV(0),    // PQ 0  ** 56 **
  _BV(1),    // PQ 1  ** 57 **
  _BV(2),    // PQ 2  ** 58 **
  _BV(3),    // PQ 3  ** 59 **
  _BV(0),    // PR 0  ** 60 **
  _BV(1),    // PR 1  ** 61 **
  _BV(0),    // PA 0  ** 62 **
  _BV(1),    // PA 1  ** 63 **
  _BV(2),    // PA 2  ** 64 **
  _BV(3),    // PA 3  ** 65 **
  _BV(4),    // PA 4  ** 66 **
  _BV(5),    // PA 5  ** 67 **
  _BV(6),    // PA 6  ** 68 **
  _BV(7),    // PA 7  ** 69 **
  _BV(0),    // PB 0  ** 70 **
  _BV(1),    // PB 1  ** 71 **
  _BV(2),    // PB 2  ** 72 **
  _BV(3),    // PB 3  ** 73 **
  _BV(4),    // PB 4  ** 74 **
  _BV(5),    // PB 5  ** 75 **
  _BV(6),    // PB 6  ** 76 **
  _BV(7),    // PB 7  ** 77 **
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  TIMERC2,         // PC 2  ** 00 **
  TIMERC2,         // PC 3  ** 01 **
  TIMERE2,         // PE 5  ** 02 **
  TIMERE2,         // PE 4  ** 03 **
  TIMERE2,         // PE 3  ** 04 **
  TIMERE2,         // PE 2  ** 05 **
  TIMERE2,         // PE 1  ** 06 **
  TIMERE2,         // PE 0  ** 07 **
  TIMERD2,         // PD 5  ** 08 **
  TIMERD2,         // PD 4  ** 09 **
  TIMERC2,         // PC 4  ** 10 **
  TIMERC2,         // PC 5  ** 11 **
  TIMERC2,         // PC 6  ** 12 **
  TIMERC2,         // PC 7  ** 13 **
  TIMERF2,         // PF 3  ** 14 **
  TIMERF2,         // PF 2  ** 15 **
  TIMERE2,         // PE 7  ** 16 **
  TIMERE2,         // PE 6  ** 17 **
  TIMERD2,         // PD 3  ** 18 **
  TIMERD2,         // PD 2  ** 19 **
  TIMERD2,         // PD 0  ** 20 **
  TIMERD2,         // PD 1  ** 21 **
  NOT_ON_TIMER,    // PJ 0  ** 22 **
  NOT_ON_TIMER,    // PJ 1  ** 23 **
  NOT_ON_TIMER,    // PJ 2  ** 24 **
  NOT_ON_TIMER,    // PJ 3  ** 25 **
  NOT_ON_TIMER,    // PJ 4  ** 26 **
  NOT_ON_TIMER,    // PJ 5  ** 27 **
  NOT_ON_TIMER,    // PJ 6  ** 28 **
  NOT_ON_TIMER,    // PJ 7  ** 29 **
  NOT_ON_TIMER,    // PK 7  ** 30 **
  NOT_ON_TIMER,    // PK 6  ** 31 **
  NOT_ON_TIMER,    // PK 5  ** 32 **
  NOT_ON_TIMER,    // PK 4  ** 33 **
  NOT_ON_TIMER,    // PK 3  ** 34 **
  NOT_ON_TIMER,    // PK 2  ** 35 **
  NOT_ON_TIMER,    // PK 1  ** 36 **
  NOT_ON_TIMER,    // PK 0  ** 37 **
  NOT_ON_TIMER,    // PH 3  ** 38 **
  NOT_ON_TIMER,    // PH 2  ** 39 **
  NOT_ON_TIMER,    // PH 1  ** 40 **
  NOT_ON_TIMER,    // PH 0  ** 41 **
  NOT_ON_TIMER,    // PH 7  ** 42 **
  NOT_ON_TIMER,    // PH 6  ** 43 **
  NOT_ON_TIMER,    // PH 5  ** 44 **
  NOT_ON_TIMER,    // PH 4  ** 45 **
  TIMERD2,         // PD 7  ** 46 **
  TIMERD2,         // PD 6  ** 47 **
  TIMERC2,         // PC 1  ** 48 **
  TIMERC2,         // PC 0  ** 49 **
  TIMERF2,         // PF 6  ** 50 **
  TIMERF2,         // PF 5  ** 51 **
  TIMERF2,         // PF 7  ** 52 **
  TIMERF2,         // PF 4  ** 53 **
  TIMERF2,         // PF 1  ** 54 **
  TIMERF2,         // PF 0  ** 55 **
  NOT_ON_TIMER,    // PQ 0  ** 56 **
  NOT_ON_TIMER,    // PQ 1  ** 57 **
  NOT_ON_TIMER,    // PQ 2  ** 58 **
  NOT_ON_TIMER,    // PQ 3  ** 59 **
  NOT_ON_TIMER,    // PR 0  ** 60 **
  NOT_ON_TIMER,    // PR 1  ** 61 **
  NOT_ON_TIMER,    // PA 0  ** 62 **
  NOT_ON_TIMER,    // PA 1  ** 63 **
  NOT_ON_TIMER,    // PA 2  ** 64 **
  NOT_ON_TIMER,    // PA 3  ** 65 **
  NOT_ON_TIMER,    // PA 4  ** 66 **
  NOT_ON_TIMER,    // PA 5  ** 67 **
  NOT_ON_TIMER,    // PA 6  ** 68 **
  NOT_ON_TIMER,    // PA 7  ** 69 **
  NOT_ON_TIMER,    // PB 0  ** 70 **
  NOT_ON_TIMER,    // PB 1  ** 71 **
  NOT_ON_TIMER,    // PB 2  ** 72 **
  NOT_ON_TIMER,    // PB 3  ** 73 **
  NOT_ON_TIMER,    // PB 4  ** 74 **
  NOT_ON_TIMER,    // PB 5  ** 75 **
  NOT_ON_TIMER,    // PB 6  ** 76 **
  NOT_ON_TIMER,    // PB 7  ** 77 **
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



