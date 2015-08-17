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
  PORTR - digital 16-17 (built-in LED on PORTR pin 1, aka '17')
  PORTA - analog 0-7, digital 18-25

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

  This version of the xmega Arduino startup+support files supports HARDWARE FLOW CONTROL on BOTH serial ports via
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

  NOTE:  you can even use PORTA pins for this, if you don't need to measure analog volts on those pins

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


// NOTE:  'E' series can have analog inputs on PORTD.  It can also support ARef on PORTA pin 0, or PORTD pin 0
#define USE_AREF analogReference_PORTA0 /* see 28.16.3 in 'AU' manual - this is the REFCTRL bits for the reference select, AREF on PORTA (PA0) */

#define NUM_DIGITAL_PINS            18

#ifdef USE_AREF

#define NUM_ANALOG_INPUTS           7 /* could be 15 in an alternate version if I use PORTD as analog inputs */

// PORTA pin 0 is not a valid input, so the first bit would be '1'
#ifdef analogInPinToBit
#undef analogInPinToBit
#endif // analogInPinToBit
#define analogInPinToBit(P) (((P) + 1) & 7) /* analog pin 0 = 1 (PORTA), analog pin 7 = 0 (PORTD) */

#else // USE_AREF

#define NUM_ANALOG_INPUTS           8 /* could be 16 in an alternate version if I use PORTD as analog inputs */

#endif // USE_AREF

#define analogInputToAnalogPin(p) ((p)-A0)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + NUM_DIGITAL_PINS : -1)


// pins with PWM
// no shift:  0, 5-15
// shift:  3-6, 8-13, 16-17
#ifdef DIGITAL_IO_PIN_SHIFT
#define digitalPinHasPWM(p)         (((p) >= 3 && (p) <= 13 && (p) != 7) || (p) == 16 || (p) == 17)
#else // no digital I/O pin shift
#define digitalPinHasPWM(p)         ((p) == 0 || ((p) >= 5 && (p) <= 15))
#endif // DIGITAL_IO_PIN_SHIFT

// the first macro, 'digitalPinToInterrupt', is for the 'interruptNum' parameter in 'attachInterrupt' and 'detachInterrupt'
// the second macro, 'digitalPinToIntMode', is for the 'mode' parameter in 'attachInterrupt'.
#define digitalPinToInterrupt(p) \
  { register uint8_t uiPort = pgm_read_byte((&digital_pin_to_port_PGM[p])); \
    uiPort == _PD ? PORTD_INT0 : uiPort == _PC ? PORTC_INT0 : uiPort == _PA ? PORTA_INT0 : uiPort == _PR ? PORTR_INT0 : -1; }

#define digitalPinToIntMode(p) ((uint16_t)(pgm_read_byte(&(digital_pin_to_bit_mask_PGM[p]))) << INT_MODE_PIN_SHIFT)


// xmega-specific - Interrupt 'vector number' assignments:

// Interrupts are PORT-SPECIFIC, not pin specific.
// pin 2 on any port is always asynchronous (except for 'R' which doesn't have a pin 2)
// all other pins can manage synchronous interrupts.  'wakeup' from sleep mode
// and other async interrupts must be on a 'pin 2', on ports A through E
//
// On the 'E5' each port has only 1 interrupt vector, unlike other xmega processors.

#define PORTD_INT0  0
#define PORTC_INT0  1
#define PORTA_INT0  2
#define PORTR_INT0  3

#define EXTERNAL_NUM_INTERRUPTS 4 /* defined here instead of wiring_private.h */

// was in wiring_external.h, moved here
#define EXTERNAL_INT_0 0
#define EXTERNAL_INT_1 1
#define EXTERNAL_INT_2 2
#define EXTERNAL_INT_3 3
//#define EXTERNAL_INT_4 4
//#define EXTERNAL_INT_5 5
//#define EXTERNAL_INT_6 6
//#define EXTERNAL_INT_7 7


// xmega 'E' series has 2 sets of UART and SPI.
// The default UART is assigned on Port D, pins PD2-3
// The default SPI is assigned on Port C, pins PC4-7
//
// There is only ONE TWI on the 'E' series, on port C pins 0,1
//
// Standard GPIO pins are assigned as follows:
// PD0-7 Digital 0-7
// PC0-7 Digital 8-15
// PR0-1 digital 16-17
// PA0-7 analog A0-A7
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
// Timer TC4 should be configured as 'TC5' (for 8 PWM outputs) by default, essentially
// as a dual 8-bit timer, more or less compatible with the Arduino's 3 timers and
// supporting all 8 pins on port C for PWM output.  Port C's timer supports
// the system clock. Port D's timer has only PD5, so only pins 4-7 can be used for
// PWM output.  TD5 won't remap the pins at all to 0-4.
//
// See 'E' manual (chapter 13?) on TC4/5 and TD5 for more on this


// --------------------------------------------
// DEFINITIONS FOR SERIAL PORTS AND DEFAULT TWI
// --------------------------------------------

#define DEFAULT_TWI TWIC
#define TWIC_VECT_ENABLE /* use this to select the correct interrupt vectors */

#define DEFAULT_SPI SPIC

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


// For atmega/Arduino Uno shield compatibility, with DIGITAL_IO_PIN_SHIFT defined,
// the typical board/pin layout might be like this (for shield pins):
//
// TWI is on TWIC (port C pins 0/1).  pins marked as '~' have PWM
//
//               M M
//             S I O   T R
//         A   C S S S x x
//     S S R G K O I S 2 2              T R
//     C D E N 1 1 1 1                  x x
//     L A F D 3 2 1 0 9 8  7 6 5 4 3 2 1 0
// ----o-o-o-o-o-o-o-o-o-o--o-o-o-o-o-o-o-o----
//     P P P   P P P P P P  P P P P P P P P
//     C C A   C C C C C C  D D D D D D D D
//     1 0 0   5 6 7 4 3 2  1 7 6 5 4 0 3 2
//     ~ ~     ~ ~ ~ ~ ~ ~    ~ ~ ~ ~
//
//               T O P   V I E W
//
//                           p
//               3   3       w  P P P P P P
//               V   V 5     r  A A A A A A
//               3   3 V     +  6 5 4 3 2 1
// ------------o-o-o-o-o-o-o-o--o-o-o-o-o-o----
//             U I R 3 5 G G V  A A A A A A
//             N O E . V N N i  5 4 3 2 1 0
//             U R S 3   D D n
//             S E E V
//             E F T
//             D
//
// RESERVED PINS (not brought out):  PA7, PR0, PR1 (connected to LED_BUILTIN)
//
// ARef should have a 100 ohm (or similar) resistor to Vcc with a decoupling capacitor
// IOREF must be connected to 3.3V so that compatible shields can detect 3.3V logic
// Vin connects to the 'power in' pin (may have 9V or more on this pin)
// 5V can be left unconnected, or use a separate voltage reg for 5V
// 3.3v must be regulated and able to supply 1A or more for shields.
//
// NOTE:  PA0 is connected to 'AREF', and PORTA pins are shifted to PA1 through PA6 for
//        A0 through A5.
//
// As with the MEGA2560 and other 'mega' Arduino boards, additional connectors would
// break out the additional pins, with appropriate labeling.  Additionally, there should
// be an LED on PORTR pin 1 for 'LED_BUILTIN'.
//
// This layout is based on the 'Rev 3' Arduino.  Earlier Arduino didn't have IOREF, SDA, SCL.
//
// Uno has automatic switching between 5V reg out and 5V from USB power.  When Vin is more
// than 6.6v, the Vin source supplies 5V (and 3.3V) to the board.  otherwise, the USB
// 5V power supplies 5V (and 3.3V) to the board.
//
// NOTE:  on the E5, PC7 is MOSI, PC5 is SCK (it's the other way on the other CPUs)
//
// Summary Map:  D0   D1   D2   D3   D4   D5   D6   D7   D8   D9   D10  D11  D12  D13
//               PD2  PD3  PD0  PD4  PD5  PD6  PD7  PD1  PC2  PC3  PC4  PC7  PC6  PC5
//               Rx   Tx                                 Rx2  Tx2  SS   MOSI MISO SCK
//
//               A0   A1   A2   A3   A4   A5
//               PA1  PA2  PA3  PA4  PA5  PA6
//
// TWIC:  PC0 (SDA), PC1 (SCL)
// AREF:  PA0 (~10k to Vcc)
//
// ASYNC INTERRUPTS:  A1, D0, D8 (can be used for 'wakeup')
//

#ifdef DIGITAL_IO_PIN_SHIFT // aka digital I/O pin 0 is PORTD pin 2, as diagrammed above

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

//// secondary SPI on PD4-7 (not present on E series)
//static const uint8_t SS1   = 2;
//static const uint8_t MOSI1 = 3;
//static const uint8_t MISO1 = 4;
//static const uint8_t SCK1  = 5;

// default 2-wire on PC0,PC1 - TWIC
static const uint8_t SDA = 16;
static const uint8_t SCL = 17;

// keep track of the indices for port R since its control register
// settings should be slightly different - D manual table 11-6 etc.
#define PR0 14
#define PR1 15

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

//// secondary SPI on PD4-7 (not present on E series)
//static const uint8_t SS1   = 4;
//static const uint8_t MOSI1 = 5;
//static const uint8_t MISO1 = 6;
//static const uint8_t SCK1  = 7;

// default 2-wire on PC0,PC1 - TWIC
static const uint8_t SDA = 8;
static const uint8_t SCL = 9;

// keep track of the indices for port R since its control register
// settings should be slightly different - D manual table 11-6 etc.
#define PR0 16
#define PR1 17

#endif // DIGITAL_IO_PIN_SHIFT


// default 'status' LED on PR1
//static const uint8_t LED_BUILTIN = PR1;
#define LED_BUILTIN PR1 /* Arduino 1.06 uses #define, not a const uint8_t */

static const uint8_t A0 = 18;
static const uint8_t A1 = 19;
static const uint8_t A2 = 20;
static const uint8_t A3 = 21;
static const uint8_t A4 = 22;
static const uint8_t A5 = 23;
static const uint8_t A6 = 24;
static const uint8_t A7 = 25;

// on the xmega 'E' series, PA2, PC2, and PD2 are asynchronous ints.  Others are 'synchronous' which means
// that they must be held in their 'interrupt state' long enough for the system to detect them.  In any case
// all digital input pins can be use as interrupts, synchronous or otherwise.


#ifdef ARDUINO_MAIN

// TODO:  modify 'core' code to have different indices for E series?  For now it's ok as-is

const uint16_t PROGMEM port_to_mode_PGM[] = {
  NOT_A_PORT,                  // 0
  (uint16_t) &PORTA_DIR,       // PA
  NOT_A_PORT,                  // 2  [E series has no 'port B'
  (uint16_t) &PORTC_DIR,       // PC
  (uint16_t) &PORTD_DIR,       // PD
  NOT_A_PORT,                  // 5  [E series has no 'port E'
  (uint16_t) &PORTR_DIR,       // PR
};

const uint16_t PROGMEM port_to_output_PGM[] = {
  NOT_A_PORT,                  // 0
  (uint16_t) &PORTA_OUT,       // PA
  NOT_A_PORT,                  // 2  [E series has no 'port B'
  (uint16_t) &PORTC_OUT,       // PC
  (uint16_t) &PORTD_OUT,       // PD
  NOT_A_PORT,                  // 5  [E series has no 'port E'
  (uint16_t) &PORTR_OUT,       // PR
};

const uint16_t PROGMEM port_to_input_PGM[] = {
  NOT_A_PORT,                  // 0
  (uint16_t) &PORTA_IN,        // PA
  NOT_A_PORT,                  // 2  [E series has no 'port B'
  (uint16_t) &PORTC_IN,        // PC
  (uint16_t) &PORTD_IN,        // PD
  NOT_A_PORT,                  // 5  [E series has no 'port E'
  (uint16_t) &PORTR_IN,        // PR
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
// NOTE:  PORTA through PORTE (PORTF?) support 'input buffer disabled' and this setting is recommended
//        for analog inputs.  PORTR apparently does NOT support this (set to zero?)

const uint16_t PROGMEM digital_pin_to_control_PGM[] = {
#ifndef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN0CTRL,  // PD 0 ** 0 **
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  (uint16_t) &PORTD_PIN2CTRL,  // PD 2 ** 2 ** USARTD_RX     ASYNC
  (uint16_t) &PORTD_PIN3CTRL,  // PD 3 ** 3 ** USARTD_TX
#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN0CTRL,  // PD 0 ** 8 ** 
#endif // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN4CTRL,  // PD 4 ** 4 **
  (uint16_t) &PORTD_PIN5CTRL,  // PD 5 ** 5 **
  (uint16_t) &PORTD_PIN6CTRL,  // PD 6 ** 6 **
  (uint16_t) &PORTD_PIN7CTRL,  // PD 7 ** 7 **
#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1 ** 9 ** PC 0,1 must be used for TWI
#else // no pin shifting
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0 ** 8 ** SDA
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1 ** 9 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN2CTRL,  // PC 2 ** 10 **              ASYNC
  (uint16_t) &PORTC_PIN3CTRL,  // PC 3 ** 11 **
  (uint16_t) &PORTC_PIN4CTRL,  // PC 4 ** 12 ** SPI_SS

#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN7CTRL,  // PC 7 ** 15 ** SPI_MOSI
#else // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN5CTRL,  // PC 5 ** 13 ** SPI_SCK
#endif // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN6CTRL,  // PC 6 ** 14 ** SPI_MISO
#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN5CTRL,  // PC 5 ** 13 ** SPI_SCK
#else // DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN7CTRL,  // PC 7 ** 15 ** SPI_MOSI
#endif // DIGITAL_IO_PIN_SHIFT

  (uint16_t) &PORTR_PIN0CTRL,  // PR 0 ** 16 **
  (uint16_t) &PORTR_PIN1CTRL,  // PR 1 ** 17 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0 ** the new 16 ** SDA
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1 ** the new 17 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
#ifndef USE_AREF
  (uint16_t) &PORTA_PIN0CTRL,  // PA 0 ** 18 ** A0           AREF (when USE_AREF is defined, and then it won't be mapped)
#endif // USE_AREF
  (uint16_t) &PORTA_PIN1CTRL,  // PA 1 ** 19 ** A1
  (uint16_t) &PORTA_PIN2CTRL,  // PA 2 ** 20 ** A2           ASYNC
  (uint16_t) &PORTA_PIN3CTRL,  // PA 3 ** 21 ** A3
  (uint16_t) &PORTA_PIN4CTRL,  // PA 4 ** 22 ** A4
  (uint16_t) &PORTA_PIN5CTRL,  // PA 5 ** 23 ** A5
  (uint16_t) &PORTA_PIN6CTRL,  // PA 6 ** 24 ** A6
  (uint16_t) &PORTA_PIN7CTRL,  // PA 7 ** 25 ** A7
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  // PORTLIST
  // -------------------------------------------
#ifndef DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 0 ** 0 **
  _PD,  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  _PD,  // PD 2 ** 2 ** USARTD_RX
  _PD,  // PD 3 ** 3 ** USARTD_TX
#ifdef DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 0 ** 2 **
#endif // DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 4 ** 4 **
  _PD,  // PD 5 ** 5 **
  _PD,  // PD 6 ** 6 **
  _PD,  // PD 7 ** 7 **
#ifdef DIGITAL_IO_PIN_SHIFT
  _PD,  // PD 1 ** 9 **
#else // no pin shifting
  _PC,  // PC 0 ** 8 ** SDA
  _PC,  // PC 1 ** 9 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
  _PC,  // PC 2 ** 10 **
  _PC,  // PC 3 ** 11 **
  _PC,  // PC 4 ** 12 ** SPI_SS
// NOTE port doesn't change with DIGITAL_IO_PIN_SHIFT
  _PC,  // PC 5 ** 13 ** SPI_SCK
  _PC,  // PC 6 ** 14 ** SPI_MISO
  _PC,  // PC 7 ** 15 ** SPI_MOSI
  _PR,  // PR 0 ** 16 **
  _PR,  // PR 1 ** 17 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  _PC,  // PC 0 ** the new 16 ** SDA
  _PC,  // PC 1 ** the new 17 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
#ifndef USE_AREF
  _PA,  // PA 0 ** 18 ** A0
#endif // USE_AREF
  _PA,  // PA 1 ** 19 ** A1
  _PA,  // PA 2 ** 20 ** A2
  _PA,  // PA 3 ** 21 ** A3
  _PA,  // PA 4 ** 22 ** A4
  _PA,  // PA 5 ** 23 ** A5
  _PA,  // PA 6 ** 24 ** A6
  _PA,  // PA 7 ** 25 ** A7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  // PIN IN PORT
  // -------------------------------------------
#ifndef DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PD 0 ** 0 **
  _BV( 1 ),  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  _BV( 2 ),  // PD 2 ** 2 ** USARTD_RX
  _BV( 3 ),  // PD 3 ** 3 ** USARTD_TX
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PD 0 ** 2 **
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 4 ),  // PD 4 ** 4 **
  _BV( 5 ),  // PD 5 ** 5 **
  _BV( 6 ),  // PD 6 ** 6 **
  _BV( 7 ),  // PD 7 ** 7 **
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 1 ),  // PD 1 ** 9 **
#else // no pin shifting
  _BV( 0 ),  // PC 0 ** 8 ** SDA
  _BV( 1 ),  // PC 1 ** 9 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 2 ),  // PC 2 ** 10 **
  _BV( 3 ),  // PC 3 ** 11 **
  _BV( 4 ),  // PC 4 ** 12 ** SPI_SS
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 7 ),  // PC 7 ** 15 ** SPI_MOSI
#else // no pin shifting
  _BV( 5 ),  // PC 5 ** 13 ** SPI_SCK
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 6 ),  // PC 6 ** 14 ** SPI_MISO
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 5 ),  // PC 5 ** 13 ** SPI_SCK
#else // no pin shifting
  _BV( 7 ),  // PC 7 ** 15 ** SPI_MOSI
#endif // DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PR 0 ** 16 **
  _BV( 1 ),  // PR 1 ** 17 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  _BV( 0 ),  // PC 0 ** the new 16 ** SDA
  _BV( 1 ),  // PC 1 ** the new 17 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
#ifndef USE_AREF
  _BV( 0 ),  // PA 0 ** 18 ** A0
#endif // USE_AREF
  _BV( 1 ),  // PA 1 ** 19 ** A1
  _BV( 2 ),  // PA 2 ** 20 ** A2
  _BV( 3 ),  // PA 3 ** 21 ** A3
  _BV( 4 ),  // PA 4 ** 22 ** A4
  _BV( 5 ),  // PA 5 ** 23 ** A5
  _BV( 6 ),  // PA 6 ** 24 ** A6
  _BV( 7 ),  // PA 7 ** 25 ** A7
};




const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  // TIMERS
  // -------------------------------------------
  // The timers on the E5 are a bit different than the others
  // Since TIMERD5 only goes tp PD4, PD5, PD6, PD7, I have to
  // map them to places that have PWM.  Also, PD6 and PD7 don't
  // seem to work very well.  PC0 through PC3 work pretty well,
  // but PC0 and PC1 are TWI pins.  As such, the mapping won't
  // work very well if Arduino compatibility is desired.

#ifndef DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  // PD 0 ** 0 **
  NOT_ON_TIMER,  // PD 1 ** 1 **
#endif // DIGITAL_IO_PIN_SHIFT
// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  NOT_ON_TIMER,  // PD 2 ** 2 ** USARTD_RX
  NOT_ON_TIMER,  // PD 3 ** 3 ** USARTD_TX
#ifdef DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  // PD 0 ** 2 ** non-PWM 2
#endif // DIGITAL_IO_PIN_SHIFT
  TIMERD5,       // PD 4 ** 4 ** PWM 3
  TIMERD5,       // PD 5 ** 5 ** PWM 4
  NOT_ON_TIMER,  //TIMERD5,       // PD 6 ** 6 ** PWM 5
  NOT_ON_TIMER,  //TIMERD5,       // PD 7 ** 7 ** PWM 6
#ifdef DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  // PD 1 ** 9 **
#else // no pin shifting
  TIMERC4,       // PC 0 ** 8 ** SDA
  TIMERC4,       // PC 1 ** 9 ** SCL
#endif // DIGITAL_IO_PIN_SHIFT
  TIMERC4,       // PC 2 ** 10 **
  TIMERC4,       // PC 3 ** 11 **
  NOT_ON_TIMER,  //TIMERC4,       // PC 4 ** 12 ** SPI_SS
// NOTE:  timer doesn't change with DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  //TIMERC4,       // PC 5 ** 13 ** SPI_SCK
  NOT_ON_TIMER,  //TIMERC4,       // PC 6 ** 14 ** SPI_MISO
  NOT_ON_TIMER,  //TIMERC4,       // PC 7 ** 15 ** SPI_MOSI
// NOTE:  'not on timer' doesn't change with DIGITAL_IO_PIN_SHIFT
  NOT_ON_TIMER,  // PR 0 ** 16 **
  NOT_ON_TIMER,  // PR 1 ** 17 ** default LED
#ifdef DIGITAL_IO_PIN_SHIFT
  TIMERC4,       // PC 0 ** the new 16 **
  TIMERC4,       // PC 1 ** the new 17 **
#endif // DIGITAL_IO_PIN_SHIFT

#ifndef USE_AREF
  NOT_ON_TIMER,  // PA 0 ** 18 ** A0
#endif // USE_AREF
  NOT_ON_TIMER,  // PA 1 ** 19 ** A1
  NOT_ON_TIMER,  // PA 2 ** 20 ** A2
  NOT_ON_TIMER,  // PA 3 ** 21 ** A3
  NOT_ON_TIMER,  // PA 4 ** 22 ** A4
  NOT_ON_TIMER,  // PA 5 ** 23 ** A5
  NOT_ON_TIMER,  // PA 6 ** 24 ** A6
  NOT_ON_TIMER,  // PA 7 ** 25 ** A7
};

#endif


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

#endif


