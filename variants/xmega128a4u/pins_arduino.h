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


  SPI is assumed to be on PORTC (pins 4-7)
  USB is assumed to be on PORTD (pins 6-7) (this eliminates using SPI on port D)
  Serial1 is implemented on PORTD (USARTD0), Serial2 and Serial3 on PORTC
  Serial1 and Serial2 use pins 2,3; Serial3 uses pins 6,7.  No flow control is assigned
  PORTR pins 0 and 1 are the RX and TX LEDs.  Pin D13 (PC7) is the 'built-in' LED, defined
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

  NOTE:  you can even use PORTA or PORTB pins for this, if you don't need to measure analog volts on those pins

  X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A   X M E G A

*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// ---------------------------
// DEFAULT 2-WIRE PORT IS TWIC
// ---------------------------
#define USE_TWIC /* define this to re-map TWIC to digital pins 20 and 21, similar to an Arduino Mega2560.  requires DIGITAL_IO_PIN_SHIFT */

#if !defined(USE_TWIC) && !defined (USE_TWIE) /* if neither USE_TWIC nor USE_TWIE defined */
#define USE_TWIE /* if neither is defined, as in I comment the above line, define 'USE_TWIE' */
#endif // !defined(USE_TWIC) && !defined (USE_TWIE)

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           12
#define analogInputToDigitalPin(p)  ((p < 12) ? (p) + 22 : -1)
#define digitalPinHasPWM(p)         ((p) < 16 || (p) == 18 || (p) == 19) /* PORTC pins 0 and 1 are 20 and 21, respectively */


// this returns the DEFAULT INTERRUPT (in this case, interrupt 0) for any digital or analog pin
// If you choose a port's pin 2, it will be the same as using 'PORTn_INT0'
#define digitalPinToInterrupt(p) \
  ( pgm_read_byte(&port_to_int0_PGM[pgm_read_byte(&digital_pin_to_port_PGM[p])]) | \
    ( ((pgm_read_byte(&digital_pin_to_bit_mask_PGM[p]) - 2) & 7) << 5 ) )



//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            _   _  ____   ____                            //
//                           | | | |/ ___| | __ )                           //
//                           | | | |\___ \ |  _ \                           //
//                           | |_| | ___) || |_) |                          //
//                            \___/ |____/ |____/                           //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// USB support code and definitions go here.  This affects a lot of things  //
// This file is otherwise identical to the non-u-suffix version.            //
//                                                                          //
// The USB Vendor and Product ID values are for the Arduino Mega 2560       //
//     DO! NOT! USE! THEM! OUTSIDE! OF! PERSONAL! EXPERIMENTATION!          //
// In particular, do NOT distribute ANY products or firmware using them!    //
// The ONLY reason they are here is so that your device will actually WORK  //
// when you plug it in (assuming you have a driver for the Mega 2560)       //
//                                                                          //
// If you need a FREE USB Vendor/Product ID set, you can visit THIS site:   //
//   http://wiki.openmoko.org/wiki/USB_Product_IDs                          //
//                                                                          //
// Or you can use one of the IDs specified here (restrictions apply):       //
//   https://github.com/arduino/ArduinoISP/                                 //
//     file: usbdrv/USB-IDs-for-free.txt                                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define USBCON /* says I have a USB controller, modifies startup code and default serial port accordingly */

// if there are no boards.txt entries for the USB VID or PID, they'll be defined as 'null'
#ifdef USB_VID
#if USB_VID==null
#undef USB_VID
#endif // USB_VID==null
#endif // USB_VID

#ifdef USB_PID
#if USB_PID==null
#undef USB_PID
#endif // USB_PID==null
#endif // USB_PID

#ifndef USB_VID
// NOTE:  you can eliminate the warnings by making a copy of this for your own variant
//        but REMEMBER! YOU! MUST! CHANGE! THE! IDS!

#warning using Arduino Mega2560 USB Vendor ID - do NOT ship product with these IDs
#define USB_VID 0x2341 /* this is the Arduino vendor ID - you should probably get your own */
#warning using Arduino Mega2560 USB Vendor ID - do NOT ship product with these IDs
#endif // USB_VID

#ifndef USB_PID
// NOTE:  you can eliminate the warnings by making a copy of this for your own variant
//        but REMEMBER! YOU! MUST! CHANGE! THE! IDS!

#warning using Arduino Mega2560 USB Product ID - do NOT ship product with these IDs
#define USB_PID 0x0010 /* this is the Arduino Mega2560 R3 product ID - you should probably use your own */
#warning using Arduino Mega2560 USB Product ID - do NOT ship product with these IDs
#endif // USB_PID

#define CDC_ENABLED    /* this switches to device class 2, useful for serial I/O implementations */



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

#define EXTERNAL_NUM_INTERRUPTS 12 /* defined here instead of wiring_private.h - max value is 32 */

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

// xmega 'A4' series has 4 sets of UART, and 2 sets of SPI.
// The 'A4U' series has USB on Port D pins PD6-7 (so USARTD1 cannot be used)
// The default UART is assigned on Port D, pins PD2-3
// The default SPI is assigned on Port C, pins PC4-7
//
// Also there are multiple 2-wire ports, the default being assigned to PC0-1
// see definition for DEFAULT_TWI and USE_TWIC
//
// GPIO pins are assigned as follows:
// PD2-5 Digital 0-3
// PE2-3 Digital 4-5 (this provides an async interrupt on 4)
// PD0-1 Digital 6-7
// PC2-7 Digital 8-13 (so SPI shows up on 10-13)
// PE0-1 digital 14-15 (for TWIE)
// PR0-1 digital 16-17 (RX LED on 16, TX LED on 17)
// PC0-1 digital 18-19 (for TWIC)
// PD6-7 USB D-, D+
// PA0-7 analog A0-A7
// PB0-3 analog A8-A11
//
// These assignments are primarily for 'Arduino Uno' pin mapping compatibility, to
// limit the things that need to be re-assigned for an xmega.
//
// '#define'ing USE_TWIC puts PC0-1 on 18-19 (corresponding to TWI pins on later Arduinos)
// '#define'ing USE_TWIE puts PE0-1 on 18-19 (and PC0-1 on 14-15)
//
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
// See 'AU' manual chapter 13 (I/O ports) for more on this


// ------------------------------------------
// DEFINITIONS FOR SERIAL PORTS AND TWI PORTS
// ------------------------------------------


// TWI ports
#define DEFAULT_TWI TWIC /* note see definitions for SDA and SCL, below - alter accordingly */

// the XMega64D4 has two TWI ports
#define TWI_PORT0 TWIC
#define TWI_VECTOR_S0 TWIC_TWIS_vect
#define TWI_VECTOR_M0 TWIC_TWIM_vect
#define TWI_PORT1 TWIE
#define TWI_VECTOR_S1 TWIE_TWIS_vect
#define TWI_VECTOR_M1 TWIE_TWIM_vect

#define TWI_INTERFACE_COUNT 2


// obsolete - consider removal in all of them
//#define TWIC_VECT_ENABLE /* use this to select the correct interrupt vectors for default */


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
#define SERIAL_1_RX_PIN_INDEX 2 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_1_TX_PIN_INDEX 3 /* the pin number on the port, not the mapped digital pin number */
#define USARTC0_VECTOR_EXISTS

// serial port 2
#define SERIAL_2_PORT_NAME PORTC
#define SERIAL_2_USART_NAME USARTC1
#define SERIAL_2_USART_DATA USARTC1_DATA
#define SERIAL_2_RXC_ISR ISR(USARTC1_RXC_vect)
#define SERIAL_2_DRE_ISR ISR(USARTC1_DRE_vect)
#define SERIAL_2_RX_PIN_INDEX 6 /* the pin number on the port, not the mapped digital pin number */
#define SERIAL_2_TX_PIN_INDEX 7 /* the pin number on the port, not the mapped digital pin number */
#define USARTC1_VECTOR_EXISTS


// For atmega/Arduino shield compatibility, a typical board/pin layout might be as follows:
//
// NOTE:  this design/layout assumes USE_TWIC is defined, so TWI is on TWIC (port C pins 0/1)
//        and the 'Rev 3' pins for SDA/SCL are connected to PC0,1 respectively
//
//
//               M M
//             S I O   T R
//         A   C S S S x x
//     S S R G K O I S 2 2              T R
//     C D E N 1 1 1 1                  x x
//     L A F D 3 2 1 0 9 8  7 6 5 4 3 2 1 0
// ----o-o-o-o-o-o-o-o-o-o--o-o-o-o-o-o-o-o----
//     P P P   P P P P P P  P P P P P P P P
//     C C A   C C C C C C  D D E E D D D D
//     1 0 0   7 6 5 4 3 2  1 0 3 2 5 4 3 2
//
//
//               T O P   V I E W
//
//
//               3   3       V  P P P P P P
//               V   V 5     i  A A A A A A
//               3   3 V     n  6 5 4 3 2 1
// ------------o-o-o-o-o-o-o-o--o-o-o-o-o-o----
//             G I R 3 5 G G V  A A A A A A
//             N O E . V N N i  5 4 3 2 1 0
//             D R S 3   D D n
//               E E V
//               F T
//
// As with the MEGA2560 and other 'mega' Arduino boards, additional connectors would
// break out the additional pins, with appropriate labeling.
// There should be 3 LED's:  RX on PR0, TX on PR1, and BUILTIN on PC7 (D13)
//
// 'USB-' is on PD6, and 'USB+' on PD7. Each should have a series 22 ohm resistor.
// TWIE is on PE0 and PE1, and should be broken out separately.
// AREF should be internally connected to Vcc (3.3v) through a 2.2k resistor.
//
//
//             ARDUINO REV 3 SHIELD LAYOUT and 3.3v SHIELD COMPATIBILITY
//             ---------------------------------------------------------
// This layout is based on the 'Rev 3' Arduino, with extra pins for IOREF and SDA/SCL.
// Note that SDA2/SCL2 is on PE0,1 respectively.  Both are exposed on pins NOT present
// on earlier Arduino shields.  But then again, earlier shields didn't have IOREF which
// is needed for 3.3v/5v shield compatibility.  So if you have a 3.3v compatible shield
// with the IOREF pin and extra pins for SDA,SCL you should be good to go.  But earlier
// shields may not work properly.
//
//                      COMPATIBLE ANALOG INPUTS ON PORT A ONLY
//                      ---------------------------------------
// Analog inputs use PA1 through PA6.  Only port A can use diff with gain against
// Vcc/2 as a reference; however, this is the default for compatibility reasons.  The
// 'AREF' pin is connected to PA0.  It can be selected using 'analogReference()'.
//
// PB0-3 can still be exposed separately, but they must use AREF (they won't work with
// the same 'hack' used for A1-7 using diff input with gain on internal Vcc/2).
//
// NOTE:  it is possible to use PB0 as an AREF. For now, PA0 is connected to AREF.
//
// NOTE -  AREF is a bit of an issue on xmega because it DOES! NOT! WORK! THE! SAME!
// as it does on the ATmegaXXX and so you need to (literally) steal one of the
// additional analog input pins to implement it, and for things to work right, it
// needs to be either PA0 or PB0 (because only they can be AREF).  The D4 processor
// handles these a bit better, but D4 doesn't have a USB...
//



// I/O pins (atmega compatibility) - see xmega mod description in comment at top

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

// default 2-wire on PC0,PC1 - TWIC [note if I use TWIE, it's shifted around to the same pins]
static const uint8_t SDA = 18;
static const uint8_t SCL = 19;

// PD2-5 Digital 0-3
// PE2-3 Digital 4-5 (this provides an async interrupt on 4)
// PD0-1 Digital 6-7
// PC2-7 Digital 8-13 (so SPI shows up on 10-13; LED_BUILTIN also on 13)
// PE0-1 digital 14-15 (for TWIE)
// PR0-1 digital 16-17 (RX LED on 16, TX LED on 17)
// PC0-1 digital 18-19 (for TWIC)
// PD6-7 USB D-, D+ (in that order)
// PA0   AREF (connect with a 2.2k resistor to Vcc)
// PA1-7 analog A0-A6
// PB0-3 analog A7-A10


// port-specific 2-wire
#ifdef USE_TWIC
static const uint8_t SDA0 = 18;
static const uint8_t SCL0 = 19;
static const uint8_t SDA1 = 14;
static const uint8_t SCL1 = 15;
#else // !USE_TWIC, assume TWIE
static const uint8_t SDA0 = 14;
static const uint8_t SCL0 = 15;
static const uint8_t SDA1 = 18;
static const uint8_t SCL1 = 19;
#endif // USE_TWIC


// keep track of the indices for port R since its control register
// settings should be slightly different - D manual table 11-6
#define PR0 16 /* RX LED */
#define PR1 17 /* TX LED */


// default 'status' LED on PR1
//static const uint8_t LED_BUILTIN = 13;
#define LED_BUILTIN 13 /* Arduino >=1.06 uses #define, not a const uint8_t */

// NOTE:  TX LED is PR1, RX LED is PR0 - these next definitions are for USBCore.cpp
#define TX_RX_LED_INIT() { pinMode(PR0,OUTPUT); pinMode(PR1,OUTPUT); \
                           digitalWrite(PR0,0); digitalWrite(PR1,0); }
#define TXLED0() digitalWrite(PR1,0)
#define TXLED1() digitalWrite(PR1,1)
#define RXLED0() digitalWrite(PR0,0)
#define RXLED1() digitalWrite(PR0,1)


static const uint8_t A0 = 20;
static const uint8_t A1 = 21;
static const uint8_t A2 = 22;
static const uint8_t A3 = 23;
static const uint8_t A4 = 24;
static const uint8_t A5 = 25;
static const uint8_t A6 = 26;
static const uint8_t A7 = 27;
static const uint8_t A8 = 28;
static const uint8_t A9 = 29;
static const uint8_t A10 = 30;
static const uint8_t A11 = 31; // note that this is PA0, also AREF


// DIGITAL TO ANALOG CONVERTER DACA and DACB
// DACA Output on pins A1 (CH0) and A2 (CH1), DACB on A8 (CH0) and A9 (CH1)

#define DACA_CH0_PIN A1  /* check for #define to determine DACA, DACB support */
#define DACA_CH1_PIN A2
#define DACB_CH0_PIN A8
#define DACB_CH1_PIN A9



// on the xmega128a4u, PA2, PB2, PC2, PD2, and PE2 are asynchronous ints.  Others are 'synchronous' which means
// that they must be held in their 'interrupt state' long enough for the system to detect them.  In any case
// all digital input pins can be use as interrupts, synchronous or otherwise.



#ifdef ARDUINO_MAIN /* I only have memory assignments for these when 'ARDUINO_MAIN' is defined */


const uint16_t PROGMEM port_to_mode_PGM[] =
{
  NOT_A_PORT,             // 0
  (uint16_t) &PORTA_DIR,       // PA
  (uint16_t) &PORTB_DIR,       // PB
  (uint16_t) &PORTC_DIR,       // PC
  (uint16_t) &PORTD_DIR,       // PD
  (uint16_t) &PORTE_DIR,       // PE
  (uint16_t) &PORTR_DIR,       // PR
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
  NOT_A_PORT,              // 0
  (uint16_t) &PORTA_OUT,       // PA
  (uint16_t) &PORTB_OUT,       // PB
  (uint16_t) &PORTC_OUT,       // PC
  (uint16_t) &PORTD_OUT,       // PD
  (uint16_t) &PORTE_OUT,       // PE
  (uint16_t) &PORTR_OUT,       // PR
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
  NOT_A_PORT,             // 0
  (uint16_t) &PORTA_IN,       // PA
  (uint16_t) &PORTB_IN,       // PB
  (uint16_t) &PORTC_IN,       // PC
  (uint16_t) &PORTD_IN,       // PD
  (uint16_t) &PORTE_IN,       // PE
  (uint16_t) &PORTR_IN,       // PR
};

const uint8_t PROGMEM port_to_int0_PGM[] =
{
  NOT_AN_INTERRUPT,           // 0
  PORTA_INT0,                 // PA
  PORTB_INT0,                 // PB
  PORTC_INT0,                 // PC
  PORTD_INT0,                 // PD
  PORTE_INT0,                 // PE
  PORTR_INT0,                 // PR
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

const uint16_t PROGMEM digital_pin_to_control_PGM[] =
{
  (uint16_t) &PORTD_PIN2CTRL,  // PD 2 ** 0 **  USARTD0_RX   ASYNC
  (uint16_t) &PORTD_PIN3CTRL,  // PD 3 ** 1 **  USARTD0_TX
  (uint16_t) &PORTD_PIN4CTRL,  // PD 4 ** 2 **
  (uint16_t) &PORTD_PIN5CTRL,  // PD 5 ** 3 **

  (uint16_t) &PORTE_PIN2CTRL,  // PE 2 ** 4 **               ASYNC
  (uint16_t) &PORTE_PIN3CTRL,  // PE 3 ** 5 **

  (uint16_t) &PORTD_PIN0CTRL,  // PD 0 ** 6 **
  (uint16_t) &PORTD_PIN1CTRL,  // PD 1 ** 7 **

  (uint16_t) &PORTC_PIN2CTRL,  // PC 2 ** 8 **  USARTC0_RX,  ASYNC
  (uint16_t) &PORTC_PIN3CTRL,  // PC 3 ** 9 **  USARTC0_TX
  (uint16_t) &PORTC_PIN4CTRL,  // PC 4 ** 10 ** SPI_SS
  (uint16_t) &PORTC_PIN5CTRL,  // PC 5 ** 11 ** SPI_MOSI
  (uint16_t) &PORTC_PIN6CTRL,  // PC 6 ** 12 ** SPI_MISO, USARTC1_RX
  (uint16_t) &PORTC_PIN7CTRL,  // PC 7 ** 13 ** SPI_SCK, USARTC1_TX

#ifdef USE_TWIC
  (uint16_t) &PORTE_PIN0CTRL,  // PE 0 ** 14 ** SDA1
  (uint16_t) &PORTE_PIN1CTRL,  // PE 1 ** 15 ** SCL1
#else
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0 ** 14 ** SDA0
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1 ** 15 ** SCL0
#endif // USE_TWIC

  (uint16_t) &PORTR_PIN0CTRL,  // PR 0 ** 16 **
  (uint16_t) &PORTR_PIN1CTRL,  // PR 1 ** 17 ** default LED

#ifdef USE_TWIC
  (uint16_t) &PORTC_PIN0CTRL,  // PC 0 ** 18 ** SDA, SDA0
  (uint16_t) &PORTC_PIN1CTRL,  // PC 1 ** 19 ** SCL, SCL0
#else
  (uint16_t) &PORTE_PIN0CTRL,  // PE 0 ** 18 ** SDA, SDA1
  (uint16_t) &PORTE_PIN1CTRL,  // PE 1 ** 19 ** SCL, SCL1
#endif // USE_TWIC

  (uint16_t) &PORTA_PIN1CTRL,  // PA 1 ** 20 ** A0
  (uint16_t) &PORTA_PIN2CTRL,  // PA 2 ** 21 ** A1
  (uint16_t) &PORTA_PIN3CTRL,  // PA 3 ** 22 ** A2           ASYNC
  (uint16_t) &PORTA_PIN4CTRL,  // PA 4 ** 23 ** A3
  (uint16_t) &PORTA_PIN5CTRL,  // PA 5 ** 24 ** A4
  (uint16_t) &PORTA_PIN6CTRL,  // PA 6 ** 25 ** A5
  (uint16_t) &PORTA_PIN7CTRL,  // PA 7 ** 26 ** A6
  (uint16_t) &PORTB_PIN0CTRL,  // PB 0 ** 27 ** A7
  (uint16_t) &PORTB_PIN1CTRL,  // PB 1 ** 28 ** A8
  (uint16_t) &PORTB_PIN2CTRL,  // PB 2 ** 29 ** A9
  (uint16_t) &PORTB_PIN3CTRL,  // PB 3 ** 30 ** A10         ASYNC
  (uint16_t) &PORTA_PIN0CTRL,  // PA 0 ** 31 ** A11         AREF
};

// PD2-5 Digital 0-3
// PE2-3 Digital 4-5 (this provides an async interrupt on 4)
// PD0-1 Digital 6-7
// PC2-7 Digital 8-13 (so SPI shows up on 10-13)
// PE0-1 digital 14-15 (for TWIE)
// PR0-1 digital 16-17 (LED on 17)
// PC0-1 digital 18-19 (for TWIC)
// PD6-7 USB D-, D+
// PA0-7 analog A0-A7
// PB0-3 analog A8-A11

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  // PORTLIST
  // -------------------------------------------
  _PD,  // PD 2 ** 0 **  USARTD_RX
  _PD,  // PD 3 ** 1 **  USARTD_TX
  _PD,  // PD 4 ** 2 **
  _PD,  // PD 5 ** 3 **

  _PE,  // PE 2 ** 4 **
  _PE,  // PE 3 ** 5 **

  _PD,  // PD 0 ** 6 **
  _PD,  // PD 1 ** 7 **

  _PC,  // PC 2 ** 8 **  USARTC0_RX
  _PC,  // PC 3 ** 9 **  USARTC0_TX
  _PC,  // PC 4 ** 10 ** SPI_SS
  _PC,  // PC 5 ** 11 ** SPI_MOSI
  _PC,  // PC 6 ** 12 ** SPI_MISO, USARTC1_RX
  _PC,  // PC 7 ** 13 ** SPI_SCK, USARTC1_TX

#ifdef USE_TWIC
  _PE,  // PE 0 ** 14 ** SDA1
  _PE,  // PE 1 ** 15 ** SCL1
#else
  _PC,  // PC 0 ** 14 ** SDA0
  _PC,  // PC 1 ** 15 ** SCL0
#endif // USE_TWIC

  _PR,  // PR 0 ** 16 **
  _PR,  // PR 1 ** 17 ** default LED

#ifdef USE_TWIC
  _PC,  // PC 0 ** 18 ** SDA, SDA0
  _PC,  // PC 1 ** 19 ** SCL, SCL0
#else
  _PD,  // PE 0 ** 18 ** SDA, SDA1
  _PD,  // PE 1 ** 19 ** SCL, SCL1
#endif // USE_TWIC

  _PA,  // PA 1 ** 20 ** A0
  _PA,  // PA 2 ** 21 ** A1
  _PA,  // PA 3 ** 22 ** A2
  _PA,  // PA 4 ** 23 ** A3
  _PA,  // PA 5 ** 24 ** A4
  _PA,  // PA 6 ** 25 ** A5
  _PA,  // PA 7 ** 26 ** A6
  _PB,  // PB 0 ** 27 ** A7
  _PB,  // PB 1 ** 28 ** A8
  _PB,  // PB 2 ** 29 ** A9
  _PB,  // PB 3 ** 30 ** A10
  _PA,  // PA 0 ** 31 ** A11  AREF
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  // PIN IN PORT
  // -------------------------------------------
// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  _BV( 2 ),  // PD 2 ** 0 **  USARTD_RX
  _BV( 3 ),  // PD 3 ** 1 **  USARTD_TX
  _BV( 4 ),  // PD 4 ** 2 **
  _BV( 5 ),  // PD 5 ** 3 **

  _BV( 2 ),  // PE 2 ** 4 **
  _BV( 3 ),  // PE 3 ** 5 **

  _BV( 0 ),  // PD 0 ** 6 **
  _BV( 1 ),  // PD 1 ** 7 **

  _BV( 2 ),  // PC 2 ** 8 **  USARTC0_RX
  _BV( 3 ),  // PC 3 ** 9 **  USARTC0_TX
  _BV( 4 ),  // PC 4 ** 10 ** SPI_SS
  _BV( 5 ),  // PC 5 ** 11 ** SPI_MOSI
  _BV( 6 ),  // PC 6 ** 12 ** SPI_MISO, USARTC1_RX
  _BV( 7 ),  // PC 7 ** 13 ** SPI_SCK, USARTC1_TX

#ifdef USE_TWIC
  _BV( 0 ),  // PE 0 ** 14 ** SDA1
  _BV( 0 ),  // PE 1 ** 15 ** SCL1
#else
  _BV( 0 ),  // PC 0 ** 14 ** SDA0
  _PV( 0 ),  // PC 1 ** 15 ** SCL0
#endif // USE_TWIC

  _BV( 0 ),  // PR 0 ** 16 **
  _BV( 1 ),  // PR 1 ** 17 ** default LED

#ifdef USE_TWIC
  _BV( 0 ),  // PC 0 ** 18 ** SDA, SDA0
  _BV( 1 ),  // PC 1 ** 19 ** SCL, SCL0
#else
  _BV( 0 ),  // PE 0 ** 18 ** SDA, SDA1
  _BV( 1 ),  // PE 1 ** 19 ** SCL, SCL1
#endif // USE_TWIC

  _BV( 1 ),  // PA 1 ** 20 ** A0
  _BV( 2 ),  // PA 2 ** 21 ** A1
  _BV( 3 ),  // PA 3 ** 22 ** A2
  _BV( 4 ),  // PA 4 ** 23 ** A3
  _BV( 5 ),  // PA 5 ** 24 ** A4
  _BV( 6 ),  // PA 6 ** 25 ** A5
  _BV( 7 ),  // PA 7 ** 26 ** A6
  _BV( 0 ),  // PB 0 ** 27 ** A7
  _BV( 1 ),  // PB 1 ** 28 ** A8
  _BV( 2 ),  // PB 2 ** 29 ** A9
  _BV( 3 ),  // PB 3 ** 30 ** A10
  _BV( 0 ),  // PA 0 ** 31 ** A11  AREF
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  // TIMERS
  // -------------------------------------------
  // for now 'NOT_ON_TIMER' for all - later, assign timers based
  // on pins 0-3 being enabled as PWM out for ports A through E
  // corresponding to timers A through D (see D manual sections 11.12.14,
  // also see D manual sect 13.6 for using the 'compare' channel on 'TCx2' to generate
  // a PWM output.  Must select pin as output, _AND_ enable the 'compare' output
  // for the appropriate pin.  LCMPENx/HCMPENx registers to enable it.

// subtract 2 from the digital pin number if DIGITAL_IO_PIN_SHIFT is defined
  TIMERD2,       // PD 2 ** 0 **  USARTD_RX
  TIMERD2,       // PD 3 ** 1 **  USARTD_TX
  TIMERD2,       // PD 4 ** 2 **
  TIMERD2,       // PD 5 ** 3 **

  TIMERE0,       // PE 2 ** 4 **
  TIMERE0,       // PE 3 ** 5 **

  TIMERD2,       // PD 0 ** 6 **
  TIMERD2,       // PD 1 ** 7 **

  TIMERC2,       // PC 2 ** 8 **  USARTC0_RX
  TIMERC2,       // PC 3 ** 9 **  USARTC0_TX
  TIMERC2,       // PC 4 ** 10 ** SPI_SS
  TIMERC2,       // PC 5 ** 11 ** SPI_MOSI
  TIMERC2,       // PC 6 ** 12 ** SPI_MISO, USARTC1_RX
  TIMERC2,       // PC 7 ** 13 ** SPI_SCK, USARTC1_TX

#ifdef USE_TWIC
  TIMERE0,       // PE 0 ** 14 ** SDA1
  TIMERE0,       // PE 1 ** 15 ** SCL1
#else
  TIMERC2,       // PE 0 ** 14 ** SDA0
  TIMERC2,       // PE 1 ** 15 ** SCL0
#endif // USE_TWIC

  NOT_ON_TIMER,  // PR 0 ** 16 **
  NOT_ON_TIMER,  // PR 1 ** 17 ** default LED

#ifdef USE_TWIC
  TIMERC2,      // PC 0 ** 18 ** SDA, SDA0
  TIMERC2,      // PC 1 ** 19 ** SCL, SCL0
#else
  TIMERE0,      // PE 0 ** 18 ** SDA, SDA1
  TIMERE0,      // PE 1 ** 19 ** SCL, SCL1
#endif // USE_TWIC

  NOT_ON_TIMER,  // PA 1 ** 20 ** A0
  NOT_ON_TIMER,  // PA 2 ** 21 ** A1
  NOT_ON_TIMER,  // PA 3 ** 22 ** A2
  NOT_ON_TIMER,  // PA 4 ** 23 ** A3
  NOT_ON_TIMER,  // PA 5 ** 24 ** A4
  NOT_ON_TIMER,  // PA 6 ** 25 ** A5
  NOT_ON_TIMER,  // PA 7 ** 26 ** A6
  NOT_ON_TIMER,  // PB 0 ** 27 ** A7
  NOT_ON_TIMER,  // PB 1 ** 28 ** A8
  NOT_ON_TIMER,  // PB 2 ** 29 ** A9
  NOT_ON_TIMER,  // PB 3 ** 30 ** A10
  NOT_ON_TIMER,  // PA 0 ** 31 ** A11  AREF
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
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_MONITOR        Serial1
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial2

#endif


