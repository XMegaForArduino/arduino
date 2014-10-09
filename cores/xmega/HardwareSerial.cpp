/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus

  Updated for 'xmega' core by bob frazier, S.F.T. Inc. - http://mrp3.com/

  In some cases, the xmega updates make assumptions about the pin assignments.
  See 'pins_arduino.h' for more detail.

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "wiring_private.h"
#include "HardwareSerial.h"



// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer, in which 'head' is the index of the location to
// which to write the next incoming character and 'tail' is the index of the
// location from which to read.

#define SERIAL_BUFFER_SIZE /*64*/128

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head; // TODO:  make this a uint8_t for speed/size improvement and ignore warnings or use explicit casts to uint16_t where needed
  volatile unsigned int tail; // TODO:  make this a uint8_t for speed/size improvement and ignore warnings or use explicit casts to uint16_t where needed
};

// ring buffers for serial ports 1 and 2 (must zero them out on startup)
ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer  =  { { 0 }, 0, 0 };
ring_buffer rx_buffer2  =  { { 0 }, 0, 0 };
ring_buffer tx_buffer2  =  { { 0 }, 0, 0 };


#if defined(SERIAL_0_CTS_ENABLED)
void InitSerialFlowControlInterrupt0(void)
{
register8_t *pCTRL;
uint8_t oldSREG;


  pCTRL = &(SERIAL_0_CTS_PORT->PIN0CTRL) + SERIAL_0_CTS_PIN_INDEX;

  SERIAL_0_CTS_PORT->DIR &= ~SERIAL_0_CTS_PIN; // it's an input

  *pCTRL = PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc; //PORT_ISC_FALLING_gc; // interrupt on falling, pulldown resistor

  // this next section enables actual interrupts

  oldSREG = SREG; // store the interrupt flag basically

  cli(); // disable interrupts for a bit

  SERIAL_0_CTS_PORT->INT1MASK &= ~SERIAL_0_CTS_PIN;
//  SERIAL_0_CTS_PORT->INTCTRL &= ~PORT_INT1LVL_gm;  // interrupt initially off

  SREG = oldSREG; // restore
}
#endif // defined(SERIAL_0_CTS_ENABLED)


#if defined(SERIAL_1_CTS_ENABLED)
static void InitSerialFlowControlInterrupt1(void)
{
register8_t *pCTRL;
uint8_t oldSREG;

  pCTRL = &(SERIAL_1_CTS_PORT->PIN0CTRL) + SERIAL_1_CTS_PIN_INDEX;

  SERIAL_1_CTS_PORT->DIR &= ~SERIAL_1_CTS_PIN; // it's an input

  *pCTRL = PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc; //PORT_ISC_FALLING_gc; // interrupt on falling, pulldown resistor

  // this next section enables actual interrupts

  oldSREG = SREG; // store the interrupt flag basically

  cli(); // disable interrupts for a bit

  SERIAL_1_CTS_PORT->INT1MASK &= ~SERIAL_1_CTS_PIN; // interrupt off (for now)
//  SERIAL_1_CTS_PORT->INTCTRL |= PORT_INT1LVL_gm; // max priority when I do this

  SREG = oldSREG; // restore
}
#endif // defined(SERIAL_1_CTS_ENABLED)


void InitSerialFlowControlInterrupts(void)
{
uint8_t oldSREG=SREG;

  cli(); // disable interrupts for a bit

#if defined(SERIAL_0_CTS_ENABLED)
  InitSerialFlowControlInterrupt0();
#endif // defined(SERIAL_0_CTS_ENABLED)

#if defined(SERIAL_1_CTS_ENABLED)
  InitSerialFlowControlInterrupt1();
#endif // defined(SERIAL_1_CTS_ENABLED)

  SREG = oldSREG; // restore
}


inline void store_char(unsigned char c, ring_buffer *buffer)
{
  unsigned int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail)
  {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

inline char set_not_rts(ring_buffer *buffer)
{
  unsigned int i1 = (unsigned int)(buffer->head + 3) % SERIAL_BUFFER_SIZE;
  unsigned int i2 = (unsigned int)(buffer->head + 2) % SERIAL_BUFFER_SIZE;
  unsigned int i3 = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  return i1 == buffer->tail || i2 == buffer->tail || i3 == buffer->tail;
}

ISR(USARTD0_RXC_vect)
{
unsigned char c;

#ifdef SERIAL_0_RTS_ENABLED
  if(set_not_rts(&rx_buffer)) // do I need to turn off RTS ?
  {
    SERIAL_0_RTS_PORT->OUT |= SERIAL_0_RTS_PIN; // set to '1'
    SERIAL_0_RTS_PORT->DIR |= SERIAL_0_RTS_PIN; // make sure it's an output
  }
#endif // SERIAL_0_RTS_ENABLED

  if(USARTD0_STATUS & _BV(USART_RXCIF_bp)) // if there is data available
  {
    c = USARTD0_DATA;
    store_char(c, &rx_buffer);
  }
  else // I got an interrupt for some reason, just eat data from data reg
  {
    c = USARTD0_DATA;
  }
}

ISR(USARTC0_RXC_vect)
{
unsigned char c;

#ifdef SERIAL_1_RTS_ENABLED
  if(set_not_rts(&rx_buffer2)) // do I need to turn off RTS ?
  {
    SERIAL_1_RTS_PORT->OUT |= SERIAL_1_RTS_PIN; // set to '1'
    SERIAL_1_RTS_PORT->DIR |= SERIAL_1_RTS_PIN; // make sure it's an output
  }
#endif // SERIAL_0_RTS_ENABLED

  if(USARTC0_STATUS & _BV(USART_RXCIF_bp)) // if there is data available
  {
    c = USARTC0_DATA;
    store_char(c, &rx_buffer2);
  }
  else // I got an interrupt for some reason, just eat data from data reg
  {
    c = USARTC0_DATA;
  }
}


void serialEvent() __attribute__((weak));
void serialEvent() {}
#define serialEvent_implemented

void serialEvent2() __attribute__((weak));
void serialEvent2() {}
#define serialEvent2_implemented



void serialEventRun(void)
{
// TODO: support this

//  if (Serial.available())
//    serialEvent();
//
//  if (Serial2.available())
//    serialEvent2();
}

#ifdef SERIAL_0_CTS_ENABLED
static char bWasCTS0;
#endif // SERIAL_0_CTS_ENABLED
#ifdef SERIAL_1_CTS_ENABLED
static char bWasCTS1;
#endif // SERIAL_1_CTS_ENABLED

ISR(USARTD0_DRE_vect)
{
#ifdef SERIAL_0_CTS_ENABLED
uint8_t oldSREG;
char bCTS = SERIAL_0_CTS_PORT->IN & SERIAL_0_CTS_PIN;
#endif // SERIAL_0_CTS_ENABLED


  if (
#ifdef SERIAL_0_CTS_ENABLED
      bCTS ||
#endif // SERIAL_0_CTS_ENABLED
      tx_buffer.head == tx_buffer.tail)
  {
#ifdef SERIAL_0_CTS_ENABLED
    if(bCTS)
    {
      oldSREG = SREG; // store the interrupt flag basically

      cli(); // disable interrupts for a bit

      bWasCTS0 = 1; // to mark that I set the interrupt

      SERIAL_0_CTS_PORT->INT1MASK |= SERIAL_0_CTS_PIN;
      SERIAL_0_CTS_PORT->INTCTRL |= PORT_INT1LVL_gm; // max priority when I do this

      SREG = oldSREG; // restore
    }
#endif // SERIAL_0_CTS_ENABLED

    // Buffer empty, so disable interrupts
    // section 19.14.3 - the CTRLA register (interrupt stuff)
    USARTD0_CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp); // only set these 2 (the DRE int is now OFF)
  }
  else
  {
    // There is more data in the output buffer. Send the next byte
    register unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    tx_buffer.tail = (tx_buffer.tail + 1) % SERIAL_BUFFER_SIZE;

    USARTD0_DATA = c;
  }
}

ISR(USARTC0_DRE_vect)
{
#ifdef SERIAL_1_CTS_ENABLED
uint8_t oldSREG;
char bCTS = SERIAL_1_CTS_PORT->IN & SERIAL_1_CTS_PIN;
#endif // SERIAL_1_CTS_ENABLED


  if (
#ifdef SERIAL_1_CTS_ENABLED
      bCTS ||
#endif // SERIAL_1_CTS_ENABLED
      tx_buffer2.head == tx_buffer2.tail)
  
  {
#ifdef SERIAL_1_CTS_ENABLED
    if(bCTS)
    {
      oldSREG = SREG; // store the interrupt flag basically

      cli(); // disable interrupts for a bit

      SERIAL_1_CTS_PORT->INT1MASK |= SERIAL_1_CTS_PIN;
      SERIAL_1_CTS_PORT->INTCTRL |= PORT_INT1LVL_gm; // max priority when I do this

      SREG = oldSREG; // restore
    }
#endif // SERIAL_1_CTS_ENABLED

    // Buffer empty, so disable interrupts
    // section 19.14.3 - the CTRLA register (interrupt stuff)
    USARTC0_CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp); // only set these 2 (the DRE int is now OFF)
  }
  else
  {
    // There is more data in the output buffer. Send the next byte
    register unsigned char c = tx_buffer2.buffer[tx_buffer2.tail];
    tx_buffer2.tail = (tx_buffer2.tail + 1) % SERIAL_BUFFER_SIZE;

    USARTC0_DATA = c;
  }
}

// helpers for hardware flow control
// these will send the 'next character' _NOW_ if one is available by
// restoring the 'DRE' interrupt.

void serial_0_cts_callback(void)
{
uint8_t oldSREG = SREG; // get this FIRST
#ifdef SERIAL_0_CTS_ENABLED
char bCTS = SERIAL_0_CTS_PORT->IN & SERIAL_0_CTS_PIN;
#endif // SERIAL_0_CTS_ENABLED


  cli(); // in case I'm currently doing somethign ELSE that affects tx_buffer

#ifdef SERIAL_0_CTS_ENABLED
  if(!bCTS) // it's cleared - turn off the interrupt
  {
    SERIAL_0_CTS_PORT->INT1MASK &= ~SERIAL_0_CTS_PIN;
//    SERIAL_0_CTS_PORT->INTCTRL |= PORT_INT1LVL_gm; // max priority when I do this
  }
#endif // SERIAL_0_CTS_ENABLED

  if(tx_buffer.head != tx_buffer.tail) // only when there's something to send
  {
    // re-enable the DRE interrupt - this will cause transmission to
    // occur again without code duplication.  see HardwareSerial::write()

    USARTD0_CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp)
                  | _BV(USART_DREINTLVL1_bp) | _BV(USART_DREINTLVL0_bp); // set int bits for rx and dre (sect 19.14.3)
  }

  SREG=oldSREG; // interrupts re-enabled
}

void serial_1_cts_callback(void)
{
uint8_t oldSREG = SREG; // get this FIRST
#ifdef SERIAL_1_CTS_ENABLED
char bCTS = SERIAL_1_CTS_PORT->IN & SERIAL_1_CTS_PIN;
#endif // SERIAL_1_CTS_ENABLED


  cli(); // in case I'm currently doing somethign ELSE that affects tx_buffer

#ifdef SERIAL_1_CTS_ENABLED
  if(!bCTS) // it's cleared - turn off the interrupt
  {
    SERIAL_1_CTS_PORT->INT1MASK &= ~SERIAL_1_CTS_PIN;
//    SERIAL_1_CTS_PORT->INTCTRL |= PORT_INT1LVL_gm; // max priority when I do this
  }
#endif // SERIAL_1_CTS_ENABLED

  if (tx_buffer2.head != tx_buffer2.tail) // only when there's something to send
  {
    // re-enable the DRE interrupt - this will cause transmission to
    // occur again without code duplication.  see HardwareSerial::write()

    USARTC0_CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp)
                  | _BV(USART_DREINTLVL1_bp) | _BV(USART_DREINTLVL0_bp); // set int bits for rx and dre (sect 19.14.3)
  }

  SREG=oldSREG; // interrupts re-enabled
}



#define NEW_BAUD_METHOD

uint16_t temp_get_baud(unsigned long baud, uint8_t use_u2x)
{
#ifdef NEW_BAUD_METHOD
uint16_t i1;
static const unsigned long aBaud[] PROGMEM = // standard baud rates
{
  2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600,
  76800, 115200, 230400, 460800, 921600
};

static const uint16_t a2x[] PROGMEM = // 2x constants for standard baud rates
{
  (7 << 12) | 12,   // 2400
  (6 << 12) | 12,   // 4800
  (5 << 12) | 12,   // 9600
  (1 << 12) | 138,  // 14400
  (4 << 12) | 12,   // 19200
  138,              // 28800
  (3 << 12) | 12,   // 38400
  (uint16_t)(-1 << 12) | 137, // 57600
  (2 << 12) | 12,   // 76800
  (uint16_t)(-2 << 12) | 135, // 115200
  (uint16_t)(-3 << 12) | 131, // 230400
  (uint16_t)(-4 << 12) | 123, // 460800
  (uint16_t)(-5 << 12) | 107  // 921600
};

static const uint16_t a1x[] PROGMEM = // 1x constants for standard baud rates
{
  (6 << 12) | 12,   // 2400
  (5 << 12) | 12,   // 4800
  (4 << 12) | 12,   // 9600
  138,              // 14400
  (3 << 12) | 12,   // 19200
  (uint16_t)(-1 << 12) | 137, // 28800
  (2 << 12) | 12,   // 38400
  (uint16_t)(-2 << 12) | 135, // 57600
  (1 << 12) | 12,   // 76800
  (uint16_t)(-3 << 12) | 131, // 115200
  (uint16_t)(-4 << 12) | 123, // 230400
  (uint16_t)(-5 << 12) | 107, // 460800
  (uint16_t)(-6 << 12) | 75   // 921600
};

  // TODO:  binary search is faster, but uses more code

  for(i1=0; i1 < sizeof(aBaud)/sizeof(aBaud[0]); i1++)
  {
    unsigned long dw1 = pgm_read_dword(&aBaud[i1]);
    if(baud == dw1)
    {
      if(use_u2x)
      {
        return pgm_read_word(&a2x[i1]);
      }
      else
      {
        return pgm_read_word(&a1x[i1]);
      }
    }
  }

  // NOTE:  baud <= F_CPU / 16 for 1x, F_CPU / 8 for 2x
  //
  // X = clk_2x ? 8 : 16    bscale >= 0:  bsel = F_CPU / ( (2 ^ bscale) * X * baud) - 1
  //                                      baud = F_CPU / ( (2 ^ bscale) * X * (bsel + 1) )
  //                        bscale < 0:   bsel = (1 / (2 ^ (bscale))) * (F_CPU / (X * baud) - 1)
  //                                      baud = F_CPU / ( X * (((2 ^ bscale) * bsel) + 1) )
  //
  // NOTE:  if bsel is zero for a given bscale, then use bscale=0 and bsel=2^(bscale - 1)
  //        see section 19.3.1
  //
  // find 'best fit baud' by calculating the best 'bscale' and 'bsel' for a given baud
  // bscale is -7 through +7 so this can be done in a simple loop

  return 1; // for now [half the maximum baud rate]

#else // NEW_BAUD_METHOD [old code left for reference]

  if(baud == 9600) // most common - see table 19-5, pg 220 in D manual
  {
    // NOTE:  initial testing showed this closer to 9800 baud - clock issues?

    if(use_u2x)
    {
      return (5 << 12) | 12;
    }
    else
    {
      return (4 << 12) | 12;
    }
  }
  else if(baud == 2400)
  {
    if(use_u2x)
    {
      return (7 << 12) | 12;
    }
    else
    {
      return (6 << 12) | 12;
    }
  }
  else if(baud == 4800)
  {
    if(use_u2x)
    {
      return (6 << 12) | 12;
    }
    else
    {
      return (5 << 12) | 12;
    }
  }
  else if(baud == 14400)
  {
    if(use_u2x)
    {
      return (1 << 12) | 138;
    }
    else
    {
      return 138;
    }
  }
  else if(baud == 19200)
  {
    if(use_u2x)
    {
      return (4 << 12) | 12;
    }
    else
    {
      return (3 << 12) | 12;
    }
  }
  else if(baud == 28800)
  {
    if(use_u2x)
    {
      return 138;
    }
    else
    {
      return (-1 << 12) | 137;
    }
  }
  else if(baud == 38400)
  {
    if(use_u2x)
    {
      return (3 << 12) | 12;
    }
    else
    {
      return (2 << 12) | 12;
    }
  }
  else if(baud == 57600)
  {
    if(use_u2x)
    {
      return (-1 << 12) | 137;
    }
    else
    {
      return (-2 << 12) | 135;
    }
  }
  else if(baud == 76800)
  {
    if(use_u2x)
    {
      return (2 << 12) | 12;
    }
    else
    {
      return (1 << 12) | 12;
    }
  }
  else if(baud == 115200)
  {
    if(use_u2x)
    {
      return (-2 << 12) | 135;
    }
    else
    {
      return (-3 << 12) | 131;
    }
  }
  else if(baud == 230400)
  {
    if(use_u2x)
    {
      return (-3 << 12) | 131;
    }
    else
    {
      return (-4 << 12) | 123;
    }
  }
  else if(baud == 460800)
  {
    if(use_u2x)
    {
      return (-4 << 12) | 123;
    }
    else
    {
      return (-5 << 12) | 107;
    }
  }
  else if(baud == 921600)
  {
    if(use_u2x)
    {
      return (-5 << 12) | 107;
    }
    else
    {
      return (-6 << 12) | 75;
    }
  }
  else
  {
    // TODO:  calculate it
    //
    // X = clk_2x ? 8 : 16    bscale >= 0:  bsel = F_CPU / ( (2 ^ bscale) * X * baud) - 1
    //                        bscale < 0:   bsel = (1 / (2 ^ (bscale))) * (F_CPU / (X * baud) - 1)
    //
    // TODO:  find 'best fit baud' by tweeking bscale?  bscale range is -7 to 7

    return 1; // for now [half the maximum baud rate]
  }

#endif // NEW_BAUD_METHOD
}




// Constructors ////////////////////////////////////////////////////////////////

void HardwareSerial::init(ring_buffer *rx_buffer0, ring_buffer *tx_buffer0,
                          uint16_t usart0)
{
  _rx_buffer = rx_buffer0;
  _tx_buffer = tx_buffer0;
  _usart = (volatile USART_t *)usart0;
}

HardwareSerial::HardwareSerial(ring_buffer *rx_buffer0, ring_buffer *tx_buffer0,
                               uint16_t usart0) /*__attribute__ ((noinline))*/
{
  _rx_buffer = rx_buffer0;
  _tx_buffer = tx_buffer0;
  _usart = (volatile USART_t *)usart0;
}

// Public Methods //////////////////////////////////////////////////////////////

// 'D' manual, section 19.5
// USART Initialization
// USART initialization should use the following sequence:
// 1. Set the TxD pin value high, and optionally set the XCK pin low.
// 2. Set the TxD and optionally the XCK pin as output.
// 3. Set the baud rate and frame format.
// 4. Set the mode of operation (enables XCK pin output in synchronous mode).
// 5. Enable the transmitter or the receiver, depending on the usage.
// For interrupt-driven USART operation, global interrupts should be disabled during the initialization.
// Before doing a re-initialization with a changed baud rate or frame format, be sure that there are no ongoing transmissions
// while the registers are changed.

void HardwareSerial::begin(unsigned long baud)
{
  uint16_t baud_setting;
  uint8_t use_u2x;


  if (baud <= 57600)
  {
    use_u2x = 0;
  }
  else
  {
    use_u2x = _BV(USART_CLK2X_bp);  // enable CLK2X - bit 2 in the CTRLB register (section 19.14.4)
  }

  uint8_t oldSREG = SREG;
  cli(); // clear interrupt flag until I'm done assigning pin stuff

  if(_usart == &USARTD0)
  {
    uint8_t bit = _BV(3); // TX on PD3
    volatile uint8_t *reg = &PORTD_DIR;
    volatile uint8_t *out = &PORTD_OUT;
    volatile uint8_t *ctrl = &PORTD_PIN3CTRL;

    *ctrl = _BV(2) | _BV(1) | _BV(0); // 'INTPUT_DISABLED' (sic) with 'totem pole' (the default)
    *out |= bit;  // set to 'HIGH'
    *reg |= bit;  // set as output

    bit = _BV(2); // RX on PD2
    ctrl = &PORTD_PIN2CTRL;

    *ctrl = 0; //_BV(2) | _BV(1) | _BV(0);  experimentation shows I need a zero here
    *out &= ~bit; // low
    *reg &= ~bit; // set as an input

#ifdef SERIAL_0_RTS_ENABLED
    ctrl = &(SERIAL_0_RTS_PORT->PIN0CTRL) + SERIAL_0_RTS_PIN_INDEX;

    *ctrl = PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
    SERIAL_0_RTS_PORT->OUT &= ~SERIAL_0_RTS_PIN; // set to '0'
    SERIAL_0_RTS_PORT->DIR |= SERIAL_0_RTS_PIN; // make sure it's an output
#endif // SERIAL_0_RTS_ENABLED

#ifdef SERIAL_0_CTS_ENABLED
    InitSerialFlowControlInterrupt0();
#endif // SERIAL_0_CTS_ENABLED
  }
  else if(_usart == &USARTC0)
  {
    uint8_t bit = _BV(3); // TX on PC3
    volatile uint8_t *reg = &PORTC_DIR;
    volatile uint8_t *out = &PORTC_OUT;
    volatile uint8_t *ctrl = &PORTC_PIN3CTRL;

    *ctrl = _BV(2) | _BV(1) | _BV(0); // 'INTPUT_DISABLED' (sic) with 'totem pole' (the default)
    *out |= bit;  // set to 'HIGH'
    *reg |= bit;  // set as output

    bit = _BV(2); // RX on PC2
    ctrl = &PORTC_PIN2CTRL;

    *ctrl = 0; // _BV(2) | _BV(1) | _BV(0);  experimentation shows I need a zero here
    *out &= ~bit; // low
    *reg &= ~bit; // set as an input

#ifdef SERIAL_1_RTS_ENABLED
    ctrl = &(SERIAL_0_RTS_PORT->PIN0CTRL) + SERIAL_1_RTS_PIN_INDEX;

    SERIAL_1_RTS_PORT->CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
    SERIAL_1_RTS_PORT->OUT &= ~SERIAL_1_RTS_PIN; // set to '0'
    SERIAL_1_RTS_PORT->DIR |= SERIAL_1_RTS_PIN; // make sure it's an output
#endif // SERIAL_1_RTS_ENABLED

#ifdef SERIAL_1_CTS_ENABLED
    InitSerialFlowControlInterrupt1();
#endif // SERIAL_1_CTS_ENABLED
  }

  SREG = oldSREG;

  // baud rate calc - page 220 table 19-5 [for standard values]
  //                  table 19-1 (page 211) for calculation formulae
  // (also see theory discussion on page 219)

  // section 19.4.4
  _usart->CTRLB = use_u2x; // enable clock 2x (everything else disabled)

  baud_setting = temp_get_baud(baud, use_u2x);


  // section 19.14.5 - USART mode, parity, bits
  // CMODE 7:6 = 00 [async]  PMODE 5:4 = 00 [none]  SBMODE 3 = 0 [1 bit]   CHSIZE 2:0 = 3 (8-bit)
  _usart->CTRLC = SERIAL_8N1;

  _usart->BAUDCTRLA = (uint8_t)(baud_setting & 0xff);
  _usart->BAUDCTRLB = (uint8_t)(baud_setting >> 8);

  transmitting = false;

  // section 19.4.4
  _usart->CTRLB = use_u2x | _BV(USART_RXEN_bp) | _BV(USART_TXEN_bp); // enable RX, enable TX.  Bit 2 will be 1 or 0 based on clock 2x/1x.  multi-processor disabled.  bit 9 = 0
  _usart->CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp); // priority 3 for RX interrupts.  DRE and TX interrupts OFF.

}


void HardwareSerial::begin(unsigned long baud, byte config)
{
  uint16_t baud_setting;
  uint8_t use_u2x;


  if (baud <= 57600)
  {
    use_u2x = 0;
  }
  else
  {
    use_u2x = _BV(USART_CLK2X_bp);  // enable CLK2X - bit 2 in the CTRLB register (section 19.14.4)
  }


  uint8_t oldSREG = SREG;
  cli(); // clear interrupt flag until I'm done assigning pin stuff

  if(_usart == &USARTD0)
  {
    uint8_t bit = _BV(3); // TX on PD3
    volatile uint8_t *reg = &PORTD_DIR;
    volatile uint8_t *out = &PORTD_OUT;
    volatile uint8_t *ctrl = &PORTD_PIN3CTRL;

    *ctrl = 0; // trigger on BOTH, totem, no pullup   //_BV(2) | _BV(1) | _BV(0); // 'INTPUT_DISABLED' (sic) with 'totem pole' (the default)
    *out |= bit;  // set to 'HIGH'
    *reg |= bit;  // set as output

    bit = _BV(2); // RX on PD2
    ctrl = &PORTD_PIN2CTRL;

    *ctrl = 0; // triger on BOTH, no pullup
    *out &= ~bit; // off
    *reg &= ~bit; // set as input
  }
  else if(_usart == &USARTC0)
  {
    uint8_t bit = _BV(3); // TX on PC3
    volatile uint8_t *reg = &PORTC_DIR;
    volatile uint8_t *out = &PORTC_OUT;
    volatile uint8_t *ctrl = &PORTC_PIN3CTRL;

    *ctrl = 0; // trigger on BOTH, totem, no pullup    //_BV(2) | _BV(1) | _BV(0); // 'INTPUT_DISABLED' (sic) with 'totem pole' (the default)
    *out |= bit;  // set to 'HIGH'
    *reg |= bit;  // set as output

    bit = _BV(2); // RX on PC2
    ctrl = &PORTC_PIN2CTRL;

    *ctrl = 0; // triger on BOTH, no pullup
    *out &= ~bit; // off
    *reg &= ~bit; // set as input
  }

  SREG = oldSREG;

  // baud rate calc - page 220 table 19-5 [for standard values]
  //                  table 19-1 (page 211) for calculation formulae
  // (also see theory discussion on page 219)

  // section 19.4.4
  _usart->CTRLB = use_u2x; // enable clock 2x (everything else disabled)

  baud_setting = temp_get_baud(baud, use_u2x);


  // section 19.14.5 - USART mode, parity, bits
  // CMODE 7:6   00 [async]
  // PMODE 5:4   00=none  10=even 11=odd
  // SBMODE 3    0=1 stop  1=2 stop
  // CHSIZE 2:0  000=5 bit 001=6 bit  010=7 bit  011=8 bit  111=9 bit
  _usart->CTRLC = config & ~(_BV(USART_CMODE1_bp)|_BV(USART_CMODE0_bp)); // make sure bits 6 and 7 are cleared

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)

  _usart->BAUDCTRLA = (uint8_t)(baud_setting & 0xff);
  _usart->BAUDCTRLB = (uint8_t)(baud_setting >> 8);

  transmitting = false;

  // section 19.4.4
  _usart->CTRLB = use_u2x | _BV(USART_RXEN_bp) | _BV(USART_TXEN_bp); // enable RX, enable TX.  Bit 2 will be 1 or 0 based on clock 2x/1x.  multi-processor disabled.  bit 9 = 0
  _usart->CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp); // priority 3 for RX interrupts.  DRE and TX interrupts OFF.
}

void HardwareSerial::end()
{
  // wait for transmission of outgoing data
  while (_tx_buffer->head != _tx_buffer->tail)
    ;

  _usart->CTRLB = 0; // disable RX, TX
  _usart->CTRLA = 0; // disable interrupts

  // clear any received data
  _rx_buffer->head = _rx_buffer->tail;
}

int HardwareSerial::available(void)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % SERIAL_BUFFER_SIZE;
}

int HardwareSerial::peek(void)
{
  if (_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
    return _rx_buffer->buffer[_rx_buffer->tail];
  }
}

int HardwareSerial::read(void)
{
  // each time I'm ready to read a byte, double-check that the RTS (when enabled)
  // needs to be set to a 0 value [which enables things to be sent to me].  As
  // I deplete the buffer, RTS will enable, and as I fill it, RTS will disable.

  // This section is the 'deplete' part.  So I'll set RTS to 'LOW' which is 'ok to send'
  // if the buffer is _NOT_ too full (the set_not_rts() function determines that)

#ifdef SERIAL_0_RTS_ENABLED
  uint8_t oldSREG = SREG;

  cli(); // clear interrupt flag until I'm done assigning pin stuff

  if(_rx_buffer == &rx_buffer && // it's serial #0
     !set_not_rts(&rx_buffer))   // do I need to turn off RTS ?
  {
    SERIAL_0_RTS_PORT->OUT &= ~SERIAL_0_RTS_PIN; // set to '0'
    SERIAL_0_RTS_PORT->DIR |= SERIAL_0_RTS_PIN; // make sure it's an output
  }

  SREG = oldSREG;
#endif // SERIAL_0_RTS_ENABLED

#ifdef SERIAL_1_RTS_ENABLED
  uint8_t oldSREG = SREG;

  cli(); // clear interrupt flag until I'm done assigning pin stuff

  if(_rx_buffer == &rx_buffer2 && // it's serial #0
     !set_not_rts(&rx_buffer2))   // do I need to turn off RTS ?
  {
    SERIAL_1_RTS_PORT->OUT &= ~SERIAL_1_RTS_PIN; // set to '0'
    SERIAL_1_RTS_PORT->DIR |= SERIAL_1_RTS_PIN; // make sure it's an output
  }

  SREG = oldSREG;
#endif // SERIAL_1_RTS_ENABLED

  // back to regular serial I/O handling

  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
    uint8_t oldSREG = SREG;

    cli(); // clear interrupt flag until I'm done assigning pin stuff

    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];

    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;

    SREG = oldSREG;
    return c;
  }
}

void HardwareSerial::flush()
{
  // TODO:  force an 'sei' here?

  // DATA is kept full while the buffer is not empty, so TXCIF triggers when EMPTY && SENT
  while (transmitting && !(_usart->STATUS & _BV(USART_TXCIF_bp))) // TXCIF bit 6 indicates transmit complete
    ;

  transmitting = false;
}

size_t HardwareSerial::write(uint8_t c)
{
register unsigned int i1;
uint8_t oldSREG = SREG; // get this FIRST

  cli(); // in case I'm currently doing somethign ELSE that affects the _tx_buffer

  i1 = (unsigned int)((_tx_buffer->head + 1) % SERIAL_BUFFER_SIZE); // next head after this char

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  // ???: return 0 here instead?

  while (i1 == _tx_buffer->tail)
  {
    // make sure the interrupt is enabled in this case, so I can actually send things
    // NOTE:  this messes with flow control, but I'm waiting so who cares.  it will still work

    cli(); // always turn ints off before making this assignment

    _usart->CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp)
                  | _BV(USART_DREINTLVL1_bp) | _BV(USART_DREINTLVL0_bp); // set them all

    sei(); // can't hurt to repeat it multiple times - but interrupts MUST be enabled for this to work
  }

  cli();   // for these next steps, disable interrupts.  it helps prevent 'ISR enable/disable thrashing'
  // if I didn't have to wait for buffer space, I'm already covered

  _tx_buffer->buffer[_tx_buffer->head] = c;
  _tx_buffer->head = i1;

  // NOTE:  this messes with flow control.  it will still work
//  _usart->CTRLA |= _BV(1) | _BV(0); // make sure I (re)enable the DRE interrupt (sect 19.14.3)
  _usart->CTRLA = _BV(USART_RXCINTLVL1_bp) | _BV(USART_RXCINTLVL0_bp)
                | _BV(USART_DREINTLVL1_bp) | _BV(USART_DREINTLVL0_bp); // set int bits for rx and dre (sect 19.14.3)

  transmitting = true;
//  sbi(_usart->STATUS,6);  // clear the TXCIF bit by writing a 1 to its location (sect 19.14.2)
  _usart->STATUS = _BV(USART_TXCIF_bp); // other bits must be written as zero

  SREG=oldSREG; // interrupts re-enabled

  return 1;
}

HardwareSerial::operator bool()
{
  return true;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

HardwareSerial Serial(&rx_buffer, &tx_buffer, (uint16_t)&USARTD0);
HardwareSerial Serial2(&rx_buffer2, &tx_buffer2, (uint16_t)&USARTC0);





