/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

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

  Modified 28 September 2010 by Mark Sproul

  Updated for 'xmega' core by bob frazier, S.F.T. Inc. - http://mrp3.com/

  In some cases, the xmega updates make assumptions about the pin assignments.
  See 'pins_arduino.h' for more detail.

*/

#define ARDUINO_MAIN
#include "wiring_private.h"
#include "pins_arduino.h"

void pinMode(uint8_t pin, uint8_t mode)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t sense = mode & INPUT_SENSE_MASK;
  uint8_t invert = mode & INPUT_OUTPUT_INVERT;
  volatile uint8_t *reg, *out, *ctrl;

  mode &= INPUT_OUTPUT_MASK; // remove 'sense' bits

  if (port == NOT_A_PIN)
  {
    return;
  }

  if(sense == INPUT_SENSE_DISABLED && reg != &PORTR_DIR) // 'DISABLED'
  {
    sense = PORT_ISC_INPUT_DISABLE_gc; // bit values for 'INTPUT_DISABLED' (sic)
  }
  else if(sense == INPUT_SENSE_RISING)
  {
    sense = PORT_ISC_RISING_gc; // rising
  }
  else if(sense == INPUT_SENSE_FALLING)
  {
    sense = PORT_ISC_FALLING_gc; // falling
  }
  else if(sense == INPUT_SENSE_LEVEL)
  {
    sense = PORT_ISC_LEVEL_gc; // LOW level (except events, which use HIGH level)
  }
  else // if(sense == INPUT_SENSE_BOTH) all others including 'DEFAULT'
  {
    sense = PORT_ISC_BOTHEDGES_gc; // 'both rising and falling'
  }

  if(invert) // inverted bit value
  {
    sense |= _BV(PORT_INVEN_bp); // see 11.12.15 in D manual - 'invert' bit
  }


  // JWS: can I let the optimizer do this?
  reg = portModeRegister(port);   // D manual section 11.12.1
  out = portOutputRegister(port); // D manual section 11.12.5
  ctrl = pinControlRegister(pin); // D manual section 11.12.15

  uint8_t oldSREG = SREG;
  cli(); // clear interrupt flag until I'm done assigning pin stuff

  if (mode == INPUT)
  {
    *ctrl = sense | PORT_OPC_TOTEM_gc;

    *reg &= ~bit;
  }
  else if (mode == INPUT_PULLUP)
  {
    *ctrl = sense | PORT_OPC_PULLUP_gc;         // input pullup

    *reg &= ~bit;
  }
  else if (mode == INPUT_AND_PULLUP)
  {
    *ctrl = sense | PORT_OPC_WIREDANDPULL_gc;   // wired 'and' (open drain) with pullup

    *reg &= ~bit;
  }
  else if (mode == INPUT_PULLDOWN)
  {
    *ctrl = sense | PORT_OPC_PULLDOWN_gc;       // input pullDOWN

    *reg &= ~bit;
  }
  else if (mode == INPUT_OR_PULLDOWN)
  {
    *ctrl = sense | PORT_OPC_WIREDORPULL_gc;    // wired 'or' (open drain) with pulldown

    *reg &= ~bit;
  }
  else if (mode == INPUT_BUS_KEEPER)
  {
    *ctrl = sense | PORT_OPC_BUSKEEPER_gc;      // bus keeper

    *reg &= ~bit;
  }
  else if (mode == OUTPUT_OR)
  {
    *ctrl = sense | PORT_OPC_WIREDOR_gc;        // wired 'or' (open drain)

    *reg |= bit;
  }
  else if (mode == OUTPUT_AND)
  {
    *ctrl = sense | PORT_OPC_WIREDAND_gc;       // wired 'and' (open drain)

    *reg |= bit;
  }
  else if (mode == OUTPUT_OR_PULLDOWN)
  {
    *ctrl = sense | PORT_OPC_WIREDORPULL_gc;    // wired 'or' (open drain) with pulldown

    *reg |= bit;
  }
  else if (mode == OUTPUT_AND_PULLUP)
  {
    *ctrl = sense | PORT_OPC_WIREDANDPULL_gc;   // wired 'and' (open drain) with pullup

    *reg |= bit;
  }
  else // if(mode == OUTPUT)  assume OUTPUT without open drain and/or nor pullup/down
  {
    *ctrl = sense | PORT_OPC_TOTEM_gc;          // 'totem pole' (the default)

    *reg |= bit;
  }

  SREG = oldSREG;

}

// Forcing this inline keeps the callers from having to push their own stuff
// on the stack. It is a good performance win and only takes 1 more byte per
// user than calling. (It will take more bytes on the 168.)
//
// But shouldn't this be moved into pinMode? Seems silly to check and do on
// each digitalread or write.
//
// Mark Sproul:
// - Removed inline. Save 170 bytes on atmega1280
// - changed to a switch statment; added 32 bytes but much easier to read and maintain.
// - Added more #ifdefs, now compiles for atmega645
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)

// BBB - added 'bit' parameter for xmega - it's a bit MASK, not a bit number
//       use the result from digitalPinToBitMask(pin) for 'bit'
static void turnOffPWM(uint8_t timer, uint8_t bit)
{
  switch (timer)
  {
    case TIMERD2:
      TCD2_CTRLB &= ~bit; // DISables PWM output
      break;

    case TIMERC2:
      TCC2_CTRLB &= ~bit; // DISables PWM output
      break;

    case TIMERE0:
      TCE0_CTRLB &= ~(bit << 4); // DISables PWM output
                                 // note that the 'enable' bits are in CTRLB and in upper nybble
      break;
  }
}

void digitalWrite(uint8_t pin, uint8_t val)
{
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out, *ctrl;

  if (port == NOT_A_PIN)
  {
    return;
  }

  ctrl = pinControlRegister(pin); // D manual section 11.12.15

  if(*ctrl & _BV(PORT_INVEN_bp)) // inverted
  {
    val = !val; // invert the value (so it's consistent with the pin)
  }

  // If the pin that support PWM output, we need to turn it off
  // before doing a digital write.

  // TODO:  move this feature to pinMode() like it should be
  //        or set a flag to be used with analogWrite()
  // (for now it's probably faster just to call it)

  if (timer != NOT_ON_TIMER)
  {
    turnOffPWM(timer, bit);
  }

  out = portOutputRegister(port);

  uint8_t oldSREG = SREG;
  cli();

  if (val == LOW)
  {
    *out &= ~bit;
  }
  else
  {
    *out |= bit;
  }

  SREG = oldSREG;
}

int digitalRead(uint8_t pin)
{
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ctrl;
  uint8_t bSet;

  if (port == NOT_A_PIN)
  {
    return LOW;
  }

  // If the pin that support PWM output, we need to turn it off
  // before getting a digital reading.

  // TODO:  move this feature to pinMode() like it should be
  //        or set a flag to be used with analogWrite()
  // (for now it's probably faster just to call it)

  if (timer != NOT_ON_TIMER)
  {
    turnOffPWM(timer, bit);
  }

  bSet = (*portInputRegister(port) & bit) ? true : false;

  // if the 'invert' flag is on, I invert the digital value
  // this is so that the result of 'digitalRead' and 'digitalWrite'
  // are ALWAYS consistent with the actual pin level.  Inversion is
  // needed for proper interrupt control.  So for the best consistency,
  // the invert flag will only (really) be needed for LEVEL interrupts.

  ctrl = pinControlRegister(pin); // D manual section 11.12.15

  if(*ctrl & _BV(PORT_INVEN_bp)) // inverted
  {
    bSet = !bSet;
  }

  return bSet ? HIGH : LOW;
}

