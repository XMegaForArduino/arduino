/*
  wiring_analog.c - analog input and output
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

#include "wiring_private.h"
#include "pins_arduino.h"

#ifndef ADCA_SAMPCTRL
#define ADCA_SAMPCTRL  _SFR_MEM8(0x0208) /* missing from header for some reason, struct defines it as reserved_0x08 */
#endif // ADCA_SAMPCTRL


uint8_t analog_reference = 4;// the default analog reference is Vcc / 2

// adc_setup() - call this from init() and whenever you wake up from sleep mode
void adc_setup(void)
{
  // calibration is a 16-bit register - CAL0 + (CAL1 << 8)
  ADCA_CAL = (uint16_t)readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_ADCACAL0)
           | (((uint16_t)readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_ADCACAL1)) << 8);

  // must make sure power reduction register enables the ADC
  PR_PRPA &= ~PR_ADC_bm; // clear this bit to enable the ADC clock

  // assign clock prescaler for 100khz, and clear the interrupt bit
  // also make sure the 'interrupt enable' is OFF

  ADCA_EVCTRL = 0; // no triggering events (sect 22.14.4)

  ADCA_PRESCALER = ADC_PRESCALER_DIV256_gc; // 100khz, approximately, for 32Mhz clock

  ADCA_CTRLA = _BV(ADC_ENABLE_bp); // enables the ADC
  ADCA_CTRLB = _BV(6) | _BV(4); // medium current limit, signed mode [temporary]
  //   _BV(6) | _BV(5);     // section 22.14.2, 'HIGH' current limit, no apparent bit value constants in iox64d4.h
             // NOTE:  all other bits are zero - no 'freerun', 12-bit right-justified unsigned mode

  ADCA_REFCTRL = _BV(ADC_REFSEL2_bp);         // bit 100 --> Vcc/2 as reference
             // NOTE:  all other bits are zero (bandgap, tempref) - section 22.14.3
             // TODO:  use 'analog_reference', bit-shifted (see also 'analogReference()' below)

  // TODO:  is this actually a RESERVED functionality?
  ADCA_SAMPCTRL = 24; // sect 22.14.8 - this value + 1 is # of "half cycles" used for sampling
                      // in this case, it's 25 "half cycles" at 100khz, or appx 8khz (125uS)
                      // adjust this for 'best accuracy' without taking for-freaking-evar
                      // also make sure the sample rate is lower than the 'HIGH LIMIT' max rate (see 22.14.2)

  // set up the channel (no offset calc at this time - later do an offset calc)
  ADCA_CH0_SCAN = 0;       // disable scan
  ADCA_CH0_INTCTRL = 0;    // no interrupts, flag on complete sect 22.15.3

  // NOTE:  the E5 has a significant difference in how it handles 'DIFF WITH GAIN'
  //        ADC_CH_INPUTMODE_DIFFWGAINL_gc uses A0-A3, GND, and internal GND (not same as D and earlier)
  //        ADC_CH_INPUTMODE_DIFFWGAINH_gc uses A4-A7 and GND (see E manual table 24-16,17 pg 366-7) (similar to D and earlier)
#if defined (__AVR_ATxmega16E5__) || defined (__AVR_ATxmega32E5__)
  ADCA_CH0_CTRL = ADC_CH_INPUTMODE_DIFFWGAINH_gc | // so MUXNEG 111b aka '7' still picks "GND" for 'diff with gain'
                  ADC_CH_GAIN_DIV2_gc; // (see 24.15.1 in E manual - they differ)
#else // everything else not an 'E' series
  ADCA_CH0_CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc |
                  ADC_CH_GAIN_DIV2_gc; // (see 22.15.1 in D manual - they differ)
#endif

  // clear interrupt flag (probably not needed)
  ADCA_INTFLAGS = _BV(ADC_CH0IF_bp); // write a 1 to the interrupt bit (which clears it - 22.14.6)

  analogRead(0); // do a single conversion so that everything stabilizes

// ADCA.CALL = readCalibrationData(&PRODSIGNATURES_ADCACAL0);
// ADCA.CALH = readCalibrationData(&PRODSIGNATURES_ADCACAL1);
}

void analogReference(uint8_t mode)
{
  // can't actually set the register here because the default setting
  // will connect AVCC and the AREF pin, which would cause a short if
  // there's something connected to AREF.

  // NOTE: on atmega, this is definitely the case.  On xmega, there's no 'AREF' output.

  analog_reference = mode;

  // valid xmega modes are 0-4 (5-7 are reserved)

//  ADCA_REFCTRL = (ADCA_REFCTRL & ~ADC_REFSEL_gm)
//               | ((mode & 7) << ADC_REFSEL_gp);         // section 22.14.3
}

// For 100% atmega compatibility, analogRead will return a value of 0-1023
// for input voltages of 0 to Vcc (assuming AVCC is connected to VCC, etc.)
// by using a gain of 1/2, a comparison of Vcc/2, and signed conversion

int analogRead(uint8_t pin)
{
  short iRval;

  // this is pure XMEGA code

  if(pin >= A0)
  {
#if NUM_ANALOG_PINS <= 8
    if(pin > A7)
#elif NUM_ANALOG_PINS <= 12
    if(pin > A11)
#else // NUM_ANALOG_PINS <= 16
    if(pin > A15)
#endif // NUM_ANALOG_PINS
    {
      return 0;
    }
    pin -= A0; // allow both 'pin number' and 'channel number'
  }

  // TODO:  re-configure the analog reference? for now, leave it

  ADCA_CH0_SCAN = 0; // disable scan
  ADCA_CH0_MUXCTRL = (pin << ADC_CH_MUXPOS_gp) // sect 22.15.2
                   | 7; // GND is the 'other input' (NOTE:  this is MUXNEG)

  ADCA_CH0_INTCTRL = 0;    // no interrupts, flag on complete sect 22.15.3

#ifdef ADC_CH_IF_bm /* iox16e5.h and iox32e5.h - probably the ATMel Studio version */
  ADCA_CH0_INTFLAGS = ADC_CH_IF_bm; // write a 1 to the interrupt bit (which clears it - 22.15.4)
#else // everyone else
  ADCA_CH0_INTFLAGS = ADC_CH_CHIF_bm; // write a 1 to the interrupt bit (which clears it - 22.15.4)
#endif // ADC_CH_IF_bm

// old code, for reference
//  ADCA_CH0_CTRL = ADC_CH_START_bm       // conversion start
//                | ADC_CH_INPUTMODE0_bm; // zero gain and input mode '01' (see 22.15.1)

// AS NOTED ABOVE, the E series is quite a bit different here with respect to how it handles
// differential inputs.  it's actually better, but incompatible (hence the need for '#if' block)
#if defined (__AVR_ATxmega16E5__) || defined (__AVR_ATxmega32E5__)
  ADCA_CH0_CTRL = ADC_CH_START_bm |       // conversion start
                  ADC_CH_INPUTMODE_DIFFWGAINH_gc | // so MUXNEG 111b aka '7' still picks "GND" for 'diff with gain'
                  ADC_CH_GAIN_DIV2_gc; // (see 24.15.1 in E manual - they differ)
#else // everything else not an 'E' series
  ADCA_CH0_CTRL = ADC_CH_START_bm |       // conversion start
                  ADC_CH_INPUTMODE_DIFFWGAIN_gc |
                  ADC_CH_GAIN_DIV2_gc; // (see 22.15.1 in D manual - they differ)
#endif

#ifdef ADC_CH_IF_bm /* iox16e5.h and iox32e5.h - probably the ATMel Studio version */
  while(!(ADCA_CH0_INTFLAGS & ADC_CH_IF_bm)) { }
#else // everyone else
  while(!(ADCA_CH0_INTFLAGS & ADC_CH_CHIF_bm)) { }
#endif // ADC_CH_IF_bm

  iRval = ADCA_CH0_RES;

  if(iRval < 0) // backward compatibility
  {
    return 0;
  }

  return iRval / 2;  // -1023 to 1023 [TODO:  clip at zero?]
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint8_t pin, int val)
{
  // We need to make sure the PWM output is enabled for those pins
  // that support it, as we turn it off when digitally reading or
  // writing with them.  Also, make sure the pin is in output mode
  // for consistenty with Wiring, which doesn't require a pinMode
  // call for the analog output pins.

  // NOTE:  period registers all contain zeros, which is the MAXIMUM period of 0-255
#ifdef TCC4 /* 'E' series and later that have TCC4 */
  uint8_t mode;
#endif // TCC4
  uint8_t bit = digitalPinToBitMask(pin);

  pinMode(pin, OUTPUT); // forces 'totem pole' - TODO allow for something different?

  // note 'val' is a SIGNED INTEGER.  deal with 'out of range' values accordingly

  if (val <= 0)
  {
    digitalWrite(pin, LOW);
  }
  else if (val >= 255)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {
    switch(digitalPinToTimer(pin))
    {
#ifdef TCC4 /* 'E' series and later that have TCC4 */

      case TIMERC4:

        if(bit == 1)
        {
          ((uint8_t *)&(TCC4_CCA))[0] = val; // NOTE:  these are 16-bit registers and low/high byte access is needed
          mode = (TCC4_CTRLE & ~TC4_LCCAMODE_gm) | TC4_LCCAMODE1_bm;
        }
        else if(bit == 2)
        {
          ((uint8_t *)&(TCC4_CCA))[1] = val;
          mode = (TCC4_CTRLE & ~TC4_LCCBMODE_gm) | TC4_LCCBMODE1_bm;
        }
        else if(bit == 4)
        {
          ((uint8_t *)&(TCC4_CCB))[0] = val;
          mode = (TCC4_CTRLE & ~TC4_LCCCMODE_gm) | TC4_LCCCMODE1_bm;
        }
        else if(bit == 8)
        {
          ((uint8_t *)&(TCC4_CCB))[1] = val;
          mode = (TCC4_CTRLE & ~TC4_LCCDMODE_gm) | TC4_LCCDMODE1_bm;
        }
        else if(bit == 16)
        {
          ((uint8_t *)&(TCC4_CCC))[0] = val;
          mode = (TCC4_CTRLF & ~TC4_HCCAMODE_gm) | TC4_HCCAMODE1_bm;
        }
        else if(bit == 32)
        {
          ((uint8_t *)&(TCC4_CCC))[1] = val;
          mode = (TCC4_CTRLF & ~TC4_HCCBMODE_gm) | TC4_HCCBMODE1_bm;
        }
        else if(bit == 64)
        {
          ((uint8_t *)&(TCC4_CCD))[0] = val;
          mode = (TCC4_CTRLF & ~TC4_HCCCMODE_gm) | TC4_HCCAMODE1_bm;
        }
        else if(bit == 128)
        {
          ((uint8_t *)&(TCC4_CCD))[1] = val;
          mode = (TCC4_CTRLF & ~TC4_HCCDMODE_gm) | TC4_HCCDMODE1_bm;
        }
        else
        {
          break;
        }

// this is a reminder that the low nybble should be assigned the correct value for single-slope PWM mode
//        TCC4_CTRLB = TC45_WGMODE_SINGLESLOPE_gc;
        if(bit <= 8)
        {
          TCC4_CTRLE = mode;
        }
        else
        {
          TCC4_CTRLF = mode;
        }

        break;

      case TIMERD5:

        if(bit == 1 || bit == 16)
        {
          ((uint8_t *)&(TCD5_CCA))[0] = val; // NOTE:  these are 16-bit registers and low/high byte access is needed
          mode = (TCD5_CTRLE & ~TC5_LCCAMODE_gm) | TC5_LCCAMODE1_bm;
        }
        else if(bit == 2 || bit == 32)
        {
          ((uint8_t *)&(TCD5_CCA))[1] = val;
          mode = (TCD5_CTRLE & ~TC5_LCCBMODE_gm) | TC5_LCCBMODE1_bm;
        }
        else if(bit == 4 || bit == 64)
        {
          ((uint8_t *)&(TCD5_CCB))[0] = val;
          mode = (TCD5_CTRLF & ~TC5_HCCAMODE_gm) | TC5_HCCAMODE1_bm;
        }
        else if(bit == 8 || bit == 128)
        {
          ((uint8_t *)&(TCD5_CCB))[1] = val;
          mode = (TCD5_CTRLF & ~TC5_HCCBMODE_gm) | TC5_HCCBMODE1_bm;
        }
        else
        {
          break;
        }

// this is a reminder that the low nybble should be assigned the correct value for single-slope PWM mode
//        TCD5_CTRLB = TC45_WGMODE_SINGLESLOPE_gc;

        if(bit == 1 || bit == 2 ||  bit == 16 || bit == 32)
        {
          TCD5_CTRLE = mode;
        }
        else
        {
          TCD5_CTRLF = mode;
        }

        break;
#else // everything else
      case TIMERD2:

#ifndef TCD2

        // NOTE:  timers C2 and D2 count DOWN, always.  However, the output starts at zero
        //        and flips to 1 when CTR reaches the CMP value.  So a value of 255 would be
        //        '1' and 0 would be '0', as is expected.  See 'D' manual 13.6.2
        if(bit == 1)
        {
          ((uint8_t *)&(TCD0_CCA))[0] = val;
        }
        else if(bit == 2)
        {
          ((uint8_t *)&(TCD0_CCB))[0] = val;
        }
        else if(bit == 4)
        {
          ((uint8_t *)&(TCD0_CCC))[0] = val;
        }
        else if(bit == 8)
        {
          ((uint8_t *)&(TCD0_CCD))[0] = val;
        }
        else if(bit == 16)
        {
          ((uint8_t *)&(TCD0_CCA))[1] = val;
        }
        else if(bit == 32)
        {
          ((uint8_t *)&(TCD0_CCB))[1] = val;
        }
        else if(bit == 64)
        {
          ((uint8_t *)&(TCD0_CCC))[1] = val;
        }
        else if(bit == 128)
        {
          ((uint8_t *)&(TCD0_CCD))[1] = val;
        }

        TCD0_CTRLB |= bit; // enables output

#else // TCD2 defined

        // NOTE:  timers C2 and D2 count DOWN, always.  However, the output starts at zero
        //        and flips to 1 when CTR reaches the CMP value.  So a value of 255 would be
        //        '1' and 0 would be '0', as is expected.  See 'D' manual 13.6.2
        if(bit == 1)
        {
          TCD2_LCMPA = val;
        }
        else if(bit == 2)
        {
          TCD2_LCMPB = val;
        }
        else if(bit == 4)
        {
          TCD2_LCMPC = val;
        }
        else if(bit == 8)
        {
          TCD2_LCMPD = val;
        }
        else if(bit == 16)
        {
          TCD2_HCMPA = val;
        }
        else if(bit == 32)
        {
          TCD2_HCMPB = val;
        }
        else if(bit == 64)
        {
          TCD2_HCMPC = val;
        }
        else if(bit == 128)
        {
          TCD2_HCMPD = val;
        }

        TCD2_CTRLB |= bit; // enables output
#endif // TCD2 defined
        break;

      case TIMERC2:

#ifndef TCC2

        // NOTE:  timers C2 and D2 count DOWN, always.  However, the output starts at zero
        //        and flips to 1 when CTR reaches the CMP value.  So a value of 255 would be
        //        '1' and 0 would be '0', as is expected.  See 'D' manual 13.6.2
        if(bit == 1)
        {
          ((uint8_t *)&(TCC0_CCA))[0] = val;
        }
        else if(bit == 2)
        {
          ((uint8_t *)&(TCC0_CCB))[0] = val;
        }
        else if(bit == 4)
        {
          ((uint8_t *)&(TCC0_CCC))[0] = val;
        }
        else if(bit == 8)
        {
          ((uint8_t *)&(TCC0_CCD))[0] = val;
        }
        else if(bit == 16)
        {
          ((uint8_t *)&(TCC0_CCA))[1] = val;
        }
        else if(bit == 32)
        {
          ((uint8_t *)&(TCC0_CCB))[1] = val;
        }
        else if(bit == 64)
        {
          ((uint8_t *)&(TCC0_CCC))[1] = val;
        }
        else if(bit == 128)
        {
          ((uint8_t *)&(TCC0_CCD))[1] = val;
        }

        TCC0_CTRLB |= bit; // enables output

#else // TCC2 defined
        // NOTE:  timers C2 and D2 count DOWN, always.  However, the output starts at zero
        //        and flips to 1 when CTR reaches the CMP value.  So a value of 255 would be
        //        '1' and 0 would be '0', as is expected.  See 'D' manual 13.6.2
        if(bit == 1)
        {
          TCC2_LCMPA = val;
        }
        else if(bit == 2)
        {
          TCC2_LCMPB = val;
        }
        else if(bit == 4)
        {
          TCC2_LCMPC = val;
        }
        else if(bit == 8)
        {
          TCC2_LCMPD = val;
        }
        else if(bit == 16)
        {
          TCC2_HCMPA = val;
        }
        else if(bit == 32)
        {
          TCC2_HCMPB = val;
        }
        else if(bit == 64)
        {
          TCC2_HCMPC = val;
        }
        else if(bit == 128)
        {
          TCC2_HCMPD = val;
        }

        TCC2_CTRLB |= bit; // enables output

#endif // TCC2 defined
        break;

#if NUM_DIGITAL_PINS > 18 /* meaning there is a PORT E available */

      case TIMERE0:
        // timer E0 counts UP, but a value of 0 would still generate a '0' output because
        // the output STARTS at a 1, and flips to 0 when the CTR reaches the CC register
        // Similarly, a value of 255 would generate a '1'.  see section 12.8.3 in the 'D' manual
        if(bit == 1)
        {
          TCE0_CCA = val; // NOTE:  these are 16-bit registers (but I'm in 8-bit mode so it's fine)
        }
        else if(bit == 2)
        {
          TCE0_CCB = val;
        }
        else if(bit == 4)
        {
          TCE0_CCC = val;
        }
        else if(bit == 8)
        {
          TCE0_CCD = val;
        }

// this is a reminder that the low nybble should be assigned the correct value for single-slope PWM mode
//        TCE0_CTRLB = TC_WGMODE_SS_gc; // single-slope PWM.  NOTE:  this counts UP, whereas the other timers count DOWN

        TCE0_CTRLB |= (bit << 4); // enables output (0-3 only, but that's all PORT E has anyway)
                                  // note that the 'enable' bits are in CTRLB and in upper nybble
        break;

#endif // NUM_DIGITAL_PINS > 18

#endif // TCC4 check

      case NOT_ON_TIMER:
      default:
        if (val < 128)
        {
          digitalWrite(pin, LOW);
        }
        else
        {
          digitalWrite(pin, HIGH);
        }
    }
  }
}

