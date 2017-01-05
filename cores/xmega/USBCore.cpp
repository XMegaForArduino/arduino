/* Copyright (c) 2010, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/


// **************************************************************************
// This software incorporates Arthur C. Clarke's 3rd law:
//  "Any sufficiently advanced technology is indistinguishable from magic."
//  (no chickens were harmed nor sacrificed in the completion of this work)
// **************************************************************************

// Updated for the XMegaForArduino project by Bob Frazier, S.F.T. Inc.


#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"

#include "wiring_private.h"


#if defined(USBCON)

// CONDITIONALS FOR COMPILE-TIME OPTIONS

#define INIT_EP0_ONE_TIME_ONLY
#define DEBUG_CODE 1 // TEMPORARY


// This next block of code is to deal with gcc bug 34734 on compilers < 4.6
#if __GNUC__ > 4 || (__GNUC__ > 4 && __GNUC_MINOR__ >= 6)
#define PROGMEM_ORIG PROGMEM
#else // PROGMEM workaround

// to avoid the bogus "initialized variables" warning
#ifdef PROGMEM
#undef PROGMEM
#endif // PROGMEM re-define

#define PROGMEM __attribute__((section(".progmem.usbcore")))
#define PROGMEM_ORIG __attribute__((__progmem__))

#endif // check for GNUC >= or < 4.6


// number of endpoints - to determine buffer array sizes
// see definition for _initEndpoints (below)
#ifdef CDC_ENABLED
#ifdef HID_ENABLED
#define INTERNAL_NUM_EP 5
#else // HID_ENABLED _not_ defined
#define INTERNAL_NUM_EP 4
#endif // HID_ENABLED
#elif defined(HID_ENABLED)
#define INTERNAL_NUM_EP 2
#else
#define INTERNAL_NUM_EP 1
#endif


//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//    ____  _____  ____   _   _   ____  _____  _   _  ____   _____  ____    //
//   / ___||_   _||  _ \ | | | | / ___||_   _|| | | ||  _ \ | ____|/ ___|   //
//   \___ \  | |  | |_) || | | || |      | |  | | | || |_) ||  _|  \___ \   //
//    ___) | | |  |  _ < | |_| || |___   | |  | |_| ||  _ < | |___  ___) |  //
//   |____/  |_|  |_| \_\ \___/  \____|  |_|   \___/ |_| \_\|_____||____/   //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define INTERNAL_BUFFER_LENGTH 64

typedef struct __INTERNAL_BUFFER__
{
  struct __INTERNAL_BUFFER__ * volatile pNext;
  volatile uint8_t iIndex; // current pointer
  volatile uint8_t iLen;   // max pointer
  // NOTE:  if 'iLen' is zero, the buffer is being filled and 'iIndex' is the length
  //        when the buffer is released to send, iLen gets the length, iIndex is assigned
  //        to 0xff.  when sendING, iIndex is assigned to 0xfe.  On send complete, it's free'd

  uint8_t aBuf[INTERNAL_BUFFER_LENGTH];
} INTERNAL_BUFFER;




// bug in iox128a1u.h and iox64a1u.h
// the definition for USB_TRNCOMPL_vect and USB_TRNCOMPL_vect_num is wrong
// this can be corrected here.  bug reported 'upstream' for avr-libc 1.8.0 and 1.8.1
//   https://savannah.nongnu.org/bugs/index.php?44279

#if defined (__AVR_ATxmega64A1U__) || defined (__AVR_ATxmega128A1U__)
#undef USB_TRNCOMPL_vect
#undef USB_TRNCOMPL_vect_num
#define USB_TRNCOMPL_vect_num  126
#define USB_TRNCOMPL_vect      _VECTOR(126)  /* Transaction complete interrupt */
#endif // __AVR_ATxmega64A1U__, __AVR_ATxmega128A1U__



// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG
#ifdef DEBUG_CODE
extern void DebugOut(unsigned long lVal) __attribute__((weak));
extern void DebugOut(const void * PROGMEM_ORIG pStr) __attribute__((weak));
extern void DebugDumpRegs(const XMegaEPDataStruct * pRegs) __attribute__((weak));

// 'weak link' overloads for debug functions that prevent link fail
extern void DebugOut(unsigned long lVal) { }
extern void DebugOut(const void * PROGMEM_ORIG pStr) { }
extern void DebugDumpRegs(const XMegaEPDataStruct * pRegs) { }
#define DEBUG_OUT(X) DebugOut(X)
#define DEBUG_DUMP_REGS(X) DebugDumpRegs(X)

// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG


#define LED_SIGNAL0 (LED_BUILTIN-2) /* PQ3 */
#define LED_SIGNAL1 (LED_BUILTIN-3) /* PQ2 */
#define LED_SIGNAL2 (LED_BUILTIN-4) /* PQ1 */
#define LED_SIGNAL3 (LED_BUILTIN-5) /* PQ0 */

#else // DEBUG_CODE
#define DEBUG_OUT(X)
#define DEBUG_DUMP_REGS(X)
#endif // DEBUG_CODE
// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG


#ifdef USB_VID
#if USB_VID==null
#error cannot work with NULL value for VID
#endif // USB_VID==null
#else
#error must define USB_VID
#endif // USB_VID

#ifdef USB_PID
#if USB_PID==null
#error cannot work with NULL value for PID
#endif // USB_PID==null
#else
#error must define USB_PID
#endif // USB_PID


#define EP_TYPE_CONTROL        0x00
#define EP_TYPE_BULK_IN        0x81
#define EP_TYPE_BULK_OUT      0x80
#define EP_TYPE_INTERRUPT_IN    0xC1
#define EP_TYPE_INTERRUPT_OUT    0xC0
#define EP_TYPE_ISOCHRONOUS_IN    0x41
#define EP_TYPE_ISOCHRONOUS_OUT    0x40



// NOTE:  auto ZLP is broken according to 128A1U errata
#define ZLP_BIT 0/*(((uint16_t)USB_EP_ZLP_bm)<<8)*/

//==================================================================
//==================================================================

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//         ____  ___   _   _  ____  _____   _     _   _  _____  ____        //
//        / ___|/ _ \ | \ | |/ ___||_   _| / \   | \ | ||_   _|/ ___|       //
//       | |   | | | ||  \| |\___ \  | |  / _ \  |  \| |  | |  \___ \       //
//       | |___| |_| || |\  | ___) | | | / ___ \ | |\  |  | |   ___) |      //
//        \____|\___/ |_| \_||____/  |_|/_/   \_\|_| \_|  |_|  |____/       //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

extern const u16 STRING_LANGUAGE[] PROGMEM;
extern const u16 STRING_IPRODUCT[] PROGMEM;
extern const u16 STRING_IMANUFACTURER[] PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptor PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptorA PROGMEM;

const u16 STRING_LANGUAGE[2] =
{
  (3<<8) | (2+2),
  0x0409  // English
};

const u16 STRING_IPRODUCT[17] =
{
  (3<<8) | (2+2*16),
#if USB_PID == 0x8036  
  'A','r','d','u','i','n','o',' ','L','e','o','n','a','r','d','o'
#elif USB_PID == 0x8037
  'A','r','d','u','i','n','o',' ','M','i','c','r','o',' ',' ',' '
#elif USB_PID == 0x803C
  'A','r','d','u','i','n','o',' ','E','s','p','l','o','r','a',' '
#elif USB_PID == 0x9208
  'L','i','l','y','P','a','d','U','S','B',' ',' ',' ',' ',' ',' '
#elif USB_PID == 0x0010 // added for 'mega' clone (testing only)
  'A','r','d','u','i','n','o',' ','M','e','g','a','2','5','6','0'
#else
  'U','S','B',' ','I','O',' ','B','o','a','r','d',' ',' ',' ',' '
#endif
};

const u16 STRING_IMANUFACTURER[12] =
{
  (3<<8) | (2+2*11),
#if USB_VID == 0x2341
  'A','r','d','u','i','n','o',' ','L','L','C'
#warning using Arduino USB Vendor ID - do NOT ship product with this ID without permission!!!
#elif USB_VID == 0x1b4f
  'S','p','a','r','k','F','u','n',' ',' ',' '
#warning using SparkFun USB Vendor ID - do NOT ship product with this ID without permission!!!
#elif USB_VID == 0x1d50 // Openmoko - see http://wiki.openmoko.org/wiki/USB_Product_IDs
  'O','p','e','n','m','o','k','o',' ',' ',' '
#warning make sure you have obtained a proper product ID from Openmoko - see http://wiki.openmoko.org/wiki/USB_Product_IDs
#else
  'U','n','k','n','o','w','n',' ',' ',' ',' '
#endif
};

#ifdef CDC_ENABLED /* this would be a virtual COM port */
#define DEVICE_CLASS 0x02
#else
#define DEVICE_CLASS 0x00 /* typical HID, most likely */
#endif

//  DEVICE DESCRIPTOR
const DeviceDescriptor USB_DeviceDescriptor PROGMEM =
  D_DEVICE(0x00,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

const DeviceDescriptor USB_DeviceDescriptorA PROGMEM =
  D_DEVICE(DEVICE_CLASS,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);



// TODO:  set up some interrupt vectors via 'EMPTY_INTERRUPT(vector)'
//        to prevent 'whatever' from booting me
//
//        see http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html



//==================================================================
//==================================================================


extern const u8 _initEndpoints[INTERNAL_NUM_EP] PROGMEM; // that's the way they did it before, and I'm not changing it
const u8 _initEndpoints[INTERNAL_NUM_EP] = 
{
  EP_TYPE_CONTROL /*0*/, // EP_TYPE_CONTROL
  
#ifdef CDC_ENABLED
  EP_TYPE_INTERRUPT_IN,    // CDC_ENDPOINT_ACM
  EP_TYPE_BULK_OUT,      // CDC_ENDPOINT_OUT
  EP_TYPE_BULK_IN,      // CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
  EP_TYPE_INTERRUPT_IN    // HID_ENDPOINT_INT
#endif
};


// xmega still uses older defs for these (TODO - update to something better)
#define EP_SINGLE_64 0x32  // EP0
#define EP_DOUBLE_64 0x36  // Other endpoints





//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//               ____  _      ___   ____     _     _      ____              //
//              / ___|| |    / _ \ | __ )   / \   | |    / ___|             //
//             | |  _ | |   | | | ||  _ \  / _ \  | |    \___ \             //
//             | |_| || |___| |_| || |_) |/ ___ \ | |___  ___) |            //
//              \____||_____|\___/ |____//_/   \_\|_____||____/             //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

// linked list of pre-allocated buffers, twice the # of endpoints plus 2
// most endpoints ONLY read OR write.  Only control does both. This lets me have
// 2 read or 2 write buffers per endpoint, perfect for 'ping pong' buffering.
INTERNAL_BUFFER *pFree = NULL;
INTERNAL_BUFFER aBuf[INTERNAL_NUM_EP * 2 + 2];  // twice the # of endpoints plus 2

INTERNAL_BUFFER *aSendQ[INTERNAL_NUM_EP];
INTERNAL_BUFFER *aRecvQ[INTERNAL_NUM_EP];

#define INTERNAL_BUFFER_FILLING(X) ((X)->iLen == 0 && (X)->iIndex < 0xfe) /* buffer filling (or empty) */
#define INTERNAL_BUFFER_SEND_READY(X) ((X)->iIndex == 0xff)               /* ready to send */
#define INTERNAL_BUFFER_MARK_SEND_READY(X) {(X)->iLen = (X)->iIndex; (X)->iIndex = 0xff;}
#define INTERNAL_BUFFER_SENDING(X) ((X)->iIndex == 0xfe)                  /* is 'sending' */
#define INTERNAL_BUFFER_MARK_SENDING(X) {(X)->iIndex = 0xfe;}             /* mark 'sending' */
#define INTERNAL_BUFFER_RECV_EMPTY(X) (!(X)->iIndex && !(X)->iLen)        /* empty receive buffer */
#define INTERNAL_BUFFER_RECV_READY(X) ((X)->iLen > 0)                     /* received data ready */



/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
#ifdef TX_RX_LED_INIT /* only when these are defined */
#define TX_RX_LED_PULSE_MS 100
volatile u8 TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
volatile u8 RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
#endif // TX_RX_LED_INIT

static XMegaEPDataStruct epData; // the data pointer for the hardware's memory registers

// for debugging only - remove later
static uint8_t led_toggle;

#ifdef INIT_EP0_ONE_TIME_ONLY
uint8_t bInitEP0 = 0;
#endif // INIT_EP0_ONE_TIME_ONLY




/////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                             //
//   _                        _   ____               _          _                              //
//  | |     ___    ___  __ _ | | |  _ \  _ __  ___  | |_  ___  | |_  _   _  _ __    ___  ___   //
//  | |    / _ \  / __|/ _` || | | |_) || '__|/ _ \ | __|/ _ \ | __|| | | || '_ \  / _ \/ __|  //
//  | |___| (_) || (__| (_| || | |  __/ | |  | (_) || |_| (_) || |_ | |_| || |_) ||  __/\__ \  //
//  |_____|\___/  \___|\__,_||_| |_|    |_|   \___/  \__|\___/  \__| \__, || .__/  \___||___/  //
//                                                                   |___/ |_|                 //
//                                                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////


static void init_buffers_and_endpoints(void);
static void internal_do_setup_request(INTERNAL_BUFFER *pBuf);
static void internal_do_endpoint_receive(char nEP, INTERNAL_BUFFER *pBuf);

static void InitEP(u8 index, u8 type, u8 size);

static bool SendDescriptor(Setup& rSetup);

static uint16_t buffer_data_pointer(INTERNAL_BUFFER *pBuf);
static INTERNAL_BUFFER * inverse_buffer_data_pointer(uint16_t dataptr);

static void internal_send0(int index);
static void internal_send(int index, const void *pData, uint16_t cbData);
static int internal_receive(int index, void *pData, uint16_t nMax);
static void configure_buffers(void);



//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//              ____   _   _  _____  _____  _____  ____   ____              //
//             | __ ) | | | ||  ___||  ___|| ____||  _ \ / ___|             //
//             |  _ \ | | | || |_   | |_   |  _|  | |_) |\___ \             //
//             | |_) || |_| ||  _|  |  _|  | |___ |  _ <  ___) |            //
//             |____/  \___/ |_|    |_|    |_____||_| \_\|____/             //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static void init_buffers_and_endpoints(void)
{
int index;
uint8_t oldSREG;

  error_printP(F("init_buffers_and_endpoints"));


  oldSREG = SREG;
  cli(); // to avoid spurious interrupts (just in case)
  
  memset(&epData, 0, sizeof(epData));
  for(index=0; index <= MAXEP; index++)
  {
    epData.endpoint[index].out.status = USB_EP_BUSNACK0_bm; // disables I/O
    epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm;  // necessary for 128A1U rev 'K' etc. (see errata)

    epData.endpoint[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)
    epData.endpoint[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)
  }

  configure_buffers(); // TODO:  move code here instead of function call?

  // set up endpoint 0 (always), now that I'm done doing that other stuff
  
  InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);  // init ep0 and zero out the others

  SREG = oldSREG;
}


#ifdef DEBUG_CODE

static char hex_digit(unsigned char b1)
{
  if(b1 < 10)
  {
    return b1 + '0';
  }
  else if(b1 < 16)
  {
    return (b1 - 10) + 'A';
  }

  return '?';
}

static void DumpBuffer(INTERNAL_BUFFER *pBuf)
{
char tbuf[60];
uint8_t *pD;
char *pOut, nCol, c1;
short iCtr;

  error_printP_(F("Dump of buffer "));
  error_printL((uint16_t)pBuf);
  error_printP_(F("  iIndex="));
  error_printL_(pBuf->iIndex);
  error_printP_(F("  iLen="));
  error_printL_(pBuf->iLen);
  error_printP_(F("  pNext="));
  error_printL((uint16_t)pBuf->pNext);
  error_printP(F("   --------------------------------------"));

  pOut = tbuf;
  *pOut = 0;

  for(iCtr=0, nCol=0, pD = pBuf->aBuf; pD < &(pBuf->aBuf[sizeof(pBuf->aBuf)]) && iCtr < pBuf->iLen; iCtr++, pD++)
  {
    if(!nCol)
    {
      pOut = tbuf;

      if(iCtr > 0)
      {
        error_print(tbuf);
      }

      *(pOut++) = hex_digit((iCtr >> 4) & 0xf);
      *(pOut++) = hex_digit(iCtr & 0xf);
      *(pOut++) = ':';
    }
    else
    {
      *(pOut++) = ' ';
    }

    c1 = *pD;

    *(pOut++) = hex_digit((c1 >> 4) & 0xf);
    *(pOut++) = hex_digit(c1 & 0xf);
    *pOut = 0; // always do this

    if(nCol < 15)
    {
      nCol++;
    }
    else
    {
      nCol = 0; // starts new line
    }
  }

  if(tbuf[0])
  {
    error_print(tbuf);
  }


}

#endif // DEBUG_CODE


static INTERNAL_BUFFER *end_of_chain(INTERNAL_BUFFER *pBuf)
{
uint8_t oldSREG;
register INTERNAL_BUFFER *pRval;


  if(!pBuf)
  {
    error_printP_(F("WARN:  pBuf is NULL"));
    return NULL;
  }

  oldSREG = SREG; // save int flag
  cli(); // locking resource

  pRval = pBuf;

  if(pRval < &(aBuf[0]) || pRval >= &(aBuf[INTERNAL_NUM_EP * 2 + 2]))
  {
    error_printP_(F("WARN:  pBuf out of bounds "));
    error_printL((unsigned long)pBuf);

    SREG = oldSREG;
    return NULL;
  }

  while(pRval->pNext)
  {
    pRval = pRval->pNext;

    if(pRval < &(aBuf[0]) || pRval >= &(aBuf[INTERNAL_NUM_EP * 2 + 2]))
    {
      error_printP_(F("WARN:  pRval out of bounds "));
      error_printL((unsigned long)pRval);

      SREG = oldSREG;
      return NULL;
    }
  }

  SREG = oldSREG;

  if(pRval >= &(aBuf[0]) && pRval < &(aBuf[INTERNAL_NUM_EP * 2 + 2]))
  {
    return pRval;
  }

  error_printP(F("WARN:  end_of_chain - corrupt buffer chain, returns NULL"));

  return NULL;
}

// simple pre-allocated buffer management
static void configure_buffers(void)
{
int i1;

  error_printP(F("configure_buffers"));

  memset(aBuf, 0, sizeof(aBuf));

  for(i1=1; i1 < INTERNAL_NUM_EP * 2 + 2; i1++)
  {
    aBuf[i1 - 1].pNext = &(aBuf[i1]);
  }

  // tail gets NULL for 'pNext'
  aBuf[INTERNAL_NUM_EP * 2 + 1].pNext = NULL;

  // head of free list is first entry
  pFree = &(aBuf[0]);

  // send and receive queues
  for(i1=0; i1 < INTERNAL_NUM_EP; i1++)
  {
    aSendQ[i1] = NULL;
    aRecvQ[i1] = NULL;
  }
}

// this function peels a pointer from the 'free' list and returns it.
// caller must call 'free_buffer' with returned pointer when done with it
// and *NOT* re-use it after calling 'free_buffer'.
static INTERNAL_BUFFER * next_buffer(void)
{
INTERNAL_BUFFER *pRval;
uint8_t oldSREG;

  error_printP(F("next_buffer"));

  oldSREG = SREG; // save int flag
  cli(); // locking resource

  pRval = pFree;

  if(pRval) // non-NULL return
  {
    pFree = pRval->pNext;
    pRval->pNext = NULL;
  }

  SREG = oldSREG;

  if(pRval < &(aBuf[0]) || pRval >= &(aBuf[INTERNAL_NUM_EP * 2 + 2]))
  {
    error_printP_(F("WARN:  pRval out of bounds "));
    error_printL((unsigned long)pRval);
  }

  return pRval; // now it belongs to the caller
}

static void free_buffer(INTERNAL_BUFFER *pBuf)
{
  // simple sanity test, must be in range ('address valid' could be tested as well)
  if(pBuf >= &(aBuf[0]) && pBuf < &(aBuf[INTERNAL_NUM_EP * 2 + 2]))
  {
    uint8_t oldSREG = SREG; // save int flag
    cli(); // locking resource

    pBuf->pNext = pFree;
    pFree = pBuf;

    SREG = oldSREG;
  }
}

static void free_queue(INTERNAL_BUFFER *pBuf)
{
INTERNAL_BUFFER *pE;
uint8_t oldSREG;

  if(!pBuf)
  {
    return;
  }

  error_printP(F("free_queue"));

  oldSREG = SREG; // save int flag
  cli(); // locking resource

  do
  {
    pE = pBuf;
    // walk to the end
    while(pE->pNext && pE->pNext->pNext)
    {
      pE = pE->pNext;
    }

    // if not NULL, free it
    if(pE->pNext)
    {
      free_buffer(pE->pNext);
      pE->pNext = NULL;
    }    

    pE = pE->pNext;
  } while(pE);

  free_buffer(pBuf);  

  SREG = oldSREG;
}

static void add_to_queue(INTERNAL_BUFFER **pQ, INTERNAL_BUFFER *pBuf)
{
INTERNAL_BUFFER *pE;
uint8_t oldSREG;

  if(!pQ || !pBuf)
  {
    return;
  }

  error_printP(F("add_to_queue"));

  oldSREG = SREG; // save int flag
  cli(); // locking resource

  pE = *pQ;

  if(!pE)
  {
    *pQ = pBuf;
  }
  else
  {
    // walk to the end
    while(pE->pNext)
    {
      pE = pE->pNext;
    }

    // attach
    pE->pNext = pBuf;
  }

  SREG = oldSREG;
}

static void remove_from_queue(INTERNAL_BUFFER **pQ, INTERNAL_BUFFER *pBuf)
{
INTERNAL_BUFFER *pE;
uint8_t oldSREG;


  if(!pQ || !pBuf)
  {
    return;
  }

  error_printP(F("remove_from_queue"));

  oldSREG = SREG; // save int flag
  cli(); // locking resource

  pE = *pQ;

  if(pE) // if not NULL [allow for problems, delete buffer anyway]
  {
    if(pBuf == pE)
    {
      *pQ = pBuf->pNext;
    }
    else
    {
      while(pE && pE->pNext != pBuf)
      {
        pE = pE->pNext; // walk the chain
      }

      if(pE && pE->pNext == pBuf)
      {
        pE->pNext = pBuf->pNext; // remove 'pBuf' from the chain
      }
    }
  }

  pBuf->pNext = NULL; // always
  free_buffer(pBuf);  // and NOW 'pBuf' is back in the 'free' pool

  SREG = oldSREG;
}

static uint16_t buffer_data_pointer(INTERNAL_BUFFER *pBuf)
{
  if(!pBuf)
  {
    return 0;
  }

  return (uint16_t)&(pBuf->aBuf[0]); // assign 'dataptr' element in USB Endpoint descriptor (sect 20.15 in 'AU' manual)
}

static INTERNAL_BUFFER * inverse_buffer_data_pointer(uint16_t dataptr)
{
  if(!dataptr)
  {
    return NULL;
  }

  dataptr -= (uint16_t)&(((INTERNAL_BUFFER *)0)->aBuf[0]);

  return (INTERNAL_BUFFER *)dataptr;
}


//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//       ___                              __  __                    _       //
//      / _ \  _   _   ___  _   _   ___  |  \/  |  __ _  _ __ ___  | |_     //
//     | | | || | | | / _ \| | | | / _ \ | |\/| | / _` || '_ ` _ \ | __|    //
//     | |_| || |_| ||  __/| |_| ||  __/ | |  | || (_| || | | | | || |_     //
//      \__\_\ \__,_| \___| \__,_| \___| |_|  |_| \__, ||_| |_| |_| \__|    //
//                                                |___/                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////


static void set_up_EP_for_receive(int index)
{
INTERNAL_BUFFER *pBuf;

  // assumed interrupts are OFF, setting up single endpoint to receive data
  // with a fresh buffer added to the queue.  existing buffers in recv queue ignored.

//  error_printP(F("set_up_EP_for_receive"));

  // queue up the next one if there is one
  pBuf = aRecvQ[index];
  
  while(pBuf)
  {
    if(INTERNAL_BUFFER_RECV_EMPTY(pBuf)) // find empty buffer already in queue
    {
      break;
    }

    pBuf = pBuf->pNext;
  }

  if(!pBuf)
  {
    pBuf = next_buffer();

    if(pBuf)
    {
      pBuf->iIndex = pBuf->iLen = 0; // make sure it's empty
      add_to_queue(&(aRecvQ[index]), pBuf);
    }
  }

  if(pBuf)
  {
    epData.endpoint[index].out.dataptr = buffer_data_pointer(pBuf); //(uint16_t)pBuf;
    epData.endpoint[index].out.status = 0; // this allows receive data
    epData.endpoint[index].out.status |= USB_EP_OVF_bm; // just turn it off [for now]
    epData.endpoint[index].out.status &= ~USB_EP_OVF_bm; // just turn it off [for now]
  }
}

static void check_recv_queue(void)
{
int index;
INTERNAL_BUFFER *pE, *pEtmp;
uint8_t oldSREG;


  oldSREG = SREG;
  cli(); // this must be done with interrupts blocked

  for(index=0; index <= MAXEP; index++)
  {
    pE = inverse_buffer_data_pointer(epData.endpoint[index].out.dataptr);

    // skip disabled endpoints
    if((epData.endpoint[index].out.ctrl & USB_EP_TYPE_gm) == USB_EP_TYPE_DISABLE_gc)
    {
      continue; // endpoint disabled
    }

    // check for an endpoint that has 'BUSNACK0' set
    if((!pE || (epData.endpoint[index].out.status & USB_EP_BUSNACK0_bm)) && // NOT receiving at the moment
       !aRecvQ[index]) // no new buffers to attach
    {
//      error_printP(F("check_recv_queue - USB_EP_BUSNACK0 [skipping]"));
      continue;  // stalled [TODO:  enable if I have a buffer in 'aRecvQ' for it?]
    }

    // check for errors before anything else
    if(epData.endpoint[index].out.status & USB_EP_OVF_bm) // underflow
    {
      // first, handle "sent" transaction
      error_printP_(F("check_recv_queue "));
      error_printL_(index);
      error_printP_(F(" - USB_EP_OVF"));

      epData.endpoint[index].out.status |= USB_EP_OVF_bm; // just turn it off [for now]
      epData.endpoint[index].out.status &= ~USB_EP_OVF_bm; // just turn it off [for now]
    }

    if(epData.endpoint[index].in.status & USB_EP_SETUP_bm)
    {
      error_printP_(F("check_recv_queue "));
      error_printL_(index);
      error_printP(F(" - USB_EP_SETUP"));

      // for now, just do nothing
    }

    if(pE) // && !(epData.endpoint[index].out.status & USB_EP_BUSNACK0_bm)) // AM receiving at the moment
    {
      // check for received data.  If received, set up length and other info, and
      // make sure it's at the end of the receive queue.  THEN set up another buffer
      // for it automatically, except for endpoint 0.  'buffer combining' might be good
      // as well, if possible.

      if((epData.endpoint[index].out.status & USB_EP_TRNCOMPL0_bm) // note:  'SETUP' is TRNCOMPL1, as needed
         || (!index && (epData.endpoint[index].out.status & USB_EP_SETUP_bm)))
      {
        error_printP_(F("check_recv_queue "));
        error_printL_(index);
        error_printP_(F(" USB_EP_TRNCOMPL0_bm "));
        error_printL_(index);
        error_printP_(F(": status="));
        error_printL_(epData.endpoint[index].out.status);
        error_printP_(F(" length="));
        error_printL_(epData.endpoint[index].out.cnt);
        error_printP_(F(" address="));
        error_printL(epData.endpoint[index].out.dataptr);

        // ASSERT( epData.endpoint[index].out.dataptr == buffer_data_pointer(aRecvQ[index]) );

            //(INTERNAL_BUFFER *)(void *)(epData.endpoint[index].out.dataptr);  <-- wrong

        // TODO:  verify pE is valid AND/OR a part of the chain

        if(pE /* and PE is valid */)
        {
          pE->iIndex = 0;
          pE->iLen = epData.endpoint[index].out.cnt;

          if(!pE->iLen) // empty packet - re-use it?
          {
            pE->iLen = 1;
            pE->iIndex = 1; // mark as 'no data', basically
          }

//          DumpBuffer(pE);
        }

        epData.endpoint[index].out.status |= USB_EP_BUSNACK0_bm; // mark 'do not receive' (temporarily stalled)
        epData.endpoint[index].out.dataptr = 0;
        epData.endpoint[index].out.cnt = 0;
        epData.endpoint[index].out.status &= ~USB_EP_TRNCOMPL0_bm;

        if(index == 0 && pE &&
           epData.endpoint[0].in.status & USB_EP_SETUP_bm)
        {
          if(pE->iLen)
          {
            internal_do_setup_request(pE); // always endpoint 0
          }

          // remove it from the queue
          remove_from_queue(&(aRecvQ[0]), pE); // removes from queue and deletes buffer
        }
        else if(pE)
        {
          if(pE->iLen)
          {
            internal_do_endpoint_receive(index, pE);
          }

          // remove it from the queue
          remove_from_queue(&(aRecvQ[0]), pE); // removes from queue and deletes buffer
        }

        epData.endpoint[index].out.status &= ~USB_EP_SETUP_bm;
      }

      set_up_EP_for_receive(index); // set up to receive
    }
  }

  SREG = oldSREG;
}

static void check_send_queue(void)
{
int index;
INTERNAL_BUFFER *pX;
uint8_t oldSREG;


  oldSREG = SREG;
  cli(); // this must be done with interrupts blocked

  for(index=0; index <= MAXEP; index++)
  {
    pX = inverse_buffer_data_pointer(epData.endpoint[index].in.dataptr);
    uint8_t status = epData.endpoint[index].in.status;

    // skip those not enabled
    if((epData.endpoint[index].in.ctrl & USB_EP_TYPE_gm) == USB_EP_TYPE_DISABLE_gc)
    {
      continue;
    }

    // check for errors before anything else

    if((status & USB_EP_STALL_bm) || // stall
       (status & USB_EP_UNF_bm))     // underflow
    {
      epData.endpoint[index].in.status |= USB_EP_STALL_bm; // to clear the bit

      if(!pX)
      {
        epData.endpoint[index].in.status &= USB_EP_TOGGLE_bm; // clear everything but this bit
      }
      else
      {
        epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm // mark 'do not send'
                                         | (epData.endpoint[index].in.status & USB_EP_TOGGLE_bm);
      }

      continue;
    }

    if(pX && 
       ((epData.endpoint[index].in.status & USB_EP_TRNCOMPL0_bm) // note:  'SETUP' is TRNCOMPL1, as needed
        || (!index && (epData.endpoint[index].in.status & USB_EP_SETUP_bm))))
    {
      error_printP_(F("check_send_queue "));
      error_printL_(index);
      error_printP_(F(" - USB_EP_TRNCOMPL0 "));
      error_printL_(index);
      error_printP_(F(": status="));
      error_printL_(epData.endpoint[index].in.status);
      error_printP_(F(" length="));
      error_printL_(epData.endpoint[index].in.cnt);
      error_printP_(F(" address="));
      error_printL(epData.endpoint[index].in.dataptr);

//      // TEMPORARY - dump its contents for reference
//      uint8_t oldLen = pX->iLen;
//      pX->iLen = epData.endpoint[index].in.cnt;
//      DumpBuffer(pX);
//      pX->iLen = oldLen;

      // ASSERT( epData.endpoint[index].in.dataptr == buffer_data_pointer(aSendQ[index]) );

      epData.endpoint[index].in.status |= USB_EP_BUSNACK0_bm; // mark 'do not send'
      epData.endpoint[index].in.cnt = 0;
      epData.endpoint[index].in.dataptr = 0;

      epData.endpoint[index].in.status &= ~(USB_EP_TRNCOMPL0_bm | // turn 'send complete' bit off
                                            USB_EP_SETUP_bm);     // turn 'setup complete' bit off

      epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm; // mark 'do not send'

      error_printP_(F("check_send_queue "));
      error_printL_(index);
      error_printP_(F(" Status is now "));
      error_printL(epData.endpoint[index].in.status);

      // once a buffer has been SENT, I can remove it from the send queue as 'completed'.

      if(INTERNAL_BUFFER_SENDING(pX)) // for now always check + remove the first one (TODO:  double-check chain)
      {
        // NOTE:  buffer is marked 'sending' so I don't remove it or alter it until after it's "sent"

        remove_from_queue(&(aSendQ[index]), pX);
      }
    }
  }

  for(index=0; index <= MAXEP; index++)
  {
    pX = aSendQ[index];

    // NOW, send any data that's waiting

    if(pX &&
       ((epData.endpoint[index].in.status & USB_EP_BUSNACK0_bm) ||  // NOT sending at the moment
        epData.endpoint[index].in.dataptr == 0))
    {
      if(INTERNAL_BUFFER_SEND_READY(pX)) // ready to send
      {
        error_printP_(F("check_send_queue "));
        error_printL_(index);
        error_printP_(F(" - attach buffer: "));
        error_printL_(buffer_data_pointer(aSendQ[index]));
        error_printP_(F(" address="));
        error_printL_(buffer_data_pointer(aSendQ[index]));
        error_printP_(F(" length="));
        error_printL_(aSendQ[index]->iLen);        

        epData.endpoint[index].in.cnt = pX->iLen;
        epData.endpoint[index].in.dataptr = buffer_data_pointer(pX);

        INTERNAL_BUFFER_MARK_SENDING(pX);
        epData.endpoint[index].in.status |= USB_EP_UNF_bm; // just turn it off [for now]
        epData.endpoint[index].in.status &= ~USB_EP_UNF_bm; // just turn it off [for now]

        epData.endpoint[index].in.status = USB_EP_TOGGLE_bm; // send (for some reason I need to set this)

        // also turn off 'underflow' flag (doing it twice to make sure, experiment to see what's right)

        error_printP_(F(" status="));
        error_printL(epData.endpoint[index].in.status);
      }
    }
  }

  SREG = oldSREG;
}

// queue up "zero length" send data
static void internal_send0(int index)
{
INTERNAL_BUFFER *pB;

  error_printP_(F("internal_send0 "));
  error_printL(index);

  pB = next_buffer();

  if(pB)
  {
    pB->iIndex = pB->iLen = 0; // zero-length

    add_to_queue(&(aSendQ[index]), pB);
    INTERNAL_BUFFER_MARK_SEND_READY(pB); // mark 'ready to send' so that it goes out

    check_send_queue();
  }
}

// queue up send data
static void internal_send(int index, const void *pData, uint16_t cbData)
{
INTERNAL_BUFFER *pB;
const uint8_t *pD = (const uint8_t *)pData;


  if(!cbData)
  {
    error_printP_(F("internal_send "));
    error_printL_(index);
    error_printP(F(" - calling internal_send0"));
    internal_send0(index);
    return;
  }

  if(!pData)
  {
    error_printP_(F("internal_send "));
    error_printL_(index);
    error_printP(F(" - NULL pointer"));
    return;
  }
  
  error_printP_(F("internal_send "));
  error_printL_(index);
  error_printP_(F(" - "));
  error_printL_(cbData);
  error_printP(F(" bytes"));

  pB = next_buffer();

  while(pB)
  {
    register uint8_t cb;
    
    if(cbData > sizeof(pB->aBuf))
    {
      cb = sizeof(pB->aBuf);
    }
    else
    {
      cb = cbData;
    }

    pB->iLen = pB->iIndex = cb; // assign these to the same value ('send ready' will use it)

    if(cb) // just in case, should never be zero here
    {
      memcpy(&(pB->aBuf[0]), pD, cb);
    }

    pD += cb;
    cbData -= cb;

    add_to_queue(&(aSendQ[index]), pB);
    INTERNAL_BUFFER_MARK_SEND_READY(pB); // mark 'ready to send' so that it goes out

    if(!cbData) // this ends the loop.  0-length data sends 1 packet.
    {
      check_send_queue(); // to force a write

      return; // done!
    }

    pB = next_buffer();
  }

  // if I get here, pB is NULL

  error_printP_(F("internal_send "));
  error_printL_(index);
  error_printP(F(" - NULL buffer"));

}

// byte-level receive
static int internal_receive(int index, void *pData, uint16_t nMax)
{
INTERNAL_BUFFER *pB;
uint8_t oldSREG;
int iRval = 0;
uint8_t *pD = (uint8_t *)pData;


  error_printP_(F("internal_receive "));
  error_printL(index);

  check_recv_queue();

  // this manipulates buffers, so stop ints temporarily

  oldSREG = SREG;
  cli(); // disable interrupts

  pB = aRecvQ[index]; // first buffer
  while(pB && nMax && INTERNAL_BUFFER_RECV_READY(pB))
  {
    uint8_t cb = pB->iLen - pB->iIndex;
    if(cb > 0)
    {
      if(cb > nMax)
      {
        cb = nMax;
      }

      if(pData)
      {
        memcpy(pD, &(pB->aBuf[pB->iIndex]), cb);
      }

      nMax -= cb;
      pD += cb;
      pB->iIndex += cb;

      iRval += cb;
    }

    if(pB->iIndex >= pB->iLen)
    {
      // get next packet
      aRecvQ[index] = pB->pNext;
      pB->pNext = NULL;
      free_buffer(pB);

      if(!index) // for control, don't exceed packet boundary
      {
        break;
      }

      pB = aRecvQ[index];
    }
  }

  SREG = oldSREG;
  return iRval;
}


//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//           _____             _                _         _                 //
//          | ____| _ __    __| | _ __    ___  (_) _ __  | |_  ___          //
//          |  _|  | '_ \  / _` || '_ \  / _ \ | || '_ \ | __|/ __|         //
//          | |___ | | | || (_| || |_) || (_) || || | | || |_ \__ \         //
//          |_____||_| |_| \__,_|| .__/  \___/ |_||_| |_| \__||___/         //
//                               |_|                                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

uint16_t endpoint_data_pointer(void)
{
  return (uint16_t)(uint8_t *)&(epData.endpoint[0]); // set 'EP Data' pointer to THIS address
}


void InitEP(u8 index, u8 type, u8 size)
{
uint8_t oldSREG;
int i1;


//  DEBUG_OUT(F("USB InitEP "));
//  DEBUG_OUT((uint32_t)index);
//  DEBUG_OUT(F("\r\n"));
  error_printP_(F("USB InitEP "));
  error_printL(index);

//  UENUM = index;     index, allows other bits to apply to correct endpoing (0-6, 7 not allowed)
//  UECONX = 1;        only sets 'EPEN' bit, aka 'endpoint enable'
//  UECFG0X = type;    bits 7:6 == 'EPTYPE' - 00=control, 01=isochronous, 02=bulk, 03=interrupt
//  UECFG1X = size;    8, 16, 32, 64 etc. - see 32u4 manual pg 267

  if(index > MAXEP || index >= INTERNAL_NUM_EP)  // MAXEP or INTERNAL_NUM_EP will be the highest index (MAXEP is inclusive)
  {
    return;
  } 

  // IMPORTANT:  the definition of 'in' and 'out' are from the perspective of the USB HOST
  //             Since I'm on 'the other end', 'in' writes data, 'out' receives it

  oldSREG = SREG;
  cli(); // disable interrupts

  // NOTE:  this code is based on my research into the documentation (inadequate) and the ATMel Studio
  //        sample project (somewhat difficult to follow) after spending a couple of weeks or so frustratingly
  //        attempting to use ONLY the information found in the 'AU' manual, which SHOULD be enough (but was not).
  //        In particular the behavior CAUSED by the 'NACK0' flag, and the requirement to set it on 'in' endpoints
  //        upon initialization for 128A1U rev K or earlier was NOT obvious, nor even mentioned as far as I know.

#ifdef INIT_EP0_ONE_TIME_ONLY
  if(!bInitEP0 || index != 0 || type != EP_TYPE_CONTROL)
#endif // INIT_EP0_ONE_TIME_ONLY
  {
    // if the endpoint is in the middle of something, this will 'cancel' it
    // NOTE: 'BUSNACK0' is needed by 'in' on 128A1U rev K or earlier [a bug, apparently] - leave it on
    epData.endpoint[index].out.status |= USB_EP_BUSNACK0_bm;
    epData.endpoint[index].in.status |= USB_EP_BUSNACK0_bm;

    // zero out the 'aEPBuff' structure entry right away
    // as well as the 'endpoint' structures
    memset(&(epData.endpoint[index]), 0, sizeof(epData.endpoint[0]));

    epData.endpoint[index].out.status = USB_EP_BUSNACK0_bm;
    epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm;

    // disable the endpoints
    epData.endpoint[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)
    epData.endpoint[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)

    epData.endpoint[index].in.cnt = 0;
    epData.endpoint[index].out.cnt = 0;

    epData.endpoint[index].in.dataptr = 0;
    epData.endpoint[index].out.dataptr = 0;

    epData.endpoint[index].in.auxdata = 0;
    epData.endpoint[index].out.auxdata = 0;

    if(/*epData.endpoint[index].in.dataptr ==*/ aSendQ[index])
    {
      free_queue(aSendQ[index]);
      aSendQ[index] = NULL;
    }
    if(/*epData.endpoint[index].out.dataptr ==*/ aRecvQ[index])
    {
      free_queue(aRecvQ[index]);
      aRecvQ[index] = NULL;
    }
  }

  if(index == 0 && type == EP_TYPE_CONTROL) // control (these can receive SETUP requests)
  {
#ifdef INIT_EP0_ONE_TIME_ONLY
    if(!bInitEP0)
#endif // INIT_EP0_ONE_TIME_ONLY
    {
#ifdef INIT_EP0_ONE_TIME_ONLY
      bInitEP0 = 1;
#endif // INIT_EP0_ONE_TIME_ONLY

      // aBuf1 is output, aBuf2 is input

      aRecvQ[0] = next_buffer();  // allocate buffer

      error_printP_(F("TEMP:  initialize aRecvQ "));
      error_printL_(index);
      error_printP_(F(" with "));
      error_printL((uint32_t)aRecvQ[0]);

      epData.endpoint[0].out.dataptr = buffer_data_pointer(aRecvQ[index]);

      // NOTE:  size will be sent as 'EP_SINGLE_64'

      // TODO:  do I do 'in' as well, or just 'out'?  'out' receives... 'in' sends
      epData.endpoint[0].in.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                                 | USB_EP_SIZE_64_gc;//(size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */

      epData.endpoint[0].out.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                                  | USB_EP_SIZE_64_gc;//(size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */

      // NOTE: 'BUSNACK0' is needed by 'in' on 128A1U rev K or earlier [a bug, apparently, see errata]
      epData.endpoint[0].in.status = USB_EP_BUSNACK0_bm; // leave 'BUSNACK0' bit ON (stalls sending data)
      epData.endpoint[0].out.status = 0; // make sure they're ready to go (this allows receive data)

      // zero out the rest of them, leaving ONLY the control
      for(i1=1; i1 <= MAXEP; i1++)
      {
        epData.endpoint[i1].in.status = USB_EP_BUSNACK0_bm; // make sure (stall sending)
        epData.endpoint[i1].out.status = USB_EP_BUSNACK0_bm; // make sure (stall receiving, cancels 'whatever')

        epData.endpoint[i1].in.ctrl = USB_EP_TYPE_DISABLE_gc;
        epData.endpoint[i1].out.ctrl = USB_EP_TYPE_DISABLE_gc;

        epData.endpoint[i1].in.dataptr = 0;
        epData.endpoint[i1].in.auxdata = 0;
        epData.endpoint[i1].in.cnt = 0;

        epData.endpoint[i1].out.dataptr = 0;
        epData.endpoint[i1].out.auxdata = 0;
        epData.endpoint[i1].out.cnt = 0;

        if(i1 < INTERNAL_NUM_EP)
        {
          if(aSendQ[i1])
          {
            free_queue(aSendQ[i1]);
            aSendQ[i1] = NULL;
          }
          if(aRecvQ[i1])
          {
            free_queue(aRecvQ[i1]);
            aRecvQ[i1] = NULL;
          }
        }
      }
    }
  }
//  else if(index == 0)
//  {
//    DEBUG_OUT(F("USB Incorrect EP type "));
//    DEBUG_OUT((uint16_t)type);
//    DEBUG_OUT(F("\r\n"));
//  }
  else if(type == EP_TYPE_INTERRUPT_IN || type == EP_TYPE_BULK_IN
          || type == EP_TYPE_ISOCHRONOUS_IN) /* these types have *ME* write data and send to 'in' for host */
  {
    // 'in' is actually the WRITE/SEND function
    epData.endpoint[index].in.dataptr = 0;
    epData.endpoint[index].in.auxdata = 0;

    epData.endpoint[index].in.cnt = 0; // broken ZLP_BIT; // no data (so I won't send) plus 'auto-zero-length packet'

    epData.endpoint[index].in.ctrl = (type == EP_TYPE_ISOCHRONOUS_IN ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                                   | (type == EP_TYPE_BULK_IN ? USB_EP_INTDSBL_bm : 0)       /* disable interrupt */
                                   | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc :             // TODO:  set 'double buffer' flag?
                                      size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);         /* data size */

    epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm; // leave 'BUSNACK0' bit ON (stalls sending data)
  }
  else if(type == EP_TYPE_INTERRUPT_OUT || type == EP_TYPE_BULK_OUT /* these send *ME* data */
          || type == EP_TYPE_ISOCHRONOUS_OUT)
  {
    // 'out' is actually the RECEIVE function

    aRecvQ[index] = next_buffer();  // allocate buffer
    epData.endpoint[index].out.dataptr = buffer_data_pointer(aRecvQ[index]);

    epData.endpoint[index].out.auxdata = 0;

    epData.endpoint[index].out.cnt = 0; // no data (so I can receive)

    epData.endpoint[index].out.ctrl = (type == EP_TYPE_ISOCHRONOUS_OUT ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                                    | (type == EP_TYPE_BULK_OUT ? USB_EP_INTDSBL_bm : 0)      /* disable interrupt */
                                    | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc :             // TODO:  set 'double buffer' flag?
                                       size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);         /* data size */

    epData.endpoint[index].out.status = 0; // this allows receive data
  }
  // TODO:  'INOUT' types?
  else
  {
    // endpoint 'disabled' now
  }

  SREG = oldSREG; // restore interrupts (etc.)
  
  // TODO:  anything else?

//  DEBUG_OUT(F("USB InitEP "));
//  DEBUG_OUT((uint32_t)index);
//  DEBUG_OUT(F(" (done!)\r\n"));
  error_printP_(F("USB InitEP "));
  error_printL_(index);
  error_printP(F(" (done!)"));
}

void internal_do_endpoint_receive(char nEP, INTERNAL_BUFFER *pBuf)
{
  error_printP_(F("internal_do_endpoint receive "));
  error_printL(nEP);


  DumpBuffer(pBuf); // TEMPORARILY do this


}

void internal_do_setup_request(INTERNAL_BUFFER *pBuf)
{
INTERNAL_BUFFER *pX;
uint8_t requestType;

  while(pBuf && (pBuf->iLen - pBuf->iIndex) >= 8) // TODO:  does looping on this help?
  {
    Setup *pSetup = (Setup *)&(pBuf->aBuf[pBuf->iIndex]);
    uint8_t ok = 0;

    pBuf->iIndex += 8; // add the # of bytes I have read thus far

// TEMPORARY
//    DEBUG_OUT(F("USB setup "));
//    DEBUG_OUT((uint32_t)pSetup->bRequest);
//    DEBUG_OUT(F(","));
//    DEBUG_OUT((uint32_t)pSetup->bmRequestType);
//    DEBUG_OUT(F(","));
//    DEBUG_OUT((uint16_t)pSetup->wValueH);
//    DEBUG_OUT(F(":"));
//    DEBUG_OUT((uint16_t)pSetup->wValueL);
//    DEBUG_OUT(F(","));
//    DEBUG_OUT((uint16_t)pSetup->wLength);
//    DEBUG_OUT(F("\r\n"));

    error_printP_(F("USB setup "));
    error_printL_(pSetup->bRequest);
    error_printP_(F(","));
    error_printL_(pSetup->bmRequestType);
    error_printP_(F(","));
    error_printL_(pSetup->wValueH);
    error_printP_(F(":"));
    error_printL_(pSetup->wValueL);
    error_printP_(F(","));
    error_printL(pSetup->wLength);

  // typically get '5,0,0:n:0' first [set address to 'n']
  // then get 6,128,1:0,8 - GET_DESCRIPTOR, REQUEST_DEVICETOHOST, USB_DEVICE_DESCRIPTOR_TYPE, 8
  //    (does that have a 2nd 8 byte request after it????)


//  DEBUG_OUT(F("USB setup (2)\r\n"));

    requestType = pSetup->bmRequestType;

    if (requestType & REQUEST_DEVICETOHOST)
    {
//      WaitIN(); // not needed now?
      error_printP(F("dev to host"));

      // TODO:  will there be multiple requests in the buffer?
    }
    else
    {
//      ClearIN(); don't really need THIS now either [I can just ignore the rest of the packet]
      error_printP(F("eat packet"));

      pBuf->iIndex = pBuf->iLen; // this eats the remainder of the packet
    }


    if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
    {
      //  Standard Requests
      uint8_t r;

      error_printP_(F("standard request "));
      error_printL(pSetup->bRequest);

      r = pSetup->bRequest;

      if (GET_STATUS == r)
      {
        error_printP(F("get status"));

        // send a 2-byte packet with 2 zero bytes in it
        pX = next_buffer();

        if(pX)
        {
          pX->iLen = pX->iIndex = 2;
          pX->aBuf[0] = 0;
          pX->aBuf[1] = 0;

          INTERNAL_BUFFER_MARK_SEND_READY(pX);

          add_to_queue(&aSendQ[0], pX);
          check_send_queue(); // transmit available packets
        }
      }
      else if (CLEAR_FEATURE == r)
      {
        error_printP(F("clear feature"));

        ok = true; // temporary        
      }
      else if (SET_FEATURE == r)
      {
        error_printP(F("set feature"));

        ok = true; // temporary        
      }
      else if (SET_ADDRESS == r) // 5
      {
        error_printP_(F("set address ="));
        error_printL(pSetup->wValueL & 0x7f);

        epData.endpoint[0].in.status &= ~(USB_EP_SETUP_bm | USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm | USB_EP_OVF_bm);
        epData.endpoint[0].in.status |= USB_EP_TOGGLE_bm; // sample 'Endpoint_ClearSETUP()' does this
        epData.endpoint[0].out.status |= USB_EP_TOGGLE_bm; // sample 'Endpoint_ClearSETUP()' does this
        
        // TODO:  do I asynchronously wait for something to happen??

        // *SET* *THE* *USB* *ADDRESS* (this has been verified, though timing may be wrong)

        USB_ADDR = pSetup->wValueL & 0x7f;

        ok = true;
      }
      else if (GET_DESCRIPTOR == r) // 6
      {
        error_printP(F("get descriptor"));

        ok = SendDescriptor(*pSetup);  // TODO POOBAH FIX THIS
      }
      else if (SET_DESCRIPTOR == r)
      {
        error_printP(F("set descriptor"));

        ok = false;
      }
      else if (GET_CONFIGURATION == r)
      {
        error_printP(F("get config"));

        // send a 1-byte packet with a 1 byte in it
        pX = next_buffer();

        if(pX)
        {
          pX->iLen = pX->iIndex = 1;
          pX->aBuf[0] = 1;

          INTERNAL_BUFFER_MARK_SEND_READY(pX);

          add_to_queue(&aSendQ[0], pX);
          check_send_queue(); // transmit available packets
        }

        ok = true;
      }
      else if (SET_CONFIGURATION == r)
      {
        error_printP(F("set config"));

        if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
        {
          error_printP(F("request device"));

          // SEE SECTION 20.3 in 'AU' manual for sequence of operation

          error_printP(F("USB InitEndpoints")); // was 'InitEndpoints()'

          // init the first one as a control input
          for (uint8_t i = 1; i < sizeof(_initEndpoints); i++)
          {
            InitEP(i, pgm_read_byte(_initEndpoints+i), EP_DOUBLE_64); // NOTE:  EP_DOUBLE_64 allocates a 'double bank' of 64 bytes, with 64 byte max length
          }

//          _usbConfiguration = pSetup->wValueL;  TODO POOBAH FIX THIS

          ok = true;
        }
        else
        {
          ok = false;
        }
      }
      else if (GET_INTERFACE == r)
      {
        error_printP(F("get interface"));

        ok = true; // TEMPORARY
      }
      else if (SET_INTERFACE == r)
      {
        error_printP(F("set interface"));

        ok = true; // TEMPORARY
      }
      else
      {
        error_printP(F("unknown request"));

        ok = false; // TEMPORARY
      }
    }
    else
    {
      error_printP_(F("Class Interface request "));
      error_printL(pSetup->bRequest);

      uint8_t idx = pSetup->wIndex;

      ok = false; // initial value

#ifdef CDC_ENABLED
      if (CDC_ACM_INTERFACE == idx)
      {
        ok = CDC_Setup(*pSetup);
      }
#endif

#ifdef HID_ENABLED
      if (HID_INTERFACE == idx)
      {
        ok = HID_Setup(*pSetup);
      }
#endif

//      TODO POOBAH FIX THIS
//      InitControl(pSetup->wLength);    //  Max length of transfer
//      ok = ClassInterfaceRequest(setup);

      // if NOT ok, do a STALL ???
    }

    if (ok)
    {
//      ClearIN();
      pBuf->iIndex = pBuf->iLen; // this eats the remainder of the packet
    }
    else
    {
//      Stall();
      // TODO:  NOT finished with the packet (yet), so don't allow receiving anything
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////
//                                                                                //
//   _   _  _         _             _                    _      _     ____  ___   //
//  | | | |(_)  __ _ | |__         | |  ___ __   __ ___ | |    / \   |  _ \|_ _|  //
//  | |_| || | / _` || '_ \  _____ | | / _ \\ \ / // _ \| |   / _ \  | |_) || |   //
//  |  _  || || (_| || | | ||_____|| ||  __/ \ V /|  __/| |  / ___ \ |  __/ | |   //
//  |_| |_||_| \__, ||_| |_|       |_| \___|  \_/  \___||_| /_/   \_\|_|   |___|  //
//             |___/                                                              //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

// "high level" API - manipulates buffers (async only) - no blocking
int USB_SendControlP(uint8_t flags, const void * PROGMEM_ORIG d, int len)
{
  if(!len)
  {
    return 0;
  }

  uint8_t *p1 = (uint8_t *)malloc(len);

  if(!p1)
  {
    return 1;
  }

  memcpy_P(p1, d, len);
  
  internal_send(0, p1, len);
  free(p1);

  return 0;
}

int USB_SendControl(uint8_t flags, const void* d, int len)
{
  if(!len)
  {
    return 0;
  }

  internal_send(0, d, len);
  return 0;
}

// return -1 on error, 0 if all bytes read, 1 if more bytes remain
int USB_RecvControl(void* d, int len)
{
int iRval = internal_receive(0, d, len);

  return iRval < 0 ? -1 : iRval == len ? 0 : 1;
}

uint8_t	USB_Available(uint8_t ep)
{
INTERNAL_BUFFER *pB;
uint8_t oldSREG;
int iRval = 0;

  // this manipulates buffers, so stop ints temporarily

  oldSREG = SREG;
  cli(); // disable interrupts

  pB = aRecvQ[ep];

  while(pB)
  {
    if(INTERNAL_BUFFER_RECV_READY(pB))
    {
      if(pB->iIndex < pB->iLen)
      {
        iRval = 1;
        break;
      }
    }

    pB = pB->pNext;
  }

  SREG = oldSREG;

  return iRval;
}

int USB_Send(uint8_t ep, const void* data, int len)
{
  internal_send(ep, data, len);
  return 0;
}

int USB_Recv(uint8_t ep, void* data, int len)
{
int iRval = internal_receive(ep, data, len);

  return iRval < 0 ? -1 : iRval == len ? 0 : 1;
}

int USB_Recv(uint8_t ep)
{
static uint8_t a1;
int iRval;

  iRval = internal_receive(ep, &a1, 1);

  if(iRval < 0)
  {
    return -1;
  }
  else if(!iRval) // there was data
  {
    return a1;
  }
  else
  {
    return 0; // for now [no data] [TODO:  block?]
  }
}


// an API for debugging help (mostly)
uint16_t GetFrameNumber(void)
{
  return epData.framenum; // last frame number read in by the device
}



//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//         ___         _                                   _                //
//        |_ _| _ __  | |_  ___  _ __  _ __  _   _  _ __  | |_  ___         //
//         | | | '_ \ | __|/ _ \| '__|| '__|| | | || '_ \ | __|/ __|        //
//         | | | | | || |_|  __/| |   | |   | |_| || |_) || |_ \__ \        //
//        |___||_| |_| \__|\___||_|   |_|    \__,_|| .__/  \__||___/        //
//                                                 |_|                      //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//  General interrupt (for the xmega, it's 'BUSEVENT' to handle SOF, etc.)
ISR(USB_BUSEVENT_vect) // USB_GEN_vect)
{
uint8_t udint;

  udint = *((volatile uint8_t *)&(USB_INTFLAGSACLR));// UDINT;

  // on startup of the hardware
  if((udint & (USB_SUSPENDIF_bm | USB_RESUMEIF_bm)) == (USB_SUSPENDIF_bm | USB_RESUMEIF_bm))
  {
    // NOTE:  I have observed that when you power up the USB for the first time, you get an interrupt
    //        in which BOTH the RESUME and SUSPEND bits are set.  This may *not* be a documented behavior
    //        but it happens, and I'd like to do something maybe... ?
//    DEBUG_OUT(F("USB powerup\r\n"));
    error_printP(F("USB powerup"));
  }
  else if(udint & USB_SUSPENDIF_bm)
  {
    // NOTE:  I get several suspend/resume combos after pulling the USB cable out

    // TODO:  use THIS to detect 'disconnected' - if the frame # has not changed,
    //        then I'm no longer connected.
//    error_printP(F("USB suspend IF"));
  }
  else if(udint & USB_RESUMEIF_bm)
  {
    // NOTE:  I get several suspend/resume combos after pulling the USB cable out

    // TODO:  use THIS to detect 'connected', along with frame # change.
//    error_printP(F("USB resume IF"));
  }

  //  End of Reset - happens when you first plug in, etc.
  if(udint & USB_RSTIF_bm) //(1<<EORSTI))
  {
//    DEBUG_OUT(F("USB RST\r\n"));
    error_printP(F("USB RESET IF"));

    USB_ADDR = 0; // IMMEDIATELY set USB address to 0 on reset.  not sure if this is necessary however

    // TODO:  clear the queues


    // TODO:  reset all global variables?
//    _cdcComposite = 0; // the default (this is a global, resetting should reset this too?)
//    _usbConfiguration = 0;      // not configured yet

    // TODO:  see if endpoint 0 needs to be re-done or not, perhaps just leaving it
    //        'as-is' may be MORE stable than re-doing it every! single! time!
    InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);  // init ep0

    // clear any 'stall' event this might cause
    *((volatile uint8_t *)&(USB_INTFLAGSACLR)) = USB_STALLIF_bm;
  }

  //  Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
  if(udint & USB_SOFIF_bm)//(1<<SOFI))
  {
//#ifdef TX_RX_LED_INIT
//    // check whether the one-shot period has elapsed.  if so, turn off the LED
//    if (TxLEDPulse && !(--TxLEDPulse))
//      TXLED0;
//    if (RxLEDPulse && !(--RxLEDPulse))
//      RXLED0;
//#endif // TX_RX_LED_INIT

    // if anything needs to be sent, send it NOW

    check_send_queue();
  }

//  if(udint & ~(USB_RSTIF_bm | USB_SOFIF_bm)) // anything else
//  {
//    DEBUG_OUT(F("USB other "));
//    DEBUG_OUT((uint32_t)udint);
//    DEBUG_OUT(F("\r\n"));
//  }

the_end: // label to allow graceful exit

  // exit code goes here

  *((volatile uint8_t *)&(USB_INTFLAGSACLR)) = udint; // this clears all of the flags that had been set
  // if any other flags were set during the ISR, I should get another interrupt
}

//  Transaction Complete
ISR(USB_TRNCOMPL_vect)
{
INTERNAL_BUFFER *pBuf, *pX;
uint8_t udint, requestType;


  udint = *((volatile uint8_t *)&(USB_INTFLAGSBCLR));

//  error_printP_(F("Tran compl "));
//  error_printL(udint);

  if(udint & USB_TRNIF_bm) // transaction complete - check all of them [has bit value of 2 for later ref]
  {
    // NOTE:  this can happen a LOT

    check_recv_queue();
    check_send_queue();
  }

  // ok, if I'm CURRENTLY in a setup transaction, I'll have some
  // more thingies to work on.  If the 'SETUPIF' bit is set, I'm
  // initiating a setup transaction.


  pBuf = /*end_of_chain*/(aRecvQ[0]); // the buffer from endpoint 0's receive queue

  // The next code *ONLY* handles 'SETUP' (well, I'll handle receive queue for endpoint 0 too)

  if (!(udint & USB_SETUPIF_bm) && (!pBuf || !pBuf->iLen))
  {
    // for now I only do setup transactions (??? not this time if pBuf not NULL)

    goto the_end;
  }

//  DEBUG_OUT(F("USB setup\r\n"));
  error_printP_(F("USB setup "));

  // treat like a completed receive for endpoint 0 only
  // then dispatch the setup

  if(pBuf &&
     epData.endpoint[0].out.dataptr == buffer_data_pointer(pBuf)) // I'm reading "that buffer"
  {
    pBuf->iLen = epData.endpoint[0].out.cnt; // note:  'out' is actually 'in' for whatever reason
    pBuf->iIndex = 0;

    error_printP_(F(" read "));
    error_printL_(pBuf->iLen);
    error_printP_(F(" bytes, index is "));
    error_printL(pBuf->iIndex);
  }
  else
  {
    error_printP(F("(no buffer)"));
  }

  if(!pBuf ||
     epData.endpoint[0].out.dataptr == buffer_data_pointer(pBuf)) // I'm reading "that buffer"
  {
    // now that I've received data, NULL data pointer
    epData.endpoint[0].out.status = USB_EP_BUSNACK0_bm; // make sure (stalls receiving anything more until I finish with this)
    epData.endpoint[0].out.dataptr = 0;
    epData.endpoint[0].out.cnt = 0;

//  epData.endpoint[0].in.ctrl &= ~USB_EP_SETUP_bm; // TODO:  do I need to do this???
    epData.endpoint[0].out.ctrl &= ~USB_EP_SETUP_bm;
  }

  if(!pBuf || (pBuf->iLen - pBuf->iIndex) < 8) // TODO:  does looping on this help?
  {
    error_printP_(F("WHOOPS:  "));
    error_printL_((uint32_t)pBuf);
    error_printP_(F(", "));
    error_printL_(pBuf->iLen);
    error_printP_(F(", "));
    error_printL_(pBuf->iIndex);
    error_printP_(F(", "));
    error_printL(pBuf->iLen - pBuf->iIndex);
  }
  else
  {
//    DumpBuffer(pBuf);

    internal_do_setup_request(pBuf);
  }

  // this is where I eat the packet and re-enable receiving data
  if(pBuf)
  {
    remove_from_queue(&(aRecvQ[0]), pBuf); // removes from queue and deletes buffer

    if(!epData.endpoint[0].out.dataptr)
    {
      set_up_EP_for_receive(0); // set up EP 0 to receive more data
    }
  }

the_end:

  *((volatile uint8_t *)&(USB_INTFLAGSBCLR)) = udint; // clears the bits
}


/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//   __  __  _      _         _                    _   _   _  _    _  _        //
//  |  \/  |(_)  __| |       | |  ___ __   __ ___ | | | | | || |_ (_)| | ___   //
//  | |\/| || | / _` | _____ | | / _ \\ \ / // _ \| | | | | || __|| || |/ __|  //
//  | |  | || || (_| ||_____|| ||  __/ \ V /|  __/| | | |_| || |_ | || |\__ \  //
//  |_|  |_||_| \__,_|       |_| \___|  \_/  \___||_|  \___/  \__||_||_||___/  //
//                                                                             //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

// mostly OLD CODE that needs to be adapted, or eliminated

//==================================================================
//==================================================================

u8 USBGetConfiguration(void)
{
  return 0; // for now
  //return _usbConfiguration;
}

//  Handle CLASS_INTERFACE requests
static bool ClassInterfaceRequest(Setup& rSetup)
{
  u8 i = rSetup.wIndex;

#ifdef CDC_ENABLED
  if (CDC_ACM_INTERFACE == i)
    return CDC_Setup(rSetup);
#endif

#ifdef HID_ENABLED
#error NO
  if (HID_INTERFACE == i)
    return HID_Setup(rSetup);
#endif
  return false;
}


int SendInterfaces()
{
  int total = 0;
  u8 interfaces = 0;

#ifdef CDC_ENABLED
  total = CDC_GetInterface(&interfaces);
#endif

#ifdef HID_ENABLED
  total += HID_GetInterface(&interfaces);
#endif

  return interfaces;
}

//  Construct a dynamic configuration descriptor
//  This really needs dynamic endpoint allocation etc
//  TODO
static bool SendConfiguration(int maxlen)
{
int interfaces;

//  DEBUG_OUT(F("USB SendConfiguration\r\n"));
  error_printP(F("USB SendConfiguration"));

  //  Count and measure interfaces
  // to do this properly, I'll allocate a packet, build it, and then
  // add it to the send queue.  MUCH better!


  interfaces = SendInterfaces();
  ConfigDescriptor config = D_CONFIG(/*_cmark +*/ sizeof(ConfigDescriptor),interfaces);

  //  Now send them
  USB_SendControl(0, &config, sizeof(ConfigDescriptor) /*, maxlen*/);
  SendInterfaces();
  return true;
}

u8 _cdcComposite = 0;


// NOTE this function does *NOT* do what it says on the tin...

static bool SendDescriptor(Setup& rSetup)
{
u8 t;
bool bRval;

//  DEBUG_OUT(F("USB SendDescriptor\r\n"));
  error_printP(F("USB SendDescriptor"));

//#ifdef LED_SIGNAL1
//  digitalWrite(LED_SIGNAL1,digitalRead(LED_SIGNAL1) == LOW ? HIGH : LOW);
//#endif // LED_SIGNAL1

  t = rSetup.wValueH;
  if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
  {
    error_printP(F("Send Configuration"));

    bRval = SendConfiguration(rSetup.wLength);
//    ClearIN(); /// misleading function name, sends the data      
    return bRval;
  }
  

#ifdef HID_ENABLED
  if (HID_REPORT_DESCRIPTOR_TYPE == t)
  {
    error_printP(F("HID Get Descriptor"));

    return HID_GetDescriptor(t, rSetup.wLength);
  }
#endif

  u8 desc_length = 0;
  const u8* desc_addr = 0;
  if (USB_DEVICE_DESCRIPTOR_TYPE == t)
  {
    error_printP(F("Device Descriptor Type"));

    if (rSetup.wLength == 8)
    {
      _cdcComposite = 1;
    }
//    else
//    {
//      _cdcComposite = 0; // the default (added by me to restore default as needed)
//    }

    desc_addr = _cdcComposite ?  (const u8*)&USB_DeviceDescriptorA : (const u8*)&USB_DeviceDescriptor;
  }
  else if (USB_STRING_DESCRIPTOR_TYPE == t)
  {
    error_printP_(F("String Descriptor Type - "));

    if (rSetup.wValueL == 0)
    {
      error_printP(F("Language"));

      desc_addr = (const u8*)&STRING_LANGUAGE;
    }
    else if (rSetup.wValueL == IPRODUCT) 
    {
      error_printP(F("Product"));

      desc_addr = (const u8*)&STRING_IPRODUCT;
    }
    else if (rSetup.wValueL == IMANUFACTURER)
    {
      error_printP(F("Manufacturer"));

      desc_addr = (const u8*)&STRING_IMANUFACTURER;
    }
    else
    {
      error_printP_(F("SendDescriptor - bad setup index "));
      error_printL(rSetup.wValueL);

      return false;
    }
  }

  if(desc_addr == 0)
  {
    error_printP(F("SendDescriptor - zero pointer"));

    return false;
  }

  if(desc_length == 0)
  {
    desc_length = pgm_read_byte(desc_addr);

    error_printP_(F("Data Length = "));
    error_printL(desc_length);
  }

  USB_SendControlP(TRANSFER_PGM, desc_addr, desc_length);

  return true;
}



////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                            //
//   _   _  ____   ____   ____                _                          _                    //
//  | | | |/ ___| | __ ) |  _ \   ___ __   __(_)  ___  ___          ___ | |  __ _  ___  ___   //
//  | | | |\___ \ |  _ \ | | | | / _ \\ \ / /| | / __|/ _ \        / __|| | / _` |/ __|/ __|  //
//  | |_| | ___) || |_) || |_| ||  __/ \ V / | || (__|  __/       | (__ | || (_| |\__ \\__ \  //
//   \___/ |____/ |____/ |____/  \___|  \_/  |_| \___|\___|_____   \___||_| \__,_||___/|___/  //
//                                                        |_____|                             //
//                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////

USBDevice_ USBDevice;

USBDevice_::USBDevice_()
{
//  DEBUG_OUT(F("USB - device const\r\n"));
}

void USBDevice_::attach()
{
uint16_t index;
unsigned long lTick = millis();

//  DEBUG_OUT(F("USB - attach\r\n"));
  error_printP(F("USB - attach"));

  uint8_t oldSREG = SREG; // save int flag
  cli(); // no interrupt handling until I'm done setting this up

#ifdef INIT_EP0_ONE_TIME_ONLY
  bInitEP0 = 0;
#endif // INIT_EP0_ONE_TIME_ONLY

  USB_INTCTRLA = 0;
  USB_INTCTRLB = 0;

  USB_INTFLAGSACLR = 0xff; // clear all int flags
  USB_INTFLAGSBCLR = 0x3;  // clear all int flags

//  _usbConfiguration = 0;

  // enable the USB clock using the 32mhz RC oscillator
  // assume either slow (6mhz) or fast (48mhz)
  // and of course the pre-scaler must be assigned accordingly
  // Also, assume that the oscillator is *SET UP* *PROPERLY* already, and
  // that all I have to do is configure the PLL to run at 48Mhz

  // setting up the PLL - source is RC32M 'divided by 4' then multiplied by 6 for 48Mhz

  USB_CTRLA = 0; // shut down USB
  USB_CTRLB = 0; // detach D- and D+

//  CCP = CCP_IOREG_gc; // is this needed? see D manual, sect 3.14.1 (protected I/O)
  CLK_USBCTRL = 0; // shut off USB clock

//  CCP = CCP_IOREG_gc; // is this needed? see D manual, sect 3.14.1 (protected I/O)
  OSC_CTRL &= ~(OSC_PLLEN_bm); // disable PLL osc

  SREG=oldSREG; // re-enable ints

// TODO;  use 32Mhz clock as 48Mhz, shifting system clock to the PLL at 32Mhz ?
#ifdef USE_RC2M
  OSC_CTRL |= OSC_RC2MEN_bm;     // enable 2M osc
  
  while(!(OSC_STATUS & OSC_RC2MRDY_bm)) // wait for 2M RC osc to be 'ready'
  {
    // TODO:  timeout?
  }

  // now config PLL and USB clock stuff

  // 2Mhz as the source, multiplicatino factor of 24 = 48Mhz
  OSC_PLLCTRL = OSC_PLLSRC_RC2M_gc | 24; // 24 times the 2Mhz frequency
  // TODO:  set up the calibration PLL for 2Mhz ?
#else // USE_RC2M
  // 32Mhz (divided by 4, so it's 8Mhz) as the source
  // multiplication factor of 6 - result = 48Mhz
  OSC_PLLCTRL = OSC_PLLSRC_RC32M_gc | 6; // 6 times the 8Mhz frequency
#endif // USE_RC2M

//  CCP = CCP_IOREG_gc; // is this needed? see D manual, sect 3.14.1 (protected I/O)
  OSC_CTRL |= OSC_PLLEN_bm; // re-enable PLL

  while(!(OSC_STATUS & OSC_PLLRDY_bm)) // wait for PLL to be 'ready'
  {
    // TODO:  timeout?
  }

  error_printP_(F("TEMPORARY:  this part took "));
  error_printL_(millis() - lTick);
  error_printP(F(" milliseconds"));

  oldSREG = SREG; // save int flag (again)
  cli(); // no interrupt handling until I'm done setting this up

  // protected I/O reg
//  CCP = CCP_IOREG_gc; // is this needed? see D manual, sect 3.14.1 (protected I/O)
#ifdef FAST_USB /* note this is 12Mbit operation, 'FULL' speed, not 'HIGH' speed 480Mbit */
  CLK_USBCTRL = CLK_USBSRC_PLL_gc; // use PLL (divide by 1, no division)

#else // SLOW
  CLK_USBCTRL = CLK_USBSRC_PLL_gc  // use PLL
              | CLK_USBPSDIV_8_gc; // divide by 8 for 6mhz operation (12Mhz?  see 7.3.6 which says 12Mhz or 48Mhz)

#endif // FAST_USB or SLOW

//  CCP = CCP_IOREG_gc; // is this needed? see D manual, sect 3.14.1 (protected I/O)
  CLK_USBCTRL |= CLK_USBEN_bm;     // enable bit


  // assign CAL register from product signatures (4.17.17,18)
  USB_CAL0 = readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_USBCAL0); // docs say 'CALL'
  USB_CAL1 = readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_USBCAL1); // docs say 'CALH'

  // set the max # of endpoints, speed, and 'store frame number' flags
  USB_CTRLA = MAXEP /* max # of endpoints minus 1 */
#ifdef FAST_USB
            | USB_SPEED_bm /* all ahead 'FULL' - aka 'FULL' speed ahead! */
#endif // FAST_USB
            | USB_STFRNUM_bm // store the frame number
   // TODO:  FIFO ?
            ;

  init_buffers_and_endpoints();

  USB_EPPTR = endpoint_data_pointer();

  USB_ADDR = 0; // set USB address to 0 (default before 'SETUP')

  // LAST of all, enable interrupts
  USB_INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;    // enable 'transaction complete' and 'setup' interrupts
  USB_INTCTRLA  = USB_SOFIE_bm                     // enable the start of frame interrupt
                | USB_BUSEVIE_bm                   // 'bus event' interrupt - suspend, resume, reset
                                                   // for the RESET event, RESETIF will be set (20.14.11 in AU manual)
                | USB_INTLVL0_bm | USB_INTLVL1_bm; // int level 3


//  InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);  // init ep0 (do this NOW)

  // NOW enable the USB
  USB_CTRLA |= USB_ENABLE_bm;

  SREG = oldSREG; // restore int flags

  // attach the wiring for D- and D+ AFTER everything else.

  USB_CTRLB = USB_ATTACH_bm; // attach D- and D+ (also enables pullup resistors based on speed)
  // on the 128A1U, this is PD6 (D-) and PD7 (D+) [YMMV on the other processors]
  // this is partly why it's good to use PORTC for the primary SPI, TWI, etc.
  // since PORTD gets used for 'other things', but it's STILL ok to use PD2,3 as a serial port
  // and so the default pin config works pretty well.

#ifdef TX_RX_LED_INIT  
  TX_RX_LED_INIT;
#endif // TX_RX_LED_INIT

//  DEBUG_OUT(F("USB Attach (done)\r\n"));
  error_printP(F("USB Attach (done)"));

  // NOTE:  at THIS point the ISR will manage everything else.  The USB will receive
  //        messages to set up the connection, etc. and the ISR will drive it.
}

void USBDevice_::detach()
{
//  DEBUG_OUT(F("USB - detach\r\n"));
  error_printP(F("USB - detach"));

  uint8_t oldSREG = SREG; // save int flag
  cli(); // no interrupt handling until I'm done setting this up

  USB_INTCTRLA = 0; // disabling interrupts
  USB_INTCTRLB = 0;
  USB_CTRLB = 0; // detach D- and D+
  USB_CTRLA = 0; // shut down USB
  CLK_USBCTRL = 0; // shut off USB clock

  init_buffers_and_endpoints(); // re-initialize these

//  _usbConfiguration = 0;

  SREG = oldSREG; // restore int flags

//  DEBUG_OUT(F("USB Detach\r\n"));
  error_printP(F("USB Detach (done)"));
}

// added this for access to USB device structures
XMegaEPDataStruct *USBDevice_::GetEPData()
{
  return &epData;
}

//  Check for interrupts
//  TODO: VBUS detection
bool USBDevice_::configured()
{
//  return _usbConfiguration;
  return 0; // FOR NOW   TODO POOBAH FIX THIS
}

void USBDevice_::poll()
{
}

#endif /* if defined(USBCON) */

