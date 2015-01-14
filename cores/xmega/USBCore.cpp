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

#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"

#include "wiring_private.h"


#if defined(USBCON)

// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG
//#define DEBUG_CODE 1 // TEMPORARY
#ifdef DEBUG_CODE
extern void DebugOut(unsigned long lVal) __attribute__((weak));
extern void DebugOut(const void * PROGMEM pStr) __attribute__((weak));
extern void DebugDumpRegs(const XMegaEPDataStruct * pRegs) __attribute__((weak));

// 'weak link' overloads for debug functions that prevent link fail
extern void DebugOut(unsigned long lVal) { }
extern void DebugOut(const void * PROGMEM pStr) { }
extern void DebugDumpRegs(const XMegaEPDataStruct * pRegs) { }
#define DEBUG_OUT(X) DebugOut(X)
#define DEBUG_DUMP_REGS(X) DebugDumpRegs(X)

#define LED_SIGNAL0 (LED_BUILTIN-2)
#define LED_SIGNAL1 (LED_BUILTIN-3)
#define LED_SIGNAL2 (LED_BUILTIN-4)
#define LED_SIGNAL3 (LED_BUILTIN-5)

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

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
#ifdef TX_RX_LED_INIT /* only when these are defined */
#define TX_RX_LED_PULSE_MS 100
volatile u8 TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
volatile u8 RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
#endif // TX_RX_LED_INIT

// NOTE:  auto ZLP is broken according to 128A1U errata
#define ZLP_BIT 0/*(((uint16_t)USB_EP_ZLP_bm)<<8)*/

//==================================================================
//==================================================================

extern const u16 STRING_LANGUAGE[] PROGMEM;
extern const u16 STRING_IPRODUCT[] PROGMEM;
extern const u16 STRING_IMANUFACTURER[] PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptor PROGMEM;
extern const DeviceDescriptor USB_DeviceDescriptorA PROGMEM;

const u16 STRING_LANGUAGE[2] = {
  (3<<8) | (2+2),
  0x0409  // English
};

const u16 STRING_IPRODUCT[17] = {
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

const u16 STRING_IMANUFACTURER[12] = {
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
const DeviceDescriptor USB_DeviceDescriptor =
  D_DEVICE(0x00,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);

const DeviceDescriptor USB_DeviceDescriptorA =
  D_DEVICE(DEVICE_CLASS,0x00,0x00,64,USB_VID,USB_PID,0x100,IMANUFACTURER,IPRODUCT,0,1);



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

#define INTERNAL_BUFFER_LENGTH 64

typedef struct
{
  uint8_t aBuf1[INTERNAL_BUFFER_LENGTH]; // normally received data (for 'out' or 'control')
  uint8_t aBuf2[INTERNAL_BUFFER_LENGTH]; // normally send data (for 'in' or 'control') [also 'ping pong' 2nd buffer?]
  volatile uint8_t iBufIndex1, iBufIndex2; // index into buffer
  volatile uint8_t iBufLen1, iBufLen2;     // length of data in buffer
} INTERNAL_BUFFER;

INTERNAL_BUFFER aEPBuff[INTERNAL_NUM_EP];


static void DoTransactionComplete(void);


//==================================================================
//==================================================================


// NOTE:  PR_PRGEN should have PR_USB_bm bit cleared to enable the USB

volatile u8 _usbConfiguration = 0;

static XMegaEPDataStruct epData;

static uint8_t endpointIndex = 0;

static inline void WaitIN(void)
{
////  while (!(UEINTX & (1<<TXINI)));

// TODO:  do i need this?
//uint16_t spin_ctr = 0;
//
//  // must call 'setEP' beforehand
//  while(!(epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm))
//  {
//    if(!(++spin_ctr))
//    {
//      DEBUG_OUT(F("WaitIN timeout\r\n"));
//      break;
//    }
//  }

// TODO:  USB_EP_SETUP_bm as well?

// alternate method, requires calling USB_TRNCOMPL_vect ISR if 'i' flag clear
// since I don't add to the buffer, the buffer would have had to be empty already
// and I wait until I get a packet, then it "fills" (not one char at a time)
//  while(aEPBuff[endpointIndex].iBufIndex1 >= aEPBuff[endpointIndex].iBufLen1)
//  {
//  }

  // since it's async, do I need to bother?

  if(aEPBuff[endpointIndex].iBufLen2 &&
     aEPBuff[endpointIndex].iBufIndex2 >= aEPBuff[endpointIndex].iBufLen2) // 'sending' marker (for now)
  {
    // for now do it THIS way; later, maybe different?
    while(aEPBuff[endpointIndex].iBufLen2)//epData.endpoint[endpointIndex].in.cnt) // when non-zero I am writing data
    {
      if((*((volatile uint8_t *)&(USB_INTFLAGSBCLR)) & USB_TRNIF_bm) ||
         (epData.endpoint[endpointIndex].out.status & USB_EP_TRNCOMPL0_bm) || // data received
         (epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm))    // data sent
      {
        *((volatile uint8_t *)&(USB_INTFLAGSBCLR)) = USB_TRNIF_bm; // clear this bit
        DoTransactionComplete();
        break;
      }
//      else if(!(epData.endpoint[endpointIndex].in.cnt & 0x3ff)) // check? (epData.endpoint[endpointIndex].in.status & USB_EP_BUSNACK0_bm)
//      {
//        aEPBuff[endpointIndex].iBufLen2 = aEPBuff[endpointIndex].iBufIndex2 = 0;
//        break;
//      }

      // TODO:  max loop count?
    }
  }
}

static inline void ClearIN(void) // this SENDS data, uses buffer 2
{
////  UEINTX = ~(1<<TXINI);
//  epData.endpoint[endpointIndex].in.status &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_SETUP_bm);

  epData.endpoint[endpointIndex].in.cnt = aEPBuff[endpointIndex].iBufLen2 | ZLP_BIT; // SEND data

  aEPBuff[endpointIndex].iBufIndex2 = aEPBuff[endpointIndex].iBufLen2; // marks 'sending' kinda

  epData.endpoint[endpointIndex].in.status &= ~USB_EP_BUSNACK0_bm;

}

static inline void WaitOUT(void) // this RECEIVES data (wait for received data)
{
////  while (!(UEINTX & (1<<RXOUTI)))
////    ;

// TODO:  do i need this?
//uint16_t spin_ctr = 0;
//
//  while(!(epData.endpoint[endpointIndex].out.status & USB_EP_TRNCOMPL0_bm))
//  {
//    if(!(++spin_ctr))
//    {
//      DEBUG_OUT(F("WaitOUT timeout\r\n"));
//      break;
//    }
//  }

// TODO:  USB_EP_SETUP_bm as well?


  // if it's important enough to call WaitOUT(), I should implement something
  // which means doing some kind of asynchronous processing and handling I/O
  // although it *might* be 'out of order' when I do it

  while(aEPBuff[endpointIndex].iBufIndex1 >= aEPBuff[endpointIndex].iBufLen1)
  {
    if(aEPBuff[endpointIndex].iBufLen1 > 0) // if there's data in it, clear it
    {
      aEPBuff[endpointIndex].iBufIndex1 = 0;
      aEPBuff[endpointIndex].iBufLen1 = 0;

      epData.endpoint[endpointIndex].out.cnt = 0; // no data (so I can receive again)
    }

    if(*((volatile uint8_t *)&(USB_INTFLAGSBCLR)) & USB_TRNIF_bm)
    {
      *((volatile uint8_t *)&(USB_INTFLAGSBCLR)) = USB_TRNIF_bm; // clear this bit
      DoTransactionComplete();
    }
  }
}

static inline void ClearOUT(void) // this RECEIVES data, uses buffer 1
{
////  UEINTX = ~(1<<RXOUTI);
//
//  epData.endpoint[endpointIndex].out.status &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_SETUP_bm);

  aEPBuff[endpointIndex].iBufIndex1 = 0;
  aEPBuff[endpointIndex].iBufLen1 = 0; // "clear" the buffer [for now, later double-buffer?]

  epData.endpoint[endpointIndex].out.cnt = 0; // no data (so I can receive again)
}

#if 0 /* remove bogus thingy */
static inline u8 WaitForINOrOUT()
{
////  while (!(UEINTX & ((1<<TXINI)|(1<<RXOUTI))))
////    ;
////  return (UEINTX & (1<<RXOUTI)) == 0;

// TODO:  do i need this?
//uint16_t spin_ctr = 0;
//
//  while(!(epData.endpoint[endpointIndex].out.status & USB_EP_TRNCOMPL0_bm) &&
//        !(epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm))
//  {
//    if(!(++spin_ctr))
//    {
//      DEBUG_OUT(F("WaitForINOrOUT timeout\r\n"));
//      break;
//    }
//  }

// TODO:  USB_EP_SETUP_bm as well?

//  return (epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm) == 0;

  // if it's important enough to call WaitOUT(), I should implement something
  // which means doing some kind of asynchronous processing and handling I/O
  // although it *might* be 'out of order' when I do it

  while(aEPBuff[endpointIndex].iBufIndex1 >= aEPBuff[endpointIndex].iBufLen1 &&
        epData.endpoint[endpointIndex].in.cnt) // when non-zero I am writing data
  {
    if(aEPBuff[endpointIndex].iBufLen1 > 0) // if there's data in it, clear it
    {
      aEPBuff[endpointIndex].iBufIndex1 = 0;
      aEPBuff[endpointIndex].iBufLen1 = 0;

      epData.endpoint[endpointIndex].out.cnt = 0; // no data (so I can receive again)
    }

    if(*((volatile uint8_t *)&(USB_INTFLAGSBCLR)) & USB_TRNIF_bm)
    {
      *((volatile uint8_t *)&(USB_INTFLAGSBCLR)) = USB_TRNIF_bm; // clear this bit
      DoTransactionComplete();
    }
  }

  return aEPBuff[endpointIndex].iBufLen1 > 0; // meaning it has received data
}
#endif // 0

static inline u8 Recv8()
{
#ifdef TX_RX_LED_INIT
  RXLED1;          // light the RX LED
  RxLEDPulse = TX_RX_LED_PULSE_MS;
#endif // TX_RX_LED_INIT
  uint8_t rVal, oldSREG;

  // return UEDATX;  the old way

  oldSREG = SREG;
  cli(); // disable interrupts

  if(aEPBuff[endpointIndex].iBufIndex1 < aEPBuff[endpointIndex].iBufLen1)
  {
    // I am receiving with buffer 1

    rVal = aEPBuff[endpointIndex].aBuf1[(aEPBuff[endpointIndex].iBufIndex1)++];
  }
//  else if(aEPBuff[endpointIndex].iBufIndex2 < aEPBuff[endpointIndex].iBufLen2 && endpointIndex != 0)
//  {
//    // I am receiving with buffer 2 [note:  is there a way to prevent going back to '1' again ?]
//
//    rVal = aEPBuff[endpointIndex].aBuf2[(aEPBuff[endpointIndex].iBufIndex2)++];
//  }
  else
  {
    // TODO: wait for input??

    rVal = 0xff; // for now...
  }

  SREG=oldSREG;

  return rVal;
}

void Recv(volatile u8* data, u8 count)
{
  while (count--)
  {
    *data++ = Recv8();
  }

#ifdef TX_RX_LED_INIT  
  RXLED1;          // light the RX LED
  RxLEDPulse = TX_RX_LED_PULSE_MS;  
#endif // TX_RX_LED_INIT  
}

static inline void Send8(u8 d)
{
  uint8_t iTemp, oldSREG;
//  UEDATX = d;

  oldSREG = SREG;
  cli(); // disable interrupts

  // auto wait-to-send if buffer being sent right now

  WaitIN(); // automatic (for now), misleading function name waits for OUTPUT COMPLETE

  iTemp = aEPBuff[endpointIndex].iBufLen2;

  if(iTemp < INTERNAL_BUFFER_LENGTH)
  {

    aEPBuff[endpointIndex].aBuf2[iTemp++] = d;

    aEPBuff[endpointIndex].iBufLen2 = iTemp;

    if(iTemp >= INTERNAL_BUFFER_LENGTH)
    {
      // do I force-send the buffer now?
      ClearIN(); /// misleading function name, sends the data      
    }

  }

  SREG=oldSREG;
}

static inline void SetEP(u8 ep)
{
//  UENUM = ep;
  endpointIndex = ep; // now a var, as an index into 'epData'
}

static inline uint8_t ReadByteCount()
{
  if(aEPBuff[endpointIndex].iBufIndex1 < aEPBuff[endpointIndex].iBufLen1)
  {
    return aEPBuff[endpointIndex].iBufLen1 - aEPBuff[endpointIndex].iBufIndex1;
  }

  return 0;  // TODO:  am I reading or writing on endpoint 0?
}

static inline uint8_t WriteByteCount()
{
  if(aEPBuff[endpointIndex].iBufIndex2 < aEPBuff[endpointIndex].iBufLen2)
  {
    return aEPBuff[endpointIndex].iBufLen2 - aEPBuff[endpointIndex].iBufIndex2;
  }

  return 0;  // TODO:  am I reading or writing on endpoint 0?
}

//static inline u8 FifoByteCount()
//{
////  return UEBCLX;
////  return 0; // for now, later see how it's used
//  if(aEPBuff[endpointIndex].iBufIndex1 < aEPBuff[endpointIndex].iBufLen1)
//  {
//    return aEPBuff[endpointIndex].iBufLen1 - aEPBuff[endpointIndex].iBufIndex1;
//  }
////  else if(aEPBuff[endpointIndex].iBufIndex2 < aEPBuff[endpointIndex].iBufLen2 && endpointIndex != 0)
////  {
////    return aEPBuff[endpointIndex].iBufLen2 - aEPBuff[endpointIndex].iBufIndex2;
////  }
//
//  return 0;  // TODO:  am I reading or writing on endpoint 0?
//}

//static inline u8 ReceivedSetupInt()
//{
////  return UEINTX & (1<<RXSTPI);
//  return USB_INTFLAGSBCLR & USB_SETUPIF_bm;
//}

//static inline void ClearSetupInt()
//{
////  UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
//  USB_INTFLAGSBCLR = USB_SETUPIF_bm; // clear the bit
//}

static inline void Stall()
{
//  UECONX = (1<<STALLRQ) | (1<<EPEN);

  // TODO:  how do I do this?  (POOBAH)

  DEBUG_OUT(F("USB STALL\r\n"));

  epData.endpoint[endpointIndex].out.ctrl |= 4; // this bit isn't properly defined, so see 20.15.2 in AU manual
  // NOTE:  if isochronous, this doesn't work
}

static inline u8 ReadWriteAllowed()
{
//  return UEINTX & (1<<RWAL);
  return 1; // for now, since it's buffered I/O
}

static inline u8 Stalled()
{
//  return UEINTX & (1<<STALLEDI);
  return epData.endpoint[endpointIndex].out.ctrl & 0x4 != 0; // USB_INTFLAGSACLR & USB_STALLIF_bm;
  // this bit isn't properly defined, so see 20.15.2 in AU manual
}

static inline u8 FifoFree()
{
//  return UEINTX & (1<<FIFOCON);

  return 0;  // for now
}

static inline void ReleaseRX()
{
//  UEINTX = 0x6B;  // FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1

  uint8_t oldSREG = SREG;
  cli(); // disable interrupts

//  if(aEPBuff[endpointIndex].iBufIndex1 < aEPBuff[endpointIndex].iBufLen1)
//  {
    // I am receiving with buffer 1

    aEPBuff[endpointIndex].iBufIndex1 = aEPBuff[endpointIndex].iBufLen1 = 0;
//  }
//  else if(aEPBuff[endpointIndex].iBufIndex2 < aEPBuff[endpointIndex].iBufLen2 && endpointIndex != 0)
//  {
//    // I am receiving with buffer 2 [note:  is there a way to prevent going back to '1' again ?]
//
//    aEPBuff[endpointIndex].iBufIndex2 = aEPBuff[endpointIndex].iBufLen2 = 0;
//  }

  epData.endpoint[endpointIndex].out.cnt = 0; // allows me to receive another packet
  epData.endpoint[endpointIndex].out.ctrl &= ~0x4; // turn off 'stall' if on - see 20.15.2 in AU manual

  epData.endpoint[endpointIndex].out.status &= ~USB_EP_BUSNACK0_bm;

  SREG = oldSREG;
}

static inline void ReleaseTX() // function name is misleading, change it
{
//  UEINTX = 0x3A;  // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0

  uint8_t oldSREG = SREG;
  cli(); // disable interrupts

  // TODO:  auto-zero-length packet? see 20.15.4 in AU manual
  epData.endpoint[endpointIndex].in.cnt = aEPBuff[endpointIndex].iBufLen2 | ZLP_BIT; // sends the data (?)
  aEPBuff[endpointIndex].iBufIndex2 = aEPBuff[endpointIndex].iBufLen2; // to indicate I am sending?  or something like that

  epData.endpoint[endpointIndex].in.status &= ~USB_EP_BUSNACK0_bm;

  SREG = oldSREG;
}

// an API for debugging help
uint16_t GetFrameNumber(void)
{
  return epData.framenum;
}

// removed - not used
//static inline u8 FrameNumber()
//{
//  // return UDFNUML;
//  return epData.framenum.l; // NOTE:  why ONLY the low byte?  ah, well, that's what the OLD code did...
//}

//==================================================================
//==================================================================

u8 USBGetConfiguration(void)
{
  return _usbConfiguration;
}

#define USB_RECV_TIMEOUT
class LockEP
{
  u8 _sreg;
public:
  LockEP(u8 ep) : _sreg(SREG)
  {
// POOBAH - see if there's a better way
//    cli();
    SetEP(ep & 7);
  }
  ~LockEP()
  {
    SREG = _sreg;
  }
};

//  Number of bytes, assumes a rx endpoint
u8 USB_Available(u8 ep)
{
  LockEP lock(ep);
//  return FifoByteCount();
  return ReadByteCount();
}

//  Non Blocking receive
//  Return number of bytes read
int USB_Recv(u8 ep, void* d, int len)
{
  if (!_usbConfiguration || len < 0)
    return -1;
  
  LockEP lock(ep);
  u8 n = ReadByteCount();//FifoByteCount();
  len = min(n,len);
  n = len;
  u8* dst = (u8*)d;
  while (n--)
    *dst++ = Recv8();
  if (len && !ReadByteCount()) //FifoByteCount())  // release empty buffer
    ReleaseRX();
  
  return len;
}

//  Recv 1 byte if ready
int USB_Recv(u8 ep)
{
  u8 c;
  if (USB_Recv(ep,&c,1) != 1)
    return -1;
  return c;
}

//  Space in send EP
u8 USB_SendSpace(u8 ep)
{
  LockEP lock(ep);
  if (!ReadWriteAllowed())
    return 0;
  return INTERNAL_BUFFER_LENGTH - WriteByteCount(); //FifoByteCount();
}

//  Blocking Send of data to an endpoint
int USB_Send(u8 ep, const void* d, int len)
{
  if (!_usbConfiguration)
    return -1;

  int r = len;
  const u8* data = (const u8*)d;
  u8 zero = ep & TRANSFER_ZERO;
  u8 timeout = 250;    // 250ms timeout on send? TODO
  while (len)
  {
    u8 n = USB_SendSpace(ep);
    if (n == 0)
    {
      if (!(--timeout))
        return -1;
      delay(1);
      continue;
    }

    if (n > len)
      n = len;
    {
      LockEP lock(ep);
      // Frame may have been released by the SOF interrupt handler
      if (!ReadWriteAllowed())
        continue;
      len -= n;
      if (ep & TRANSFER_ZERO)
      {
        while (n--)
          Send8(0);
      }
      else if (ep & TRANSFER_PGM)
      {
        while (n--)
          Send8(pgm_read_byte(data++));
      }
      else
      {
        while (n--)
          Send8(*data++);
      }
      if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))  // Release full buffer
        ReleaseTX();
    }
  }

#ifdef TX_RX_LED_INIT
  TXLED1;          // light the TX LED
  TxLEDPulse = TX_RX_LED_PULSE_MS;
#endif // TX_RX_LED_INIT

  return r;
}

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


// SEE SECTION 20.3 in 'AU' manual for sequence of operation

static
void InitEP(u8 index, u8 type, u8 size)
{
uint8_t oldSREG;
int i1;


  DEBUG_OUT(F("USB InitEP "));
  DEBUG_OUT((uint32_t)index);
  DEBUG_OUT(F("\r\n"));

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

  // if the endpoint is in the middle of something, this will 'cancel' it
  epData.endpoint[index].out.status |= USB_EP_BUSNACK0_bm;
  epData.endpoint[index].in.status |= USB_EP_BUSNACK0_bm;

  // zero out the 'aEPBuff' structure entry right away
  // as well as the 'endpoint' structures
  memset(&(aEPBuff[index]), 0, sizeof(aEPBuff[0]));
  memset(&(epData.endpoint[index]), 0, sizeof(epData.endpoint[0]));

  // disable the endpoints
  epData.endpoint[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)
  epData.endpoint[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)

  // NOTE: 'BUSNACK0' is needed by 'in' on 128A1U rev K or earlier [a bug, apparently]
  epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm; // leave 'BUSNACK0' bit ON (stalls sending data)
  epData.endpoint[index].out.status = 0; // set this one to zero (ready to receive data)

  if(index == 0 && type == EP_TYPE_CONTROL) // control (these can receive SETUP requests)
  {
    // aBuf1 is output, aBuf2 is input

    epData.endpoint[index].in.dataptr = (uint16_t)&(aEPBuff[index].aBuf2[0]);  // NOTE:  this is 'send' data (?)
    epData.endpoint[index].in.auxdata = 0;
    epData.endpoint[index].in.cnt = 0; //ZLP_BIT; // no data (so I won't send) plus "auto zero-length packet"

    epData.endpoint[index].out.dataptr = (uint16_t)&(aEPBuff[index].aBuf1[0]); // NOTE:  this is 'receive' data
    epData.endpoint[index].out.auxdata = 64; // temporary - see if it helps
    epData.endpoint[index].out.cnt = 0; // no data (so I can receive)

    // NOTE:  size will be sent as 'EP_SINGLE_64'

    // TODO:  do I do 'in' as well, or just 'out'?  'out' receives... 'in' sends
    epData.endpoint[index].in.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                                   | USB_EP_SIZE_64_gc;//(size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */

    epData.endpoint[index].out.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                                    | USB_EP_SIZE_64_gc;//(size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */

    epData.endpoint[index].in.status = USB_EP_BUSNACK0_bm; // leave 'BUSNACK0' bit ON (stalls sending data)
    epData.endpoint[index].out.status = 0; // make sure they're ready to go

    // zero out the rest of them, leaving ONLY the control
    for(i1=1; i1 <= MAXEP; i1++)
    {
      if(i1 < INTERNAL_NUM_EP)
      {
        memset(&(aEPBuff[i1]), 0, sizeof(aEPBuff[0]));
      }

      epData.endpoint[i1].in.ctrl = USB_EP_TYPE_DISABLE_gc;
      epData.endpoint[i1].out.ctrl = USB_EP_TYPE_DISABLE_gc;

      epData.endpoint[i1].in.status = USB_EP_BUSNACK0_bm; // make sure (stall sending)
      epData.endpoint[i1].out.status = 0; // make sure

      epData.endpoint[i1].in.dataptr = 0;
      epData.endpoint[i1].in.auxdata = 0;
      epData.endpoint[i1].in.cnt = 0;

      epData.endpoint[i1].out.dataptr = 0;
      epData.endpoint[i1].out.auxdata = 0;
      epData.endpoint[i1].out.cnt = 0;
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
    epData.endpoint[index].in.dataptr = (uint16_t)&(aEPBuff[index].aBuf2[0]); // 'in' gets aBuf2 [for now]
    epData.endpoint[index].in.auxdata = 0;

    epData.endpoint[index].in.cnt = ZLP_BIT; // no data (so I won't send) plus 'auto-zero-length packet'

    epData.endpoint[index].in.ctrl = (type == EP_TYPE_ISOCHRONOUS_IN ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                                   | (type == EP_TYPE_BULK_IN ? USB_EP_INTDSBL_bm : 0)       /* disable interrupt */
                                   | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc :             // TODO:  set 'double buffer' flag?
                                      size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);         /* data size */
  }
  else if(type == EP_TYPE_INTERRUPT_OUT || type == EP_TYPE_BULK_OUT /* these send *ME* data */
          || type == EP_TYPE_ISOCHRONOUS_OUT)
  {
    epData.endpoint[index].out.dataptr = (uint16_t)&(aEPBuff[index].aBuf1[0]); // 'out' gets aBuf1 [for now] [this way I can 'ping-pong']
    epData.endpoint[index].out.auxdata = 0;

    epData.endpoint[index].out.cnt = 0; // no data (so I can receive)

    epData.endpoint[index].out.ctrl = (type == EP_TYPE_ISOCHRONOUS_OUT ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                                    | (type == EP_TYPE_BULK_OUT ? USB_EP_INTDSBL_bm : 0)      /* disable interrupt */
                                    | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc :             // TODO:  set 'double buffer' flag?
                                       size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);         /* data size */
  }
  // TODO:  'INOUT' types?
  else
  {
    // endpoint 'disabled' now
  }

  SREG = oldSREG; // restore interrupts (etc.)
  
  // TODO:  anything else?

  DEBUG_OUT(F("USB InitEP "));
  DEBUG_OUT((uint32_t)index);
  DEBUG_OUT(F(" (done!)\r\n"));
}

static
void InitEndpoints()
{
  DEBUG_OUT(F("USB InitEndpoints\r\n"));

  // init the first one as a control input
  for (u8 i = 1; i < sizeof(_initEndpoints); i++)
  {
    InitEP(i, pgm_read_byte(_initEndpoints+i), EP_DOUBLE_64); // NOTE:  EP_DOUBLE_64 allocates a 'double bank' of 64 bytes, with 64 byte max length
//    UENUM = i;
//    UECONX = 1;
//    UECFG0X = pgm_read_byte(_initEndpoints+i);
//    UECFG1X = EP_DOUBLE_64;
  }

//  UERST = 0x7E;  // And reset them
//  UERST = 0;

  // TODO:  how do I do this?
}

//  Handle CLASS_INTERFACE requests
static
bool ClassInterfaceRequest(Setup& setup)
{
  u8 i = setup.wIndex;

#ifdef CDC_ENABLED
  if (CDC_ACM_INTERFACE == i)
    return CDC_Setup(setup);
#endif

#ifdef HID_ENABLED
#error NO
  if (HID_INTERFACE == i)
    return HID_Setup(setup);
#endif
  return false;
}

// NOTE:  this is BOGUS.  fix it later
int _cmark;
int _cend;
void InitControl(int end)
{
  SetEP(0);
  _cmark = 0;
  _cend = end;
}

static
bool SendControl(u8 d)
{
//  if (_cmark < _cend)
//  {
//    if (!WaitForINOrOUT())
//      return false;
//    Send8(d);
//    if (!((_cmark + 1) & 0x3F))
//      ClearIN();  // Fifo is full, release this packet
//  }
//  _cmark++;

  Send8(d);
//  if( // POOBAH

  return true;
};

//  Clipped by _cmark/_cend
int USB_SendControl(u8 flags, const void* d, int len)
{
  int sent = len;
  const u8* data = (const u8*)d;
  bool pgm = flags & TRANSFER_PGM;
  while (len--)
  {
    u8 c = pgm ? pgm_read_byte(data++) : *data++;
    if (!SendControl(c))
      return -1;
  }
  return sent;
}

//  Does not timeout or cross fifo boundaries
//  Will only work for transfers <= 64 bytes
//  TODO
int USB_RecvControl(void* d, int len)
{
  WaitOUT();

  // NOTE: if the 
  Recv((u8*)d,len);

  ClearOUT(); // allows me to receive data again

  return len;
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
static
bool SendConfiguration(int maxlen)
{
  DEBUG_OUT(F("USB SendConfiguration\r\n"));

  //  Count and measure interfaces
  InitControl(0);  
  int interfaces = SendInterfaces();
  ConfigDescriptor config = D_CONFIG(_cmark + sizeof(ConfigDescriptor),interfaces);

  //  Now send them
  InitControl(maxlen);
  USB_SendControl(0,&config,sizeof(ConfigDescriptor));
  SendInterfaces();
  return true;
}

u8 _cdcComposite = 0;

static
bool SendDescriptor(Setup& setup)
{
//  DEBUG_OUT(F("USB SendDescriptor\r\n"));

#ifdef LED_SIGNAL1
  digitalWrite(LED_SIGNAL1,digitalRead(LED_SIGNAL1) == LOW ? HIGH : LOW);
#endif // LED_SIGNAL1

  u8 t = setup.wValueH;
  if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
    return SendConfiguration(setup.wLength);

  InitControl(setup.wLength);
#ifdef HID_ENABLED
  if (HID_REPORT_DESCRIPTOR_TYPE == t)
    return HID_GetDescriptor(t);
#endif

  u8 desc_length = 0;
  const u8* desc_addr = 0;
  if (USB_DEVICE_DESCRIPTOR_TYPE == t)
  {
    if (setup.wLength == 8)
      _cdcComposite = 1;
    desc_addr = _cdcComposite ?  (const u8*)&USB_DeviceDescriptorA : (const u8*)&USB_DeviceDescriptor;
  }
  else if (USB_STRING_DESCRIPTOR_TYPE == t)
  {
    if (setup.wValueL == 0)
      desc_addr = (const u8*)&STRING_LANGUAGE;
    else if (setup.wValueL == IPRODUCT) 
      desc_addr = (const u8*)&STRING_IPRODUCT;
    else if (setup.wValueL == IMANUFACTURER)
      desc_addr = (const u8*)&STRING_IMANUFACTURER;
    else
      return false;
  }

  if (desc_addr == 0)
    return false;
  if (desc_length == 0)
    desc_length = pgm_read_byte(desc_addr);

  USB_SendControl(TRANSFER_PGM,desc_addr,desc_length);
  return true;
}

// for debugging only - remove later
static uint8_t led_toggle;


// this assumes I'm locked.  for now, just code for it that way
static void DoTransactionComplete(void)
{
register int i1;

//  if(!(led_toggle++))
  {
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);
  }

  uint8_t oldSREG = SREG; // restore interrupts (etc.)
  cli(); // NO recursion

  DEBUG_DUMP_REGS(&epData);

  for(i1=0; i1 <= MAXEP; i1++)
  {
    if((epData.endpoint[i1].in.ctrl & USB_EP_TYPE_gm) && // 'in' enabled
//       !(epData.endpoint[i1].in.ctrl & USB_EP_INTDSBL_bm) && // ints enabled
       (epData.endpoint[i1].in.status & USB_EP_TRNCOMPL0_bm)) // data sent
    {
      // a write operation has completed

      aEPBuff[i1].iBufIndex2 = 0; // for now *JUST* do this
      aEPBuff[i1].iBufLen2 = 0; // buffer is now "empty"
    }

    if((epData.endpoint[i1].out.ctrl & USB_EP_TYPE_gm) &&     // 'out' enabled
//       !(epData.endpoint[i1].out.ctrl & USB_EP_INTDSBL_bm) && // ints enabled
       (epData.endpoint[i1].out.status & USB_EP_TRNCOMPL0_bm)) // data received
    {
      // a READ operation has completed (TODO  double buffering?)

      aEPBuff[i1].iBufIndex1 = 0; // for now *JUST* do this
      aEPBuff[i1].iBufLen1 = 0x3ff & epData.endpoint[i1].out.cnt; // byte count of received data
      // note that only bits 0 and 1 of the high byte are valid
    }

    epData.endpoint[i1].in.status &= ~USB_EP_TRNCOMPL0_bm;
    epData.endpoint[i1].out.status &= ~USB_EP_TRNCOMPL0_bm;
  }

  SREG = oldSREG; // re-enable interrupts again
}

//  Endpoint 0 interrupt (???) (for the xmega, 'transaction complete')
ISR(USB_TRNCOMPL_vect) // USB_COM_vect) was 'communication event'
{
uint8_t udint;

  uint8_t oldSREG = SREG; // restore interrupts (etc.)
  cli(); // NO recursion

  // for XMega, this will be called whenever you get a completed 'IN' or 'OUT' transaction
  // or whenever you get a completed SETUP transaction.

//  if(!(led_toggle++))
//  {
#ifdef LED_SIGNAL0
    digitalWrite(LED_SIGNAL0,digitalRead(LED_SIGNAL0) == LOW ? HIGH : LOW);
#endif // LED_SIGNAL0
//  }

  udint = *((volatile uint8_t *)&(USB_INTFLAGSBCLR));

  *((volatile uint8_t *)&(USB_INTFLAGSBCLR)) = udint; // clear the flags

  if(udint & USB_TRNIF_bm) // transaction complete
  {
//    DEBUG_OUT(F("USB TRN\r\n"));
    // go through list of active endpoints and mark buffer states

    DoTransactionComplete();
  }

  // The next code *ONLY* handles 'SETUP'

  if (!(udint & USB_SETUPIF_bm))
  {
    SREG = oldSREG; // restore interrupts (etc.)
  
    return; // only handle setup (for now)
  }

  DEBUG_OUT(F("USB setup\r\n"));


  SetEP(0);

  aEPBuff[0].iBufIndex1 = 0; // for now *JUST* do this
  aEPBuff[0].iBufLen1 = 0x3ff & epData.endpoint[0].out.cnt; // byte count of received data
  // note that only bits 0 and 1 of the high byte are valid

  epData.endpoint[0].in.ctrl &= ~USB_EP_SETUP_bm;
  epData.endpoint[0].out.ctrl &= ~USB_EP_SETUP_bm;

/*static*/ Setup setup;

  Recv((u8*)&setup,8);
//  ClearSetupInt(); already done in 1st few lines

// TEMPORARY
  DEBUG_OUT(F("USB setup "));
  DEBUG_OUT((uint32_t)setup.bRequest);
  DEBUG_OUT(F(","));
  DEBUG_OUT((uint32_t)setup.bmRequestType);
  DEBUG_OUT(F("\r\n"));


//  DEBUG_OUT(F("USB setup (2)\r\n"));

  u8 requestType = setup.bmRequestType;
  if (requestType & REQUEST_DEVICETOHOST)
  {
//    WaitIN(); not needed now
  }
  else
  {
    ClearIN();
  }

  bool ok = true;

#if 1
  if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
  {
    //  Standard Requests
    u8 r = setup.bRequest;
    if (GET_STATUS == r)
    {
      Send8(0);    // TODO
      Send8(0);
    }
    else if (CLEAR_FEATURE == r)
    {
    }
    else if (SET_FEATURE == r)
    {
    }
    else if (SET_ADDRESS == r)
    {
//      WaitIN(); not needed

      // this next one is verified
      USB_ADDR = setup.wValueL; // UDADDR = setup.wValueL | (1<<ADDEN);
    }
    else if (GET_DESCRIPTOR == r)
    {
#if 1
      ok = SendDescriptor(setup);
#endif // 0
    }
    else if (SET_DESCRIPTOR == r)
    {
      ok = false;
    }
    else if (GET_CONFIGURATION == r)
    {
      Send8(1);
    }
    else if (SET_CONFIGURATION == r)
    {
      if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
      {
        InitEndpoints();
        _usbConfiguration = setup.wValueL;
      }
      else
      {
        ok = false;
      }
    }
    else if (GET_INTERFACE == r)
    {
    }
    else if (SET_INTERFACE == r)
    {
    }
  }
  else
  {
    InitControl(setup.wLength);    //  Max length of transfer
#if 1 /* this WAS where the problem is */
    ok = ClassInterfaceRequest(setup);
#endif // 0
  }

  if (ok)
    ClearIN();
  else
  {
    Stall();
  }
#endif // 0

  SREG = oldSREG; // restore interrupts (etc.)
}

void USB_Flush(u8 ep)
{
  SetEP(ep);
  if (WriteByteCount())//FifoByteCount())
    ReleaseTX();
}

//  General interrupt (for the xmega, it's 'BUSEVENT' to handle SOF, etc.)
ISR(USB_BUSEVENT_vect) // USB_GEN_vect)
{
  // for the XMega, this is called for any of these:
  // bus events:   SOF, suspend, resume, reset 
  // error events: crc, overflow, underflow, stall

  // NOTE:  this seems to spin a *LOT*  
//  if(!(led_toggle++))
//  {
#ifdef LED_SIGNAL2
    digitalWrite(LED_SIGNAL2,digitalRead(LED_SIGNAL2) == LOW ? HIGH : LOW);
#endif // LED_SIGNAL2
//  }


  u8 udint = *((volatile uint8_t *)&(USB_INTFLAGSACLR));// UDINT;
//  UDINT = 0;
  *((volatile uint8_t *)&(USB_INTFLAGSACLR)) = udint; // this clears all of them (hopefully)

  // on startup of the hardware
  if((udint & (USB_SUSPENDIF_bm | USB_RESUMEIF_bm)) == (USB_SUSPENDIF_bm | USB_RESUMEIF_bm))
  {
    // NOTE:  I have observed that when you power up the USB for the first time, you get an interrupt
    //        in which BOTH the RESUME and SUSPEND bits are set.  This may *not* be a documented behavior
    //        but it happens, and I'd like to do something maybe... ?
    DEBUG_OUT(F("USB powerup\r\n"));
  }
  else if(udint & USB_SUSPENDIF_bm)
  {
    // NOTE:  I get several suspend/resume combos after pulling the USB cable out

    DEBUG_OUT(F("USB suspend\r\n"));

    // TODO:  use THIS to detect 'connected' - if the frame # has not changed,
    //        then I'm no longer connected.

    DEBUG_DUMP_REGS(&epData);
  }
  else if(udint & USB_RESUMEIF_bm)
  {
    // NOTE:  I get several suspend/resume combos after pulling the USB cable out

    DEBUG_OUT(F("USB resume\r\n"));

    // TODO:  use THIS to detect 'connected' - if the frame # has not changed,
    //        then I'm no longer connected.

    DEBUG_DUMP_REGS(&epData);
  }

  //  End of Reset - happens when you first plug in, etc.
  if(udint & USB_RSTIF_bm) //(1<<EORSTI))
  {
    DEBUG_OUT(F("USB RST\r\n"));
//    DEBUG_OUT((uint32_t)USB_ADDR);
//    DEBUG_OUT(F("\r\n"));

    USB_ADDR = 0; // set USB address to 0 on reset.  not sure if this is necessary

    _usbConfiguration = 0;      // not configured yet (moved)

    // TODO:  see if endpoint 0 needs to be re-done or not, perhaps just leaving it
    //        'as-is' may be MORE stable than re-doing it every! single! time!
    InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);  // init ep0
//    _usbConfiguration = 0;      // not configured yet (was here)

    // clear any 'stall' event this might cause
    *((volatile uint8_t *)&(USB_INTFLAGSACLR)) = USB_STALLIF_bm;

// xmega will do this automatically
//    UEIENX = 1 << RXSTPE;      // Enable interrupts for ep0

    DEBUG_DUMP_REGS(&epData);
  }

  //  Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
  if(udint & USB_SOFIF_bm)//(1<<SOFI))
  {
//    DEBUG_OUT(F("USB SOF"));
//    DEBUG_OUT((uint32_t)(epData.framenum));
//    DEBUG_OUT(F("\r\n"));

//    epData.framenum = 0x7ff; // temporary
// removed - will find a better way
//#ifdef CDC_ENABLED
//    USB_Flush(CDC_TX);        // Send a tx frame if found
//    ReleaseTX(); maybe???
//#endif

#ifdef TX_RX_LED_INIT    
    // check whether the one-shot period has elapsed.  if so, turn off the LED
    if (TxLEDPulse && !(--TxLEDPulse))
      TXLED0;
    if (RxLEDPulse && !(--RxLEDPulse))
      RXLED0;
#endif // TX_RX_LED_INIT

//    if(epData.endpoint[0].out.status)
//    {
#ifdef LED_SIGNAL3
//      digitalWrite(LED_SIGNAL3,digitalRead(LED_SIGNAL3) == LOW ? HIGH : LOW);
#endif // LED_SIGNAL3
//
//      // TEMPORARY clear control endpoint bits
//      epData.endpoint[0].out.status = 0;
//    }

    DoTransactionComplete(); // maybe *THIS* will help?
  }

//  if(udint & ~(USB_RSTIF_bm | USB_SOFIF_bm)) // anything else
//  {
//    DEBUG_OUT(F("USB other "));
//    DEBUG_OUT((uint32_t)udint);
//    DEBUG_OUT(F("\r\n"));
////#ifdef LED_SIGNAL3
//    digitalWrite(LED_SIGNAL3,digitalRead(LED_SIGNAL3) == LOW ? HIGH : LOW);
////#endif // LED_SIGNAL3
//  }

}

// this function removed - it's potentially a 'hang' problem
////  VBUS or counting frames
////  Any frame counting?
//u8 USBConnected()
//{
//  u8 f = FrameNumber(); // UDFNUML;
//  delay(3); // in an ISR, this is FATAL!
//  return f != FrameNumber(); // UDFNUML;
//}

//=======================================================================
//=======================================================================

USBDevice_ USBDevice;

USBDevice_::USBDevice_()
{
  DEBUG_OUT(F("USB - device const\r\n"));
}

void USBDevice_::attach()
{
  DEBUG_OUT(F("USB - attach\r\n"));

  uint8_t oldSREG = SREG; // save int flag
  cli(); // NO recursion

  USB_INTCTRLA = 0;
  USB_INTCTRLB = 0;

  USB_INTFLAGSACLR = 0xff; // clear all int flags
  USB_INTFLAGSBCLR = 0x3;  // clear all int flags

  _usbConfiguration = 0;

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

//  DEBUG_OUT(F("CalData = "));
//  DEBUG_OUT((uint32_t) USB_CAL0);
//  DEBUG_OUT(F(","));
//  DEBUG_OUT((uint32_t) USB_CAL1);
//  DEBUG_OUT(F("\r\n"));

  // set the max # of endpoints, speed, and 'store frame number' flags
  USB_CTRLA = MAXEP /* max # of endpoints minus 1 */
#ifdef FAST_USB
            | USB_SPEED_bm /* all ahead 'FULL' - aka 'FULL' speed ahead! */
#endif // FAST_USB
            | USB_STFRNUM_bm // store the frame number
   // TODO:  FIFO ?
            ;

  memset(&epData, 0, sizeof(epData));

// debug code - remove later
//#ifdef A1U_SERIES
//  if((uint16_t)(uint8_t *)&(epData.endpoint[0]) & 15)
//  {
//    DEBUG_OUT(F("WRONG ALIGNMENT\r\n"));
//    return;
//  }
//#endif // A1U_SERIES

  USB_EPPTR = (uint16_t)(uint8_t *)&(epData.endpoint[0]); // set 'EP Data' pointer to THIS address

  USB_ADDR = 0; // set USB address to 0 (default before 'SETUP')

  // LAST of all, enable interrupts
  USB_INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;    // enable 'transaction complete' and 'setup' interrupts
  USB_INTCTRLA  = USB_SOFIE_bm                     // enable the start of frame interrupt
                | USB_BUSEVIE_bm                   // 'bus event' interrupt - suspend, resume, reset
                                                   // for the RESET event, RESETIF will be set (20.14.11 in AU manual)
                | USB_INTLVL0_bm | USB_INTLVL1_bm; // int level 3

  // NOW enable the USB
  USB_CTRLA |= USB_ENABLE_bm;

  SREG = oldSREG; // restore int flags

//  __builtin_avr_delay_cycles(F_CPU / 2); // delay approximately 100 msec

  // attach the wiring for D- and D+

  USB_CTRLB = USB_ATTACH_bm; // attach D- and D+ (also enables pullup resistors based on speed)
  // on the 128A1U, this is PD6 (D-) and PD7 (D+) [YMMV on the other processors]
  // this is partly why it's good to use PORTC for the primary SPI, TWI, etc.
  // since PORTD gets used for 'other things', but it's STILL ok to use PD2,3 as a serial port
  // and so the default pin config works pretty well.



#if 0 /* old code for mega */
  UHWCON = 0x01;            // power internal reg
  USBCON = (1<<USBE)|(1<<FRZCLK);    // clock frozen, usb enabled
#if F_CPU == 16000000UL
  PLLCSR = 0x12;            // Need 16 MHz xtal
#elif F_CPU == 8000000UL
  PLLCSR = 0x02;            // Need 8 MHz xtal
#endif
  while (!(PLLCSR & (1<<PLOCK)))    // wait for lock pll
    ;

  // Some tests on specific versions of macosx (10.7.3), reported some
  // strange behaviuors when the board is reset using the serial
  // port touch at 1200 bps. This delay fixes this behaviour.
  delay(1);

  USBCON = ((1<<USBE)|(1<<OTGPADE));  // start USB clock
  UDIEN = (1<<EORSTE)|(1<<SOFE);    // Enable interrupts for EOR (End of Reset) and SOF (start of frame)
  UDCON = 0;              // enable attach resistor
#endif // 0

#ifdef TX_RX_LED_INIT  
  TX_RX_LED_INIT;
#endif // TX_RX_LED_INIT

  DEBUG_OUT(F("USB Attach (done)\r\n"));
}

void USBDevice_::detach()
{
  DEBUG_OUT(F("USB - detach\r\n"));

  USB_INTCTRLA = 0; // disabling interrupts
  USB_INTCTRLB = 0;
  USB_CTRLA = 0; // shut down USB
  USB_CTRLB = 0; // detach D- and D+
  CLK_USBCTRL = 0; // shut off USB clock

  DEBUG_OUT(F("USB Detach\r\n"));
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
  return _usbConfiguration;
}

void USBDevice_::poll()
{
}

#endif /* if defined(USBCON) */

