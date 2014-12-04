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

#if defined(USBCON)

#define EP_TYPE_CONTROL        0x00
#define EP_TYPE_BULK_IN        0x81
#define EP_TYPE_BULK_OUT      0x80
#define EP_TYPE_INTERRUPT_IN    0xC1
#define EP_TYPE_INTERRUPT_OUT    0xC0
#define EP_TYPE_ISOCHRONOUS_IN    0x41
#define EP_TYPE_ISOCHRONOUS_OUT    0x40

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
#define TX_RX_LED_PULSE_MS 100
volatile u8 TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
volatile u8 RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */

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
#warning using Arduino USB Vendor ID - do NOT ship product with this ID!!!
#elif USB_VID == 0x1b4f
  'S','p','a','r','k','F','u','n',' ',' ',' '
#warning using SparkFun USB Vendor ID - do NOT ship product with this ID!!!
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

//==================================================================
//==================================================================


// NOTE:  PR_PRGEN should have PR_USB_bm bit cleared to enable the USB

volatile u8 _usbConfiguration = 0;

static XMegaEPDataStruct epData;

static uint8_t endpointIndex = 0;

static inline void WaitIN(void)
{
//  while (!(UEINTX & (1<<TXINI)));

  // must call 'setEP' beforehand
  while(!(epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm)) { }

// TODO:  USB_EP_TRNCOMPL1_bm as well?
}

static inline void ClearIN(void)
{
//  UEINTX = ~(1<<TXINI);
  epData.endpoint[endpointIndex].in.status &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_TRNCOMPL1_bm);
}

static inline void WaitOUT(void)
{
//  while (!(UEINTX & (1<<RXOUTI)))
//    ;
  while(!(epData.endpoint[endpointIndex].out.status & USB_EP_TRNCOMPL0_bm)) { }

// TODO:  USB_EP_TRNCOMPL1_bm as well?
}

static inline u8 WaitForINOrOUT()
{
//  while (!(UEINTX & ((1<<TXINI)|(1<<RXOUTI))))
//    ;
//  return (UEINTX & (1<<RXOUTI)) == 0;

  while(!(epData.endpoint[endpointIndex].out.status & USB_EP_TRNCOMPL0_bm) &&
        !(epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm)) { }

// TODO:  USB_EP_TRNCOMPL1_bm as well?

  return (epData.endpoint[endpointIndex].in.status & USB_EP_TRNCOMPL0_bm) == 0;
}

static inline void ClearOUT(void)
{
//  UEINTX = ~(1<<RXOUTI);

  epData.endpoint[endpointIndex].out.status &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_TRNCOMPL1_bm);
}

void Recv(volatile u8* data, u8 count)
{
  while (count--)
    *data++ = UEDATX;
  
  RXLED1;          // light the RX LED
  RxLEDPulse = TX_RX_LED_PULSE_MS;  
}

static inline u8 Recv8()
{
  RXLED1;          // light the RX LED
  RxLEDPulse = TX_RX_LED_PULSE_MS;

  return UEDATX;  
}

static inline void Send8(u8 d)
{
  UEDATX = d;
}

static inline void SetEP(u8 ep)
{
//  UENUM = ep;
  endpointIndex = ep; // now a var, as an index into 'epData'
}

static inline u8 FifoByteCount()
{
  return UEBCLX;
}

static inline u8 ReceivedSetupInt()
{
//  return UEINTX & (1<<RXSTPI);
  return USB_INTFLAGSBCLR & USB_SETUPIF_bm;
}

static inline void ClearSetupInt()
{
//  UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
  USB_INTFLAGSBCLR = USB_SETUPIF_bm; // clear the bit
}

static inline void Stall()
{
  UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline u8 ReadWriteAllowed()
{
//  return UEINTX & (1<<RWAL);
  return 1; // for now, since it's buffered I/O
}

static inline u8 Stalled()
{
//  return UEINTX & (1<<STALLEDI);
  return USB_INTFLAGSACLR & USB_STALLIF_bm;
}

static inline u8 FifoFree()
{
//  return UEINTX & (1<<FIFOCON);
}

static inline void ReleaseRX()
{
//  UEINTX = 0x6B;  // FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
//  UEINTX = 0x3A;  // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
}

static inline u8 FrameNumber()
{
  // return UDFNUML;
  return epData.framenum.l; // NOTE:  why ONLY the low byte?  ah, well, that's what the OLD code did...
}

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
    cli();
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
  return FifoByteCount();
}

//  Non Blocking receive
//  Return number of bytes read
int USB_Recv(u8 ep, void* d, int len)
{
  if (!_usbConfiguration || len < 0)
    return -1;
  
  LockEP lock(ep);
  u8 n = FifoByteCount();
  len = min(n,len);
  n = len;
  u8* dst = (u8*)d;
  while (n--)
    *dst++ = Recv8();
  if (len && !FifoByteCount())  // release empty buffer
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
  return 64 - FifoByteCount();
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
  TXLED1;          // light the TX LED
  TxLEDPulse = TX_RX_LED_PULSE_MS;
  return r;
}

extern const u8 _initEndpoints[] PROGMEM;
const u8 _initEndpoints[] = 
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

#define EP_SINGLE_64 0x32  // EP0
#define EP_DOUBLE_64 0x36  // Other endpoints

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
  uint8_t aBuf1[INTERNAL_BUFFER_LENGTH];
  uint8_t aBuf2[INTERNAL_BUFFER_LENGTH];
} INTERNAL_BUFFER;

INTERNAL_BUFFER aEPBuff[INTERNAL_NUM_EP];

static
void InitEP(u8 index, u8 type, u8 size)
{
//  UENUM = index;     index, allows other bits to apply to correct endpoing (0-6, 7 not allowed)
//  UECONX = 1;        only sets 'EPEN' bit, aka 'endpoint enable'
//  UECFG0X = type;    bits 7:6 == 'EPTYPE' - 00=control, 01=isochronous, 02=bulk, 03=interrupt
//  UECFG1X = size;    8, 16, 32, 64 etc. - see 32u4 manual pg 267

  if(index > MAXEP)  // MAXEP will be the highest index (inclusive)
  {
    return;
  } 

  if(type == EP_TYPE_INTERRUPT_IN || type == EP_TYPE_BULK_IN
     || type == EP_TYPE_ISOCHRONOUS_IN)
  {
    epData[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)

    epData[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)

    epData[index].in.dataptr = &(aEPBuff[index].aBuf1[0]);
    epData[index].in.auxdata = &(aEPBuff[index].aBuf2[0]); // for now, later ???

    epData[index].in.ctr = 0; // no data (so I can receive)

    epData[index].in.ctrl = (type == EP_TYPE_ISOCHRONOUS_IN ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                          | (type == EP_TYPE_BULK_IN ? USB_EP_INTDSBL_bm : 0)      /* disable interrupt */
                          | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */
  }
  else if(type == EP_TYPE_INTERRUPT_OUT || type == EP_TYPE_BULK_OUT)
  {
    epData[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)

    epData[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)

    epData[index].out.dataptr = &(aEPBuff[index].aBuf1[0]);
    epData[index].out.auxdata = &(aEPBuff[index].aBuf2[0]); // for now, later ???

    epData[index].out.ctr = 0; // no data (so I won't send)

    epData[index].out.ctrl = (type == EP_TYPE_ISOCHRONOUS_OUT ? USB_EP_TYPE_ISOCHRONOUS_gc : USB_EP_TYPE_BULK_gc)
                           | (type == EP_TYPE_BULK_OUT ? USB_EP_INTDSBL_bm : 0)      /* disable interrupt */
                           | (size == EP_DOUBLE_64 ? USB_EP_SIZE_64_gc : 0);         /* data size */
  }
  else if(type == EP_TYPE_CONTROL) // control
  {
    // aBuf1 is output, aBuf2 is input

    epData[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)
    epData[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)

    epData[index].in.dataptr = &(aEPBuff[index].aBuf2[0]);
    epData[index].in.auxdata = 0;

    epData[index].in.ctr = 0; // no data (so I can receive)

    // TODO:  verify this
    epData[index].out.dataptr = &(aEPBuff[index].aBuf1[0]);
    epData[index].out.auxdata = 0;
    epData[index].out.ctr = 0; // no data (so I won't send)

    // NOTE:  size will be sent as 'EP_SINGLE_64'

    // do I do 'out' as well or just 'in' ??
    epData[index].in.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                          | (size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */

    epData[index].out.ctrl = USB_EP_TYPE_CONTROL_gc // NOTE:  interrupt enabled
                           | (size == EP_SINGLE_64 ? USB_EP_SIZE_64_gc : 0);        /* data size */
  }
  else
  {
    // disable the endpoint
    epData[index].out.ctrl = USB_EP_TYPE_DISABLE_gc; // to disable it (endpoint 'type 0' disables)
    epData[index].in.ctrl = USB_EP_TYPE_DISABLE_gc; // initially (disable)
  }
  

}

static
void InitEndpoints()
{
  // init the first one as a control input
  for (u8 i = 1; i < sizeof(_initEndpoints); i++)
  {
    InitEP(i, pgm_read_byte(_initEndpoints+i), EP_DOUBLE_64); // NOTE:  EP_DOUBLE_64 allocates a 'double bank' of 64 bytes, with 64 byte max length
//    UENUM = i;
//    UECONX = 1;
//    UECFG0X = pgm_read_byte(_initEndpoints+i);
//    UECFG1X = EP_DOUBLE_64;
  }

  UERST = 0x7E;  // And reset them
  UERST = 0;
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
  if (HID_INTERFACE == i)
    return HID_Setup(setup);
#endif
  return false;
}

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
  if (_cmark < _cend)
  {
    if (!WaitForINOrOUT())
      return false;
    Send8(d);
    if (!((_cmark + 1) & 0x3F))
      ClearIN();  // Fifo is full, release this packet
  }
  _cmark++;
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
  Recv((u8*)d,len);
  ClearOUT();
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

//  Endpoint 0 interrupt (???) (for the xmega, 'transaction complete')
ISR(USB_TRNCOMPL_vect) // USB_COM_vect) was 'communication event'
{
  // for XMega, this will be called whenever you get a completed 'IN' or 'OUT' transaction
  // or whenever you get a completed SETUP transaction.

  SetEP(0);

  // THIS code *ONLY* handles 'SETUP'
  if (!ReceivedSetupInt())
    return;

  Setup setup;
  Recv((u8*)&setup,8);
  ClearSetupInt();

  u8 requestType = setup.bmRequestType;
  if (requestType & REQUEST_DEVICETOHOST)
  {
    WaitIN();
  }
  else
  {
    ClearIN();
  }

  bool ok = true;

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
      WaitIN();
      UDADDR = setup.wValueL | (1<<ADDEN);
    }
    else if (GET_DESCRIPTOR == r)
    {
      ok = SendDescriptor(setup);
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
    ok = ClassInterfaceRequest(setup);
  }

  if (ok)
    ClearIN();
  else
  {
    Stall();
  }
}

void USB_Flush(u8 ep)
{
  SetEP(ep);
  if (FifoByteCount())
    ReleaseTX();
}

//  General interrupt (for the xmega, it's 'BUSEVENT' to handle SOF, etc.)
ISR(USB_BUSEVENT_vect) // USB_GEN_vect)
{
  // for the XMega, this is called for any of these:
  // bus events:   SOF, suspend, resume, reset 
  // error events: crc, overflow, underflow, stall

  u8 udint = USB_INTFLAGSACLR;// UDINT;
//  UDINT = 0;
  USB_INTFLAGSACLR = udint; // this clears all of them

  //  End of Reset
  if(udint & USB_RSTIF_bm)//(1<<EORSTI))
  {
    _usbConfiguration = 0;      // not configured yet (moved)
    InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);  // init ep0
//    _usbConfiguration = 0;      // not configured yet (was here)

// xmega will do this automatically
//    UEIENX = 1 << RXSTPE;      // Enable interrupts for ep0
  }

  //  Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
  if(udint & USB_SOFIF_bm)//(1<<SOFI))
  {
#ifdef CDC_ENABLED
    USB_Flush(CDC_TX);        // Send a tx frame if found
#endif
    
    // check whether the one-shot period has elapsed.  if so, turn off the LED
    if (TxLEDPulse && !(--TxLEDPulse))
      TXLED0;
    if (RxLEDPulse && !(--RxLEDPulse))
      RXLED0;
  }
}

//  VBUS or counting frames
//  Any frame counting?
u8 USBConnected()
{
  u8 f = FrameNumber(); // UDFNUML;
  delay(3);
  return f != FrameNumber(); // UDFNUML;
}

//=======================================================================
//=======================================================================

USBDevice_ USBDevice;

USBDevice_::USBDevice_()
{
}

void USBDevice_::attach()
{
  _usbConfiguration = 0;

  // enable the USB clock using the 32mhz RC oscillator
  // assume either slow (6mhz) or fast (48mhz)
  // and of course the pre-scaler must be assigned accordingly
  // Also, assume that the oscillator is *SET UP* *PROPERLY* already, and
  // that all I have to do is configure the PLL to run at 48Mhz

  // setting up the PLL - source is RC32M 'divided by 4' then multiplied by 6

  USB_CTRLA = 0; // shut down USB
  USB_CTRLB = 0; // detach D- and D+
  CLK_USBCTRL = 0; // shut off USB clock
  OSC_CTRL &= ~(OSC_PLLEN_bm); // disable PLL osc

  // now config PLL and USB clock stuff
  OSC_PLLCTRL = OSC_PLLSRC_RC32M_gc | 6; /* 6 being the multiplication factor for the PLL */

  OSC_CTRL |= OSC_PLLEN_bm; // re-enable PLL

  while(!(OSC_STATUS & OSC_PLLRDY_bm))
  {
    // TODO:  timeout?
  }
  

  CCP = CCP_IOREG_gc; // 0xd8 - see D manual, sect 3.14.1 (protected I/O)
#ifdef FAST_USB
  CLK_USBCTRL = CLK_USBSRC_PLL_gc | CLK_USBSEN_bm;

#else // SLOW
  CLK_USBCTRL = CLK_USBPSDIV_8_gc // divide by 8 for 6mhz operation
              | CLK_USBSRC_PLL_gc
              | CLK_USBSEN_bm;

#endif // FAST_USB or SLOW

  // assign CAL register from product signatures
  USB_CAL0 = readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_USBCAL0);
  USB_CAL1 = readCalibrationData((uint8_t)(uint16_t)&PRODSIGNATURES_USBCAL1);

  USB_CTRLA = MAXEP /* max # of endpoints minus 1 */
#ifdef FAST_USB
            | USB_SPEED_bm
#endif // FAST_USB
            | USB_STFRNUM_bm // store the frame number
   // TODO:  FIFO ?
            ;

  USB_CTRLB = USB_ATTACH_bm; // attach D- and D+ (also enables pullup resistors based on speed)
  // on the 128A1U, this is PD6 (D-) and PD7 (D+) [YMMV on the other processors]
  // this is partly why it's good to use PORTC for the primary SPI, TWI, etc.
  // since PORTD gets used for 'other things', but it's STILL ok to use PD2,3 as a serial port

  memset(&epData, 0, sizeof(epData));
  USB_EPPTR = (uint16_t)&(epData.endpoint[0]); // set 'EP Data' pointer to THIS address


  USB_INTFLAGSACLR = 0xff; // clear all int flags

  // enable interrupts
  USB_INTCTRLA  = USB_SOFIE_bm                     // enable the start of frame interrupt
                | USB_BUSEVIE_bm                   // 'bus event' interrupt - suspend, resume, reset
                                                   // for the RESET event, RESETIF will be set (20.14.11 in AU manual)
                | USB_INTLVL0_bm | USB_INTLVL1_bm; // int level 3

  USB_INTCTRLB = 0;

  USB_CTRLA |= USB_ENABLE_bm; // enable the USB interface           

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
  
  TX_RX_LED_INIT;
#endif // 0

}

void USBDevice_::detach()
{
  USB_INTCTRLA = 0; // disabling interrupts
  USB_INTCTRLB = 0;
  USB_CTRLA = 0; // shut down USB
  USB_CTRLB = 0; // detach D- and D+
  CLK_USBCTRL = 0; // shut off USB clock
}

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

