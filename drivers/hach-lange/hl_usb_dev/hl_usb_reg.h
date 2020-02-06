#ifndef _HL_USB_REG_H_
#define _HL_USB_REG_H_

#define PORT_CTRL (port_ctrl)
#define PORT_DATA (port_data)

//------------------------------------------------------------------
// Write/Read config
//------------------------------------------------------------------
#define WriteCtrlOutCfg 0x20 // Write Control OUT Configuration, Endpoint Configuration Register endpoint 0 OUT,  write 1 byte
#define WriteCtrlInCfg  0x21 // Write Control IN Configuration, Endpoint Configuration Register endpoint 0 IN,  write 1 byte
#define WriteEP1Cfg     0x22 // Write Endpoint n Configuration, Endpoint Configuration Register endpoint 1, write 1 byte
#define WriteEP2Cfg     0x23
#define WriteEP3Cfg     0x24
#define WriteEP4Cfg     0x25
#define WriteEP5Cfg     0x26
#define WriteEP6Cfg     0x27
#define WriteEP7Cfg     0x28
#define WriteEP8Cfg     0x29
#define WriteEP9Cfg     0x2A
#define WriteEP10Cfg    0x2B
#define WriteEP11Cfg    0x2C
#define WriteEP12Cfg    0x2D
#define WriteEP13Cfg    0x2E
#define WriteEP14Cfg    0x2F

#define ReadCtrlOutCfg  0x30 // Read Control OUT Configuration, Endpoint Configuration Register endpoint 0 OUT, read 1 byte
#define ReadCtrlInCfg   0x31 // Read Control IN Configuration, Endpoint Configuration Register endpoint 0 IN, read 1 byte
#define ReadEP1Cfg      0x32 // Read Endpoint n Configuration, Endpoint Configuration Register endpoint 1, read 1 byte
#define ReadEP2Cfg      0x33
#define ReadEP3Cfg      0x34
#define ReadEP4Cfg      0x35
#define ReadEP5Cfg      0x36
#define ReadEP6Cfg      0x37
#define ReadEP7Cfg      0x38
#define ReadEP8Cfg      0x39
#define ReadEP9Cfg      0x3A
#define ReadEP10Cfg     0x3B
#define ReadEP11Cfg     0x3C
#define ReadEP12Cfg     0x3D
#define ReadEP13Cfg     0x3E
#define ReadEP14Cfg     0x3F

// EP config bits
#define FIFOEN 0x80 // 1=enabled FIFO with allocated mem, 0=disabled FIFO
#define EPDIR  0x40 // EP direction: 0=out, 1=in or DMA direction: 0=read, 1=write
#define DBLBUF 0x20 // 1=double buffering
#define FFOISO 0x10 // 1=isochronous, 0=bulk/interrupt
#define FFOSZ  0x0f // FIFO size: 0000=8B, 0001=16B, 0010=32B, 0011=64B


//------------------------------------------------------------------
// Write/Read device addr
//------------------------------------------------------------------
#define WriteDevAddr    0xB6 // Write/Read Device Address, Address Register, write/read 1 byte
#define ReadDevAddr     0xB7

// address bits
#define DEVEN  0x80 // 1=device enabled
#define DEVADR 0x7f // device address


//------------------------------------------------------------------
// Write/Read mode
//------------------------------------------------------------------
#define WriteMode       0xB8 // Write/Read Mode Register, Mode Register, write/read 1 byte
#define ReadMode        0xB9  

// mode bits
#define DMAWD  0x80 // 1=16b DMA bus, 0=8b DMA bus
#define GOSUSP 0x20 // Writing '1' followed by '0' activates 'suspend'
#define INTENA 0x08 // 1=enable all interrupts
#define DBGMOD 0x04 // 1=enable interrupts on NAK and errors
#define SOFTCT 0x01 // 1=SoftConnect (ignored if EXTPUL = 1, HW config reg)

//------------------------------------------------------------------
// Write/Read HW config
//------------------------------------------------------------------
#define WriteHwCfg      0xBA // Write/Read Hardware Configuration, Hardware Configuration Register, write/read 2 bytes
#define ReadHwCfg       0xBB

// HW config bits
#define EXTPUL 0x4000 // 1=pull-up resistor used on pin D+ (Softconnect not use)
#define NOLAZY 0x2000 // 1=disable CLKOUT of LazyClock freq during 'suspend' state
#define CLKRUN 0x1000 // 1=ext clock always running, during 'suspend' state this must be made '0'
#define CLKDIV 0x0f00 // 48/(N+1) with N=0 to 15 with a reset value of 12MHz (N=3)
#define DAKOLY 0x0080 // 1=DACK-only DMA, 0=8237 compatible DMA
#define DRQPOL 0x0040 // 1=DREQ signal polarity active HIGH, 0=active LOW
#define DAKPOL 0x0020 // 1=DACK "
#define EOTPOL 0x0010 // 1=EOT  "
#define WKUPCS 0x0008 // 1=enable remote wake-up via LOW level input /CS (VBus must be present)
#define PWROFF 0x0004 // 1=enable powering off during 'suspend' state (controls output SUSPEND)
#define INTLVL 0x0002 // 1=interrupt signalling mode on output INT pulsed, 0=level
#define INTPOL 0x0001 // 1=INT signal polarity active HIGH, 0=active LOW


//------------------------------------------------------------------
// Write/Read interrupt enable
//------------------------------------------------------------------
#define WriteIntEnable  0xC2 // Write/Read Interrupt Enable Register, Interrupt Enable Register, write/read 4 bytes
#define ReadIntEnable   0xC3

#define IEP14           0x00800000 // 1=enable interrupt from indicated EP
//...
#define IEP5            0x00004000
//...
#define IEP3            0x00001000
#define IEP2            0x00000800
#define IEP1            0x00000400
#define IEP0IN          0x00000200 // 1=enable int from control IN EP
#define IEP0OUT         0x00000100
#define SP_IEEOT        0x00000040 // 1=enable int upon detection of short packet
#define IEPSOF          0x00000020 // 1=enable 1ms interrupt upon detection of Pseudo SOF
#define IESOF           0x00000010 // 1=enable int upon detection of SOF
#define IEEOT           0x00000008 // 1=          "                  EOT
#define IESUSP          0x00000004 // 1=          "                  'suspend' state
#define IERESM          0x00000002 // 1=          "                  'resume' state
#define IERST           0x00000001 // 1=          "                  bus reset

//------------------------------------------------------------------
// Write/Read DMA config
//------------------------------------------------------------------
#define WriteDmaCfg     0xF0 // Write/Read DMA Configuration, DMA Configuration Register, write/read 2 bytes
#define ReadDmaCfg      0xF1

#define CNTREN          0x8000 // 1=enable generation of EOT when DMA counter reaches 0
#define SHORTP          0x4000 // 1=enable generation of EOT when short/empty packet received
#define EPDIX           0x00f0 // indicates destination EP for DMA
#define DMAEN           0x0008 // 1=DMA enabled
#define BURSTL          0x0003 // selects DMA burst length: 00=single cycle 1B, 01=burst 4B, 10=burst 8B, 11=burst 16B

//------------------------------------------------------------------
// Write/Read DMA counter
//------------------------------------------------------------------
#define WriteDmaCnt     0xF2 // Write/Read DMA Counter, DMA Counter Register, write/read 2 bytes
#define ReadDmaCnt      0xF3

//------------------------------------------------------------------
// Reset
//------------------------------------------------------------------
#define ResetDev        0xF6 // Reset Device resets all registers

//------------------------------------------------------------------
// Write/Read data
//------------------------------------------------------------------
#define WriteCtrlInBuf  0x01 // Write Control IN Buffer, FIFO endpoint 0 IN, N <= 64 bytes
#define WriteEP1Buf     0x02 // Write Endpoint n Buffer, FIFO endpoint 1 (IN endpoints only), isochronous: N <= 1023 bytes, interrupt/bulk: N <= 64 bytes
#define WriteEP2Buf     0x03
#define WriteEP3Buf     0x04
#define WriteEP4Buf     0x05
#define WriteEP5Buf     0x06
#define WriteEP6Buf     0x07
#define WriteEP7Buf     0x08
#define WriteEP8Buf     0x09
#define WriteEP9Buf     0x0A
#define WriteEP10Buf    0x0B
#define WriteEP11Buf    0x0C
#define WriteEP12Buf    0x0D
#define WriteEP13Buf    0x0E
#define WriteEP14Buf    0x0F

#define ReadCtrlOutBuf  0x10 // Read Control OUT Buffer, FIFO endpoint 0 OUT, N <= 64 bytes
#define ReadEP1Buf      0x12 // Read Endpoint n Buffer, FIFO endpoint 1 (OUT endpoints only), isochronous: N <= 1023 bytes, interrupt/bulk: N <= 64 bytes
#define ReadEP2Buf      0x13
#define ReadEP3Buf      0x14
#define ReadEP4Buf      0x15
#define ReadEP5Buf      0x16
#define ReadEP6Buf      0x17
#define ReadEP7Buf      0x18
#define ReadEP8Buf      0x19
#define ReadEP9Buf      0x1A
#define ReadEP10Buf     0x1B
#define ReadEP11Buf     0x1C
#define ReadEP12Buf     0x1D
#define ReadEP13Buf     0x1E
#define ReadEP14Buf     0x1F


//------------------------------------------------------------------
// Stall/Unstall EP
//------------------------------------------------------------------
#define StallCtrlOutEP  0x40 // Stall Control OUT Endpoint, Endpoint 0 OUT, 
#define StallCtrlInEP   0x41 // Stall Control IN Endpoint, Endpoint 0 IN
#define StallEP1        0x42 // Stall Endpoint n 
#define StallEP2        0x43
#define StallEP3        0x44
#define StallEP4        0x45
#define StallEP5        0x46
#define StallEP6        0x47
#define StallEP7        0x48
#define StallEP8        0x49
#define StallEP9        0x4A
#define StallEP10       0x4B
#define StallEP11       0x4C
#define StallEP12       0x4D
#define StallEP13       0x4E
#define StallEP14       0x4F

#define UnstallCtrlOutEP 0x80 // Unstall Control OUT Endpoint, Endpoint 0 OUT, 
#define UnstallCtrlInEP  0x81 // Unstall Control IN Endpoint, Endpoint 0 IN
#define UnstallEP1       0x82 // Unstall Endpoint n
#define UnstallEP2       0x83
#define UnstallEP3       0x84
#define UnstallEP4       0x85
#define UnstallEP5       0x86
#define UnstallEP6       0x87
#define UnstallEP7       0x88
#define UnstallEP8       0x89
#define UnstallEP9       0x8A
#define UnstallEP10      0x8B
#define UnstallEP11      0x8C
#define UnstallEP12      0x8D
#define UnstallEP13      0x8E
#define UnstallEP14      0x8F

//------------------------------------------------------------------
// Read EP state (will clear IEPn of corresponding EP)
//------------------------------------------------------------------
#define ReadCtrlOutStat 0x50 // Read Control OUT Status, Endpoint Status Register, endpoint 0 OUT, read 1 byte
#define ReadCtrlInStat  0x51 // Read Control IN Status, Endpoint Status Register, endpoint 0 IN, read 1 byte
#define ReadEP1Stat     0x52 // Read Endpoint n Status, Endpoint Status Register n, endpoint n, read 1 byte
#define ReadEP2Stat     0x53
#define ReadEP3Stat     0x54
#define ReadEP4Stat     0x55
#define ReadEP5Stat     0x56
#define ReadEP6Stat     0x57
#define ReadEP7Stat     0x58
#define ReadEP8Stat     0x59
#define ReadEP9Stat     0x5A
#define ReadEP10Stat    0x5B
#define ReadEP11Stat    0x5C
#define ReadEP12Stat    0x5D
#define ReadEP13Stat    0x5E
#define ReadEP14Stat    0x5F

// EP state bits
#define EPSTAL          0x80 // 1=EP stalled
#define EPFULL1         0x40 // 1=2nd EP buffer full
#define EPFULL0         0x20 // 1=1st EP buffer full
#define DATA_PID        0x10 // data PID of next packet (1=DATA1 PID, 0=DATA0 PID)
#define OVERWRITE       0x08 // 1=setup overwrites previous setup w/o previous ACK nor EP being stalled...
#define SETUPT          0x04 // 1=buffer contains setup packet
#define CPUBUF          0x02 // indicates which buffer is selected for data transfer (0=1st buffer, 1=2nd buffer)

//------------------------------------------------------------------
// Validate data (for transmission to USB host)
//------------------------------------------------------------------
#define ValidateCtrlInBuf 0x61 // Validate Control IN Buffer, FIFO endpoint 0 IN
#define ValidateEP1       0x62 // Validate Endpoint n Buffer, FIFO endpoint n
#define ValidateEP2       0x63
#define ValidateEP3       0x64
#define ValidateEP4       0x65
#define ValidateEP5       0x66
#define ValidateEP6       0x67
#define ValidateEP7       0x68
#define ValidateEP8       0x69
#define ValidateEP9       0x6A
#define ValidateEP10      0x6B
#define ValidateEP11      0x6C
#define ValidateEP12      0x6D
#define ValidateEP13      0x6E
#define ValidateEP14      0x6F

//------------------------------------------------------------------
// Clear buffer (allow reception of new packets)
//------------------------------------------------------------------
#define ClearCtrlOutBuf  0x70 // Clear Control OUT Buffer, FIFO endpoint 0 OUT
#define ClearEP1         0x72 // Clear Endpoint n Buffer, FIFO endpoint n
#define ClearEP2         0x73 
#define ClearEP3         0x74
#define ClearEP4         0x75
#define ClearEP5         0x76
#define ClearEP6         0x77
#define ClearEP7         0x78
#define ClearEP8         0x79
#define ClearEP9         0x7A
#define ClearEP10        0x7B
#define ClearEP11        0x7C
#define ClearEP12        0x7D
#define ClearEP13        0x7E
#define ClearEP14        0x7F

//------------------------------------------------------------------
// Checks status bits w/o clearing any status bit or IEPn
//------------------------------------------------------------------
#define CheckCtrlOutStat 0xD0 // Check Control OUT Status, Endpoint Status Image Register, endpoint 0 OUT, read 1 byte
#define CheckCtrlInStat  0xD1 // Check Control IN Status, Endpoint Status Image Register, endpoint 0 IN, read 1 byte
#define CheckEP1Stat     0xD2 // Check Endpoint n Status, Endpoint Status Image Register n, endpoint n, read 1 byte
#define CheckEP2Stat     0xD3
#define CheckEP3Stat     0xD4
#define CheckEP4Stat     0xD5
#define CheckEP5Stat     0xD6
#define CheckEP6Stat     0xD7
#define CheckEP7Stat     0xD8
#define CheckEP8Stat     0xD9
#define CheckEP9Stat     0xDA
#define CheckEP10Stat    0xDB
#define CheckEP11Stat    0xDC
#define CheckEP12Stat    0xDD
#define CheckEP13Stat    0xDE
#define CheckEP14Stat    0xDF

//------------------------------------------------------------------
// Ack setup packet 
// (RX of setup packet disables Validate and Clear cmd - AckSetup
//  enables these again.)
//------------------------------------------------------------------
#define AckSetup         0xF4 // Acknowledge Setup, Endpoint 0 IN and OUT, 

//------------------------------------------------------------------
// Read status of last transaction
//------------------------------------------------------------------
#define ReadCtrlOutErr   0xA0 // Read Control OUT Error Code, Error Code Register, endpoint 0 OUT, read 1 byte
#define ReadCtrlInErr    0xA1 // Read Control IN Error Code, Error Code Register, endpoint 0 IN, read 1 byte
#define ReadEP1Err       0xA2 // Read Endpoint n Error Code, Error Code Register, endpoint n, read 1 byte
#define ReadEP2Err       0xA3
#define ReadEP3Err       0xA4
#define ReadEP4Err       0xA5
#define ReadEP5Err       0xA6
#define ReadEP6Err       0xA7
#define ReadEP7Err       0xA8
#define ReadEP8Err       0xA9
#define ReadEP9Err       0xAA
#define ReadEP10Err      0xAB
#define ReadEP11Err      0xAC
#define ReadEP12Err      0xAD
#define ReadEP13Err      0xAE
#define ReadEP14Err      0xAF

// error bits
#define UNREAD 0x80 // 1=new event occurred before prev status read
#define DATA01 0x40 // indicates PID of last successful rx/tx packet (1=DATA1 PID)
#define ERROR  0x1E // see err codes
#define RTOK   0x01 // 1=rx or tx ok

// error codes
enum {
  EPERR_NONE,
  EPERR_PID_ENCODE,
  EPERR_PID_UNKNOWN,
  EPERR_PAC_UNEXPECT,
  EPERR_TOK_CRC,
  EPERR_DAT_CRC,
  EPERR_TIMEOUT,
  EPERR_BABBLE,
  EPERR_EOP_UNEXPECT,
  EPERR_NAK,
  EPERR_SENT_STALL,
  EPERR_RX_OVERFLOW,
  EPERR_SENT_EMPTY,
  EPERR_BIT_STUFF,
  EPERR_SYNC,
  EPERR_WRONG_TOGGLE,
};


//------------------------------------------------------------------
// Unlock device (to be done after each resume or suspend)
//------------------------------------------------------------------
#define UnlockDev        0xB0 // Unlock Device, all registers with write access, write 2 bytes

#define UnlockCode 0xAA37

//------------------------------------------------------------------
// Write/Read scratch
//------------------------------------------------------------------
#define WriteScratch     0xB2 // Write/Read Scratch Register, Scratch Register, write/read 2 bytes
#define ReadScratch      0xB3

//------------------------------------------------------------------
// Read frame number
//------------------------------------------------------------------
#define ReadFrameNum     0xB4 // Read Frame Number, Frame Number Register,read 1 or 2 bytes
// frame number bits
#define SOFR 0x07FF // SOF frame number


//------------------------------------------------------------------
// Read chip id
//------------------------------------------------------------------
#define ReadChipId       0xB5 // Read Chip ID, Chip ID Register, read 2 bytes

#define ISP1181B_CHIP_ID 0x8100 // evaluate only high byte of 0x8142 (copied from WINCE)


//------------------------------------------------------------------
// Read interrupt
//------------------------------------------------------------------
#define ReadInt          0xC0 // Read Interrupt Register, Interrupt Register, read 4 bytes

// interrupt register bits
#define EP14     0x00800000 // interrupt src=EP14
//...
#define EP5      0x00004000
#define EP3      0x00001000
#define EP2      0x00000800
#define EP1      0x00000400
#define EP0IN    0x00000200
#define EP0OUT   0x00000100
#define BUSTATUS 0x00000080 // 0=awake, 1=suspend
#define SP_EOT   0x00000040 // 1=EOT interrupt for short packet
#define PSOF     0x00000020 // 1=Pseudo SOF interrupt every 1ms (after 3 times -> suspend)
#define SOF      0x00000010 // 1=SOF condition
#define EOT      0x00000008 // 1=DMA counter reaching zero (EOT)
#define SUSPND   0x00000004 // 1='awake' to 'suspend' state detected
#define RESUME   0x00000002 // 1='resume' detected
#define RESET    0x00000001 // 1=bus reset detected


/**************************************/
typedef unsigned char BYTE;
typedef __u16         X2BYTES;
typedef __u16         WORD;
typedef unsigned int  DWORD;

/****** other defines *********/
#define   ON    1
#define   OFF   0

#define   YES   1 
#define   NO    0

#define   TRUE  1
#define   FALSE 0

#endif /* _HL_USB_REG_H_ */
