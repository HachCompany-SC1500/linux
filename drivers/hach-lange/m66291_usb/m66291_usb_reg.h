/*************************************************************
Author                   :Nikhil Varghese
Renesas M66291 Register List

 *****************************************************************/
#ifndef _HL_USB_REG_H_
#define _HL_USB_REG_H_

#define PORT_CTRL (port_ctrl)
#define PORT_DATA (port_data)
#define MAX_EP0_SEND_LEN   256
#define EP0_IVAL  (0x2000)
#define CTRLEND             0x1
#define        EP0_ODLN                (0x00ff)
#define        EP0_CCPL                (0x0400)
#define        CTRLSTALL       0x0
#define        STALL                                    0x8000
#define        CPU_BSWP                                (0x0080)
#define        CPU_IVAL                (0x2000)
#define        MAX_EP_NO                       6
#define        DATA_NONE                       0

#define RQ_POWR                  0x0000         /* Powered State */
#define RQ_DFLT                  0x0010         /* Default State */
#define RQ_ADDS                  0x0020         /* Address State */
#define RQ_CNFG                  0x0030         /* Configured State         */
#define RQ_SPDP                  0x0040         /* Power Suspend State      */
#define RQ_SPDD                  0x0050         /* Default Suspend State    */
#define RQ_SPDA                  0x0060         /* Address Suspend State    */
#define RQ_SPDC                  0x0070         /* Configured Suspend State */
#define RQ_IDST                  0x0000         /* Idle or setup stage */
#define RQ_RDDS                  0x0001         /* Control read transfer data stage */
#define RQ_RDSS                  0x0002         /* Control read transfer status stage */
#define RQ_WRDS                  0x0003         /* Control write transfer data stage */
#define RQ_WRSS                  0x0004         /* Control write transfer status stage */
#define RQ_WRND                  0x0005         /* Control write no data transfer status stage */
#define RQ_SQER                  0x0006         /* Control transfer sequence error */
#define RQ_UNDF                  0x0007         /* Not assigned */


#define EP0_ISEL (0x0001)
#define BUF  0x4000
#define	USB_ENABLE	0x00
#define	XTAL	0x3000	/*	b13-12:	Crystal	selection	*/	/*	6MHz	*/
#define	XTAL48	0x0000	/*	48MHz	*/
#define	XTAL24	0x2000	/*	24MHz	*/
#define	XTAL12	0x1000	/*	12MHz	*/
#define	XCKE	0x8000	/*	b15:	External	clock	enable	*/
#define	PLLC	0x4000	/*	b14:	PLL	control	*/
#define	SCKE	0x0800	/*	b11:	USB	clock	enable	*/
#define	USBE	0x0001	/*	b0:	USB	module	operation	enable	*/
#define	TRONL	0x0100	/*	b9-8:	Tr_on	Output	Control*/	/*TrON	Output	=L*/
#define	TRONH	0x0300	/*	TrON	Output	=	H	*/
#define	USBPC	0x0400	/*	b10:	USB	Transceiver	Power	Control	*/

#define	REMOTEWAKEUP	0x02
#define	WKUP	0x0001	/*	b0:	Remote	Wakeup	*/

#define	SEQUENCE_BIT	0x04	/*	Sequence	Bit	Clear	Register	*/
#define	SQCLR	0x007F	/*	b6:	Clear	Sequence	bit	*/

#define	USB_ADDRESS	0x08
#define	USB_ADDR	0x007F	/*	b6-0:USB	Address	assigned	by	host	*/

#define	ISOCHRONOUS_STATUS	0x0A
#define	FMOD	0x0800	/*	b11	:	Frame	Number	Mode	*/
#define	FRNM	0x007F	/*	b10-0	:	Frame	Number	*/

#define	SOF_CNT	0x0C
#define	SOFOE	0x8000	/*	b:15	SOF	Output	Enable	*/
#define	SOFA	0x4000	/*	b:14	SOF	Polarity	*/

#define	POLARITY_CNT	0x0E
#define	VB01	0x8000	/*	b15:	VBus	Inturrupt	Assign	*/
#define	RM01	0x4000	/*	b14:	Resume	Inturrupt	Assign	*/
#define	SF01	0x2000	/*	b13:	SOF	Direct	Interrupt	Assign	*/
#define	DS01	0x1000	/*	b12:	Device	State	Transition	Interrupt	Assign	*/
#define	CT01	0x0800	/*	b11:	Control	Transfer	Transition	Interrupt	Assign	*/
#define	BE01	0x0400	/*	b10:	Buffer	Empty/Size	Over	Error	Interrupt	Assign	*/
#define	NR01	0x0200	/*	b9:	Buffer	Not	Ready	Interrupt	Assign	*/
#define	RD01	0x0100	/*	b8:	Buffer	Ready	Interrupt	Assign	*/
#define	RDYM	0x0004	/*	b2:	Buffer	Ready	Mode	*/
#define	INTL	0x0002	/*	b1:	Interrupt	Output	Sense	*/
#define	INTA	0x0001	/*	b0:	Interrupt	Polarity	*/

#define	D0_FIFO_SELECT	0x48
#define	D1_FIFO_SELECT	0x50
#define	BUST	0x8000	/*	b15:	Burst	Mode	*/
#define	DFORM	0x6000	/*	b14-13:	Transfer	Method	*/
#define	DSRW	0x0000	/*	Controls	by	DACK	signal	and	read/write	signal	*/
#define	DSO	0x2000	/*	Controls	by	DACK	signal	only	*/
#define	CSSRW	0x4000	/*	Controls	by	chip	select/address	signal	and	read/write	signal	*/
#define	RWND	0x1000	/*	b12:	Buffer	Rewind	*/
#define	ACKA	0x0800	/*	b11:	DACK	Polarity	*/
#define	REQA	0x0400	/*	b10:	DREQ	Polarity	*/
#define	INTM	0x0200	/*	b9:	DMA	Interrupt	Mode*/
#define	DMAEN	0x0100	/*	b8:	DMA	Enable	*/
#define	BSWP	0x0080	/*	b7:	Byte	Swap	Mode	*/
#define	DMA_Octl	0x0040	/*	b6:	Register	8-Bit	Mode	*/
#define	DMA_EP	0x000F	/*	b3-0:	DMA	Transfer	Endpoint	Designate	*/
#define	EP1	0x0001	/*	End	Point	1	*/
#define	EP2	0x0002	/*	End	Point	2	*/
#define	EP3	0x0003	/*	End	Point	3	*/
#define	EP4	0x0004	/*	End	Point	4	*/
#define	EP5	0x0005	/*	End	Point	5	*/
#define	EP6	0x0006	/*	End	Point	6	*/

#define	D0_FIFO_CONTROL	0x4A
#define	D1_FIFO_CONTROL	0x52
#define	TRCLR	0x8000	/*	b15:	Transaction	Count	Clear	*/
#define	TREN	0x4000	/*	b14:	Transaction	Count	Enable	*/
#define	IVAL	0x2000	/*	b13:	IN	Buffer	Set/OUT	Buffer	Status	*/
#define	BCLR	0x1000	/*	b12:	Buffer	clear	*/
#define	Dreq	0x8000	/*	b11:	FIFO	ready	*/
#define	DMA_DTLN	0x07FF	/*	b10-0:	FIFO	received	data	length	*/

#define	D0_FIFO_DATA	0x4C
#define	D1_FIFO_DATA	0x54
#define	D1FIFO	0xFFFF	/*	b15-0:	D_FIFO	Data	*/

#define	DMA0_TRN_COUNT	0x4E
#define	DMA1_TRN_COUNT	0x56
#define	TRNCNT	0xFFFF	/*	b15-0:	Transaction	Count	*/

#define	INT_ENABLE0	0x10
#define	VBSE	0x8000	/*	b15:	VBUS	interrupt	*/
#define	RSME	0x4000	/*	b14:	Resume	interrupt	*/
#define	SOFE	0x2000	/*	b13:	Frame	update	interrupt	*/
#define	DVSE	0x1000	/*	b12:	Device	state	transition	interrupt	*/
#define	CTRE	0x0800	/*	b11:	Control	transfer	stage	transition	irq	*/
#define	BEMPE	0x0400	/*	b10:	Buffer	empty	interrupt	*/
#define	INTNE	0x0200	/*	b9:	Buffer	not	ready	interrupt	*/
#define	INTRE	0x0100	/*	b8:	Buffer	ready	interrupt	*/
#define	URST	0x0080	/*	b7:	USB	reset	detected	interrupt	*/
#define	SADR	0x0040	/*	b6:	Set	address	executed	interrupt	*/
#define	SCFG	0x0020	/*	b5:	Set	configuration	executed	interrupt	*/
#define	SUSP	0x0010	/*	b4:	Suspend	detected	interrupt	*/
#define	WDST	0x0008	/*	b3:	Control	write	data	stage	completed	irq	*/
#define	RDST	0x0004	/*	b2:	Control	read	data	stage	completed	irq	*/
#define	CMPL	0x0002	/*	b1:	Control	transfer	complete	interrupt	*/
#define	SERR	0x0001	/*	b0:	Sequence	error	interrupt	*/

#define	INT_ENABLE1	0x12
#define	EPB_RE	0x007F	/* b6-0: Buffer Ready Interrupt Enable */

#define	INT_ENABLE2	0x14
#define	EPB_NRE	0x007F	/*  b6-0: Buffer Not Ready Interrupt Enable */

#define	INT_ENABLE3	0x16
#define	EPB_EMPE	0x007F	/* b6-0: Buffer Empty/Size Over error Interrupt Enable */
#define EPB0_ENB 0x0001 /*      b0 --> EP0 */
#define EPB1_ENB 0x0002 /*      b1 --> EP1 */
#define EPB2_ENB 0x0004 /*      b2 --> EP2 */
#define EPB3_ENB 0x0008 /*      b3 --> EP3 */
#define EPB4_ENB 0x0010 /*      b4 --> EP4 */
#define EPB5_ENB 0x0020 /*      b5 --> EP5 */
#define EPB6_ENB 0x0040 /*      b6 --> EP6 */

#define	INT_STATUS0	0x18
#define	VBUS	0x8000	/*	b15:	VBUS	interrupt	*/
#define	RESM	0x4000	/*	b14:	Resume	interrupt	*/
#define	SOFR	0x2000	/*	b13:	SOF	frame	update	interrupt	*/
#define	DVST	0x1000	/*	b12:	Device	state	transition	*/
#define	CTRT	0x0800	/*	b11:	Control	stage	transition	*/
#define	BEMP	0x0400	/*	b10:	Buffer	empty	interrupt	*/
#define	INTN	0x0200	/*	b9:	Buffer	not	ready	interrupt	*/
#define	INTR	0x0100	/*	b8:	Buffer	ready	interrupt	*/
#define	Vbus	0x0080	/*	b7:	VBUS	input	port	*/
#define	DVSQ	0x0070	/*	b6-4:	Device	state	*/
#define	DS_SPD_CNFG	0x0070	/*	Suspend	Configured	*/
#define	DS_SPD_ADDR	0x0060	/*	Suspend	Address	*/
#define	DS_SPD_DFLT	0x0050	/*	Suspend	Default	*/
#define	DS_SPD_POWR	0x0040	/*	Suspend	Powered	*/
#define	DS_SUSP	0x0040	/*	Suspend	*/
#define	DS_CNFG	0x0030	/*	Configured	*/
#define	DS_ADDS	0x0020	/*	Address	*/
#define	DS_DFLT	0x0010	/*	Default	*/
#define	DS_POWR	0x0000	/*	Powered	*/
#define	DVSQS	0x0030	/*	b5-4:	Device	state	*/
#define	VALID	0x0008	/*	b3:	Setup	packet	detected	flag	*/
#define	CTSQ	0x0007	/*	b2-0:	Control	transfer	stage	*/
#define	CS_SQER	0x0006	/*	Sequence	error	*/
#define	CS_WRND	0x0005	/*	Control	write	nodata	status	*/
#define	CS_WRSS	0x0004	/*	Control	write	status	stage	*/
#define	CS_WRDS	0x0003	/*	Control	write	data	stage	*/
#define	CS_RDSS	0x0002	/*	Control	read	status	stage	*/
#define	CS_RDDS	0x0001	/*	Control	read	data	stage	*/
#define	CS_IDST	0x0000	/*	Idle	or	setup	stage	*/

#define	INT_STATUS1	0x1A
#define	EPB_RDY	0x007F	/*	b6-0:	Buffer	Ready	interupt	*/
#define EPB0_REQ 0x0001 /*	b0 --> EP0 */
#define EPB1_REQ 0x0002 /*	b1 --> EP1 */
#define EPB2_REQ 0x0004 /*	b2 --> EP2 */
#define EPB3_REQ 0x0008 /*      b3 --> EP3 */
#define EPB4_REQ 0x0010 /*      b4 --> EP4 */
#define EPB5_REQ 0x0020 /*      b5 --> EP5 */
#define EPB6_REQ 0x0040 /*      b6 --> EP6 */

#define	INT_STATUS2	0x1C
#define	EPB_NRDY	0x007F	/*	b6-0: Buffer Not Ready interupt */

#define INT_STATUS3	0x1E
#define EPB_EMP_OVR	0x007F	/*	b6-0: Buffer Empty/ Size Over Interrupt Enable */

#define	REQUEST_TYPE	0x20
//#define bRequest	0xFF00	/*	b15-8: bRequest */
//#define bmRequestType	0x00FF	/*	b7-0: bmRequestType */
#define REQTYP                                   0x60
#define STANDARD                                 0x00
#define CLASS                                    0x20
#define VENDER                                   0x40
#define TypRESERVED                              0x60
/* Recipient */
#define DEVICE                                   0x00
#define INTERFACE                                0x01
#define ENDPOINT                                 0x02
#define OTHER                                    0x03
#define ReqRESERVED                              0x02
#define ReqRESERVED1                     0x04
#define GET_CONFIGURATION                0x08
#define SET_CONFIGURATION                0x09
#define SYNCH_FRAME                              0x0C


#define	REQUEST_VALUE	0x22
#define	REQUEST_INDEX	0x24
#define	TEST_SELECT	0xFF00	/*	b15-b8:	Test	Mode	*/
#define	TEST_J	0x0100	/*	Test_J	*/
#define	TEST_K	0x0200	/*	Test_K	*/
#define	TEST_SE0_NAK	0x0300	/*	Test_SE0_NAK	*/
#define	TEST_PACKET	0x0400	/*	Test_Packet	*/
#define	TEST_FORCE_ENABLE	0x0500	/*	Test_Force_Enable	*/
#define	TEST_STSelectors	0x0600	/*	Standard	test	selectors	*/
#define	TEST_Reserved	0x4000	/*	Reserved	*/
#define	TEST_VSTModes	0xC000	/*	Vendor-specific	tests	*/
#define	EP_DIR	0x0080	/*	b7:	Endpoint	Direction	*/
#define	EP_DIR_IN	0x0080
#define	EP_DIR_OUT	0x0000

#define REQUEST_LENGTH	0x26


#define	CONTROL_TRANSFER	0x28
#define	CTRR	0x8000	/*	b15:	Control	Read	Transfer	Continuous	Transmit	Mode	*/
#define	Ctr_Rd_Buf_Nmb	0x3F00	/*	b13-8:	Control	Read	Buffer	Start	Number	*/
#define	CTRW	0x0080	/*	b7	:	Control	Write	Transfer	Continuous	Receive	Mode	*/
#define	Ctr_Wr_Buf_Nmb	0x003F	/*	b5-0:	Control	Write	Buffer	Start	Number	*/

#define	EP0_PACKET_SIZE	0x2A
#define	EP0_MXPS	0x0008	/*	Maximum	Packet	Size	*/

#define	AUTO_RESPONSE_CONTROL	0x2C
#define	ASCN	0x0002	/*	SET_CONFIGURATION	Automatic	Response	Mode	*/
#define	ASAD	0x0001	/*	SET_ADDRESS	Automatic	Response	Mode	*/

#define	EP0_FIFO_SELECT	0x30
#define	RCNT	0x8000	/*	b15:	Read	Count	Mode	*/
#define	EP0_Octl	0x0400	/*	b10:	Register	8-Bit	Mode	*/
#define	BSWP	0x0080	/*	b7:	Byte	Swap	Mode	*/
#define	ISEL	0x0001	/*	b0:	Buffer	Select	*/
#define	EP0_FIFO_CONTROL	0x32
#define	EP0_PID	0xC000	/*	b14-15:	Response	PID*/
#define	EP0_PID_STALL	0x8000	/*	STALL	*/
#define	EP0_PID_BUF	0x4000	/*	BUF	*/
#define	EP0_PID_NAK	0x0000	/*	NAK	*/
#define	IVAL	0x2000	/*	b13:	IN	Buffer	Set/OUT	Buffer	Status	*/
#define	BCLR	0x1000	/*	b12:	Buffer	Clear	*/
#define	E0req	0x0800	/*	b11:	EP0_FIFO	Ready	*/
#define	CCPL	0x0400	/*	b10:	Control	Transfer	Control	*/
#define	ODLN	0x01FF	/*	b8-0:	Control	Write	Receive	Data	Length	*/
#define	EP0_FIFO_DATA	0x34
#define	EP0_FIFO	0xFFFF	/*	b15-0:	EP0_FIFO	Data	*/
#define	EP0_SEND_LEN	0x36
#define	SDLN	0x01FF	/*	b8-0:	Control	Read	Continuous	Transmit	Data	Length	*/

#define	CPU_FIFO_SELECT	0x40
#define	RCTN	0x8000	/*	b15:	Read	Count	Mode*/
#define	RWND	0x1000	/*	b12:	Buffer	Rewind	*/
#define	BSWP	0x0080	/*	b7:	Byte	Swap	Mode	*/
#define	CPU_Octl	0x0040	/*	b6:	Register	8-Bit	Mode	*/
#define	CPU_EP	0x000F	/*	b0-3:	CPU	Access	Endpoint	Designate	*/
#define	EP1	0x0001	/*	End	Point	1	*/
#define	EP2	0x0002	/*	End	Point	2	*/
#define	EP3	0x0003	/*	End	Point	3	*/
#define	EP4	0x0004	/*	End	Point	4	*/
#define	EP5	0x0005	/*	End	Point	5	*/
#define	EP6	0x0006	/*	End	Point	6	*/

#define	CPU_FIFO_CONTROL	0x42
#define	IDLY	0x4000	/*	b14:	Isochronous	Transmit	Delay	Set	*/
#define	IVAL	0x2000	/*	b13:	IN	Buffer	Set/OUT	Buffer	Status	*/
#define	BCLR	0x1000	/*	b12:	Buffer	Clear	*/
#define	Creq	0x0800	/*	b11:	CPU_FIFO	Ready	*/
#define	CPU_DTLN	0x07FF	/*	b10-0:	CPU_FIFO	Receive	Data	Length	Register	*/

#define	CPU_FIFO_DATA	0x44
#define	CPU_FIFO	0xFFFF	/*	CPU_FIFO	Data	*/

#define	SIE_FIFO_STATUS	0x46
#define	TGL	0x2000	/*	b13:	Buffer	Toggle	*/
#define	SCLR	0x1000	/*	b12:	Buffer	Clear	*/
#define	Sreq	0x0800	/*	b11:	SIE_FIFO	Ready	*/
#define	SIE_DTLN	0x07FF	/*	b10-0:	SIE_FIFO	Receive	Data	Length	*/

#define	FIFOSTATUS	0x58
#define	EPB_STS	0x007F	/*	b6-0:	Endpoint	0~6	Buffer	Status	*/

#define	PORT_CNTL	0x5A
#define	PEIN	0xFF00	/*	b15-8:	Port	Input	Enable	*/
#define	PDIR	0x007F	/*	b6-0:	Port	Direction	*/

#define	PORT_DAT	0x5C
#define	PDAT	0x007F	/*	b6-0:	Port	Input	Enable	*/

#define	I_ADJ	0x5E
#define	LDRV	0x0001	/*	b0:	Drive	Current	Adjust	*/

#define	EP1_0CONFIG	0x60
#define	EP2_0CONFIG	0x64
#define	EP3_0CONFIG	0x68
#define	EP4_0CONFIG	0x6C
#define	EP5_0CONFIG	0x70
#define	EP6_0CONFIG	0x74
#define	EPi_TYP_ISO	0x6000	/*	b15-14:	Transfer	Type	Isochronous	transfer	*/
#define	EPi_TYP_BULK	0x2000	/*	Bulk	Transfer	*/
#define	EPi_TYP_INTR	0x4000	/*	Interrupt	Transfer	*/
#define	EPi_DIR	        0x2000	/*	b13:	Transfer	Direction	*/
#define	EPi_ITMD	0x1000	/*	b12:	Interrupt	Toggle	Mode	*/
#define	EPi_Buf_siz	0x0F00	/*	b11-8:	Buffer	Size	*/
#define	EPi_DBLB	0x0080	/*	b7:	Double	Buffer	Mode	*/
#define	EPi_RWMD	0x0040	/*	b6:	Continuous	Transmit/Receive	Mode	*/
#define	EPi_Buf_Nmb	0x003F	/*	b5-0:	Buffer	Start	Number	*/
#define	EPi_Buf_64	0x0100	/*	b11-8:	Buffer	Size	*/
#define PCK_SIZE        64

#define	EP1_1CONFIG	0x62
#define	EP2_1CONFIG	0x66
#define	EP3_1CONFIG	0x6A
#define	EP4_1CONFIG	0x6E
#define	EP5_1CONFIG	0x72
#define	EP6_1CONFIG	0x76
#define	EPi_PID	0x6000	/*	b15-14:	Response	PID	STALL	*/
#define	EPi_PID_STALL	0x8000	/*	STALL	*/
#define	EPi_PID_BUF	0x4000	/*	BUF	*/
#define	EPi_PID_NAK	0x0000	/*	NAK	*/
#define	EPi_NULMD	0x1000	/*	b12:	Zero-Length	Packet	Addtion	Transmit	Mode	*/
#define	EPi_ACLR	0x0800	/*	b11:	OUT	Buffer	Automatic	Clear	Mode	*/
#define	EPi_Octl	0x0400	/*	b10:	Register	8-Bit	Mode	*/
#define	EPi_MXPS	0x0200	/*	b9-0:	Maximum	Packet	Size	*/
#define	get_pipectr_addr(pipenum)	(M66592_EP1_0CONFIG	+	(pipenum	-	1)	*	4)
#define	M66592_MAX_NUM_PIPE	7



#define EPC0_DIR                (0x2000)        /* b13:Transfer direction */
#define         OUT_DIR                      0x0000         /* Out Direction */
#define         IN_DIR                       0x2000         /* In Direction */
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

/**************************************/
typedef unsigned char BYTE;
//typedef __u16         X2BYTES;
typedef unsigned short  X2BYTES;
//typedef __u16         WORD;
typedef unsigned short   WORD;
typedef unsigned int  DWORD;

/****** other defines *********/
#define   ON    1
#define   OFF   0

#define   YES   1
#define   NO    0

#define   TRUE  1
#define   FALSE 0

#endif /* _HL_USB_REG_H_ */
