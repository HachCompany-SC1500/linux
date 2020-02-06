
#define DM9013_MAX_NUM_IPHY	2  // Max number of internal PHY

#define DM9013_DWORD_MODE	1
#define DM9013_BYTE_MODE	2
#define DM9013_WORD_MODE	0

#define TRUE			1
#define FALSE			0

#define CONT_RX_PKT_CNT		0xFFFF

#define DM9013_PKT_RDY		0x01	/* Packet ready to receive */

#ifndef CONFIG_ARCH_MAINSTONE
#pragma pack(push, 1)
#endif

typedef struct _RX_DESC
{
	u8 rxbyte;
	u8 status;
	u16 length;
}RX_DESC;

typedef union{
	u8 buf[4];
	RX_DESC desc;
} rx_t;
#ifndef CONFIG_ARCH_MAINSTONE
#pragma pack(pop)
#endif

enum DM9013_PHY_mode {
	DM9013_10MHD   = 0, 
	DM9013_100MHD  = 1, 
	DM9013_10MFD   = 4,
	DM9013_100MFD  = 5, 
	DM9013_AUTO    = 8, 
};

enum DM9013_MAC_reg {
  DM9013_NCR	=	0x00,	/* Network control Reg.*/
  DM9013_NSR	=	0x01,	/* Network Status Reg.*/
  DM9013_TCR	=	0x02,	/* TX control Reg.*/
  DM9013_RXCR	=	0x05,	/* RX control Reg.*/
  DM9013_BPTR	=	0x08,
  DM9013_FCR	=	0x0a,
  DM9013_EPCR	=	0x0b,
  DM9013_EPAR	=	0x0c,
  DM9013_EPDRL	=	0x0d,
  DM9013_EPDRH	=	0x0e,
  DM9013_GPR	=	0x1f,	/* General purpose register */
  DM9013_VID_L	=	0x28,
  DM9013_VID_H	=	0x29,
  DM9013_PID_L	=	0x2A,
  DM9013_PID_H	=	0x2B,
  DM9013_TCR2	=	0x2d,
  DM9013_SMCR	=	0x2f, 	/* Special Mode Control Reg.*/
  DM9013_ETXCSR	=	0x30,	/* Early Transmit control/status Reg.*/
  DM9013_TCCR	=	0x31,	/* Checksum cntrol Reg. */
  DM9013_RCSR	=	0x32,	/* Receive Checksum status Reg.*/
  DM9013_INTCR	=	0x39,	
  DM9013_MONITOR2=	0x41,	
  
  DM9013_PPCR   =       0x60,
  DM9013_PPSDR  =       0x62,
  
  DM9013_MRCMDX	=	0xf0,
  DM9013_MRCMD	=	0xf2,
  DM9013_MDRAL	=	0xf4,
  DM9013_MDRAH	=	0xf5,
  DM9013_MWCMD	=	0xf8,
  DM9013_TXPLL	=	0xfc,
  DM9013_TXPLH	=	0xfd,
  DM9013_ISR	=	0xfe,
  DM9013_IMR	=	0xff
};

/* TX status */
#define TX_Jabber_timeout	(1<<7)
#define	TX_LossCarrier		(1<<6)
#define	TX_NoCarrier		(1<<5)
#define	TX_LateColli		(1<<4)
#define TX_ColliPkt		(1<<3)
#define TX_ExcessColli		(1<<2)

/* RX status */
#define RX_RuntFrame		(1<<7)
#define	RX_MultiFrame		(1<<6)
#define	RX_LateColli		(1<<5)
#define	RX_Watchdog_timeout	(1<<4)
#define	RX_PhyErr		(1<<3)
#define	RX_AlignErr		(1<<2)
#define	RX_CRCErr		(1<<1)
#define	RX_FIFO_over		1

/* DM9013_NCR */
#define NCR_MAC_loopback	2
#define NCR_Reset		1

/* DM9013_NSR */
#define NSR_10M			(1<<7)
#define NSR_Link_OK		(1<<6)
#define NSR_TX2END		(1<<3)
#define	NSR_TX1END		(1<<2)

/* DM9013_TCR */
#define TCR_TX_Request	1

/* DM9013_RXCR */
#define	RXCR_Discard_LongPkt	(1<<5)
#define RXCR_Discard_CRCPkt	(1<<4)
#define	RXCR_Pass_AllMulti	(1<<3)
#define	RXCR_Pass_RuntPkt	(1<<2)
#define	RXCR_Promiscuous	(1<<1)
#define	RXCR_RxEnable		1

/* DM9013_BPTR */
enum Jam_Pattern_Time{
	JPT_5us = 0,
	JPT_10us = 1,
	JPT_15us = 2,
	JPT_25us = 3,
	JPT_50us = 4,
	JPT_100us = 5,
	JPT_150us = 6,
	JPT_200us = 7,
	JPT_250us = 8,
	JPT_300us = 9,
	JPT_350us = 10,
	JPT_400us = 11,
	JPT_450us = 12,
	JPT_500us = 13,
	JPT_550us = 14,
	JPT_600us = 15
};

/* DM9013_FCR */
#define	FCR_TX_PausePkt		(1<<5)
#define	FCR_FlowCtlEable	1

/* DM9013_EPCR */
#define EPCR_WriteEEPROM_Enable	(1<<4)
#define EPCR_PHY_Sele		(1<<3) /*bit 3 = 0, select EEPROM*/
#define EPCR_Read		(1<<2)
#define EPCR_Write		(1<<1)

/* DM9013_EPAR */
#define DM9013_PHY		0x40	/* PHY address 0x01 */

/* DM9013_GPR */
#define GPR_PHYDown		1
#define GPR_PHYUp		0

/* DM9013_TCR2 */
#define TCR2_LedMode1		(1<<7)

/* DM9013_ETXCSR */
#define ETXCSR_EarlyTrans	(1<<7)
#define	Threshold_12		0
#define	Threshold_25		1
#define	Threshold_50		2
#define	Threshold_75		3

/* DM9013_TCCR */
#define TCCR_UDP_Chksum 	(1<<2)
#define TCCR_TCP_Chksum		(1<<1)
#define TCCR_IP_Chksum		1

/* DM9013_RCSR */
#define UDP_Chksum_Err		(1<<7)
#define TCP_Chksum_Err		(1<<6)
#define IP_Chksum_Err		(1<<5)
#define RCSR_RX_Chksum_enable	(1<<1)

/* DM9013_ISR */
#define ISR_Link_change		(1<<5)
#define ISR_TX_underrun		(1<<4)
#define ISR_RX_OFcnt_overflow	(1<<3)
#define ISR_RX_Overflow		(1<<2)
#define ISR_TX_complete		(1<<1)
#define ISR_RX_coming		1

/* DM9013_IMR */
#define	IMR_SRAM_autoReturn	(1<<7)
#define IMR_Link_change		(1<<5)
#define IMR_TX_underrun		(1<<4)
#define IMR_RX_OFcnt_overflow	(1<<3)
#define IMR_RX_Overflow		(1<<2)
#define IMR_TX_complete		(1<<1)
#define IMR_RX_coming		1

/* switch's register and setting */
#define DM9013_SCR	0x52
#define DM9013_SCR_ResetSwitch	(1<<6)
#define DM9013_SCR_ANLG_Reset	(1<<5)
#define DM9013_VLANCR 	0x53
#define DM9013_PIndex	0x60
#define DM9013_PCTRL	0x61
#define DM9013_PRATE	0x66
#define DM9013_PBW	0x67
#define DM9013_PPRI	0x6D
#define DM9013_VLAN_TAGL	0x6E

enum VLAN_GROUP{
	VLAN_GROUP0=0xb0,
	VLAN_GROUP1=0xb1,
	VLAN_GROUP2=0xb2,
	VLAN_GROUP3=0xb3,
	VLAN_GROUP4=0xb4,
	VLAN_GROUP5=0xb5,
	VLAN_GROUP6=0xb6,
	VLAN_GROUP7=0xb7,
	VLAN_GROUP8=0xb8,
	VLAN_GROUP9=0xb9,
	VLAN_GROUP10=0xba,
	VLAN_GROUP11=0xbb,
	VLAN_GROUP12=0xbc,
	VLAN_GROUP13=0xbd,
	VLAN_GROUP14=0xbe,
	VLAN_GROUP15=0xbf,
};	

enum {
	DM9013_BW_FULL=0,
	DM9013_BW_64K,
	DM9013_BW_128K,
	DM9013_BW_256K,
	DM9013_BW_512K,
	DM9013_BW_1M,
	DM9013_BW_2M,
	DM9013_BW_4M,
	DM9013_BW_8M,
	DM9013_BW_16M,
	DM9013_BW_32M,
	DM9013_BW_48M,
	DM9013_BW_64M,
	DM9013_BW_72M,
	DM9013_BW_80M,
	DM9013_BW_88M,
	DM9013_BW_LAST
};

//void IC_Notice(board_info_t *db);
//void IC_PHY_Notice(board_info_t *db, int i);