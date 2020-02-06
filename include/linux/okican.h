/*
 * drivers/char/can/oki_ml9620_core.h
 *
 * OKI ML9620 CAN Controller driver
 *
 * Author: Martin Nylund <mnylund@emtrion.de>
 *
 * Copyright 2004 (c) Manfred Gruber
 * Copyright 2004 (c) Contec Steuerungstechnik & Automation Ges.m.b.H
 * Copyright 2008 (c) Martin Nylund - emtrion GmbH
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#ifndef _OKICAN_H_
#define _OKICAN_H_

#if !defined(__KERNEL__)
#include <stdint.h>
#include <sys/ioctl.h>
#endif

/**************************************************************************/
/*          DRIVER API                                                    */
/**************************************************************************/

struct can_msg {
	/* CAN ID aka. arbitration field */
	uint32_t id;  

	/* Data Length Code */
	uint16_t dlc : 4;

	/* Remote Transmission request flag */
	uint16_t rtr : 1;

	/* Frame format - FF_NORMAL or FF_EXTENDED */
	uint16_t ff : 1;

	/* Data Overrun Status - one or more messages were lost before
	 * this message. Only valid with received messages */
	uint16_t dos : 2;

	/* Number of the message object (0:31), which received the message is
	 * saved here. Only relevant for received messages */
	uint16_t mobj:5;

	/* Message data */
	uint8_t  data[8];	/* data bytes                          */
};

/* Frame formats */
#define FF_NORMAL 0
#define FF_EXTENDED 1


#define IOC_MAGIC 'O'
/**************************************************************************/
#define IOC_START		               _IO (IOC_MAGIC, 1)
/**************************************************************************/
/* Start the CAN node */

/**************************************************************************/
#define IOC_STOP	                       _IO (IOC_MAGIC, 2)
/**************************************************************************/
/* Stop the can node */

/**************************************************************************/
#define IOC_SET_BITRATE	                       _IOW (IOC_MAGIC, 3, uint32_t)
/**************************************************************************/
/* Set bitrate. Valid values are: 10, 20, 50, 100, 125, 250, 500, 800 and
 * 1000 */

/**************************************************************************/
#define IOC_GET_STATUS	                       _IOR (IOC_MAGIC, 4, uint32_t)
/**************************************************************************/
/* Get status of the CAN node. Use following bitmasks to interpret the
 * value (for applications only error flags are of interest): */
#define S_TxOK (1<<3)   
#define S_RxOK (1<<4)
#define S_EPass (1<<5)
#define S_EWarn (1<<6)
#define S_BOff  (1<<7)

/**************************************************************************/
#define IOC_CFG_MSG_OBJECT        _IOW (IOC_MAGIC, 5, struct msg_obj_params)
/**************************************************************************/
/* If you want to configure the message objects yourself, you'll need to
 * study the ML9620 Datasheet and use this ioctl call to change the
 * parameters. */
struct msg_obj_params {
	/* number of the message object; 1 to 32 */
	int i;  

	/* IFxID1 to IFxID4 as a single 32 bit word */
	uint32_t id;

	/* IFxMASK1 to IFxMASK4 as a single 32 bit word */
	uint32_t mask;

	/* IFxMCONT1 and IFxMCONT2 */
	uint8_t ctrl_1;
	uint8_t ctrl_2;
};



/**************************************************************************/
/*          A SIMPLE EXAMPLE PROGRAM                                      */
/**************************************************************************/
/* Open a can node, configure it, write a CAN telegram and start to listen
 * for the traffic */
#if 0
#include <linux/okican.h>

int main(int argc, char *argv[])
{
    int ret,i,val;
    int canFd;
    struct can_msg msg;

    /* Open the CAN node for reading and writing */
    canFd = open("/dev/can0", O_RDWR);
    assert(canFd>0);
    
    /* Set bitrate to 125 kbit/s */
    val=125;
    ret=ioctl(canFd,IOC_SET_BITRATE,&val);
    perror("hui");
    assert(ret==0);

    /* Start the CAN node */
    ret=ioctl(canFd,IOC_START);
    assert(ret==0);

    /* Compose a CAN message with some dummy data */
    memset(&msg,0,sizeof(struct can_msg));
    msg.ext = 0;
    msg.id = 0xab;
    msg.dlc = 8;
    for(i=0;i<msg.dlc;i++){
	msg.data[i]=i;
    }

    /* Write the message to the canFd and receive it with canFd */
    ret=write(canFd,&msg,sizeof(struct can_msg));
    assert(ret==sizeof(msg));

    /* Read messages from CAN bus */
    while(1){
	ret=read(canFd,&msg,sizeof(struct can_msg));
	assert(ret==sizeof(msg));

	printf("received message with id %x\n",msg.id);
    }

    close(canFd);


    return 0;
}
#endif 

/**************************************************************************/
/*          REST OF THE DEFINITIONS ARE FOR THE DRIVER ONLY               */
/**************************************************************************/
#ifdef __KERNEL__


#ifdef CONFIG_SH_HICO7780
 #include <asm/hico7780.h>
 #define CAN_CLOCK_8MHZ
 #define REG_SHIFT 2
#else
 #define CAN_CLOCK_8MHZ
 #define REG_SHIFT 0
#endif

#define CANCONT		(0x0  << REG_SHIFT) 
#define RESERVE1	(0x1  << REG_SHIFT) 
#define CANSTAT 	(0x2  << REG_SHIFT) 
#define RESERVE2 	(0x3  << REG_SHIFT) 
#define CANTXERRC 	(0x4  << REG_SHIFT) 
#define CANRXERRC	(0x5  << REG_SHIFT) 
#define CANBITT1 	(0x6  << REG_SHIFT) 
#define CANBITT2 	(0x7  << REG_SHIFT) 
#define CANINT1 	(0x8  << REG_SHIFT) 
#define CANINT2 	(0x9  << REG_SHIFT) 
#define CANOPT		(0xA  << REG_SHIFT) 
#define CANBPRE		(0xC  << REG_SHIFT) 
#define IF1CREQ		(0x10 << REG_SHIFT)
#define IF1BUSY		(0x11 << REG_SHIFT)
#define IF1CMASK	(0x12 << REG_SHIFT)
#define IF1MASK1	(0x14 << REG_SHIFT)
#define IF1MASK2	(0x15 << REG_SHIFT)
#define IF1MASK3	(0x16 << REG_SHIFT)
#define IF1MASK4	(0x17 << REG_SHIFT)
#define IF1ID1		(0x18 << REG_SHIFT)
#define IF1ID2		(0x19 << REG_SHIFT)
#define IF1ID3		(0x1A << REG_SHIFT)
#define IF1ID4		(0x1B << REG_SHIFT)
#define IF1MCONT1	(0x1C << REG_SHIFT)
#define IF1MCONT2	(0x1D << REG_SHIFT)
#define IF1DATA1	(0x1E << REG_SHIFT)
#define IF1DATA2	(0x1F << REG_SHIFT)
#define IF1DATA3	(0x20 << REG_SHIFT)
#define IF1DATA4	(0x21 << REG_SHIFT)
#define IF1DATA5	(0x22 << REG_SHIFT)
#define IF1DATA6	(0x23 << REG_SHIFT)
#define IF1DATA7	(0x24 << REG_SHIFT)
#define IF1DATA8	(0x25 << REG_SHIFT)
#define IF2CREQ		(0x40 << REG_SHIFT)
#define IF2BUSY		(0x41 << REG_SHIFT)
#define IF2CMASK	(0x42 << REG_SHIFT)
#define IF2MASK0	(0x44 << REG_SHIFT)
#define IF2MASK1	(0x45 << REG_SHIFT)
#define IF2MASK2	(0x46 << REG_SHIFT)
#define IF2MASK3	(0x47 << REG_SHIFT)
#define IF2ID1		(0x48 << REG_SHIFT)
#define IF2ID2		(0x49 << REG_SHIFT)
#define IF2ID3		(0x4A << REG_SHIFT)
#define IF2ID4		(0x4B << REG_SHIFT)
#define IF2MCONT1	(0x4C << REG_SHIFT)
#define IF2MCONT2	(0x4D << REG_SHIFT)
#define IF2DATA1	(0x4E << REG_SHIFT)
#define IF2DATA2	(0x4F << REG_SHIFT)
#define IF2DATA3	(0x50 << REG_SHIFT)
#define IF2DATA4	(0x51 << REG_SHIFT)
#define IF2DATA5	(0x52 << REG_SHIFT)
#define IF2DATA6	(0x53 << REG_SHIFT)
#define IF2DATA7	(0x54 << REG_SHIFT)
#define IF2DATA8	(0x55 << REG_SHIFT)
#define CANTREQ1	(0x80 << REG_SHIFT)
#define CANTREQ2	(0x81 << REG_SHIFT)
#define CANTREQ3	(0x82 << REG_SHIFT)
#define CANTREQ4	(0x83 << REG_SHIFT)
#define CANNDATA1	(0x90 << REG_SHIFT)
#define CANNDATA2	(0x91 << REG_SHIFT)
#define CANNDATA3	(0x92 << REG_SHIFT)
#define CANNDATA4	(0x93 << REG_SHIFT)
#define CANIPEND1	(0xA0 << REG_SHIFT)
#define CANIPEND2	(0xA1 << REG_SHIFT)
#define CANIPEND3	(0xA2 << REG_SHIFT)
#define CANIPEND4	(0xA3 << REG_SHIFT)
#define CANMVAL1	(0xB0 << REG_SHIFT)
#define CANMVAL2	(0xB1 << REG_SHIFT)
#define CANMVAL3	(0xB2 << REG_SHIFT)
#define CANMVAL4	(0xB3 << REG_SHIFT)
#define CANSTBY		(0xC0 << REG_SHIFT)

#include <linux/cdev.h>
#include <linux/kfifo.h>
#include <linux/proc_fs.h>
struct can_device {

	/* ioremapped virtual base address of the chip registers */
	unsigned char __iomem *va_regs;

	/* character device structure */
	struct cdev cdev;
	/* This is only used in the dev_err/dbg/info messages */
	struct device *dev;

	/* IRQ number of the chip */
	unsigned int irq;

	/* waitqueues for interacting between interrupt handler and the
	 * read/write file operations */
	wait_queue_head_t rx_waitq;
	wait_queue_head_t tx_waitq;

	/* Message buffers */
	struct kfifo rx_fifo;
#ifdef USE_TX_FIFO
	struct kfifo tx_fifo;
#else
	/* flag to synchrise Tx process between ISR and write file
	 * operation */
	volatile unsigned char tx_in_progress;
#endif

	/* This flag is set when an overrun occures. The value is the
	 * written to the next succesfully received CAN message (see dos
	 * bit in struct can_msg */
	int dos:2;

	/* Open count */
	int can_opened;
};


/* This message object is used for transmitting messages. For the default
 * configuration it needs to be the last one */
#define TX_MSG_OBJ 32

/* Size of the kernel buffer for incoming can messages */
#define RX_FIFO_SIZE 100


/* Bitmasks for values in CAN control register */
#define CONT_FLAG_INIT		0x01
#define CONT_FLAG_IE		0x02
#define CONT_FLAG_SIE		0x04
#define CONT_FLAG_EIE		0x08
#define CONT_FLAG_CCE		0x40

/* bitmasks for IFmMCONT2 */
#define RxIE   (1<<2)
#define TxIE   (1<<3)
#define UMask  (1<<4)
#define MsgLst (1<<6)
#define NewDat (1<<7)

struct ML9620BDRATE {
	unsigned char btiming0;
	unsigned char btiming1;
	unsigned char bpre;
};

static const struct ML9620BDRATE bd_tbl[9] = {
#ifdef CAN_CLOCK_16MHZ
/* for 16 Mhz Quarz                */
/* bus timing 0  | bus timing 1    */
/* SJW PRESCALER | SAM TSEG1 TSEG2 */
	{0x23, 0x1C},		/*  10K bit/s */
	{0x31, 0x1C},		/*  20K bit/s */
	{0x13, 0x1C},		/*  50K bit/s */
	{0x09, 0x1C},		/* 100K bit/s */
	{0x07, 0x1C},		/* 125K bit/s */
	{0x03, 0x1C},		/* 250K bit/s */
	{0x01, 0x1C},		/* 500K bit/s */
	{0x01, 0x16},		/* 800K bit/s */
	{0x01, 0x14}		/*   1M bit/s */
#endif
#ifdef CAN_CLOCK_8MHZ
/* for 8 Mhz Quarz                */
/* bus timing 0  | bus timing 1    */
/* SJW PRESCALER | SAM TSEG1 TSEG2 | CANBRPE */
	{0x31, 0x1C, 0x01},	/*  10K bit/s           */
	{0x18, 0x1C, 0x00},	/*  20K bit/s           */
	{0x09, 0x1C, 0x00},	/*  50K bit/s           */
	{0x04, 0x1C, 0x00},	/* 100K bit/s           */
	{0x03, 0x1C, 0x00},	/* 125K bit/s           */
	{0x01, 0x1C, 0x00},	/* 250K bit/s           */
	{0x00, 0x1C, 0x00},	/* 500K bit/s           */
	{0x00, 0x16, 0x00},	/* 800K bit/s           */
	{0x00, 0x14, 0x00}	/*   1M bit/s           */
#endif
};

#endif /* __KERNEL__ */
#endif /* _OKICAN_H_ */
