/***************************************************************************
  usb_drv.c  -  description
  -------------------
begin                : Don Sep 23 2004
copyright            : (C) 2004 by Thomas Siegmund
email                : tsiegmund@hach-lange.de
Author               : Nikhil Varghese
**********Function Edited for support of M66291***************
static int BufferFull(void)
static inline int readfifo(BYTE* rxdata, BYTE ep_num, int bufsiz)
inline void SendBulkData(unsigned int count_t)
void dev_req_handler(void)
static int usb_dev_clear_feature(USB_request*  req)
void reset_usb(void)
void activate_usb(void)
void deactivate_usb(void)
void clear_usb_status(void)
void config_ep_fifos(void)
static inline void writefifo(U8 *txdata, BYTE ep_num, int nobytes)
static void init_Statistic_Device_State(void)
static inline void SetUnstallEpx(int ep_no)
static inline void SetStallEpx(int ep_no)
static void endpoint_status_init(void)


******************8Function Added for support of m66291*************
void epx_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum) -> Writing data to EP1-EP4
void config_usb(void)						      -> Configuring USB Chip
void ep5_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum) ->Writing data to EP1-EP4
void Control_read(unsigned char *EP0Ptr,int count_t)                  ->Reading EP0
void write_epx_to_host_Data_In(unsigned short * dtptr,U8 Con_Num, U8 EPnum,unsigned int count)->Writing EP0
void Control_End_CCPL(void)                                            -> End of Control Transfer
void DisableIntR(U16 EP_Num)                                           -> Disabling Buffer emply interrupt
void Control_End(U16 resp)                                             -> Last packet of Control transfer
unsigned short read_from_host_or_Control_write(void)                   -> ->Writing EP0
void write_to_host_Control_read(unsigned char *EP0Ptr,int count_t)     ->Reading EP0
void CW_Start()                                                        -> Starting Control Operation
void EP0clear(void)                                                    -> Clearing EP0 Buffer 

 ***************************************************************************/

#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/smp_lock.h>
#ifdef CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#endif
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctl.h>
#include <asm/irq.h>
#include <linux/usb.h>    // /opt/Embedix/home/project/gamma_tsg/build/rpmdir/BUILD/linux/include/linux
#include <linux/time.h>

/**** own headers ************/
#include "m66291_usb_reg.h"
#include "m66291_usb_drv.h"
#include "m66291_usb_desc.h"
#include "m66291_usb_ioctl.h"
#include "m66291_usb_viper.h"
#include "m66291_usb_state.h"
/////////////////////////////////////////

#define DBG_TIME printk("%s[%d]: TIME=%ld\n", __FUNCTION__, __LINE__, jiffies);

#define U16 unsigned short
#define U8 unsigned char
U16 EP0Count;
U8*         EP0Ptr;
unsigned char   Buffer_Write_Data_Flag[MAX_EP_NO + 1];          /* Flag of transmit data */


/***************************************************
 **                    Prototypes
 ****************************************************/
static inline BYTE USB_Transmit_Data(BYTE *tdata, int data_size, EndPoint ep_num,int packet);
static inline void writefifo(BYTE* txdata, BYTE ep_num, int noBYTEs,int packet);
static inline void tx_enable_epx(int ep_num);
static inline void send_ZLP_2_finish_EPx(BYTE ep_num);
static inline void usb_device_reset(void);
static inline void SetStallEpx(int ep_no);
static int  BufferFull(void);
static void init_Statistic_Device_State(void);
static void clear_control_buffer(control_buffer*  ctrl_buf);
static void endpoint_status_init(void);
static void send_control_data(BYTE* data, int size);
void Control_End(U16 resp);
void Control_read(unsigned char *EP0Ptr,int count_t);
void epx_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum,int packet);
void ep5_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum,int packet);


typedef void (*USB_req_handler)(USB_request*);

/*=====================================================*
 *       Prototypes    standard requests
 *=====================================================*/
static void usb_dev_get_status(USB_request*          req);
static void usb_dev_clear_feature(USB_request*       req);
static void usb_dev_set_feature(USB_request*         req);
static void usb_dev_set_address(USB_request*         req);
static void usb_std_dev_get_descriptor(USB_request*  req);
static void usb_std_dev_set_descriptor(USB_request*  req);
static void usb_dev_get_config(USB_request*   		  req);
static void  usb_dev_set_config(USB_request*    	  req);
static void  usb_dev_get_interface(USB_request* 	  req);
static void  usb_dev_set_interface(USB_request* 	  req);
static void  usb_dev_sync_frame(USB_request*    	  req);

/*********************************************************
 * requests' sequence is according to USB 1.1 spec values
 *********************************************************/
static USB_req_handler usb_std_device_req[] =
{
	usb_dev_get_status,
	usb_dev_clear_feature,
	NULL,
	usb_dev_set_feature,
	NULL,
	usb_dev_set_address,
	usb_std_dev_get_descriptor,
	usb_std_dev_set_descriptor,
	usb_dev_get_config,
	usb_dev_set_config,
	usb_dev_get_interface,
	usb_dev_set_interface,
	usb_dev_sync_frame,
};

/***************************************
 **    Prototypes  vendor requests
 ****************************************/
void Cmd_Data_Parser(USB_request* req);
//void Data_Parser(USB_request*    req);

/****************************************
 *    Vendor device request handlers
 ****************************************/
static const USB_req_handler usb_vendor_device_req[] =
{
	NULL,
	Cmd_Data_Parser
};

#pragma pack(push, old_pack_val, 1)
/**********************************
 ** Device USB device descriptor
 ***********************************/
static const USB_device_desc usb_device_desc =
{
	sizeof(USB_device_desc),
	DEVICE_DESCRIPTOR,
	USB_VERSION,
	CLASS_NOT_DEFINED,      /* CLASS_VENDOR, vendor specific */
	0,                      /* Device Sub-Class */
	0,                      /* Device Protocol */
	EP0_FIFO_SIZE,          /* Max Packet Size for EndPoint Zero*/
	VENDOR_ID,
	PRODUCT_ID,
	BCDDEVICE,             /* device release number: 01.08 BCDDEVICE */
	STR_MANUFACTURER,
	STR_PRODUCT,
	STR_VERSION,                      /* Device's serial number */
	1                       /* Num of configurations */
};


/*-------------------------------------
 * Device Long Configuration descriptor
 *------------------------------------*/
static const USB_long_config_desc usb_dev_long_config_desc =
{
	//Device Configuration descriptor
	{
		sizeof(USB_config_desc),    /* CONFIG_DESC_LENGTH */
		CONFIG_DESCRIPTOR,
		sizeof(USB_long_config_desc), /* TOTAL CONFIG_DESC_LENGTH */
		1,                          /* one interface supported */
		1,                          /* Configuration number */
		0,                          /* no descriptor string STR_PRODUCT */
		SELF_POWERED,
		0
	},
	{
		sizeof(USB_interface_desc),
		INTERFACE_DESCRIPTOR,
		0,                  // The only interface concurrently supported by this configuration 
		0,                  // Alternate Setting 
		NUM_OF_ENDPOINTS_FOR_ALT_0, // Num of endpoints of this interface excluding endpoint zero 
		CLASS_MASS_STORAGE,        // Mass-Storage device class 
		CLASS_VENDOR,      //0x06,                    // Sub class 
		0x01,              // Interface Protocol  Contorl/Bulk/Interrupt Protocol but without Command implementation 
		0
	},

	{
		//The IN endpoint 1 is used for bulk data transfer
		{
			sizeof(USB_endpoint_desc),
			ENDPOINT_DESCRIPTOR,
			{
				EPT1, //0x81
				0,
				IN
			},
			BULK_EP,
			TX_BULK_EP_FIFO_SIZE, /* Max Packet Size */
			0
		},

		//The OUT endpoint 2 is used for bulk data transfer
		{
			sizeof(USB_endpoint_desc),
			ENDPOINT_DESCRIPTOR,
			{
				EPT2, //0x02
				0,
				OUT
			},
			BULK_EP,
			RX_BULK_EP_FIFO_SIZE, /* Max Packet Size */
			0
		},
		//The IN endpoint 3 is used for interrupt transfer
		{
			sizeof(USB_endpoint_desc),
			ENDPOINT_DESCRIPTOR,
			{
				EPT3, //0x83
				0,
				IN
			},
			INTERRUPT_EP,
			TX_INTR_EP_FIFO_SIZE_COMM,    /* Max Packet Size 8 byte*/
			0xFF                          /* Interrupt Interval, 255 ms */
		},
		//The IN endpoint 5 is used for interrupt transfer for GAMMA Measurements
		{
			sizeof(USB_endpoint_desc),
			ENDPOINT_DESCRIPTOR,
			{
				EPT5, //0x85
				0,
				IN
			},
			INTERRUPT_EP,
			TX_INTR_EP_FIFO_SIZE_MEAS,    /* Max Packet Size 64 byte*/
			0x01                          /* Interrupt Interval, 1 ms */
		}   
	}  
};

//String descriptors
const struct {
	BYTE  bLength;
	BYTE  bDescriptorType;
	WORD  bstring;
} langid_str_desc = {sizeof(langid_str_desc), STRING_DESCRIPTOR, ENGLISH_US};

const struct {
	BYTE  bLength;
	BYTE  bDescriptorType;
	char  bstring[sizeof(MANUFACTURE_STR)];
} manufacture_str_desc = {sizeof(manufacture_str_desc), STRING_DESCRIPTOR, MANUFACTURE_STR};

const struct {
	BYTE  bLength;
	BYTE  bDescriptorType;
	char  bstring[sizeof(PRODUCT_STR)];
} product_str_desc = {sizeof(product_str_desc), STRING_DESCRIPTOR, PRODUCT_STR};

const struct {
	BYTE  bLength;
	BYTE  bDescriptorType;
	char  bstring[sizeof(VERSION_STRW)];
} version_str_desc = {sizeof(version_str_desc), STRING_DESCRIPTOR, VERSION_STRW};
#pragma pack(pop, old_pack_val)

/*****************************************************************
 **if we get alternate settings then we to have revisit the place
 **and make 2 array instead of 1
 *****************************************************************/
//List of endpoint descriptors   -> usb_dev_endpoints[NUM_OF_ALTERNATESETTING][NUM_OF_ENDPOINTS]
//there isn't an other alternate stetting
static const USB_endpoint_desc  *usb_dev_endpoints[2][NUM_OF_ENDPOINTS] =
{
	{
		NULL,                                                      /* Endpoint 0 */
		&usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[0],  /* Endpoint 1  Bulk IN     */
		&usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[1],  /* Endpoint 2  Bulk OUT    */
		&usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[2],  /* Endpoint 3  Interrupt IN*/
		NULL,
		&usb_dev_long_config_desc.usb_dev_endpoint_alt_0_desc[3],  /* Endpoint 5  Interrupt IN*/
		NULL
	},
	{
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	}
};

/*********************************************************
 * List of all string descriptors, Be sure that the order
 * of the list is the same as String_index enum
 **********************************************************/
const USB_string_desc *string_descs[] = {
	(USB_string_desc*)&langid_str_desc,
	(USB_string_desc*)&manufacture_str_desc,
	(USB_string_desc*)&product_str_desc,
	(USB_string_desc*)&version_str_desc
};

#ifdef _SELF_MODULE_UNLOAD_
extern int selfCleanUp(void);
static DWORD ResetEvCnt   = 0; 
#endif


static const int fifo_sizes[] =
{
	EP0_FIFO_SIZE,             /* control tx0, rx0   EPT0 */
	TX_BULK_EP_FIFO_SIZE,      /* bulk tx            EPT1 */
	RX_BULK_EP_FIFO_SIZE,      /* bulk rx            EPT2 */
	TX_INTR_EP_FIFO_SIZE_COMM, /* int  tx   8 byte    EPT3 */
	NO_ENDPOINT_ZERO_SIZE,
	TX_INTR_EP_FIFO_SIZE_MEAS, /* int  tx  64 byte    EPT5 */
	NO_ENDPOINT_ZERO_SIZE
};


/******* USB device status ***********/
static DEVICE_status   device_status;
static Device_buffers  device_buffers;

/******* Endpoint states ************/
static endpoint_status tx0_ep_st;
static endpoint_status tx1_ep_st;
static endpoint_status rx1_ep_st;
static endpoint_status tx2_ep_st;
static endpoint_status tx3_ep_st;

endpoint_status *ep_stat[] =
{
	&tx0_ep_st,
	&rx1_ep_st,
	&tx1_ep_st,
	&tx2_ep_st,
	NULL,
	&tx3_ep_st,
	NULL
};

static control_buffer ctrl_send_buf;
static control_buffer ctrl_receive_buf;

//static BYTE reqfifo[EP0_FIFO_SIZE];

void dbg_time(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
}
/**********************************************************/
static void clear_control_buffer(control_buffer*  ctrl_buf)
{
	ctrl_buf->data      = NULL;
	ctrl_buf->byte_cnts = 0;
}
/*********************************************************/
static void endpoint_status_init(void)
{
	int i;
	for(i=0; i<NUM_OF_ENDPOINTS; i++) 
	{
		if(ep_stat[i] != NULL)  
		{
			ep_stat[i]->fifo_state = EMPTY;

			if(i==0) //control endpoint
				ep_stat[i]->toggle_bit = 0x01;
			else //tx & rx endpoints
				ep_stat[i]->toggle_bit = 0x00;
		}
	}
}
/*****************************************************************/
static inline void SetStallEpx(int ep_no)
{
	if(ep_no == EPT0) 
	{
		m66291_set_reg(EP0_FIFO_CONTROL,(m66291_read_reg(EP0_FIFO_CONTROL)| EP0_PID));
	}
	else 
	{
		m66291_set_reg(EP1_1CONFIG + ((ep_no-1)*4),(m66291_read_reg(EP1_1CONFIG + ((ep_no-1)*4))| EPi_PID));
	}                                    

	spin_lock(&DevState.lock);
	DevState.UsbState.EpState[ep_no] = EP_STALL;
	spin_unlock(&DevState.lock);
}
/*****************************************************************/
static inline void SetUnstallEpx(int ep_no)
{
	if(ep_no == EPT0) 
	{
		m66291_set_reg(EP0_FIFO_CONTROL, EP0_PID_BUF);
	}
	else 
	{
		switch (ep_no)
		{
			case EPT1:
				m66291_set_reg(EP1_1CONFIG,m66291_read_reg(EP1_1CONFIG)|EPi_PID_NAK );
				m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB1_REQ );
				m66291_set_reg(EP1_1CONFIG,m66291_read_reg(EP1_1CONFIG)|EPi_PID_BUF );
				break;
			case EPT2:
				m66291_set_reg(EP2_1CONFIG,m66291_read_reg(EP2_1CONFIG)|EPi_PID_NAK );
				m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB2_REQ );
				m66291_set_reg(EP2_1CONFIG,m66291_read_reg(EP2_1CONFIG)|EPi_PID_BUF );
				break;
			case EPT3:
				m66291_set_reg(EP3_1CONFIG,m66291_read_reg(EP3_1CONFIG)|EPi_PID_NAK );
				m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB3_REQ );
				m66291_set_reg(EP3_1CONFIG,m66291_read_reg(EP3_1CONFIG)|EPi_PID_BUF );
				break;
			case EPT4:
				m66291_set_reg(EP4_1CONFIG,m66291_read_reg(EP4_1CONFIG)|EPi_PID_NAK );
				m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB4_REQ );
				m66291_set_reg(EP4_1CONFIG,m66291_read_reg(EP4_1CONFIG)|EPi_PID_BUF );
				break;
			case EPT5:
				m66291_set_reg(EP5_1CONFIG,m66291_read_reg(EP5_1CONFIG)|EPi_PID_NAK );
				m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB5_REQ );
				m66291_set_reg(EP5_1CONFIG,m66291_read_reg(EP5_1CONFIG)|EPi_PID_BUF );
				break;
			default:
				break;
		}
	}                                    

	spin_lock(&DevState.lock);    
	DevState.UsbState.EpState[ep_no] = EP_ENABLED;
	spin_unlock(&DevState.lock);
}
/*****************************************************************/
static void init_Statistic_Device_State(void)
{ 
	int i;
	spin_lock(&DevState.lock);
	DevState.UsbState.BusState = USB_DISCONNECTED;

	for(i=1; i<NUM_OF_ENDPOINTS; i++)
	{
		DevState.UsbState.EpState[i] = EP_NOT_ASSIGNED;    
	}
	spin_unlock(&DevState.lock);
}

/*****************************************************************/
void EP0clear(void)
{
	register unsigned short buffer;
	m66291_set_reg(EP0_FIFO_SELECT, m66291_read_reg(EP0_FIFO_SELECT) & (~ISEL));
	buffer = m66291_read_reg(EP0_FIFO_CONTROL);
	buffer = (buffer & (~IVAL)) | BCLR;
	m66291_set_reg(EP0_FIFO_CONTROL, buffer);
	m66291_set_reg(EP0_FIFO_SELECT, m66291_read_reg(EP0_FIFO_SELECT) | ISEL);
	buffer = m66291_read_reg(EP0_FIFO_CONTROL);
	buffer = (buffer & (~IVAL)) | BCLR;
	m66291_set_reg(EP0_FIFO_CONTROL, buffer);

}

/*****************************************************************/


void CW_Start() {
	U16 buffer;
	EP0clear();
	buffer=m66291_read_reg(EP0_FIFO_SELECT);
	buffer  &= ~EP0_ISEL;
	m66291_set_reg(EP0_FIFO_SELECT,buffer);
	m66291_set_reg(INT_STATUS1,m66291_read_reg(INT_STATUS1)&(~EPB0_REQ));
	m66291_set_reg(INT_ENABLE1,m66291_read_reg(INT_ENABLE1)|EPB0_ENB);

	m66291_set_reg(EP0_FIFO_CONTROL,m66291_read_reg(EP0_FIFO_CONTROL)|BUF|EP0_PID);
}

/*****************************************************************/

void write_to_host_Control_read(unsigned char *EP0Ptr,int count_t) {
	U16    count, max_size;
	U16    buffer, odd;

	CW_Start();


	buffer=m66291_read_reg(CONTROL_TRANSFER); 
	max_size = MAX_EP0_SEND_LEN;
	if( EP0Count < max_size ) {
		count = count_t;

	} else {
		count = max_size;
	}
	m66291_set_reg(INT_STATUS3,m66291_read_reg(INT_STATUS3)&(~EPB0_REQ)); /* BEMP status clear */

	m66291_set_reg(EP0_SEND_LEN, count);
	buffer=m66291_read_reg(EP0_FIFO_SELECT);
	if( !(buffer & EP0_Octl) ) {
		odd = count % 2;
		count = count / 2;
		while(count) {
			do {
				buffer=m66291_read_reg(EP0_FIFO_CONTROL);
			} while( buffer & E0req );
			buffer  = (U16)(*(U16*)EP0Ptr);

			m66291_set_reg(EP0_FIFO_DATA, buffer);
			EP0Ptr++;
			EP0Ptr++;
			count--;
		}
		if(odd) {
			count = odd;
			m66291_set_reg(EP0_FIFO_SELECT,m66291_read_reg(EP0_FIFO_SELECT)|EP0_Octl);
		}
	}
	while(count) {
		do {
			buffer=m66291_read_reg(EP0_FIFO_CONTROL);
		} while( buffer & E0req );
		buffer  = (U16)(*EP0Ptr);
		m66291_set_reg(EP0_FIFO_DATA,(U8) buffer);
		EP0Ptr++;
		count--;
	}
	if (EP0Count < max_size) {
		EP0Count = 0;
		m66291_set_reg(EP0_FIFO_CONTROL,m66291_read_reg(EP0_FIFO_CONTROL)|EP0_IVAL);
	} else {
		EP0Count -= max_size;
		m66291_set_reg(INT_ENABLE3,m66291_read_reg(INT_ENABLE3)|EPB0_ENB);
	}
	Control_End(CTRLEND);
}



/*****************************************************************/

unsigned short read_from_host_or_Control_write(void) {
	U16 count, max_size;
	U16    buffer, odd;
	unsigned short *EP0Ptr=0;

	buffer=m66291_read_reg(EP0_FIFO_CONTROL);
	max_size = buffer&EP0_ODLN;
	if( max_size > EP0Count ) {
		Control_End(CTRLSTALL);
		max_size        = EP0Count;
	}
	count           = max_size;

	buffer=m66291_read_reg(EP0_FIFO_SELECT);
	if( !(buffer & EP0_Octl) ) {
		odd = count % 2;
		count = count / 2;

		while(count) {
			do {
				buffer=m66291_read_reg(EP0_FIFO_CONTROL);
			} while( buffer & E0req );
			buffer=m66291_read_reg(EP0_FIFO_DATA);
			*(U16*)EP0Ptr   = buffer;

			EP0Ptr++;
			EP0Ptr++;
			count--;
		}

		if(odd) {
			count = odd;
		}
	}
	while(count) {
		do {
			buffer=m66291_read_reg(EP0_FIFO_CONTROL);
		} while( buffer & E0req );
		buffer=m66291_read_reg(EP0_FIFO_DATA);
		*EP0Ptr = (U8)buffer;
		EP0Ptr++;
		count--;
	}
	EP0Count -= max_size;
	Control_End(CTRLEND);
	return *EP0Ptr;
}

/*****************************************************************/
void Control_End(U16 resp) {
	U16 buffer;
	buffer=m66291_read_reg(INT_STATUS0);
	if      (buffer & VALID){
		return;
	}

	switch (resp) {
		case CTRLEND:
			buffer=m66291_read_reg(EP0_FIFO_CONTROL);
			if((buffer&EP0_PID)!=STALL) {
				m66291_set_reg(EP0_FIFO_CONTROL, m66291_read_reg(EP0_FIFO_CONTROL) | EP0_PID_BUF);									                }
			break;
		case CTRLSTALL:
			m66291_set_reg(EP0_FIFO_CONTROL, m66291_read_reg(EP0_FIFO_CONTROL) | EP0_PID_STALL);
			break;
		default:
			break;
	}
}

/*****************************************************************/
void Control_End_CCPL(void) {
	m66291_set_reg(EP0_FIFO_CONTROL, m66291_read_reg(EP0_FIFO_CONTROL) | EP0_CCPL);

}
/*****************************************************************/
void DisableIntR(U16 EP_Num) {
	switch (EP_Num) {
		case EP1: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB1_ENB));break;
		case EP2: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB2_ENB));break;
		case EP3: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB3_ENB));break;
		case EP4: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB4_ENB));break;
		case EP5: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB5_ENB));break;
		case EP6: m66291_set_reg( INT_ENABLE1,m66291_read_reg(INT_ENABLE1)&(~EPB6_ENB));break;
	}
}

/*****************************************************************/


void write_epx_to_host_Data_In(unsigned short * dtptr,U8 Con_Num, U8 EPnum,unsigned int count) {
	U16   Size;  U16    buffer;    Size=EP0_MXPS;
	buffer  = m66291_read_reg(CPU_FIFO_SELECT) | EPnum;
	m66291_set_reg(CPU_FIFO_SELECT, buffer);
	//Set as non Continious transmit mode
	switch(EPnum)
	{
		case 1:
			m66291_set_reg( EP1_0CONFIG,m66291_read_reg(EP1_0CONFIG)&(~EPi_RWMD));
			m66291_set_reg( EP1_0CONFIG,m66291_read_reg(EP1_0CONFIG)&(~EPi_DBLB));
			m66291_set_reg( EP1_0CONFIG,m66291_read_reg(EP1_0CONFIG)|EPi_Buf_siz|EPi_TYP_BULK);
			break;
		case 2:
			m66291_set_reg( EP2_0CONFIG,m66291_read_reg(EP2_0CONFIG)&(~EPi_RWMD));
			m66291_set_reg( EP2_0CONFIG,m66291_read_reg(EP2_0CONFIG)&(~EPi_DBLB));
			m66291_set_reg( EP2_0CONFIG,m66291_read_reg(EP2_0CONFIG)|0x0100|EPi_TYP_BULK);
			break;
		case 3:
			m66291_set_reg( EP3_0CONFIG,m66291_read_reg(EP3_0CONFIG)&(~EPi_RWMD));
			m66291_set_reg( EP3_0CONFIG,m66291_read_reg(EP3_0CONFIG)&(~EPi_DBLB));
			m66291_set_reg( EP3_0CONFIG,m66291_read_reg(EP3_0CONFIG)|EPi_Buf_siz|EPi_TYP_BULK);
			break;
		case 5:
			m66291_set_reg( EP5_0CONFIG,m66291_read_reg(EP5_0CONFIG)&(~EPi_RWMD));
			m66291_set_reg( EP5_0CONFIG,m66291_read_reg(EP5_0CONFIG)&(~EPi_DBLB));
			m66291_set_reg( EP5_0CONFIG,m66291_read_reg(EP5_0CONFIG)|EPi_Buf_siz|EPi_TYP_BULK);
			break;
		default:
			break;
	}
	while(count) {

		do {
			buffer = m66291_read_reg(CPU_FIFO_CONTROL);
		} while (buffer & Creq);
		buffer  = (U16)(*dtptr);
		m66291_set_reg(CPU_FIFO_DATA,(U8) buffer);
		dtptr++;
		count--;
	}
	if (count < Size) {                                                      /* calculate transmit data number */
		count = 0;
		m66291_set_reg(CPU_FIFO_CONTROL, m66291_read_reg(CPU_FIFO_CONTROL) | CPU_IVAL);
		DisableIntR(EPnum);                                                             /* Disable INTR interrupption   */
		Buffer_Write_Data_Flag[EPnum] = DATA_NONE;              /* Set flag     DATA_NONE                       */
		DisableIntR(EPnum);                                                             /* Disable INTR interrupption   */
		Buffer_Write_Data_Flag[EPnum] = DATA_NONE;              /* Set flag     DATA_NONE                       */
	}
	else {
		count -= Size;
	}

	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT) & (~CPU_EP));

}
/*****************************************************************/
static inline void writefifo(U8 *txdata, BYTE ep_num, int nobytes,int packet)
{
	if(0==ep_num){
		Control_read(txdata,nobytes);

	}
	if(0!=ep_num){
		if(ep_num == 5)
			ep5_write(txdata,nobytes,ep_num,packet);
		else	
			epx_write(txdata,nobytes,ep_num,packet);

	}
}
/*****************************************************************/
static void fill_ctrl_fifo(void)
{
	BYTE *data = ctrl_send_buf.data;
	// get number of bytes to be sent
	int count = min(ctrl_send_buf.byte_cnts, (unsigned int)EP0_FIFO_SIZE);


	//update control buffer parameters
	ctrl_send_buf.data      += count;
	ctrl_send_buf.byte_cnts -= count;

	writefifo(data, EPT0, count,0);

}
/*****************************************************************/
static void dev_enable_epx( const USB_endpoint_desc*  ep )
{
	EndPoint ep_no = ep->bEndpointAddress.address;
	//enable endpoint & set its address

	if(ep->bEndpointAddress.direction == OUT)
		FlushTx(ep_no);

	spin_lock(&DevState.lock);
	DevState.UsbState.EpState[ep_no] = EP_ENABLED;
	spin_unlock(&DevState.lock);
}
/*********************************************************/
static void dev_disable_epx(const USB_endpoint_desc *ep)
{
	spin_lock(&DevState.lock);
	DevState.UsbState.EpState[ep->bEndpointAddress.address] = EP_DISABLED;
	spin_unlock(&DevState.lock);
}
/*****************************************************************/
static inline void tx_enable_epx(int ep_num)
{
	;
}
/*********************************************************/
void zero_length_data_responseEP0(void)
{
	// MS: FLUSH_TX0;
	send_control_data(0,0);
}
/*********************************************************/
static inline void send_ZLP_2_finish_EPx(BYTE ep_num)
{
	// MS: FLUSH_TXEP(ep_num);
	tx_enable_epx(ep_num);
}

/*******************************************************/
/* all 16 EPs have to be configured in a row for the   */
/* fifos to be valid                                   */
/*******************************************************/
#define         FS0064         0x0000 
#define         NONE           0x0000
#define         BULK           0x4000
#define         OUT            0x0000
#define         DOUBLE         0x0080
#define         CONT           0x0040 
#define         BNM8           0x0008
#define         BNM20          20
#define         IN             0x2000
#define         SINGLE         0x0000
#define         INT            0x8000
void config_ep_fifos(void)
{
	U16 sTemp;
	U16 buffer;

	m66291_set_reg(INT_ENABLE1,0x000); //Clear All the interrupts
	m66291_set_reg(INT_ENABLE2,0x000);
	m66291_set_reg(INT_ENABLE3,0x000);

	m66291_set_reg(EP1_1CONFIG, m66291_read_reg(EP1_1CONFIG)|EPi_PID_NAK);
	m66291_set_reg(EP2_1CONFIG, m66291_read_reg(EP2_1CONFIG)|EPi_PID_NAK);
	m66291_set_reg(EP3_1CONFIG, m66291_read_reg(EP3_1CONFIG)|EPi_PID_NAK);
	m66291_set_reg(EP4_1CONFIG, m66291_read_reg(EP4_1CONFIG)|EPi_PID_NAK);
	m66291_set_reg(EP5_1CONFIG, m66291_read_reg(EP5_1CONFIG)|EPi_PID_NAK);

	m66291_set_reg(EP1_0CONFIG, m66291_read_reg(EP1_0CONFIG)|BULK|IN|NONE|FS0064|SINGLE|CONT|BNM8);
	m66291_set_reg(EP1_1CONFIG, m66291_read_reg(EP1_1CONFIG ) | EPi_PID_BUF|0x40);
	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT)|RCTN);

	m66291_set_reg(EP2_0CONFIG, m66291_read_reg(EP2_0CONFIG)|BULK|OUT|NONE|FS0064|SINGLE|CONT|BNM20);
	m66291_set_reg(EP2_1CONFIG, m66291_read_reg(EP2_1CONFIG)|EPi_PID_BUF|0x40);
	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT)|RCTN);

	m66291_set_reg(EP3_0CONFIG, m66291_read_reg(EP3_0CONFIG)|INT|IN|NONE|FS0064|SINGLE|CONT|32);
	m66291_set_reg(EP3_1CONFIG, m66291_read_reg(EP3_1CONFIG)|EPi_PID_BUF|0x40);
	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT)|RCTN);

	m66291_set_reg(EP5_0CONFIG, m66291_read_reg(EP5_0CONFIG)|INT|IN|NONE|FS0064|SINGLE|CONT|46);
	m66291_set_reg(EP5_1CONFIG, m66291_read_reg(EP3_1CONFIG)|EPi_PID_BUF|0x40);
	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT)|RCTN);
	m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|SQCLR); 
	m66291_set_reg(EP1_1CONFIG,m66291_read_reg(EP1_1CONFIG)|EPi_ACLR); 
	m66291_set_reg(EP1_1CONFIG,m66291_read_reg(EP1_1CONFIG)&(~EPi_ACLR));
	m66291_set_reg(EP2_1CONFIG,m66291_read_reg(EP2_1CONFIG)|EPi_ACLR); 
	m66291_set_reg(EP2_1CONFIG,m66291_read_reg(EP2_1CONFIG)&(~EPi_ACLR));
	m66291_set_reg(EP3_1CONFIG,m66291_read_reg(EP3_1CONFIG)|EPi_ACLR); 
	m66291_set_reg(EP3_1CONFIG,m66291_read_reg(EP3_1CONFIG)&(~EPi_ACLR));
	m66291_set_reg(EP4_1CONFIG,m66291_read_reg(EP4_1CONFIG)|EPi_ACLR); 
	m66291_set_reg(EP4_1CONFIG,m66291_read_reg(EP4_1CONFIG)&(~EPi_ACLR));
	m66291_set_reg(EP5_1CONFIG,m66291_read_reg(EP5_1CONFIG)|EPi_ACLR); 
	m66291_set_reg(EP5_1CONFIG,m66291_read_reg(EP5_1CONFIG)&(~EPi_ACLR));



	m66291_set_reg(CPU_FIFO_CONTROL,m66291_read_reg(CPU_FIFO_CONTROL)&(~CPU_IVAL)); 
	m66291_set_reg(CPU_FIFO_CONTROL,m66291_read_reg(CPU_FIFO_CONTROL)|BCLR); 

	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT)|EP2);
	m66291_set_reg(EP2_0CONFIG,m66291_read_reg(EP2_0CONFIG)|EPi_DBLB|EPi_Buf_64);
	m66291_set_reg(EP2_1CONFIG,m66291_read_reg(EP2_1CONFIG)|EPi_PID_BUF );
	m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB2_REQ );
	m66291_set_reg(EP2_1CONFIG,0x5440 );
	m66291_set_reg(CPU_FIFO_SELECT,m66291_read_reg(CPU_FIFO_SELECT)| EP2 | RCTN | RWND| CPU_Octl); 
	m66291_set_reg(INT_ENABLE1, m66291_read_reg(INT_ENABLE1)|EPB2_ENB);
	m66291_set_reg(INT_ENABLE2, m66291_read_reg(INT_ENABLE2)|EPB2_ENB);
	m66291_set_reg(INT_ENABLE3, m66291_read_reg(INT_ENABLE3)|EPB2_ENB);


	sTemp = CTRR | CTRW | 0x0004;
	m66291_set_reg(CONTROL_TRANSFER, sTemp);
	m66291_set_reg(EP0_PACKET_SIZE, 0x0040);
	m66291_set_reg(EP0_FIFO_SELECT, RCNT );
	m66291_set_reg(EP0_FIFO_SELECT, ISEL);                 /* OUT-FIFO Select    */
	buffer = m66291_read_reg(EP0_FIFO_CONTROL);
	buffer = (buffer & (~IVAL)) | BCLR; 
	m66291_set_reg(EP0_FIFO_CONTROL, buffer);
	m66291_set_reg(EP0_FIFO_SELECT, m66291_read_reg(EP0_FIFO_SELECT) | ISEL);                      /* IN-FIFO Select     */
	m66291_set_reg(EP0_FIFO_CONTROL, (m66291_read_reg(EP0_FIFO_CONTROL) | BCLR) & (~IVAL));



}

/*********************************************************/
void clear_usb_status(void)
{
	int iStat;
	do
	{
		m66291_set_reg(USB_ENABLE, USBE);
		iStat = m66291_read_reg(USB_ENABLE);
	}while((iStat & USBE) != USBE);
        
	m66291_set_reg(USB_ENABLE, 0x00);
	m66291_set_reg(USB_ENABLE, XTAL);
	m66291_set_reg(USB_ENABLE, m66291_read_reg(USB_ENABLE) | XCKE);
        mdelay(10); // Renesas_M66291GP.pdf: 2.5.6 Clock: b. Wait until oscillation stabilizes
	m66291_set_reg(USB_ENABLE, m66291_read_reg(USB_ENABLE) | PLLC);
	mdelay(1);  // Renesas_M66291GP.pdf: 2.5.6 Clock: d. Wait until PLL oscillation stabilizes (less than 1ms)
	m66291_set_reg(USB_ENABLE, (m66291_read_reg(USB_ENABLE) | SCKE | USBPC | TRONH | USBE));
        
}
/*********************************************************/
void config_usb(void)
{
	/* hw config */
	/*************/
	/* clear status / irq flags */
	/****************************/
	clear_usb_status();

	/* irq config */
	/**************/
	// enable reset, resume, suspend, EP0, EP1, EP2, EP3, EP5 irq
	/* config fifos */
	/****************/
	config_ep_fifos();


}
/*******************************************************/
void deactivate_usb(void)
{

	// soft unplug USB
	// reset m66291
	m66291_set_reg(INT_ENABLE0, 0x0000);
	m66291_set_reg(USB_ENABLE, 0x0000);
}
/*******************************************************/
void activate_usb(void)
{
	m66291_set_reg(INT_ENABLE0, URST);       /* Enable USB reset detect interrupt */
	m66291_set_reg(INT_ENABLE0, m66291_read_reg(INT_ENABLE0) | DVSE);        /* Enable Device State Transition Interrupt */
	m66291_set_reg(INT_ENABLE0, m66291_read_reg(INT_ENABLE0) | WDST  | RDST  | CTRE );   /* Enable Ctrl Transfer Transition Interrupt */
	m66291_set_reg(INT_ENABLE0, m66291_read_reg(INT_ENABLE0) | INTRE | INTNE | BEMPE);   /* Enable Buffer-Rdy/Not Ready/Empty Intrpt */
	m66291_set_reg(INT_ENABLE0, m66291_read_reg(INT_ENABLE0) | VBSE);   /* Enable Buffer-Rdy/Not Ready/Empty Intrpt */

}

/*********************************************************/
void reset_usb(void)
{
  clear_control_buffer(&ctrl_send_buf);
  clear_control_buffer(&ctrl_receive_buf);
  endpoint_status_init();
  init_Statistic_Device_State();
  config_usb();
  // clear_usb_status(); // done in config_usb()
  activate_usb();
}

/*********************************************************/
void reset_usb_nt(void)
{
  clear_control_buffer(&ctrl_send_buf);
  clear_control_buffer(&ctrl_receive_buf);
  endpoint_status_init();
  init_Statistic_Device_State();
}
/*****************************************************************/
static inline void usb_device_reset(void)
{
	if(device_status.state == ADDRESSED || 
			device_status.state == CONFIGURED)
	{
		device_status.state = ATTACHED;
	}
	device_buffers.zero_data = 0;
}
/**********************************************************/
static void send_control_data(BYTE* data, int size)
{
	// MS: FLUSH_TX0;
	ctrl_send_buf.data = data;
	ctrl_send_buf.byte_cnts = size;
	fill_ctrl_fifo();
}
/***************************************************************
 ** standard device requests
 ****************************************************************/
static void usb_dev_get_status(USB_request*  req)
{

	BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;
	U16 buffer; 

	if((device_status.state == ATTACHED) || 
			(IS_REQ_VALUE_NOT_ZERO(req)) || 
			(REQ_LENGTH(req) != 2)) 
	{
		SetStallEpx(EPT0);
		return;
	}

	//Clear all reserved bits
	device_buffers.status.msb.as_byte = 0;

	switch(REQ_RECIPIENT(req)) 
	{
		case DEVICE_REQ:
			if(REQ_INDEX(req).as_bytes.lsb != 0)
			{//device's behaviour is not specified
				SetStallEpx(EPT0);
				return;
			}
			device_buffers.status.msb.device.wakeup = OFF;
			//self powered
			device_buffers.status.msb.device.selfpowered = ON;
			break;

		case INTERFACE_REQ:
			if(REQ_INDEX(req).interface.inf_no != 0)
			{
				SetStallEpx(EPT0);
				return;
			}
			//Reserved
			device_buffers.status.msb.interface.value = 0;
			break;

		case ENDPOINT_REQ:
			switch(ep_no) {
				case EPT0:
					device_buffers.status.msb.endpoint.stalled = ((buffer=m66291_read_reg(EP0_FIFO_CONTROL))|EP0_PID_STALL)? ON : OFF;
					break;
				case EPT1:
					device_buffers.status.msb.endpoint.stalled =((buffer=m66291_read_reg(EP1_1CONFIG))|EPi_PID_STALL)? ON : OFF;
					break;
				case EPT2:
					device_buffers.status.msb.endpoint.stalled =((buffer=m66291_read_reg(EP2_1CONFIG))|EPi_PID_STALL)? ON : OFF;
					break;
				case EPT3:
					device_buffers.status.msb.endpoint.stalled =((buffer=m66291_read_reg(EP3_1CONFIG))|EPi_PID_STALL)? ON : OFF;
					break;
				case EPT5:
					device_buffers.status.msb.endpoint.stalled =((buffer=m66291_read_reg(EP3_1CONFIG))|EPi_PID_STALL)? ON : OFF;
					break;
				case EPT4:            
				case EPT6:
				default:
					SetStallEpx(EPT0);
					return;
			}
			break;

		case OTHER_REQ:
		default: //undefined recipient
			SetStallEpx(EPT0);
			return;
	}
	//Reserved
	device_buffers.status.lsb = 0;
	send_control_data((BYTE *)&device_buffers.status, sizeof(USB_device_status));
}
/****************************************************************/
static void usb_dev_clear_feature(USB_request*  req)
{
	BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;

	if((device_status.state == ATTACHED) || 
			(REQ_LENGTH(req) != 0))
	{
		SetStallEpx(EPT0);
		return;
	}

	switch(REQ_RECIPIENT(req))
	{
		case DEVICE_REQ: // wakeup feature is not supported
			if(REQ_VALUE(req).feature.bSelector == DEVICE_REMOTE_WAKEUP)
			{
				SetStallEpx(EPT0);
			}
			break;

		case INTERFACE_REQ:
			break;

		case ENDPOINT_REQ: //clear stall state of appropriate endpoint    
			if((REQ_VALUE(req).feature.bSelector != ENDPOINT_STALL) ||(ep_no >= EPT_LAST) ||(usb_dev_endpoints[device_status.curAltSetting][ep_no] == NULL))
			{
				SetStallEpx(EPT0);
			}
			else  
			{
				SetUnstallEpx(ep_no);
				//actually the toogle bit should be reset
				ep_stat[ep_no]->toggle_bit = (ep_no == EPT0) ?  0x01 : 0x00;
				switch(ep_no)
				{
					case EP2:
						m66291_set_reg(SIE_FIFO_STATUS,m66291_read_reg(SIE_FIFO_STATUS)|TGL);
						break;
					case EP5:
						m66291_set_reg(EP5_0CONFIG,m66291_read_reg(EP5_0CONFIG)|EPi_DBLB );
						m66291_set_reg(EP5_1CONFIG,m66291_read_reg(EP5_1CONFIG)|EPi_PID_BUF );
						m66291_set_reg(SEQUENCE_BIT,m66291_read_reg(SEQUENCE_BIT)|EPB5_REQ );
						m66291_set_reg(EP5_1CONFIG,0x5440 );
						m66291_set_reg(CPU_FIFO_SELECT,m66291_read_reg(CPU_FIFO_SELECT)| EP5 | RCTN | RWND| CPU_Octl);
						m66291_set_reg(SIE_FIFO_STATUS,m66291_read_reg(SIE_FIFO_STATUS)|TGL);
						break;
					default:
						break;
				}
				Control_End(CTRLEND); 
				Control_End_CCPL();
			}
			break;

		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/****************************************************************/
static void usb_dev_set_feature(USB_request*  req)
{
	BYTE ep_no = REQ_INDEX(req).endpoint.ep_num;

	if((device_status.state == ATTACHED) || (REQ_LENGTH(req) != 0))
	{
		SetStallEpx(EPT0);
		return;
	}
	switch(REQ_RECIPIENT(req))
	{
		case DEVICE_REQ:
			if(REQ_VALUE(req).feature.bSelector == DEVICE_REMOTE_WAKEUP)
				SetStallEpx(EPT0); //remote wakeup is not supported
			break;

		case INTERFACE_REQ:
			break;

		case ENDPOINT_REQ:
			if(REQ_VALUE(req).feature.bSelector != ENDPOINT_STALL ||
					(ep_no >= EPT_LAST) || (usb_dev_endpoints[device_status.curAltSetting][ep_no] == NULL))
				SetStallEpx(EPT0);
			else
			{//set appropriate endpoint to stall state
				SetStallEpx(ep_no);            
				zero_length_data_responseEP0();
			}
			break;

		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/****************************************************************/
static void usb_dev_set_address(USB_request *req)
{
	ushort address;
	if(IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x0) || 
			(device_status.state == CONFIGURED)) 
	{
		SetStallEpx(EPT0);
		return;
	}
	switch(REQ_RECIPIENT(req))
	{
		case DEVICE_REQ://ENABLE_DEFAULT_ADDRESS
			address = REQ_VALUE(req).as_bytes.lsb;
			address= address&USB_ADDR;
			m66291_set_reg(USB_ADDRESS,address);
			if(REQ_VALUE(req).as_bytes.lsb == 0x0){
				device_status.state =  ATTACHED;
			}
			else   
				device_status.state =  ADDRESSED;
			break;

		case INTERFACE_REQ:
		case ENDPOINT_REQ:
		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			return;
			break;
	}
	Control_End(CTRLEND);
	Control_End_CCPL();
}
/****************************************************************/
static void usb_std_dev_get_descriptor(USB_request*   req)
{
	int desc_length = 0;
	int desc_index = REQ_VALUE(req).descriptor.bDescriptorIndex;
	BYTE *desc_buf = NULL;
	int max_desc_length = REQ_LENGTH(req);
	switch(REQ_RECIPIENT(req))   
	{
		case DEVICE_REQ:
			switch(REQ_VALUE(req).descriptor.bDescriptorType)
			{
				case DEVICE_DESCRIPTOR:
					desc_length = usb_device_desc.bLength;
					desc_buf = (BYTE *)&usb_device_desc;
					break;

				case CONFIG_DESCRIPTOR:
					desc_length = usb_dev_long_config_desc.usb_dev_config_desc.wTotalLength;
					desc_buf = (BYTE *)&usb_dev_long_config_desc;
					break;

				case STRING_DESCRIPTOR:
					if(desc_index < STR_LAST_INDEX && desc_index >= 0)   
					{
						desc_length = string_descs[desc_index]->bLength;
						desc_buf = (BYTE *)string_descs[desc_index];
						break;
					}

				default: //wrong string index
					SetStallEpx(EPT0);
					return;
			}
			desc_length = (desc_length < max_desc_length)? desc_length : max_desc_length;

			if(desc_length % EP0_FIFO_SIZE)
				device_buffers.zero_data=0;

			else
				device_buffers.zero_data=1;

			send_control_data(desc_buf, desc_length);

			break;

		case INTERFACE_REQ:
		case ENDPOINT_REQ:
		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}//switch(REQ_RECIPIENT)
}
/****************************************************************/
static void usb_std_dev_set_descriptor(USB_request *req)
{//not supportted  optional
	SetStallEpx(EPT0);
}
/****************************************************************/
static void usb_dev_get_config(USB_request*  req)
{
	if((device_status.state==ATTACHED) || IS_REQ_VALUE_NOT_ZERO(req) || IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x1))  
	{
		SetStallEpx(EPT0);
		return;
	}
	switch(REQ_RECIPIENT(req))  
	{
		case DEVICE_REQ://returns configuration number as appears at the device configuration register
			device_buffers.state = (device_status.state == CONFIGURED)?
				usb_dev_long_config_desc.usb_dev_config_desc.bConfigurationValue : 0;

			send_control_data((BYTE *)&device_buffers.state, sizeof(device_buffers.state));
			break;

		case INTERFACE_REQ:
		case ENDPOINT_REQ:

		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/****************************************************************/
static void usb_dev_set_config(USB_request*  req)
{
	int i;
	if(IS_REQ_INDEX_NOT_ZERO(req) || (REQ_LENGTH(req) != 0))  
	{
		SetStallEpx(EPT0);
		return;
	}

	switch(REQ_RECIPIENT(req))
	{
		case DEVICE_REQ:
			if(device_status.state == ATTACHED)  
			{
				SetStallEpx(EPT0);
				break;
			}
			if(REQ_VALUE(req).as_bytes.lsb == 0) 
			{
				if(device_status.state == CONFIGURED) 
				{//Deactivate current configuration
					for (i=1; i<MAX_NUM_ENDPOINTS; i++)  
					{
						if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
							dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
					}
					device_status.state = ADDRESSED;
					zero_length_data_responseEP0();
				}
				else{
					SetStallEpx(EPT0);
				}
			}
			else if(REQ_VALUE(req).as_bytes.lsb == usb_dev_long_config_desc.usb_dev_config_desc.bConfigurationValue) 
			{
				for (i=1; i< MAX_NUM_ENDPOINTS; i++) 
				{
					if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
						dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
				}
				device_status.curAltSetting = 0;
				//Activate this configuration
				reset_usb_nt();
				for(i=0; i< MAX_NUM_ENDPOINTS; i++)
				{
					if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
					{
						dev_enable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
					}
				}
				endpoint_status_init();
				device_status.state = CONFIGURED;

				if(device_status.state == CONFIGURED) 
				{
					spin_lock(&DevState.lock);
					DevState.UsbState.BusState = USB_CONNECTED;
					spin_unlock(&DevState.lock);			 
#ifdef _SELF_MODULE_UNLOAD_					 
					UsbBusError = 0;
#endif	
					config_ep_fifos();	 

				}
			}
			else //wrong confpiguration value
			{
				SetStallEpx(EPT0);
			}
			break;

		case INTERFACE_REQ:
		case ENDPOINT_REQ:
		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}

	Control_End(CTRLEND);
	Control_End_CCPL();

}
/****************************************************************/
static void usb_dev_get_interface(USB_request*   req)
{
	BYTE interface_no  = REQ_INDEX(req).as_bytes.lsb;
	if((device_status.state != CONFIGURED) || (interface_no != 0) || 
			IS_REQ_VALUE_NOT_ZERO(req) || (REQ_LENGTH(req) != 0x1))
	{
		SetStallEpx(EPT0);
		return;
	}

	switch(REQ_RECIPIENT(req)) 
	{
		case INTERFACE_REQ:
			device_buffers.state = device_status.curAltSetting;
			send_control_data((BYTE *)&device_buffers.state, sizeof(device_buffers.state));
			break;
		case DEVICE_REQ:
		case ENDPOINT_REQ:
		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/****************************************************************/
static void usb_dev_set_interface(USB_request*   req)
{
	BYTE interface_no  = REQ_INDEX(req).as_bytes.lsb;
	BYTE altSet = REQ_VALUE(req).as_bytes.lsb;
	int i;
	//only single interface is supported
	if((device_status.state != CONFIGURED) || (interface_no != 0) 
			|| IS_REQ_VALUE_NOT_ZERO(req) || (REQ_LENGTH(req) != 0)) 
	{
		SetStallEpx(EPT0);
		return;
	}
	switch(REQ_RECIPIENT(req))
	{
		case INTERFACE_REQ:  //we have to revisit this place if we want to support
			if(altSet > 0)   //alternate settings at least 1, now we have 0 settings
			{//Not supported Alternative setting
				SetStallEpx(EPT0);
				break;
			}
			if(altSet != device_status.curAltSetting)
			{
				//Change setting & deactivate previous setting
				for(i=1; i< MAX_NUM_ENDPOINTS; i++) 
				{
					if(usb_dev_endpoints[device_status.curAltSetting][i] != NULL)
						dev_disable_epx(usb_dev_endpoints[device_status.curAltSetting][i]);
				}
				device_status.curAltSetting = altSet;

				//Activate this configuration
				reset_usb();
			}
			zero_length_data_responseEP0();
			break;

		case DEVICE_REQ:
		case ENDPOINT_REQ:
		case OTHER_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/****************************************************************/
static void usb_dev_sync_frame(USB_request*  req)
{//optional not supported
	SetStallEpx(EPT0);
}
/****************************************************************/

/********************************************************
 ** vendor requests
 *********************************************************/
void Cmd_Data_Parser(USB_request* req)
{
	BYTE  command = REQ_VENDOR(req);
	WORD  reqlen  = REQ_LENGTH(req);

	switch(command) 
	{
		case BULK_DATA_OUT_AMOUNT:
			gusbtransf.BulkTfout.dataSize = ((DWORD)(REQ_VALUE(req).lsw)|(((DWORD)(REQ_INDEX(req).msw)) << 16));
			gusbtransf.BulkTfout.rest2process = gusbtransf.BulkTfout.dataSize;
			gusbtransf.BulkTfout.command = command;
			//wake up the read function
			break;

		case BULK_DATA_IN_AMOUNT:
			gusbtransf.BulkTfin.dataSize = ((DWORD)(REQ_VALUE(req).lsw)|(((DWORD)(REQ_INDEX(req).msw)) << 16));
			gusbtransf.BulkTfin.rest2process = gusbtransf.BulkTfin.dataSize;
			gusbtransf.BulkTfin.command = command;
			break;

		case BULK_DATA_IN_STREAM:
			gusbtransf.BulkTfin.command = BULK_DATA_IN_STREAM;
			break;

		case CTRL_DATA_OUT:
			gusbtransf.CtrlTfout.dataSize = reqlen;
			gusbtransf.CtrlTfout.rest2process = reqlen;
			gusbtransf.CtrlTfout.command = command;

			if(gusbtransf.CtrlTfout.dataSize)   gusbtransf.CtrlTfout.ready = NO;
			break;

		case CTRL_DATA_IN:
		case BULK_DATA_OUT_STREAM:
			break;

		default:
			SetStallEpx(EPT0);
			break;
	}

	//re-enable receive of data
	FlushRx(EPT0);

	if(gusbtransf.CtrlTfout.rest2process == 0) 
		zero_length_data_responseEP0();
}
/****************************************************************/
void dev_req_handler(void)
{
	BYTE *msg = ctrl_receive_buf.data;

	USB_request *req = (USB_request*) msg;


	if(msg == NULL){
		return;
	}
	switch(REQ_TYPE(req)) 
	{
		case STANDARD_REQ:
			(*usb_std_device_req[REQ_DEVICE(req)])(req);

			break;

		case VENDOR_REQ:
			(*usb_vendor_device_req[REQ_VENDOR_TYPE(req)])(req);
			break;

		case CLASS_REQ:
		default:
			SetStallEpx(EPT0);
			break;
	}
}
/**********************************************************/
static inline BYTE USB_Transmit_Data(BYTE *tdata, int data_size, EndPoint ep_num,int packet)
{
	BYTE size;

	//assure that data size is no more then fifo length
	size = min(data_size, fifo_sizes[ep_num]);
	writefifo(tdata, ep_num, data_size,packet);

	return size;
}
/**********************************************************/
static inline BYTE USB_Retransmit_Data(EndPoint ep_num)
{
	BYTE size;
	// retransmit
	tx_enable_epx(ep_num);

	return size;
}
/**********************************************************/
inline int SendInterruptData(BYTE *idata, DWORD size, int ep_no)
{
	return USB_Transmit_Data(idata, size, ep_no,0);
}
/*************************************************************/
inline void SendBulkData(unsigned int count_t,unsigned char *buff, int packet)
{ 
	BYTE cnt = 0;
	register U16  int_enable0_bk;
	register U16  buffer;

	spin_lock(&DevState.lock);
	DevState.UsbState.EpState[EPT1] = EP_IN_PROGRESS;
	spin_unlock(&DevState.lock);

	spin_unlock_wait(&gusbtransf.BulkTfin.lock);
	spin_lock(&gusbtransf.BulkTfin.lock);


	if(count_t > 0) 
	{
		//Is the is more date to be sent?
		SetBulkSendDataTimeout();

		cnt = min(gusbtransf.BulkTfin.rest2process, (DWORD)TX_BULK_EP_FIFO_SIZE);

		USB_Transmit_Data(buff, count_t , EPT1,packet);

		gusbtransf.BulkTfin.buffcnt += cnt;
		gusbtransf.BulkTfin.rest2process -= cnt;
	}
	else  
	{
		gusbtransf.BulkTfin.ready = YES;
		rtxEP1Len = 0; // not rtx needed!

		if(gusbtransf.BulkTfin.exitOnZLP == YES) 
		{
			//more data to send or shall be finished with a zero length package??
			SetBulkSendDataTimeout();   
			send_ZLP_2_finish_EPx(EPT1);
		}
	}

	if(gusbtransf.BulkTfin.ready == YES)  
	{
	}

	spin_unlock(&gusbtransf.BulkTfin.lock);
	int_enable0_bk = m66291_read_reg(INT_ENABLE0);
	buffer = int_enable0_bk & 0x00ff;
	m66291_set_reg(INT_ENABLE0, buffer);


	buffer = m66291_read_reg(USB_ENABLE);
	m66291_set_reg(INT_ENABLE0,int_enable0_bk);

	//    wake_up_interruptible(&gusbtransf.BulkTfin.waitq);

}

/******************************************************************/
static int BufferFull(void)
{
	int ret;

	spin_unlock_wait(&gusbtransf.BulkTfout.lock);
	spin_lock(&gusbtransf.BulkTfout.lock);
#ifdef DEBUG
	if(BULK_MAX_BUFFER_SIZE == gusbtransf.BulkTfout.buffcnt) 
#endif
		ret = (BULK_MAX_BUFFER_SIZE == gusbtransf.BulkTfout.buffcnt) ? YES : NO;

	spin_unlock(&gusbtransf.BulkTfout.lock);

	return ret;
}

void CR_Start()
{
	unsigned short buffer;
	EP0clear();
	buffer = m66291_read_reg(EP0_FIFO_SELECT);
	buffer      |= ISEL;
	m66291_set_reg(EP0_FIFO_SELECT, buffer);
}

/******************************************************************/

void Control_read(unsigned char *EP0Ptr,int count_t)
{
	unsigned short  count, max_size;
	register unsigned short  	buffer, odd,EP0Count;

	CR_Start();
	EP0Count=count_t;

	buffer = m66291_read_reg(CONTROL_TRANSFER);

	if( buffer & CTRR )
	{
		max_size = MAX_EP0_SEND_LEN;
	}
	else
	{
		max_size = count_t;
	}
	if( EP0Count < max_size )
	{
		count = EP0Count;
	}
	else
	{
		count = max_size;
	}

	m66291_set_reg(INT_STATUS3, m66291_read_reg(INT_STATUS3) & (~EPB0_REQ));		/* BEMP status clear */

	/* FIFO Write */
	m66291_set_reg(EP0_SEND_LEN, count);
	buffer = m66291_read_reg(EP0_FIFO_SELECT);

	/* 16bit access */
	if( !(buffer & EP0_Octl) )
	{
		odd = count % 2;
		count = count / 2;
		if ((unsigned long)EP0Ptr & 0x00000001)
		{
			while(count)
			{
				do
				{
					buffer = m66291_read_reg(EP0_FIFO_CONTROL);
				} while( buffer & E0req );
				buffer	  =  (unsigned short)(*EP0Ptr);
				EP0Ptr++;
				buffer	  |= ((unsigned short)(*EP0Ptr) << 8);
				EP0Ptr++;
				m66291_set_reg(EP0_FIFO_DATA, (unsigned short)buffer);
				count--;
			}
		}
		else
		{
			while(count)
			{
				m66291_set_reg(EP0_FIFO_CONTROL, m66291_read_reg(EP0_FIFO_CONTROL) & (~IVAL));
				buffer = m66291_read_reg(EP0_FIFO_CONTROL);
				buffer	= (unsigned short)(*(unsigned short*)EP0Ptr);
				m66291_set_reg(EP0_FIFO_DATA, (unsigned short)buffer);
				EP0Ptr++;
				EP0Ptr++;
				count--;
			}
		}

		if(odd)
		{
			count = odd;
			m66291_set_reg(EP0_FIFO_SELECT, m66291_read_reg(EP0_FIFO_SELECT) | EP0_Octl);		/* 8bit bus */
		}
	}

	/* 8bit access */
	while(count)
	{
		do
		{
			buffer = m66291_read_reg(EP0_FIFO_CONTROL);
		} while( buffer & E0req );
		buffer	= (unsigned short)(*EP0Ptr);
		m66291_set_reg(EP0_FIFO_DATA,(unsigned char)buffer);
		EP0Ptr++;
		count--;
	}


	if (EP0Count < max_size)
	{					/* check of data which transmits  */
		EP0Count = 0;
		m66291_set_reg(EP0_FIFO_CONTROL, m66291_read_reg(EP0_FIFO_CONTROL) | IVAL);
	}
	else
	{
		EP0Count -= max_size;
		m66291_set_reg(INT_ENABLE3, m66291_read_reg(INT_ENABLE3) | EPB0_ENB);		/* Buffer Empty interrupt enable */
	}
	Control_End(CTRLEND);
	mdelay(1);
}
/******************************************************************/
void ep5_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum,int packet)
{
	unsigned short  count, max_size;
	register unsigned short     buffer, EP0Count;

	if (gusbtransf.IntMeasTf.DevOpen == NO)
	{
		deactivate_usb();
		reset_usb();
		return;
	}
	buffer =  m66291_read_reg(CPU_FIFO_SELECT);
	buffer &= 0xFFF0;
	m66291_set_reg(CPU_FIFO_SELECT, buffer|epnum);

	m66291_set_reg( (EP1_0CONFIG + (epnum - 1) *4) , m66291_read_reg(EP1_0CONFIG +(epnum -1) *4 )|EPi_RWMD);

	m66291_set_reg( (EP1_1CONFIG + (epnum - 1) *4) , m66291_read_reg(EP1_1CONFIG + (epnum -1) *4)|EPi_Octl);
	m66291_set_reg((EP1_1CONFIG + (epnum -1) * 4) , m66291_read_reg((EP1_1CONFIG) + (epnum - 1) *4 ) | EPi_PID_BUF);

	EP0Count=count_t;
	max_size = MAX_EP0_SEND_LEN;
	if( EP0Count < max_size )
	{
		count = EP0Count;
	}
	else
	{
		count = max_size;
	}

	while(count)
	{
		do
		{
			buffer = m66291_read_reg(CPU_FIFO_CONTROL);
		} while(buffer & Creq);
		buffer  = (unsigned short)(*EP0Ptr);
		m66291_set_reg(CPU_FIFO_DATA,(unsigned char)buffer);
		EP0Ptr++;
		count--;
	}

	if (EP0Count < max_size)
	{                                   /* check of data which transmits  */
		EP0Count = 0;
		m66291_set_reg(CPU_FIFO_CONTROL, m66291_read_reg(CPU_FIFO_CONTROL) | IVAL);
	}
	else
	{
		EP0Count -= max_size;
		m66291_set_reg(INT_ENABLE3, m66291_read_reg(INT_ENABLE3) | EPB0_ENB);          /* Buffer Empty interrupt enable */
	}

	m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT) & (~CPU_EP));
	buffer = m66291_read_reg(CPU_FIFO_CONTROL);
	buffer = (buffer & (~IVAL)) | BCLR;
	m66291_set_reg(CPU_FIFO_CONTROL, buffer);
	if(packet==last_pack){
		mdelay(2);
	 	flag = 1;
    		wake_up_interruptible(&wq);
	}
}

/******************************************************************/
#define EPX_0CONFIG(epx) (EP1_0CONFIG+(epx-1)*4)
#define EPX_1CONFIG(epx) (EP1_1CONFIG+(epx-1)*4)
extern int gSendBulkErr;
void epx_write(unsigned char *EP0Ptr,int count_t,unsigned char epnum,int packet)
{
  register unsigned short buffer;
  unsigned short          div=0, mod=0, EPi_Buf;
  unsigned long           startJiffies;
  
  buffer =  m66291_read_reg(CPU_FIFO_SELECT);
  buffer &= 0xFFF0; 
  m66291_set_reg(CPU_FIFO_SELECT, buffer|epnum);

  if(count_t>=PCK_SIZE)
  {
    div=count_t/PCK_SIZE;
    mod=count_t%PCK_SIZE;
    if(div<15){
      if(mod>=1)
        div=div+1;
    }
  }
  else ;
  div=div<<8;
  
  EPi_Buf  = m66291_read_reg(EPX_0CONFIG(epnum));
  EPi_Buf &= 0xf0ff;
  EPi_Buf |= div;  
  
  m66291_set_reg(EPX_0CONFIG(epnum), m66291_read_reg(EPX_0CONFIG(epnum))|EPi_RWMD);
  m66291_set_reg(EPX_1CONFIG(epnum), m66291_read_reg(EPX_1CONFIG(epnum))|EPi_Octl);
  m66291_set_reg(EPX_0CONFIG(epnum), m66291_read_reg(EPX_0CONFIG(epnum))|EPi_Buf);
  
  while(count_t)
  {
    startJiffies = jiffies;
    do
    {
      buffer = m66291_read_reg(CPU_FIFO_CONTROL);
      if (jiffies - startJiffies > 1000){
        printk("ERR: m66291: Connect receiver\n");
        m66291_set_reg(EPX_1CONFIG(epnum),m66291_read_reg(EPX_1CONFIG(epnum))|EPi_ACLR); 
	m66291_set_reg(EPX_1CONFIG(epnum),m66291_read_reg(EPX_1CONFIG(epnum))&(~EPi_ACLR));
        m66291_set_reg(CPU_FIFO_CONTROL,(m66291_read_reg(CPU_FIFO_CONTROL)&~IVAL)|BCLR);
        gSendBulkErr = -EAGAIN;
        flag = 1;
        wake_up_interruptible(&wq);
        return;
      }
    } while(buffer & Creq);
    buffer	= (unsigned short)(*EP0Ptr);
    m66291_set_reg(CPU_FIFO_DATA,(unsigned char)buffer);
    EP0Ptr++;
    count_t--;
  }
  
  m66291_set_reg(EPX_1CONFIG(epnum), m66291_read_reg(EPX_1CONFIG(epnum)) | EPi_PID_BUF);
  m66291_set_reg(CPU_FIFO_CONTROL, m66291_read_reg(CPU_FIFO_CONTROL) | IVAL);
 
  startJiffies = jiffies;
  do
  {
    buffer = m66291_read_reg(CPU_FIFO_CONTROL);
    if (jiffies - startJiffies > 1000){
      printk("ERR: m66291: Connect receiver\n");
      m66291_set_reg(EPX_1CONFIG(epnum),m66291_read_reg(EPX_1CONFIG(epnum))|EPi_ACLR); 
      m66291_set_reg(EPX_1CONFIG(epnum),m66291_read_reg(EPX_1CONFIG(epnum))&(~EPi_ACLR));
      m66291_set_reg(CPU_FIFO_CONTROL,(m66291_read_reg(CPU_FIFO_CONTROL)&~IVAL)|BCLR);
      gSendBulkErr = -EAGAIN;
      flag = 1;
      wake_up_interruptible(&wq);
      return;
    }
  } while(buffer & Creq);
  
  m66291_set_reg(CPU_FIFO_SELECT, m66291_read_reg(CPU_FIFO_SELECT) & (~CPU_EP));
  buffer = m66291_read_reg(CPU_FIFO_CONTROL);
  buffer = (buffer & (~IVAL)) | BCLR;
  m66291_set_reg(CPU_FIFO_CONTROL, buffer);
  
  if(packet==last_pack){
    mdelay(2);
    flag = 1;
    wake_up_interruptible(&wq);
  }
}
/******************************************************************/

