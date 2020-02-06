/***************************************************************************
  usbdrv.h  -  description
  -------------------
begin                : Die Jul 13 2004
copyright            : (C) 2004 by Thomas Siegmund
email                : tsiegmund@hach-lange.de
Author               : Nikhil Varghese
 ***************************************************************************/

#ifndef _USBDRV_H_
#define _USBDRV_H_

typedef enum {
	EPT0 = 0,
	EPT1,
	EPT2,
	EPT3,
	EPT4,
	EPT5,
	EPT6,
	EPT7,
	EPT8,
	EPT9,
	EPT10,
	EPT11,
	EPT12,
	EPT13,
	EPT_LAST // EPT_14
} EndPoint;

typedef enum  {
	DETACHED = 0,
	ATTACHED_WAIT,
	ATTACHED, //default
	ADDRESSED,
	CONFIGURED
} USB_device_state;

typedef struct {
	USB_device_state   state;
	int         config;
	int         curAltSetting;
	EndPoint    endpts[7];
} DEVICE_STATUS;

typedef enum {
	EMPTY,
	FULL,
	WAIT_FOR_ACK
} FIFO_STATUS;

typedef struct {
	FIFO_STATUS  fifo_state;
	BYTE  toggle_bit;
} endpoint_status;


typedef struct {
	BYTE* data;
	unsigned int byte_cnts; //counter of data BYTEs currently stored in the buffer
}control_buffer;

#define IS_REQ_VALUE_NOT_ZERO(req) (REQ_VALUE(req).as_bytes.msb != 0x0 && REQ_VALUE(req).as_bytes.lsb != 0x0)
#define IS_REQ_INDEX_NOT_ZERO(req) (REQ_INDEX(req).as_bytes.msb != 0x0 && REQ_INDEX(req).as_bytes.lsb != 0x0)
//#define IS_EP0_STALLED ((UsbRd2B(ReadCtrlOutStat) & EPSTAL) || (UsbRd2B(ReadCtrlInStat) & EPSTAL))
//#define IS_EP_STALLED(ep_no) (UsbRd2B( + (ep_no)) & EPSTAL)

///////////////////////////////////////////
//All endpoints inclusive enpdpoint zero
#define NUM_OF_ENDPOINTS   7

#define NO_ENDPOINT_ZERO_SIZE    0
#define EP0_FIFO_SIZE           64 // 8
#define RX_BULK_EP_FIFO_SIZE    64
#define TX_BULK_EP_FIFO_SIZE    64
#define RX_INTR_EP_FIFO_SIZE     8
#define TX_INTR_EP_FIFO_SIZE_COMM     8
#define TX_INTR_EP_FIFO_SIZE_MEAS    64

#endif /* _USBDRV_H_ */
