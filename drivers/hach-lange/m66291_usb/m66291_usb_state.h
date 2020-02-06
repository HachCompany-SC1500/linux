/***************************************************************************
                          UsbState.h  -  description
                             -------------------
    begin                : Die Okt 19 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
    Author               : Nikhil Varghese
***************************************************************************/


#ifndef _USBSTATE_H_
#define _USBSTATE_H_

#include "m66291_usb_drv.h"

/*****************
** USB state ****
*****************/
typedef enum {
  EP_NOT_ASSIGNED = -1,
  EP_ENABLED = 0,
  EP_DISABLED,
  EP_IN_PROGRESS,
  EP_STALL
} EP_STATES;

typedef enum {
   USB_DISCONNECTED = -1,
   USB_CONNECTED = 0   
} USB_STATES;

struct _USB_STATE {
   int  BusState;
   int  EpState[NUM_OF_ENDPOINTS];
};

struct _DEVICE_STATE {
     struct _USB_STATE   UsbState;
     spinlock_t          lock;
};

struct _DEVICE_STATE  DevState;

#endif /* _USBSTATE_H_ */
