/***************************************************************************
                          ucore.h  -  description
                             -------------------
    begin                : Fre Nov 5 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
    Author               : Nikhil Varghese
 ***************************************************************************/
#define  USBGAMMA_MAJOR   240
/**********************************
 ** 1st device for Measurement
 ++ 2nd deivce for Big data to send
 ++ 3rd device for vendor out request
 ***********************************/
#define USB_M66291_DEVICES  3

#define USB_M66291_CNTRL_DEVICE         0
#define USB_M66291_MEASUREMENT_DEVICE   1
#define USB_M66291_BULK_DEVICE          2
/**********************************************************/

#ifdef MODULE
static
#else
extern
#endif
/****************************
 ** proc entry declarations
 *****************************/
#ifdef CONFIG_PROC_FS
int __init    usb_init(void);

int        usb_cleanup(void);

int init_sh4_reg(void);
int init_USB_m66291_reg(void);
/////////////// device 0 -> cntrl (cntrl transfer)
static unsigned int usb_m66291_poll_cntrl(struct file *filp, poll_table *wait);
static int usb_m66291_read_cntrl(struct file *filp, char *buf, size_t count, loff_t *f_pos);

///////////////// devices 1 -> measurement
static unsigned int usb_m66291_poll_bulk(struct file *filp, poll_table *wait);
static int usb_m66291_read_bulk(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static int usb_m66291_write_measurement(struct file *filp, const char *buf, size_t count, loff_t *f_pos);

//////////////// devices 2 -> big data (bulk)
static unsigned int usb_m66291_poll_bulk(struct file *filp, poll_table *wait);
static int usb_m66291_write_bulk(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
static int usb_m66291_read_bulk(struct file *filp, char *buf, size_t count, loff_t *f_pos);

/***** all devices ******************/
static int usbdev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int usbdev_open(struct inode *inode, struct file *file);
static int usbdev_release(struct inode *inode, struct file *file);
#else /* undef CONFIG_PROC_FS */

void  GetUsbVersion(void);

#define usb_init() 0
#define usb_cleanup() 0

#endif /* CONFIG_PROC_FS */

struct file_operations usb_cntrl_fops = {
owner:    THIS_MODULE,
	  ioctl:    usbdev_ioctl,
	  read:     usb_m66291_read_cntrl,
	  poll:     usb_m66291_poll_cntrl,
	  release:  usbdev_release
};      

struct file_operations usb_measurement_fops = {
owner:    THIS_MODULE,
	  ioctl:    usbdev_ioctl,
	  read:     usb_m66291_read_bulk,
	  write:    usb_m66291_write_measurement,
	  poll:     usb_m66291_poll_bulk,
	  release:  usbdev_release,
};

struct file_operations usb_bulk_data_fops = {
owner:    THIS_MODULE,
	  ioctl:    usbdev_ioctl,
	  read:     usb_m66291_read_bulk,
	  write:    usb_m66291_write_bulk,
	  poll:     usb_m66291_poll_bulk,
	  release:  usbdev_release
}; 
/**********************************************
 ** minor number represents the array element !!!!!
 ***********************************************/
struct file_operations *usbn_fop_array[/*minor number*/]={
	&usb_cntrl_fops,
	&usb_measurement_fops,
	&usb_bulk_data_fops,    
};

/**********************************************************/
static struct file_operations usbdev_fops = {
open:    usbdev_open
};
