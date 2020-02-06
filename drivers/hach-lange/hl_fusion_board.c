/* ========================================================================
 *
 *  Hach-Lange fusion board driver
 *  created: 2015-07-06 Jochen Sparbier
 *
 *   This driver handles the following hardware extras on the Fusion Board:
 *  1.) Check if left button was pushed during booting of linux.
 *  2.) Set and query staus for both fans
 *  3.) Set and query LED status (red/green/off).
 *
 *   All features are accessible in the sysfs directory tree.
 *
 * ======================================================================== */


#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/io.h>



#define CARDNAME	"hl_fusion"
#define DRV_VERSION	"0.10"




/* -----------------------------------------------------------------------------
 * prototypes:
 * -----------------------------------------------------------------------------*/
static ssize_t button_left_pressed_show( struct device *dev, struct device_attribute *attr, char *buf );

static ssize_t led_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t led_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count );

static ssize_t fans_power_sh4board_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t fans_power_sh4board_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count );


/* -----------------------------------------------------------------------------
 * attributes:
 * -----------------------------------------------------------------------------*/
static DEVICE_ATTR( button_left_pressed, S_IRUGO, button_left_pressed_show, NULL/*button_left_pressed_store*/ );
static DEVICE_ATTR( led, S_IRUGO | S_IWUGO, led_show, led_store );
static DEVICE_ATTR( fans_power_sh4board, S_IRUGO | S_IWUGO, fans_power_sh4board_show, fans_power_sh4board_store );


/* -----------------------------------------------------------------------------
 *  attribute led:
 * -----------------------------------------------------------------------------*/

#define PORT_PZCR	0xA405014C
#define PORT_PZDR	0xA405016C


static ssize_t led_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t status = 0;
	int led = 2;
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8 = 0x00;

	/*
	 * configure PTZ.6 and PTZ.7 to operate as output pin, bits 5,4 = 01
	 */
	reg_val_u16 = readw( PORT_PZCR );
	/*	printk("reading PORT_PZCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PZCR);  */
	reg_val_u16 |= 0x5000;
	reg_val_u16 &= ~0xA000;
	/*	printk("Configure PORT_PZCR with 0x%04X\n", reg_val_u16);  */
	writew( reg_val_u16, PORT_PZCR );

	/*
	 * set PTZ.6 to high/low and and PTZ.7 to low/high level
	 */
	reg_val_u8 = readb( PORT_PZDR );

	if (reg_val_u8 & 0x80) {
		if (reg_val_u8 & 0x40) {
			led = 3;
		}
		else {
			led = 1;
		}
	}
	else {
		if (reg_val_u8 & 0x40) {
			led = 2;
		}
		else {
			led = 0;
		}
	}

	status += snprintf( &buf[status], PAGE_SIZE, "%d\n", led );
	//printk( KERN_INFO "%s %s() %d\n", CARDNAME, __FUNCTION__, led );
	return status;
}


static ssize_t led_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	int new_val = 0, ret;
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8 = 0x00;

	ret = sscanf( buf, "%d", &new_val );
	if (ret != 1) {
		printk( KERN_ERR "%s %s() cannot parse\n", CARDNAME, __FUNCTION__ );
	}


	/*
	 * configure PTZ.6 and PTZ.7 to operate as output pin, bits 5,4 = 01
	 */
	reg_val_u16 = readw( PORT_PZCR );
	/*	printk("reading PORT_PZCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PZCR);  */
	reg_val_u16 |= 0x5000;
	reg_val_u16 &= ~0xA000;
	/*	printk("Configure PORT_PZCR with 0x%04X\n", reg_val_u16);  */
	writew( reg_val_u16, PORT_PZCR );

	/*
	 * set PTZ.6 to high/low and and PTZ.7 to low/high level
	 */
	reg_val_u8 = readb( PORT_PZDR );
	/*	printk("reading PORT_PZDR = 0x%02X from address 0x%X\n", reg_val_u8, PORT_PZDR);  */
	switch (new_val) {
	case 0: /* led off (both to GND) */
		reg_val_u8 &= ~0x80;
		reg_val_u8 &= ~0x40;
		break;
	case 1: /* led green */
		//printk("Enabling LED (green)\n");
		reg_val_u8 |= 0x80;
		reg_val_u8 &= ~0x40;
		break;
	case 2: /* led red */
		reg_val_u8 &= ~0x80;
		reg_val_u8 |= 0x40;
		break;
	case 3: /* led off (both to +Vcc) */
		reg_val_u8 |= 0x80;
		reg_val_u8 |= 0x40;
		break;
	default:
		printk( KERN_ERR "%s %s() %d invalid\n", CARDNAME, __FUNCTION__, new_val );
		break;
	}
	/*	printk("Configure PORT_PZDR with 0x%02X\n", reg_val_u8);  */
	writeb( reg_val_u8, PORT_PZDR );

	//printk( KERN_INFO "%s %s() %d\n", CARDNAME, __FUNCTION__, new_val );
	return strnlen( buf, PAGE_SIZE );
}


/* -----------------------------------------------------------------------------
 *  attribute fans_power_sh4board:
 * -----------------------------------------------------------------------------*/

#define PORT_PDCR   0xA4050106
#define PORT_PDDR   0xA4050126


static ssize_t fans_power_sh4board_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t status = 0;
	int fans_power_sh4board = 1;
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8 = 0x00;

	/*
	 * configure PTD.2 to operate as output pin, bits 5,4 = 01
	 */
	reg_val_u16 = readw( PORT_PDCR );
	/*	printk("reading PORT_PDCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PDCR);  */
	reg_val_u16 |= 0x0010;
	reg_val_u16 &= ~0x0020;
	/*	printk("Configure PORT_PDCR with 0x%04X\n", reg_val_u16);  */
	writew( reg_val_u16, PORT_PDCR );

	/*
	 * set PTD.2 to high level
	 */
	reg_val_u8 = readb( PORT_PDDR );
	fans_power_sh4board = ((reg_val_u8 & 0x04) ? 1 : 0);

	status += snprintf( &buf[status], PAGE_SIZE, "%d\n", fans_power_sh4board );
	printk( KERN_INFO "%s %s() %d\n", CARDNAME, __FUNCTION__, fans_power_sh4board );
	return status;
}


static ssize_t fans_power_sh4board_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	int new_val = -1, ret;
	uint16_t reg_val_u16 = 0x0000;
	uint8_t reg_val_u8 = 0x00;

	ret = sscanf( buf, "%d", &new_val );
	if (ret != 1) {
		printk( KERN_ERR "%s %s() cannot parse\n", CARDNAME, __FUNCTION__ );
	}

	/*
	 * configure PTD.2 to operate as output pin, bits 5,4 = 01
	 */
	reg_val_u16 = readw( PORT_PDCR );
	/*	printk("reading PORT_PDCR = 0x%04X from address 0x%X\n", reg_val_u16, PORT_PDCR);  */
	reg_val_u16 |= 0x0010;
	reg_val_u16 &= ~0x0020;
	/*	printk("Configure PORT_PDCR with 0x%04X\n", reg_val_u16);  */
	writew( reg_val_u16, PORT_PDCR );

	/*
	 * set PTD.2 to high level
	 */
	reg_val_u8 = readb( PORT_PDDR );
	/*	printk("reading PORT_PDDR = 0x%02X from address 0x%X\n", reg_val_u8, PORT_PDDR);  */
	if (new_val != 0) {
		reg_val_u8 |= 0x04;
	}
	else {
		reg_val_u8 &= ~0x04;
	}
	/*	printk("Configure PORT_PDDR with 0x%02X\n", reg_val_u8);  */
	//printk( "Starting Fan\n" );
	writeb( reg_val_u8, PORT_PDDR );

	printk( KERN_INFO "%s %s() %d\n", CARDNAME, __FUNCTION__, new_val );
	return strnlen( buf, PAGE_SIZE );
}



/* -----------------------------------------------------------------------------
 *  attribute button_left_pressed:
 * -----------------------------------------------------------------------------*/

#define CPORT_FUSIONBOARD_BUTTON_LEFT      (0xa4050106)  /* PDCR */
#define PORT_FUSIONBOARD_BUTTON_LEFT       (0xa4050126)  /* PDDR */
#define BIT_FUSIONBOARD_BUTTON_LEFT        (3)           /* Bit number */

static short button_left_pressed = 0;

/* called during XXX_probe(): */
static void button_left_pressed_detect( void )
{
	unsigned short is_pressed = 0;

	/*
	 * read S02 form fusion board ZBB057
	 * SODIMM connector #98, SDC1 D1 -- PTD3
	 */

	__raw_writew( (__raw_readw( CPORT_FUSIONBOARD_BUTTON_LEFT ) | (0x3 << (2 * BIT_FUSIONBOARD_BUTTON_LEFT))),
		 CPORT_FUSIONBOARD_BUTTON_LEFT
		 );

	is_pressed = __raw_readb( PORT_FUSIONBOARD_BUTTON_LEFT );
	/* pressing gives a 0: */
	is_pressed = (~is_pressed) & (1 << BIT_FUSIONBOARD_BUTTON_LEFT);

	/* there is no way to reset button_left_pressed_set */
	printk( "hl_fusion: %s(0x%x)\n", __FUNCTION__, (unsigned int) is_pressed );
	button_left_pressed |= is_pressed;
	return;
}


static ssize_t button_left_pressed_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t status = 0;

	status += snprintf( &buf[status], PAGE_SIZE, "%d\n", (button_left_pressed ? 1 : 0) );
	return status;
}




/* -----------------------------------------------------------------------------
 *  device and driver:
 * -----------------------------------------------------------------------------*/

static struct platform_device *pdev = NULL;

static struct platform_driver hl_fusion_driver = {
	.driver =
	{
		.name = "hl_fusion",
		.owner = THIS_MODULE,
	},
};


static int __init hl_fusion_init( void )
{
	int ret;
	printk( KERN_INFO "%s Hach-Lange Fusion Board Driver, V%s\n", CARDNAME, DRV_VERSION );

	ret = platform_driver_register( &hl_fusion_driver );
	printk( KERN_INFO "%s platform_driver_register() returns %d\n", CARDNAME, ret );

	pdev = platform_device_register_simple( "hl_fusion_board",
					 -1 /*	int id */,
					 NULL /*	const struct resource *res */,
					 0 /*	unsigned int num - of resources */ );
	ret = 0;
	ret |= device_create_file( &(pdev->dev), &dev_attr_button_left_pressed );
	button_left_pressed_detect( );
	ret |= device_create_file( &(pdev->dev), &dev_attr_led );
	ret |= device_create_file( &(pdev->dev), &dev_attr_fans_power_sh4board );

	ret = (ret ? 1 : 0);
	return ret;
}


module_init( hl_fusion_init );

MODULE_AUTHOR( "Jochen Sparbier" );
MODULE_DESCRIPTION( "Hach-Lange Fusion Board driver" );
MODULE_LICENSE( "GPL" );
MODULE_ALIAS( "platform:hl_fusion" );
