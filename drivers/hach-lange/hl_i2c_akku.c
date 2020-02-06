
//#define MODULE        // zum testen wird es ein modul

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/platform_device.h>


#include <asm/uaccess.h>

#include <linux/ioport.h>
#include <asm/io.h>
// #include <asm/sc1000.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

// #include "arch/sh/boards/mach-emtrion/hico7723/sh7723.h"

static int           i2c_debug  = 0;
static int           i2c_udelay = 12;
static unsigned char PortData   = 0;
static unsigned int  PortCtrl   = 0;

#define PZDR 0xA405016C // DATA
#define PZCR 0xA405014C // CONTROL
#define HIZC 0xA405015C // INIT

#define PZDR_BIT_SDA  6
#define PZDR_BIT_SCL  4
#define PZCR_BIT_SDA 12 // 12,13
#define PZCR_BIT_SCL  8 //  8, 9

#define PORT_MASK   0x3
#define PORT_INPUT  0x3
#define PORT_OUTPUT 0x1

#define I2C_HW_B_ELV 0x3

/* ----- local functions ----------------------------------------------	*/
static void hl_i2c_akku_setsda(void *data, int state)
{
  unsigned long flags;
  
  local_save_flags(flags);
  local_irq_disable();

  PortData = ctrl_inb(PZDR);
  PortCtrl = ctrl_inw(PZCR);
  if (state) 
  {
    // printk("SDA: IN : 1\n");
    PortCtrl &= ~(PORT_MASK   << PZCR_BIT_SDA);
    PortCtrl |=  (PORT_INPUT  << PZCR_BIT_SDA);
    PortData |=  (1           << PZDR_BIT_SDA);
    ctrl_outb(PortData, PZDR);
    ctrl_outw(PortCtrl, PZCR);
  } 
  else 
  {
    // printk("SDA: OUT: 0\n");
    PortCtrl &= ~(PORT_MASK   << PZCR_BIT_SDA);
    PortCtrl |=  (PORT_OUTPUT << PZCR_BIT_SDA);
    PortData &= ~(1           << PZDR_BIT_SDA);
    ctrl_outw(PortCtrl, PZCR);
    ctrl_outb(PortData, PZDR);
  }
  /* printk(" sda  Ctrl %x ; Data %x ; HIZC %x\n", */
  /*        (ctrl_inw(PZCR)>>PZCR_BIT_SDA)&PORT_MASK, */
  /*        (ctrl_inb(PZDR)>>PZDR_BIT_SDA)&1, */
  /*        (ctrl_inw(HIZC)>>3)&1); */
  
  local_irq_restore(flags);
}

static void hl_i2c_akku_setscl(void *data, int state)
{
  unsigned long flags;
  
  local_save_flags(flags);
  local_irq_disable();
  
  PortCtrl = ctrl_inw(PZCR);
  PortData = ctrl_inb(PZDR);
  if (state) 
  {
    // printk("SCL: IN : 1\n");
    PortCtrl &= ~(PORT_MASK  << PZCR_BIT_SCL);
    PortCtrl |=  (PORT_INPUT << PZCR_BIT_SCL);
    PortData |=  (1          << PZDR_BIT_SCL);
    ctrl_outb(PortData, PZDR);
    ctrl_outw(PortCtrl, PZCR);
  } 
  else 
  {
    // printk("SCL: OUT: 0\n");
    PortCtrl &= ~(PORT_MASK   << PZCR_BIT_SCL);
    PortCtrl |=  (PORT_OUTPUT << PZCR_BIT_SCL);
    PortData &= ~(1           << PZDR_BIT_SCL);
    ctrl_outw(PortCtrl, PZCR);
    ctrl_outb(PortData, PZDR);
  }
  /* printk(" scl  Ctrl %x ; Data %x ; HIZC %x\n", */
  /*        (ctrl_inw(PZCR)>>PZCR_BIT_SCL)&PORT_MASK, */
  /*        (ctrl_inb(PZDR)>>PZDR_BIT_SCL)&1, */
  /*        (ctrl_inw(HIZC)>>3)&1); */
  
  local_irq_restore(flags);
}

static int hl_i2c_akku_getscl(void *data)
{
  unsigned char cTmp = ((ctrl_inb(PZDR) & (1 << PZDR_BIT_SCL)))>0;
  // printk("getscl: %d\n",cTmp);
  return cTmp;
}

static int hl_i2c_akku_getsda(void *data)
{
  unsigned char cTmp = ((ctrl_inb(PZDR) & (1 << PZDR_BIT_SDA)))>0;
  // printk("getsda: %d\n",cTmp);
  return cTmp;
}

static int hl_i2c_akku_init(void)
{
  // HiZC3 bit3: I/O buffer operates normally=0
  PortCtrl  = ctrl_inw(HIZC);
  PortCtrl &= ~(0x8); // HiZC3
  ctrl_outw(PortCtrl, HIZC);
  
  // PZCR bit12,13 (PTZ6,SDA), bit8,9 (PTZ4,SCL)
  PortCtrl = ctrl_inw(PZCR);
  PortCtrl &= ~(PORT_MASK  << PZCR_BIT_SDA);
  PortCtrl &= ~(PORT_MASK  << PZCR_BIT_SCL);
  PortCtrl |=  (PORT_INPUT << PZCR_BIT_SDA);
  PortCtrl |=  (PORT_INPUT << PZCR_BIT_SCL);
  ctrl_outw(PortCtrl, PZCR);

  return 0;
}

static void hl_i2c_akku_exit(void)
{
  /*	release_region( base , (base == 0x3bc)? 3 : 8 );  */
}

/* ------------------------------------------------------------------------
 * Encapsulate the above functions in the correct operations structure.
 * This is only done when more than one hardware adapter is supported.
 */
static struct i2c_algo_bit_data hl_i2c_akku_data = {
  /* NULL, */
  .setsda  = hl_i2c_akku_setsda,
  .setscl  = hl_i2c_akku_setscl,
  .getsda  = hl_i2c_akku_getsda,
  .getscl  = hl_i2c_akku_getscl,
  .udelay  = 12,  /* 12, 12, */
  .timeout = 100, /* 100, */
};

static struct i2c_adapter hl_i2c_akku_ops = {
  .owner     = THIS_MODULE,
  .id        = I2C_HW_B_ELV,
  .name      = "HL akku i2c bus",
  .algo_data = &hl_i2c_akku_data,
};

int __init i2c_bit_akku_init(void)
{
  printk("hl_i2c_akku: HachLange - i2c port module\n");
  hl_i2c_akku_data.udelay = i2c_udelay;
  hl_i2c_akku_data.data=NULL;
  if (hl_i2c_akku_init()==0)
  {
    if(i2c_bit_add_bus(&hl_i2c_akku_ops) < 0)
    {
      printk("ERR: i2c_bit_add_bus failed\n");
      return -ENODEV;
    }
  } 
  else
  {
    printk("ERR: hl_i2c_akku_init failed\n");
    return -ENODEV;
  }
  
  return 0;
}

int __init hl_i2c_akku_module_init(void)
{
  return i2c_bit_akku_init();
}

static void __exit hl_i2c_akku_module_exit(void)
{
  // hl_i2c_del_bus(&bit_hl_i2c_akku_ops);
  hl_i2c_akku_exit();
  return;
}

module_init(hl_i2c_akku_module_init);
module_exit(hl_i2c_akku_module_exit);

