/* NMI: set/get NMI flag for SH7723
 *
 * author: mschreiber
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/io.h>

#define HL_GEN_MAJOR  43

// commands for ioctl
#define HL_GEN_INIT   0x01

// registers
#define PSCR   0xA405011E
#define PWCR   0xA4050146
#define PSELD  0xA4050154
#define HIZCRB 0xA405015A
#define HIZCRD 0xA405015E
#define DRVCRA 0xA405018A

// prototypes
static int hl_gen_open (struct inode *inode, struct file *file);
static ssize_t hl_gen_read (struct file *file, 
                            char *buf, 
                            size_t count,
                            loff_t *offset);
static int hl_gen_ioctl (struct inode *inode, struct file *file,
                         unsigned int cmd, unsigned long arg);

#ifdef MODULE
extern int init_module(void);
extern int cleanup_module(void);
static int __init hl_gen_init(void);
#else
extern int __init hl_gen_init(void);
#endif

static struct file_operations hl_gen_fops = {
  ioctl:   hl_gen_ioctl,
  read:    hl_gen_read,
  open:    hl_gen_open,
};

static ssize_t hl_gen_read (struct file *file, 
                            char *buf, 
                            size_t count,
                            loff_t *offset)
{
  memset(buf,0,2); // currently not used
  // memcpy(buf,(char*)NMIFCR,2); // NMIFCR length = 16bit
  return 2;
}

static int hl_gen_ioctl (struct inode *inode, struct file *file,
                         unsigned int cmd, unsigned long arg)
{
  unsigned short sTmpPWCR, sTmpHIZCRD;
  unsigned short sTmpPSCR, sTmpHIZCRB;

  switch(cmd)
  {
    case HL_GEN_INIT:
      /* read registers */
      /******************/
      sTmpPWCR   = ctrl_inw(PWCR);
      sTmpHIZCRD = ctrl_inw(HIZCRD);
      sTmpPSCR   = ctrl_inw(PSCR);
      sTmpHIZCRB = ctrl_inw(HIZCRB);

      /* set values */
      /**************/
      printk("HL_GEN: set PTS5 (camera reset) to output\n");
      sTmpPSCR &= ~0x0C00;   // init to 0
      sTmpPSCR |=  0x0400;   // PTS5: PSCR bit10,11: output=01
      sTmpHIZCRB &= ~0x0008; // PTS5: HIZCRB bit3: i/o buffer operates normally=0

      printk("HL_GEN: set PTS6 (exposure) to output\n");
      sTmpPSCR &= ~0x3000;   // init to 0
      sTmpPSCR |=  0x1000;   // PTS6: PSCR bit12,13: output=01
      sTmpHIZCRB &= ~0x0004; // PTS6: HIZCRB bit2: i/o buffer operates normally=0

      printk("HL_GEN: set PTW0 (LED B - green) to output\n");
      sTmpPWCR &= ~0x0003;   // init to 0
      sTmpPWCR |=  0x0001;   // PTW0: PWCR bit0,1: output=01
      sTmpHIZCRD &= ~0x0100; // PTW0: HIZCRD bit8: i/o buffer operates normally=0

      printk("HL_GEN: set PTW1 (LED A - yellow) to output\n");
      sTmpPWCR &= ~0x000C;   // init to 0
      sTmpPWCR |=  0x0004;   // PTW1: PWCR bit2,3: output=01
      sTmpHIZCRD &= ~0x0200; // PTW1: HIZCRD bit9: i/o buffer operates normally=0

      printk("HL_GEN: set PTW4 (IRQ A) to input\n");
      sTmpPWCR |= 0x0300;    // PTW4: PWCR bit8,9: input=11      

      /* write back */
      /**************/
      ctrl_outw(sTmpPWCR,  PWCR);
      ctrl_outw(sTmpHIZCRD,HIZCRD);
      ctrl_outw(sTmpPSCR,  PSCR);
      ctrl_outw(sTmpHIZCRB,HIZCRB);
      
      /* test */
      /********/
      /* printk("PTS6: PSCR[13:12]=%x, HIZB[2]=%x\n",  */
      /*        (ctrl_inw(PSCR)>>12)&0x3, (ctrl_inw(HIZCRB)>>2)&0x1); */
      /* printk("PTS5: PSCR[11:10]=%x, HIZB[3]=%x\n",  */
      /*        (ctrl_inw(PSCR)>>10)&0x3, (ctrl_inw(HIZCRB)>>3)&0x1); */
      /* printk("PTW0: PWCR[1:0]=%x, PSELD[7:6]=%x, HIZD[8]=%x, DRVA[11:10]=%x\n", */
      /*        ctrl_inw(PWCR)&0x3,(ctrl_inw(PSELD)>>6)&0x3, */
      /*        (ctrl_inw(HIZCRD)>>8)&0x1, (ctrl_inw(DRVCRA)>>10)&0x3); */
      break;
      
    default:
      return -EINVAL;
      break;
  }
  return 0;
}

static int hl_gen_open (struct inode *inode, struct file *file)
{
  return 0;
}

int __init hl_gen_init(void)
{
  printk("HL_GEN init\n");
  
  if (register_chrdev(HL_GEN_MAJOR,"hl_gen",&hl_gen_fops)) 
  {
    printk("HL_GEN: unable to get major %d for HL_GEN\n",
           HL_GEN_MAJOR);
    return -EIO;
  }
  return 0;
}

int hl_gen_cleanup(void)
{
  unregister_chrdev(HL_GEN_MAJOR, "hl_gen");
  return 0;
}

int __init hl_gen_module_init(void)
{
  return hl_gen_init();
}

static void __exit hl_gen_module_exit(void)
{
  hl_gen_cleanup();
  return;
}

module_init(hl_gen_module_init);
module_exit(hl_gen_module_exit);
