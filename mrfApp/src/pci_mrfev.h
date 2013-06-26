#ifndef	PCI_MRFEV_H
#define	PCI_MRFEV_H

/*
  pci_mrfev.h -- Definitions for Micro-Research Event Generator /
                 Event Receiver Linux 2.6 driver

  Author: Jukka Pietarinen (MRF)
  Date:   29.11.2006

*/

#define DEVICE_MINOR_NUMBERS      4
#define MAX_MRF_DEVICES           8

#define PCI_DEVICE_ID_PLX_9030      0x9030
#define PCI_VENDOR_ID_MRF           0x1A3E
#define PCI_DEVICE_ID_MRF_PMCEVR200 0x10C8
#define PCI_DEVICE_ID_MRF_PMCEVR230 0x11E6
#define PCI_DEVICE_ID_MRF_PXIEVR220 0x10DC
#define PCI_DEVICE_ID_MRF_PXIEVG220 0x20DC
#define PCI_DEVICE_ID_MRF_PXIEVR230 0x10E6
#define PCI_DEVICE_ID_MRF_PXIEVG230 0x20E6

#define MODULE_VENDOR_ID_NOCONF     PCI_VENDOR_ID_PLX
#define MODULE_DEVICE_ID_NOCONF     PCI_DEVICE_ID_PLX_9030
#define MODULE_SUBVENDOR_ID_NOCONF  0
#define MODULE_SUBDEVICE_ID_NOCONF  0

#define DEVICE_MINORS 4
#define DEVICE_FIRST  0
#define DEVICE_EEPROM 0
#define DEVICE_XCF    1
#define DEVICE_FPGA   2
#define DEVICE_EV     3
#define DEVICE_LAST   3

/* Define the maximum number of words in EEPROM */
#define EEPROM_MAX_WORDS            256

/* Define space needed for data */
#define EEPROM_DATA_ALLOC_SIZE      0x00002000
/*
#define XCF_DATA_ALLOC_SIZE         0x00100000
#define FPGA_DATA_ALLOC_SIZE        0x00100000
*/
#define XCF_DATA_ALLOC_SIZE         0x000400
#define FPGA_DATA_ALLOC_SIZE        0x001000

#define XCF_BLOCK_SIZE              1024
#define XCF_ERASE_TCKS              140000000

#if 0	/* This stuff won't compile and link w/o other MRF src we don't have */
struct mrf_dev {
  int    access_mode;        /* Only one minor device is allowed open
                                at a time. access_mode hold either 0 for
                                no devices open or O_WRONLY or O_RDONLY. */
  int    access_device;      /* access_device holds the minor number of
                                the device open. */
  int    refcount_ev;        /* FPGA memory mapped register map reference
				count. We allow several simultaneos connections
				on this minor device and keep track on the
				number of these connections. */
  char             *eeprom_data;
  int              eeprom_data_size;
  char             *xcf_data;
  int              xcf_data_pos;
  char             *fpga_conf_data;
  int              fpga_conf_size;
  int              major;          /* Hold major number of device */
  struct cdev      cdev;
  struct semaphore sem;
  unsigned long    mrLC;           /* Here we hold the PCI address of BAR0 */
  void             *pLC;       
  unsigned long    lenLC;
  unsigned long    mrEv;
  void             *pEv;
  unsigned long    lenEv;
  unsigned int     jtag_refcount;
  int              fpgaid;
  int              xcfid;
  int              fpgairlen;
  int              xcfirlen;
  int              irq;            /* Interrupt line */
  struct fasync_struct *async_queue;
};

/* fops prototypes */

int ev_open(struct inode *inode, struct file *filp);
int ev_release(struct inode *inode, struct file *filp);
ssize_t ev_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos);
ssize_t ev_write(struct file *filp, const char __user *buf, size_t count,
		 loff_t *f_pos);
int ev_ioctl(struct inode *inode, struct file *filp,
	     unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
long ev_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif
int ev_fasync(int fd, struct file *filp, int mode);
int ev_remap_mmap(struct file *filp, struct vm_area_struct *vma);
irqreturn_t ev_interrupt(int irq, void *dev_id, struct pt_regs *regs);
#endif	/* This stuff won't compile and link w/o other MRF src we don't have */

#define EV_IOC_MAGIC 220
#define EV_IOCRESET  _IO(EV_IOC_MAGIC, 0)
#define EV_IOCIRQEN  _IO(EV_IOC_MAGIC, 1)
#define EV_IOCIRQDIS _IO(EV_IOC_MAGIC, 2)

#define EV_IOC_MAX   3

#define PLX9030_INTCSR_LINTI1_ENA  0x0001 /* LINTi1 enable */
#define PLX9030_INTCSR_LINTI1_POL  0x0002 /* LINTi1 polarity, 1 = active high */
#define PLX9030_INTCSR_LINTI1_STAT 0x0004 /* LINTi1 status, 1 = interrupt active */
#define PLX9030_INTCSR_LINTI2_ENA  0x0008 /* LINTi2 enable */
#define PLX9030_INTCSR_LINTI2_POL  0x0010 /* LINTi2 polarity, 1 = active high */
#define PLX9030_INTCSR_LINTI2_STAT 0x0020 /* LINTi2 status, 1 = interrupt active */
#define PLX9030_INTCSR_PCI_IRQENA  0x0040 /* PCI interrupt enable, 1 = enabled */
#define PLX9030_INTCSR_SWINT       0x0080 /* Software interrupt, 1 = generate PCI IRQ */
#define PLX9030_INTCSR_LINTI1_SENA 0x0100 /* LINTi1 select enable,
					     0 = level, 1 = edge triggerable */
#define PLX9030_INTCSR_LINTI2_SENA 0x0200 /* LINTi1 select enable,
					     0 = level, 1 = edge triggerable */
#define PLX9030_INTCSR_LINTI1_ICLR 0x0400 /* LINTi1 edge triggerable IRQ clear,
					     writing 1 clears irq */
#define PLX9030_INTCSR_LINTI2_ICLR 0x0800 /* LINTi2 edge triggerable IRQ clear,
					     writing 1 clears irq */

#define PLX9030_CNTRL_EESK         0x01000000
#define PLX9030_CNTRL_EECS         0x02000000
#define PLX9030_CNTRL_EEDI         0x04000000
#define PLX9030_CNTRL_EEDO         0x08000000
#define PLX9030_CNTRL_RESET        0x00780000

#define PLX9030_GPIOC_GPIO0_nWAIT  0x00000001
#define PLX9030_GPIOC_GPIO0_DIR_O  0x00000002
#define PLX9030_GPIOC_GPIO0_DATA   0x00000004
#define PLX9030_GPIOC_GPIO1_nLLOCK 0x00000008
#define PLX9030_GPIOC_GPIO1_DIR_O  0x00000010
#define PLX9030_GPIOC_GPIO1_DATA   0x00000020
#define PLX9030_GPIOC_GPIO2_nCS2   0x00000040
#define PLX9030_GPIOC_GPIO2_DIR_O  0x00000080
#define PLX9030_GPIOC_GPIO2_DATA   0x00000100
#define PLX9030_GPIOC_GPIO3_nCS3   0x00000200
#define PLX9030_GPIOC_GPIO3_DIR_O  0x00000400
#define PLX9030_GPIOC_GPIO3_DATA   0x00000800

#define PLX9030_GPIOC_TMS        PLX9030_GPIOC_GPIO1_DATA
#define PLX9030_GPIOC_TDI        PLX9030_GPIOC_GPIO3_DATA
#define PLX9030_GPIOC_TCK        PLX9030_GPIOC_GPIO0_DATA
#define PLX9030_GPIOC_TDO        PLX9030_GPIOC_GPIO2_DATA
#define PLX9030_GPIOC_RESET      0x00249000
#define PLX9030_GPIOC_INIT       0x00249C36
#define PLX9030_GPIOC_JTAG       0x00249412

#define XC2VP_CFGOUT    0xffc4
#define XC2VP_CFGIN     0xffc5
#define XC2VP_IDCODE    0xffc9
#define XC2VP_JPROG_B   0xffcb
#define XC2VP_JSTART    0xffcc
#define XC2VP_JSHUTDOWN 0xffcd
#define XC2VP_BYPASS    0xffff

#define XC2VP4_DID    0x0123E093L
#define XC2VP7_DID    0x0124A093L
#define XC2VP20_DID   0x01266093L
#define XC2VP30_DID   0x0127e093L
#define XCF08P_DID    0x05057093L
#define XCF16P_DID    0x05058093L
#define XC2VP4_IRLEN  10
#define XC2VP7_IRLEN  10
#define XC2VP20_IRLEN 14
#define XC2VP30_IRLEN 14
#define XCFXXP_IRLEN  16

#define XCFXXP_BYPASS            0xffff
#define XCFXXP_SAMPLE            0x0001
#define XCFXXP_EXTEST            0x0000
#define XCFXXP_IDCODE            0x00fe
#define XCFXXP_USERCODE          0x00fd
#define XCFXXP_HIGHZ             0x00fc
#define XCFXXP_CLAMP             0x00fa
#define XCFXXP_ISC_ENABLE        0x00e8
#define XCFXXP_XSC_ENABLEC       0x00e9
#define XCFXXP_ISC_PROGRAM       0x00ea
#define XCFXXP_ISC_ADDRESS_SHIFT 0x00eb
#define XCFXXP_ISC_READ          0x00f8
#define XCFXXP_ISC_ERASE         0x00ec
#define XCFXXP_ISC_DATA_SHIFT    0x00ed
#define XCFXXP_XSC_READ          0x00ef
#define XCFXXP_XSC_BLANK_CHECK   0x000d
#define XCFXXP_ISC_DISABLE       0x00f0
#define XCFXXP_ISC_NOOP          0x00e0
#define XCFXXP_XSC_CONFIG        0x00ee
#define XCFXXP_XSC_CLR_STATUS    0x00f4
#define XCFXXP_XSC_OP_STATUS     0x00e3
#define XCFXXP_XSC_MFG_READ      0x00f1
#define XCFXXP_XSC_CONFIG_PLUS   0x00f6
#define XCFXXP_XSC_UNLOCK        0xaa55
#define XCFXXP_XSC_DATA_BTC      0x00f2
#define XCFXXP_XSC_DATA_WRPT     0x00f7
#define XCFXXP_XSC_DATA_RDPT     0x0004
#define XCFXXP_XSC_DATA_UC       0x0006
#define XCFXXP_XSC_DATA_CC       0x0007
#define XCFXXP_XSC_DATA_DONE     0x0009
#define XCFXXP_XSC_DATA_CCB      0x000c
#define XCFXXP_XSC_DATA_BLANK    0x00f5
#define XCFXXP_XSC_DATA_SUCR     0x000e
#define XCFXXP_XSC_ADDRESS_DUMP  0x00e6

#if 0	/* This stuff won't compile and link w/o other MRF src we don't have */
struct Pci9030LocalConf
{
  unsigned int LAS0RR;    /* 0x00 Local Address Space 0 Range */
  unsigned int LAS1RR;    /* 0x04 Local Address Space 1 Range */
  unsigned int LAS2RR;    /* 0x08 Local Address Space 2 Range */
  unsigned int LAS3RR;    /* 0x0C Local Address Space 3 Range */
  unsigned int EROMRR;    /* 0x10 Expansion ROM Range */
  unsigned int LAS0BA;    /* 0x14 Local Address Space 0 Local Base Address */
  unsigned int LAS1BA;    /* 0x18 Local Address Space 1 Local Base Address */
  unsigned int LAS2BA;    /* 0x1C Local Address Space 2 Local Base Address */
  unsigned int LAS3BA;    /* 0x20 Local Address Space 3 Local Base Address */
  unsigned int EROMBA;    /* 0x24 Expansion ROM Local Base Address */
  unsigned int LAS0BRD;   /* 0x28 Local Address Space 0 Bus Region Descriptor */
  unsigned int LAS1BRD;   /* 0x2C Local Address Space 1 Bus Region Descriptor */
  unsigned int LAS2BRD;   /* 0x30 Local Address Space 2 Bus Region Descriptor */
  unsigned int LAS3BRD;   /* 0x34 Local Address Space 3 Bus Region Descriptor */
  unsigned int EROMBRD;   /* 0x38 Expansion ROM Bus Region Descriptor */
  unsigned int CS0BASE;   /* 0x3C Chip Select 0 Base Address */
  unsigned int CS1BASE;   /* 0x40 Chip Select 1 Base Address */
  unsigned int CS2BASE;   /* 0x44 Chip Select 2 Base Address */
  unsigned int CS3BASE;   /* 0x48 Chip Select 3 Base Address */
  unsigned short INTCSR;    /* 0x4C Interrupt Control/Status */
  unsigned short PROT_AREA; /* 0x4E Serial EEPROM Write-Protected Address Boundary */
  unsigned int CNTRL;     /* 0x50 PCI Target Response, Serial EEPROM, and
                       Initialization Control */
  unsigned int GPIOC;     /* 0x54 General Purpose I/O Control */
  unsigned int reserved1; /* 0x58 */
  unsigned int reserved2; /* 0x5C */
  unsigned int reserved3; /* 0x60 */
  unsigned int reserved4; /* 0x64 */
  unsigned int reserved5; /* 0x68 */
  unsigned int reserved6; /* 0x6C */
  unsigned int PMDATASEL; /* 0x70 Hidden 1 Power Management Data Select */
  unsigned int PMDATASCALE; /* 0x74 Hidden 2 Power Management Data Scale */
};

void evr_dumpLC(struct Pci9030LocalConf *pLC);
int eeprom_sprint(struct Pci9030LocalConf *pLC, char *buf, int size);
int eeprom_sscan(struct Pci9030LocalConf *pLC, char *buf);

void jtag_init(struct mrf_dev *ev_dev);
int jtag_open(struct mrf_dev *ev_dev);
int jtag_release(struct mrf_dev *ev_dev);
void jtag_pulseTCK(volatile struct Pci9030LocalConf *pLC, int n);
void jtag_TLR(volatile struct Pci9030LocalConf *pLC);
void jtag_TLRtoRTI(volatile struct Pci9030LocalConf *pLC);
void jtag_TLRtoSIR(volatile struct Pci9030LocalConf *pLC);
void jtag_TLRtoSDR(volatile struct Pci9030LocalConf *pLC);
void jtag_EX1toSDR(volatile struct Pci9030LocalConf *pLC);
void jtag_EX1toRTI(volatile struct Pci9030LocalConf *pLC);
int jtag_SDI(volatile struct Pci9030LocalConf *pLC, int n, int tdi,
	     int *tdo, int last);
void jtag_SCFD(volatile struct Pci9030LocalConf *pLC, char cb, int last);
int jtag_validate(struct mrf_dev *ev_dev);
int jtag_SIR(volatile struct Pci9030LocalConf *pLC, int hir, int irlen,
	     int tdi, int tir, int *tdo, int tlr);
int jtag_SDR(volatile struct Pci9030LocalConf *pLC, int hdr, int drlen,
	     char *tdi, int tdr, char *tdo, int tlr);
/* Read FPGA status register (see Configuration Details in Xilinx
   Virtex-II Pro Users Guide, ug012.pdf) */

int jtag_read_fpga_status(struct mrf_dev *ev_dev);
int fpga_sprint(struct mrf_dev *ev_dev, char *buf, int size);

/* Issue JTAG command XSC_CONFIG which forces PROG_B pin on Platform
   Flash low and initiates a new configuration from Flash */ 

int jtag_XCF_conf(struct mrf_dev *ev_dev);

/* Read Platform Flash contents.
   addr and size have to be multiples of 1024 */

int jtag_XCF_readstart(struct mrf_dev *ev_dev);
int jtag_XCF_read(struct mrf_dev *ev_dev,
		  char *data, unsigned int addr, unsigned int size);
int jtag_XCF_readend(struct mrf_dev *ev_dev);

/* Program data to Platform Flash 
   These functions erase the flash prior to programming */

int jtag_XCF_progstart(struct mrf_dev *ev_dev);
int jtag_XCF_write(struct mrf_dev *ev_dev, char *data,
		   int addr, int size);
int jtag_XCF_progend(struct mrf_dev *ev_dev);

/* Configure FPGA from .bit file.
   Startup clock has to be set to JTAG in bitgen. */

int jtag_init_load_fpga(struct mrf_dev *ev_dev);
#define jtag_load_fpga_byte(x, y) jtag_SCFD(x, y, 0)
int jtag_end_load_fpga(struct mrf_dev *ev_dev);
#endif	/* This stuff won't compile and link w/o other MRF src we don't have */

#endif	/*	PCI_MRFEV_H	*/
