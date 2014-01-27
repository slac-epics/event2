/*
  local_conf.c -- Micro-Research Event Receiver Linux 2.6 driver
                  PCI9030 Local Configuration functions

  Author: Jukka Pietarinen (MRF)
  Date:   29.11.2006

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>     // Needed for the macros
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>

#include "pci_mrfev.h"

void evr_dumpLC(struct Pci9030LocalConf *pLC)
{
  printk(KERN_INFO "Local Address Space 0 Range 0x%08x\n", (pLC->LAS0RR));
  printk(KERN_INFO "Local Address Space 1 Range 0x%08x\n", (pLC->LAS1RR));
  printk(KERN_INFO "Local Address Space 2 Range 0x%08x\n", (pLC->LAS2RR));
  printk(KERN_INFO "Local Address Space 3 Range 0x%08x\n", (pLC->LAS3RR));
  printk(KERN_INFO "Expansion ROM Range 0x%08x\n", (pLC->EROMRR));
  printk(KERN_INFO "Local Address Space 0 Local Base Address 0x%08x\n",
	 (pLC->LAS0BA));
  printk(KERN_INFO "Local Address Space 1 Local Base Address 0x%08x\n",
	 (pLC->LAS1BA));
  printk(KERN_INFO "Local Address Space 2 Local Base Address 0x%08x\n",
	 (pLC->LAS2BA));
  printk(KERN_INFO "Local Address Space 3 Local Base Address 0x%08x\n",
	 (pLC->LAS3BA));
  printk(KERN_INFO "Expansion ROM Local Base Address 0x%08x\n", (pLC->EROMBA));
  printk(KERN_INFO "Local Address Space 0 Bus Region Descriptor 0x%08x\n",
	 (pLC->LAS0BRD));
  printk(KERN_INFO "Local Address Space 1 Bus Region Descriptor 0x%08x\n",
	 (pLC->LAS1BRD));
  printk(KERN_INFO "Local Address Space 2 Bus Region Descriptor 0x%08x\n",
	 (pLC->LAS2BRD));
  printk(KERN_INFO "Local Address Space 3 Bus Region Descriptor 0x%08x\n",
	 (pLC->LAS3BRD));
  printk(KERN_INFO "Expansion ROM Bus Region Descriptor 0x%08x\n",
	 (pLC->EROMBRD));
  printk(KERN_INFO "Chip Select 0 Base Address 0x%08x\n", (pLC->CS0BASE));
  printk(KERN_INFO "Chip Select 1 Base Address 0x%08x\n", (pLC->CS1BASE));
  printk(KERN_INFO "Chip Select 2 Base Address 0x%08x\n", (pLC->CS2BASE));
  printk(KERN_INFO "Chip Select 3 Base Address 0x%08x\n", (pLC->CS3BASE));
  printk(KERN_INFO "Interrupt Control/Status 0x%04x\n", (pLC->INTCSR) >> 16);
  printk(KERN_INFO "Serial EEPROM Write-Protected Address Boundary 0x%04x\n", 
	 (pLC->PROT_AREA) >> 16);
  printk(KERN_INFO "PCI Target Response, Serial EEPROM, and Initialization"
	 "Control 0x%08x\n", (pLC->CNTRL));
  printk(KERN_INFO "General Purpose I/O Control 0x%08x\n", (pLC->GPIOC));
  printk(KERN_INFO "Hidden 1 Power Management Data Select 0x%08x\n",
	 (pLC->PMDATASEL));
  printk(KERN_INFO "Hidden 2 Power Management Data Scale 0x%08x\n",
	 (pLC->PMDATASCALE));
}
