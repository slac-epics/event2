/*
  mrfevr.c -- Micro-Research Event Receiver
              Linux 2.6 driver

	      This driver handles multiple instances

  Author: Jukka Pietarinen (MRF)
  Date:   29.11.2006

*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#include <asm/page.h>
#include <asm/uaccess.h>

#include "pci_mrfev.h"

MODULE_LICENSE("GPL");

#define DEVICE_NAME           "evr_device"

extern struct mrf_dev mrf_devices[MAX_MRF_DEVICES];
static struct timer_list mrf_timer;

/*
  File operations for Event Boards
*/
static struct file_operations evr_fops = {
  .owner = THIS_MODULE,
  .read = ev_read,
  .write = ev_write,
#ifdef HAVE_UNLOCKED_IOCTL
  .unlocked_ioctl = ev_unlocked_ioctl,
#else
  .ioctl = ev_ioctl,
#ifdef CONFIG_COMPAT
  .compat_ioctl = ev_compat_ioctl,
#endif
#endif
  .open = ev_open,
  .release = ev_release,
  .fasync = ev_fasync,
  .mmap = ev_remap_mmap,
};

/* The first prototypes shipped had both the vendor ID and subsystem
   vendor ID set to MRF's vendor ID. Now, boards are shipped with
   vendor ID and device ID set to PLX's 9030 default values and
   subsystem field set to MRF's codes. This driver supports both
   conventions. */

static struct pci_device_id evr_ids[] = {
  { PCI_DEVICE(PCI_VENDOR_ID_XILINX_PCIE,PCI_DEVICE_ID_XILINX_PCIE) }, /* SLAC EVR */
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PMCEVR200, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PMCEVR230, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PXIEVR220, },
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_MRF,
    .subdevice = PCI_DEVICE_ID_MRF_PXIEVR230, },
	/* This is how our PMC-EVR230 devices are programmed from the factory */
  { .vendor = PCI_VENDOR_ID_PLX,
    .device = PCI_DEVICE_ID_PLX_9030,
    .subvendor = PCI_VENDOR_ID_PLX,
    .subdevice = PCI_DEVICE_ID_PLX_9030, },
  { PCI_DEVICE(PCI_VENDOR_ID_MRF, PCI_DEVICE_ID_MRF_PMCEVR200), },
  { PCI_DEVICE(PCI_VENDOR_ID_MRF, PCI_DEVICE_ID_MRF_PXIEVR220), },
#ifdef DETECT_UNCONFIGURED_BOARD
  { .vendor = MODULE_VENDOR_ID_NOCONF,
    .device = MODULE_DEVICE_ID_NOCONF,
    .subvendor = MODULE_SUBVENDOR_ID_NOCONF,
    .subdevice = MODULE_SUBDEVICE_ID_NOCONF, },
#endif
  { 0, }};
MODULE_DEVICE_TABLE(pci, evr_ids);

static int pci_evr_probe(struct pci_dev *pcidev, const struct pci_device_id *dev_id)
{
  unsigned long local_conf_start;
  unsigned long local_conf_end;
  unsigned long evr_base_start;
  unsigned long evr_base_end;
  int i, res;
  dev_t chrdev = 0;
  struct mrf_dev *ev_device;
  struct pci_device_id *id = (struct pci_device_id *) dev_id;

  /* We keep device instance number in id->driver_data */
  id->driver_data = -1;

  /* Find empty mrf_dev structure */
  for (i = 0; i < MAX_MRF_DEVICES; i++)
    if (mrf_devices[i].mrEv == 0)
      {
	id->driver_data = i;
	break;
      }

  if (id->driver_data < 0)
    {
      printk(KERN_WARNING DEVICE_NAME ": too many devices.\n");
      return -EMFILE;
    }

  ev_device = &mrf_devices[id->driver_data];
  if (PDI_IS_SLAC(id)) {
    ev_device->device_first = DEVICE_FIRST_SLAC;
  } else {
    ev_device->device_first = DEVICE_FIRST_MRF;
  }
  ev_device->qmem = kmalloc(sizeof(struct EvrQueues) + PAGE_SIZE, GFP_KERNEL);
  if (!ev_device->qmem) {
    printk(KERN_WARNING DEVICE_NAME ": no memory available.\n");
    return -ENOMEM;
  }
  memset(ev_device->qmem, 0, sizeof(struct EvrQueues) + PAGE_SIZE);
  ev_device->evrq = (void *)((long)(ev_device->qmem + PAGE_SIZE - 1) & PAGE_MASK);
  ev_device->vmas = 0;

  /* Allocate device numbers for character device. */
  res = alloc_chrdev_region(&chrdev, 0, DEVICE_MINOR_NUMBERS, DEVICE_NAME);
  if (res < 0)
    {
      printk(KERN_WARNING DEVICE_NAME ": cannot register char device.\n");
      return res;
    }

  /* Initialize device structure */
  ev_device->major = MAJOR(chrdev);
  cdev_init(&ev_device->cdev, &evr_fops);
  ev_device->cdev.owner = THIS_MODULE;
  ev_device->cdev.ops = &evr_fops;
  ev_device->access_mode = 0;
  ev_device->access_device = -1;
  ev_device->eeprom_data = NULL;
  ev_device->eeprom_data_size = 0;
  ev_device->xcf_data = NULL;
  ev_device->xcf_data_pos = -1;
  ev_device->fpga_conf_data = NULL;
  ev_device->fpga_conf_size = 0;
#ifndef init_MUTEX
  sema_init(&ev_device->sem, 1);
#else
  init_MUTEX(&ev_device->sem);
#endif
  res = cdev_add(&ev_device->cdev, chrdev, DEVICE_MINOR_NUMBERS);
  ev_device->pLC = 0;
  ev_device->lenLC = 0;
  ev_device->pEv = 0;
  ev_device->lenEv = 0;
  ev_device->jtag_refcount = 0;
  ev_device->fpgaid = 0;
  ev_device->xcfid = 0;
  ev_device->fpgairlen = 0;
  ev_device->xcfirlen = 0;

  if (res)
    {
      printk(KERN_WARNING DEVICE_NAME ": error adding " DEVICE_NAME "0-%d\n.",
	     DEVICE_MINOR_NUMBERS);
    }

  /* Here we enable device before we can do any accesses to device. */
  if (pci_enable_device(pcidev)) {
    printk(KERN_WARNING "%s: Could not enable device \n", DEVICE_NAME);
    return -1;
  } 

  if (SLAC_EVR(ev_device)) {
    evr_base_start = pci_resource_start(pcidev, 0);
    evr_base_end = pci_resource_end(pcidev, 0);
    ev_device->mrEv = evr_base_start;
    ev_device->lenEv = evr_base_end - evr_base_start + 1;
    if (request_mem_region(ev_device->mrEv, ev_device->lenEv,
			   DEVICE_NAME) != NULL)
      {
        ev_device->pEv = ioremap_nocache(evr_base_start,
					 ev_device->lenEv);
      }
  } else {
    /* Get the local configuration register memory mapped base address
       (BAR0), end address */
    local_conf_start = pci_resource_start(pcidev, 0);
    local_conf_end = pci_resource_end(pcidev, 0);
    ev_device->mrLC = local_conf_start;
    ev_device->lenLC = local_conf_end - local_conf_start + 1;
    if (request_mem_region(ev_device->mrLC, ev_device->lenLC,
		 	   DEVICE_NAME) != NULL)
      {
        ev_device->pLC = ioremap_nocache(local_conf_start,
				         ev_device->lenLC);
      }

    /* Get the Event Receiver registers memory mapped base address
       (BAR2), end address and flags*/
    evr_base_start = pci_resource_start(pcidev, 2);
    evr_base_end = pci_resource_end(pcidev, 2);
    ev_device->mrEv = evr_base_start;
    ev_device->lenEv = evr_base_end - evr_base_start + 1;
    if (request_mem_region(ev_device->mrEv, ev_device->lenEv,
			   DEVICE_NAME) != NULL)
      {
        ev_device->pEv = ioremap_nocache(evr_base_start,
					 ev_device->lenEv);
      }

    /* Check the firmware version, and abort if we can't deal with it! */
    if (((struct MrfErRegs *)ev_device->pEv)->FPGAVersion == be32_to_cpu(BAD_EVR_FIRMWARE_REV)) {
      printk(KERN_ALERT "Unsupported MRF EVR with firmware version 0x%08x, ignoring!\n",
             BAD_EVR_FIRMWARE_REV);
      kfree(ev_device->qmem);
      /* Release memory regions for BAR0 and BAR2 */
      iounmap(ev_device->pEv);
      release_mem_region(ev_device->mrEv, ev_device->lenEv);
      iounmap(ev_device->pLC);
      release_mem_region(ev_device->mrLC, ev_device->lenLC);
      pci_disable_device(pcidev);
      cdev_del(&ev_device->cdev);
      unregister_chrdev_region(MKDEV(ev_device->major, 0), DEVICE_MINOR_NUMBERS);
      ev_device->mrLC = 0;
      return -EIO;
    }
  }

  /* Check the interrupt line */
  ev_device->irq = pcidev->irq;

  for (i = 0; i < MAX_EVR_OPENS; i++) {
      ev_device->shared[i].parent = NULL;
      ev_device->shared[i].idx = i;
      init_waitqueue_head(&ev_device->shared[i].waitq);
      spin_lock_init(&ev_device->shared[i].lock);
  }

  for (i = 0; i < EVR_MAX_PULSES; i++)
      ev_device->pulse[i] = -1;

  ErInitializeRams(ev_device->pEv);
  ((struct MrfErRegs *)ev_device->pEv)->IrqEnable = 0;

  return 0;
}

static void pci_evr_remove(struct pci_dev *pcidev)
{
  int i;
  struct mrf_dev *ev_device = NULL;

  for (i = 0; i < MAX_MRF_DEVICES; i++)
    if ((SLAC_EVR(&mrf_devices[i]) ? mrf_devices[i].mrEv : mrf_devices[i].mrLC)
        == pci_resource_start(pcidev, 0))
      {
	ev_device = &mrf_devices[i];
	break;
      }

  if (ev_device == NULL)
    {
      printk(KERN_ALERT "Trying to remove uninstalled driver for bus\n");
    }
  else
    {
      kfree(ev_device->qmem);
      /* Release memory regions for BAR0 and BAR2 */
      iounmap(ev_device->pEv);
      release_mem_region(ev_device->mrEv, ev_device->lenEv);
      if (MRF_EVR(ev_device)) {
        iounmap(ev_device->pLC);
        release_mem_region(ev_device->mrLC, ev_device->lenLC);
      }
      pci_disable_device(pcidev);
      cdev_del(&ev_device->cdev);
      unregister_chrdev_region(MKDEV(ev_device->major, 0), DEVICE_MINOR_NUMBERS);
      ev_device->mrLC = 0;
    }
}

static struct pci_driver evr_driver = {
  .name = "pci_mrfevr",
  .id_table = evr_ids,
  .probe = pci_evr_probe,
  .remove = pci_evr_remove,
};

int ev_assign_irq(struct mrf_dev *ev_device)
{
  int result;

  result = request_irq(ev_device->irq,
		       &ev_interrupt,
#if LINUX_VERSION_CODE > (0x020619)
                       IRQF_SHARED,
#else
                       SA_SHIRQ,
#endif
		       DEVICE_NAME,
		       (void *) ev_device);
  if (result)
    printk(KERN_INFO DEVICE_NAME ": cannot get interrupt %d\n",
	   ev_device->irq);
  return result;
}

void mrf_callback(unsigned long data)
{
    int i;

    mod_timer(&mrf_timer, jiffies + msecs_to_jiffies(1000));
    for (i = 0; i < MAX_MRF_DEVICES; i++) {
        if (mrf_devices[i].mrEv != 0 && mrf_devices[i].access_device == DEVICE_SHEV) {
            volatile struct MrfErRegs *pEr = (struct MrfErRegs *)mrf_devices[i].pEv;
            pEr->IrqFlag = be32_to_cpu(1 << C_EVR_IRQFLAG_VIOLATION);
        }
    }
}

static int __init pci_evr_init(void)
{
  /* Allocate and clear memory for all devices. */
  memset(mrf_devices, 0, sizeof(struct mrf_dev)*MAX_MRF_DEVICES);

  setup_timer(&mrf_timer, mrf_callback, 0);
  mod_timer(&mrf_timer, jiffies + msecs_to_jiffies(1000));

  printk(KERN_ALERT "Event Receiver PCI driver 2.1 init.\n");
  return pci_register_driver(&evr_driver);    
}

static void __exit pci_evr_exit(void)
{
  printk(KERN_ALERT "Event Receiver PCI driver 2.1 exiting.\n");

  del_timer(&mrf_timer);

  pci_unregister_driver(&evr_driver);
}  

module_init(pci_evr_init);
module_exit(pci_evr_exit);
