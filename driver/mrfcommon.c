/*
  mrfcommon.c -- Micro-Research Event Generator/Event Receiver
                 Linux 2.6 driver common functions

		 This driver handles multiple boards.

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
#include <linux/version.h>

#include <asm/page.h>
#include <asm/uaccess.h>

#include "evrmemmap.h"
#include "pci_mrfev.h"
MODULE_LICENSE("GPL");

#ifndef DEVICE_NAME
#define DEVICE_NAME "pci_mrf"
#endif

struct mrf_dev mrf_devices[MAX_MRF_DEVICES];

int ev_assign_irq(struct mrf_dev *ev_device);
int ev_plx_irq_enable(struct mrf_dev *ev_device);
int ev_plx_irq_disable(struct mrf_dev *ev_device);
void set_irq_mask(struct mrf_dev *ev_device);
void set_event_table(struct mrf_dev *ev_device);

/*
  Open and close
*/

int ev_open(struct inode *inode, struct file *filp)
{
  int result, i;
  struct mrf_dev *ev_device;
  int major, minor;

  ev_device = container_of(inode->i_cdev, struct mrf_dev, cdev);

  major = MAJOR(inode->i_rdev);
  minor = MINOR(inode->i_rdev);
  
  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  /* Check that none of the first three devices is open.
     We allow several simultaneous connections on the 4th and 5th minor devices. */
  if (ev_device->access_device >= 0 && 
      (ev_device->access_device < DEVICE_EV || ev_device->access_device != minor))
    {
      printk(KERN_WARNING DEVICE_NAME ": open device major %d minor %d"
	     "already open. Only one open minor device allowed.\n",
	     major, ev_device->access_device);
      result = -EPERM;
      goto out;
    }

  if (minor >= ev_device->device_first && minor <= DEVICE_LAST)
    {
      ev_device->access_mode = filp->f_flags & O_ACCMODE;
      ev_device->access_device = minor;

      switch (ev_device->access_device)
	{
	case DEVICE_EEPROM:
          ev_device->shared[0].parent = ev_device;
          filp->private_data = &ev_device->shared[0]; 
	  /* Allocate and clear memory for EEPROM data */
	  ev_device->eeprom_data = kmalloc(EEPROM_DATA_ALLOC_SIZE,
					   GFP_KERNEL);
	  memset(ev_device->eeprom_data, 0, EEPROM_DATA_ALLOC_SIZE);

	  /* If the EEPROM device was opened in read-only mode we need
             to read in the data from the EEPROM. */
	  if (ev_device->access_mode == O_RDONLY)
	    {
	      result = eeprom_sprint(ev_device->pLC, ev_device->eeprom_data,
				     EEPROM_DATA_ALLOC_SIZE);
	      if (result < 0)
		{
		  /* If there was an error, clean up */
		  kfree(ev_device->eeprom_data);
		  ev_device->eeprom_data = NULL;
		  ev_device->access_mode = 0;
		  ev_device->access_device = -1;
		  goto out;
		}

	      ev_device->eeprom_data_size = result;
	    }
	  break;
	case DEVICE_XCF:
          ev_device->shared[0].parent = ev_device;
          filp->private_data = &ev_device->shared[0]; 
	  ev_device->xcf_data = kmalloc(XCF_DATA_ALLOC_SIZE,
					GFP_KERNEL);
	  memset(ev_device->xcf_data, 0xffffffff, XCF_DATA_ALLOC_SIZE);
	  /* If the XCF device was opened in write-only mode we
	     erase the Platform Flash and start the programming
	     sequence */
	  if (ev_device->access_mode == O_RDONLY)
	    {
	      ev_device->xcf_data_pos = -1;
	      result = jtag_XCF_readstart(ev_device);
	      if (result < 0)
		{
		  /* If there was an error, clean up */
		  kfree(ev_device->xcf_data);
		  ev_device->xcf_data = NULL;
		  ev_device->access_mode = 0;
		  ev_device->xcf_data_pos = 0;
		  ev_device->access_device = -1;
		  goto out;
		}
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      ev_device->xcf_data_pos = 0;
	      result = jtag_XCF_progstart(ev_device);
	      if (result < 0)
		{
		  /* If there was an error, clean up */
		  kfree(ev_device->xcf_data);
		  ev_device->xcf_data = NULL;
		  ev_device->access_mode = 0;
		  ev_device->access_device = -1;
		  goto out;
		}
	    }
	  break;
	case DEVICE_FPGA:
          ev_device->shared[0].parent = ev_device;
          filp->private_data = &ev_device->shared[0]; 
	  /* Allocate and clear memory for FPGA data */
	  ev_device->fpga_conf_data = kmalloc(FPGA_DATA_ALLOC_SIZE,
					      GFP_KERNEL);
	  memset(ev_device->fpga_conf_data, 0, FPGA_DATA_ALLOC_SIZE);
	  /* If the FPGA device was opened in read-only mode we read
	     in the FPGA status - reading back configuration data is
	     not supported */
	  result = -1;
	  if (ev_device->access_mode == O_RDONLY)
	    {
	      result = fpga_sprint(ev_device, ev_device->fpga_conf_data,
				   FPGA_DATA_ALLOC_SIZE);
	      ev_device->fpga_conf_size = result;
	    }
	  if (ev_device->access_mode == O_WRONLY)
	    {
	      result = jtag_init_load_fpga(ev_device);
	      ev_device->fpga_conf_size = 0;	      
	    }	  
	  if (result < 0)
	    {
	      /* If there was an error, clean up */
	      kfree(ev_device->fpga_conf_data);
	      ev_device->fpga_conf_data = NULL;
	      ev_device->access_mode = 0;
	      ev_device->access_device = -1;
	      goto out;
	    }
	  break;
	case DEVICE_EV:
	case DEVICE_SHEV:
          for (i = 0; i < MAX_EVR_OPENS; i++)
            if (!ev_device->shared[i].parent)
              break;
          if (i == MAX_EVR_OPENS) {
            result = -ENXIO;
            goto out;
          }
          ev_device->shared[i].parent = ev_device;
          filp->private_data = &ev_device->shared[i];

	  /* Assign interrupt handler when first instance opened. */
	  if (!(ev_device->refcount_ev))
	    {
	      result = ev_assign_irq(ev_device);
	      ev_plx_irq_enable(ev_device);
              if (ev_device->access_device == DEVICE_SHEV) {
                  volatile struct MrfErRegs *pEr = ev_device->pEv;
                  int form = (be32_to_cpu(pEr->FPGAVersion)>>24) & 0x0F;
                  int j;
                  /*
                   * In shared mode, we need to initialize the device!
                   */
                  pEr->Control |= be32_to_cpu(1 << C_EVR_CTRL_MASTER_ENABLE);
                  pEr->EvCntPresc = be32_to_cpu(1);
                  pEr->FracDiv = be32_to_cpu(CLOCK_119000_MHZ);
                  for (j = 0; j < 12; j++) {
                      if(pLinuxErCard->ErCard.FormFactor == EVR_FORM_PMC)
                          pEr->FPOutMap[j] = be16_to_cpu(j);
                      else if(pLinuxErCard->ErCard.FormFactor == EVR_FORM_CPCI ||
                              pLinuxErCard->ErCard.FormFactor == EVR_FORM_SLAC)
                          pEr->UnivOutMap[j] = be16_to_cpu(j);
                  }
              }
	    }
	  /* Inrease device reference count. */
	  ev_device->refcount_ev++;
	  break;
	default:
	  result = -ENXIO;
	  goto out;
	}
    }
  else
    {
      result = -ENXIO;
      goto out;
    }

  result = 0;

 out:
  up(&ev_device->sem);
  return result;
}

int ev_release(struct inode *inode, struct file *filp)
{
  int result = 0;
  struct shared_mrf *shared = (struct shared_mrf *) filp->private_data;
  struct mrf_dev *ev_device = shared->parent;

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
    case DEVICE_EEPROM:
      if (ev_device->access_mode == O_WRONLY)
	{
	  result = eeprom_sscan(ev_device->pLC, ev_device->eeprom_data);
	}
      /* Release memory allocated for EEPROM data */
      kfree(ev_device->eeprom_data);
      ev_device->eeprom_data = NULL;
      ev_device->access_mode = 0;
      ev_device->access_device = -1;
      break;
    case DEVICE_XCF:
      if (ev_device->access_mode == O_RDONLY)
	{
	  result = jtag_XCF_readend(ev_device);
	}
      if (ev_device->access_mode == O_WRONLY)
	{
	  /* Flush buffer */
	  jtag_XCF_write(ev_device, ev_device->xcf_data,
			 ev_device->xcf_data_pos, XCF_DATA_ALLOC_SIZE);
	  result = jtag_XCF_progend(ev_device);
	  
	  /* Issue load FPGA from flash (cycle nPROG) */
	  jtag_XCF_conf(ev_device);
	}
      /* Release memory allocated for XCF data */
      kfree(ev_device->xcf_data);
      ev_device->xcf_data = NULL;
      ev_device->access_mode = 0;
      ev_device->access_device = -1;
      break;	  
    case DEVICE_FPGA:
      if (ev_device->access_mode == O_WRONLY)
	{
	  result = jtag_end_load_fpga(ev_device);
	}
      /* Release memory allocated for FPGA data */
      kfree(ev_device->fpga_conf_data);
      ev_device->fpga_conf_data = NULL;
      ev_device->access_mode = 0;
      ev_device->access_device = -1;
      break;	  
    case DEVICE_SHEV:
      shared->parent = NULL;
      set_irq_mask(ev_device);
      set_event_table(ev_device);
      shared->parent = ev_device;
    case DEVICE_EV:
      /* Remove from list of asynchronously notified filps */ 
      ev_fasync(-1, filp, 0);
      if (ev_device->refcount_ev)
	ev_device->refcount_ev--;
      if (!ev_device->refcount_ev)
	{
	  /* Release interrupt handler when device is closed for the last
	     time. */
	  ev_plx_irq_disable(ev_device);
	  free_irq(ev_device->irq, (void *) ev_device);
          if (ev_device->access_device == DEVICE_SHEV) {
              volatile struct MrfErRegs *pEr = ev_device->pEv;
              /*
               * In shared mode, disable the device when the last close happens.
               */
              pEr->Control &= be32_to_cpu(~(1 << C_EVR_CTRL_MASTER_ENABLE));
          }
	  ev_device->access_mode = 0;
	  ev_device->access_device = -1;
	}
      break;
    default:
      printk(KERN_WARNING DEVICE_NAME ": Trying to release unknown device major %d minor %d\n",
	     MAJOR(inode->i_rdev), MINOR(inode->i_rdev));
      result = -ENXIO;
    }

  shared->parent = NULL;

  up(&ev_device->sem);
  return result;
}

ssize_t ev_write(struct file *filp, const char __user *buf, size_t count,
		 loff_t *f_pos)
{
  ssize_t retval = -ENOMEM;
  struct shared_mrf *shared = ((struct shared_mrf *) filp->private_data);
  struct mrf_dev *ev_device = shared->parent;

#ifdef DEBUG
  printk(KERN_INFO DEVICE_NAME ": write %d bytes.\n", count);
#endif

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
      case DEVICE_EEPROM:
	{
	  if (*f_pos > EEPROM_DATA_ALLOC_SIZE - 1)
	    goto write_out;

	  if (*f_pos + count > EEPROM_DATA_ALLOC_SIZE - 1)
	    count = EEPROM_DATA_ALLOC_SIZE - 1 - *f_pos;

	  if (copy_from_user(&ev_device->eeprom_data[*f_pos], buf, count))
	    {
	      retval = -EFAULT;
	      goto write_out;
	    }

	  *f_pos += count;
	  ev_device->eeprom_data_size = *f_pos;

	  retval = count;
	  break;
	}
      case DEVICE_XCF:
	{
	  if (*f_pos < ev_device->xcf_data_pos)
	    goto write_out;

	  if (*f_pos + count > ev_device->xcf_data_pos +
	      XCF_DATA_ALLOC_SIZE)
	    count = ev_device->xcf_data_pos + XCF_DATA_ALLOC_SIZE - *f_pos;

	  if (copy_from_user(&ev_device->xcf_data[*f_pos & (XCF_DATA_ALLOC_SIZE - 1)], buf, count))
	    {
	      retval = -EFAULT;
	      goto write_out;
	    }

	  *f_pos += count;
	  if (*f_pos - ev_device->xcf_data_pos == XCF_DATA_ALLOC_SIZE)
	    {
	      jtag_XCF_write(ev_device, ev_device->xcf_data, 
			     ev_device->xcf_data_pos, XCF_DATA_ALLOC_SIZE);
	      memset(ev_device->xcf_data, 0xffffffff, XCF_DATA_ALLOC_SIZE);
	      ev_device->xcf_data_pos = *f_pos;
	    }

	  retval = count;
	  break;
	}
      case DEVICE_FPGA:
	{
	  int i;

	  if (count > FPGA_DATA_ALLOC_SIZE - 1)
	    count = FPGA_DATA_ALLOC_SIZE - 1;

	  if (copy_from_user(ev_device->fpga_conf_data, buf, count))
	    {
	      retval = -EFAULT;
	      goto write_out;
	    }

	  *f_pos += count;
	  ev_device->fpga_conf_size = *f_pos;

	  for (i = 0; i < count; i++)
	    jtag_load_fpga_byte(ev_device->pLC, ev_device->fpga_conf_data[i]);

	  retval = count;
	  break;
	}
    }

 write_out:
  up(&ev_device->sem);
  return retval;
}

#ifdef CONFIG_COMPAT
long ev_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return ev_ioctl(NULL, file, cmd, arg);
}
#endif

/*
  Virtual Memory Area operations
*/

void ev_vma_open(struct vm_area_struct *vma)
{
  struct mrf_dev *ev_device = (struct mrf_dev *)vma->vm_private_data;

  ev_device->vmas++;
}

void ev_vma_close(struct vm_area_struct *vma)
{
  struct mrf_dev *ev_device = (struct mrf_dev *)vma->vm_private_data;

  ev_device->vmas--;
}

struct page *ev_vma_nopage(struct vm_area_struct *vma, unsigned long address, int *type)
{
    return NOPAGE_SIGBUS;
}

static struct vm_operations_struct ev_remap_vm_ops = {
  .open = ev_vma_open,
  .close = ev_vma_close,
  .nopage = ev_vma_nopage,
};

/*
 * Now, the map has two parts:
 *    0 to sizeof(struct MrfErRegs) maps to the actual registers.
 *    sizeof(struct MrfErRegs) to sizeof(struct MrfErRegs) + sizeof(struct EvrQueues) maps to memory.
 */
int ev_remap_mmap(struct file *filp, struct vm_area_struct *vma)
{
  struct mrf_dev *ev_device = ((struct shared_mrf *) filp->private_data)->parent;
  unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
  unsigned long vsize = vma->vm_end - vma->vm_start;
  unsigned long start = vma->vm_start;
  int result;

  /* Memory mapping supported only on minor device DEVICE_EV and DEVICE_SHEV. */
  if (ev_device->access_device < DEVICE_EV)
    {
      printk(KERN_NOTICE DEVICE_NAME ": mmap not allowed for access_device %d\n",
	     ev_device->access_device);
      return -EPERM;
    }

  if (ev_device->access_device == DEVICE_EV) {
    if (vsize > ev_device->lenEv)
      {
        printk(KERN_NOTICE DEVICE_NAME ": mmap vsize %08x, ev_device->lenEv %08x\n",
	       (unsigned int) vsize, (unsigned int) ev_device->lenEv);
        return -EINVAL;
      }
  } else {
    if (vsize > EVR_MEM_WINDOW)
      {
        printk(KERN_NOTICE DEVICE_NAME ": mmap vsize %08x, ev_device->lenEv %08x regsize %08lx qsize %08lx\n",
	       (unsigned int) vsize, (unsigned int) ev_device->lenEv,
               sizeof(struct MrfErRegs), sizeof(struct EvrQueues));
        return -EINVAL;
      }
  }

  if (offset < sizeof(struct MrfErRegs)) { /* At least some register space! */
      unsigned long psize = sizeof(struct MrfErRegs) - offset;    /* Size of region from offset to end */
      unsigned long physical = ((unsigned long) ev_device->mrEv) + offset;
      if (vsize < psize) /* Maybe we don't want to map the entire register space? */
          psize = vsize;
#if LINUX_VERSION_CODE > 0x020609
      result = io_remap_pfn_range(vma, start, physical >> PAGE_SHIFT,
                                  psize, vma->vm_page_prot);
#else
      result = io_remap_page_range(vma, start, physical, psize, 
                                   vma->vm_page_prot);
#endif  
      if (result)
          return -EAGAIN;
      offset += psize;    /* Remove the part of the memory space that we've mapped in here. */
      vsize  -= psize;
      start  += psize;
  }

  if (vsize > 0) {   /* If we still want more, it has to be from the kernel memory! */
      offset -= sizeof(struct MrfErRegs);
      /* Map this read only!! */
      result = remap_pfn_range(vma, start,
                               __pa(ev_device->evrq + offset) >> PAGE_SHIFT,
                               vsize, PAGE_READONLY);
      if (result)
          return -EAGAIN;
  }
  
  vma->vm_ops = &ev_remap_vm_ops;
  vma->vm_private_data = ev_device;
  ev_vma_open(vma);
  
  return 0;  
}

int ev_fasync(int fd, struct file *filp, int mode)
{
  struct mrf_dev *ev_device = ((struct shared_mrf *) filp->private_data)->parent;

  return fasync_helper(fd, filp, mode, &ev_device->async_queue);
}

int ev_plx_irq_enable(struct mrf_dev *ev_device)
{
  if (ev_device->slac) {
    volatile struct MrfErRegs *pEv = ev_device->pEv;
    pEv->pcieInt = 0x01000000;
    barrier();
    return pEv->pcieInt;
  } else {
    volatile struct Pci9030LocalConf *pLC = ev_device->pLC;

    pLC->INTCSR = __constant_cpu_to_le16(PLX9030_INTCSR_LINTI1_ENA |
				         PLX9030_INTCSR_LINTI1_POL |
				         PLX9030_INTCSR_PCI_IRQENA);
    barrier();
    return le16_to_cpu(pLC->INTCSR);
  }
}

int ev_plx_irq_disable(struct mrf_dev *ev_device)
{
  if (ev_device->slac) {
    volatile struct MrfErRegs *pEv = ev_device->pEv;
    pEv->pcieInt = 0;
    barrier();
    return pEv->pcieInt;
  } else {
    volatile struct Pci9030LocalConf *pLC = ev_device->pLC;

    pLC->INTCSR = __constant_cpu_to_le16(PLX9030_INTCSR_LINTI1_ENA |
				         PLX9030_INTCSR_LINTI1_POL);
    barrier();
    return le16_to_cpu(pLC->INTCSR);
  }
}

/**********************************************************************\
*                                                                      *
* OK, here are the three functions that actually do most of the        *
* sharing work. ev_ioctl supports the IRQMASK and EVTTAB write         *
* functions, and ev_interrupt actually queues up the work for the      *
* ev_read routine.                                                     *
*                                                                      *
\**********************************************************************/

int ev_ioctl(struct inode *inode, struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
  struct shared_mrf *shared = ((struct shared_mrf *) filp->private_data);
  struct mrf_dev *ev_device = shared->parent;
  int ret = 0;

  /* Check that cmd is valid */
  if (_IOC_TYPE(cmd) != EV_IOC_MAGIC)
    return -ENOTTY;
  if (_IOC_NR(cmd) > EV_IOC_MAX)
    return -ENOTTY;

  switch (cmd)
    {
    case EV_IOCRESET:
      ret = 0;
      break;

    case EV_IOCIRQEN:
      ev_plx_irq_enable(ev_device);
      ret = 0;
      break;

    case EV_IOCIRQDIS:
      ret = 0;
      break;

    case EV_IOCIRQMASK:
      if (ev_device->access_device == DEVICE_SHEV) {
        if (copy_from_user(&shared->irqmask, (u32 *)arg, sizeof(shared->irqmask))) {
          return -EACCES;
        }
        set_irq_mask(ev_device);
        ret = 0;
      } else
        ret = -ENOTTY;
      break;

    case EV_IOCEVTTAB:
      if (ev_device->access_device == DEVICE_SHEV) {
        if (copy_from_user(&shared->evttab, (u16 *)arg, sizeof(shared->evttab))) {
          return -EACCES;
        }
        set_event_table(ev_device);
        ret = 0;
      } else
        ret = -ENOTTY;
      break;

    default:
      return -ENOTTY;
    }

  return ret;
}

irqreturn_t ev_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mrf_dev *ev_device = (struct mrf_dev *) dev_id;
    volatile struct MrfErRegs *pEv = ev_device->pEv;

    /* We use shared interrupts: return immediately if irq is not from our
       device. */
    if (ev_device->slac) {
        if (!(pEv->pcieInt & 0x02000000))
            return IRQ_NONE;
    } else {
        volatile struct Pci9030LocalConf *pLC = ev_device->pLC;

        if (!(le16_to_cpu(pLC->INTCSR) & PLX9030_INTCSR_LINTI1_STAT))
            return IRQ_NONE;
    }

    if (ev_device->access_device == DEVICE_EV) {
        /* We turn off the interrupt line here to signal the kernel that the interrupt is
           handled and we notify the user space application about a pending irq. */
        ev_plx_irq_disable(ev_device);
        kill_fasync(&ev_device->async_queue, SIGIO, POLL_IN);
    } else {
        struct EvrQueues *evrq = ev_device->evrq;
        volatile struct MrfErRegs *pEr = ev_device->pEv;
        int flags = be32_to_cpu(pEr->IrqFlag);
        int i;

        /* Clear everything but FIFOFULL. */
        if(flags & ~EVR_IRQFLAG_FIFOFULL)
            pEr->IrqFlag = be32_to_cpu(flags & ~EVR_IRQFLAG_FIFOFULL); 

        if(flags & EVR_IRQFLAG_DATABUF) {
            int databuf_sts = be32_to_cpu(pEr->DataBufControl);
            long long nextbp = evrq->dwp + 1;
            int idx = nextbp & (MAX_EVR_DBQ - 1);

            evrq->dbq[idx].status = databuf_sts;
            if (!(databuf_sts & (1<<C_EVR_DATABUF_CHECKSUM))) {
                /* If no checksum error, grab the buffer too. */
                int size = (databuf_sts & ((1<<(C_EVR_DATABUF_SIZEHIGH+1))-1));

                for (i = 0;  i < (size >> 2);  i++)
                    evrq->dbq[idx].data[i] = be32_to_cpu(pEr->Databuf[i]);
            }
            evrq->dwp = nextbp;
            /* TBD - barrier? */
            /* If no one is listening, stop the data buffer! */
            if (EVR_IRQFLAG_DATABUF & be32_to_cpu(((struct MrfErRegs *)ev_device->pEv)->IrqEnable))
                pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_LOAD);
            else
                pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_STOP);
        }

        if(flags & EVR_IRQFLAG_EVENT) {
            long long nextbp;
            int idx;
            int stat = flags;
            i = 0;

            do {
                nextbp = evrq->ewp + 1;
                idx = nextbp & (MAX_EVR_EVTQ - 1);
                evrq->evtq[idx].EventCode = be32_to_cpu(pEr->FIFOEvent);
                evrq->evtq[idx].TimestampHigh = be32_to_cpu(pEr->FIFOSeconds);
                evrq->evtq[idx].TimestampLow = be32_to_cpu(pEr->FIFOTimestamp);
                i++;
                stat = be32_to_cpu(pEr->IrqFlag);
                evrq->ewp = nextbp;
                /* TBD - barrier? */
            } while ((stat & (1 << C_EVR_IRQFLAG_EVENT)) && (i < EVR_FIFO_EVENT_LIMIT));
        }

        if(flags & EVR_IRQFLAG_FIFOFULL) {
            int ctrl = be32_to_cpu(pEr->Control);
            ctrl |= (1 << C_EVR_CTRL_RESET_EVENTFIFO);
            pEr->Control = be32_to_cpu(ctrl);
            evrq->fifofull++;
        }

        pEr->IrqFlag = be32_to_cpu(flags);

        for (i = 0; i < MAX_EVR_OPENS; i++)
            if (ev_device->shared[i].parent) {
                unsigned long flags;
                u32 pendingirq;
                spin_lock_irqsave(&ev_device->shared[i].lock, flags);
                pendingirq = ev_device->shared[i].pendingirq | (flags & ev_device->shared[i].irqmask);
                ev_device->shared[i].pendingirq = pendingirq;
                spin_unlock_irqrestore(&ev_device->shared[i].lock, flags);
                if (pendingirq)
                    wake_up(&ev_device->shared[i].waitq);
            }
    }

    return IRQ_HANDLED;
}

ssize_t ev_read(struct file *filp, char __user *buf, size_t count,
		 loff_t *f_pos)
{
  ssize_t retval = 0;
  struct shared_mrf *shared = ((struct shared_mrf *) filp->private_data);
  struct mrf_dev *ev_device = shared->parent;

  if (down_interruptible(&ev_device->sem))
    return -ERESTARTSYS;

  switch (ev_device->access_device)
    {
    case DEVICE_EEPROM:
      {
	if (*f_pos > ev_device->eeprom_data_size)
	  break;

	if (*f_pos + count > ev_device->eeprom_data_size)
	  count = ev_device->eeprom_data_size - *f_pos;
	
	if (copy_to_user(buf, &ev_device->eeprom_data[*f_pos], count))
	  {
	    retval = -EFAULT;
	    break;
	  }

	*f_pos += count;

	retval = count;
	break;
      }
    case DEVICE_XCF:
      {
	if ((*f_pos & ~(XCF_DATA_ALLOC_SIZE - 1)) !=
	    ev_device->xcf_data_pos)
	  {
	    jtag_XCF_read(ev_device, ev_device->xcf_data,
			  (unsigned int) (*f_pos & ~(XCF_DATA_ALLOC_SIZE - 1)),
			  XCF_DATA_ALLOC_SIZE);
	    ev_device->xcf_data_pos = *f_pos & ~(XCF_DATA_ALLOC_SIZE - 1);
	  }

	if (*f_pos + count > ev_device->xcf_data_pos +
	    XCF_DATA_ALLOC_SIZE)
	  count = ev_device->xcf_data_pos + XCF_DATA_ALLOC_SIZE - *f_pos;
	
	if (copy_to_user(buf, &ev_device->xcf_data[(*f_pos & (XCF_DATA_ALLOC_SIZE - 1))], count))
	  {
	    retval = -EFAULT;
	    break;
	  }

	*f_pos += count;

	retval = count;
	break;
      }
    case DEVICE_FPGA:
      {
	if (*f_pos > ev_device->fpga_conf_size)
	  break;

	if (*f_pos + count > ev_device->fpga_conf_size)
	  count = ev_device->fpga_conf_size - *f_pos;
	
	if (copy_to_user(buf, &ev_device->fpga_conf_data[*f_pos], count))
	  {
	    retval = -EFAULT;
	    break;
	  }

	*f_pos += count;

	retval = count;
	break;
      }
    case DEVICE_SHEV:
      {
        u32 pendingirq;
        unsigned long flags;

        /* If they don't want the whole thing, don't give 'em any! */
        if (count < sizeof(pendingirq))
          break;
        while (!shared->pendingirq) {
          up(&ev_device->sem);
          if (filp->f_flags & O_NONBLOCK)
            return -EAGAIN;
          if (wait_event_interruptible(shared->waitq, shared->pendingirq))
            return -ERESTARTSYS;
          if (down_interruptible(&ev_device->sem))
            return -ERESTARTSYS;
        }
        spin_lock_irqsave(&shared->lock, flags);
        pendingirq = shared->pendingirq;
        shared->pendingirq = 0;
        spin_unlock_irqrestore(&shared->lock, flags);
	if (copy_to_user(buf, &pendingirq, sizeof(pendingirq))) {
	    retval = -EFAULT;
	    break;
	  }
	*f_pos += sizeof(pendingirq);
	retval = sizeof(pendingirq);
        break;
      }
    }

  up(&ev_device->sem);
  return retval;
}

/**********************************************************************\
*                                                                      *
* Functions below this actually program the EVR hardware.              *
*                                                                      *
\**********************************************************************/

void set_irq_mask(struct mrf_dev *ev_device)
{
    int i;
    u32 mask = 0;

    for (i = 0; i < MAX_EVR_OPENS; i++)
        if (ev_device->shared[i].parent)
            mask |= ev_device->shared[i].irqmask;
    ((struct MrfErRegs *)ev_device->pEv)->IrqEnable = be32_to_cpu(mask);
}

void set_event_table(struct mrf_dev *ev_device)
{
    int i, j;

    if (ev_device->refcount_ev == 1) {
        /* Let's avoid a copy if we only have one open! */
        for (i = 0; i < MAX_EVR_OPENS; i++)
            if (ev_device->shared[i].parent) {
                ErUpdateRam(ev_device->pEv, ev_device->shared[i].evttab);
                return;
            }
    } else {
        u16 evttab[256];

        memset(evttab, 0, sizeof(evttab));
        for (j = 0; j < MAX_EVR_OPENS; j++)
            if (ev_device->shared[j].parent)
                for (i = 0; i < 256; i++)
                    evttab[i] |= ev_device->shared[j].evttab[i];
        ErUpdateRam(ev_device->pEv, evttab);
    }
}

