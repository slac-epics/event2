#undef DEBUG
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
#include <linux/sched.h>

#include <asm/page.h>
#include <asm/uaccess.h>

#include "pci_mrfev.h"
MODULE_LICENSE("GPL");

#ifndef DEVICE_NAME
#define DEVICE_NAME "pci_mrf"
#endif
#ifdef DEBUG
#define DPF(a) printk a
#else
#define DPF(a)
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

              /*
               * In shared mode, we need to initialize the device!
               */
              if (ev_device->access_device == DEVICE_SHEV) {
                  volatile struct MrfErRegs *pEr = ev_device->pEv;
                  int form = (be32_to_cpu(pEr->FPGAVersion)>>24) & 0x0F;
                  int j;

                  pEr->Control |= be32_to_cpu(1 << C_EVR_CTRL_MASTER_ENABLE);
                  pEr->EvCntPresc = be32_to_cpu(1);
                  pEr->FracDiv = be32_to_cpu(CLOCK_119000_MHZ);
                  for (j = 11; j >= 0; j--) {
                      if(form == EVR_FORM_PMC)
                          pEr->FPOutMap[j] = be16_to_cpu(j);
                      else if(form == EVR_FORM_CPCI || form == EVR_FORM_SLAC)
                          pEr->UnivOutMap[j] = be16_to_cpu(j);
                  }
                  pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_MODE);
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
  int i;
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
      for (i = 0; i < EVR_MAX_PULSES; i++)
          if (ev_device->pulse[i] == shared->idx)
              ev_device->pulse[i] = -1;
      /* Fall through! */
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

  DPF((KERN_INFO DEVICE_NAME ": write %d bytes.\n", (int) count));

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

#ifndef HAVE_UNLOCKED_IOCTL
#ifdef CONFIG_COMPAT
long ev_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return ev_ioctl(NULL, file, cmd, arg);
}
#endif
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

#if LINUX_VERSION_CODE < 0x020617
/*
 * nopage was changed to fault circa 2.6.23.  This version might be off by one, but
 * close enough.  All we want to do is fail here anyway, so...
 */
struct page *ev_vma_nopage(struct vm_area_struct *vma, unsigned long address, int *type)
{
    return NOPAGE_SIGBUS;
}
#else
int ev_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    return VM_FAULT_SIGBUS;
}
#endif

static struct vm_operations_struct ev_remap_vm_ops = {
  .open = ev_vma_open,
  .close = ev_vma_close,
#if LINUX_VERSION_CODE < 0x020617
  .nopage = ev_vma_nopage,
#else
  .fault = ev_vma_fault,
#endif
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

  if (offset) {
      printk(KERN_NOTICE DEVICE_NAME ": mmap offset %ld is not zero!\n", offset);
      return -EINVAL;
  }

  if (ev_device->access_device == DEVICE_EV) {
    if (vsize > ev_device->lenEv) {
      printk(KERN_NOTICE DEVICE_NAME ": mmap vsize %08x, ev_device->lenEv %08x\n",
             (unsigned int) vsize, (unsigned int) ev_device->lenEv);
      return -EINVAL;
    } else {
      unsigned long psize = sizeof(struct MrfErRegs);    /* Size of region from offset to end */
      unsigned long physical = (unsigned long) ev_device->mrEv;
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
    }
  } else {
    if (vsize > EVR_SH_MEM_WINDOW) {
      printk(KERN_NOTICE DEVICE_NAME ": mmap vsize %08x, qsize %08zx\n",
             (unsigned int) vsize, sizeof(struct EvrQueues));
      return -EINVAL;
    } else {
      /* Map this read only!! */
      result = remap_pfn_range(vma, start,
                               __pa(ev_device->evrq) >> PAGE_SHIFT,
                               vsize, PAGE_READONLY);
      if (result)
          return -EAGAIN;
    }
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
  if (SLAC_EVR(ev_device)) {
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
  if (SLAC_EVR(ev_device)) {
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
#ifdef HAVE_UNLOCKED_IOCTL
long ev_unlocked_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg)
#else
int ev_ioctl(struct inode *inode, struct file *filp,
             unsigned int cmd, unsigned long arg)
#endif
{
  struct shared_mrf *shared = ((struct shared_mrf *) filp->private_data);
  struct mrf_dev *ev_device = shared->parent;
  int ret = 0;

  /* Check that cmd is valid */
  if (_IOC_TYPE(cmd) != EV_IOC_MAGIC)
    return -ENOTTY;
  if (_IOC_NR(cmd) > EV_IOC_MAX)
    return -ENOTTY;
  /* Check access */
  if (_IOC_DIR(cmd) & _IOC_READ)
    ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if (ret)
    return -EFAULT;

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
        DPF((KERN_ALERT "Setting IRQ for fd%d to 0x%08x\n", shared->idx, shared->irqmask));
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

    case EV_IOCTRIG:
      if (ev_device->access_device == DEVICE_SHEV) {
        struct EvrIoctlTrig p;
        if (copy_from_user(&p, (struct EvrIoctlTrig *)arg, sizeof(struct EvrIoctlTrig))) {
          return -EACCES;
        }
        if (p.Id >= EVR_MAX_PULSES)
          return -EINVAL;
        switch (p.Op) {
        case EvrTrigFree:
            if (ev_device->pulse[p.Id] == shared->idx)
                ev_device->pulse[p.Id] = -1;
            else
                ret = -EBUSY;
            break;
        case EvrTrigAlloc:
            if (ev_device->pulse[p.Id] != shared->idx &&
                ev_device->pulse[p.Id] != -1) {
                ret = -EBUSY;
                break;
            } else
                ev_device->pulse[p.Id] = shared->idx;
            break;
        case EvrTrigSteal:
            ev_device->pulse[p.Id] = shared->idx;
            break;
        case EvrTrigCheck:
            if (ev_device->pulse[p.Id] != shared->idx &&
                ev_device->pulse[p.Id] != -1) {
                ret = -EBUSY;
            }
            break;
        default:
            ret = -EINVAL;
            break;
        }
      } else
        ret = -ENOTTY;
      break;

    case EV_IOCPULSE:
      if (ev_device->access_device == DEVICE_SHEV) {
        struct EvrIoctlPulse p;
        if (copy_from_user(&p, (struct EvrIoctlPulse *)arg, sizeof(struct EvrIoctlPulse))) {
          return -EACCES;
        }
        if (p.Id >= EVR_MAX_PULSES)
          return -ENOTTY;
        if (ev_device->pulse[p.Id] == -1) {
          /* If this pulse is unused, claim it! */
          ev_device->pulse[p.Id] = shared->idx;
        } else if (ev_device->pulse[p.Id] != shared->idx) {
          /* If this pulse is claimed by someone else, fail! */
          return -EBUSY;
        }
        {
          volatile struct MrfErRegs *pEr = ev_device->pEv;
          pEr->Pulse[p.Id].Control = be32_to_cpu(p.Pulse.Control);
          if (p.Pulse.Control & (1 << C_EVR_PULSE_ENA)) {
            DPF((KERN_ALERT "Programming pulse %d for %d: ps=%d, d=%d, w=%d, pol=%s\n",
                 p.Id, shared->idx, p.Pulse.Prescaler, p.Pulse.Delay, p.Pulse.Width,
                 (p.Pulse.Control & (1 << C_EVR_PULSE_POLARITY)) ? "NEG" : "POS"));
            /* If we're enabling, program the times too. */
            pEr->Pulse[p.Id].Prescaler = be32_to_cpu(p.Pulse.Prescaler);
            pEr->Pulse[p.Id].Delay = be32_to_cpu(p.Pulse.Delay);
            pEr->Pulse[p.Id].Width = be32_to_cpu(p.Pulse.Width);
          } else {
            DPF((KERN_ALERT "Disabling pulse %d for %d.\n", p.Id, shared->idx));
            /* Change of plan: don't free the pulse! */
          }
        }
      } else
        ret = -ENOTTY;
      break;

    case EV_IOCREAD32:
      if (ev_device->access_device == DEVICE_SHEV) {
        volatile u32 *pEr = ev_device->pEv;
        u32 *buf32 = shared->tmp;
        u32 offset, size, i, limit;

        if (copy_from_user(buf32, (u32 *)arg, sizeof(u32) * 2)) {
          return -EACCES;
        }
        /* Args: offset, size */
        offset = buf32[0];
        size   = buf32[1];
        /* Offset and size must be aligned, and the read can't exceed EVR_MAX_READ. */
        if ((offset % sizeof(u32)) || (size % sizeof(u32)) || size > EVR_MAX_READ) {
            return -ENOTTY;
        }
        /* No reading the event queue! */
        if ((offset < 0x7c) && (offset + size) > 0x70) {
            return -ENOTTY;
        }
        offset = offset / sizeof(u32);
        limit  = size   / sizeof(u32);
        for (i = 0; i < limit; i++)
            buf32[i] = be32_to_cpu(pEr[offset + i]);
        if (copy_to_user((u32 *)arg, buf32, size)) {
            return -EFAULT;
        }
      } else
        ret = -ENOTTY;
      break;

    case EV_IOCREAD16:
      if (ev_device->access_device == DEVICE_SHEV) {
        volatile u16 *pEr = ev_device->pEv;
        u16 *buf16 = (u16 *)shared->tmp;
        u16 offset, size, i, limit;

        if (copy_from_user(buf16, (u16 *)arg, sizeof(u16) * 2)) {
          return -EACCES;
        }
        /* Args: offset, size */
        offset = buf16[0];
        size   = buf16[1];
        /* Offset and size must be aligned, and the read can't exceed EVR_MAX_READ. */
        if ((offset % sizeof(u16)) || (size % sizeof(u16)) || size > EVR_MAX_READ)
            return -ENOTTY;
        /* No reading the event queue! */
        if ((offset < 0x7c) && (offset + size) > 0x70)
            return -ENOTTY;
        offset = offset / sizeof(u16);
        limit  = size   / sizeof(u16);
        for (i = 0; i < limit; i++)
            buf16[i] = be16_to_cpu(pEr[offset + i]);
        if (copy_to_user((u16 *)arg, buf16, size))
            return -EFAULT;
      } else
        ret = -ENOTTY;
      break;

    default:
      return -ENOTTY;
    }

  return ret;
}

#if LINUX_VERSION_CODE < 0x020617
irqreturn_t ev_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
irqreturn_t ev_interrupt(int irq, void *dev_id)
#endif
{
    struct mrf_dev *ev_device = (struct mrf_dev *) dev_id;
    volatile struct MrfErRegs *pEv = ev_device->pEv;

    /* We use shared interrupts: return immediately if irq is not from our
       device. */
    if (SLAC_EVR(ev_device)) {
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
        /* Cache all the interrupt flags. For SLAC G2, we attempt to handle all interrupt sources
           in one pass of the ISR. */
        int flags = be32_to_cpu(pEr->IrqFlag);
        int i;
        DPF((KERN_ALERT "I%08x\n", flags));
        /* Clear everything but FIFOFULL. */
        if(flags & ~EVR_IRQFLAG_FIFOFULL)
            pEr->IrqFlag = be32_to_cpu(flags & ~EVR_IRQFLAG_FIFOFULL);
            /* Now, reset interrupt flags to zero - MRF and SLAC G1 are immune to this, SLAC G2
               needs it. */
            pEr->IrqFlag = be32_to_cpu(0); 

        if(flags & EVR_IRQFLAG_DATABUF) {
            int databuf_sts = be32_to_cpu(pEr->DataBufControl);
            long long nextbp = evrq->dwp;
            int idx = nextbp & (MAX_EVR_DBQ - 1);

            evrq->dbq[idx].status = databuf_sts;
            if (!(databuf_sts & (1<<C_EVR_DATABUF_CHECKSUM))) {
                /* If no checksum error, grab the buffer too. */
                int size = (databuf_sts & ((1<<(C_EVR_DATABUF_SIZEHIGH+1))-1));
                u32 *dd   = evrq->dbq[idx].data;
                volatile u32 *ds   = pEr->Databuf;

                for (i = 0;  i < (size >> 2);  i++)
                    dd[i] = be32_to_cpu(ds[i]);
#if 0
                printk(KERN_ALERT "D%d: %08x %08x.%08x\n", idx, dd[0], dd[7], dd[8]);
#endif
            }
            evrq->dwp++;
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
                nextbp = evrq->ewp;
                idx = nextbp & (MAX_EVR_EVTQ - 1);
                evrq->evtq[idx].EventCode = be32_to_cpu(pEr->FIFOEvent);
                evrq->evtq[idx].TimestampHigh = be32_to_cpu(pEr->FIFOSeconds);
                evrq->evtq[idx].TimestampLow = be32_to_cpu(pEr->FIFOTimestamp);
                DPF((KERN_ALERT "E%d@%x = %d\n",
                     idx, evrq->evtq[idx].TimestampHigh, evrq->evtq[idx].EventCode));
                i++;
                stat = be32_to_cpu(pEr->IrqFlag);
                evrq->ewp = nextbp + 1;
                /* TBD - barrier? */
            } while ((stat & (1 << C_EVR_IRQFLAG_EVENT)) && (i < EVR_FIFO_EVENT_LIMIT));
        }

        if(flags & EVR_IRQFLAG_FIFOFULL) {
            int ctrl = be32_to_cpu(pEr->Control);
            ctrl |= (1 << C_EVR_CTRL_RESET_EVENTFIFO);
            pEr->Control = be32_to_cpu(ctrl);
            evrq->fifofull++;

            pEr->IrqFlag = be32_to_cpu(EVR_IRQFLAG_FIFOFULL);
            /* Now, reset interrupt flags to zero - MRF and SLAC G1 are immune to this, SLAC G2
               needs it. */
            pEr->IrqFlag = be32_to_cpu(0); 
        }

        /* This is a redundant interrupt flag clear-all. */
        /* pEr->IrqFlag = be32_to_cpu(flags); */

        for (i = 0; i < MAX_EVR_OPENS; i++)
            if (ev_device->shared[i].parent) {
                unsigned long state;
                u32 pendingirq;
                spin_lock_irqsave(&ev_device->shared[i].lock, state);
                pendingirq = ev_device->shared[i].pendingirq | (flags & ev_device->shared[i].irqmask);
                ev_device->shared[i].pendingirq = pendingirq;
                spin_unlock_irqrestore(&ev_device->shared[i].lock, state);
                DPF((KERN_ALERT "O%d %08x\n", i, pendingirq));
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
    volatile struct MrfErRegs * pEr = (struct MrfErRegs *)ev_device->pEv;

    for (i = 0; i < MAX_EVR_OPENS; i++)
        if (ev_device->shared[i].parent)
            mask |= ev_device->shared[i].irqmask;
    DPF((KERN_ALERT "New IrqEnable = 0x%08x\n", mask));
    pEr->IrqEnable = be32_to_cpu(mask);
    if (EVR_IRQFLAG_DATABUF & mask) {
        pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_LOAD);
    } else {
        pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_STOP);
    }
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

