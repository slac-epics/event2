/*
  erapi.c -- Functions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006
  Butcher: Michael Browne (SLAC)
  Date:   06.26.2013

  Heavily edited to support a shared board.  All routines that could screw other users have
  been removed.
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <endian.h>
#include <byteswap.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#define DEFINE_READ_EVR
#define INLINE_READ_EVR static
#include "erapi.h"

extern void EvrIrqHandlerThreadCreate(void (**handler) (int, int), int);

/*
#define DEBUG 1
*/

#define DEBUG_PRINTF printf
unsigned int	erapiDebug	= 1;

int EvrOpen(void **pEq, char *device_name)
{
  int fd;

  /* Open Event Receiver device for read/write */
  fd = open(device_name, O_RDWR);
#ifdef DEBUG
  DEBUG_PRINTF("EvrOpen: open(\"%s\", O_RDWR) returned %d\n", device_name, fd);
#endif
  if (fd == -1)
  {
  	  printf( "EvrOpen Error: open(\"%s\", O_RDWR) returned -1, errno %d\n", device_name, errno );
  }
  else
  {
      /* Memory map Event Receiver registers */
      *pEq = (void *) mmap(0, EVR_SH_MEM_WINDOW, PROT_READ, MAP_SHARED, fd, 0);
#ifdef DEBUG
      DEBUG_PRINTF("EvrOpen: mmap returned %p, errno %d\n", *pEq,
                   errno);
#endif
      if (*pEq == MAP_FAILED)
	  {
        printf( "EvrOpen Error: mmap returned %p, errno %d\n", *pEq, errno );
	    close(fd);
	    return -1;
	  }
  }

  return fd;
}

int EvrClose(int fd)
{
  int result;

  result = munmap(0, EVR_SH_MEM_WINDOW);
  return close(fd);
}

int EvrGetViolation(int fd)
{
  return READ_EVR_REGISTER(fd, IrqFlag) & (1 << C_EVR_IRQFLAG_VIOLATION);
}

void EvrDumpStatus(int fd)
{
  int result;

  result = READ_EVR_REGISTER(fd, Status);
  DEBUG_PRINTF("Status %08x ", result);
  if (result & (1 << C_EVR_STATUS_LEGACY_VIO))
    DEBUG_PRINTF("LEGVIO ");
  if (result & (1 << C_EVR_STATUS_FIFO_STOPPED))
    DEBUG_PRINTF("FIFOSTOP ");
  DEBUG_PRINTF("\n");
  result = READ_EVR_REGISTER(fd, Control);
  DEBUG_PRINTF("Control %08x: ", result);
  if (result & (1 << C_EVR_CTRL_MASTER_ENABLE))
    DEBUG_PRINTF("MSEN ");
  if (result & (1 << C_EVR_CTRL_EVENT_FWD_ENA))
    DEBUG_PRINTF("FWD ");
  if (result & (1 << C_EVR_CTRL_TXLOOPBACK))
    DEBUG_PRINTF("TXLP ");
  if (result & (1 << C_EVR_CTRL_RXLOOPBACK))
    DEBUG_PRINTF("RXLP ");
  if (result & (1 << C_EVR_CTRL_TS_CLOCK_DBUS))
    DEBUG_PRINTF("DSDBUS ");
  if (result & (1 << C_EVR_CTRL_MAP_RAM_ENABLE))
    DEBUG_PRINTF("MAPENA ");
  if (result & (1 << C_EVR_CTRL_MAP_RAM_SELECT))
    DEBUG_PRINTF("MAPSEL ");
  DEBUG_PRINTF("\n");
  result = READ_EVR_REGISTER(fd, IrqFlag);
  DEBUG_PRINTF("IRQ Flag %08x: ", result);
  if (result & (1 << C_EVR_IRQ_MASTER_ENABLE))
    DEBUG_PRINTF("IRQEN ");
  if (result & (1 << C_EVR_IRQFLAG_DATABUF))
    DEBUG_PRINTF("DBUF ");
  if (result & (1 << C_EVR_IRQFLAG_PULSE))
    DEBUG_PRINTF("PULSE ");
  if (result & (1 << C_EVR_IRQFLAG_EVENT))
    DEBUG_PRINTF("EVENT ");
  if (result & (1 << C_EVR_IRQFLAG_HEARTBEAT))
    DEBUG_PRINTF("HB ");
  if (result & (1 << C_EVR_IRQFLAG_FIFOFULL))
    DEBUG_PRINTF("FF ");
  if (result & (1 << C_EVR_IRQFLAG_VIOLATION))
    DEBUG_PRINTF("VIO ");
  DEBUG_PRINTF("\n");
  result = READ_EVR_REGISTER(fd, DataBufControl);
  DEBUG_PRINTF("DataBufControl %08x\n", result);
}

int EvrSetPulseParams(int fd, int pulse, u32 presc, u32 delay, u32 width,
                      int polarity, int enable)
{
  struct EvrIoctlPulse p;
  if (pulse < 0 || pulse >= EVR_MAX_PULSES)
    return EINVAL;
  p.Id              = pulse;
  p.Pulse.Prescaler = presc;
  p.Pulse.Delay     = delay;
  p.Pulse.Width     = width;
  p.Pulse.Control   = 0;
  if (polarity == 1)
    p.Pulse.Control |= (1 << C_EVR_PULSE_POLARITY);
  if (enable == 1)
    p.Pulse.Control |= (1 << C_EVR_PULSE_MAP_TRIG_ENA) | (1 << C_EVR_PULSE_ENA);
  if ( ioctl(fd, EV_IOCPULSE, &p ) ) {
	/* Note, if perror is used here, it complains device is busy for invalid pulse parameters */
	int	io_ret = errno;
	switch ( io_ret )
	{
	  case EBUSY:
		fprintf( stderr, "EVR pulse %d owned by a different process!\n", pulse );
		break;
	  case ENOTTY:
		fprintf( stderr, "Not a valid EVR request for pulse %d!\n", pulse );
		break;
	  case EFAULT:
		fprintf( stderr, "Unable to access EVR device!\n" );
		break;
	  default:
		perror("Programming pulse failed");
		break;
	}
	return io_ret;
  }
  return 0;
}

void EvrDumpPulses(int fd, int pulses)
{
  int i, control;
  struct PulseStruct p[EVR_MAX_PULSES];

  if (READ_EVR_REGION32(fd, Pulse, (u32 *)p, sizeof(struct PulseStruct) * pulses))
    return;

  for (i = 0; i < pulses; i++)
    {
      DEBUG_PRINTF("Pulse %02x Presc %08x Delay %08x Width %08x", i,
		   p[i].Prescaler, p[i].Delay, p[i].Width);
      control = p[i].Control;
      DEBUG_PRINTF(" Output %d", control & (1 << C_EVR_PULSE_OUT) ? 1 : 0);
      if (control & (1 << C_EVR_PULSE_POLARITY))
	DEBUG_PRINTF(" NEG");
      if (control & (1 << C_EVR_PULSE_MAP_RESET_ENA))
	DEBUG_PRINTF(" MAPRES");
      if (control & (1 << C_EVR_PULSE_MAP_SET_ENA))
	DEBUG_PRINTF(" MAPSET");
      if (control & (1 << C_EVR_PULSE_MAP_TRIG_ENA))
	DEBUG_PRINTF(" MAPTRIG");
      if (control & (1 << C_EVR_PULSE_ENA))
	DEBUG_PRINTF(" ENA");
      DEBUG_PRINTF("\n");
    }
}

void EvrIrqAssignHandler(int fd, void (*handler)(int, int))
{
  static int have_thread = 0;
  static void (*h)(int, int) = NULL;

  /*
   * The Newest Regime: We create a separate handler that reads from our fd.
   */
  h = handler;
  if (!have_thread)
      EvrIrqHandlerThreadCreate(&h, fd);
}

int EvrGetTimestampCounter(int fd)
{
  return READ_EVR_REGISTER(fd, TimestampEventCounter);
}
