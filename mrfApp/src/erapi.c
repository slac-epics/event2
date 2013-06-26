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

#include "erapi.h"

extern void EvrIrqHandlerThreadCreate(void (**handler) (int), int);

/*
#define DEBUG 1
*/

#define DEBUG_PRINTF printf
unsigned int	erapiDebug	= 1;

int EvrOpen(struct MrfErRegs **pEr, char *device_name)
{
  int fd;

  /* Open Event Receiver device for read/write */
  fd = open(device_name, O_RDWR);
#ifdef DEBUG
  DEBUG_PRINTF("EvrOpen: open(\"%s\", O_RDWR) returned %d\n", device_name, fd);
#endif
  if (fd != -1)
    {
      /* Memory map Event Receiver registers */
      *pEr = (struct MrfErRegs *) mmap(0, EVR_MEM_WINDOW, PROT_READ | PROT_WRITE,
					MAP_SHARED, fd, 0);
#ifdef DEBUG
  DEBUG_PRINTF("EvrOpen: mmap returned %p, errno %d\n", *pEr,
	       errno);
#endif
      if (*pEr == MAP_FAILED)
	{
	  close(fd);
	  return -1;
	}
    }

  return fd;
}

int EvrClose(int fd)
{
  int result;

  result = munmap(0, EVR_MEM_WINDOW);
  return close(fd);
}

int EvrGetEnable(volatile struct MrfErRegs *pEr)
{
  return be32_to_cpu(pEr->Control & be32_to_cpu(1 << C_EVR_CTRL_MASTER_ENABLE));
}

int EvrGetViolation(volatile struct MrfErRegs *pEr, int clear)
{
  int result;

  result = be32_to_cpu(pEr->IrqFlag & be32_to_cpu(1 << C_EVR_IRQFLAG_VIOLATION));
  if (clear && result)
    pEr->IrqFlag = be32_to_cpu(result);

  return result;
}

void EvrDumpStatus(volatile struct MrfErRegs *pEr)
{
  int result;

  result = be32_to_cpu(pEr->Status);
  DEBUG_PRINTF("Status %08x ", result);
  if (result & (1 << C_EVR_STATUS_LEGACY_VIO))
    DEBUG_PRINTF("LEGVIO ");
  if (result & (1 << C_EVR_STATUS_FIFO_STOPPED))
    DEBUG_PRINTF("FIFOSTOP ");
  DEBUG_PRINTF("\n");
  result = be32_to_cpu(pEr->Control);
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
  result = be32_to_cpu(pEr->IrqFlag);
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
  result = be32_to_cpu(pEr->DataBufControl);
  DEBUG_PRINTF("DataBufControl %08x\n", result);
}

int EvrDumpMapRam(volatile struct MrfErRegs *pEr, int ram)
{
  int code;
  int intev;
  int ptrig, pset, pclr;

  for (code = 0; code <= EVR_MAX_EVENT_CODE; code++)
    {
      intev = be32_to_cpu(pEr->MapRam[ram][code].IntEvent);
      ptrig = be32_to_cpu(pEr->MapRam[ram][code].PulseTrigger);
      pset = be32_to_cpu(pEr->MapRam[ram][code].PulseSet);
      pclr = be32_to_cpu(pEr->MapRam[ram][code].PulseClear);

      if (intev ||
	  ptrig ||
	  pset ||
	  pclr)
	{
	  DEBUG_PRINTF("Code 0x%02x (%3d): ", code, code);
	  if (intev & (1 << C_EVR_MAP_SAVE_EVENT))
	    DEBUG_PRINTF("SAVE ");
	  if (intev & (1 << C_EVR_MAP_LATCH_TIMESTAMP))
	    DEBUG_PRINTF("LTS ");
	  if (intev & (1 << C_EVR_MAP_LED_EVENT))
	    DEBUG_PRINTF("LED ");
	  if (intev & (1 << C_EVR_MAP_FORWARD_EVENT))
	    DEBUG_PRINTF("FWD ");
	  if (intev & (1 << C_EVR_MAP_STOP_FIFO))
	    DEBUG_PRINTF("STOPFIFO ");
	  if (intev & (1 << C_EVR_MAP_HEARTBEAT_EVENT))
	    DEBUG_PRINTF("HB ");
	  if (intev & (1 << C_EVR_MAP_RESETPRESC_EVENT))
	    DEBUG_PRINTF("RESPRSC ");
	  if (intev & (1 << C_EVR_MAP_TIMESTAMP_RESET))
	    DEBUG_PRINTF("RESTS ");
	  if (intev & (1 << C_EVR_MAP_TIMESTAMP_CLK))
	    DEBUG_PRINTF("TSCLK ");
	  if (intev & (1 << C_EVR_MAP_SECONDS_1))
	    DEBUG_PRINTF("SEC1 ");
	  if (intev & (1 << C_EVR_MAP_SECONDS_0))
	    DEBUG_PRINTF("SEC0 ");
	  if (ptrig)
	    DEBUG_PRINTF("Trig %08x", ptrig);
	  if (pset)
	    DEBUG_PRINTF("Set %08x", pset);
	  if (pclr)
	    DEBUG_PRINTF("Clear %08x", pclr);
	  DEBUG_PRINTF("\n");
	}
    }
  return 0;
}

int EvrClearFIFO(volatile struct MrfErRegs *pEr)
{
  int ctrl;

  ctrl = be32_to_cpu(pEr->Control);
  ctrl |= (1 << C_EVR_CTRL_RESET_EVENTFIFO);
  pEr->Control = be32_to_cpu(ctrl);

  return be32_to_cpu(pEr->Control);
}

int EvrGetFIFOEvent(volatile struct MrfErRegs *pEr, struct FIFOEvent *fe)
{
  int stat;

  stat = be32_to_cpu(pEr->IrqFlag);
  if (stat & (1 << C_EVR_IRQFLAG_EVENT))
    {
      fe->EventCode = be32_to_cpu(pEr->FIFOEvent);
      fe->TimestampHigh = be32_to_cpu(pEr->FIFOSeconds);
      fe->TimestampLow = be32_to_cpu(pEr->FIFOTimestamp);
      return 0;
    }
  else
    return -1;
}

int EvrDumpFIFO(volatile struct MrfErRegs *pEr)
{
  struct FIFOEvent fe;
  int i;

  do
    {
      i = EvrGetFIFOEvent(pEr, &fe);
      if (!i)
	{
	  printf("Code %08x, %08x:%08x\n",
		 fe.EventCode, fe.TimestampHigh, fe.TimestampLow);
	}
    }
  while (!i);

  return 0;
}

int EvrSetPulseParams(volatile struct MrfErRegs *pEr, int pulse, u32 presc,
		      u32 delay, u32 width)
{
  if (pulse < 0 || pulse >= EVR_MAX_PULSES)
    return -1;

  pEr->Pulse[pulse].Prescaler = be32_to_cpu(presc);
  pEr->Pulse[pulse].Delay = be32_to_cpu(delay);
  pEr->Pulse[pulse].Width = be32_to_cpu(width);
  if ( erapiDebug	>= 1 )
  {
	/*
	 * Sanity check on prescaler value (due to fixed bug in generator allocation)
	 * Prescaler value is readable on generators 0-1
	 * A MRF firmware bug prevents reading prescaler on generators 2-3
	 * Generators 4-9 do not support prescaling and always read back 0
	 */
	if ( pulse < 3 )
	{
	  if ( be32_to_cpu(pEr->Pulse[pulse].Prescaler) != presc )
		printf( "%s Pulse %d: Unable to update prescaler from %d to %d\n", __func__,
				pulse, be32_to_cpu(pEr->Pulse[pulse].Prescaler), presc );
	  else if ( presc != 0 )
		printf( "%s Pulse %d: Success! prescaler is now %d\n", __func__, pulse, presc );
	}
  }
  return 0;
}

void EvrDumpPulses(volatile struct MrfErRegs *pEr, int pulses)
{
  int i, control;

  for (i = 0; i < pulses; i++)
    {
      DEBUG_PRINTF("Pulse %02x Presc %08x Delay %08x Width %08x", i,
		   be32_to_cpu(pEr->Pulse[i].Prescaler), 
		   be32_to_cpu(pEr->Pulse[i].Delay), 
		   be32_to_cpu(pEr->Pulse[i].Width));
      control = be32_to_cpu(pEr->Pulse[i].Control);
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

int EvrSetPulseProperties(volatile struct MrfErRegs *pEr, int pulse, int polarity,
			  int map_reset_ena, int map_set_ena, int map_trigger_ena,
			  int enable)
{
  int result;

  if (pulse < 0 || pulse >= EVR_MAX_PULSES)
    return -1;

  result = be32_to_cpu(pEr->Pulse[pulse].Control);

  /* 0 clears, 1 sets, others don't change */
  if (polarity == 0)
    result &= ~(1 << C_EVR_PULSE_POLARITY);
  if (polarity == 1)
    result |= (1 << C_EVR_PULSE_POLARITY);

  if (map_reset_ena == 0)
    result &= ~(1 << C_EVR_PULSE_MAP_RESET_ENA);
  if (map_reset_ena == 1)
    result |= (1 << C_EVR_PULSE_MAP_RESET_ENA);

  if (map_set_ena == 0)
    result &= ~(1 << C_EVR_PULSE_MAP_SET_ENA);
  if (map_set_ena == 1)
    result |= (1 << C_EVR_PULSE_MAP_SET_ENA);

  if (map_trigger_ena == 0)
    result &= ~(1 << C_EVR_PULSE_MAP_TRIG_ENA);
  if (map_trigger_ena == 1)
    result |= (1 << C_EVR_PULSE_MAP_TRIG_ENA);

  if (enable == 0)
    result &= ~(1 << C_EVR_PULSE_ENA);
  if (enable == 1)
    result |= (1 << C_EVR_PULSE_ENA);

#ifdef DEBUG
  DEBUG_PRINTF("Pulse[%d].Control %08x\n", pulse, result);
#endif

  pEr->Pulse[pulse].Control = be32_to_cpu(result);

  return 0;
}

int EvrSetUnivOutMap(volatile struct MrfErRegs *pEr, int output, int map)
{
  if (output < 0 || output >= EVR_MAX_UNIVOUT_MAP)
    return -1;

  pEr->UnivOutMap[output] = be16_to_cpu(map);
  return 0;
}

void EvrDumpUnivOutMap(volatile struct MrfErRegs *pEr, int outputs)
{
  int i;

  for (i = 0; i < outputs; i++)
    DEBUG_PRINTF("UnivOut[%d] %02x\n", i, be16_to_cpu(pEr->UnivOutMap[i]));
}

int EvrSetFPOutMap(volatile struct MrfErRegs *pEr, int output, int map)
{
  if (output < 0 || output >= EVR_MAX_FPOUT_MAP)
    return -1;

  pEr->FPOutMap[output] = be16_to_cpu(map);
  return 0;
}

void EvrDumpFPOutMap(volatile struct MrfErRegs *pEr, int outputs)
{
  int i;

  for (i = 0; i < outputs; i++)
    DEBUG_PRINTF("FPOut[%d] %02x\n", i, be16_to_cpu(pEr->FPOutMap[i]));
}

int EvrSetTBOutMap(volatile struct MrfErRegs *pEr, int output, int map)
{
  if (output < 0 || output >= EVR_MAX_TBOUT_MAP)
    return -1;

  pEr->TBOutMap[output] = be16_to_cpu(map);
  return 0;
}

void EvrDumpTBOutMap(volatile struct MrfErRegs *pEr, int outputs)
{
  int i;

  for (i = 0; i < outputs; i++)
    DEBUG_PRINTF("TBOut[%d] %02x\n", i, be16_to_cpu(pEr->TBOutMap[i]));
}

void EvrIrqAssignHandler(volatile struct MrfErRegs *pEr, int fd,
			 void (*handler)(int))
{
  int oflags;
  static int have_thread = 0;
  static void (*h)(int) = NULL;

  /*
   * The Newest Regime: We create a separate handler that reads from our fd.
   */
  h = handler;
  if (!have_thread)
      EvrIrqHandlerThreadCreate(&h, fd);
}

int EvrIrqEnable(volatile struct MrfErRegs *pEr, int mask)
{
  pEr->IrqEnable = be32_to_cpu(mask);
  return be32_to_cpu(pEr->IrqEnable);
}

int EvrGetIrqFlags(volatile struct MrfErRegs *pEr)
{
  return be32_to_cpu(pEr->IrqFlag);
}

int EvrClearIrqFlags(volatile struct MrfErRegs *pEr, int mask)
{
  pEr->IrqFlag = be32_to_cpu(mask);
  return be32_to_cpu(pEr->IrqFlag);
}

void EvrIrqHandled(int fd)
{
  ioctl(fd, EV_IOCIRQEN);
}

int EvrSetPulseIrqMap(volatile struct MrfErRegs *pEr, int map)
{
  pEr->PulseIrqMap = be32_to_cpu(map);
  return 0;
}

void EvrClearDiagCounters(volatile struct MrfErRegs *pEr)
{
  pEr->DiagReset = 0xffffffff;
  pEr->DiagReset = 0x0;
}

void EvrEnableDiagCounters(volatile struct MrfErRegs *pEr, int enable)
{
  if (enable)
    pEr->DiagCE = 0xffffffff;
  else
    pEr->DiagCE = 0;
}

u32 EvrGetDiagCounter(volatile struct MrfErRegs *pEr, int idx)
{
  return be32_to_cpu(pEr->DiagCounter[idx]);
}

void EvrDumpHex(volatile struct MrfErRegs *pEr)
{
  u32 *p = (u32 *) pEr;
  int i,j;

  for (i = 0; i < 0x600; i += 0x20)
    {
      printf("%08x: ", i);
      for (j = 0; j < 8; j++)
	printf("%08x ", be32_to_cpu(*p++));
      printf("\n");
    }
}

int EvrSetDBufMode(volatile struct MrfErRegs *pEr, int enable)
{
  if (enable)
    pEr->DataBufControl = be32_to_cpu(1 << C_EVR_DATABUF_MODE);
  else
    pEr->DataBufControl = 0;

  return EvrGetDBufStatus(pEr);
}

int EvrGetDBufStatus(volatile struct MrfErRegs *pEr)
{
  return be32_to_cpu(pEr->DataBufControl);
}

int EvrReceiveDBuf(volatile struct MrfErRegs *pEr, int enable)
{
  if (enable)
    pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_LOAD);
  else
    pEr->DataBufControl |= be32_to_cpu(1 << C_EVR_DATABUF_STOP);

  return EvrGetDBufStatus(pEr);
}

int EvrGetTimestampCounter(volatile struct MrfErRegs *pEr)
{
  return be32_to_cpu(pEr->TimestampEventCounter);
}

int EvrGetFormFactor(volatile struct MrfErRegs *pEr)
{
  int stat;
  
  stat = be32_to_cpu(pEr->FPGAVersion);
  return ((stat >> 24) & 0x0f);
}
