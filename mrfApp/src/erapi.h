/*
  erapi.h -- Definitions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006

*/

#include<sys/user.h>  /* For PAGE_* */
#include "evrmemmap.h"

/* Function prototypes */
int EvrOpen(struct MrfErRegs **pEr, char *device_name);
int EvrClose(int fd);
int EvrGetViolation(volatile struct MrfErRegs *pEr);
void EvrDumpStatus(volatile struct MrfErRegs *pEr);
int EvrSetPulseParams(volatile struct MrfErRegs *pEr, int pulse, u32 presc,
		      u32 delay, u32 width);
void EvrDumpPulses(volatile struct MrfErRegs *pEr, int pulses);
int EvrSetPulseProperties(volatile struct MrfErRegs *pEr, int pulse, int polarity,
			  int map_reset_ena, int map_set_ena, int map_trigger_ena,
			  int enable);
void EvrDumpUnivOutMap(volatile struct MrfErRegs *pEr, int outputs);
void EvrDumpFPOutMap(volatile struct MrfErRegs *pEr, int outputs);
void EvrDumpTBOutMap(volatile struct MrfErRegs *pEr, int outputs);
void EvrIrqAssignHandler(volatile struct MrfErRegs *pEr, int fd, void (*handler)(int, int));
int EvrGetTimestampCounter(volatile struct MrfErRegs *pEr);
