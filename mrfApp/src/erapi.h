/*
  erapi.h -- Definitions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006

*/

#include<sys/user.h>  /* For PAGE_* */
#include<stddef.h>    /* For offsetof */
#include "evrmemmap.h"

/* Function prototypes */
int EvrOpen(void **pEq, char *device_name);
int EvrClose(int fd);
int EvrGetViolation(int fd);
void EvrDumpStatus(int fd);
int EvrSetPulseParams(int fd, int pulse, u32 presc, u32 delay, u32 width,
                      int polarity, int enable);
void EvrDumpPulses(int fd, int pulses);
void EvrIrqAssignHandler(int fd, void (*handler)(int, int));
int EvrGetTimestampCounter(int fd);
