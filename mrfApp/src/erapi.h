/*
  erapi.h -- Definitions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006

*/

/*
  Note: Byte ordering is big-endian.
 */

#include<sys/user.h>  /* For PAGE_* */
#include "evrmemmap.h"

/* Function prototypes */
int EvrOpen(struct MrfErRegs **pEr, char *device_name);
int EvrClose(int fd);
int EvrEnable(volatile struct MrfErRegs *pEr, int state);
int EvrGetEnable(volatile struct MrfErRegs *pEr);
void EvrDumpStatus(volatile struct MrfErRegs *pEr);
int EvrGetViolation(volatile struct MrfErRegs *pEr, int clear);
int EvrDumpMapRam(volatile struct MrfErRegs *pEr, int ram);
int EvrMapRamEnable(volatile struct MrfErRegs *pEr, int ram, int enable);
int EvrSetForwardEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrEnableEventForwarding(volatile struct MrfErRegs *pEr, int enable);
int EvrGetEventForwarding(volatile struct MrfErRegs *pEr);
int EvrSetLedEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetFIFOEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetLatchEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetFIFOStopEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrClearFIFO(volatile struct MrfErRegs *pEr);
int EvrGetFIFOEvent(volatile struct MrfErRegs *pEr, struct FIFOEvent *fe);
int EvrEnableFIFOStopEvent(volatile struct MrfErRegs *pEr, int enable);
int EvrGetFIFOStopEvent(volatile struct MrfErRegs *pEr);
int EvrEnableFIFO(volatile struct MrfErRegs *pEr, int enable);
int EvrGetFIFOState(volatile struct MrfErRegs *pEr);
int EvrDumpFIFO(volatile struct MrfErRegs *pEr);
int EvrSetPulseMap(volatile struct MrfErRegs *pEr, int ram, int code, int trig,
		   int set, int clear);
int EvrClearPulseMap(volatile struct MrfErRegs *pEr, int ram, int code, int trig,
		   int set, int clear);
int EvrSetPulseParams(volatile struct MrfErRegs *pEr, int pulse, u32 presc,
		      u32 delay, u32 width);
void EvrDumpPulses(volatile struct MrfErRegs *pEr, int pulses);
int EvrSetPulseProperties(volatile struct MrfErRegs *pEr, int pulse, int polarity,
			  int map_reset_ena, int map_set_ena, int map_trigger_ena,
			  int enable);
int EvrSetUnivOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpUnivOutMap(volatile struct MrfErRegs *pEr, int outputs);
int EvrSetFPOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpFPOutMap(volatile struct MrfErRegs *pEr, int outputs);
int EvrSetTBOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpTBOutMap(volatile struct MrfErRegs *pEr, int outputs);
void EvrIrqAssignHandler(volatile struct MrfErRegs *pEr, int fd, void (*handler)(int));
int EvrIrqEnable(volatile struct MrfErRegs *pEr, int mask);
int EvrGetIrqFlags(volatile struct MrfErRegs *pEr);
int EvrClearIrqFlags(volatile struct MrfErRegs *pEr, int mask);
void EvrIrqHandled(int fd);
int EvrSetPulseIrqMap(volatile struct MrfErRegs *pEr, int map);
void EvrClearDiagCounters(volatile struct MrfErRegs *pEr);
void EvrEnableDiagCounters(volatile struct MrfErRegs *pEr, int enable);
u32 EvrGetDiagCounter(volatile struct MrfErRegs *pEr, int idx);
int EvrUnivDlyEnable(volatile struct MrfErRegs *pEr, int dlymod, int enable);
int EvrUnivDlySetDelay(volatile struct MrfErRegs *pEr, int dlymod, u32 dly0, u32 dly1);
void EvrDumpHex(volatile struct MrfErRegs *pEr);
int EvrSetFracDiv(volatile struct MrfErRegs *pEr, int fracdiv);
int EvrGetFracDiv(volatile struct MrfErRegs *pEr);
int EvrSetDBufMode(volatile struct MrfErRegs *pEr, int enable);
int EvrGetDBufStatus(volatile struct MrfErRegs *pEr);
int EvrReceiveDBuf(volatile struct MrfErRegs *pEr, int enable);
int EvrGetDBuf(volatile struct MrfErRegs *pEr, char *dbuf, int size);
int EvrSetTimestampDivider(volatile struct MrfErRegs *pEr, int div);
int EvrGetTimestampCounter(volatile struct MrfErRegs *pEr);
int EvrGetSecondsCounter(volatile struct MrfErRegs *pEr);
int EvrGetTimestampLatch(volatile struct MrfErRegs *pEr);
int EvrGetSecondsLatch(volatile struct MrfErRegs *pEr);
int EvrSetTimestampDBus(volatile struct MrfErRegs *pEr, int enable);
int EvrSetPrescaler(volatile struct MrfErRegs *pEr, int presc, u32 div);
int EvrSetExtEvent(volatile struct MrfErRegs *pEr, int ttlin, int code, int enable);
int EvrSetBackEvent(volatile struct MrfErRegs *pEr, int ttlin, int code, int enable);
int EvrSetBackDBus(volatile struct MrfErRegs *pEr, int ttlin, int dbus);
int EvrSetTxDBufMode(volatile struct MrfErRegs *pEr, int enable);
int EvrGetTxDBufStatus(volatile struct MrfErRegs *pEr);
int EvrSendTxDBuf(volatile struct MrfErRegs *pEr, char *dbuf, int size);
int EvrGetFormFactor(volatile struct MrfErRegs *pEr);
