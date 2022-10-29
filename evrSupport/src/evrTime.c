/*=============================================================================
 
  Name: evrTime.c
           evrTimeInit       - Fiducial Processing Initialization
           evrTime           - Fiducial Processing (360Hz)
           evrTimeProcInit   - Fiducial Record Processing Initialization
           evrTimeProc       - Fiducial Record Processing  (360Hz)
           evrTimeDiag       - Fiducial Diagnostics (0.5Hz)
           evrTimeRate       - Rate Calculation for an Event (0.5Hz)
           evrTimeCount      - Update Rate Counter for an Event (ISR)
           evrTimeEvent      - Update Timestamp for an Event (Event Rate)
           evrTimeGetFromPipeline - Get Timestamp from Pipeline
           evrTimeGetFromEdef     - Get Timestamp from EDEF
           evrTimeGetFromEdefTime - Get Timestamp from EDEF and timestamp
           evrTimeGet        - Get Timestamp for an Event
           evrTimePutPulseID - Encode Pulse ID into a Timestamp
           evrTimeGetSystem  - Get System Time with Encoded Invalid Pulse ID
           evrTimePatternPutStart - Start New Time/Pattern Update
           evrTimePatternPutEnd   - End   New Time/Pattern Update

  Abs: This file contains all support for evr time processing
       records.
       
  Rem: All functions called by a subroutine record get passed one argument:

         psub                       Pointer to the subroutine record data.
          Use:  pointer
          Type: struct longSubRecord *
          Acc:  read/write
          Mech: reference

         All functions return a long integer.  0 = OK, -1 = ERROR.
         The subroutine record ignores the status returned by the Init
         routines.  For the calculation routines, the record status (STAT) is
         set to SOFT_ALARM (unless it is already set to LINK_ALARM due to
         severity maximization) and the severity (SEVR) is set to psub->brsv
         (BRSV - set by the user in the database though it is expected to
          be invalid).

  Auth:  
  Rev:  
-----------------------------------------------------------------------------*/

#include "copyright_SLAC.h" /* SLAC copyright comments */
 
/*-----------------------------------------------------------------------------
 
  Mod:  (newest to oldest)  
 
=============================================================================*/

/* c includes */

#include <string.h>           /* for memset                */
#include <sys/time.h>
#include <inttypes.h>
#include <stdint.h>
#include <math.h>
#include "subRecord.h"        /* for struct subRecord      */
#include "longSubRecord.h"    /* for struct longSubRecord  */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "iocsh.h"            /* for iocshRegister         */
#include "epicsTime.h"        /* epicsTimeStamp and protos */
#include "epicsStdio.h"       /* EPICS stdio redirection support */
#include "epicsGeneralTime.h" /* generalTimeTpRegister     */
#include "generalTimeSup.h"
#include "epicsMutex.h"       /* epicsMutexId and protos   */
#include "alarm.h"            /* INVALID_ALARM             */
#include "dbScan.h"           /* for post_event            */
#include <aSubRecord.h>
#include <recGbl.h>
#include <stdlib.h>

#include "mrfCommon.h"        /* MRF_NUM_EVENTS */    
#include "evrMessage.h"       /* EVR_MAX_INT    */    
#include "evrTime.h"       
#include "evrPattern.h"        
#include <epicsVersion.h>
#include <time.h>

#ifdef DIAG_TIMER
#include "HiResTime.h"
#else
#include "HiResTimeStub.h"
#endif	/* DIAG_TIMER */
#include "timingFifoApi.h"

#ifndef epicsTimeERROR
/*
 * EPICS 3.16 drops support for epicsTimeERROR in favor of S_time_* error codes,
 * but those are all postive values > 34M and evrTime API
 * specifies -1 function returns on errors.
 * */
#define epicsTimeERROR (-1)
#endif

#ifndef epicsTimeERROR
/*
 * EPICS 3.16 drops support for epicsTimeERROR in favor of S_time_* error codes,
 * but those are all postive values > 34M and evrTime API
 * specifies -1 function returns on errors.
 * */
#define epicsTimeERROR (-1)
#endif

#define  EVR_TIME_OK 0
#define  EVR_TIME_INVALID 1

#define MAX_FID_QUEUE          50              /* # fiducial hiRes timestamps in ISR queue */

/* Pattern and timestamp table */
typedef struct {
  evrMessagePattern_ts   pattern_s;
  unsigned long          timeslot;
  unsigned long          patternStatus;
  int                    timeStatus; /* 0=OK; -1=invalid                 */
} evrTimePattern_ts;

/* EVR Timestamp table */
typedef struct {
  epicsTimeStamp      time;   /* epics timestamp:                        */
                              /* 1st 32 bits = # of seconds since 1990   */
                              /* 2nd 32 bits = # of nsecs since last sec */
                              /*           except lower 17 bits = pulsid */
  int                 status; /* 0=OK; -1=invalid                        */
  EventTimingData     fifoInfo[MAX_TS_QUEUE];
  long long int       fifo_tsc_nom[MAX_TS_QUEUE];
  unsigned long long  ts_idx;
  int                 count;         /* # times this event has happened  */

	/* Diagnostics */
  int                 dbgcnt;
  long long			  fifoDeltaMin;
  long long			  fifoDeltaMax;
  int				  nCntEarly;
  int				  nCntLate;
  int				  nCntLateMin;
  int				  nCntLateMax;
  int				  nCntOnTime;
  int				  nCurFidBad;
  int				  nFidCorrected;
  int				  nFidQBad;
  int				  nFidQEarly;
  int				  nFidQLate;
  int				  nSetFidInvalid;
  int				  nFidQLateMin;
  int				  nFidQLateMax;
  int				  nFidQOnTime;
  int				  nTimeStampOK;
  int				  nFidQCountGT1;
  int				  nTimeStampFailed;
  t_HiResTime		  fidqtsc[ MAX_FID_QUEUE ];
  int                 fidq[    MAX_FID_QUEUE ];
  int                 fidR;
  int                 fidW;
  long long           cum_isr_latency;				/* units are tsc */
  long long           num_isr_latency_measurements;
  long long           max_isr_latency;				/* units are tsc */
} evrTime_ts;

/* EDEF Timestamp table */
typedef struct {
  epicsTimeStamp      time;   /* EDEF epics timestamp:                   */
                              /* 1st 32 bits = # of seconds since 1990   */
                              /* 2nd 32 bits = # of nsecs since last sec */
                              /*           except lower 17 bits = pulsid */
  epicsTimeStamp      timeInit;/* Time of EDEF initialization            */
  int                 avgdone;/* BSA averaging is done for this pulse    */
  epicsEnum16         sevr;   /* Severity at or above which data is not
                                 used in Beam Synchronous Acquisition    */
} evrTimeEdef_ts;

#ifdef BSA_DEBUG
int bsa_debug_mask  = 0x0; /* BSA debugging mask */
int bsa_debug_level = 0;
#endif

int evrTimeEventVerbose = 0;

/* evr_aps easy access macros */
#define EVR_APS_STATUS(i)       ( evr_aps[i]->timeStatus )
#define EVR_APS_TIME(i)         ( evr_aps[i]->pattern_s.time )
#define EVR_APS_NSEC(i)         ( evr_aps[i]->pattern_s.time.nsec )
#define EVR_APS_PULSEID(i)      ( evr_aps[i]->pattern_s.time.nsec & 0x1FFFF )
#define EVR_APS_SECPASTEPOCH(i) ( evr_aps[i]->pattern_s.time.secPastEpoch )

/* Pattern and timestamp pipeline */
static evrTimePattern_ts   evr_as[MAX_EVR_TIME+1];
static evrTimePattern_ts  *evr_aps[MAX_EVR_TIME+1];
extern unsigned long evrFiducialTime;
unsigned long evrActiveFiducialTime = 0;

/* Event definition (BSA) patterns */
static evrTimeEdef_ts      edef_as[EDEF_MAX][MAX_EDEF_TIME];
static unsigned int        edef_idx[EDEF_MAX];     /* The next write location! */

/* Event code timestamps */
static evrTime_ts          eventCodeTime_as[MRF_NUM_EVENTS+1];
/* EVR Time Timestamp RWMutex */
static epicsMutexId        evrTimeRWMutex_ps = 0;

static unsigned long msgCount         = 0; /* # fiducials processed since boot/reset */ 
static unsigned long msgRolloverCount = 0; /* # time msgCount reached EVR_MAX_INT    */ 
static unsigned long samePulseCount   = 0; /* # same pulses                          */
static unsigned long skipPulseCount   = 0; /* # skipped pulses                       */
static unsigned long pulseErrCount    = 0; /* # invalid pulses                       */
static unsigned long fiducialStatus   = EVR_TIME_INVALID;
/* Each IOC will only process records on 2 of 6 time slots.  Default to 1 and 4.  */
/* Other valid combination are 2 and 5 and 3 and 6.                               */
static unsigned long firstTimeSlot    = 1;
static unsigned long secondTimeSlot   = 4;
static unsigned long activeTimeSlot   = 0; /* 1=time slot active on the current pulse*/
static epicsTimeStamp mod720time;


/*=============================================================================

  Name: evrTimeGetSystem

  Abs:  Returns system time and status of call
  
  Ret:  -1=Failed; 0 = Success
  
==============================================================================*/

static int evrTimeGetSystem (epicsTimeStamp  *epicsTime_ps, evrTimeId_te id)
{
  if ( epicsTimeGetCurrent (epicsTime_ps) ) return epicsTimeERROR;
  /* Set pulse ID to invalid */
  evrTimePutPulseID(epicsTime_ps, PULSEID_INVALID);

  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeGetFromPipeline

  Abs:  Get the evr epics timestamp from the pipeline, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid

        Optionally get the pattern.
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsTimeStamp * epicsTime_ps     Write    Timestamp of this pattern
        evrTimeId_te        id            Read     Timestamp pipeline index;
                                          0=time associated w this pulse
                                          1,2,3 = time associated w next pulses
                                          4 = last active pulse
        evrModifier_ta  modifier_a        Write    First 6 longwords of the pattern
        epicsUInt32   * patternStatus_p   Write    Pattern Status (see evrPattern.h)
        epicsUInt32   * edefAvgDoneMask_p Write    EDEF average-done mask
        epicsUInt32   * edefMinorMask_p   Write    EDEF minor severity mask
        epicsUInt32   * edefMajorMask_p   Write    EDEF major severity mask

  Rem:  Routine to get the epics timestamp and pattern from the evr timestamp
        table that is populated from incoming broadcast from EVG.
        All outputs are set to zero

  Side: None.

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFromPipeline(epicsTimeStamp  *epicsTime_ps,
                           evrTimeId_te     id,
                           evrModifier_ta   modifier_a, 
                           epicsUInt32  *   patternStatus_p,
                           epicsUInt32  *   edefAvgDoneMask_p,
                           epicsUInt32  *   edefMinorMask_p,
                           epicsUInt32  *   edefMajorMask_p)
{
  evrTimePattern_ts *evr_ps;
  int status;
  int idx;

  /* Set all outputs to zero if there is any problem locking the pipeline. */
  if ((id > evrTimeActive) || (!evrTimeRWMutex_ps) ||
      epicsMutexLock(evrTimeRWMutex_ps)) {
    if (epicsTime_ps) {
      epicsTime_ps->secPastEpoch = 0;
      epicsTime_ps->nsec         = 0;
    }
    if (modifier_a) {
      for (idx = 0; idx < MAX_EVR_MODIFIER; idx++) modifier_a[idx] = 0;
      if (patternStatus_p  ) *patternStatus_p   = 0;
      if (edefAvgDoneMask_p) *edefAvgDoneMask_p = 0;
      if (edefMinorMask_p  ) *edefMinorMask_p   = 0;
      if (edefMajorMask_p  ) *edefMajorMask_p   = 0;
    }
    return epicsTimeERROR;
  }
  /* Read requested time index */
  evr_ps = evr_aps[id];
  if (evr_ps->timeStatus) status = evr_ps->timeStatus;
  else                    status = fiducialStatus;
  if (epicsTime_ps) *epicsTime_ps = evr_ps->pattern_s.time;
  if (modifier_a) {
    for (idx = 0; idx < MAX_EVR_MODIFIER; idx++)
      modifier_a[idx] = evr_ps->pattern_s.modifier_a[idx];
    if (patternStatus_p  ) *patternStatus_p   = evr_ps->patternStatus;
    if (edefAvgDoneMask_p) *edefAvgDoneMask_p = evr_ps->pattern_s.edefAvgDoneMask;
    if (edefMinorMask_p  ) *edefMinorMask_p   = evr_ps->pattern_s.edefMinorMask;
    if (edefMajorMask_p  ) *edefMajorMask_p   = evr_ps->pattern_s.edefMajorMask;
  }
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return status;
}

/*=============================================================================

  Name: evrTimeGetFromEdefTime

  Abs:  Get the evr epics timestamp from EDEF, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        unsigned int        edefIdx       Read     EDEF Index (0 to EDEF_MAX-1)
        epicsTimeStamp *    edefTime_ps   Read     EDEF active timestamp to match
        epicsTimeStamp *    edefTimeInit_ps Write  EDEF init   timestamp
        int *               edefAvgDone_p Write    EDEF average-done flag
        epicsEnum16  *      edefSevr_p    Write    EDEF severity

  Rem:  Routine to get the epics timestamp and flags from the EDEF timestamp
        table that is populated from incoming broadcast from EVG.

        This is used when we are processing data that potentially has an old
        timestamp.  We search backwards until we find an edef that hopefully
        matches but at least predates the data.

  Side: None

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFromEdefTime(unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p,
                           unsigned int    *edefGen_p)
{
  unsigned int first;
  int idx, i; 
  evrTimeEdef_ts *edef;

  if ((edefIdx >= EDEF_MAX) || (!evrTimeRWMutex_ps)) return epicsTimeERROR;
  /* if the r/w mutex is valid, and we can lock with it, read requested time index */
  if (epicsMutexLock(evrTimeRWMutex_ps)) return epicsTimeERROR;

  first = edef_idx[edefIdx] - 1;      /* Back one == last written! */
  idx = first & MAX_EDEF_TIME_MASK;
  edef = &edef_as[edefIdx][idx];

  for (i = 0; i < MAX_EDEF_TIME - 2; i++) {
      if ((edef->time.secPastEpoch == edefTime_ps->secPastEpoch) &&
          (edef->time.nsec == edefTime_ps->nsec)) {
          *edefTimeInit_ps = edef->timeInit;
          *edefAvgDone_p   = edef->avgdone;
          *edefSevr_p      = edef->sevr;
          *edefGen_p       = first - i;
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 2)
              printf("%08x:%08x BSA%d, slot %d.\n", edefTime_ps->secPastEpoch, edefTime_ps->nsec, edefIdx, idx);
#endif
          return epicsTimeOK;
      } else if ((edef->time.secPastEpoch < edefTime_ps->secPastEpoch) ||
                 ((edef->time.secPastEpoch == edefTime_ps->secPastEpoch) &&
                  (edef->time.nsec < edefTime_ps->nsec))) {
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 3) {
              int idx2 = (edef_idx[edefIdx] - 1) & MAX_EDEF_TIME_MASK; /* Back one == last written! */
              evrTimeEdef_ts *edef2 = &edef_as[edefIdx][idx2];

              printf("%08x:%08x BSA%d, not found, slot %d %08x:%08x, current %d %08x:%08x.\n", edefTime_ps->secPastEpoch,
                     edefTime_ps->nsec, edefIdx,
                     idx, edef->time.secPastEpoch, edef->time.nsec,
                     idx2, edef2->time.secPastEpoch, edef2->time.nsec);
          }
#endif
          return epicsTimeERROR; /* No match! */
      } else {
          idx = (idx - 1) & MAX_EDEF_TIME_MASK;
          edef = &edef_as[edefIdx][idx];
      }
  }
  epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
  if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 2)
      printf("%08x:%08x BSA%d not found.\n", edefTime_ps->secPastEpoch, edefTime_ps->nsec, edefIdx);
#endif
  return epicsTimeERROR; /* No match! */
}


/*=============================================================================

  Name: evrTimeGetInitGen

  Abs:  Search backwards to find the first index in this acquisition.  Counting
        back, we either find the init time or a zero time.  In the latter case,
        we go forward one to find the first time.

  Args: edefIdx, an edef identifier.
        edefGen  an index pointing to an observation in a new acquisition epoch.
  

  Return: An index for the first value in the new acquisition epoch.

==============================================================================*/
unsigned int evrTimeGetInitGen(unsigned int edefIdx, unsigned int edefGen)
{
  epicsTimeStamp *itime;
  evrTimeEdef_ts *edef;
  unsigned int gen = edefGen;
  int i;

  if ((edefIdx >= EDEF_MAX) || (!evrTimeRWMutex_ps)) return epicsTimeERROR;
  if (epicsMutexLock(evrTimeRWMutex_ps)) return epicsTimeERROR;
  itime = &edef_as[edefIdx][gen & MAX_EDEF_TIME_MASK].timeInit;
#ifdef BSA_DEBUG
  if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 1)
      printf("eTGIG: looking for %08x:%08x (index %d)\n", itime->secPastEpoch, itime->nsec, gen);
#endif
  for (i = 0; i < MAX_EDEF_TIME - 2; i++) {
      edef = &edef_as[edefIdx][gen & MAX_EDEF_TIME_MASK];
      if (edef->time.nsec == itime->nsec &&
          edef->time.secPastEpoch == itime->secPastEpoch) {
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 1)
              printf("eTGIG: found match (index %d)\n", gen);
#endif
          return gen;
      }
      if (edef->time.nsec == 0 && edef->time.secPastEpoch == 0) {
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 1)
              printf("eTGIG: found zero time (index %d) --> return %d\n", gen, gen+1);
#endif
          return gen + 1;
      }
      gen--;
  }
  /*
   * Couldn't find it?  Let's just say now then!  This will screw up any
   * BSA in progress while we're starting up, but it will be OK for the
   * BSA that never really start (BR, 1H, etc.)
   */
  epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
  if (((1 << edefIdx) & bsa_debug_mask) && bsa_debug_level >= 1)
      printf("eTGIG: no match, returning %d\n", edefGen);
#endif
  return edefGen;
}


/*=============================================================================

  Name: evrTimeGet

  Abs:  Get the epics timestamp associated with an event code, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
        
  Args: Type     Name           Access     Description
        -------  -------    ---------- ----------------------------
  epicsTimeStamp * epicsTime_ps write  ptr to epics timestamp to be returned
  unsigned int   eventCode      read   Event code 0 to 255.
                                      0,1=time associated w this pulse
                                          (event code 1 = fiducial)
                                          1 to 255 = EVR event codes

  Rem:  Routine to get the epics timestamp from the event code timestamp table
        that is populated by the EVR event IRQ handler.  If the IRQ is off or
        the event code has not been sent for a while, the timestamp will be
        very old.

  Side: 

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGet (epicsTimeStamp  *epicsTime_ps, unsigned int eventCode)
{
  int status;

  /* Hack event code to get pre-bundled general-time behavior */
  if ( (unsigned int)epicsTimeEventBestTime == eventCode )
    eventCode = 0;
 
  /* Check for valid event code and r/w mutex */
  if ((eventCode > MRF_NUM_EVENTS) || (!evrTimeRWMutex_ps)) {
    return epicsTimeERROR;
  }

  if ( epicsMutexLock(evrTimeRWMutex_ps) )
    return epicsTimeERROR;

  *epicsTime_ps = eventCodeTime_as[eventCode].time;
  status        = eventCodeTime_as[eventCode].status;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return status; 
}

/*=============================================================================

  Name: timingGetCurTimeStamp

  Abs:  Retrieve the most recent timestamp available
        
  Ret:  -1=Failed; 0 = Success
  		Most recent timestamp via pTimeStampDest
  
==============================================================================*/ 
int timingGetCurTimeStamp(	epicsTimeStamp  *   pTimeStampDest )
{
#if 1
	return evrTimeGet ( pTimeStampDest, EVENT_FIDUCIAL );
#else
	int						status	= 0;
	unsigned long long		idx		=	0LL;
	EventTimingData			fifoInfo;

	if ( pTimeStampDest == NULL )
		return -1;

	status = timingFifoRead(	EVENT_FIDUCIAL, TS_INDEX_INIT, &idx, &fifoInfo );
	if ( status < 0 )
	{
		pTimeStampDest->secPastEpoch = 0;
		pTimeStampDest->nsec		 = PULSEID_INVALID;
	}
	else
	{
		pTimeStampDest->secPastEpoch = fifoInfo.fifo_time.secPastEpoch;
		pTimeStampDest->nsec		 = fifoInfo.fifo_time.nsec;
	}

	return status;
#endif
}

/*=============================================================================

  Name: timingGetEventTimeStamp

  Abs:  Retrieve the most recent timestamp and pulseId for the specified eventCode
        
  Ret:  -1=Failed; 0 = Success
  		Most recent timestamp via pTimeStampDest
  
==============================================================================*/ 
int timingGetEventTimeStamp(    epicsTimeStamp  *   pTimeStampDest,
                                int        eventCode   	)
{
#if 1
	return evrTimeGet ( pTimeStampDest, eventCode );
#else
	int						status	= 0;
	unsigned long long		idx		=	0LL;
	EventTimingData			fifoInfo;

	if ( pTimeStampDest == NULL )
		return -1;

	status = timingFifoRead(	eventCode, TS_INDEX_INIT, &idx, &fifoInfo );
	if ( status < 0 )
	{
		pTimeStampDest->secPastEpoch = 0;
		pTimeStampDest->nsec		 = PULSEID_INVALID;
	}
	else
	{
		pTimeStampDest->secPastEpoch = fifoInfo.fifo_time.secPastEpoch;
		pTimeStampDest->nsec		 = fifoInfo.fifo_time.nsec;
	}

	return status;
#endif
}

/*=============================================================================

  Name: timingFifoRead

  Abs:  Get the timing associated with an event code from the fifo, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
        
  Args: Type     Name       Access     Description
        -------  -------    ---------- ----------------------------
  unsigned int   eventCode     read    Event code 0 to 255.
  int            incr          read    How far to move ahead (TS_INDEX_INIT if idx 
                                       is uninitialized).
  unsigned long long *idx   read/write The last fifo index we read.

  Rem:  Routine to get the epics timestamp from a queue of timestamps.  This must
        be called no faster than timestamps come in, as there is no checking for 
        bounds.

  Side: 

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int timingFifoRead(unsigned int            eventCode,
                   int                     incr,
                   uint64_t               *idx,
                   EventTimingData        *pFifoInfoRet   )
{
	if (	(pFifoInfoRet == NULL)
		||	(eventCode > MRF_NUM_EVENTS)
		||	(!evrTimeRWMutex_ps) )
		return epicsTimeERROR;

    pFifoInfoRet->fifo_time.secPastEpoch = 0;
    pFifoInfoRet->fifo_time.nsec         = 0;
    pFifoInfoRet->fifo_tsc               = 0LL;
    pFifoInfoRet->fifo_fid               = TIMING_PULSEID_INVALID;

	if ( epicsMutexLock( evrTimeRWMutex_ps ) != 0 )
		return epicsTimeERROR;

	if (incr == TS_INDEX_INIT)
		if (eventCodeTime_as[eventCode].ts_idx == 0)
			*idx = 0LL;
		else
			*idx = eventCodeTime_as[eventCode].ts_idx - 1;
	else
		*idx += incr;

	/*
	 * eventCodeTime_as[eventCode].ts_idx is the monotonically increasing location for the *next*
	 * timestamp... it isn't valid yet!  Since there are MAX_TS_QUEUE entries, this means that
	 * the valid entries are between eventCodeTime_as[eventCode].ts_idx - MAX_TS_QUEUE and 
	 * eventCodeTime_as[eventCode].ts_idx - 1.
	 */
	if ( *idx + MAX_TS_QUEUE < eventCodeTime_as[eventCode].ts_idx ||
		 *idx >= eventCodeTime_as[eventCode].ts_idx) {
		epicsMutexUnlock(evrTimeRWMutex_ps);
		/* FIFO is empty */
		if (evrTimeEventVerbose)
		{
			printf( "timingFifoRead: Empty Fifo!" );
			if ( *idx + MAX_TS_QUEUE < eventCodeTime_as[eventCode].ts_idx )
				printf( " *idx+MAX_TS_QUEUE=%lu < ts_idx=%llu\n", *idx + MAX_TS_QUEUE, eventCodeTime_as[eventCode].ts_idx );
		 	if ( *idx >= eventCodeTime_as[eventCode].ts_idx)
				printf( " *idx=%lu >= ts_idx=%llu\n", *idx, eventCodeTime_as[eventCode].ts_idx );
		}
		return epicsTimeERROR;
	}

	/* Copy the requested fifo information */
	*pFifoInfoRet	= eventCodeTime_as[eventCode].fifoInfo[*idx & MAX_TS_QUEUE_MASK];

	epicsMutexUnlock(evrTimeRWMutex_ps);
	return 0; 
}

/*=============================================================================

  Name: evrTimePutPulseID

  Abs:  Puts pulse ID in the lower 17 bits of the nsec field
        of the epics timestamp.
        
  Ret:  0 = Success
  
==============================================================================*/ 
int evrTimePutPulseID (epicsTimeStamp  *epicsTime_ps, unsigned int pulseID)
{
  epicsTime_ps->nsec &= UPPER_15_BIT_MASK;
  epicsTime_ps->nsec |= pulseID;
  if (epicsTime_ps->nsec >= NSEC_PER_SEC) {
    epicsTime_ps->secPastEpoch++;
    epicsTime_ps->nsec -= NSEC_PER_SEC;
    epicsTime_ps->nsec &= UPPER_15_BIT_MASK;
    epicsTime_ps->nsec |= pulseID;
  }
  return epicsTimeOK;
}

/*===============================================
  Wrapper function for generalTime: Temporal
=================================================*/
static int evrTimeGet_gtWrapper(epicsTimeStamp *epicsTime_ps, int eventCode)
{
    return evrTimeGet(epicsTime_ps, (unsigned int)eventCode);
}

static int evrTimeGetSystem_gtWrapper(epicsTimeStamp *epicsTime_ps, int eventCode)
{
    return evrTimeGetSystem(epicsTime_ps, evrTimeCurrent);
}




/*=============================================================================

  Name: evrTimeInit

  Abs:  Creates the evrTimeRWMutex_ps read/write mutex 
        and initializes the timestamp arrays.
    The evr timestamp table is initialized to system time, invalid status,
        and invalid pattern. During processing, if a timestamp status goes
    invalid, the time is overwritten to the last good evr timestamp.
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsInt32      firstTimeSlotIn Read       1st timeslot for this IOC (0,1,2,3)
                                                   (0 = don't use 1st timeslot)
        epicsInt32     secondTimeSlotIn Read       2nd timeslot for this IOC (0,4,5,6)
                                                   (0 = don't use 2nd timeslot)

  Side: EVR Time Timestamp table
  pulse pipeline n  , status - 0=OK; -1=invalid
  0 = current pulse (Pn), status
  1 = next (upcoming) pulse (Pn-1), status
  2 = two pulses in the future (Pn-2), status
  3 = three pulses in the future (Pn-3), status

  Status is invalid when
  1) Bootup - System time is entered (as opposed to evr timestamp).
  2) the PULSEID of most recent index is is the same as the previous index.
  3) pattern waveform record invalid - timestamp is last good time
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 
int evrTimeInit(epicsInt32 firstTimeSlotIn, epicsInt32 secondTimeSlotIn)
{
  int idx, idx2;
  int timeslotDiff;

  if ((firstTimeSlotIn  >= 0) && (secondTimeSlotIn >= 0) &&
      (firstTimeSlotIn  <= TIMESLOT_MAX) &&
      (secondTimeSlotIn <= TIMESLOT_MAX) &&
      ((firstTimeSlotIn != 0) || (secondTimeSlotIn != 0))) {
    timeslotDiff = firstTimeSlotIn - secondTimeSlotIn;
    if ((firstTimeSlotIn == 0) || (secondTimeSlotIn == 0) ||
        (timeslotDiff ==  TIMESLOT_DIFF) ||
        (timeslotDiff == -TIMESLOT_DIFF)) {
      firstTimeSlot = firstTimeSlotIn;
      secondTimeSlot = secondTimeSlotIn;
    }
  }
  /* create read/write mutex around evr timestamp table array */
  if (!evrTimeRWMutex_ps) {
        if (evrTimeGetSystem(&mod720time, evrTimeCurrent))
          return epicsTimeERROR;
    /* init patterns in pipeline */
        for (idx=0; idx<MAX_EVR_TIME+1; idx++) {
          memset(&evr_as[idx].pattern_s, 0, sizeof(evrMessagePattern_ts));
          evr_as[idx].pattern_s.time = mod720time;
          evr_as[idx].timeStatus     = epicsTimeERROR;
          evr_aps[idx] = &evr_as[idx];
        }
        /* Init EDEF pattern array */
        for (idx=0; idx<EDEF_MAX; idx++) {
          memset(&edef_as[idx], 0, MAX_EDEF_TIME * sizeof(evrTimeEdef_ts));
          for (idx2=0; idx2<MAX_EDEF_TIME; idx2++) {
              edef_as[idx][idx2].timeInit = mod720time;
              edef_as[idx][idx2].sevr = INVALID_ALARM;
          }
        }
        /* init timestamp structures to invalid status & system time*/
        for (idx=0; idx<=MRF_NUM_EVENTS; idx++) {
    	  evrTime_ts  *   pevrTime = &eventCodeTime_as[idx];
          int idx2;    
		  /* t_HiResTime			hiResTsc	= GetHiResTicks(); */
          for (idx2 = 0; idx2 < MAX_TS_QUEUE; idx2++) {
              pevrTime->fifoInfo[idx2].fifo_time   = mod720time;
              pevrTime->fifoInfo[idx2].fifo_fid    = TIMING_PULSEID_INVALID;
              pevrTime->fifoInfo[idx2].fifo_tsc    = 0LL;
              pevrTime->fifo_tsc_nom[idx2]         = 0LL;
          }
          pevrTime->ts_idx  = 0LL;
          pevrTime->time    = mod720time;
          pevrTime->status  = epicsTimeERROR;
          pevrTime->count   = 0;

          pevrTime->dbgcnt  = 0;
          pevrTime->fifoDeltaMin  = 0LL;
          pevrTime->fifoDeltaMax  = 0LL;
          pevrTime->nCntEarly   = 0;
          pevrTime->nCntLate   = 0;
          pevrTime->nCntLateMin  = EVR_MAX_INT;
          pevrTime->nCntLateMax  = 0;
          pevrTime->nCntOnTime  = 0;
		  pevrTime->nCurFidBad	= 0;
		  pevrTime->nFidCorrected	= 0;
          pevrTime->nFidQEarly   = 0;
          pevrTime->nFidQLate   = 0;
          pevrTime->nFidQBad   = 0;
          pevrTime->nSetFidInvalid   = 0;
		  pevrTime->nFidQLateMin   = EVR_MAX_INT;
		  pevrTime->nFidQLateMax   = 0;
          pevrTime->nFidQCountGT1   = 0;
          pevrTime->nFidQOnTime   = 0;
		  pevrTime->nTimeStampOK		= 0;
  		  pevrTime->nTimeStampFailed	= 0;
          pevrTime->fidR    = 0;
          pevrTime->fidW    = 0;
        }

        /* For IOCs that support iocClock (RTEMS and vxWorks), register
           evrTimeGet with generalTime so it is used by epicsTimeGetEvent */
        if(generalTimeRegisterEventProvider("evrTimeGet", 1000, (TIMEEVENTFUN) evrTimeGet_gtWrapper))
          return epicsTimeERROR;

        if(generalTimeRegisterEventProvider("evrTimeGetSystem", 2000, (TIMEEVENTFUN) evrTimeGetSystem_gtWrapper))
          return epicsTimeERROR;
        evrTimeRWMutex_ps = epicsMutexCreate();
        if (!evrTimeRWMutex_ps)
            return epicsTimeERROR;
  }
  return epicsTimeOK;
}

/* For IOCs that don't support iocClock (linux), supply a dummy
   iocClockRegister to keep the linker happy. */
#ifndef EVR_DRIVER_SUPPORT
void iocClockRegister(TIMECURRENTFUN getCurrent,
                      TIMEEVENTFUN   getEvent) 
{
}
#endif

/*=============================================================================

  Name: evrTime

  Abs:  Processes every time the fiducial event code is received @ 360 Hz.
        Performs evr error checking, and then advances 
        the timestamp/pattern table in the evrTime_as array.
 
  Error Checking:

  Pulse ID error (any PULSEID of 0 or non-consecutive PULSEIDs) - 
    Set appropriate counters. Set error flag.
  Set EVR timestamp status to invalid if PULSEIDs are not changing.
  Error advancing EVR timestamps - set error flag used later to disable 
    triggers (EVR) or event codes (EVG). 
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsUInt32         mpsModifier read       MPS pattern modifier

  Rem:  

  Side: Upon entry, pattern is:
        n P0   don't care about n, this was acted upon for beam occuring last 
           fiducial
        n-1 P1 this is the pattern for this fiducial.
        n-2 P2
        n-3 P3
    Algorithm: 
    During proper operation, every pulse in the pattern should be consecutive;
    differing by "1".  Error until all three pulseids are consecutive.
    Upon error, set error flag that will disable events/triggers.       

  Ret:  -1=Failed; 0 = Success
   
==============================================================================*/
int evrTime(epicsUInt32 mpsModifier)
{
  int idx;
  evrTimePattern_ts *evr_ps;
  epicsInt32  pulseid, pulseidNext1, pulseidNext2, pulseidDiff;
  unsigned long timeslot;
  unsigned long edefMask;

  /* Keep a count of fiducials and reset before overflow */
  if (msgCount < EVR_MAX_INT) {
    msgCount++;
  } else {
    msgRolloverCount++;
    msgCount = 0;
  }
  if (evrTimeRWMutex_ps && (!epicsMutexLock(evrTimeRWMutex_ps))) {
    fiducialStatus = EVR_TIME_OK;
    /* Advance the evr pattern in the pipeline.  Update MPS
       information (which is not pipelined) into the pattern. */
    evr_ps = evr_aps[evrTimeCurrent];
    for (idx=0;idx<evrTimeNext3;idx++) {
      evr_aps[idx] = evr_aps[idx+1];
      evr_aps[idx]->pattern_s.modifier_a[MOD6_IDX] = mpsModifier;
    }
    evr_aps[evrTimeNext3] = evr_ps;
    evr_aps[evrTimeNext3]->timeStatus = epicsTimeERROR;

    /* Update the EDEF array for any initialized or active EDEF -
       this array is used later by BSA processing.
       NOTE: Now, we are *always* looking ahead!
       Before, we were using evrTimeCurrent, not evrTimeNext2!
     */
    evr_ps = evr_aps[evrTimeNext2];
    if (evr_ps->pattern_s.edefInitMask ||
        evr_ps->pattern_s.modifier_a[MOD5_IDX] & MOD5_EDEF_MASK) {
      for (idx=0;idx<EDEF_MAX;idx++) {
        edefMask = 1 << idx;
        if (!(evr_ps->pattern_s.edefInitMask & edefMask) &&
            !(evr_ps->pattern_s.modifier_a[MOD5_IDX] & edefMask))
          continue;
        else {
          int idx2 = edef_idx[idx]++ & MAX_EDEF_TIME_MASK;
          evrTimeEdef_ts *edef = &edef_as[idx][idx2];
          *edef = edef_as[idx][(idx2 - 1) & MAX_EDEF_TIME_MASK]; /* Init from previous! */
          /* EDEF initialized? - check the newest mask so init done ASAP */
          if (evr_ps->pattern_s.edefInitMask & edefMask) {
            edef->timeInit = evr_ps->pattern_s.time;
            edef->time.secPastEpoch = 0;
            edef->time.nsec = 0;
#ifdef BSA_DEBUG
            if ((bsa_debug_mask & edefMask) && bsa_debug_level >= 2)
                printf("%08x:%08x EDEF%d slot %d, initialized.\n",
                       edef->timeInit.secPastEpoch, edef->timeInit.nsec, idx, idx2);
#endif            
          }
          /* EDEF active? - set time and flags used by BSA processing later */
          if (evr_ps->pattern_s.modifier_a[MOD5_IDX] & edefMask) {
            edef->time = evr_ps->pattern_s.time;
            if (evr_ps->pattern_s.edefAvgDoneMask & edefMask)
              edef->avgdone = 1;
            else
              edef->avgdone = 0;
            if (evr_ps->pattern_s.edefMinorMask & edefMask)
              edef->sevr = MINOR_ALARM;
            else if (evr_ps->pattern_s.edefMajorMask & edefMask)
              edef->sevr = MAJOR_ALARM;
            else
              edef->sevr = INVALID_ALARM;
          }
#ifdef BSA_DEBUG
          if ((bsa_debug_mask & edefMask) && bsa_debug_level >= 2)
              printf("%08x:%08x (%08x:%08x) EDEF%d slot %d, done=%d.\n",
                     edef->time.secPastEpoch, edef->time.nsec,
                     edef->timeInit.secPastEpoch, edef->timeInit.nsec,
                     idx, idx2, edef->avgdone);
#endif            
        }
      }
    }

    evr_ps = evr_aps[evrTimeCurrent];
    pulseidNext2 = PULSEID(evr_aps[evrTimeNext2]->pattern_s.time);
    pulseidNext1 = PULSEID(evr_aps[evrTimeNext1]->pattern_s.time);
    pulseid      = PULSEID(evr_aps[evrTimeCurrent]->pattern_s.time);
    timeslot     = evr_ps->timeslot;
    /* The 3 next pulses must be valid before all is considered OK to go */
    if (evr_aps[evrTimeCurrent]->patternStatus ||
        evr_aps[evrTimeNext1]->patternStatus   ||
        evr_aps[evrTimeNext2]->patternStatus) {
      fiducialStatus = EVR_TIME_INVALID;
      pulseErrCount++;
    /* Diff between first and second and second and third must be 1 */
    /* pulse ID may have rolled over */
    } else {
      pulseidDiff = pulseidNext2 - pulseidNext1;
      if ((pulseidDiff != 1) && (pulseidDiff != -PULSEID_MAX)) {
        fiducialStatus = EVR_TIME_INVALID;
        if (pulseidDiff==0)
          ++samePulseCount; /* same pulse coming into the pipeline */
        else
          ++skipPulseCount; /* skipped pulse (non-consecutive) in the pipeline */
      } else {
        pulseidDiff = pulseidNext1 - pulseid;
        if ((pulseidDiff != 1) && (pulseidDiff != -PULSEID_MAX)) {
          fiducialStatus = EVR_TIME_INVALID;
        }
      }
    }
    if ((timeslot == 0) ||
        (firstTimeSlot == timeslot) || (secondTimeSlot == timeslot)) {
      evr_as[evrTimeActive] = *evr_ps;
      evrActiveFiducialTime = evrFiducialTime;
      activeTimeSlot = 1;
    } else {
      activeTimeSlot = 0;
    }
    /* determine if the next 3 pulses are all the same. */
    /* Same pulses means the EVG is not sending timestamps and this forces   
       record timestamps to revert to system time */
    if ((pulseidNext2==pulseidNext1) && (pulseidNext2==pulseid)) {
      for (idx=0;idx<evrTimeNext3;idx++)
        evr_aps[idx]->timeStatus = epicsTimeERROR;
      eventCodeTime_as[0].status = epicsTimeERROR;
      eventCodeTime_as[EVENT_FIDUCIAL].status = epicsTimeERROR;
    } else {
      if (activeTimeSlot) {
        eventCodeTime_as[0].time   = evr_ps->pattern_s.time;
        eventCodeTime_as[0].status = evr_ps->timeStatus;
      }
      eventCodeTime_as[EVENT_FIDUCIAL].time   = evr_ps->pattern_s.time;
      eventCodeTime_as[EVENT_FIDUCIAL].status = evr_ps->timeStatus;
    }
    epicsMutexUnlock(evrTimeRWMutex_ps);
  /* If we cannot lock - bad problem somewhere. */
  } else {
    fiducialStatus = EVR_TIME_INVALID;
    eventCodeTime_as[0].status              = epicsTimeERROR;
    eventCodeTime_as[EVENT_FIDUCIAL].status = epicsTimeERROR;
    return epicsTimeERROR;
  }
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeProcInit

  Abs:  Initialization for the fiducial processing record.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Init Subroutine for IOC:LOCA:UNIT:FIDUCIAL

  Side: None.
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 
static int evrTimeProcInit(longSubRecord *psub)
{
  evrTimeInit((epicsInt32)psub->i, (epicsInt32)psub->j);
  /* Register this record for the start of fiducial processing */
  if (evrMessageRegister(EVR_MESSAGE_FIDUCIAL_NAME, 0, (dbCommon *)psub) < 0)
    return epicsTimeERROR;  
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeProc

  Abs:  Record processing every time the fiducial event code is received @ 360 Hz.
        Get data from storage.
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:FIDUCIAL

  Side: None.
        
  Sub Inputs/ Outputs:
   Input/Outputs:
    I - First  Active Time Slot for this IOC (1, 2, 3)
    J - Second Active Time Slot for this IOC (4, 5, 6)   
   Outputs:
    A - Fiducial High Resolution Clock Time (360hz)
    B - Active Time Slot High Resolution Clock Time (120hz)
    L - Enable/Disable flag for current pattern and timestamp update
    VAL = Error Flag

  Ret:  -1=Failed; 0 = Success
   
==============================================================================*/
static int evrTimeProc (longSubRecord *psub)
{
  psub->a = evrFiducialTime;
  psub->b = evrActiveFiducialTime;
  if (evrTimeRWMutex_ps && (!epicsMutexLock(evrTimeRWMutex_ps))) {
    psub->l   = activeTimeSlot?0:1;
    psub->val = fiducialStatus;
    /* See if user wants different time slots */
    if ((psub->i != firstTimeSlot) || (psub->j != secondTimeSlot)) {
      evrTimeInit((epicsInt32)psub->i, (epicsInt32)psub->j);
      psub->i = firstTimeSlot;
      psub->j = secondTimeSlot;
    }
    epicsMutexUnlock(evrTimeRWMutex_ps);
  } else {
    psub->l   = 0;
    psub->val = EVR_TIME_INVALID;
  }
  if (psub->val) return epicsTimeERROR;
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeDiag

  Abs:  Expose counters.
        This subroutine is for status only and should update at a low
        rate like 1Hz.
  
  Inputs:
  R - Counter Reset Flag

  Outputs:
  A  Fiducial Delay Time (us) - due to the epicsEventSignal
  B  Minimum of Fiducial Delay Time (us)
  C  Minimum of Fiducial Delay Time (us)
 
  D  Absolute Fiducial Delay (us) - based on the evr clock outer
  E  Minimum of Absolute Fiducial Delay (us)
  F  Maximum of Absolute Fiducial Delay (us) 

  G  Absolute start time for the data buffer handling (us)  
  H  Minimum of the start time for the data buffer handling (us)
  I  Maximum of the start time for the data buffer handling (us)
  
  J  Pended message for the evrEvent task
  K  Maximum number of pended message for the evrEventTask

  L Spare

  M  fiducial counter
  N  Number of times M has rolled over
  O  Number of same pulses
  P  Number of skipped pulses
  Q  Spare
  S  Number of invalid pulses
  T  Number of fiducial interrupts
  U  Number of times T has rolled over
  V  Minimum Fiducial Delta Start Time (us)
  W  Maximum Fiducial Delta Start Time (us)
  X  Average Fiducial Processing Time  (us)
  Y  Number of missed fiducials
  Z  Maximum Fiducial Processing Time  (us)
  VAL = Error flag from evrTime
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 

static long evrTimeDiag (longSubRecord *psub)
{
  epicsUInt32  dummy;
  
  psub->val = fiducialStatus;
  psub->m = msgCount;          /* # fiducials processed since boot/reset */
  psub->n = msgRolloverCount;  /* # time msgCount reached EVR_MAX_INT    */
  psub->o = samePulseCount;
  psub->p = skipPulseCount;
  psub->s = pulseErrCount;
  evrMessageCounts(EVR_MESSAGE_FIDUCIAL,
                   &psub->t,&psub->u,&dummy  ,&psub->y,&dummy,
                   &dummy,  &psub->v,&psub->w,&psub->x,&psub->z);
  evrMessageCountsFiducial(EVR_MESSAGE_FIDUCIAL,
                           &psub->a, &psub->b, &psub->c);

  evrMessageCountsClockCounter(EVR_MESSAGE_FIDUCIAL,
                               &psub->d, &psub->e, &psub->f);
  evrMessageCountsClockCounter(EVR_MESSAGE_PATTERN,
                               &psub->g, &psub->h, &psub->i);
  evrMessageCountsQ(EVR_MESSAGE_FIDUCIAL,
                    &psub->j, &psub->k);

  if (psub->r > 0) {
    psub->r           = 0;
    msgCount          = 0;
    msgRolloverCount  = 0;
    samePulseCount    = 0;
    skipPulseCount    = 0;
    pulseErrCount     = 0;
    evrMessageCountReset(EVR_MESSAGE_FIDUCIAL);
  }
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeRate

  Abs:  Calculate rate that an event code is received by the EVR ISR.
        It is assumed this subroutine processes at 0.5hz.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        subRecord *         psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:NAMERATE

  Inputs:
       E - Event Code
     
  Outputs:
       VAL = Rate in Hz
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
static long evrTimeRate(subRecord *psub)
{
  int eventCode = psub->e + 0.5;

  if ((eventCode > 0) && (eventCode <= MRF_NUM_EVENTS)) {
    if (evrTimeRWMutex_ps && (!epicsMutexLock(evrTimeRWMutex_ps))) {
      int   eventCodeCount = eventCodeTime_as[eventCode].count;
      psub->val = 0;
      /* Only compute count if event code is the same as last time */
      if ( psub->f == psub->e )
        psub->val   = eventCodeCount - psub->a;
      psub->f   = psub->e;
      psub->a   = eventCodeCount;
      epicsMutexUnlock(evrTimeRWMutex_ps);

      /* Check for integer rollover */
      if ( psub->val < 0 )
        psub->val += EVR_MAX_INT;

      /* Convert 0.5Hz count (MODULO 720 fiducials) to a rate in sec */
      psub->val /= MODULO720_SECS;
      return epicsTimeOK;
    }
  }
  psub->val = 0.0;
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimeCount

  Abs:  Increment a counter for an event code.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        unsigned int        eventCode    read       Event Code

  Rem:  This routine is currently called from interrupt level by evrEvent(),
  		which is called once for each event code interrupt.
        When evrEvent() is called from the ISR for event code 1, the fiducial,
		it signals a semaphore that the evrTask() thread is waiting on.
		After evrTask() calls evrPattern() to extract the pattern and
		evrTime() to advance the pipeline, it posts an eventMessage for event code 1
		to the eventTaskQueue.
		For other event codes, the eventMessage is posted directly from evrEvent in the interrupt context.
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimeCount(unsigned int eventCode, unsigned int fiducial, t_HiResTime hiResTsc )
{
  if ((eventCode > 0) && (eventCode <= MRF_NUM_EVENTS)) {
    evrTime_ts  *   pevrTime = &eventCodeTime_as[eventCode];
    /* Rollover if value gets too big */
    if (pevrTime->count < EVR_MAX_INT)
        pevrTime->count++;
    else
        pevrTime->count = 1;
	if( fiducial > PULSEID_MAX )
		fiducial = PULSEID_INVALID;
    pevrTime->fidq[		pevrTime->fidW ] = fiducial;
    pevrTime->fidqtsc[	pevrTime->fidW ] = hiResTsc;
    if (++pevrTime->fidW == MAX_FID_QUEUE)
        pevrTime->fidW = 0;
    return epicsTimeOK;
  }
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimeEvent

  Abs:  Update the event code timestamp and increment a diagnostic counter.

        WARNING: This longsub processing routine is called from the EPICS db
        processing queue via an FLNK from the event record I/O Scan.
        It is not reliable to handle event counts, timestamps, or status here
        as this routine has no guarantees as to how soon it is called, or
        that it will be called once for each interrupt, as multiple callback
        requests for the same record can get optimized to one.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:NAMECNT

  Inputs:
       A - Event code
       VAL - Counter that is updated every time the event is received
     
  Outputs:
       VAL - Incremented by 1
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
static long evrTimeEvent(longSubRecord *psub)
{
    if ((psub->a <= 0) || (psub->a > MRF_NUM_EVENTS)) {
        return epicsTimeERROR;
    }

    if (!evrTimeRWMutex_ps || epicsMutexLock(evrTimeRWMutex_ps)) {
        return epicsTimeERROR;
    }

    /*
     * Update the count and timestamp, unlock the mutex, and go home!
     */
    psub->val   = eventCodeTime_as[psub->a].count;
    psub->time  = eventCodeTime_as[psub->a].time;
    psub->s     = eventCodeTime_as[psub->a].status;
    epicsMutexUnlock(evrTimeRWMutex_ps);
    return epicsTimeOK;
}

/*
 * evrTimeEventProcessing handles updating the timestamps for
 * the specified event number in the event code timestamp FIFO.
 * This code used to be inline in evrTimeEvent(), but was
 * broken out into a separate routine so it could be called
 * in a more timely manner and possibly combined with evrTimeCount().
 *
 * fiddbg printf'e replaced with diag counters to avoid skewing our timestamping
 */
long evrTimeEventProcessing( epicsInt16 eventNum )
{
    evrTime_ts      	    *   pevrTime    	= NULL;
    static epicsTimeStamp   *   pLastGoodTS   	= NULL;
    int                         curFiducial	    = PULSEID_INVALID;
    epicsTimeStamp              curTimeStamp	= mod720time;
    int                         curTimeStatus	= epicsTimeERROR;
    epicsTimeStamp              newTimeStamp	= mod720time;
    int                         newTimeStatus	= epicsTimeERROR;
    int                         fidqFiducial	= PULSEID_INVALID;
	t_HiResTime	 				fidqTsc 		= 0LL;
	const double				fidInterval		= 1.0 / 360.0;
    int                         countDiff;
    if ((eventNum <= 0) || (eventNum > MRF_NUM_EVENTS)) {
        return epicsTimeERROR;
    }

    if (!evrTimeRWMutex_ps || epicsMutexLock(evrTimeRWMutex_ps)) {
        eventCodeTime_as[eventNum].status = epicsTimeERROR;
        return epicsTimeERROR;
    }

    pevrTime    = &eventCodeTime_as[eventNum];

	/* While fidq not drained */
    while (pevrTime->fidR != pevrTime->fidW) 
    {
        /*
         * First, check our local dbgcnt event counter vs the one
         * incr by the IRQ call to evrTimeCount()
         */
        if (pevrTime->dbgcnt >= pevrTime->count && 
            (pevrTime->dbgcnt < EVR_MAX_INT-100 || pevrTime->count > 100)) {
            pevrTime->nCntEarly++;
            epicsMutexUnlock(evrTimeRWMutex_ps);
            return epicsTimeERROR;
        }

        /* Increment the dbgcnt */
        if (pevrTime->dbgcnt < EVR_MAX_INT)
            pevrTime->dbgcnt++;
        else
            pevrTime->dbgcnt = 1;

        /* Update Debug Count stats */
        countDiff = pevrTime->dbgcnt - pevrTime->count;
        if( countDiff == 0 )
            pevrTime->nCntOnTime++;
        else
            pevrTime->nCntLate++;
        if( pevrTime->nCntLateMin > -countDiff )
            pevrTime->nCntLateMin = -countDiff;
        if( pevrTime->nCntLateMax < -countDiff )
            pevrTime->nCntLateMax = -countDiff;

        /* Get the fiducial from the fiducial Q */
        fidqFiducial = pevrTime->fidq[pevrTime->fidR];
    	fidqTsc		 = pevrTime->fidqtsc[pevrTime->fidR];
        if (++pevrTime->fidR == MAX_FID_QUEUE)
            pevrTime->fidR = 0;
    	if (pevrTime->fidR != pevrTime->fidW)
          	pevrTime->nFidQCountGT1++;

        /* Grab the values from the current timestamp */
        curFiducial		= EVR_APS_PULSEID( evrTimeCurrent );
        curTimeStamp	= EVR_APS_TIME(	   evrTimeCurrent );
        curTimeStatus	= EVR_APS_STATUS(  evrTimeCurrent );

        /* Check for a valid fiducial from the fidq */
        if ( PULSEID_INVALID == fidqFiducial )
            {
                /*
                 * fidQ has an invalid pulse
                 * Just use current time and status
                 */
                newTimeStamp	= curTimeStamp;
                newTimeStatus	= curTimeStatus;
                pevrTime->nFidQBad++;
            }
        else if (   curTimeStatus == epicsTimeOK
                &&  curFiducial	  == fidqFiducial	)
            {
                /*
                 * We're perfectly in sync.  Yay us.
                 */
                pLastGoodTS		= &pevrTime->time;
                newTimeStamp	= curTimeStamp;
                newTimeStatus	= epicsTimeOK;
                pevrTime->nFidQOnTime++;
            }
        else if (   EVR_APS_STATUS(  evrTimeNext1 ) == epicsTimeOK
                &&  EVR_APS_PULSEID( evrTimeNext1 ) == fidqFiducial )
            {
                /*
                 * We're here a little early, and the other thread hasn't rotated
                 * the pattern buffers yet.  We can grab the next pattern anyway.
                 */
                pLastGoodTS		= &pevrTime->time;
                newTimeStamp	= EVR_APS_TIME( evrTimeNext1 );
                newTimeStatus	= epicsTimeOK;
                pevrTime->nFidQEarly++;
            }
        else	if (	pLastGoodTS	  != NULL
#if 0
					|| (	curTimeStatus == epicsTimeOK
						&&  curFiducial   != PULSEID_INVALID )
#endif
						)
            {
                /*
                 * fidq fiducial is valid, but doesn't match evrTimeCurrent or evrTimeNext1
                 * OK, we are late, we have a good time basis!  We will
                 * construct a timestamp from this that will be close to the EVG time,
                 * and will have the correct fiducial.
                 */
                int			fidDiff;
                int			lastGoodFiducial;

				/* We see too many of these! */
                pevrTime->nFidQLate++;

#if 0
                /* See if we have a new last good timestamp */
				if (	curTimeStatus == epicsTimeOK
					&&  curFiducial   != PULSEID_INVALID )
                    pLastGoodTS = &EVR_APS_TIME( evrTimeCurrent );
#endif

                /* Grab the last good fiducial */
                lastGoodFiducial = PULSEID(*pLastGoodTS);

                fidDiff = FID_DIFF( fidqFiducial, lastGoodFiducial );

                /* Update fidq stats */
                if( pevrTime->nFidQLateMin > fidDiff )
                    pevrTime->nFidQLateMin = fidDiff;
                if( pevrTime->nFidQLateMax < fidDiff )
                    pevrTime->nFidQLateMax = fidDiff;

                if (abs(fidDiff) > 750)
                    {
                        /*
                         * We haven't seen a fiducial for over 2 seconds?
                         * This is hopeless, just give up.
                         */
                        newTimeStamp 	    =  EVR_APS_TIME( evrTimeNext1 );
                        newTimeStamp.nsec	|= 0x1ffff;		/* Make sure it's invalid! */
                        newTimeStatus		=  epicsTimeERROR;
                        pevrTime->nSetFidInvalid++;
                    }
                else
                    {
                        int		deltaNs	= fidDiff * 2777778;	/* Scale the delta to ns */

                        /* Grab the last good timestamp */
                        newTimeStamp	= *pLastGoodTS;
                        newTimeStamp.nsec &= 0xfffe0000;  /* Clear the fiducial */
	
                        /* Correct the fid based on the timestamps and fidDiff */
                        while (-deltaNs > (int)newTimeStamp.nsec) { /* Borrow! */
                            newTimeStamp.secPastEpoch--;
                            deltaNs += 1000000000;
                        }
                        newTimeStamp.nsec += deltaNs;       /* Add in the deltaNs */
                        while (newTimeStamp.nsec > 1000000000) { /* Carry! */
                            newTimeStamp.secPastEpoch++;
                            newTimeStamp.nsec -= 1000000000;
                        }
                        if (newTimeStamp.nsec & 0x10000)  /* Round up! */
                            newTimeStamp.nsec += 0x10000;
                        newTimeStamp.nsec	&= 0xfffe0000;

						/* I think this must be wrong */
						/* edtPdvCamera::TimeStampImage is reporting duplicate */
						/* timestamps each time nFidCorrected is incremented */

                        /* Insert the fiducial */
                        newTimeStamp.nsec	|= fidqFiducial;
                        newTimeStatus		= epicsTimeOK;
                        pevrTime->nFidCorrected++;
                    }
            }
        else if ( eventNum != EVENT_FIDUCIAL )
            {
                /*
                 * Bah.  Bad pattern *and* we've never seen a good one?
                 * Just give up...
                 */
                newTimeStamp      =  EVR_APS_TIME(evrTimeCurrent );
                newTimeStamp.nsec |= 0x1ffff;  /* Make sure it's invalid! */
                newTimeStatus	  =  epicsTimeERROR;
                pevrTime->nCurFidBad++;
            }

        /* Update newTimeStamp stats */
        if ( PULSEID( newTimeStamp ) != PULSEID_INVALID
             &&	newTimeStatus		  == epicsTimeOK )
            pevrTime->nTimeStampOK++;
        else
            pevrTime->nTimeStampFailed++;

        /* Don't mess with the fiducial timestamp in eventCodeTime_as[1]
         * as it's set by evrTime() based on the pattern modifiers from the IRQ */
        if ( eventNum != EVENT_FIDUCIAL )
            {
        		assert( pevrTime != &eventCodeTime_as[1] );
                /* Update the latest timestamp for this event */
                pevrTime->time      = newTimeStamp;
                pevrTime->status    = newTimeStatus;
            }

        {
			/* Keep stats on min and max deltaTsc vs prior fidqTsc in the fifo queue */
            unsigned long long	prior_idx		= (pevrTime->ts_idx - 1) & MAX_TS_QUEUE_MASK;
            long long			prior_tsc		= pevrTime->fifoInfo[prior_idx].fifo_tsc;
            long long			prior_tsc_nom	= pevrTime->fifo_tsc_nom[prior_idx];
            int					prior_fid		= pevrTime->fifoInfo[prior_idx].fifo_fid;
			long long			tscPerFid		= llround( (double) HiResTicksPerSecond() * fidInterval );
			/* if( prior_tsc_nom != 0LL ) prior_tsc = prior_tsc_nom; */
			long long		deltaTsc		= fidqTsc - prior_tsc;

			// Don't set fifoDeltaMax on the first deltaTsc as it's possibly invalid
			if( pevrTime->fifoDeltaMin != 0
			&&	pevrTime->fifoDeltaMax < deltaTsc )
				pevrTime->fifoDeltaMax = deltaTsc;

			// Set fifoDeltaMin on the first one to get us going
			if( pevrTime->fifoDeltaMin == 0
			||	pevrTime->fifoDeltaMin > deltaTsc )
				pevrTime->fifoDeltaMin = deltaTsc;

            /* Add the timestamp and fidqTsc to the fifo queue
             * EVENT_FIDUCIAL also saves timestamps here,
             * so if you want to see corrected FIDUCIAL timestamps,
             * call evrTimeGetFifoInfo() to get it from the event FIFO */
            unsigned int		idx		        = (pevrTime->ts_idx++) & MAX_TS_QUEUE_MASK;
            pevrTime->fifoInfo[idx].fifo_time   = pevrTime->time;
            pevrTime->fifoInfo[idx].fifo_tsc    = fidqTsc;
            pevrTime->fifoInfo[idx].fifo_fid    = timingGetFiducialForTimeStamp( pevrTime->time );

			// Advance fifo_tsc_nom
			int	cur_fid	= PULSEID(pevrTime->time);
			if (	prior_tsc_nom == 0LL				/* No prior_tsc_nom		*/
				||	prior_fid == PULSEID_INVALID		/* Invalid prior_fid	*/
				||	cur_fid == PULSEID_INVALID			/* Invalid cur_fid		*/
				||	HiResTicksPerSecond() < 1.5e8 )		/* diagTimer not calib.	*/
			{
            	pevrTime->fifo_tsc_nom[idx]	= fidqTsc;
				pevrTime->cum_isr_latency	= 0LL;
				pevrTime->num_isr_latency_measurements = 0LL;
				pevrTime->max_isr_latency = 0LL;
			}
			else
			{
				/* fifo_tsc_nom should be a multiple of 360hz vs prior fifo_tsc_nom.
				 * Anything more than that is an indication of less than optimal interrupt latency.
				 * Anything less is due to a shorter interrupt latency than previously seen.
				 */
				long long	fid_delta	= FID_DIFF(cur_fid, prior_fid);
				long long	tsc_nom		= prior_tsc_nom + tscPerFid * fid_delta;
				if ( fid_delta <= 0 )
				{
					pevrTime->fifo_tsc_nom[idx]	= fidqTsc;
                                        if (evrTimeEventVerbose) {
                                            printf( "event %d: Error: cur_fid <= prior_fid: tsc_nom %lld, fidqTsc %lld, tscPerFid %lld, prior_fid %d, cur_fid %d\n",
							eventNum, tsc_nom, fidqTsc, tscPerFid, prior_fid, cur_fid );
                                        }
					tsc_nom	= fidqTsc;
				}
				else if ( fidqTsc < tsc_nom )
				{
					pevrTime->fifo_tsc_nom[idx]	= fidqTsc;
					// HACK - Remove printf after initial testing
                                        if (evrTimeEventVerbose) {
                                            printf( "event %d: tsc_nom %lld, fidqTsc %lld, tscPerFid %lld, prior_fid %d, cur_fid %d\n",
							eventNum, tsc_nom, fidqTsc, tscPerFid, prior_fid, cur_fid );
                                            printf( "event %d: fifo_tsc_nom backed up %lld tsc (%.4f us)\n",
							eventNum, (tsc_nom - fidqTsc), (double)(tsc_nom - fidqTsc) / HiResTicksPerSecond() * 1.0e6 );
                                        }
					tsc_nom	= fidqTsc;
				}
				else if ( (fidqTsc - tsc_nom) > (fid_delta * tscPerFid) )
				{
					pevrTime->fifo_tsc_nom[idx]	= fidqTsc;
					// HACK - Remove printf after initial testing
                                        if (evrTimeEventVerbose) {
                                            printf( "event %d: tsc_nom %lld, fidqTsc %lld, tscPerFid %lld, prior_fid %d, cur_fid %d\n",
							eventNum, tsc_nom, fidqTsc, tscPerFid, prior_fid, cur_fid );
                                            printf( "event %d: fifo_tsc_nom advanced by %lld tsc (%.4f us)\n",
							eventNum, (fidqTsc - tsc_nom ), (double)(fidqTsc - tsc_nom) / HiResTicksPerSecond() * 1.0e6 );
                                        }
					tsc_nom	= fidqTsc;
				}
				else
					pevrTime->fifo_tsc_nom[idx]	= tsc_nom;

				/* Compare w/ actual ISR fidqTsc for diagnostics */
				pevrTime->cum_isr_latency	+= fidqTsc - tsc_nom;
				pevrTime->num_isr_latency_measurements++;
				if( pevrTime->max_isr_latency < (fidqTsc - tsc_nom) )
					pevrTime->max_isr_latency = (fidqTsc - tsc_nom);
			}
        }
    }

    epicsMutexUnlock(evrTimeRWMutex_ps);
    return epicsTimeOK;
}



/*=============================================================================

  Name: evrTimePatternPutStart

  Abs:  Lock Mutex and Return pointer to newest pattern

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        evrMessagePattern_ts ** pattern_pps Write  Pointer to pattern
        unsigned long **        timeslot_pp Write  Pointer to timeslot
        unsigned long **     patternStatus_pp Write  Pointer to status
        epicsTimeStamp *     mod720time_pps Write  Pointer to mod720 timestamp

    Rem:  The caller MUST call evrTimePatternPutEnd after the pattern is filled in.

  Side: Mutex is left locked.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimePatternPutStart(evrMessagePattern_ts **pattern_pps,
                           unsigned long        **timeslot_pp,
                           unsigned long        **patternStatus_pp,
                           epicsTimeStamp       **mod720time_pps)
{
  evrTimePattern_ts *evr_ps;
  
  if (evrTimeRWMutex_ps && (!epicsMutexLock(evrTimeRWMutex_ps))) {
    evr_ps                 = evr_aps[evrTimeNext3];
    evr_ps->timeStatus     = epicsTimeOK;
    evr_ps->pattern_s.time = evr_aps[evrTimeNext2]->pattern_s.time;
    *pattern_pps           = &evr_ps->pattern_s;
    *timeslot_pp           = &evr_ps->timeslot;
    *patternStatus_pp      = &evr_ps->patternStatus;
    *mod720time_pps        = &mod720time;
    return epicsTimeOK;
  }
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimePatternPutEnd

  Abs:  Post modulo 720 event and update timestamp if requested.  Unlock Mutex.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        int                 modulo720Flag read     Modulo 720 Flag

  Rem:  

  Side: Event posted.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimePatternPutEnd(int modulo720Flag)
{
  /* Post the modulo-720 sync event if the pattern has that bit set */
  if (modulo720Flag) {
#if EPICS_VERSION < 3 || (EPICS_VERSION == 3 && EPICS_REVISION < 15)
    post_event(EVENT_MODULO720);
#else
    static EVENTPVT modulo720evt = NULL;
    if (!modulo720evt)
      modulo720evt = eventNameToHandle(EVENT_MODULO720_STR);
    postEvent(modulo720evt);
#endif
    epicsTimeGetCurrent(&mod720time);
  }
  if (evrTimeRWMutex_ps) {
    epicsMutexUnlock(evrTimeRWMutex_ps);
    return epicsTimeOK;
  }
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimeGetFiducial

  Abs:  Used in a aSub record to extract the fiducial from a record's timestamp.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        struct aSubRecord* psub       read/write The aSub record.

  Rem:  TSEL should be a DBLINK to the TIME field of the record with the
  		desired timestamp, and TSE should be -2.

  Ret:  Fiducial number, to be put into VAL field of aSub.

==============================================================================*/

long evrTimeGetFiducial(struct aSubRecord *psub)
{
	recGblGetTimeStamp( psub );
	return psub->time.nsec & 0x1ffff;
}

extern void eventDebug(int arg1, int arg2)
{
	int iFifoDump = 0;
	int nFifoDump = 10;
    int doreset = 0;
	evrTimeEventVerbose = 1;
    if (arg1 < 0) {
        arg1 = -arg1;
        doreset = 1;
    }
    do {
		uint64_t			idx         = 0LL;
		long long			delta_tsc   = 0LL;
		long long			prior_tsc   = 0LL;
		long long			tsc_latency = 0LL;
        evrTime_ts      *   pevrTime    = &eventCodeTime_as[arg1];
        printf( "Event Code %d:\n", arg1 );
        printf( "   Count = %d, time = %08x.%08x, status = %d\n",
                pevrTime->count,
				pevrTime->time.secPastEpoch,
                pevrTime->time.nsec,   
				pevrTime->status    );
        printf(	"   dbgcnt = %d, nCntEarly = %d, nCntLate = %d, nCntOnTime = %d\n",
				pevrTime->dbgcnt, pevrTime->nCntEarly, pevrTime->nCntLate, pevrTime->nCntOnTime );
        printf(	"   nCntLateMin = %d, nCntLateMax = %d\n",
				pevrTime->nCntLateMin, pevrTime->nCntLateMax );
		for ( iFifoDump = 0; iFifoDump < nFifoDump; iFifoDump++ )
		{
			char	strTime[40];
			EventTimingData	fifoInfo;
			int		incr;
			int		status;
			if ( iFifoDump == 0 )
				incr	= TS_INDEX_INIT;
			else
				incr	= -1;

			status = timingFifoRead( arg1, incr, &idx, &fifoInfo );
  			long long int	tsc_nom = pevrTime->fifo_tsc_nom[idx & MAX_TS_QUEUE_MASK];
			if ( iFifoDump == 0 )
			{
				int fidx = idx & MAX_TS_QUEUE_MASK;
				int lidx = (idx + MAX_TS_QUEUE - 1) & MAX_TS_QUEUE_MASK;
				printf( "   FIFO: idx = 0x%" PRIx64 ", fidx = 0x%x, lidx = 0x%x\n", idx, fidx, lidx );
				delta_tsc    = 0LL;
                tsc_latency  = 0LL;
			}
			else
			{
				tsc_latency	= fifoInfo.fifo_tsc - tsc_nom;
				delta_tsc	= prior_tsc - fifoInfo.fifo_tsc;
			}
			prior_tsc = fifoInfo.fifo_tsc;
			epicsTimeToStrftime( strTime, 40, "%M:%S.%05f", &fifoInfo.fifo_time );
#ifdef HI_RES_TIME_H
#if 1
			printf( "     time(%2d) = %14s, fid %d, delta %.3fms, latency %lld ticks\n", -iFifoDump,
					strTime, fifoInfo.fifo_time.nsec & 0x1ffff,
					HiResTicksToSeconds( delta_tsc ) * 1000,
					tsc_latency );
#else
			printf( "     time(%2d) = %14s, fid %d, delta %.3fms, fifo_tsc %lld fifo_tsc_nom %lld\n", -iFifoDump,
					strTime, fifoInfo.fifo_time.nsec & 0x1ffff,
					HiResTicksToSeconds( delta_tsc ) * 1000,
					(long long int) fifoInfo.fifo_tsc, tsc_nom );
#endif
#else
			printf( "     time(%d) = %14s, fid %d, delta %lld ticks, latency %lld ticks\n", -iFifoDump,
					strTime, fifoInfo.fifo_time.nsec & 0x1ffff, delta_tsc, tsc_latency );
#endif /* HI_RES_TIME_H */
			if ( status < 0 )
			{
				printf( "    Nothing more in FIFO\n" );
				break;
			}
		}
        printf(	"   FIFO: deltaMin = %.3fms, deltaMax = %.3fms\n",
					HiResTicksToSeconds( pevrTime->fifoDeltaMin ) * 1000,
					HiResTicksToSeconds( pevrTime->fifoDeltaMax ) * 1000	);
        printf( "   lastfid     = %d\n", evrGetLastFiducial() );
		int	fidW = pevrTime->fidW - 1;
		int	fidR = pevrTime->fidR - 1;
		if( fidW < 0 ) fidW = MAX_FID_QUEUE - 1;
		if( fidR < 0 ) fidR = MAX_FID_QUEUE - 1;
		printf( "   Wr fidq[%d] = %d, Rd fidq[%d] = %d\n",
					fidW, pevrTime->fidq[fidW],
					fidR, pevrTime->fidq[fidR] );
        printf( "   nFidQEarly = %d, nFidQLate = %d, nFidQOnTime = %d\n",
					pevrTime->nFidQEarly, pevrTime->nFidQLate, pevrTime->nFidQOnTime );
        printf( "   nFidQBad = %d, FidQCountGT1 = %d, nSetFidInvalid = %d\n",
					pevrTime->nFidQBad, pevrTime->nFidQCountGT1, pevrTime->nSetFidInvalid );
        printf(	"   nFidQLateMin = %d, nFidQLateMax = %d\n",
					pevrTime->nFidQLateMin, pevrTime->nFidQLateMax );
        printf( "   nCurFidBad = %d, nFidCorrected = %d\n",
					pevrTime->nCurFidBad, pevrTime->nFidCorrected );
        printf( "   nTimeStampOK = %d, nTimeStampFailed = %d\n",
					pevrTime->nTimeStampOK, pevrTime->nTimeStampFailed );
        printf( "   avg_isr_latency = %.3f us, max_isr_latency = %.3f us\n",
					(double) pevrTime->cum_isr_latency / pevrTime->num_isr_latency_measurements / HiResTicksPerSecond() * 1.0e6,
					pevrTime->max_isr_latency / HiResTicksPerSecond() * 1.0e6 );

        if (doreset) {
			// if ( evrTimeRWMutex_ps && epicsMutexLock(evrTimeRWMutex_ps) == 0 )
			// No need to take the mutex, potentially delaying the ISR,
			// as these are just independent diagnostic counts
			pevrTime->dbgcnt = pevrTime->count;
			pevrTime->fifoDeltaMin = 0LL;
			pevrTime->fifoDeltaMax = 0LL;
			pevrTime->nCntEarly   = 0;
			pevrTime->nCntLate   = 0;
			pevrTime->nCntLateMin  = EVR_MAX_INT;
			pevrTime->nCntLateMax  = 0;
			pevrTime->nCntOnTime  = 0;
			pevrTime->nCurFidBad	= 0;
			pevrTime->nFidCorrected	= 0;
			pevrTime->nFidQEarly   = 0;
			pevrTime->nFidQBad   = 0;
			pevrTime->nSetFidInvalid   = 0;
			pevrTime->nFidQLate   = 0;
			pevrTime->nFidQLateMin   = EVR_MAX_INT;
			pevrTime->nFidQLateMax   = 0;
			pevrTime->nFidQOnTime   = 0;
			pevrTime->nFidQCountGT1   = 0;
			pevrTime->nTimeStampOK		= 0;
			pevrTime->nTimeStampFailed	= 0;
			pevrTime->cum_isr_latency	= 0LL;
			pevrTime->num_isr_latency_measurements = 0LL;
			pevrTime->max_isr_latency = 0LL;
		//	epicsMutexUnlock(evrTimeRWMutex_ps);
        }
        arg1++;
    } while (arg1 <= arg2);
	evrTimeEventVerbose = 0;
    fflush(stdout);
}

/* iocsh command: eventDebug */
static const iocshArg eventDebugArg0 = {"ECstart" , iocshArgInt};
static const iocshArg eventDebugArg1 = {"ECfinal" , iocshArgInt};
static const iocshArg *const eventDebugArgs[2] = {&eventDebugArg0, &eventDebugArg1};
static const iocshFuncDef eventDebugDef = {"eventDebug", 2, eventDebugArgs};

static void eventDebugCall(const iocshArgBuf * args)
{
    int ecstart = args[0].ival, ecfinal = args[1].ival;
    if (ecfinal < abs(ecstart))
        ecfinal = abs(ecstart);
    eventDebug(ecstart, ecfinal);
}

/* Registration APIs */
static void evrSupportRegistrar()
{
    iocshRegister(  &eventDebugDef, eventDebugCall );
}
epicsExportRegistrar(	evrSupportRegistrar	);
epicsRegisterFunction(	evrTimeProcInit	);
epicsRegisterFunction(	evrTimeProc	);
epicsRegisterFunction(	evrTimeDiag	);
epicsRegisterFunction(	evrTimeRate	);
epicsRegisterFunction(	evrTimeEvent	);
epicsRegisterFunction(	evrTimeGetFiducial	);
epicsRegisterFunction(	eventDebug	);
epicsExportAddress(int, evrTimeEventVerbose );
