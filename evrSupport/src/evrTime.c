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
           evrTimeGetFifo    - Get Timestamp from an Event FIFO
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
#include "subRecord.h"        /* for struct subRecord      */
#include "longSubRecord.h"    /* for struct longSubRecord  */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "iocsh.h"            /* for iocshRegister         */
#include "epicsTime.h"        /* epicsTimeStamp and protos */
#include "epicsGeneralTime.h" /* generalTimeTpRegister     */
#include "generalTimeSup.h"
#include "epicsMutex.h"       /* epicsMutexId and protos   */
#include "alarm.h"            /* INVALID_ALARM             */
#include "dbScan.h"           /* for post_event            */
#include <dbAccess.h>         /* for dbNameToAddr          */
#include <genSubRecord.h>
#include <stdlib.h>

#include "mrfCommon.h"        /* MRF_NUM_EVENTS */    
#include "evrMessage.h"       /* EVR_MAX_INT    */    
#include "evrTime.h"       
#include "evrPattern.h"        
#include <time.h>

#define  EVR_TIME_OK 0
#define  EVR_TIME_INVALID 1

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
  epicsTimeStamp      fifotime[MAX_TS_QUEUE];
  int                 fifostatus[MAX_TS_QUEUE];
  unsigned long long  ts_idx;
  int                 count;         /* # times this event has happened  */

	/* Diagnostics */
  int                 dbgcnt;
  int				  nCntEarly;
  int				  nCntLate;
  int				  nCntLateMin;
  int				  nCntLateMax;
  double			  nCntLateSmoo;
  int				  nCntOnTime;
  int				  nCurFidBad;
  int				  nFidCorrected;
  int				  nFidQBad;
  int				  nFidQEarly;
  int				  nFidQLate;
  int				  nSetFidInvalid;
  int				  nFidQLateMin;
  int				  nFidQLateMax;
  double			  nFidQLateSmoo;
  int				  nFidQOnTime;
  int				  nTimeStampOK;
  int				  nTimeStampFailed;
  int                 fidq[MAX_TS_QUEUE];
  int                 fidR;
  int                 fidW;
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
int bsa_debug_mask = 0; /* BSA debugging mask */
#endif

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
static int                 edef_idx[EDEF_MAX];     /* The next write location! */

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

/* Fiducial debug variables */
int		fiddbg		= 0;
int		fiddbgcnt	= 0;


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

#if 0
/*
 * OK, now to win a race in evrTimeGetFromEdefTime, we are looking ahead and
 * putting the latest result we've received into the queue.  The problem is
 * that perhaps we've gotten ahead of ourselves and need to go back *two*
 * entries.  However, I don't think anyone is using this routine at this point,
 * so I'm not going to fix this right now.
 */
/*=============================================================================

  Name: evrTimeGetFromEdef

  Abs:  Get the evr epics timestamp from EDEF, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
        
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        unsigned int        edefIdx       Read     EDEF Index (0 to EDEF_MAX-1)
        epicsTimeStamp *    edefTime_ps   Write    EDEF active timestamp
        epicsTimeStamp *    edefTimeInit_ps Write  EDEF init   timestamp
        int *               edefAvgDone_p Write    EDEF average-done flag
        epicsEnum16  *      edefSevr_p    Write    EDEF severity

  Rem:  Routine to get the epics timestamp and flags from the EDEF timestamp
        table that is populated from incoming broadcast from EVG

  Side: None

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFromEdef    (unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p)
{
  int idx; 
  evrTimeEdef_ts *edef;

  if ((edefIdx >= EDEF_MAX) || (!evrTimeRWMutex_ps)) return epicsTimeERROR;
  /* if the r/w mutex is valid, and we can lock with it, read requested time index */
  if (epicsMutexLock(evrTimeRWMutex_ps)) return epicsTimeERROR;

  idx = (edef_idx[edefIdx] - 1) & MAX_EDEF_TIME_MASK; /* Back one == last written! */
  edef = &edef_as[edefIdx][idx];

  *edefTime_ps     = edef->time;
  *edefTimeInit_ps = edef->timeInit;
  *edefAvgDone_p   = edef->avgdone;
  *edefSevr_p      = edef->sevr;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return epicsTimeOK;
}
#endif

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
                           epicsEnum16     *edefSevr_p)
{
  int idx, i; 
  evrTimeEdef_ts *edef;

  if ((edefIdx >= EDEF_MAX) || (!evrTimeRWMutex_ps)) return epicsTimeERROR;
  /* if the r/w mutex is valid, and we can lock with it, read requested time index */
  if (epicsMutexLock(evrTimeRWMutex_ps)) return epicsTimeERROR;

  idx = (edef_idx[edefIdx] - 1) & MAX_EDEF_TIME_MASK; /* Back one == last written! */
  edef = &edef_as[edefIdx][idx];

  for (i = 0; i < MAX_EDEF_TIME - 2; i++) {
      if ((edef->time.secPastEpoch == edefTime_ps->secPastEpoch) &&
          (edef->time.nsec == edefTime_ps->nsec)) {
          *edefTimeInit_ps = edef->timeInit;
          *edefAvgDone_p   = edef->avgdone;
          *edefSevr_p      = edef->sevr;
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if ((1 << edefIdx) & bsa_debug_mask)
              printf("%08x:%08x BSA%d, slot %d.\n", edefTime_ps->secPastEpoch, edefTime_ps->nsec, edefIdx, idx);
#endif
          return epicsTimeOK;
      } else if ((edef->time.secPastEpoch < edefTime_ps->secPastEpoch) ||
                 ((edef->time.secPastEpoch == edefTime_ps->secPastEpoch) &&
                  (edef->time.nsec < edefTime_ps->nsec))) {
          epicsMutexUnlock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
          if ((1 << edefIdx) & bsa_debug_mask) {
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
  if ((1 << edefIdx) & bsa_debug_mask)
      printf("%08x:%08x BSA%d not found.\n", edefTime_ps->secPastEpoch, edefTime_ps->nsec, edefIdx);
#endif
  return epicsTimeERROR; /* No match! */
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

  Name: evrTimeGetFifo

  Abs:  Get the epics timestamp associated with an event code from the fifo, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
        
  Args: Type     Name           Access     Description
        -------  -------    ---------- ----------------------------
  epicsTimeStamp * epicsTime_ps write  ptr to epics timestamp to be returned
  unsigned int   eventCode      read   Event code 0 to 255.
                                      0,1=time associated w this pulse
                                          (event code 1 = fiducial)
                                          1 to 255 = EVR event codes
  unsigned long long * idx      read/write The last fifo index we read
  int            incr           read       How far to move ahead (MAX_TS_QUEUE if idx is uninitialized)

  Rem:  Routine to get the epics timestamp from a queue of timestamps.  This must
        be called no faster than timestamps come in, as there is no checking for bounds.
        
        "evrTimeGetFifo(&ts, event, &idx, MAX_TS_QUEUE)" and "evrTimeGet(&ts, event)" return
        identical timestamps.
        

  Side: 

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFifo (epicsTimeStamp  *epicsTime_ps, unsigned int eventCode, unsigned long long *idx, int incr)
{
  int status = 0, i;
  
  if ((eventCode > MRF_NUM_EVENTS) || (!evrTimeRWMutex_ps) || epicsMutexLock(evrTimeRWMutex_ps))
    return epicsTimeERROR;
  if (incr == MAX_TS_QUEUE)
      *idx = eventCodeTime_as[eventCode].ts_idx - 1;
  else
      *idx += incr;
  /*
   * eventCodeTime_as[eventCode].ts_idx is the monotonically increasing location for the *next*
   * timestamp... it isn't valid yet!  Since there are MAX_TS_QUEUE entries, this means that
   * the valid entries are between eventCodeTime_as[eventCode].ts_idx - MAX_TS_QUEUE and 
   * eventCodeTime_as[eventCode].ts_idx - 1.
   */
  if (*idx + MAX_TS_QUEUE < eventCodeTime_as[eventCode].ts_idx ||
      *idx > eventCodeTime_as[eventCode].ts_idx) {
      epicsMutexUnlock(evrTimeRWMutex_ps);
      return epicsTimeERROR;
  }
 
  /*
   * If we're here *slightly* early (eventCodeTime_as[eventCode].ts_idx == *idx), we'll wait.
   * Otherwise, we report an error.
   */
  if (*idx == eventCodeTime_as[eventCode].ts_idx) {
      epicsMutexUnlock(evrTimeRWMutex_ps);
#define TO_LIM 4
      for (i = 1; i < TO_LIM; i++) {
          struct timespec req = {0, 1000000}; /* 1 ms */
          nanosleep(&req, NULL);
          if (*idx < eventCodeTime_as[eventCode].ts_idx)
              break;
      }
      if (i == TO_LIM) {
          if ( (fiddbg & 1) ) {
              printf("\nevrTimeGetFIFO timeout %lld %lld!\n", *idx, eventCodeTime_as[eventCode].ts_idx);fflush(stdout);
              if (!--fiddbgcnt)
                fiddbg &= ~1;
          }
          status = 0x1ffff; /* We missed, so at least flag this as invalid! */
      }
      epicsMutexLock(evrTimeRWMutex_ps);
  }
  *epicsTime_ps = eventCodeTime_as[eventCode].fifotime[     *idx & MAX_TS_QUEUE_MASK ];
  epicsTime_ps->nsec |= status;
  status        = eventCodeTime_as[eventCode].fifostatus[   *idx & MAX_TS_QUEUE_MASK ];
  epicsMutexUnlock(evrTimeRWMutex_ps);
 
  return status; 
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
    return evrTimeGetSystem(epicsTime_ps, 0);
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
          evr_aps[idx] = evr_as + idx;
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
          for (idx2 = 0; idx2 < MAX_TS_QUEUE; idx2++) {
              pevrTime->fifotime[idx2] = mod720time;
              pevrTime->fifostatus[idx2] = epicsTimeERROR;
          }
          pevrTime->ts_idx  = 0LL;
          pevrTime->time    = mod720time;
          pevrTime->status  = epicsTimeERROR;
          pevrTime->count   = 0;

          pevrTime->dbgcnt  = 0;
          pevrTime->nCntEarly   = 0;
          pevrTime->nCntLate   = 0;
          pevrTime->nCntLateMin  = EVR_MAX_INT;
          pevrTime->nCntLateMax  = 0;
          pevrTime->nCntLateSmoo  = 0;
          pevrTime->nCntOnTime  = 0;
		  pevrTime->nCurFidBad	= 0;
		  pevrTime->nFidCorrected	= 0;
          pevrTime->nFidQEarly   = 0;
          pevrTime->nFidQLate   = 0;
          pevrTime->nFidQBad   = 0;
          pevrTime->nSetFidInvalid   = 0;
		  pevrTime->nFidQLateMin   = EVR_MAX_INT;
		  pevrTime->nFidQLateMax   = 0;
		  pevrTime->nFidQLateSmoo   = 0;
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
#ifdef BSA_DEBUG
            if (bsa_debug_mask & edefMask)
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
            if (bsa_debug_mask & edefMask)
                printf("%08x:%08x EDEF%d slot %d, done=%d.\n", edef->time.secPastEpoch, edef->time.nsec, idx, idx2, edef->avgdone);
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

  Rem:  This routine is called at interrupt level.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimeCount(unsigned int eventCode, unsigned int fiducial)
{
  if ((eventCode > 0) && (eventCode <= MRF_NUM_EVENTS)) {
    evrTime_ts  *   pevrTime = &eventCodeTime_as[eventCode];
    /* Rollover if value gets too big */
    if (pevrTime->count < EVR_MAX_INT)
        pevrTime->count++;
    else
        pevrTime->count = 1;
#ifndef __rtems__foo
	if( fiducial > PULSEID_MAX )
		fiducial = PULSEID_INVALID;
    pevrTime->fidq[pevrTime->fidW] = fiducial;
    if (++pevrTime->fidW == MAX_TS_QUEUE)
        pevrTime->fidW = 0;
#endif  /* __rtems__ */
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

#define USE_EVENT_MSG_Q 1
#if USE_EVENT_MSG_Q == 0
    /*
     * Only modify the event time if this is the FLNK of an event.
     * We don't actually know this, but we assume it if we're passive.
     */
    if (psub->scan == SCAN_PASSIVE) {
        evrTimeEventProcessing( psub->a );
    }
#endif  /* USE_EVENT_MSG_Q */

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
 * the specified event number.
 * This code used to be inline in evrTimeEvent(), but was
 * broken out into a separate routine so it could be called
 * in a more timely manner and possibly combined with evrTimeCount().
 *
 * fiddbg printf'e replaced with diag counters to avoid skewing our timestamping
 */
long evrTimeEventProcessing(epicsInt16 eventNum)
{
    evrTime_ts      	    *   pevrTime    	= NULL;
    static epicsTimeStamp   *   pLastGoodTS   	= NULL;
    int                         curFiducial	= PULSEID_INVALID;
    epicsTimeStamp              curTimeStamp	= mod720time;
    int                         curTimeStatus	= epicsTimeERROR;
    epicsTimeStamp              newTimeStamp	= mod720time;
    int                         newTimeStatus	= epicsTimeERROR;
    int                         fidqFiducial	= PULSEID_INVALID;
    int                         countDiff;

    if ((eventNum <= 0) || (eventNum > MRF_NUM_EVENTS)) {
        return epicsTimeERROR;
    }

    if (!evrTimeRWMutex_ps || epicsMutexLock(evrTimeRWMutex_ps)) {
        eventCodeTime_as[eventNum].status = epicsTimeERROR;
        return epicsTimeERROR;
    }

    pevrTime    = &eventCodeTime_as[eventNum];

#ifndef __rtems__
    while (pevrTime->fidR != pevrTime->fidW) 
#endif
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
        pevrTime->nCntLateSmoo = pevrTime->nCntLateSmoo * 0.9 + -countDiff * 0.1;

        /* Get the fiducial from the fiducial Q */
        fidqFiducial = pevrTime->fidq[pevrTime->fidR];
        if (++pevrTime->fidR == MAX_TS_QUEUE)
            pevrTime->fidR = 0;

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
                newTimeStamp	= EVR_APS_TIME( evrTimeCurrent );
                newTimeStatus	= epicsTimeOK;
                pevrTime->nFidQOnTime++;
            }
        else if (	EVR_APS_STATUS(  evrTimeNext1 )	== epicsTimeOK
                        &&  EVR_APS_PULSEID( evrTimeNext1 ) == fidqFiducial	)
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
			||	(	curTimeStatus == epicsTimeOK
                                        &&  curFiducial   != PULSEID_INVALID ) )
            {
                /*
                 * fidq fiducial is valid, but doesn't match evrTimeCurrent or evrTimeNext1
                 * OK, we are late, we have a good time basis!  We will
                 * construct a timestamp from this that will be close to the EVG time,
                 * and will have the correct fiducial.
                 */
                int			fidDiff;
                int			lastGoodFiducial;

                pevrTime->nFidQLate++;

                /* See if we have a new last good timestamp */
                if  (   epicsTimeOK     == EVR_APS_STATUS(  evrTimeCurrent )
                        &&  PULSEID_INVALID != EVR_APS_PULSEID( evrTimeCurrent ) )
                    pLastGoodTS = &EVR_APS_TIME( evrTimeCurrent );

                /* Grab the last good fiducial */
                lastGoodFiducial = PULSEID(*pLastGoodTS);

                fidDiff = FID_DIFF( fidqFiducial, lastGoodFiducial );
#if 0
                if ( delta != fidDiff ) {
                    printf( "ERROR: fidq-fid %d - lastGoodFiducial %d = %d, not %d\n",
                            fidqFiducial, lastGoodFiducial, fidDiff, delta );
                    fflush(stdout);
                }
#endif

                /* Update fidq stats */
                if( pevrTime->nFidQLateMin > fidDiff )
                    pevrTime->nFidQLateMin = fidDiff;
                if( pevrTime->nFidQLateMax < fidDiff )
                    pevrTime->nFidQLateMax = fidDiff;
                pevrTime->nFidQLateSmoo = pevrTime->nFidQLateSmoo * 0.9 + fidDiff * 0.1;

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
                /* Update the latest timestamp for this event */
                pevrTime->time      = newTimeStamp;
                pevrTime->status    = newTimeStatus;
            }

        {
            /* Add the timestamp to the fifo queue
             * EVENT_FIDUCIAL also saves timestamps here,
             * so if you want to see corrected FIDUCIAL timestamps,
             * call evrTimeGetFifo() to get it from the event FIFO */
            unsigned int    idx = (pevrTime->ts_idx++) & MAX_TS_QUEUE_MASK;
            pevrTime->fifotime[idx]   = pevrTime->time;
            pevrTime->fifostatus[idx] = pevrTime->status;
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
    post_event(EVENT_MODULO720);
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

  Abs:  Used in a genSub record to extract the fiducial from a record's timestamp.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        struct genSubRecord* psub       read/write The genSub record.

  Rem:  INPA should be a DBLINK to the NAME field of the record with the desired
        timestamp.  The name is extracted into psub->a as a string, from which we
        then find the database record and store into psub->dpvt for quick access.

  Ret:  Fiducial number, to be put into VAL field of genSub.

==============================================================================*/

long evrTimeGetFiducial(struct genSubRecord *psub)
{
    struct dbCommon *precord;
    if (!psub->dpvt) {
        struct dbAddr addr;
        if (dbNameToAddr((char *)psub->a, &addr))
            return 0x1ffff;
        else
            psub->dpvt = addr.precord;
    }
    precord = (struct dbCommon *)psub->dpvt;
    if (psub->tse == epicsTimeEventDeviceTime)
        psub->time = precord->time;
    return precord->time.nsec & 0x1ffff;
}

extern void eventDebug(int arg1, int arg2)
{
    int doreset = 0;
    if (arg1 < 0) {
        arg1 = -arg1;
        doreset = 1;
    }
    do {
        evrTime_ts      *   pevrTime    = &eventCodeTime_as[arg1];
        unsigned long long  idx         = pevrTime->ts_idx;
        int fidx = idx & MAX_TS_QUEUE_MASK;
        int lidx = (idx + MAX_TS_QUEUE - 1) & MAX_TS_QUEUE_MASK;
        printf( "Event Code %d:\n", arg1 );
        printf( "   Count = %d, time = %08x.%08x, status = %d\n",
                pevrTime->count,
				pevrTime->time.secPastEpoch,
                pevrTime->time.nsec,   
				pevrTime->status    );
        printf(	"   count = %d, dbgcnt = %d, nCntEarly = %d, nCntLate = %d, nCntOnTime = %d\n",
				pevrTime->count, pevrTime->dbgcnt, pevrTime->nCntEarly, pevrTime->nCntLate, pevrTime->nCntOnTime );
        printf(	"   nCntLateMin = %d, nCntLateMax = %d, nCntLateSmoo = %f\n",
				pevrTime->nCntLateMin, pevrTime->nCntLateMax, pevrTime->nCntLateSmoo );
        printf("    idx = %llu, fidx = %d, lidx = %d\n", idx, fidx, lidx );
        printf("    first time = %08x.%08x\n",
                pevrTime->fifotime[fidx].secPastEpoch,
                pevrTime->fifotime[fidx].nsec);
        printf("    last time  = %08x.%08x\n",
                pevrTime->fifotime[lidx].secPastEpoch,
                pevrTime->fifotime[lidx].nsec);
        printf("    lastfid    = 0x%05x\n", lastfid );
        printf("    Wr fidq[%d] = 0x%05x, Rd fidq[%d] = 0x%05x\n",
					pevrTime->fidW, pevrTime->fidq[pevrTime->fidW],
					pevrTime->fidR, pevrTime->fidq[pevrTime->fidR] );
        printf("    nFidQEarly = %d, nFidQLate = %d, nFidQOnTime = %d\n",
					pevrTime->nFidQEarly, pevrTime->nFidQLate, pevrTime->nFidQOnTime );
        printf("    nFidQBad = %d, nSetFidInvalid = %d\n",
					pevrTime->nFidQBad, pevrTime->nSetFidInvalid );
        printf(	"   nFidQLateMin = %d, nFidQLateMax = %d, nFidQLateSmoo = %f\n",
					pevrTime->nFidQLateMin, pevrTime->nFidQLateMax, pevrTime->nFidQLateSmoo );
        printf("    nCurFidBad = %d, nFidCorrected = %d\n",
					pevrTime->nCurFidBad, pevrTime->nFidCorrected );
        printf("    nTimeStampOK = %d, nTimeStampFailed = %d\n",
					pevrTime->nTimeStampOK, pevrTime->nTimeStampFailed );

        if (doreset) {
			if ( evrTimeRWMutex_ps && epicsMutexLock(evrTimeRWMutex_ps) == 0 )
			{
				pevrTime->fidR = pevrTime->fidW;
				pevrTime->dbgcnt = pevrTime->count;
				pevrTime->nCntEarly   = 0;
				pevrTime->nCntLate   = 0;
				pevrTime->nCntLateMin  = EVR_MAX_INT;
				pevrTime->nCntLateMax  = 0;
				pevrTime->nCntLateSmoo  = 0;
				pevrTime->nCntOnTime  = 0;
				pevrTime->nCurFidBad	= 0;
				pevrTime->nFidCorrected	= 0;
				pevrTime->nFidQEarly   = 0;
				pevrTime->nFidQBad   = 0;
				pevrTime->nSetFidInvalid   = 0;
				pevrTime->nFidQLate   = 0;
				pevrTime->nFidQLateMin   = EVR_MAX_INT;
				pevrTime->nFidQLateMax   = 0;
				pevrTime->nFidQLateSmoo   = 0;
				pevrTime->nFidQOnTime   = 0;
				pevrTime->nTimeStampOK		= 0;
				pevrTime->nTimeStampFailed	= 0;
				epicsMutexUnlock(evrTimeRWMutex_ps);
			}
        }
        arg1++;
    } while (arg1 <= arg2);
    fflush(stdout);
}


/* iocsh command: fidDebug */
static const iocshArg fidDebugArg0 = {"Mask" , iocshArgInt};
static const iocshArg fidDebugArg1 = {"Count" , iocshArgInt};
static const iocshArg *const fidDebugArgs[2] = {&fidDebugArg0, &fidDebugArg1};
static const iocshFuncDef fidDebugDef = {"fidDebug", 2, fidDebugArgs};
extern void fidDebug( int mask, int count )
{
    if ( count == 0 )
        count = 200;
	fiddbg		= mask;
	fiddbgcnt	= count;
    printf("Fiducial debugging is %d (0x%x).\n", fiddbg, fiddbg);
    fflush(stdout);
}

static void fidDebugCall(const iocshArgBuf * args)
{
    int mask	= args[0].ival;
    int count	= args[1].ival;
    fidDebug( mask, count );
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
    iocshRegister(  &fidDebugDef, 	fidDebugCall );
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
epicsRegisterFunction(	fidDebug	);
