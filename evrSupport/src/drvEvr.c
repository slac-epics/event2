/*=============================================================================
 
  Name: drvEvr.c
        evrInitialize  - EVR Data and Event Initialisation
        evrReport      - Driver Report
        evrSend        - Send EVR data to Message Queue
        evrEvent       - Process Event Codes
        evrTask        - High Priority task to process 360Hz Fiducial and Data
        evrRecord      - High Priority task to process 360Hz Records
        evrTimeRegister- Register User Routine called by evrTask 

  Abs:  Driver data and event support for EVR Receiver module or EVR simulator.   

  Auth: 22-dec-2006, S. Allison:
  Rev:  

-----------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */
 
/*-----------------------------------------------------------------------------
 
  Mod:  (newest to oldest)  
 
=============================================================================*/

#include <stdlib.h> 		/* for calloc             */
#include "drvSup.h" 		/* for DRVSUPFN           */
#include "errlog.h"		/* for errlogPrintf       */
#include "epicsExport.h" 	/* for epicsExportAddress */
#include "epicsEvent.h" 	/* for epicsEvent*        */
#include "epicsThread.h" 	/* for epicsThreadCreate  */
#include "evrMessage.h"		/* for evrMessageCreate   */
#include "evrTime.h"		/* for evrTimeCount       */
#include "evrPattern.h"		/* for evrPattern         */
#include "bsa.h"		/* for bsaInit            */
#include "drvMrfEr.h"		/* for ErRegisterDevDBuffHandler */
#include "devMrfEr.h"		/* for ErRegisterEventHandler    */

#ifdef __rtems__
#define EVR_TIMEOUT     (0.06)  /* Timeout in sec waiting for 360hz input. */
#else
#define EVR_TIMEOUT     (2)  /* Timeout in sec waiting for 360hz input. */
#endif

#define MIN(a,b)        (((a)>(b))?(b):(a))
#define MAX(a,b)        (((a)>(b))?(a):(b))
epicsUInt32 fiducial_time_stamp[PEEK_PIPE_SIZE];
epicsUInt32 fiducial_time_stamp_ix=0;
epicsUInt32 event_to_peek_fiducial=40;

static int evrReport();
struct drvet drvEvr = {
  2,
  (DRVSUPFUN) evrReport, 	/* subroutine defined in this file */
  (DRVSUPFUN) evrInitialize 	/* subroutine defined in this file */
};
epicsExportAddress(drvet, drvEvr);

static ErCardStruct    *pCard             = NULL;  /* EVR card pointer    */
static epicsEventId     evrTaskEventSem   = NULL;  /* evr task semaphore  */
static epicsEventId     evrRecordEventSem = NULL;  /* evr record task sem */
static int readyForFiducial = 1;        /* evrTask ready for new fiducial */
static int evrInitialized = 0;          /* evrInitialize performed        */
int lastfid = -1;                       /* Last fiducial seen             */

/* Fiducial User Function List */
typedef struct {
  ELLNODE node;
  FIDUCIALFUNCTION func;
  void * arg;

} evrFiducialFunc_ts;

ELLLIST evrFiducialFuncList_s;
static epicsMutexId evrRWMutex_ps = 0; 

/*=============================================================================

  Name: evrReport

  Abs: Driver support registered function for reporting system info

=============================================================================*/
static int evrReport( int interest )
{
  if (interest > 0) {
    if (pCard) {
      epicsUInt32 pulseIDfromTime;
      epicsUInt32 pulseIDfromEvr = 0;
      epicsTimeStamp currentTime;
      printf("Pattern data from %s card %d\n",
             (pCard->FormFactor==1)?"PMC":(pCard->FormFactor==2)?"Embedded":
             (pCard->FormFactor==0xF)?"SLAC":"VME",
             pCard->Cardno);
      /* Get pulse ID from timestamp. */
      evrTimeGetFromPipeline(&currentTime, evrTimeCurrent, 0, 0, 0, 0, 0);
      pulseIDfromTime = PULSEID(currentTime);
      /* Get pulse ID from EVR seconds register. */
#ifdef EVR_DRIVER_SUPPORT
      pulseIDfromEvr = ErGetSecondsSR(pCard);
#endif
      printf("Pulse ID from Data = %lu, from EVR: %lu\n",
             (unsigned long)pulseIDfromTime, (unsigned long)pulseIDfromEvr);
    }
    if (evrRWMutex_ps) {
      evrFiducialFunc_ts *fid_ps =
        (evrFiducialFunc_ts *)ellFirst(&evrFiducialFuncList_s);
      while (fid_ps) {
        printf("Registered fiducial function %p\n", fid_ps->func);
        fid_ps = (evrFiducialFunc_ts *)ellNext(&fid_ps->node);
      }
    }
    evrMessageReport(EVR_MESSAGE_FIDUCIAL, EVR_MESSAGE_FIDUCIAL_NAME,
                     interest);
    evrMessageReport(EVR_MESSAGE_PATTERN,  EVR_MESSAGE_PATTERN_NAME ,
                     interest);
  }
  return interest;
}

/*=============================================================================
 
  Name: evrSend

  Abs: Called by either ErIrqHandler to put each buffer of EVR data
       into the proper message area or by a subroutine record for
       the EVR simulator.

  Rem: Keep this routine to a minimum, so that CPU not blocked 
       too long processing each interrupt.
       
=============================================================================*/
void evrSend(void *pCard, epicsInt16 messageSize, void *message)
{
  unsigned int messageType = ((evrMessageHeader_ts *)message)->type;

  /* Look for error from the driver or the wrong message size */
  if ((pCard && ((ErCardStruct *)pCard)->DBuffError) ||
      (messageSize != sizeof(evrMessagePattern_ts))) {
    evrMessageCheckSumError(EVR_MESSAGE_PATTERN);
  } else {
    if (evrMessageWrite(messageType, (evrMessage_tu *)message))
      evrMessageCheckSumError(EVR_MESSAGE_PATTERN);
  }
}

/*=============================================================================
 
  Name: evrEvent

  Abs: Called by the ErIrqHandler to handle event code 1 (fiducial) processing.

  Rem: Keep this routine to a minimum, so that CPU not blocked 
       too long processing each interrupt.
       
=============================================================================*/
void evrEvent(void *pCard, epicsInt16 eventNum, epicsUInt32 timeNum)
{
  if(eventNum==event_to_peek_fiducial) {
    fiducial_time_stamp[fiducial_time_stamp_ix]= timeNum;
    fiducial_time_stamp_ix++;
    if (fiducial_time_stamp_ix>=PEEK_PIPE_SIZE){fiducial_time_stamp_ix=0;}
  }

  if (eventNum == EVENT_FIDUCIAL) {
    lastfid = timeNum;
    if (readyForFiducial) {
      readyForFiducial = 0;
      evrMessageStart(EVR_MESSAGE_FIDUCIAL);
      epicsEventSignal(evrTaskEventSem);
    } else {
      evrMessageNoDataError(EVR_MESSAGE_FIDUCIAL);
    }
  }
  evrTimeCount((unsigned int)eventNum, (unsigned int) timeNum);
}

/*---------------------------------------------------------------------------
Following function can allow peeking at fiducial time stamp corresponding to
a watched event.
Argument:  next_event_to_watch...The event to watch from this call onwards
                   when this call terminates, the event ID that was being
                   watched thus far gets reported in this variable.
           Ticks:  returned fiducial associated with "next_event_to_watch"
                   from last call of thisfunction.
           peek_pipe_size: These many previous time stamps are returned. The Ticks array
                   in the calling program should be at least this much size.
                   (Note: at the most PEEK_PIPE_SIZE elements are supported. For the code
                   effeciency, keep PEEK_PIPE_SIZE to a small number.)
---------------------------------------------------------------------------*/
epicsUInt32 peek_fiducial(epicsUInt32* next_event_to_watch,
                          epicsUInt32 *Ticks,
                          epicsUInt32 peek_pipe_size)
{
    epicsUInt32 next_event;
    int upload_size,ix,ix_start,ix_end,local_fx;
    local_fx=fiducial_time_stamp_ix;

    upload_size=MIN(PEEK_PIPE_SIZE,peek_pipe_size);
    ix_start=local_fx+MAX(0,PEEK_PIPE_SIZE-peek_pipe_size);
    ix_end=ix_start+upload_size-1;
    for(ix=ix_start;ix<=ix_end;ix++){
        Ticks[ix_start-ix+upload_size-1]=fiducial_time_stamp[ix%PEEK_PIPE_SIZE];
    }

    fprintf(stdout,"Were Watching %d, Commanded to watch %d\n",event_to_peek_fiducial,*next_event_to_watch);
    next_event=  *next_event_to_watch;
    *next_event_to_watch=event_to_peek_fiducial;
    event_to_peek_fiducial=next_event;
    fprintf(stdout,"Will Watch %d, Reporting Watch Value %d\n",event_to_peek_fiducial,*next_event_to_watch);
    return(upload_size);
}


/*=============================================================================
                                                         
  Name: evrTask

  Abs:  This task performs processing for the fiducial and data.
  
  Rem:  It's started by evrInitialize after the EVR module is configured. 
    
=============================================================================*/
static int evrTask()
{  
  epicsEventWaitStatus status;
  epicsUInt32          mpsModifier;

  if (evrTimeInit(0,0)) {
    errlogPrintf("evrTask: Exit due to bad status from evrTimeInit\n");
    return -1;
  }
  for (;;)
  {
    readyForFiducial = 1;
    status = epicsEventWaitWithTimeout(evrTaskEventSem, EVR_TIMEOUT);

    evrMessageLap(EVR_MESSAGE_FIDUCIAL);
    if (status == epicsEventWaitOK) {
      evrPattern(0, &mpsModifier);/* N-3           */
      evrTime(mpsModifier);       /* Move pipeline */
      /* Call routines that the user has registered for 360hz processing */
      if (evrRWMutex_ps && (!epicsMutexLock(evrRWMutex_ps))) {
        evrFiducialFunc_ts *fid_ps =
          (evrFiducialFunc_ts *)ellFirst(&evrFiducialFuncList_s);
        while (fid_ps) {
          (*(fid_ps->func))(fid_ps->arg); /* Call user's routine */
          fid_ps = (evrFiducialFunc_ts *)ellNext(&fid_ps->node);
        }
        epicsMutexUnlock(evrRWMutex_ps);
      }   
      evrMessageEnd(EVR_MESSAGE_FIDUCIAL);
    /* If timeout or other error, process the data which will result in bad
       status since there is nothing to do.  Then advance the pipeline so
       that the bad status makes it from N-3 to N-2 then to N-2 and
       then to N. */
    } else {
      readyForFiducial = 0;
      evrPattern(1, &mpsModifier);/* N-3 */
      evrTime(mpsModifier);       /* N-2 */
      evrTime(mpsModifier);       /* N-1 */
      evrTime(mpsModifier);       /* N   */
      if (status != epicsEventWaitTimeout) {
        errlogPrintf("evrTask: Exit due to bad status from epicsEventWaitWithTimeout\n");
        return -1;
      }
    }
    /* Now do record processing */
    evrMessageStart(EVR_MESSAGE_PATTERN);
    epicsEventSignal(evrRecordEventSem);
  }
  return 0;
}

/*=============================================================================
                                                         
  Name: evrRecord

  Abs:  This task performs record processing for the fiducial and data.
  
  Rem:  It's started by evrInitialize after the EVR module is configured. 
    
=============================================================================*/
static int evrRecord()
{  
  for (;;)
  {
    if (epicsEventWait(evrRecordEventSem)) {
      errlogPrintf("evrRecord: Exit due to bad status from epicsEventWait\n");
      return -1;
    }
    evrMessageProcess(EVR_MESSAGE_PATTERN);
    evrMessageProcess(EVR_MESSAGE_FIDUCIAL);
    evrMessageEnd(EVR_MESSAGE_PATTERN);
  }
  return 0;
}

/*=============================================================================
                                                         
  Name: evrInitialize

  Abs:  Driver initialization.
  
  Rem:  Called during iocInit to initialize fiducial and data processing.
        Can also be called before iocInit.
    
=============================================================================*/
int evrInitialize()
{
  if (evrInitialized == 1) return 0;
  if (evrInitialized) {
    errlogPrintf("evrInitialize: error in previous call\n");
    return -1;
  }
  evrInitialized = -1;

#ifdef _X86_
  Get_evrTicksPerUsec_for_X86(); 
#endif

  /* Initialize BSA */
  if (bsaInit()) return -1;
  
  /* Create space for the pattern + diagnostics */
  if (evrMessageCreate(EVR_MESSAGE_PATTERN_NAME,
                       sizeof(evrMessagePattern_ts)) !=
      EVR_MESSAGE_PATTERN) return -1;
  
  /* Create space for the fiducial + diagnostics */
  if (evrMessageCreate(EVR_MESSAGE_FIDUCIAL_NAME, 0) !=
      EVR_MESSAGE_FIDUCIAL) return -1;
  
  /* Create the semaphores used by the ISR to wake up the evr tasks */
  evrTaskEventSem = epicsEventMustCreate(epicsEventEmpty);
  if (!evrTaskEventSem) {
    errlogPrintf("evrInitialize: unable to create the EVR task semaphore\n");
    return -1;
  }
  evrRecordEventSem = epicsEventMustCreate(epicsEventEmpty);
  if (!evrRecordEventSem) {
    errlogPrintf("evrInitialize: unable to create the EVR record task semaphore\n");
    return -1;
  }

  /* Create the fiducial function mutex and initialize link list*/
  evrRWMutex_ps = epicsMutexCreate();
  if (!evrRWMutex_ps) {
    errlogPrintf("evrInitialize: unable to create the EVR fiducial function mutex\n");
    return -1;
  }
  ellInit(&evrFiducialFuncList_s);
  
  /* Create the processing tasks */
  if (!epicsThreadCreate("evrTask", epicsThreadPriorityHigh+1,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)evrTask, 0)) {
    errlogPrintf("evrInitialize: unable to create the EVR task\n");
    return -1;
  }
  if (!epicsThreadCreate("evrRecord", epicsThreadPriorityScanHigh+10,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)evrRecord, 0)) {
    errlogPrintf("evrInitialize: unable to create the EVR record task\n");
    return -1;
  }
  
#ifdef EVR_DRIVER_SUPPORT
  /* Get first EVR in the list */
  pCard = ErGetCardStruct(0);
  if (!pCard) {
    errlogPrintf("evrInitialize: cannot find an EVR module\n");
  /* Register the ISR functions in this file with the EVR */
  } else {
    ErRegisterDevDBuffHandler(pCard, (DEV_DBUFF_FUNC)evrSend);
    ErEnableDBuff            (pCard, 1);
#ifndef NO_DBUF_IRQ
    ErDBuffIrq               (pCard, 1);
#endif
    ErRegisterEventHandler   (pCard->Cardno,    (USER_EVENT_FUNC)evrEvent);
  }
#endif	/* EVR_DRIVER_SUPPORT */
  evrInitialized = 1;
  return 0;
}

/*=============================================================================
                                                         
  Name: evrRegisterFiducial

  Abs:  Register a routine for evrTask to call after receipt of fiducial.
  
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        FIDUCIALFUNCTION    fiducialFunc Read      Fiducial Function
        void *              fiducialArg  Read      Fiducial Function Argument

  Rem:  The same function can be registered multiple times.

  Ret:  0 = OK, -1 = Invalid argument, mutex lock error, or
        memory allocation error
    
=============================================================================*/
int evrTimeRegister(FIDUCIALFUNCTION fiducialFunc, void * fiducialArg)
{
  evrFiducialFunc_ts *fiducialFunc_ps;

  if (!fiducialFunc) {
    errlogPrintf("evrTimeRegister: invalid fiducial function argument\n");
    return -1;
  }
  if (!evrRWMutex_ps) {
    errlogPrintf("evrTimeRegister: evrTimeRegister must be called after evrInitialize\n");
    return -1;
  }
  /* Get space for this function */
  if (!(fiducialFunc_ps = calloc(1,sizeof(evrFiducialFunc_ts)))) {
    errlogPrintf("evrTimeRegister: unable to allocate memory for the fiducial function\n");
    return -1;
  }
  fiducialFunc_ps->func = fiducialFunc;
  fiducialFunc_ps->arg  = fiducialArg;
  /* Add to list */  
  if (epicsMutexLock(evrRWMutex_ps)) {
    errlogPrintf("evrTimeRegister: unable to lock the EVR fiducial function mutex\n");
    return -1;
  }
  ellAdd(&evrFiducialFuncList_s, &fiducialFunc_ps->node);
  epicsMutexUnlock(evrRWMutex_ps);
  return 0;
}

LOCAL
registryFunctionRef peek_fiducialRef [] = {
    {"peek_fiducial", (REGISTRYFUNCTION)peek_fiducial}
};

LOCAL_RTN
void peek_fiducialRegistrar (void) {
    registryFunctionRefAdd (peek_fiducialRef, NELEMENTS(peek_fiducialRef));
}

epicsExportRegistrar (peek_fiducialRegistrar);

