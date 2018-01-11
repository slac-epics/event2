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
#include <stdint.h>
#include <dbScan.h>             /* for post_event         */
#include "drvSup.h" 		/* for DRVSUPFN           */
#include "errlog.h"		/* for errlogPrintf       */
#include "epicsExport.h" 	/* for epicsExportAddress */
#include "epicsEvent.h" 	/* for epicsEvent*        */
#include "epicsThread.h" 	/* for epicsThreadCreate  */
#include "epicsMessageQueue.h"
#include "evrMessage.h"		/* for evrMessageCreate   */
#include "evrTime.h"		/* for evrTimeCount       */
#include "evrPattern.h"		/* for evrPattern         */
#include "bsa.h"		/* for bsaInit            */
#include "drvMrfEr.h"		/* for ErRegisterDevDBuffHandler */
#include "devMrfEr.h"		/* for ErRegisterEventHandler    */
#include <epicsVersion.h>

#ifdef DIAG_TIMER
#include "HiResTime.h"
#else
#include "HiResTimeStub.h"
#endif	/* DIAG_TIMER */
#include<timingFifoApi.h>

#ifdef __rtems__
#define EVR_TIMEOUT     (0.06)  /* Timeout in sec waiting for 360hz input. */
#else
#define EVR_TIMEOUT     (2)     /* Timeout in sec waiting for 360hz input. */
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

static epicsMessageQueueId  eventTaskQueue;
static ErCardStruct    *pCard             = NULL;  /* EVR card pointer    */
static epicsEventId     evrTaskEventSem   = NULL;  /* evr task semaphore  */
static epicsEventId     evrRecordEventSem = NULL;  /* evr record task sem */
static int readyForFiducial = 1;                   /* evrTask ready for new fiducial */
static int evrInitialized = 0;                     /* evrInitialize performed        */
int lastfid = -1;                           /* Last fiducial seen             */

/* Fiducial User Function List */
typedef struct {
  ELLNODE node;
  FIDUCIALFUNCTION func;
  void * arg;

} evrFiducialFunc_ts;

typedef struct {
  epicsInt16 eventNum;
} EventMessage;

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
	  /* TODO: Mv FormFactorToString() from drvLinuxEvr.c to a file used
	   * in both linux and RTEMS and call it here. */
      printf("Pattern data from %s card %d\n",
             (pCard->FormFactor==1)?"PMC":(pCard->FormFactor==2)?"Embedded":
             (pCard->FormFactor==0xF)?"SLAC":"VME",
             pCard->Cardno);
      /* Get pulse ID from timestamp. */
      evrTimeGetFromPipeline(&currentTime, evrTimeCurrent, 0, 0, 0, 0, 0);
      pulseIDfromTime = PULSEID(currentTime);
      /* Get pulse ID from EVR seconds register. */
      pulseIDfromEvr = ErGetSecondsSR(pCard);
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
  Name: evrGetLastFiducial()

  Abs: A simple routine to return the fiducial id for the most
  	   recently received EVENT_FIDUCIAL.
=============================================================================*/
int evrGetLastFiducial( )
{
	return lastfid;
}

/*=============================================================================
  Name: timingGetLastFiducial()

  Abs: A simple routine to return the fiducial id for the most
  	   recently received EVENT_FIDUCIAL, but as a long long for timesync!
=============================================================================*/
TimingPulseId timingGetLastFiducial( )
{
    if (lastfid == PULSEID_INVALID)
        return TIMING_PULSEID_INVALID;
    else
        return (TimingPulseId) lastfid;
}

/*=============================================================================
  Name: timingGetFiducialForTimeStamp()

  Abs: timingGetFiducialForTimeStamp returns the 64 bit fiducial that corresponds to the specified timestamp.
	   If the timing module cannot determine the correct fiducial, it returns TIMING_PULSEID_INVALID.
=============================================================================*/
extern TimingPulseId timingGetFiducialForTimeStamp( epicsTimeStamp timeStamp )
{
	epicsUInt32		fiducial = timeStamp.nsec & PULSEID_INVALID;
    if ( fiducial == PULSEID_INVALID )
        return TIMING_PULSEID_INVALID;
    else
        return (TimingPulseId) fiducial;
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
  epicsUInt32 evrClockCounter;
  unsigned int messageType = ((evrMessageHeader_ts *)message)->type;

  ErGetTicks(0, &evrClockCounter);

  /* Look for error from the driver or the wrong message size */
  if ((pCard && ((ErCardStruct *)pCard)->DBuffError) ||
      (messageSize != sizeof(evrMessagePattern_ts))) {
    evrMessageCheckSumError(EVR_MESSAGE_PATTERN);
  }
  else {
    if (evrMessageWrite(messageType, (evrMessage_tu *)message))
      evrMessageCheckSumError(EVR_MESSAGE_PATTERN);
    else
      evrMessageClockCounter(EVR_MESSAGE_PATTERN, evrClockCounter);
  }
}

/*=============================================================================
 
  Name: evrEvent

  Abs: Called by the ErIrqHandler to handle event code processing.
       Handles both event code 1, the fiducial, and also other event codes.
	   Under RTEMS, event code 1 is processed first.
	   No such guarantee for RedHat linux

  Rem: Keep this routine to a minimum, so that CPU not blocked 
       too long processing each interrupt.
       
=============================================================================*/
void evrEvent(int cardNo, epicsInt16 eventNum, epicsUInt32 timeNum)
{
  t_HiResTime			hiResTsc	= GetHiResTicks();
  if(eventNum == EVENT_FIDUCIAL)
    lastfid = timeNum;
  if(eventNum==event_to_peek_fiducial) {
    fiducial_time_stamp[fiducial_time_stamp_ix]= timeNum;
    fiducial_time_stamp_ix++;
    if (fiducial_time_stamp_ix>=PEEK_PIPE_SIZE){fiducial_time_stamp_ix=0;}
  }

  /* This needs to be done first, or else we might have to wait a full fiducial! */
  evrTimeCount((unsigned int)eventNum, (unsigned int) timeNum, hiResTsc );

  if (eventNum == EVENT_FIDUCIAL) {
    if (readyForFiducial) {
      epicsUInt32  evrClockCounter;                                      // NEW
      readyForFiducial = 0;
      ErGetTicks(0, &evrClockCounter);                                   // NEW
      evrMessageClockCounter(EVR_MESSAGE_FIDUCIAL, evrClockCounter);     // NEW
      evrMessageStart(EVR_MESSAGE_FIDUCIAL);
      epicsEventSignal(evrTaskEventSem);
    } else {
      evrMessageNoDataError(EVR_MESSAGE_FIDUCIAL);
    }
  } else {
	  /*---------------------
	   * Schedule processing for any event-driven records
	   */
  	  EventMessage eventMessage;
	  eventMessage.eventNum  = eventNum;
	  epicsMessageQueueSend( eventTaskQueue, &eventMessage, sizeof(eventMessage) );
  }
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
  int                  messagePending;
  EventMessage         eventMessage;

  if (evrTimeInit(0,0)) {
    errlogPrintf("evrTask: Exit due to bad status from evrTimeInit\n");
    return -1;
  }

  if(!pCard) {
    errlogPrintf("evrTask: could not find an EVR module\n");
    return -1;
  }

  eventMessage.eventNum  = EVENT_FIDUCIAL;

  for (;;)
  {
    readyForFiducial = 1;
    status = epicsEventWaitWithTimeout(evrTaskEventSem, EVR_TIMEOUT);

    evrMessageLap(EVR_MESSAGE_FIDUCIAL);
    if (status == epicsEventWaitOK) {
	  /* Note: evrPattern locks evrTimeRWMutex but releases it */
      evrPattern(0, &mpsModifier);/* N-3           */
	  /* Note: evrTime also locks evrTimeRWMutex but under linux evrEventTask could run before it takes the lock */
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

      /* Post fiducial eventMessage to eventTaskQueue */
      epicsMessageQueueSend( eventTaskQueue, &eventMessage, sizeof(eventMessage) );

      messagePending = epicsMessageQueuePending(eventTaskQueue);
      evrMessageQ(EVR_MESSAGE_FIDUCIAL, messagePending);

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

#if EPICS_VERSION > 3 || (EPICS_VERSION == 3 && EPICS_REVISION >= 15)
static EVENTPVT evtlist[256] = { NULL };

static void myPostEvent(int evt)
{
    if (evt < 1 || evt > 255)
        return;
    if (!evtlist[evt]) {
        char name[10];
        sprintf(name, "%d", evt);
        evtlist[evt] = eventNameToHandle(name);
    }
    postEvent(evtlist[evt]);
}
#endif

/*
 * This thread drains the eventTaskQueue
 */
static int evrEventTask(void)
{
	EventMessage eventMessage;

    for(;;)
	{
		/* Wait for the next EventMessage */
		epicsMessageQueueReceive(eventTaskQueue, &eventMessage, sizeof(eventMessage));

		/* Update the event code timestamps */
		evrTimeEventProcessing(eventMessage.eventNum);

		/* Tell EPICS to post the event code */
#if EPICS_VERSION < 3 || (EPICS_VERSION == 3 && EPICS_REVISION < 15)
		post_event(eventMessage.eventNum);
#else
                myPostEvent(eventMessage.eventNum);
#endif

		/* pCard cannot be NULL since the only entities which send messages are
		*  - the evrTask which bails out if pCard is NULL
		*  - the evrEvent handler which is not installed if pCard is NULL
		*/
		if (	eventMessage.eventNum >= 0
			&&	eventMessage.eventNum < sizeof(pCard->IoScanPvt)/sizeof(pCard->IoScanPvt[0]) )
		{
			/* Add a process request to the scanIO queue */
			scanIoRequest( pCard->IoScanPvt[eventMessage.eventNum] );
		}
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

#if defined(_X86_) || defined(_X86_64_)
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

  pCard = ErGetCardStruct(0);
  if (!pCard)
      pCard = ErGetCardStruct(1);

  eventTaskQueue = epicsMessageQueueCreate(256, sizeof(EventMessage));
  
  /* Create the processing tasks */
  if (!epicsThreadCreate("evrTask", epicsThreadPriorityHigh+1,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)evrTask, 0)) {
    errlogPrintf("evrInitialize: unable to create the EVR task\n");
    return -1;
  }

  if(!epicsThreadCreate("evrEventTask", epicsThreadPriorityHigh,
                        epicsThreadGetStackSize(epicsThreadStackMedium),
                        (EPICSTHREADFUNC)evrEventTask,0)) {
    errlogPrintf("evrInitialize: unable to create the evrEvent task\n");
    return -1;
  }

  if (!epicsThreadCreate("evrRecord", epicsThreadPriorityScanHigh+10,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)evrRecord, 0)) {
    errlogPrintf("evrInitialize: unable to create the EVR record task\n");
    return -1;
  }
  
  if (!pCard) {
    errlogPrintf("evrInitialize: cannot find an EVR module\n");
  /* Register the ISR functions in this file with the EVR */
  } else {
    ErRegisterDevDBuffHandler(pCard, (DEV_DBUFF_FUNC)evrSend);
#if 0
    ErEnableDBuff            (pCard, 1); // Not for event2!  It's always enabled!
#endif
#ifndef NO_DBUF_IRQ
    ErDBuffIrq               (pCard, 1);
#endif
    ErRegisterEventHandler   (pCard->Cardno,    (USER_EVENT_FUNC)evrEvent);
  }
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

static
registryFunctionRef peek_fiducialRef [] = {
    {"peek_fiducial", (REGISTRYFUNCTION)peek_fiducial}
};

static
void peek_fiducialRegistrar (void) {
    registryFunctionRefAdd (peek_fiducialRef, NELEMENTS(peek_fiducialRef));
}

epicsExportRegistrar (peek_fiducialRegistrar);

void timingPrintNow()
{
    printf("%d\n", lastfid);
}
