/*=============================================================================
 
  Name: evrTime.h

  Abs:  This include file contains external prototypes for EVR 
        timestamp/pattern routines that allow access to the EVR timestamp/pattern
        table (timestamp/pattern received in the EVG broadcast) 

  Auth: 17 NOV-2006, drogind created 
 
-----------------------------------------------------------------------------*/
/* Following are contets of "copyright_SLAC.h" */

#ifndef COPYRIGHT_SLAC_H
#define COPYRIGHT_SLAC_H
/*
**
**                                Copyright 2004
**                                      by
**                         The Board of Trustees of the
**                       Leland Stanford Junior University.
**                              All rights reserved.
**
**         Work supported by the U.S. Department of Energy under contract
**       DE-AC03-76SF00515.
**
**                               Disclaimer Notice
**
**        The items furnished herewith were developed under the sponsorship
**   of the U.S. Government.  Neither the U.S., nor the U.S. D.O.E., nor the
**   Leland Stanford Junior University, nor their employees, makes any war-
**   ranty, express or implied, or assumes any liability or responsibility
**   for accuracy, completeness or usefulness of any information, apparatus,
**   product or process disclosed, or represents that its use will not in-
**   fringe privately-owned rights.  Mention of any product, its manufactur-
**   er, or suppliers shall not, nor is it intended to, imply approval, dis-
**   approval, or fitness for any particular use.  The U.S. and the Univer-
**   sity at all times retain the right to use and disseminate the furnished
**   items for any purpose whatsoever.                       Notice 91 02 01
**
*/
#endif

/*----------------------------------------------------------------------------- 
  Mod:  (newest to oldest)  
        DD-MMM-YYYY, My Name:
           Changed such and such to so and so. etc. etc.
        DD-MMM-YYYY, Your Name:
           More changes ... The ordering of the revision history 
           should be such that the NEWEST changes are at the HEAD of
           the list.
 
=============================================================================*/

#ifndef INCevrTimeH
#define INCevrTimeH 

#ifdef __cplusplus
extern "C" {
#endif

#include    "epicsTime.h"          /* epicsTimeStamp       */
#include    "registryFunction.h"   /* REGISTRYFUNCTION     */
  
/* Masks used to decode pulse ID from the nsec part of the timestamp   */
#define UPPER_15_BIT_MASK       (0xFFFE0000)    /* (2^32)-1 - (2^17)-1 */
#define LOWER_17_BIT_MASK       (0x0001FFFF)    /* (2^17)-1            */
/* Pulse ID Definitions */
#define PULSEID(time)           ((time).nsec & LOWER_17_BIT_MASK)
#define PULSEID_BIT17           (0x00020000)    /* Bit up from pulse ID*/
#define PULSEID_INVALID         (0x0001FFFF)    /* Invalid Pulse ID    */
#define PULSEID_MAX             (0x0001FFDF)    /* Rollover value      */
#define PULSEID_RESYNC          (0x0001E000)    /* Resync Pulse ID     */
#define NSEC_PER_SEC            1E9             /* # nsec in one sec   */
  
#define MODULO720_SECS          2               /* # sec for MODULO720 */
#define TIMESLOT_MIN            1               /* Minimum time slot   */
#define TIMESLOT_MAX            6               /* Maximum time slot   */
#define TIMESLOT_DIFF           3               /* Timeslot difference */

#define MAX_TS_QUEUE           512              /* # timestamps queued per event */
#define MAX_TS_QUEUE_MASK      511

#define MAX_EV_TRIGGERS        12

extern int lastfid;                             /* The last known fiducial, set by interrupt handler */

/*
 * A few fiducial helper definitions.
 * FID_ROLL(a, b) is true if we have rolled over from fiducial a to fiducial b.  (That is, a
 * is large, and b is small.)
 * FID_GT(a, b) is true if fiducial a is greater than fiducial b, accounting for rollovers.
 * FID_DIFF(a, b) is the difference between two fiducials, accounting for rollovers.
 */
#define FID_MAX        0x1ffe0
#define FID_ROLL_LO    0x00200
#define FID_ROLL_HI    (FID_MAX-FID_ROLL_LO)
#define FID_ROLL(a,b)  ((b) < FID_ROLL_LO && (a) > FID_ROLL_HI)
#define FID_GT(a,b)    (FID_ROLL(b, a) || ((a) > (b) && !FID_ROLL(a, b)))
#define FID_DIFF(a,b)  ((FID_ROLL(b, a) ? FID_MAX : 0) + (a) - (b) - (FID_ROLL(a, b) ? FID_MAX : 0))
  
/* For time ID */
typedef enum {
  evrTimeCurrent=0, evrTimeNext1=1, evrTimeNext2=2, evrTimeNext3=3, evrTimeActive
} evrTimeId_te;
#define MAX_EVR_TIME  4

#define MAX_EDEF_TIME      16
#define MAX_EDEF_TIME_MASK 15
#undef BSA_DEBUG
#ifdef BSA_DEBUG
extern int bsa_debug_mask;
#endif

/* For modifier array */
#define MAX_EVR_MODIFIER  6
typedef epicsUInt32 evrModifier_ta[MAX_EVR_MODIFIER];
  
/* Event codes - see mrfCommon.h for reserved internal event codes      */
#define EVENT_FIDUCIAL          1        /* Fiducial event code         */
#define EVENT_EXTERNAL_TRIG     100      /* External trigger event code */
#define EVENT_MODULO720         121      /* Modulo 720 event code       */
#define EVENT_MPG               122      /* MPG update event code       */
#define EVENT_MODULO36_MIN      201      /* Min modulo 36 event code    */
#define EVENT_MODULO36_MAX      236      /* Max modulo 36 event code    */
#define MODULO36_MAX            36       /* # modulo 36 event codes     */
  
typedef void (*FIDUCIALFUNCTION)(void *arg);

int evrInitialize         (void);
int evrTimeRegister       (FIDUCIALFUNCTION fiducialFunc,
                           void *           fiducialArg);
int evrTimeGetFromPipeline(epicsTimeStamp  *epicsTime_ps,
                           evrTimeId_te     id,
                           evrModifier_ta   modifier_a, 
                           unsigned long   *patternStatus_p,
                           unsigned long   *edefAvgDoneMask_p,
                           unsigned long   *edefMinorMask_p,
                           unsigned long   *edefMajorMask_p);
#if 0
int evrTimeGetFromEdef    (unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p);
#endif
int evrTimeGetFromEdefTime(unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p);
int evrTimeGet            (epicsTimeStamp  *epicsTime_ps,
                           unsigned int     eventCode);
int evrTimeGetFifo        (epicsTimeStamp     *epicsTime_ps,
                           unsigned int        eventCode,
                           unsigned long long *idx,
                           int                 incr);
int evrTimePutPulseID     (epicsTimeStamp  *epicsTime_ps,
                           unsigned int     pulseID);
/* Routines used only by event module and Mpg application */
#ifdef INCevrMessageH
int evrTimeInit           (epicsInt32   firstTimeSlotIn,
                           epicsInt32   secondTimeSlotIn);
int evrTime               (epicsUInt32  mpsModifier);
int evrTimeCount          (unsigned int eventCode, unsigned int fiducial);
int evrTimePatternPutStart(evrMessagePattern_ts **pattern_pps,
                           unsigned long        **timeslot_pp,
                           unsigned long        **patternStatus_pp,
                           epicsTimeStamp       **mod720time_pps);
int evrTimePatternPutEnd  (int modulo720Flag);
#endif
/*  Following function can be called by other Epics modules to get a fast
   access to event time stamp.
---------------------------------------------------------------------------
Following finction can allow peeking at fiducial time stamp corresponding to
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
---------------------------------------------------------------------------
*/
#define PEEK_PIPE_SIZE  10
epicsUInt32 peek_fiducial (epicsUInt32*next_event_to_watch,epicsUInt32 *Ticks,epicsUInt32 );

extern int fiddbg; /* Temporary fiducial debug variable. */

#ifdef __cplusplus
}
#endif

#endif /*INCevrTimeH*/
