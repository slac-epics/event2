/*=============================================================================
 
  Name: bsa.h

  Abs:  This include file contains external prototypes for Beam Synchronous
        Acquisition processing routines for apps that prefer to do BSA
        processing without records.

  Auth: 07 JAN-2009, saa 
 
-----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------- 
  Mod:  (newest to oldest)  
        DD-MMM-YYYY, Your Name:
           More changes ... The ordering of the revision history 
           should be such that the NEWEST changes are at the HEAD of
           the list.

        01-Mar-2010, Sonya Hoobler:
           As part of changes to "maximize" status and severity, added
           argument to bsaSecnAvg: epicsEnum16 secnStat
 
=============================================================================*/

#ifndef INCbsaH
#define INCbsaH 

#ifdef __cplusplus
extern "C" {
#endif

#include    "epicsTime.h"          /* epicsTimeStamp */
  
int bsaSecnAvg(epicsTimeStamp *secnTime_ps,
               double          secnVal,
               epicsEnum16     secnStat,
               epicsEnum16     secnSevr,
               int             noAveraging,
               void           *dev_ps);
  
int bsaSecnInit(char          *secnName,
                int            noAverage,
                void         **dev_pps);

int bsaInit(void);


#ifdef __cplusplus
}
#endif

#endif /*INCbsaH*/
