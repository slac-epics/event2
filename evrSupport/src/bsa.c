/*=============================================================================
 
  Name: bsa.c

  Abs: This device support for beam synchronous acquisition records via
       EVG/EVR EDEFS
           bsaSecnAvg        - BSA Processing 
           bsaSecnInit       - BSA Processing Initialization
           read_bsa          - BSA Record Value Update
           init_bsa_record   - Init BSA Record Device Support
           get_ioint_info    - Get IO Intr pointer
           init_ao_record    - Init AO Record Device Support
           write_ao          - Update Data for BSA Record 

  Auth:  
  Rev:  

  ---------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */
 
/*-----------------------------------------------------------------------------
 
  Mod:  12 Feb 2010     S. Hoobler (sonya)

	To set SEVR/STAT on BSA PVs:

	  Structure bsa_ts:
	    (1) Add epicsEnum16 sevr and stat fields.

	  Routine bsaSecnAvg:
	    (1) Add epicsEnum16 secnStat to the argument list.
	    (2) In the code that initializes the average under "if
	    ((bsa_ps->avgcnt == 1) || noAverage)", set sevr and stat 
	    of the bsa_ts structure to the input secnStat and secnSevr.
	    (3) In the code that calculates running avg, add logic to 
            compute running max of stat and sev

	  Routine read_bsa:
	    (1) Add local variables dstat and dsevr to copy stat and sevr from bsa_ts.
	    (2) In the logic that calls recGblSetSevr, add an "else" that calls 
                recGblSetSevr with the local variables. 

	 Routine write_ao:
	    (1) Replace dbGetTimeStamp with dbGetField in order to get stat, sevr, 
	    and time of input record.
	    (2) In bsaSecnAvg call, replace pao->nsev with severity of input record
	    and add stat of input record.

	 README file of the event module:
	  (1) In the section that describes calling BSA routines directly
	  (II-2), update the API description to include the new argument.

 
=============================================================================*/

#include <string.h>        /* strcmp */
#include <stdlib.h>        /* calloc */
#include <math.h>          /* sqrt   */

#include "bsaRecord.h"        /* for struct bsaRecord      */
#include "aoRecord.h"         /* for struct aoRecord       */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "epicsTime.h"        /* epicsTimeStamp */
#include "epicsMutex.h"       /* epicsMutexId   */
#include "dbAccess.h"         /* dbGetTimeStamp */
#include "devSup.h"           /* for dset and DEVSUPFUN    */
#include "devLib.h"           /* for S_dev_noMemory        */
#include "recGbl.h"           /* for recGblSetSevr         */
#include "ellLib.h"           /* linked list    */
#include "dbScan.h"           /* IOSCANPVT      */
#include "alarm.h"            /* INVALID_ALARM  */
#include "evrTime.h"          /* evrTimeGetFromEdef        */
#include "evrPattern.h"       /* EDEF_MAX                  */
#include "bsa.h"              /* prototypes in this file   */
#ifdef BSA_DEBUG
#include<stdio.h>
#include<iocsh.h>
#endif

/* BSA information for one device, one EDEF */
typedef struct {

  /* Results of Averaging */
  double              val;       /* average value     */
  double              rms;       /* RMS of above      */
  int                 cnt;       /* # in average      */
  int                 readcnt;   /* # total readouts  */
  epicsTimeStamp      time;      /* time of average   */
  unsigned long       nochange;  /* Same time stamp counter */
  unsigned long       noread;    /* Data not read counter   */
  int                 readFlag;  /* Data read flag    */
  int                 reset;     /* Reset waveforms   */
  /* Intermediate Values */
  double              avg;       /* average  so far   */
  double              var;       /* variance so far   */
  int                 avgcnt;    /* count    so far   */
  epicsTimeStamp      timeData;  /* latest input time */
  epicsTimeStamp      timeInit;  /* init         time */
  IOSCANPVT           ioscanpvt; /* to process records using above fields */
  epicsEnum16         stat;      /* max status so far */
  epicsEnum16         sevr;      /* max severity so far*/

} bsa_ts;

/* BSA devices */
typedef struct {
  ELLNODE node;
  char    name[PVNAME_STRINGSZ];
  int     noAverage;
  bsa_ts  bsa_as[EDEF_MAX];

} bsaDevice_ts;

ELLLIST bsaDeviceList_s;
static epicsMutexId bsaRWMutex_ps = 0; 

/*=============================================================================

  Name: bsaSecnAvg

  Abs:  Beam Synchronous Acquisition Processing
        Computes BSA device running average and RMS values for all EDEFs
        Computes running maximum status and severity

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsTimeStamp *    secnTime_ps Read       Data timestamp
        double              secnVal     Read       Data value
        epicsEnum16         secnStat    Read       Data status
        epicsEnum16         secnSevr    Read       Data severity
	int                 noAveraging 
        void *              dev_ps      Read/Write BSA Device Structure

  Rem:  
      
  Ret:  0 = OK, -1 = Mutex problem or bad status from evrTimeGetFromEdef.

==============================================================================*/

int bsaSecnAvg(epicsTimeStamp *secnTime_ps,
               double          secnVal,
               epicsEnum16     secnStat,
               epicsEnum16     secnSevr,
               int             noAveraging,
               void           *dev_ps)
{
  epicsTimeStamp  edefTimeInit_s;
  int             edefAvgDone;
  int             noAverage;
  int             idx;
  int             status = 0;
  epicsEnum16     edefSevr;
  bsa_ts         *bsa_ps;
#ifdef BSA_DEBUG
  char           *name = ((bsaDevice_ts *)dev_ps)->name;
#endif
  
  if ((!bsaRWMutex_ps) || epicsMutexLock(bsaRWMutex_ps) || (!dev_ps))
    return -1;
  /* Request BSA processing for matching EDEFs */
  noAverage = ((bsaDevice_ts *)dev_ps)->noAverage;
  for (idx = 0; idx < EDEF_MAX; idx++) {
    /* Get EDEF information. */
    if (evrTimeGetFromEdefTime(idx, secnTime_ps, &edefTimeInit_s,
                               &edefAvgDone, &edefSevr)) {
      continue;
    }
    
    bsa_ps = &((bsaDevice_ts *)dev_ps)->bsa_as[idx];
    /* Check if the EDEF has initialized and wipe out old values if it has */
    if ((edefTimeInit_s.secPastEpoch != bsa_ps->timeInit.secPastEpoch) ||
        (edefTimeInit_s.secPastEpoch != bsa_ps->timeInit.secPastEpoch)) {
#ifdef BSA_DEBUG
        if (bsa_debug_mask & (1 << idx))
            printf("%08x:%08x EDEF%d %s reset, old epoch %08x:%08x\n",
                   edefTimeInit_s.secPastEpoch, edefTimeInit_s.nsec, idx, name,
                   bsa_ps->timeInit.secPastEpoch, bsa_ps->timeInit.nsec);
#endif
      bsa_ps->timeInit = edefTimeInit_s;
      bsa_ps->avg    = 0.0;
      bsa_ps->var    = 0.0;
      bsa_ps->avgcnt = 0;
      bsa_ps->readcnt= 0;
      if (bsa_ps->readFlag) bsa_ps->noread++;
      bsa_ps->readFlag = 0;
      bsa_ps->reset    = 1;
    }
    /* Ignore data that hasn't changed since last time */
    if ((secnTime_ps->secPastEpoch == bsa_ps->timeData.secPastEpoch) &&
        (secnTime_ps->nsec         == bsa_ps->timeData.nsec)) {
      bsa_ps->nochange++;
    } else {
      bsa_ps->timeData = *secnTime_ps;
      bsa_ps->readcnt++;
    
      /* Include this value in the average if it's OK with the EDEF */
      if (secnSevr < edefSevr) {
	bsa_ps->avgcnt++;

	/* now start the averaging */
	/* first time thru for new cycle; reset previous avg, variance */
	
	/* compute running avg and variance                      */
	/*        This is translated from REF_RMX_BPM:CUM.F86.   */
	/*                                                       */
	/*        CUM computes VAR as the sample variance:       */ 
	/*          VAR = (SUMSQ - SUM*SUM/N)/(N-1)              */
	/*          where SUM = sum of the N values, and         */
	/*           SUMSQ = sum of the squares of the N values. */
	/*                                                       */
	/*        Note that CUM's method of computing VAR avoids */
	/*        possible loss of significance.                 */
	/*                                                       */
	/*  Compute running maximum status and severity          */
                                                       
	if ((bsa_ps->avgcnt == 1) || noAverage) {
          bsa_ps->avgcnt = 1;
	  bsa_ps->avg    = secnVal;
          bsa_ps->var    = 0.0;
	  bsa_ps->stat   = secnStat;
	  bsa_ps->sevr   = secnSevr;
	} 
	else {
	  int avgcnt_1 = bsa_ps->avgcnt-1;
          int avgcnt_2 = bsa_ps->avgcnt-2;
	  double diff  = secnVal - bsa_ps->avg;
	  bsa_ps->avg += diff/(double)bsa_ps->avgcnt;
	  diff        /= (double)avgcnt_1;
	  bsa_ps->var  = ((double)avgcnt_2*(bsa_ps->var/(double)avgcnt_1)) +
                         ((double)bsa_ps->avgcnt*diff*diff);
	    if (secnSevr > bsa_ps->sevr) {
                bsa_ps->sevr = secnSevr;
                bsa_ps->stat = secnStat;
            }
	}
      } /* if good, include in averaging */
    }
    /* Finish up calcs when the average is done and force record processing */
    if (edefAvgDone) { /* values when avg is done */
      bsa_ps->val  = bsa_ps->avg;
      bsa_ps->cnt  = bsa_ps->avgcnt;
      bsa_ps->time = bsa_ps->timeData;
      if (bsa_ps->avgcnt <= 1) {
        bsa_ps->rms = 0.0;
      } else {
        bsa_ps->rms = bsa_ps->var/(double)bsa_ps->avgcnt;
        bsa_ps->rms = sqrt(bsa_ps->rms);
      }
      bsa_ps->avgcnt = 0;
      bsa_ps->avg    = 0;
      if (bsa_ps->ioscanpvt) {
        if (bsa_ps->readFlag) bsa_ps->noread++;
        else                  bsa_ps->readFlag = 1;
        scanIoRequest(bsa_ps->ioscanpvt);
#ifdef BSA_DEBUG
        if (bsa_debug_mask & (1 << idx))
            printf("%08x:%08x EDEF%d %s signaled.\n",
                   bsa_ps->time.secPastEpoch, bsa_ps->time.nsec, idx, name);
#endif
      }
    }
  }
  epicsMutexUnlock(bsaRWMutex_ps);
  return status;
}

/*=============================================================================

  Name: bsaSecnInit

  Abs:  Beam Synchronous Acquisition Processing Initialization for a Device

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        char *              secnName    Read       BSA Device Name
        int                 noAverage   Read       Skip averaging (EVG IOC only)
        void **             dev_pps     Write      BSA Device Structure Pointer
        
  Rem:  
      
  Ret:  0 = OK, -1 = Mutex lock or memory allocation error

==============================================================================*/

int bsaSecnInit(char  *secnName,
                int    noAverage,
                void **dev_pps)
{
  bsaDevice_ts *dev_ps = 0;
  
  if ((!bsaRWMutex_ps) || epicsMutexLock(bsaRWMutex_ps))
    return -1;
  /* Check if device name is already registered. */
  dev_ps = (bsaDevice_ts *)ellFirst(&bsaDeviceList_s);
  while(dev_ps) {
    if(strcmp(dev_ps->name, secnName)==0) break;
    dev_ps = (bsaDevice_ts *)ellNext(&dev_ps->node);
  }
  if (!dev_ps) {
    dev_ps = calloc(1,sizeof(bsaDevice_ts));
    if (dev_ps) {
      strcpy(dev_ps->name, secnName);
      ellAdd(&bsaDeviceList_s,&dev_ps->node);
    }
  }
  epicsMutexUnlock(bsaRWMutex_ps);
  *dev_pps = dev_ps;
  if (dev_ps) {
    if (noAverage) dev_ps->noAverage = 1;
    return 0;
  }
  return -1;
}

/*=============================================================================

  Name: bsaInit

  Abs:  Beam Synchronous Acquisition Processing Global Initialization

  Args: None.
        
  Rem:  
      
  Ret:  0 = OK, -1 = Mutex creation error

==============================================================================*/

int bsaInit()
{
  if (!bsaRWMutex_ps) {
    bsaRWMutex_ps = epicsMutexCreate();
    if (bsaRWMutex_ps) {
      ellInit(&bsaDeviceList_s);
    } else {
      return -1;
    }
  }
  return 0;
}

/*=============================================================================

  Name: read_bsa

  Abs:  Beam Synchronous Acquisition Record Update
        Updates average and RMS values of Secondary
        for PRIM:LOCA:UNIT:$SECN$MDID.VAL 

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        bsaRecord *         pbsa        Read/Write BSA Record

  Rem:  
      
  Ret:  0 = OK

==============================================================================*/

static long read_bsa(bsaRecord *pbsa)
{
  bsa_ts *bsa_ps = (bsa_ts *)pbsa->dpvt;
  short reset    = 0;
  int   noread   = 1;
  epicsEnum16 dstat = UDF_ALARM;      /* data status */
  epicsEnum16 dsevr = INVALID_ALARM;  /* data severity */

  /* Lock and update */
#ifdef BSA_DEBUG
  if (bsa_debug_mask & (1 << (pbsa->edef - 1)))
      printf("BSA%d %s res=%d, read=%d, reset=%d\n", pbsa->edef - 1, pbsa->name, pbsa->res, bsa_ps->readFlag, bsa_ps->reset);
#endif
  if (bsa_ps && bsaRWMutex_ps && (!epicsMutexLock(bsaRWMutex_ps))) {
    if (pbsa->res) {
      pbsa->res        = 0;
      bsa_ps->nochange = 0;
      bsa_ps->noread   = 0;
      bsa_ps->readcnt  = 0;
    }
    if (bsa_ps->readFlag) {
      bsa_ps->readFlag = 0;
      noread           = 0;
      pbsa->val  = bsa_ps->val;
      pbsa->rms  = bsa_ps->rms;
      pbsa->cnt  = bsa_ps->cnt;
      pbsa->time = bsa_ps->time;
      pbsa->noch = bsa_ps->nochange;
      pbsa->nore = bsa_ps->noread;
      pbsa->rcnt = bsa_ps->readcnt;
      dstat      = bsa_ps->stat;
      dsevr      = bsa_ps->sevr;
    }
    if (bsa_ps->reset) {
      bsa_ps->reset = 0;
      reset         = 1;
    }
    epicsMutexUnlock(bsaRWMutex_ps);
  }
  /* Read alarm if there was nothing to read.
     Soft alarm if there were no valid inputs to the average.
     Else set stat/sevr to max values */ 
  if (noread) {
    pbsa->val  = 0.0;
    pbsa->rms  = 0.0;
    pbsa->cnt  = 0;
    epicsTimeGetEvent(&pbsa->time, 0);
    recGblSetSevr(pbsa,READ_ALARM,INVALID_ALARM);
  } else if (pbsa->cnt == 0) {
    recGblSetSevr(pbsa,SOFT_ALARM,INVALID_ALARM);
  } else recGblSetSevr(pbsa,dstat,dsevr);

  /* Reset compress records if requested */
  if (reset) {
    dbPutLink(&pbsa->vres, DBR_SHORT, &reset, 1);
    dbPutLink(&pbsa->rres, DBR_SHORT, &reset, 1);
    dbPutLink(&pbsa->cres, DBR_SHORT, &reset, 1);
  }
  return 0;
}

static long init_record(dbCommon *prec, int noAverage, DBLINK *link)
{
  if (link->type != INST_IO) {
    errlogPrintf("init_record (%s): INP is not INST_IO\n", prec->name);
    return S_db_badField;
  }
  bsaSecnInit(link->value.instio.string, noAverage, &prec->dpvt);    
  if (!prec->dpvt) {
    errlogPrintf("init_record (%s): cannot allocate DPVT\n", prec->name);
    return S_dev_noMemory;
  }
  return 0;
}

static long init_bsa_record(bsaRecord *pbsa)
{
  long status = init_record((dbCommon *)pbsa, pbsa->noav, &pbsa->inp);
  if (status) return status;

  if ((pbsa->edef <= 0) || (pbsa->edef > EDEF_MAX)) {
    errlogPrintf("init_bsa_record (%s): Invalid EDEF %d\n",
                 pbsa->name, pbsa->edef);
    return S_db_badField;
  }
  pbsa->dpvt = &((bsaDevice_ts *)pbsa->dpvt)->bsa_as[pbsa->edef-1];
  return 0;
}

static long get_ioint_info(int cmd, bsaRecord *pbsa, IOSCANPVT *ppvt)
{
  bsa_ts *bsa_ps = (bsa_ts *)(pbsa->dpvt);

  if (bsa_ps) {
      if (bsa_ps->ioscanpvt == 0) scanIoInit(&bsa_ps->ioscanpvt);
      *ppvt = bsa_ps->ioscanpvt;
    } else {
      *ppvt = 0;
    }
    return 0;
}

static long init_ao_record(aoRecord *pao)
{
  return (init_record((dbCommon *)pao, 0, &pao->out));
}

static long write_ao(aoRecord *pao)
{
  long status = 0;

  DBADDR *paddr = dbGetPdbAddrFromLink(&pao->dol);
  long options = DBR_STATUS | DBR_TIME;
  long nrequest = 0;

  struct {
	DBRstatus
	DBRtime
  } options_s;

  /* Get the input's STAT and SEVR and timestamp (but don't get value) */

  if (!paddr) status = -1;
  else if (dbGetField(paddr, DBR_DOUBLE, &options_s, &options, &nrequest, 0)) {
    status = -1;
    if (pao->tse == epicsTimeEventDeviceTime) /* We need to timestamp it somehow! */
        epicsTimeGetCurrent(&pao->time);
  } else {
    status = bsaSecnAvg(&options_s.time, pao->val, options_s.status, options_s.severity, 0, pao->dpvt);
    if (pao->tse == epicsTimeEventDeviceTime)
        pao->time = options_s.time;
  }

  if (status) recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
  return status;

}

/* Create the device support entry tables */
typedef struct
{
  long        number;
  DEVSUPFUN   report;
  DEVSUPFUN   init;
  DEVSUPFUN   init_record;
  DEVSUPFUN   get_ioint_info;
  DEVSUPFUN   read_write;
  DEVSUPFUN   special_linconv;
} DSET;

DSET devAoBsa =
{
  6,
  NULL,
  NULL,
  init_ao_record,
  NULL,
  write_ao,
  NULL
};

DSET devBsa =
{
  6,
  NULL,
  NULL,
  init_bsa_record,
  get_ioint_info,
  read_bsa,
  NULL
};

epicsExportAddress(dset,devAoBsa);
epicsExportAddress(dset,devBsa);

#ifdef BSA_DEBUG
/* iocsh command: bsadebug */
const iocshArg bsadebugArg0 = {"Level" , iocshArgInt};
const iocshArg *const bsadebugArgs[1] = {&bsadebugArg0};
const iocshFuncDef bsadebugDef = {"bsadebug", 1, bsadebugArgs};

void bsadebugCall(const iocshArgBuf * args)
{
    printf("Changing bsa_debug_mask from 0x%05x to 0x%05x\n", bsa_debug_mask, args[0].ival);
    bsa_debug_mask = args[0].ival;
}
#endif

void bsaRegister() {
#ifdef BSA_DEBUG
    iocshRegister(&bsadebugDef  , bsadebugCall );
#endif
}
epicsExportRegistrar(bsaRegister);
