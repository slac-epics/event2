/***************************************************************************************************
|* devMrfEr.c -- EPICS Device Support Module for the Micro-Research Finland (MRF)
|*               Event Receiver Card
|*
|*--------------------------------------------------------------------------------------------------
|* Authors:  John Winans (APS)
|*           Timo Korhonen (PSI)
|*           Babak Kalantari (PSI)
|*           Eric Bjorklund (LANSCE)
|*           Dayle Kotturi (SLAC)
|* Date:     21 January 2006
|*
|*--------------------------------------------------------------------------------------------------
|* MODIFICATION HISTORY:
|* 21 Jul 1993  J.Winans        Original APS Event system device support module.
|* 21 Jan 2006  E.Bjorklund     Split driver and device support into separate software modules.
|*                              Make code OS-independent for EPICS Release 3.14.
|*
|*--------------------------------------------------------------------------------------------------
|* MODULE DESCRIPTION:
|*
|* This module contains the EPICS device-support code for the Micro-Research Finland series 200
|* event receiver card.  It is based on John Winan's original device support module for the
|* APS Global Event System, which was later modified by Timo Korhonen and Babak Kalantari to
|* support the MRF Series 100 and Series 200 event receiver boards.
|*
|* To make the software more flexible, we have separated out the driver and device support
|* functions so that different device support modules can support different event-system software
|* architectures.
|*
\**************************************************************************************************/

/**************************************************************************************************
|*                                     COPYRIGHT NOTIFICATION
|**************************************************************************************************
|*  
|* THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
|* AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
|* AND IN ALL SOURCE LISTINGS OF THE CODE.
|*
|**************************************************************************************************
|*
|* Copyright (c) 2006 The University of Chicago,
|* as Operator of Argonne National Laboratory.
|*
|* Copyright (c) 2006 The Regents of the University of California,
|* as Operator of Los Alamos National Laboratory.
|*
|* Copyright (c) 2006 The Board of Trustees of the Leland Stanford Junior
|* University, as Operator of the Stanford Linear Accelerator Center.
|*
|**************************************************************************************************
|*
|* This software is distributed under the EPICS Open License Agreement which
|* can be found in the file, LICENSE, included in this directory.
|*
\*************************************************************************************************/

/**************************************************************************************************/
/*  If EVR_DEBUG Flag is Defined, Make All Local Routines Globally Callable                       */
/**************************************************************************************************/

#ifdef EVR_DEBUG
#define LOCAL_RTN
#endif

/**************************************************************************************************/
/*  Imported Header Files                                                                         */
/**************************************************************************************************/

#include <epicsStdlib.h>        /* EPICS Standard C library support routines                      */
#include <epicsStdio.h>         /* EPICS Standard C I/O support routines                          */
#include <epicsTypes.h>         /* EPICS Architecture-independent type definitions                */
#include <epicsInterrupt.h>     /* EPICS Interrupt context support routines                       */
#include <epicsMutex.h>         /* EPICS Mutex support library                                    */
#include <string.h>

#include <alarm.h>              /* EPICS Alarm status and severity definitions                    */
#include <dbAccess.h>           /* EPICS Database access definitions                              */
#include <dbScan.h>             /* EPICS Database scan routines and definitions                   */
#include <devLib.h>             /* EPICS Device hardware addressing support library               */
#include <devSup.h>             /* EPICS Device support layer structures and symbols              */
#include <ellLib.h>             /* EPICS Linked list support library                              */
#include <errlog.h>             /* EPICS Error logging support library                            */
#include <recGbl.h>             /* EPICS Record Support global routine definitions                */
#include <registryFunction.h>   /* EPICS Registry support library                                 */

#include <erRecord.h>           /* Event Receiver (ER) Record structure                           */
#include <ereventRecord.h>      /* Event Receiver Event (EREVENT) record structure                */
#include <eventRecord.h>        /* Standard EPICS Event Record structure                          */
#include <biRecord.h>           /* Standard EPICS Event Record structure                          */
#include <stringinRecord.h>		/* Standard EPICS stringin Record structure                      */
#include <stringoutRecord.h>	/* Standard EPICS stringout Record structure                      */
#include <caeventmask.h>
#include <dbEvent.h>

#include <erDefs.h>             /* Common Event Receiver (ER) definitions                         */
#include <devMrfEr.h>           /* MRF Event Receiver device support layer interface              */
#include <drvMrfEr.h>           /* MRF Event Receiver driver support layer interface              */

#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */


/**************************************************************************************************/
/*  Prototype Function Declarations                                                               */
/**************************************************************************************************/

LOCAL_RTN void ErDevEventFunc (ErCardStruct*, epicsInt16, epicsUInt32);
LOCAL_RTN void ErDevErrorFunc (ErCardStruct*, int);


/**************************************************************************************************/
/*                     Event Receiver (ER) Record Device Support Routines                         */
/*                                                                                                */

/**************************************************************************************************/
/*  Prototype Definitions for ER Record Device Support Functions                                  */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErInitRecord (erRecord*);
LOCAL_RTN epicsStatus ErProcess    (erRecord*);


/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfEr = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* Driver-Layer routine to complete the hardware init.    */
    (DEVSUPFUN)ErInitRecord,            /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O Interrupt information routine                */
    (DEVSUPFUN)ErProcess                /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfEr);

/**************************************************************************************************
|* ErInitRecord () -- ER Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each ER
|* record in the database.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's card number by making sure it was configured in the startup script
|*     and that no two ER records have the same card number.
|*   o Initialize the Event Receiver card structure with device-support layer information such as
|*     the address of the record structure, the I/O interrupt structures for event FIFO interrupts,
|*     and the address of the device-support layer error and event handling routines.
|*   o Call the record processing routine to initialize the hardware with any pre-defined values
|*     from the ER record.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (erRecord *) Pointer to the ER record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successfull.
|*      S_dev_badCard  = Specified card number was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErInitRecord (erRecord *pRec)
{
    epicsInt16     Card;        /* Event Receiver card number for this record                     */
    int            i;           /* Loop counter                                                   */
    ErCardStruct  *pCard;       /* Pointer to Event Receiver card structure for this record       */

   /*---------------------
    * Output a debug message if the debug flag is set
    */
    if (ErDebug)
        printf ("devMrfEr::ErInitRecord(%s) entered\n", pRec->name);

   /*---------------------
    * Make sure the card number is valid by fetching its card structure
    */
    Card = pRec->out.value.vmeio.card;
    epicsSnprintf(pRec->busd,sizeof(pRec->busd)-1,"Unavailable");
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErInitErRec() Invalid card number");
        return(S_dev_badCard);
    }/*end if invalid card number*/

   /*---------------------
    * Make sure we only have one record for a given ER card
    */
    if (pCard->pRec != NULL) {
        recGblRecordError (S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErInitErRec() only one record allowed per card");
        return (S_dev_badCard);
    }/*end if card already initialized*/

	memset(pRec->busd, 0, sizeof(pRec->busd));
	if ( VME_EVR == pCard->FormFactor ) {
		epicsSnprintf(pRec->busd,sizeof(pRec->busd)-1,"VME Card %i, Slot %i, IRQ %i (Vect 0x%02x)",
			pCard->Cardno, pCard->Slot, pCard->IrqLevel, pCard->IrqVector);
	} else if ( EMBEDDED_EVR == pCard->FormFactor ) {
		epicsSnprintf(pRec->busd,sizeof(pRec->busd)-1,"Embedded Card %i, IRQ %i (Vect 0x%02x)",
			pCard->Cardno, pCard->IrqLevel, pCard->IrqVector);
	} else {
		epicsSnprintf(pRec->busd,sizeof(pRec->busd)-1,"PMC Card %i @%u/%u/%u, IRQ 0x%x",
			pCard->Cardno,
			pCard->Slot >>8,
			(pCard->Slot >>3)&0x1f,
			pCard->Slot & 7,
			pCard->IrqVector);
	}

   /*---------------------
    * Finish initializing the event receiver card structure
    */
    pCard->pRec = (void *)pRec;                        /* Set the record address                  */
    ErRegisterDevEventHandler (pCard, ErDevEventFunc); /* Set the device support event handler    */
    ErRegisterDevErrorHandler (pCard, ErDevErrorFunc); /* Set the device support error handler    */

    for (i=0;  i <= EVENT_DELAYED_IRQ;  i++)           /* Initialize the IOSCANPVT structures     */
        scanIoInit (&pCard->IoScanPvt[i]);

   /*---------------------
    * Store the address of the event receiver card structure in the record's DPVT field.
    */
    pRec->dpvt = (void *)pCard;

   /*---------------------
    * Set the Event Receiver with any pre-defined values
    */
    ErProcess (pRec);
    return (0);

}/*end ErInitRecord()*/

 /**************************************************************************************************
|* ErProcess () -- ER Record Processing Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the ER record's record processing routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Make sure we have a valid Event Receiver Card Structure in the DPVT field.  If not,
|*     set the record's PACT field so that we won't be processed again.
|*   o Lock the card structure to keep other record processing routines from interferring with us.
|*   o Set the Event Receiver card outputs from the fields in this record:  These include:
|*     - Master card enable/disable.
|*     - Trigger output (TRG) enable/disable.
|*     - Programmable width output (OTP) enable, delay, width, and polarity.
|*     - Distributed data bus output enable/disable.
|*     - Level output (OTL) enable/disable.
|*     - Programmable delay output (DG) enable, delay, width, prescaler, and polarity.
|*     - Delayed interrupt (DVME) enable, delay, and prescaler.
|*     - Front panel output configuration registers.
|*     - Event Receiver firmware version (FPGV)
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (erRecord *) Pointer to the ER record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns 0 (success)
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErProcess (erRecord  *pRec)
{
    ErCardStruct   *pCard;
    unsigned int ipov = 0;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("devMrfEr: ErProcess (%s) entered\n", pRec->name);

   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (ErCardStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Lock the card structure while we are processing the record.
    */
    epicsMutexLock (pCard->CardLock);
  
   /*---------------------
    * Update the delayed pulse generator outputs first
    * Set the programmable delay (DG) output parameters (Enable, Delay, Width, Prescaler, Polarity)
    */
    ipov = 0;
    if (pRec->ip0e) {
        if (pRec->dg0e || pRec->ld0e)
            ErSetDg (pCard, 0, pRec->dg0e, pRec->dg0d, pRec->dg0w, pRec->dg0c, pRec->dg0p);
        pRec->ld0e = pRec->dg0e;
        ipov |= 1;
    }
    if (pRec->ip1e) {
        if (pRec->dg1e || pRec->ld1e)
            ErSetDg (pCard, 1, pRec->dg1e, pRec->dg1d, pRec->dg1w, pRec->dg1c, pRec->dg1p);
        pRec->ld1e = pRec->dg1e;
        ipov |= 1 << 1;
    }
    if (pRec->ip2e) {
        if (pRec->dg2e || pRec->ld2e)
            ErSetDg (pCard, 2, pRec->dg2e, pRec->dg2d, pRec->dg2w, pRec->dg2c, pRec->dg2p);
        pRec->ld2e = pRec->dg2e;
        ipov |= 1 << 2;
    }
    if (pRec->ip3e) {
        if (pRec->dg3e || pRec->ld3e)
            ErSetDg (pCard, 3, pRec->dg3e, pRec->dg3d, pRec->dg3w, pRec->dg3c, pRec->dg3p);
        pRec->ld3e = pRec->dg3e;
        ipov |= 1 << 3;
    }
    if (pRec->ip4e) {
        if (pRec->dg4e || pRec->ld4e)
            ErSetDg (pCard, 4, pRec->dg4e, pRec->dg4d, pRec->dg4w, pRec->dg4c, pRec->dg4p);
        pRec->ld4e = pRec->dg4e;
        ipov |= 1 << 4;
    }
    if (pRec->ip5e) {
        if (pRec->dg5e || pRec->ld5e)
            ErSetDg (pCard, 5, pRec->dg5e, pRec->dg5d, pRec->dg5w, pRec->dg5c, pRec->dg5p);
        pRec->ld5e = pRec->dg5e;
        ipov |= 1 << 5;
    }
    if (pRec->ip6e) {
        if (pRec->dg6e || pRec->ld6e)
            ErSetDg (pCard, 6, pRec->dg6e, pRec->dg6d, pRec->dg6w, pRec->dg6c, pRec->dg6p);
        pRec->ld6e = pRec->dg6e;
        ipov |= 1 << 6;
    }
    if (pRec->ip7e) {
        if (pRec->dg7e || pRec->ld7e)
            ErSetDg (pCard, 7, pRec->dg7e, pRec->dg7d, pRec->dg7w, pRec->dg7c, pRec->dg7p);
        pRec->ld7e = pRec->dg7e;
        ipov |= 1 << 7;
    }
    if (pRec->ip8e) {
        if (pRec->dg8e || pRec->ld8e)
            ErSetDg (pCard, 8, pRec->dg8e, pRec->dg8d, pRec->dg8w, pRec->dg8c, pRec->dg8p);
        pRec->ld8e = pRec->dg8e;
        ipov |= 1 << 8;
    }
    if (pRec->ip9e) {
        if (pRec->dg9e || pRec->ld9e)
            ErSetDg (pCard, 9, pRec->dg9e, pRec->dg9d, pRec->dg9w, pRec->dg9c, pRec->dg9p);
        pRec->ld9e = pRec->dg9e;
        ipov |= 1 << 9;
    }
    if (pRec->ipae) {
        if (pRec->dgae || pRec->ldae)
            ErSetDg (pCard, 10, pRec->dgae, pRec->dgad, pRec->dgaw, pRec->dgac, pRec->dgap);
        pRec->ldae = pRec->dgae;
        ipov |= 1 << 10;
    }
    if (pRec->ipbe) {
        if (pRec->dgbe || pRec->ldbe)
            ErSetDg (pCard, 11, pRec->dgbe, pRec->dgbd, pRec->dgbw, pRec->dgbc, pRec->dgbp);
        pRec->ldbe = pRec->dgbe;
        ipov |= 1 << 11;
    }

    if (pRec->ipov != ipov) {
        pRec->ipov = ipov;
        db_post_events(pRec, &pRec->ipov, DBE_VALUE);
    }

    pRec->fpgv = ErGetFpgaVersion (pCard);

#if 0
   /*---------------------
    * We might have changed something, so need to update!
    *
    * MCB - Actually, I don't think so.  This only touches pulse generators, not the
    * event table!
    */
    ErUpdateRam (pCard, pCard->ErEventTab);
#endif

   /*---------------------
    * Unlock the card mutex, mark the record "processed", and return success
    */
    epicsMutexUnlock (pCard->CardLock);
    pRec->udf = 0;
    return (0);

}/*end ErProcess()*/

/**************************************************************************************************/
/*                Event Receiver Event ("erevent") Record Device Support Routines                 */
/*                                                                                                */

/**************************************************************************************************/
/*  Prototype Definitions for Event Receiver Event Record Device Support Functions                */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErEventInitRecord (ereventRecord*);
LOCAL_RTN epicsStatus ErEventProcess    (ereventRecord*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErevent = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization route                      */
    (DEVSUPFUN)ErEventInitRecord,       /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O Interrupt information routine                */
    (DEVSUPFUN)ErEventProcess           /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfErevent);

/**************************************************************************************************
|* ErEventInitRecord () -- Event Receiver Event (erevent) Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each "erevent"
|* record in the database.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's card number by making sure it was configured in the startup script.
|*   o Initialize the ereventRecord structure.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (ereventRecord *) Pointer to the "erevent" record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successfull.
|*      S_dev_badCard  = Specified card number was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEventInitRecord (ereventRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    epicsInt16     Card;        /* Event Receiver card number for this record                     */
    ErCardStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("devMrfEr::ErEventInitRec(%s)\n", pRec->name);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    Card = pRec->out.value.vmeio.card;
    pRec->dpvt = NULL;
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEventInitRec() invalid card number in OUT field");
        return (S_dev_badCard);
    }/*end if invalid card number*/

   /*---------------------
    * Initialize the record structure
    */
    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    pRec->lenm = -1;      	/* Force setting on first process         */
    pRec->lout = 0;             /* Clear the 'last' event mask            */

    return (0);

}/*end ErEventInitRecord()*/

/**************************************************************************************************
|* ErEventProcess () -- "erevent" Record Processing Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the "erevent" record's record processing routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Make sure we have a valid Event Receiver Card Structure in the DPVT field.  If not,
|*     set the record's PACT field so that we won't be processed again.
|*   o Lock the card structure to keep other record processing routines from interferring with us.
|*   o Based on the field values, processing an "erevent" record could involve any of the following:
|*     - Change the specified event number (ENM).
|*     - Change the output mapping for the specified event number (OUTx, VME).
|*     - Enable/Disable event mapping for this record (ENAB).
|*   o If the "Interrupt on Event" flag is set (VME), enable Event-FIFO interrupts.
|*   o If the "erevent" record is disabled (ENAB=0), put the record into a MINOR alarm status.
|*   o If the "erevent" record contains and invalid event number, put the record into a MAJOR
|*     alarm status.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (ereventRecord *) Pointer to the "erevent" record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns 0 (success)
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o The "erevent" record does not currently have a field for setting the "timestamp latch"
|*   bit in the event's output map.
|* o Debug output prints can be enabled either globally (every record) or locally (only this
|*   record for "erevent" records.  To enable debug outputs for every record, set the "ErDebug"
|*   global variable to a value of 11 or higher.  This can be done with a call of the form:
|*               ErDebugLevel (11);
|*   (note that this will produce a lot of other debug output as well).
|*   To enable debug output for just this record, set the TPRO (Traceback on Process) field
|*   to 11 or higher.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEventProcess (ereventRecord  *pRec)
{
   /*---------------------
    * Local variables
    */
    epicsBoolean   DebugFlag;			/* True if debug output prints are enabled        */
    epicsBoolean   LoadMask = epicsFalse;       /* True if need to load a new output mask         */
    epicsBoolean   LoadRam  = epicsFalse;       /* True if need ro re-load the Event Map RAM      */
    epicsUInt16    Mask = 0;                    /* New output mask for this event                 */
    ErCardStruct  *pCard;                       /* Pointer to Event Receiver card structure       */

   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (ErCardStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * If debug output is enabled, display the record's enable status
    * Debug output can be enabled for all records by calling "ErDebugLevel(11)",
    * or on a per-record basis by setting the TPRO field to 11.
    */
    DebugFlag = (pRec->tpro > 10) || (ErDebug > 10);
    if (DebugFlag)
        printf ("ErEventProc(%s) entered.  ENAB = %s\n",
                      pRec->name, pRec->enab?"True":"False");

   /*---------------------
    * Lock the event receiver card structure while we process this record
    */
    epicsMutexLock (pCard->CardLock);

   /*---------------------
    * If the record is enabled, see if the event number or output mask have changed.
    */
    if (pRec->enab) {
       epicsUInt32 ipv;
       long status=dbGetLink(&pRec->ipv,DBR_ULONG,&ipv,0,0);
       if (!RTN_SUCCESS(status)) /* Ow.  This isn't good. */
           ipv = 0;

       /*---------------------
        * Build the output mask for this event.
        * NOTE: We currently do not have a field for the timestamp latch....
        */
        if (pRec->out0 != 0) Mask |= EVR_MAP_CHAN_0;
        if (pRec->out1 != 0) Mask |= EVR_MAP_CHAN_1;
        if (pRec->out2 != 0) Mask |= EVR_MAP_CHAN_2;
        if (pRec->out3 != 0) Mask |= EVR_MAP_CHAN_3;
        if (pRec->out4 != 0) Mask |= EVR_MAP_CHAN_4;
        if (pRec->out5 != 0) Mask |= EVR_MAP_CHAN_5;
        if (pRec->out6 != 0) Mask |= EVR_MAP_CHAN_6;
        if (pRec->out7 != 0) Mask |= EVR_MAP_CHAN_7;
        if (pRec->out8 != 0) Mask |= EVR_MAP_CHAN_8;
        if (pRec->out9 != 0) Mask |= EVR_MAP_CHAN_9;
        if (pRec->outa != 0) Mask |= EVR_MAP_CHAN_10;
        if (pRec->outb != 0) Mask |= EVR_MAP_CHAN_11;
        if (pRec->outc != 0) Mask |= EVR_MAP_CHAN_12;
        if (pRec->outd != 0) Mask |= EVR_MAP_CHAN_13;
        Mask &= ipv;
        if (pRec->vme  != 0) Mask |= EVR_MAP_INTERRUPT;

       /*---------------------
        * Check to see if the event number has changed
        */
        if (pRec->enm != pRec->lenm) {
            if (DebugFlag)
                printf ("ErEventProc(%s) event number changed %d-%d\n", 
                              pRec->name, pRec->lenm, pRec->enm);

            /* Check to see if the new event number is already used */
            if ((pRec->enm < EVR_NUM_EVENTS) && (pRec->enm > 0)
                && pCard->ErEventTab[pRec->enm] != 0 ) {
                /*----------------
                 * The new event number is already being used by a different erevent record
                 * Force this record to keep it's prior event number
                 */
                errlogPrintf( "ErEventProcess Error: Event %d already in use!\n", pRec->enm );
                pRec->enm = pRec->lenm;

                /* This call needed as recGblSetSevr() doesn't post events for pField NULL */
                /* db_post_events( pRec, &pRec->enm, DBE_VALUE | DBE_LOG ); */

                /*----------------
                 * Clear the output mask for our old event number
                 */
                if ((pRec->lenm < EVR_NUM_EVENTS) && (pRec->lenm > 0)) {
                    pCard->ErEventTab[pRec->lenm] = 0;
                    LoadRam = epicsTrue;
                }/*end if LENM was valid*/

                /*---------------------
                 * Disable the event and set LENM to an invalid code in order to:
                 * a) Inhibit further processing until ENAB goes back to "Enabled", and
                 * b) Force a RAM re-load when ENAB does go back to "Enabled".
                 */
                Mask		= 0;
                pRec->enab	= epicsFalse;
                pRec->lenm	= -1;
                recGblSetSevr(	pRec, STATE_ALARM, MAJOR_ALARM	);
            } else {
                /* Clear the entry for the previous event number */ 
                if ((pRec->lenm < EVR_NUM_EVENTS) && (pRec->lenm > 0)) {
                    pCard->ErEventTab[pRec->lenm] = 0;
                    LoadRam = epicsTrue;
                }/*end if previous event number was valid*/

#if 0
                if ((pRec->enm < EVR_NUM_EVENTS) && (pRec->enm > 0)) {
                    /*
                     * Update the record desc field with the description
                     * for this event code
                     */
                    strncpy( &pRec->desc[0], &pCard->EventCodeDesc[pRec->enm][0],
                             MAX_STRING_SIZE+1 );
                    db_post_events(pRec, &pRec->desc, DBE_VALUE);
                }
#endif
            }

            pRec->lenm = pRec->enm;
            LoadMask = epicsTrue;
        }/*end if event number has changed*/

       /*---------------------
        * Check to see if the output mask has changed
        */
        if (Mask != pRec->lout) {
            if (DebugFlag)
                printf ("ErEventProc(%s) New RAM mask is 0x%4.4X\n", pRec->name, Mask);

            pRec->lout = Mask;
            LoadMask = epicsTrue;
        }/*end if output mask has changed*/

       /*---------------------
        * If the ENM field is valid, load the output mask for this event.
        */
        if (LoadMask && (pRec->enm < EVR_NUM_EVENTS) && (pRec->enm > 0)) {
            pCard->ErEventTab[pRec->enm] = Mask;
            LoadRam = epicsTrue;
        }/*end if we should write new output mask for this event*/

    }/*end if record is enabled*/

   /*---------------------
    * If the record is disabled, clear its output mask
    * and set the LENM field to ignore further processing.
    * Put disabled records into a STATE_ALARM with MINOR severity.
    */
    else {

       /*---------------------
        * Clear the output mask if this is the first time.
        * (Note that if we are processing because the ENAB field changed from
        * "Enable" to "Disable", then LENM will equal ENM.)
        */
        if ((pRec->lenm < EVR_NUM_EVENTS) && (pRec->lenm > 0)) {
            pCard->ErEventTab[pRec->lenm] = 0;
            LoadRam = epicsTrue;
        }/*end if LENM was valid*/

       /*---------------------
        * Set LENM to an invalid code in order to:
        * a) Inhibit further processing until ENAB goes back to "Enabled", and
        * b) Force a RAM re-load when ENAB does go back to "Enabled".
        */
        pRec->lenm = -1;
        recGblSetSevr (pRec, STATE_ALARM, MINOR_ALARM);
    }/*end if record is disabled*/

   /*---------------------
    * Re-load the Event Mapping RAM if it has changed.
    * If the event interrupt bit is specified, make sure Event FIFO interrupts are enabled.
    */
    if (LoadRam) {
        if (DebugFlag)
            printf ("ErEventProc(%s) updating Event RAM\n", pRec->name);

        if (Mask & EVR_MAP_INTERRUPT)
            ErEventIrq (pCard, epicsTrue);

        ErUpdateRam (pCard, pCard->ErEventTab);
    }/*end if we should re-load the Event Mapping Ram*/

   /*---------------------
    * Unlock the Event Record card structure
    */
    epicsMutexUnlock (pCard->CardLock);
    if (DebugFlag)
        printf ("ErEventProc(%s) I/O operations complete\n", pRec->name);

   /*---------------------
    * Raise the record severity to MAJOR, if the event number is not valid.
    */
    if ((pRec->enm >= EVR_NUM_EVENTS) || (pRec->enm < 0))
        recGblSetSevr (pRec, HW_LIMIT_ALARM, MAJOR_ALARM);

    return (0);

}/*end ErEventProcess()*/

/**************************************************************************************************/
/*                         EPICS Event Record Device Support Routines                             */
/*                                                                                                */

/**************************************************************************************************/
/*  Prototype Definitions for EPICS Event Record Device Support Functions                         */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErEpicsEventInitRec   (eventRecord*);
LOCAL_RTN epicsStatus ErEpicsEventGetIoScan (int, eventRecord*, IOSCANPVT*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsEvent = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization route                      */
    (DEVSUPFUN)ErEpicsEventInitRec,     /* Record initialization routine                          */
    (DEVSUPFUN)ErEpicsEventGetIoScan,   /* Get I/O interrupt information routine                  */
    (DEVSUPFUN)NULL                     /* -- No Record processing routine                        */
};

epicsExportAddress (dset, devMrfErEpicsEvent);

/**************************************************************************************************
|* ErEpicsEventInitRec () -- EPICS Event Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each EPICS
|* event record in the database.
|*
|* Note that this is a regular EPICS event record and not an ER or EG event record.
|* We allow the user to use the regular EPICS event records to request processing on an event
|* that is received from the Event Receiver.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's card number by making sure it was configured in the startup script
|*   o Validate that the record's signal number specifies a legal event.
|*   o Set the record's DPVT field to point to the proper IOSCANPVT element in the Event Receiver
|*     card structure so that we can later return it on "getIoScanInfo()" calls.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (eventRecord *) Pointer to the EPICS EVENT record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successfull.
|*      S_dev_badCard  = Specified card or signal (event) value was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsEventInitRec (eventRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    int            Event;       /* Event number to trigger this record                            */
    ErCardStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) and the event number (signal)
    * from the record's output link.
    */
    Card = pRec->inp.value.vmeio.card;
    Event = pRec->inp.value.vmeio.signal;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("ErEpicsEventInitRec(%s) Card %d, Event %d\n",
                      pRec->name, Card, Event);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsEventInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

   /*---------------------
    * Make sure the event number is in the correct range
    */
    if ((Event <= 0) || (Event > EVR_NUM_EVENTS)) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsEventInitRec() invalid signal number in INP field");
        return(S_dev_badCard);
    }/*end if event number is invalid*/

    /*
     * Keep a copy of the event code description for
     * later use when the ereventRecord handles changing event codes.
     */
    strncpy( pCard->EventCodeDesc[Event], pRec->desc, MAX_STRING_SIZE+1 );

   /*---------------------
    * Store the address of the IOSCANPVT structure that corresponds
    * to the requested event.
    */
    pRec->dpvt = (void *) &pCard->IoScanPvt[Event];
    return (0);

}/*end ErEpicsEventInitRec()*/

/**************************************************************************************************
|* ErEpicsEventGetIoScan () -- Return the IOSCSANPVT Structure for an EPICS Event Record
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called by the EPICS I/O interrupt scan task whenever a record is being
|* placed on or removed from an I/O scan list.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      cmd    = (int)           0 if the record is being place on a scan list
|*                               1 if the record is being removed from a scan list.
|*      pRec   = (eventRecord *) Pointer to the EPICS EVENT record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* OUTPUT PARAMTERS:
|*      pPvt   = (IOSCANPVT *)   Pointer to IOSCANPVT structure
|*                               (NULL if there is no IOSCANPVT structure for this record)
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successfull.
|*      S_dev_badCard  = Specified card or signal (event) value was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsEventGetIoScan (int cmd, eventRecord *pRec, IOSCANPVT *pPvt)
{
  if (!pRec->dpvt) return (S_dev_badCard);
  *pPvt = *((IOSCANPVT*)(pRec->dpvt));
  return (0);

}/*end ErEpicsEventGetIoScan()*/

/**************************************************************************************************/
/*                         EPICS Binary Input Record Device Support Routines                      */
/*                         to check for taxi violation                                            */

/**************************************************************************************************/
/*  Prototype Definitions for EPICS Binary Input Record Device Support Functions                  */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErEpicsBiInitRec  (biRecord*);
LOCAL_RTN epicsStatus ErEpicsBiProcess  (biRecord*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsBi = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization routine                    */
    (DEVSUPFUN)ErEpicsBiInitRec,        /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O interrupt information routine                */
    (DEVSUPFUN)ErEpicsBiProcess         /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfErEpicsBi);

/**************************************************************************************************
|* ErEpicsBiInitRec () -- EPICS Binary Input Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each EPICS
|* bi record in the database.
|*
|* Note that this is a regular EPICS bi record and not an ER or EG event record.
|* The regular EPICS bi records is used to check for taxi violation without
|* processing the Event Receiver record which has much higher overhead.  This record is
|* especially useful when the taxi violation interrupt is disabled.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's card number by making sure it was configured in the startup script
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (biRecord *) Pointer to the EPICS bi record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successfull.
|*      S_dev_badCard  = Specified card was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsBiInitRec (biRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    ErCardStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) from the record's output link.
    */
    Card = pRec->inp.value.vmeio.card;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("ErEpicsBiInitRec(%s) Card %d\n",
                      pRec->name, Card);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsBiInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    return (0);

}/*end ErEpicsBiInitRec()*/

/**************************************************************************************************
|* ErEpicsBiProcess () -- Binary Input Record Processing Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the "bi" record's record processing routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Make sure we have a valid Event Receiver Card Structure in the DPVT field.  If not,
|*     set the record's PACT field so that we won't be processed again.
|*   o Lock the card structure to keep other record processing routines from interferring with us.
|*   o Call ErCheckTaxi to get binary value.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (biRecord *) Pointer to the "bi" record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns 2 (don't convert)
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsBiProcess (biRecord  *pRec)
{
   /*---------------------
    * Local variables
    */
    ErCardStruct  *pCard;                       /* Pointer to Event Receiver card structure       */
  
   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (ErCardStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Lock the event receiver card structure while we process this record
    */
    epicsMutexLock (pCard->CardLock);

    pRec->val = ErCheckTaxi (pCard)?0:1;
    pRec->udf = 0;

   /*---------------------
    * Unlock the Event Record card structure
    */
    epicsMutexUnlock (pCard->CardLock);

    return (2);

}/*end ErEpicsBiProcess()*/

/**************************************************************************************************/
/*                         EPICS stringout Record Device Support Routines                         */
/**************************************************************************************************/
/*  Prototype Definitions for EPICS stringout Record Device Support Functions                     */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErEpicsStringoutInitRec  (stringoutRecord*);
LOCAL_RTN epicsStatus ErEpicsStringoutWrite    (stringoutRecord*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsStringout =
{
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization routine                    */
    (DEVSUPFUN)ErEpicsStringoutInitRec,        /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O interrupt information routine                */
    (DEVSUPFUN)ErEpicsStringoutWrite    /* write_stringout routine                                */
};

epicsExportAddress (dset, devMrfErEpicsStringout);

/**************************************************************************************************
|* ErEpicsStringoutInitRec () -- EPICS stringout Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each EPICS
|* stringout record in the database.
|*
|* Note that this is a regular EPICS stringout record and not an ER or EG event record.
|* Currently the main use for these routines are to capture updates to the DBF_STRING
|* records in the EVG which we use for the master copy of the event code names.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's card number by making sure it was configured in the startup script
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (stringoutRecord *) Pointer to the EPICS stringout record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successful.
|*      S_dev_badCard  = Specified card was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsStringoutInitRec (stringoutRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    ErCardStruct  *pCard;   /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) from the record's output link.
    */
    Card = pRec->out.value.vmeio.card;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("ErEpicsStringoutInitRec(%s) Card %d\n",
                      pRec->name, Card);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsStringoutInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    return (0);

}/*end ErEpicsStringoutInitRec()*/

/**************************************************************************************************
|* ErEpicsStringoutWrite () -- stringout write_string Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the "stringout" record's record processing routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Make sure we have a valid Event Receiver Card Structure in the DPVT field.  If not,
|*     set the record's PACT field so that we won't be processed again.
|*   o Lock the card structure to keep other record processing routines from interferring with us.
|*   o Capture updates to the event code names for later use
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (stringoutRecord *) Pointer to the "stringout" record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns 2 (don't convert)
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsStringoutWrite (stringoutRecord  *pRec)
{
    /*---------------------
     * Local variables
     */
    ErCardStruct  *pCard;                       /* Pointer to Event Receiver card structure       */
	int				Event;

    /*---------------------
     * Get the event number (signal)
     */
    Event = pRec->evnt;
 
    /*---------------------
     * Get the card structure.
     * Abort if we don't have a valid card structure.
     */
    if (NULL == (pCard = (ErCardStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Make sure the event number is in the correct range
    */
    if ((Event <= 0) || (Event > EVR_NUM_EVENTS)) {
		/* User has entered an invalid event number
		 * for this record.  Possibly a duplicate, in
		 * which case the field is set to -1.
		 * We don't set pact here as the user can fix this. */
        return(-1);
    }/*end if event number is invalid*/

    /*---------------------
     * Lock the event receiver card structure while we process this record
     */
    epicsMutexLock (pCard->CardLock);

	/*
	 * Keep a copy of the event code description for
	 * later use when the ereventRecord handles changing event codes
	 */
	strncpy( &pCard->EventCodeDesc[Event][0], &pRec->val[0], MAX_STRING_SIZE+1 );

    /*---------------------
     * Unlock the Event Record card structure
     */
    epicsMutexUnlock (pCard->CardLock);

    return (2);

}/*end ErEpicsStringoutWrite()*/

/**************************************************************************************************/
/*                         EPICS stringin Record Device Support Routines                         */
/**************************************************************************************************/
/*  Prototype Definitions for EPICS stringin Record Device Support Functions                     */
/**************************************************************************************************/

LOCAL_RTN epicsStatus ErEpicsStringinInitRec(	stringinRecord	*	);
LOCAL_RTN epicsStatus ErEpicsStringinRead(		stringinRecord	*	);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsStringin =
{
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization routine                    */
    (DEVSUPFUN)ErEpicsStringinInitRec,  /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O interrupt information routine                */
    (DEVSUPFUN)ErEpicsStringinRead		/* read_stringin routine                                  */
};

epicsExportAddress (dset, devMrfErEpicsStringin);

/**************************************************************************************************
|* ErEpicsStringinInitRec () -- EPICS stringin Record Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It is called once for each EPICS
|* stringin record in the database.
|*
|* Note that this is a regular EPICS stringin record and not an ER or EG event record.
|* This record is useful for fetching the name of an event code
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Validate the record's input link
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (stringinRecord *) Pointer to the EPICS stringin record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      0              = Record initialization was successful.
|*      S_dev_badCard  = Specified card was invalid.
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsStringinInitRec (stringinRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    ErCardStruct  *pCard;   /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) from the record's input link.
    */
    Card = pRec->inp.value.vmeio.card;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (ErDebug)
        printf ("ErEpicsStringinInitRec(%s) Card %d\n",
                      pRec->name, Card);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = ErGetCardStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsStringinInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    return (0);

}/*end ErEpicsStringinInitRec()*/

/**************************************************************************************************
|* ErEpicsStringinRead () -- stringin read_string Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the "stringin" record's record processing routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*   o Make sure we have a valid Event Receiver Card Structure in the DPVT field.  If not,
|*     set the record's PACT field so that we won't be processed again.
|*   o Lock the card structure to keep other record processing routines from interferring with us.
|*   o Capture updates to the event code names for later use
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pRec   = (stringinRecord *) Pointer to the "stringin" record structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns 2 (don't convert)
|*
\**************************************************************************************************/

LOCAL_RTN
epicsStatus ErEpicsStringinRead (stringinRecord  *pRec)
{
    /*---------------------
     * Local variables
     */
    ErCardStruct  *pCard;                       /* Pointer to Event Receiver card structure       */
	int				Event;

    /*---------------------
     * Get the event number (signal)
     */
    Event = pRec->evnt;
 
    /*---------------------
     * Get the card structure.
     * Abort if we don't have a valid card structure.
     */
    if (NULL == (pCard = (ErCardStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Make sure the event number is in the correct range
    */
    if ((Event <= 0) || (Event > EVR_NUM_EVENTS)) {
		/* User has entered an invalid event number
		 * for this record.  Possibly a duplicate, in
		 * which case the field is set to -1.
		 * We don't set pact here as the user can fix this. */
        return(-1);
    }/*end if event number is invalid*/

    /*---------------------
     * Lock the event receiver card structure while we process this record
     */
    epicsMutexLock (pCard->CardLock);

	/*
	 * Fetch the event code description
	 */
	strncpy( &pRec->val[0], &pCard->EventCodeDesc[Event][0], MAX_STRING_SIZE+1 );

    /*---------------------
     * Unlock the Event Record card structure
     */
    epicsMutexUnlock (pCard->CardLock);

	if ( pRec->tpro )
		printf( "ErEpicsStringinRead: %s updated to %s for EC %d\n",
				pRec->name, pRec->val, pRec->evnt );

    return (2);

}/*end ErEpicsStringinRead()*/

/**************************************************************************************************
|* ErDevEventFunc () -- Device Support Layer Interrupt-Level Event Handling Routine
|*-------------------------------------------------------------------------------------------------
|* 
|* This routine is called by the driver support layer to process events extracted from the
|* Event Receiver card's event FIFO.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard    = (ErCardStruct *)  Pointer to the Event Receiver card structure
|*      EventNum = (epicsInt16)      Event number
|*      Time     = (epicsUInt32)     Timestamp of when the event occurred.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|* o If a user-defined event handling routine has been declared for this card, it will be
|*   called with the event number and timestamp.
|*
|* o Any EPICS event records registered to process on this event will be processed via the
|*   IoScanPvt mechanism.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine runs entirely in interrupt context.
|*
\**************************************************************************************************/

LOCAL_RTN
void ErDevEventFunc (ErCardStruct *pCard, epicsInt16 EventNum, epicsUInt32 Time)
{
   /*---------------------
    * Invoke the user-defined event handler (if one is defined)
    */
    if (pCard->EventFunc != NULL)
        (*(USER_EVENT_FUNC)pCard->EventFunc)(pCard->Cardno, EventNum, Time);

   /*---------------------
    * Schedule processing for any event-driven records
    */
    scanIoRequest (pCard->IoScanPvt[EventNum]);

}/*end ErDevEventFunc()*/

/**************************************************************************************************/
/*                                 Local Callback Routines                                        */
/*                                                                                                */


/**************************************************************************************************
|* ErDevErrorFunc () -- Device Support Layer Interrupt-Level Error Handling Routine
|*-------------------------------------------------------------------------------------------------
|* 
|* This routine is called by the driver support layer to handle errors discovered in the
|* interrupt service routine.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*
|* The following conditions are handled:
|*
|*    o Receiver Link Error:
|*      Also known as a "TAXI" error (for historical reasons).
|*      Update the TAXI field in the ER record.  This field counts the number of receive link
|*      errors seen so far.
|*      Logs an error message if the debug flag is set.
|*      Invokes the user-defined error function (if one has been attached).
|*
|*    o Lost Heartbeat Error:
|*      Logs an error message if the debug flag is set.
|*      Invokes the user-defined error function (if one has been attached).
|*
|*    o FIFO Overflow Error:
|*      Logs an error message if the debug flag is set.
|*      Invokes the user-defined error function (if one has been attached).
|*
|*    o Data Stream Checksum Error:
|*      Logs an error message if the debug flag is set.
|*      Invokes the user-defined error function (if one has been attached).
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard    = (ErCardStruct *)  Pointer to the Event Receiver card structure
|*      ErrorNum = (int)             Error code
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine runs entirely in interrupt context.
|*
\**************************************************************************************************/


LOCAL_RTN
void ErDevErrorFunc (ErCardStruct *pCard, int ErrorNum)
{
   /*---------------------
    * Local Variables
    */
    int        Card = pCard->Cardno;            /* Card number of the offending board             */
#if 0
    /* Do we really not need this? */
    erRecord  *pRec = (erRecord *)pCard->pRec;  /* Address of this board's ER record              */
#endif

   /*---------------------
    * Decide how to handle the error based on the specified error code
    */
    switch (ErrorNum) {

   /*---------------------
    * Receiver Link Frame (Taxi) Error
    */
    case ERROR_TAXI:
        if(ErDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Receiver Link (Taxi) Error.  Error repetition %d...\n",
                           Card, pCard->RxvioCount);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Lost Heartbeat Error
    */
    case ERROR_HEART:
        if(ErDebug > 2) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Lost Heartbeat\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * FIFO Overflow Error
    */
    case ERROR_LOST:
        if(ErDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Event FIFO Overflow\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Data Stream Checksum Error
    */
    case ERROR_DBUF_CHECKSUM:
        if(ErDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Data Stream Checksum Error\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Invalid Error Code
    */
    default:
        if(ErDebug) {
          epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                         "ER Card %d Invalid Error Code = %d.\n", Card, ErrorNum);
          epicsInterruptContextMessage (pCard->intMsg);
        }
        return;

    }/*end switch on error number*/

   /*---------------------
    * Invoke the user-defined error handler (if defined)
    */
    if (pCard->ErrorFunc != NULL)
        (*(USER_ERROR_FUNC)pCard->ErrorFunc) (Card, ErrorNum);

}/*end ErDevErrorFunc()*/

/**************************************************************************************************/
/*                            User-Callable Device Support Routines                               */
/*                                                                                                */


/**
 *
 * Register a listener for the event system.  Every time we get an event from
 * the event receiver, we will call the registered listener and pass in the
 * event number received and the tick counter value when that event was
 * received.
 *
 **/
GLOBAL_RTN
epicsStatus ErRegisterEventHandler (int Card, USER_EVENT_FUNC EventFunc)
{
    ErCardStruct  *pCard;

    if (ErDebug){
        printf ("ErRegisterEventHandler(%d, %p)\n", Card, (void *)EventFunc);
    }

    if (NULL == (pCard = ErGetCardStruct(Card))) {
        errlogPrintf ("ErRegisterEventHandler() called with invalid card number (%d)\n", Card);
        return (-1);
    }

    pCard->EventFunc = (EVENT_FUNC)EventFunc;
    return (0);
}
/**
 *
 * Register an error handler for the event system. 
 * Every time we get an error (rxvio, etc.) from
 * the event receiver, we will call the registered listener and pass in the
 * event number received and the tick counter value when that event was
 * received.
 *
 **/
GLOBAL_RTN
epicsStatus ErRegisterErrorHandler (int Card, USER_ERROR_FUNC ErrorFunc)
{
    ErCardStruct  *pCard;

    if (ErDebug){
        printf ("ErRegisterErrorHandler(%d, %p)\n", Card, (void *)ErrorFunc);
    }

    if (NULL == (pCard = ErGetCardStruct(Card))) {
        errlogPrintf ("ErRegisterErrorHandler() called with invalid card number (%d)\n", Card);
        return (-1);
    }

    pCard->ErrorFunc = (ERROR_FUNC)ErrorFunc;
    return(0);
}

LOCAL
registryFunctionRef devMrfErRef [] = {
    {"ErRegisterEventHandler", (REGISTRYFUNCTION)ErRegisterEventHandler},
    {"ErRegisterErrorHandler", (REGISTRYFUNCTION)ErRegisterErrorHandler}
};/*end devMrfErRef[]*/

LOCAL_RTN
void devMrfErRegistrar (void) {
    registryFunctionRefAdd (devMrfErRef, NELEMENTS(devMrfErRef));
}/*end devMrfErRegister()*/

epicsExportRegistrar (devMrfErRegistrar);
