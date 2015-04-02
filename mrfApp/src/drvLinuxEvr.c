/*
 *	This code is a linux implementation of the mrfApp EVR driver code
 *
 * Copyright 2010, Stanford University
 * Authors:
 *		Remi Machet <rmachet@slac.stanford.edu>
 *		Bruce Hill <bhill@slac.stanford.edu>
 *
 * Released under the GPLv2 licence <http://www.gnu.org/licenses/gpl-2.0.html>
 */
/*
 * NOTE: This code supports *only* PMC, cPCI, and SLAC EVRs!  (The issue is that the PMC has front
 * panel outputs and cPCI and SLAC have universal outputs, and there is code that sets one in favor
 * of the other.  Other flavors are not tested for, and the VME64x card would be problematic anyway
 * in that it has *both*.)
 */

#include <sys/ioctl.h>
#define DEFINE_READ_EVR
#define INLINE_READ_EVR static
#define  EVR_DRIVER_SUPPORT_MODULE   /* Indicates we are in the driver support module environment */
#include "drvMrfEr.h"
#undef	EVR_MAX_BUFFER
#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */
#include <registryFunction.h>   /* EPICS Registry support library                                 */
#include <epicsStdio.h>
#include <epicsExit.h>
#include <epicsStdlib.h>        /* EPICS Standard C library support routines                      */
#include <errlog.h>
#include <iocsh.h>              /* EPICS iocsh support library                                    */
#include <drvSup.h>
#include <string.h>
#include <errno.h>
#include <endian.h>
#include <unistd.h>
#include <byteswap.h>
#include "erapi.h"

#define DEVNODE_NAME_BASE	"/dev/er"
#define DEVNODE_MINOR            '4'

/**************************************************************************************************/
/*                              Private types                                                     */
/*                                                                                                */

#define TOTAL_EVR_PULSES	12
#define MAX_FP_CHANNELS         12
#define TOTAL_FP_CHANNELS	((pCard->FormFactor == SLAC_EVR) ? 12 : 8)
#define TOTAL_TB_CHANNELS	40
#define MAX_DG                  ((pCard->FormFactor == SLAC_EVR) ? EVR_NUM_DG : 4)

#define EVR_IRQ_OFF      0x0000         /* Turn off All Interrupts                                */
#define EVR_IRQ_ALL      0x001f         /* Enable All Interrupts                                  */
#define EVR_IRQ_TELL     0xffff         /* Return Current Interrupt Enable Mask                   */

/**************************************************************************************************/
/*                              Global variables                                                  */
/*                                                                                                */

static ELLLIST ErCardList;                        /* Linked list of ER card structures */
static epicsBoolean bErCardListInitDone = epicsFalse;
static epicsMutexId ErCardListLock;
static epicsMutexId ErConfigureLock;

/**************************************************************************************************/
/*                              Private APIs                                                      */
/*                                                                                                */

char *	FormFactorToString( int formFactor )
{
	char	*	pString;
	switch ( formFactor )
	{
	default:		pString	= "Invalid";	break;
	case PMC_EVR:	pString	= "PMC_EVR";	break;
	case CPCI_EVR:	pString	= "cPCI_EVR";	break;
	case VME_EVR:	pString	= "VME_EVR";	break;
	case SLAC_EVR:	pString	= "SLAC_EVR";	break;
	}
	return pString;
}


int ErGetFormFactor( int fd )
{
	int		formFactor;
	int		id = (READ_EVR_REGISTER(fd, FPGAVersion)>>24) & 0x0F;
        /*
         * This is actually a bit silly.  We're translating from one constant
         * to another.
         */
	switch ( id )
	{
	case EVR_FORM_PMC:	formFactor	= PMC_EVR;	break;
	case EVR_FORM_CPCI:	formFactor	= CPCI_EVR;	break;
	case EVR_FORM_VME:	formFactor	= VME_EVR;	break;
	case EVR_FORM_SLAC:	formFactor	= SLAC_EVR;	break;
	default:        	formFactor	= -1;		break;
	}
	return formFactor;
}

epicsUInt16 ErEnableIrq_nolock (ErCardStruct *pCard, epicsUInt16 Mask)
{
	epicsUInt16 ret;
	
	if(Mask == EVR_IRQ_TELL) {
		ret = (epicsUInt16)pCard->IrqVector;
	} else if (Mask == EVR_IRQ_OFF) {
                pCard->IrqVector = 0;
                ioctl(pCard->Slot, EV_IOCIRQMASK, &pCard->IrqVector);
		ret = OK;
	} else {
		pCard->IrqVector = Mask | EVR_IRQ_MASTER_ENABLE;
                ioctl(pCard->Slot, EV_IOCIRQMASK, &pCard->IrqVector);
		ret = OK;
	}
        /*
         * If events are off, reset so we'll jump into the middle of the queue
         * when we turn them back on.
         */
        if (!(pCard->IrqVector & EVR_IRQFLAG_EVENT))
            pCard->erp = -1;
	return ret;
}

/**************************************************************************************************
|* ErIrqHandler () -- Event Receiver Interrupt Service Routine
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|* The following interrupt conditions are handled:
|*
|*    o Receiver Link Error:
|*      Also known as a "TAXI" error (for historical reasons).
|*      Reset the RXVIO bit the the Control/Status register, increment the the count of receive
|*      errors in the Event Receiver Card Structure, and report the error to the device-support layer's
|*      interrupt error reporting routine.
|*
|*    o Lost Heartbeat Error:
|*      Reset the HRTBT bit in the Control/Status register and report the error to the
|*      device-support layer's interrupt error reporting routine.
|*
|*    o Event FIFO Not Empty:
|*      Reset the IRQFL bit in the Control/Status register and extract the queued events
|*      and their timestamps from the FIFO.  In order to prevent long spin-loops at interrupt
|*      level, there is a maximum number of events that will be extracted per interrupt.  This
|*      value is specified by the EVR_FIFO_EVENT_LIMIT symbol defined in the "drvMrfEr.h" header
|*      file. For each event extracted from the FIFO, the device-support layer's event handling
|*      routine is called with the extracted event number and timestamp.
|*
|*    o FIFO Overflow Error:
|*      Reset the FF bit in the Control/Status register and report the error to the
|*      device-support layer's interrupt error reporting routine.
|*
|*    o Delayed Interrupt Condition:
|*      Reset the DIRQ bit in the Control/Status register. Invoke the device-support layer's
|*      event handling routine with the special "Delayed Interrupt" event code.
|*
|*    o Data Buffer Ready Condition:
|*      Check for and report checksum errors to the device-support layer's interrupt error
|*      reporting routine.
|*      If device support has registered a data buffer listener, copy the data buffer into
|*      the card structure and invoke the device-support data listener.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o Note that the IRQ handler is called by all card the same way
|*   it has to figure out why and who generated an interrupt
|*
\**************************************************************************************************/
int irqCount = 0;
void ErIrqHandler(int fd, int flags)
{
	struct ErCardStruct *pCard;
        struct EvrQueues *pEq;
	int i;
	epicsMutexLock(ErCardListLock);
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link)) {
		epicsMutexLock(pCard->CardLock);
		if(pCard->Slot != fd || pCard->IrqLevel != 1) {
			epicsMutexUnlock(pCard->CardLock);
			continue;
		}

                /*
                 * Found our card!!
                 */

		pEq = pCard->pEq;

                irqCount++;

                /* This should always be here 2ms before the fiducial event */
		if(flags & EVR_IRQFLAG_DATABUF) {
                    long long drp = pEq->dwp - 1; /* Read the latest! */
                    if (drp != pCard->drp) {
                        int idx = drp & (MAX_EVR_DBQ - 1);
                        int databuf_sts = pEq->dbq[idx].status;

                        if(databuf_sts & (1<<C_EVR_DATABUF_CHECKSUM)) {
                            if (pCard->DevErrorFunc != NULL)
                                (*pCard->DevErrorFunc)(pCard, ERROR_DBUF_CHECKSUM);
                        } else {
                            if (pCard->DevDBuffFunc != NULL) {
                                pCard->DBuffSize = (databuf_sts & ((1<<(C_EVR_DATABUF_SIZEHIGH+1))-1));
                                memcpy(pCard->DataBuffer, pEq->dbq[idx].data, pCard->DBuffSize);
                                (*pCard->DevDBuffFunc)(pCard, pCard->DBuffSize, pCard->DataBuffer);
                            }
                        }
                        /* TBD - Check if we skipped some? */
                        pCard->drp = drp;
                    } else {
                        /* We must have skipped one earlier, but caught up? */
                    }
		}

		if(flags & EVR_IRQFLAG_EVENT) {
                    long long erplimit = pEq->ewp;     /* Pointer to the next! */
                    long long erp      = pCard->erp;   /* Where we are now. */
                    if (erp == -1)
                        erp = erplimit - 1;            /* If just starting, jump to where we are now. */
                    if (erplimit - erp > MAX_EVR_EVTQ / 2) {
                        /* Wow, we're far behind! Catch up a bit, but flag an error. */
                        erp = erplimit - MAX_EVR_EVTQ / 2;
                        flags |= EVR_IRQFLAG_FIFOFULL;
                    }
                    for(i=0; erp < erplimit; erp++) {
                        struct FIFOEvent *fe = &pEq->evtq[erp & (MAX_EVR_EVTQ - 1)];
                        if (pCard->ErEventTab[fe->EventCode] & (1 << 15)) {
                            if (pCard->DevEventFunc != NULL)
                               (*pCard->DevEventFunc)(pCard, fe->EventCode, fe->TimestampHigh);
                            i++;
                        }
                    }
                    pCard->erp = erp;
		}

		if(flags & EVR_IRQFLAG_PULSE) {
			if(pCard->DevEventFunc != NULL)
				(*pCard->DevEventFunc)(pCard, EVENT_DELAYED_IRQ, 0);
		}
		if(flags & EVR_IRQFLAG_HEARTBEAT) {
			if (pCard->DevErrorFunc != NULL)
				(*pCard->DevErrorFunc)(pCard, ERROR_HEART);
		}
		if(flags & EVR_IRQFLAG_FIFOFULL) {
			if (pCard->DevErrorFunc != NULL)
				(*pCard->DevErrorFunc)(pCard, ERROR_LOST);
		}
		if(flags & EVR_IRQFLAG_VIOLATION) {
			pCard->RxvioCount++;
			if (pCard->DevErrorFunc != NULL)
				(*pCard->DevErrorFunc)(pCard, ERROR_TAXI);
		}
		epicsMutexUnlock(pCard->CardLock);
                break;
	}
	epicsMutexUnlock(ErCardListLock);
	return;
}
	

/**************************************************************************************************
|* ErConfigure () -- Event Receiver Card Configuration Routine
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card        =  (int)         Logical card number for this Event Receiver card.
|*      CardAddress =  (epicsUInt32) Starting address for this card's register map.
|*      IrqVector   =  (epicsUInt32) Interrupt vector for this card.  
|*      IrqLevel    =  (epicsUInt32) VME interrupt request level for this card.
|*      FormFactor  =  (int)         Specifies VME or PMC version of the card.
|*
|*-------------------------------------------------------------------------------------------------
|* IMPLICIT INPUTS:
|*      CardName    =  (char *)      ASCII string identifying the Event Receiver Card.
|*      CfgRegName
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status      =  (int)         Returns OK if the routine completed successfully.
|*                                   Returns ERROR if the routine could not configure the
|*                                   requested Event Receiver card.
|*
\**************************************************************************************************/

static int ErConfigure (
    int Card,                               /* Logical card number for this Event Receiver card   */
    epicsUInt32 CardAddress,                /* IGNORED: Starting address for this card's register map      */
    epicsUInt32 IrqVector,                  /* if VME_EVR, Interrupt request vector, if PMC_EVR set to zero*/
    epicsUInt32 IrqLevel,                   /* if VME_EVR, Interrupt request level. if PMC_EVR set to zero*/
    int FormFactor)                         /* cPCI or PMC form factor                             */
{
	int ret, fdEvr;
	int		actualFormFactor;
	char strDevice[strlen(DEVNODE_NAME_BASE) + 3];
	struct ErCardStruct *pCard;
	void   *pEq;
        u32	FPGAVersion;

	epicsMutexLock(ErCardListLock);
	/* If not already done, initialize the driver structures */
	if (!bErCardListInitDone) {
		ellInit (&ErCardList);
		bErCardListInitDone = epicsTrue;
	}
	epicsMutexUnlock(ErCardListLock);
	
	/* check parameters */
	if (Card >= EVR_MAX_CARDS) {
		errlogPrintf("%s: driver does not support %d cards (max is %d).\n", __func__, Card, EVR_MAX_CARDS);
		return ERROR;
	}
	
	epicsMutexLock(ErConfigureLock);
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link)) {
		if (pCard->Cardno == Card) {
			errlogPrintf ("%s: Card number %d has already been configured\n", __func__, Card);
			epicsMutexUnlock(ErConfigureLock);
			return ERROR;
		}
	}

	/* Look for the EVR */
	ret = snprintf(strDevice, strlen(DEVNODE_NAME_BASE) + 3, DEVNODE_NAME_BASE "%c%c",
                       Card + 'a', DEVNODE_MINOR);
	if (ret < 0) {
		errlogPrintf("%s@%d(snprintf): %s.\n", __func__, __LINE__, strerror(-ret));
		epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}
	fdEvr = EvrOpen(&pEq, strDevice);
	if (fdEvr < 0) {
		errlogPrintf("%s@%d(EvrOpen) Error: %s opening %s\n", __func__, __LINE__, strerror(errno), strDevice );
		epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}

	/* Check the firmware version */
	FPGAVersion = READ_EVR_REGISTER(fdEvr, FPGAVersion);
	printf( "EVR Found with Firmware Revision 0x%04X\n", FPGAVersion );
	switch ( FPGAVersion )
	{
	default:
	    printf( "ErConfigure: WARNING: Unknown firmware revision on PMC EVR!\n" );
	    break;
	case EVR_FIRMWARE_REV_SLAC1:
	case EVR_FIRMWARE_REV_SLAC2:
	case EVR_FIRMWARE_REV_SLAC3:
	case EVR_FIRMWARE_REV_SLAC4:
	case EVR_FIRMWARE_REV_LINUX1:
	case EVR_FIRMWARE_REV_LINUX2:
	case EVR_FIRMWARE_REV_LINUX3:
	case EVR_FIRMWARE_REV_LINUX4:
	case EVR_FIRMWARE_REV_LINUX5:
	case EVR_FIRMWARE_REV_LINUX6:
	    break;
	case EVR_FIRMWARE_REV_VME1:
	    fprintf ( stderr,
		   "\nErConfigure ERROR: This PMC EVR has firmware for a RTEMS based system\n"
		   "and needs new firmware to be used under Linux!\n" );
	    EvrClose(fdEvr);
	    epicsMutexUnlock(ErConfigureLock);
	    return ERROR;
	}

	/* Check the hardware signature for an EVR */
	if(( FPGAVersion >>28) != 0x1) {
		errlogPrintf("%s: invalid hardware signature: 0x%08x.\n", __func__, FPGAVersion );
		EvrClose(fdEvr);
	    epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}

	ret = 0;
	actualFormFactor = ErGetFormFactor( fdEvr );
	if ( FormFactor == actualFormFactor )
	{
		printf( "Found a %s %s\n",
				FormFactorToString( FormFactor ), strDevice );
	}
	else
	{
		printf( "Configured for %s form factor, but %s has %s form factor.\n",
				FormFactorToString( FormFactor ), strDevice,
				FormFactorToString( actualFormFactor ) );
		errlogPrintf("%s: wrong form factor %d, signature is 0x%08x.\n", __func__,
                             FormFactor, FPGAVersion);
		EvrClose(fdEvr);
		epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}
	
	/* Fill in the minimum of the driver structure for driver data structures management*/
	pCard = (struct ErCardStruct *)malloc(sizeof(struct ErCardStruct));
	if (pCard == NULL) {
		errlogPrintf("%s@%d(malloc): failed.\n", __func__, __LINE__);
		EvrClose(fdEvr);
		epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}
	memset(pCard, 0, sizeof(struct ErCardStruct));
        pCard->drp = -1;
        pCard->erp = -1;
	pCard->Cardno = Card;
	pCard->CardLock = epicsMutexCreate();
	if (pCard->CardLock == 0) {
		errlogPrintf("%s@%d(epicsMutexCreate): failed.\n", __func__, __LINE__);
		free(pCard);
		EvrClose(fdEvr);
		epicsMutexUnlock(ErConfigureLock);
		return ERROR;
	}
	ellAdd (&ErCardList, &pCard->Link); 
	/* Now that the card is registered there is no chance that configure will go through
		again if called with the same card number: we can release the mutex */
	epicsMutexUnlock(ErConfigureLock);

	/* Finish filling the driver structure and configuring the hardware,
		if this fails we cannot release the linked list link, instead we
		we set Cardno to an invalid value */
	epicsMutexLock(pCard->CardLock);
	pCard->pEq = (void *)pEq;
	pCard->Slot = fdEvr;	/* we steal this irrelevant field */
        pCard->FPGAVersion = FPGAVersion;
        pCard->EnableMask = 0;
	ErEnableIrq_nolock(pCard, EVR_IRQ_OFF);
	EvrIrqAssignHandler(fdEvr, ErIrqHandler);
	pCard->IrqLevel = 1;	/* Tell the interrupt handler this interrupt is enabled */
	pCard->FormFactor = FormFactor;
	epicsMutexUnlock(pCard->CardLock);
	ErResetAll(pCard);
	return OK;
}

/**************************************************************************************************/
/*                              Public APIs                                                       */
/*                                                                                                */

/**************************************************************************************************
|* ErCheckTaxi () -- Check to See if We Have A Receive Link Framing Error (TAXI)
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status    = (epicsBoolean)   True if a framing error was present.
|*                                   False if a framing error was not present.
|*
\**************************************************************************************************/
epicsBoolean ErCheckTaxi(ErCardStruct *pCard)
{
	epicsBoolean ret = epicsFalse;
	
	epicsMutexLock(pCard->CardLock);
	if (EvrGetViolation(pCard->Slot))
		ret = epicsTrue;
	epicsMutexUnlock(pCard->CardLock);
	return ret;
}

/**************************************************************************************************
|* ErDebugLevel () -- Set the Event Receiver Debug Flag to the Desired Level.
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Level  = (epicsInt32) New debug level.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* Debug Level Definitions:
|*    0: No debug output produced.
|*    1: Messages on entry to driver and device initialization routines
|*       Messages when the event mapping RAM is changed.
|*   10: All previous messages, plus:
|*       - Changes to OTP gate parameters.
|*       - Message on entry to interrupt service routine
|*   11: All previous messages, plus:
|*       - Messages describing events extracted from the event FIFO and their timestamps
|*
|* The device-support layer will add additional debug outputs to the above list.
|*
\**************************************************************************************************/
void ErDebugLevel(epicsInt32 level)
{
	ErDebug = level;   /* Set the new debug level */
	return;
}

/**************************************************************************************************
|* ErEnableIrq () -- Enable or Disable Event Receiver Interrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Mask      = (epicsUInt16)    New value to write to the interrupt enable register.
|*                                   In addition to setting a specified bitmask into the
|*                                   interrupt enable register, three additional special
|*                                   codes are defined for the Mask parameter:
|*                                     - EVR_IRQ_OFF:  Disable all interrupts.
|*                                     - EVR_IRQ_ALL:  Enable all interrupts.
|*                                     - EVR_IRQ_TELL: Return (but do not change) the current
|*                                                     value of the interrupt enable register.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Mask_or_Status = (epicsUInt16) If the value of the "Mask" parameter is "EVR_IRQ_TELL",
|*                                     return the current mask of enabled interrupts.
|*                                     Otherwise, return OK.
|*
\**************************************************************************************************/
epicsUInt16 ErEnableIrq (ErCardStruct *pCard, epicsUInt16 Mask)
{
	epicsUInt16 ret;
	
	epicsMutexLock(pCard->CardLock);
	ret = ErEnableIrq_nolock(pCard, Mask);
	epicsMutexUnlock(pCard->CardLock);
	return ret;
}

/**************************************************************************************************
|* ErFinishDrvInit () -- Complete the Event Receiver Board Driver Initialization
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      AfterRecordInit = (int)          0 if the routine is being called before record
|*                                         initialization has started.
|*                                       1 if the routine is being called after record
|*                                         initialzation completes.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK
|*
\**************************************************************************************************/
epicsStatus ErFinishDrvInit(int AfterRecordInit)
{
	return OK;
}

/**************************************************************************************************
|* ErDBuffIrq () -- Enable or Disable Event Receiver "Data Buffer Ready" Intrrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the Data Buffer Ready interrupt.
|*                                   If false, disable the Data Buffer Ready interrupt.
|*
\**************************************************************************************************/
void ErDBuffIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable)
		mask |= EVR_IRQFLAG_DATABUF;
	else
		if (pCard->IrqVector & EVR_IRQFLAG_DATABUF)
			mask ^= EVR_IRQFLAG_DATABUF;	
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErEventIrq () -- Enable or Disable Event Receiver Event Interrupts
|*-------------------------------------------------------------------------------------------------
|*
|* This routine will enable or disable the "Event FIFO" interrupt on the specified
|* Event Receiver card.  The "FIFO-Full" interrupt is also enabled/disabled at the same time.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the event FIFO interrupt.
|*                                   If false, disable the event FIFO interrupt.
|*
\**************************************************************************************************/
void ErEventIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable) {
		mask |= EVR_IRQFLAG_EVENT | EVR_IRQFLAG_FIFOFULL;
	} else {
		if (pCard->IrqVector & EVR_IRQFLAG_EVENT)
			mask ^= EVR_IRQFLAG_EVENT;
		if (pCard->IrqVector & EVR_IRQFLAG_FIFOFULL)
			mask ^= EVR_IRQFLAG_FIFOFULL;
	}
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErGetCardStruct () -- Retrieve a Pointer to the Event Receiver Card Structure
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card        = (epicsInt16)     Card number to get the Event Receiver card structure for.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      pCard       = (ErCardStruct *) Pointer to the requested card structure, or NULL if the
|*                                     requested card was not successfully configured.
|*
\**************************************************************************************************/
ErCardStruct *ErGetCardStruct(int Card)
{
	ErCardStruct  *pCard;
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link))
		if (pCard->Cardno == Card)
			return pCard;
	return NULL;
}

/**************************************************************************************************
|* ErGetFpgaVersion () -- Return the Event Receiver's FPGA Version
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      version   = (epicsUInt16)    The FPGA version of the requested Event Receiver Card.
|*
\**************************************************************************************************/
epicsUInt32 ErGetFpgaVersion(ErCardStruct *pCard)
{	
	epicsUInt32 version;

	version = pCard->FPGAVersion;
	return version;
}

/**************************************************************************************************
|* ErGetSecondsSR () -- Return the Event Receiver's Unlatched Seconds Register
|*-------------------------------------------------------------------------------------------------
|*
|* Read the unlatched timestamp "seconds" from the Event Receiver's SecondsSR register.
|*
|*-------------------------------------------------------------------------------------------------
|* CALLING SEQUENCE:
|*      version = ErGetSecondsSR (pCard);
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      seconds  = (epicsUInt16)    The timestamp "seconds" of the requested Event Receiver Card.
|*
\**************************************************************************************************/

GLOBAL_RTN
epicsUInt32 ErGetSecondsSR (ErCardStruct *pCard)
{
    /* no need for a lock, this is a read only register */
    epicsUInt32		secondsSR = READ_EVR_REGISTER(pCard->Slot, SecondsShift);
    return secondsSR;

}/*end ErGetSecondsSR()*/

/**************************************************************************************************
|* ErGetRamStatus () -- Return the Enabled/Disabled Status of the Requested Event Mapping RAM
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      RamNumber = (int)            Which Event Map RAM (1 or 2) to check.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      enabled   = (epicsBoolean)   True if the specified Event Map RAM is enabled.
|*                                   False if the specified Event Map RAM is not enabled.
|*
\**************************************************************************************************/
epicsBoolean ErGetRamStatus(ErCardStruct *pCard, int RamNumber)
{
	epicsUInt32 ctrl;
	
	epicsMutexLock(pCard->CardLock);
	ctrl = READ_EVR_REGISTER(pCard->Slot, Control);
	epicsMutexUnlock(pCard->CardLock);
	return ((ctrl>>C_EVR_CTRL_MAP_RAM_SELECT) & 1) == (RamNumber-1) ? 
			epicsTrue : epicsFalse;
}

/**************************************************************************************************
|* ErGetTicks () -- Return the Current Value of the Event Counter
|*-------------------------------------------------------------------------------------------------
|*
|* Returns the current value of the "Event Counter", which can represent either:
|*  a) Accumulated timestamp events (Event Code 0x7C),
|*  b) Clock ticks from bit 4 of the distributed data bus, or
|*  c) Scaled event clock ticks.
|* 
|*-------------------------------------------------------------------------------------------------
|* CALLING SEQUENCE:
|*      status = ErGetTicks (Card, &Ticks);
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card   = (int)           Card number of the Event Receiver board to get the event
|*                               the counter from.
|*
|*-------------------------------------------------------------------------------------------------
|* OUTPUT PARAMETERS:
|*      Ticks  = (epicsUInt32 *) Pointer to the Event Receiver card structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status = (epicsStatus)   Returns OK if the operation completed successfully.
|*                               Returns ERROR if the specified card number was not registered.
|*
\**************************************************************************************************/
epicsStatus ErGetTicks(int Card, epicsUInt32 *Ticks)
{
	ErCardStruct *pCard = ErGetCardStruct(Card);

	if(pCard == NULL)
		return ERROR;
	epicsMutexLock(pCard->CardLock);
	*Ticks = (epicsUInt32)EvrGetTimestampCounter(pCard->Slot);
	epicsMutexUnlock(pCard->CardLock);
	return OK;
}

/**************************************************************************************************
|* ErRegisterDevEventHandler () -- Register a Device-Support Level Event Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      EventFunc = (DEV_EVENT_FUNC *) Address of the device-support layer event function.
|*
\**************************************************************************************************/
void ErRegisterDevEventHandler(ErCardStruct *pCard, DEV_EVENT_FUNC EventFunc)
{
	pCard->DevEventFunc = EventFunc;
}

/**************************************************************************************************
|* ErRegisterDevErrorHandler () -- Register a Device-Support Level Error Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      ErrorFunc = (DEV_ERROR_FUNC *) Address of the device-support layer error function.
|*
\**************************************************************************************************/
void ErRegisterDevErrorHandler(ErCardStruct *pCard, DEV_ERROR_FUNC ErrorFunc)
{
	pCard->DevErrorFunc = ErrorFunc;
}

/**************************************************************************************************
|* ErRegisterDevDBuffHandler () -- Register a Device-Support Level Data Buffer Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      DBuffFunc = (DEV_DBUFF_FUNC *) Address of the device-support layer data buffer function.
|*
\**************************************************************************************************/
void ErRegisterDevDBuffHandler (ErCardStruct *pCard, DEV_DBUFF_FUNC DBuffFunc)
{
	pCard->DevDBuffFunc = DBuffFunc;
}

/**************************************************************************************************
|* ErResetAll () -- Reset the Event Receiver Card
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|* o Not much to do now... just disable our interrupts!
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard  = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
\**************************************************************************************************/
void ErResetAll(ErCardStruct *pCard)
{	
	epicsMutexLock(pCard->CardLock);
	ErEnableIrq_nolock(pCard, EVR_IRQ_OFF);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErSetDg () -- Set Parameters for a Programmable Delay (DG) Pulse
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Channel   = (int)            The DG channel (0-3) that we wish to set.
|*      Enable    = (epicsBoolean)   True if we are to enable the selected DG channel.
|*                                   False if we are to disable the selected DG channel
|*      Delay     = (epicsUInt32)    Desired delay for the DG channel.
|*      Width     = (epicsUInt32)    Desired width for the DG channel.
|*      Prescaler = (epicsUInt16)    Prescaler countdown applied to delay and width.
|*      Polarity  = (epicsBoolean)   0 for normal polarity (high true)
|*                                   1 for reverse polarity (low true)
|* 
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetDg(ErCardStruct *pCard, int Channel, epicsBoolean Enable, 
			epicsUInt32 Delay, epicsUInt32 Width, 
			epicsUInt16 Prescaler, epicsBoolean Pol)
{
	if( Channel < 0 || Channel >= EVR_NUM_DG ) {
		errlogPrintf("%s: invalid parameter: Channel = %d.\n", __func__, Channel);
                return;
	}
	if ( Channel >= MAX_DG ) /* Don't complain if SLAC-valid channel but we aren't SLAC! */
		return;

	if ( ErDebug >= 1 )
		printf( "%s: EVR card %d, slot %d %s DG %2d: pre=%u, del=%u, wid=%u, pol=%s.\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable " : "Disable" ),
				Channel, Prescaler, Delay, Width,
				( Pol ? "Inv" : "Nml" )	);

	epicsMutexLock(pCard->CardLock);
	if(Enable) {
            EvrSetPulseParams(pCard->Slot, Channel, Prescaler, Delay, Width, Pol, 1);
	} else {
            EvrSetPulseParams(pCard->Slot, Channel, 0, 0, 0, Pol, 0);
	}

	if ( ErDebug >= 2 )
		EvrDumpPulses( pCard->Slot, 10 );

	epicsMutexUnlock(pCard->CardLock);
	return;
}


/**************************************************************************************************
|* ErTaxiIrq () -- Enable or Disable Event Receiver "TAXI" Violation Interrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the Receive Link Violation interrupt.
|*                                   If false, disable the Receive Link Violation interrupt.
|*
\**************************************************************************************************/
void ErTaxiIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable)
		mask |= EVR_IRQFLAG_VIOLATION;
	else
		if (pCard->IrqVector & EVR_IRQFLAG_VIOLATION)
			mask ^= EVR_IRQFLAG_VIOLATION;	
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErUpdateRam () -- Load and Activate a New Event Map
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      RamBuf    = (epicsUInt16 *)  Pointer to the array of event output mapping words that we
|*                                   will write to the chosen Event Mapping RAM.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErUpdateRam(ErCardStruct *pCard, epicsUInt16 *RamBuf)
{
    int i;
    epicsUInt16 buf[EVR_NUM_EVENTS];

    for (i = 0; i < EVR_NUM_EVENTS; i++)
        buf[i] = RamBuf[i] & pCard->EnableMask;
    ioctl(pCard->Slot, EV_IOCEVTTAB, buf);
}

/**************************************************************************************************
|* ErDrvReport () -- Event Receiver Driver Report Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine gets called by the EPICS "dbior" routine.  For each configured Event Receiver
|* card, it display's the card's slot number, firmware version, hardware address, interrupt
|* vector, interupt level, enabled/disabled status, and number of event link frame errors
|* seen so far.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      level       = (int)     Indicator of how much information is to be displayed
|*                              (currently ignored).
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK;
|*
\**************************************************************************************************/
epicsStatus ErDrvReport (int level)
{
	int             NumCards = 0;       /* Number of configured cards we found                    */
	ErCardStruct *	pCard;              /* Pointer to card structure                              */
	int				EventNum;
	int				ram;

	for (	pCard = (ErCardStruct *)ellFirst(&ErCardList);
			pCard != NULL;
			pCard = (ErCardStruct *)ellNext(&pCard->Link)	) {
		NumCards++;

		printf ("\n-------------------- EVR#%d Hardware Report --------------------\n", pCard->Cardno);
		printf("	Form factor %s.\n", FormFactorToString( ErGetFormFactor(pCard->Slot) ) );
		printf("	Firmware Version = %4.4X.\n", ErGetFpgaVersion(pCard));
		printf ("	Address = %p.\n", pCard->pEq);
		printf ("	%d Frame Errors\n", pCard->RxvioCount);
		EvrDumpStatus( pCard->Slot );
		EvrDumpPulses(		pCard->Slot, 10 );

		/* Dump the active entries in ErEventTab */
		printf( "ErEventTab[code]: 0x8000 is IRQ, 0x0001 is OUT0, 0x0002 is OUT1, ...\n" );
		for (EventNum = 0; EventNum < EVR_NUM_EVENTS; EventNum++ )
		{
			if ( pCard->ErEventTab[EventNum] != 0 )
			{
				if (level == 0)
					printf( "ErEventTab[%3d] = 0x%04x\n", EventNum, pCard->ErEventTab[EventNum] );
				else
				{
					unsigned int chan;
					printf( "ErEventTab[%3d] = 0x%04x", EventNum, pCard->ErEventTab[EventNum] );
					for ( chan = 0; chan < EVR_MAP_N_CHAN_MAX; chan++ )
					{
						if( pCard->ErEventCnt[EventNum][chan] > 0 )
						{
							if ( chan == EVR_MAP_IRQ_CHAN )
								printf( ", IRQ cnt=%d", pCard->ErEventCnt[EventNum][chan] );
							else
								printf( ", out%d cnt=%d", chan, pCard->ErEventCnt[EventNum][chan] );
						}
					}
					printf( "\n" );
				}
			}
		}
		printf("\n");
		if (level >= 2) {
			u32 ie, trig, set, clear;
			struct MapRamItemStruct MapRam[EVR_MAX_EVENT_CODE+1];
			
			if (ErGetRamStatus(pCard, 1))
				ram = 1;
			else
				ram = 0;
			printf("Active ram: %d\n", ram);
			if (!READ_EVR_REGION32(pCard->Slot, MapRam[ram], (u32 *)MapRam, sizeof(MapRam))) {
				printf("  Index     IntEvent  Trigger     Set      Clear\n");
				printf("----------  --------  --------  --------  --------\n");
				for (EventNum = 0; EventNum < EVR_MAX_EVENT_CODE; EventNum++) {
					ie    = MapRam[EventNum].IntEvent;
					trig  = MapRam[EventNum].PulseTrigger;
					set   = MapRam[EventNum].PulseSet;
					clear = MapRam[EventNum].PulseClear;
					if (ie || trig || set || clear) {
						printf("%3d (0x%02x)  %08x  %08x  %08x  %08x\n",
							   EventNum, EventNum, ie, trig, set, clear);
					}
				}
			}
			printf("\n");
		}
	}
	if(!NumCards)
		printf ("  No Event Receiver cards were configured\n");
	return OK;
}

/**************************************************************************************************
|* ErShutdownFunc () -- Disable Event Receiver Interrupts on Soft Reboot
|*-------------------------------------------------------------------------------------------------
|*
|* This is an "exit handler" which is invoked when the IOC is soft rebooted.
|* It is enabled in the driver initialization routine (ErDrvInit) via a call to "epicsAtExit".
|*
|*-------------------------------------------------------------------------------------------------
|* IMPLICIT INPUTS:
|*      ErCardList  = (ELLLIST) Linked list of all configured Event Receiver card structures.
|*
\**************************************************************************************************/
void ErShutdownFunc (void *arg)
{
	ErCardStruct  *pCard;

	/* Loop to close all cards */
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link)) {
		close(pCard->Slot);
	}
}

/**************************************************************************************************
|* ErDrvInit () -- Driver Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It gets called prior to any of the
|* device or record support initialization routines.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*    o Disable any further calls to the ErConfigure routine.
|*    o Add a hook into EPICS to disable Event Receiver card interrupts in the event of a
|*      soft reboot.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK;
|*
\**************************************************************************************************/
epicsStatus ErDrvInit (void)
{
	epicsAtExit (&ErShutdownFunc, NULL);

	return OK;
}

/**************************************************************************************************/
/*                              EPICS records and PVs                                             */
/*                                                                                                */


drvet drvMrf200Er =
{
    2,                                  /* Number of entries in the table                         */
    (DRVSUPFUN)ErDrvReport,             /* Driver Support Layer device report routine             */
    (DRVSUPFUN)ErDrvInit                /* Driver Support layer device initialization routine     */
};

epicsExportAddress (drvet, drvMrf200Er);


/**************************************************************************************************/
/*                              EPICS iocsh extension                                             */
/*                                                                                                */

/* iocsh command: ErConfigure */
LOCAL const iocshArg ErConfigureArg0 = {"Card"       , iocshArgInt};
LOCAL const iocshArg ErConfigureArg1 = {"CardAddress", iocshArgInt};
LOCAL const iocshArg ErConfigureArg2 = {"IrqVector"  , iocshArgInt};
LOCAL const iocshArg ErConfigureArg3 = {"IrqLevel"   , iocshArgInt};
LOCAL const iocshArg ErConfigureArg4 = {"FormFactor"    , iocshArgInt};
LOCAL const iocshArg *const ErConfigureArgs[5] = {
							&ErConfigureArg0,
							&ErConfigureArg1,
							&ErConfigureArg2,
							&ErConfigureArg3,
							&ErConfigureArg4
						};
LOCAL const iocshFuncDef    ErConfigureDef     = {"ErConfigure", 5, ErConfigureArgs};

LOCAL_RTN void ErConfigureCall(const iocshArgBuf * args)
{
	ErConfigure(args[0].ival, (epicsUInt32)args[1].ival,
			(epicsUInt32)args[2].ival, (epicsUInt32)args[3].ival,
			args[4].ival);
}

/* iocsh command: ErDebugLevel */
LOCAL const iocshArg ErDebugLevelArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const ErDebugLevelArgs[1] = {&ErDebugLevelArg0};
LOCAL const iocshFuncDef ErDebugLevelDef = {"ErDebugLevel", 1, ErDebugLevelArgs};

LOCAL_RTN void ErDebugLevelCall(const iocshArgBuf * args)
{
	ErDebugLevel((epicsInt32)args[0].ival);
}

/* iocsh command: ErDrvReport */
LOCAL const iocshArg ErDrvReportArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const ErDrvReportArgs[1] = {&ErDrvReportArg0};
LOCAL const iocshFuncDef ErDrvReportDef = {"ErDrvReport", 1, ErDrvReportArgs};

LOCAL_RTN void ErDrvReportCall(const iocshArgBuf * args)
{
	ErDrvReport((epicsInt32)args[0].ival);
}

/* Registration APIs */
LOCAL void drvMrfErRegister()
{
	/* Initialize global variables */
	ErCardListLock = epicsMutexCreate();
	ErConfigureLock = epicsMutexCreate();
	/* register APIs */
	iocshRegister(	&ErConfigureDef,	ErConfigureCall );
	iocshRegister(	&ErDebugLevelDef,	ErDebugLevelCall );
	iocshRegister(	&ErDrvReportDef,	ErDrvReportCall );
}
epicsExportRegistrar(drvMrfErRegister);
