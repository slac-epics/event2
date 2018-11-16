#ifdef __rtems__
#include <rtems.h>            /* required for timex.h      */
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/timex.h>        /* for ntp_adjtime           */
#include "dbAccess.h"
#include "epicsTypes.h"
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "longSubRecord.h"    /* for struct longSubRecord */
#include "aSubRecord.h"       /* for struct aSubRecord  */
#include "alarm.h"           

static long lsubTrigSelInit(longSubRecord *prec)
{
    if ( prec->tpro )
    	printf("lsubTrigSelInit for %s\n", prec->name);

    return 0;
}

/* Find index of first non-zero input */
static long lsubTrigSel(longSubRecord *prec)
{
	epicsUInt32    i   = 0;
	epicsUInt32    *p  = &prec->a;
	epicsUInt32    *pp = &prec->z;

	for(i=0; (p+i) <= pp; i++) {
		if(*(p+i)) {
			prec->val = i;
			if ( prec->tpro >= 2 )
				printf( "lsubTrigSel %s: Input %u non-zero\n", prec->name, i );
			return 0;
		}
	}

	if ( prec->tpro >= 2 )
		printf( "lsubTrigSel %s: No inputs non-zero!\n", prec->name );
	prec->val	= -1;
	prec->brsv	= INVALID_ALARM;
	return -1;
}

static long lsubEvSelInit(longSubRecord *prec)
{
	if ( prec->tpro )
    	printf("lsubEvSelInit for %s\n", prec->name);

    return 0;
}

static long lsubEvSel(longSubRecord *prec)
{
    epicsInt32		i	= prec->z;
    epicsUInt32	*	p	= &prec->a;

    if ( i < 0 || i >= 24 )
	{
		prec->val	= 0;
		prec->brsv	= INVALID_ALARM;
    	return -1;
	}

	prec->val = *(p+i);
	if ( prec->tpro >= 2 )
    	printf("lsubEvSel %s: Input %u is %u\n", prec->name, i, prec->val);

    return 0;
}

static long aSubEvOffsetInit(aSubRecord *prec)
{
	assert( dbValueSize(prec->fta)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftb)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftc)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftd)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftva) == sizeof(epicsUInt32) );

    return 0;
}

/*
 *   -----------------
 *   Input/Output list 
 *   -----------------
 *
 * INPA: Input for event number: event number selector (epicsUInt32 type)
 * INPB: Activate/Deactivate event invariant delay (epicsUInt32 type)
 * INPC: Input for EVG delay - lookup PV (epicsUInt32 type waveform)
 * INPD: Input for previous delay (just in case, if EVNT:SYSx:1:DELAY array is not available, invalid severity)

 * OUTA: Output for delay
 *
 */
static long aSubEvOffset(aSubRecord *prec)
{
    epicsUInt32		eventNumber		= *(epicsUInt32*)(prec->a);
    epicsUInt32		activeFlag		= *(epicsUInt32*)(prec->b);
    epicsUInt32 *	pdelayArray		=  (epicsUInt32*)(prec->c);
    epicsUInt32		defaultDelay	= *(epicsUInt32*)(prec->d);
    epicsUInt32	*	poutputDelay	=  (epicsUInt32*)(prec->vala);
    epicsEnum16		sevr			= 0;

#if 0
/*
 * Don't want this severity check now that we cache a copy of the EVG DELAYS in $EVR:EC:DELAYS
 * The local cache allows us to compute delays from autosaved values even when EVG is offline
 * so invariant timing works even w/ test EVG's providing the timing signal.
 */
    if(dbGetSevr(&prec->inpc, &sevr)) {
        printf("%s: CA connection severity check error\n", prec->name);
        return 0;
    }
#endif

    if(sevr                          ||     /* record is not initialized */
       !activeFlag                   ||     /* deactivate */
       !pdelayArray                  ||     /* no lookup table */
       (eventNumber<0 || eventNumber>255)   /* out of range for event number */) {
        *poutputDelay = defaultDelay;                   /* if something is wrong, just use default delay */
		if ( prec->tpro >= 2 )
			printf( "aSubEvOffset %s ERROR: Unable to lookup EC %d, using def delay %u\n", prec->name, eventNumber, defaultDelay );
    }
    else {
		/* Everything is OK, let's use look up table */
    	epicsUInt32		outputDelay	=  *(pdelayArray + eventNumber);
		if ( prec->tpro >= 2 )
			printf( "aSubEvOffset %s: EC %d delay is %u\n", prec->name, eventNumber, outputDelay );
		*poutputDelay = outputDelay;

		/* Save the computed delay as the new default */
		*(epicsUInt32*)(prec->d) = outputDelay;
	}
    return 0;
}

static long aSubEvrDevTrigInit(aSubRecord *prec)
{
	/* Input and Output parameters */
	assert( dbValueSize(prec->fta)  == sizeof(epicsFloat64) );
	assert( dbValueSize(prec->ftva) == sizeof(epicsFloat64) );
	assert( dbValueSize(prec->ftb)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftvb) == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftc)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftvc) == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftd)  == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->ftvd) == sizeof(epicsUInt32) );
	assert( dbValueSize(prec->fte)  == sizeof(epicsFloat64) );
	assert( dbValueSize(prec->ftve) == sizeof(epicsFloat64) );

	/* Input only parameters */
	assert( dbValueSize(prec->ftm)  == sizeof(epicsInt32) );
	assert( dbValueSize(prec->ftn)  == sizeof(epicsInt32) );
	assert( dbValueSize(prec->fto)  == sizeof(epicsFloat64) );
//	assert( dbValueSize(prec->ftp)  == sizeof(epicsFloat64) );
	assert( dbValueSize(prec->ftq)  == sizeof(epicsUInt32) );

	/* Output only parameters */

    return 0;
}

/*
 *   -----------------
 *   Input/Output list 
 *   -----------------
 *
 * I/O Parameters
 *	INPA,OUTA:	Desired trigger delay (TDES)
 *	INPB,OUTB:	Trigger delay in ticks ($EVR:CTRL.DG*D)
 *	INPC,OUTC:	Trigger scale factor ($EVR:CTRL.DG*C)
 *	INPD,OUTD:	User specified trigger event code (TEC)
 *	INPE,OUTE:	DEV trigger event code, normally empty string, overridden by camera IOC
 *	INPF,OUTF:	DEV trigger delay,      normally empty string, overridden by camera IOC
 * Input only Parameters
 *	INPM:	0 (disable) or 1 (enable) invariant timing
 *	INPN:	Currently selected trigger event code from EVENT*CTRL records (EC_RBV)
 *	INPO:	Trigger reference time (TREF)
 *	INPP:	Current trigger event code offset in ticks (TOFFSET)
 *
 */
static long aSubEvrDevTrig(aSubRecord *prec)
{
    epicsFloat64	newTDES			= *(epicsFloat64*)(prec->a);
    epicsUInt32		newDGTickDelay	= *(epicsUInt32 *)(prec->b);
    epicsUInt32		newDGTickScale	= *(epicsUInt32 *)(prec->c);
    epicsUInt32		newTEC			= *(epicsUInt32 *)(prec->d);
    epicsUInt32		enableInvariant	= *(epicsUInt32 *)(prec->m);
    epicsUInt32		newEC_RBV		= *(epicsUInt32 *)(prec->n);
    epicsFloat64	newTREF			= *(epicsFloat64*)(prec->o);
    epicsUInt32 *	pDelayArray		=  (epicsUInt32 *)(prec->p);
    epicsUInt32		fStartingUp		= *(epicsUInt32 *)(prec->q);

    epicsFloat64	oldTDES			= *(epicsFloat64*)(prec->ovla);
    epicsUInt32		oldDGTickDelay	= *(epicsUInt32 *)(prec->ovlb);
    epicsUInt32		oldDGTickScale	= *(epicsUInt32 *)(prec->ovlc);
    epicsUInt32		oldTEC			= *(epicsUInt32 *)(prec->ovld);
    epicsFloat64	oldTOFFSET		= *(epicsFloat64*)(prec->ovle);

    const epicsFloat64	nsPerTick	= 1e9 / 119e6;	/* ~8.4 ns per tick */
	int				fUpdateOutputs	= 0;
	
	epicsFloat64	newTOFFSET		= 0;
	if (	pDelayArray != NULL
		&&	newTEC >= 0
		&&	newTEC <  prec->nop )
	{
		newTOFFSET = *(pDelayArray + newTEC);
	}

	if ( prec->tpro >= 2 )
	{
		printf( "aSubEvrDevTrig %s: OLD,     %s TDES=%.0f, TEC=%d, TOFFSET=%.0f, DG=%d\n", prec->name,
				( fStartingUp ? "StartingUp" : "InvariantOff" ),
				oldTDES, oldTEC, oldTOFFSET, oldDGTickDelay );
		printf( "aSubEvrDevTrig %s: OnEntry, %s TDES=%.0f, TEC=%d, TOFFSET=%.0f, DG=%d\n", prec->name,
				( fStartingUp ? "StartingUp" : "InvariantOff" ),
				newTDES, newTEC, newTOFFSET, newDGTickDelay );
	}
	if ( !fStartingUp && enableInvariant )
	{
		if (	( newTDES		!= oldTDES )
			||	( newDGTickScale!= oldDGTickScale )
		/*	||	( newTEC		!= oldTEC ) */
		/*	||	( newEC_RBV		!= oldTEC ) */
			||	( newTOFFSET	!= oldTOFFSET ) )
		{	/* New TDES from user */
			/* Compute total delay in ns */
			epicsFloat64	TDLY_NS	= newTDES + newTREF - (newTOFFSET * nsPerTick);
			if ( TDLY_NS >= 0 )
			{
				/* Convert to ticks */
				newDGTickDelay		= floor( (TDLY_NS / nsPerTick / newDGTickScale ) + 0.5 );
				fUpdateOutputs		= 1;
				if ( prec->tpro >= 2 )
					printf( "aSubEvrDevTrig %s: DG=(%.0f + %.0f - %.0f*8.4)/8.4/%d = %d\n", prec->name,
							newTDES, newTREF, newTOFFSET, newDGTickScale, newDGTickDelay );
			}
			if ( 0 && newTEC	== oldTEC && newEC_RBV != newTEC )
			{
				newTEC = newEC_RBV;
				fUpdateOutputs = 1;
				if ( prec->tpro >= 2 )
					printf( "aSubEvrDevTrig %s: newTEC = %d\n", prec->name, newTEC );
			}
		}
		else if ( newDGTickDelay != oldDGTickDelay )
		{	/* New tick delay from user */
			epicsUInt32		BW_TDLY			= newDGTickDelay * newDGTickScale;
			epicsFloat64	BW_TDES_CALC	= (newTOFFSET + BW_TDLY) * nsPerTick - newTREF;
			epicsFloat64	BW_TDES			= BW_TDES_CALC;
			newTDES	= BW_TDES;
			fUpdateOutputs		= 1;
			if ( prec->tpro >= 2 )
				printf( "aSubEvrDevTrig %s: TDES=(%.0f + %d * %d)*8.4 - %.0f = %.0f\n", prec->name,
						newTOFFSET, newDGTickDelay, newDGTickScale, newTREF, newTDES );
		}
	}
	else
	{
		if ( prec->tpro >= 3 )
			printf( "aSubEvrDevTrig %s: %s TDES=%.0f, TEC=%d, TOFFSET=%.0f, DG=%d\n", prec->name,
					( fStartingUp ? "StartingUp" : "InvariantOff" ),
					newTDES, newTEC, newTOFFSET, newDGTickDelay );
	}

	/* Update VALA, VALB, ... fields */
	*(epicsFloat64*)(prec->vala)	= newTDES;
	*(epicsUInt32 *)(prec->valb)	= newDGTickDelay;
	*(epicsUInt32 *)(prec->valc)	= newDGTickScale;
	*(epicsUInt32 *)(prec->vald)	= newTEC;
	*(epicsFloat64*)(prec->vale)	= newTOFFSET;

	if ( fUpdateOutputs == 0 )
	{
		/* Update the old values so we don't trigger monitors */
		*(epicsFloat64*)(prec->ovla)	= newTDES;
		*(epicsUInt32*)(prec->ovlb)		= newDGTickDelay;
		*(epicsUInt32*)(prec->ovlc)		= newDGTickScale;
		*(epicsUInt32*)(prec->ovld)		= newTEC;
		*(epicsFloat64*)(prec->ovle)	= newTOFFSET;
		return 1;
	}
    return 0;
}

static long lsubCountNonZeroInit(longSubRecord *prec)
{
    if ( prec->tpro )
		printf("lsubCountNonZeroInit for %s\n", prec->name);
    return 0;
}

static long lsubCountNonZero(longSubRecord *prec)
{
	epicsUInt32    count	= 0;
	epicsUInt32    i		= 0;
	epicsUInt32    *p		= &prec->a;
	epicsUInt32    *pp		= &prec->z;

	for ( i = 0; (p+i) <= pp; i++ )
	{
		if ( *(p+i) )
			++count;
	}

	if ( prec->tpro >= 2 )
		printf("lsubCountNonZero %s: Count is %u\n", prec->name, count );

	prec->val	= count;
	return 0;
}

#define	N_ASUB_ARGS		21
static long asubCopyInToOutInit( aSubRecord * prec )
{
    epicsUInt32		i;
    void		**	pVoidIn		= &prec->a;
    void		**	pVoidOut	= &prec->vala;
    epicsUInt32	**	pIn			= (epicsUInt32 **)pVoidIn;
    epicsUInt32	**	pOut		= (epicsUInt32 **)pVoidOut;
    epicsUInt32	*	pInCnt		= &prec->nea;
    epicsUInt32	*	pOutCnt		= &prec->noa;
    epicsEnum16	*	pInType		= &prec->fta;
    epicsEnum16	*	pOutType	= &prec->ftva;

	if ( prec->tpro )
    	printf("asubCopyInToOutInit for %s\n", prec->name);
    for ( i = 0; i < N_ASUB_ARGS; i++ )
	{
		assert( dbValueSize(*pInType)	== dbValueSize(DBF_ULONG) );
		assert( dbValueSize(*pOutType)	== dbValueSize(DBF_ULONG) );
		if ( prec->tpro >= 3 )
		{
			printf( "INP%c: pIn  = %p, cnt = %u\n",
					'A' + i, pIn[i], *pInCnt );
			printf( "OUT%c: pOut = %p, cnt = %u\n",
					'A' + i, pOut[i], *pOutCnt );
		}
		pInCnt++;	pOutCnt++;
		pInType++;	pOutType++;
	}

    return 0;
}

static long asubCopyInToOut( aSubRecord * prec )
{
    epicsUInt32		i;
    void		**	ppVoidIn	= &prec->a;
    void		**	ppVoidOut	= &prec->vala;
    epicsUInt32	**	ppIn		= (epicsUInt32 **)ppVoidIn;
    epicsUInt32	**	ppOut		= (epicsUInt32 **)ppVoidOut;
    epicsUInt32	*	pInCnt		= &prec->nea;
    epicsUInt32	*	pOutCnt		= &prec->nova;

	if ( prec->tpro >= 2 )
    	printf("asubCopyInToOut(%s)\n", prec->name);
    for ( i = 0; i < N_ASUB_ARGS; i++ )
	{
		size_t		count	= *pInCnt;
		if( count > *pOutCnt )
			count = *pOutCnt;
		if ( prec->tpro >= 3 )
			printf( "%s.OUT%c: memcpy( to %p, from %p, cnt %zu ) *ppIn[%d][0]=%u\n",
					prec->name, 'A' + i, ppOut[i], ppIn[i], count, i, *(ppIn[i]) );
		memcpy( ppOut[i], ppIn[i], count * sizeof(epicsUInt32) );
		pInCnt++; pOutCnt++;
	}

	if ( prec->tpro >= 3 )
    	printf("asubCopyInToOut(%s) done.\n", prec->name);
    return 0;
}

epicsRegisterFunction(lsubTrigSelInit);
epicsRegisterFunction(lsubTrigSel);
epicsRegisterFunction(lsubEvSelInit);
epicsRegisterFunction(lsubEvSel);
epicsRegisterFunction(aSubEvOffsetInit);
epicsRegisterFunction(aSubEvOffset);
epicsRegisterFunction(aSubEvrDevTrigInit);
epicsRegisterFunction(aSubEvrDevTrig);
epicsRegisterFunction(lsubCountNonZeroInit);
epicsRegisterFunction(lsubCountNonZero);
epicsRegisterFunction(asubCopyInToOutInit);
epicsRegisterFunction(asubCopyInToOut);
