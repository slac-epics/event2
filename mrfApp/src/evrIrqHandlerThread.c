#include <stdio.h>
#include <signal.h>
#include <epicsThread.h>

static int evrIrqHandlerThread(void *handler)
{
    void (**irqHandler)(int) = handler;
    int signal;
    sigset_t  sigSet;

    sigemptyset(&sigSet);      
    sigaddset(&sigSet,SIGIO);  

    while(1) {
        sigwait(&sigSet, &signal); 
        if (*irqHandler) (*irqHandler)(signal);
    }

    return 0;
}

static	unsigned int	fBlockedSIGIO	= 0;

int evrIrqHandlerInit( void )
{
	int			status;
	sigset_t	sigs;

	/* No one should get SIGIO except the IRQ thread. */
	sigemptyset( &sigs );
	sigaddset( &sigs, SIGIO );
	status = pthread_sigmask( SIG_BLOCK, &sigs, NULL );
	if ( status == 0 )
		fBlockedSIGIO	= 1;
	else
		perror( "evrIrqHandlerInit: Unable to block SIGIO signals" );
	return status;
}

void EvrIrqHandlerThreadCreate(void (**handler) (int))
{

	if ( !fBlockedSIGIO )
	{
		fprintf( stderr,
				"\n\n"
				"ERROR in EvrIrqHandlerThreadCreate: evrIrqHandlerInit() either failed or\n"
				"was not called in the process main() routine.\n"
				"EVR will NOT function until this is fixed!\n\n" );
		return;
	}

    epicsThreadMustCreate("evrIrqHandler", epicsThreadPriorityHigh+9,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)evrIrqHandlerThread,handler);
}
