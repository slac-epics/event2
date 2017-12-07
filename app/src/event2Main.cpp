#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"
#include "evrIrqHandler.h"

int main(int argc,char *argv[])
{
    char *name = argc >= 2 ? argv[1] : NULL;

	// EVR module must block SIGIO from the main process
	evrIrqHandlerInit( );

    if (name) {    
        iocsh(name);
        epicsThreadSleep(0.2);
    }
    iocsh(NULL);
    epicsExit(0);
    return(0);
}
