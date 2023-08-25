#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "evrIrqHandler.h"
#include "erapi.h"

#define EVR_FIFO_EVENT_LIMIT 256
int fd;
struct EvrQueues *pEq;
int irqCount = 0;
long long mydrp = -1;
long long myerp = -1;
u16 ErEventTab[256];
int noI = 0;
long long lasttsc = 0;

#if	defined(__x86_64__)
#define		read_tsc(tscVal)	     			       \
do								       \
{	/*	=A is for 32 bit mode reading 64 bit integer */	       \
	unsigned long	tscHi, tscLo;      			       \
	__asm__ volatile("rdtsc" : "=a" (tscLo), "=d" (tscHi) );       \
	tscVal	=  (long long)( tscHi ) << 32;      		       \
	tscVal	|= (long long)( tscLo );                               \
}	while ( 0 )
#elif	defined(__i386__)
#define	read_tsc(tscVal)	__asm__ volatile("rdtsc": "=A" (tscVal))
#endif

double ms_per_tick = 1.0L / 2400148.L;

void ErIrqHandler(int fd, int flags)
{
    int i;
    irqCount++;

    if (!noI) {
        long long tsc;
        read_tsc(tsc);
        printf("I %5.3lf %08x\n", (tsc - lasttsc) * ms_per_tick, flags);
        lasttsc = tsc;
    }

    /* This should always be here 2ms before the fiducial event */
    if(flags & EVR_IRQFLAG_DATABUF) {
        long long lastdrp = pEq->dwp - 1; /* Read the latest! */
	long long drp;
	if (mydrp == -1)
	    mydrp = lastdrp - 1;
	for (drp = mydrp + 1; drp <= lastdrp; drp++) {
            int idx = drp & (MAX_EVR_DBQ2 - 1);
            int databuf_sts = pEq->dbq2[idx].status;

            if(databuf_sts & (1<<C_EVR_DATABUF_CHECKSUM)) {
                printf("C%d\n", idx);
            } else {
                u32 *dd = pEq->dbq2[idx].data;
                printf("D%02d: %08x %08x.%08x\n", idx, dd[0], dd[7], dd[8]);
#if 0
                pCard->DBuffSize = (databuf_sts & ((1<<(C_EVR_DATABUF_SIZEHIGH+1))-1));
                memcpy(pCard->DataBuffer, pEq->dbq2[idx].data, pCard->DBuffSize);
#endif
            }
        }
        /* TBD - Check if we skipped some? */
        mydrp = lastdrp;
    } else {
        /* We must have skipped one earlier, but caught up? */
    }

    if(flags & EVR_IRQFLAG_EVENT) {
        long long erplimit = pEq->ewp;     /* Pointer to the latest! */
        if (myerp == -1) {
            myerp = erplimit - 1;
        }
        if (erplimit - myerp > MAX_EVR_EVTQ / 2) {
            /* Wow, we're far behind! Catch up a bit, but flag an error. */
            myerp = erplimit - MAX_EVR_EVTQ / 2;
            flags |= EVR_IRQFLAG_FIFOFULL;
            putchar('f');
        }
        for(i=0; myerp < erplimit && i < EVR_FIFO_EVENT_LIMIT; myerp++) {
            struct FIFOEvent *fe = &pEq->evtq[myerp & (MAX_EVR_EVTQ - 1)];
            if (ErEventTab[fe->EventCode] & (1 << 15)) {
                printf("E%02x@%x\n", fe->EventCode, fe->TimestampHigh);
                i++;
            }
        }
    }

    if(flags & EVR_IRQFLAG_PULSE) {
        putchar('P');
    }
    if(flags & EVR_IRQFLAG_HEARTBEAT) {
        putchar('H');
    }
    if(flags & EVR_IRQFLAG_FIFOFULL) {
        putchar('F');
    }
    if(flags & EVR_IRQFLAG_VIOLATION) {
        putchar('V');
    }
    putchar('\n');
    fflush(stdout);
    return;
}

void ErEnableIrq (u32 Mask)
{
    if (Mask == 0) {
        printf("Setting IRQMASK to %08x\n", Mask);
        ioctl(fd, EV_IOCIRQMASK, &Mask);
    } else {
        Mask |= EVR_IRQ_MASTER_ENABLE;
        printf("Setting IRQMASK to %08x\n", Mask);
        ioctl(fd, EV_IOCIRQMASK, &Mask);
    }
}

void ErUpdateRam()
{
    ioctl(fd, EV_IOCEVTTAB, ErEventTab);
}

void ErSetEvent(int id, int enable)
{
    if (enable)
        ErEventTab[id] = 1 << 15;
    else
        ErEventTab[id] = 0;
}

int main(int argc,char *argv[])
{
    int haveevent = 0;
    int dbuf = 0;
    int i;
    void *mem;
    char *dev = getenv("EVRNAME");

    memset(ErEventTab, 0, sizeof(ErEventTab));
    dev = dev ? dev : "/dev/era4";
    printf("Opening %s.\n", dev);
    fd = EvrOpen(&mem, dev);
    pEq = (struct EvrQueues *) mem;
    if (fd < 0) {
        perror("evrTest");
        exit(1);
    }
    EvrIrqAssignHandler(fd, ErIrqHandler);

    if (argc > 1) {
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'd')
                    dbuf = EVR_IRQFLAG_DATABUF;
                else if (argv[i][1] == 'i')
                    noI = 1;
                else if (argv[i][1] == 'h') {
                    printf("event2Test [ -d ] [ -i ] N1 N2...\n");
                    printf("    where -d = use data buffers\n");
                    printf("          -i = don't print IRQ entry\n");
                    printf("          Nn = event code to enable\n");
                    exit(0);
                } else {
                    printf("Unknown argument: %s\n", argv[i]);
                }
            } else if (argv[i][0] >= '0' && argv[i][0] <= '9') {
                int event = atoi(argv[i]);
                printf("Enabling event %d.\n", event);
                ErSetEvent(event, 1);
                haveevent = 1;
            } else
                printf("Unknown argument: %s\n", argv[i]);
        }
    }
    if (!haveevent)
        ErSetEvent(46, 1); /* 0.5 Hz */
    ErEnableIrq(EVR_IRQFLAG_EVENT | dbuf);
    ErUpdateRam();

    for (;;) {
        sleep(1);
    }

    EvrClose(fd);
}
