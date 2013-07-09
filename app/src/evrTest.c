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

void ErIrqHandler(int fd, int flags)
{
    int i;
    irqCount++;

    printf("I%08x\n", flags);

    /* This should always be here 2ms before the fiducial event */
    if(flags & EVR_IRQFLAG_DATABUF) {
        long long drp = pEq->dwp - 1; /* Read the latest! */
        if (drp != mydrp) {
            int idx = drp & (MAX_EVR_DBQ - 1);
            int databuf_sts = pEq->dbq[idx].status;

            if(databuf_sts & (1<<C_EVR_DATABUF_CHECKSUM)) {
                putchar('C');
            } else {
                u32 *dd = pEq->dbq[idx].data;
                printf("D%d: %08x %08x.%08x\n", idx, dd[0], dd[7], dd[8]);
#if 0
                pCard->DBuffSize = (databuf_sts & ((1<<(C_EVR_DATABUF_SIZEHIGH+1))-1));
                memcpy(pCard->DataBuffer, pEq->dbq[idx].data, pCard->DBuffSize);
#endif
            }
        }
        /* TBD - Check if we skipped some? */
        mydrp = drp;
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

    memset(ErEventTab, 0, sizeof(ErEventTab));
    fd = EvrOpen(&mem, "/dev/era4");
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
                else {
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
