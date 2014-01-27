#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <epicsThread.h>

struct evrIrqThreadArg {
    void (**handler)(int, int);
    int fd;
};

static int evrIrqHandlerThread(void *p)
{
    struct evrIrqThreadArg *arg = (struct evrIrqThreadArg *)p;
    void (**irqHandler)(int, int) = arg->handler;
    int fd = arg->fd;
    int mask, cnt;

    free(p);

    while(1) {
        cnt = read(fd, &mask, sizeof(mask));
        if (cnt == sizeof(mask)) {
            if (*irqHandler) (*irqHandler)(fd, mask);
        } else if (cnt < 0) {
            if (errno != EINTR) {
                perror("evrIrqHandlerThread has an unknown error");
                printf("evrIrqHandlerThread is exiting.\n");
                /*
                 * Nothing will work after this.  Do we want to just
                 * exit(1) and call it a day?!?
                 */
                break;
            }
        } else {
            printf("evrIrqHandlerThread read %d bytes?!?\n", cnt);
        }
    }

    return 0;
}

int evrIrqHandlerInit( void )
{
    return 0; /* Not needed any more! */
}

void EvrIrqHandlerThreadCreate(void (**handler) (int, int), int fd)
{
    struct evrIrqThreadArg *arg = (struct evrIrqThreadArg *)
                                  malloc(sizeof(struct evrIrqThreadArg));
    arg->handler = handler;
    arg->fd = fd;
    epicsThreadMustCreate("evrIrqHandler", epicsThreadPriorityHigh+9,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)evrIrqHandlerThread,arg);
}
