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
    int lasterrno = 0;
    int errnocnt = 0;

    free(p);

    while(1) {
        errno = 0;
        cnt = read(fd, &mask, sizeof(mask));
        if (cnt == sizeof(mask)) {
            if (*irqHandler) (*irqHandler)(fd, mask);
            if (lasterrno) {
                lasterrno = 0;
                errnocnt = 0;
            }
        } else if (cnt < 0) {
            if (errno != EINTR) {
                perror("evrIrqHandlerThread has an unknown error");
                if (errno == lasterrno) {
                    if (++errnocnt == 10) {
                        printf("evrIrqHandlerThread is exiting.\n");
                        /* 
                         * Hey, we tried... we should probably kill
                         * everything.  But we won't.
                         */
                        break;
                    }
                } else {
                    lasterrno = errno;
                    errnocnt = 0;
                }
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
