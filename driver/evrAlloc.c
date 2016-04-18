#include<stdio.h>
#include<sys/ioctl.h>
#define DEFINE_READ_EVR
#define INLINE_READ_EVR static __inline__
#include<evrmemmap.h>
#include<unistd.h>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<errno.h>
#include<stdlib.h>
#include<stddef.h>

void usage(void)
{
    printf("Usage: evrAlloc [ -d DEVICE ] [ -s SLEEP ] OP TRIGGER\n");
    printf("       evrAlloc [ -d DEVICE ]\n");
    printf("       evrAlloc -h\n");
    printf("The first form performs an EVR trigger allocation operation.\n");
    printf("    OP and TRIGGER are integers.\n");
	printf("    OP: 0=free, 1=allocate, 2=steal, 3=check\n");
	printf("    TRIGGER: trigger number\n");
	printf("    Specify a sleep duration in sec to sleep then free the trigger.\n");
    printf("The second form checks ownership of *all* triggers.\n");
    printf("The third form prints this help message.\n");
    exit(0);
}

char *name(int op)
{
    static char *opv[4] = {
        "free",
        "allocate",
        "steal",
        "check"
    };
    if (op > 3)
        return "INVALID";
    else
        return opv[op];
}

void report(int op, int id, int ret)
{
    printf("%s(%d) --> %d", name(op), id, ret);
    if (ret < 0)
        printf(" %s\n", strerror(errno));
    else
        printf("\n");
}

int main(int argc, char **argv)
{
    char *device = "/dev/era4";
    int fd = -1;
    int i = 1;
    int stime = 0;
    int len;
    struct EvrIoctlTrig eit;
    int ret;

    while (i < argc) {
        if (!strcmp(argv[i], "-h")) {
            usage();
            i++;
        } else if (!strcmp(argv[i], "-d")) {
            if (i + 1 < argc) {
                device = argv[i+1];
                i += 2;
            } else
                usage();
        } else if (!strcmp(argv[i], "-s")) {
            if (i + 1 < argc) {
                stime = atoi(argv[i+1]);
                i += 2;
            } else
                usage();
        }
        break;
    }
    fd = open(device, O_RDWR);
	if ( fd < 0 ) {
		printf( "Error: Unable to open %s\n", device );
		return -1;
	}
    if (i == argc) {
		int FPGAVersion = READ_EVR_REGISTER(fd, FPGAVersion);
		printf( "EVR Found with Firmware Revision 0x%04X\n", FPGAVersion );
        if ( ((FPGAVersion >> 24) & 0x0f) == 15 )
            len = 12;
        else
            len = 4;
		printf( "Checking all trigger channels.\n" );
        for (i = 0; i < len; i++) {
            eit.Id = i;
            eit.Op = EvrTrigCheck;
            if (ioctl(fd, EV_IOCTRIG, &eit) < 0) {
                if (errno == EBUSY)
                    printf("%2d: In Use!\n", i);
                else if ( errno == ENOTTY ) {
                    printf("%2d: Driver version does not support trigger allocation.  Update driver.\n", i );
					break;
				}
                else
                    printf("%2d: Unexpected error (%d)?\n", i, errno);
            } else {
                printf("%2d: Free\n", i);
            }
        }
    } else if (i + 2 > argc) {
        usage();
    } else {
        eit.Op = atoi(argv[i]);
        eit.Id = atoi(argv[i+1]);
        ret = ioctl(fd, EV_IOCTRIG, &eit);
        report(eit.Op, eit.Id, ret);

        if (stime != 0) {
            sleep(stime);
            if (eit.Op == EvrTrigAlloc || eit.Op == EvrTrigSteal) {
                eit.Op = EvrTrigFree;
                ret = ioctl(fd, EV_IOCTRIG, &eit);
                report(eit.Op, eit.Id, ret);
            }
        }
        printf("Exiting\n");
    }
    return 0;
}
