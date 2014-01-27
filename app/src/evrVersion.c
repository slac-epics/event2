#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include <fcntl.h>
#include <endian.h>
#include <byteswap.h>
#define  DEFINE_READ_EVR
#include "erapi.h"

#define VERSIONREG    0x002C
#define VERSIONOFFSET (VERSIONREG >> 2)

struct kind {
    unsigned int mask;
    unsigned int val;
    char *desc;
} kinds[] = {
    { 0xff000000, 0x10000000, "cPCI 3U" },
    { 0xffffffff, 0x11000002, "PMC: needs upgrade!!" },
    { 0xffffffff, 0x11000103, "PMC: latest version" },
    { 0xff000000, 0x11000000, "PMC: unknown version?!?" },
    { 0xff000000, 0x12000000, "VME64x" },
    { 0xff000000, 0x13000000, "cRIO" },
    { 0xff000000, 0x14000000, "cPCI 6U" },
    { 0xff000000, 0x1f000000, "SLAC" },
    { 0, 0, NULL }
};

char *getkind(unsigned int val)
{
    int i = 0;
    
    while (kinds[i].desc) {
        if ((val & kinds[i].mask) == kinds[i].val)
            return kinds[i].desc;
        i++;
    }
    return "Unknown";
}

int main (int argc, char **argv )
{
    int fd;
    char name[20];
    char board, device;
    unsigned int *ptr;
    unsigned int val;

    for (board = 'a'; board < 'z'; board++) {
        int done = 0;
        for (device = '3'; device <= '4'; device++) {
            sprintf(name, "/dev/er%c%c", board, device);
            fd = open(name, O_RDWR);
            if (fd < 0)
                continue;
            if (device == '3') {
                ptr = (unsigned int *) mmap(0, 0x40, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
                if ( ptr == MAP_FAILED ) {
                    fprintf(stderr, "Failed to map %s!\n", name);
                    perror("");
                    continue;
                }
                val = ptr[VERSIONOFFSET];
                val = be32_to_cpu(val);
                munmap(ptr, 0x40);
            } else {
                val = __read_evr_register(fd, VERSIONREG);
            }
            printf("%s: 0x%08x - %s\n", name, val, getkind(val));
            done = 1;
            close(fd);
            break;
        }
        if (!done)
            break;
    }
    return 0;
}
