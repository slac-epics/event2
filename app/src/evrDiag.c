#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#define  DEFINE_READ_EVR
#include "erapi.h"

struct regs {
    char *name;
    int   offset;
    int   size;
} reglist[] = {
    { "Status",                 0x0000, 4 },
    { "Control",                0x0004, 4 },
    { "IrqFlag",                0x0008, 4 },
    { "IrqEn",                  0x000C, 4 },
    { "PulseIrqMap",            0x0010, 4 },
    { "pcieInt",                0x0014, 4 },
    { "DataBufControl",         0x0020, 4 },
    { "TxDataBufControl",       0x0024, 4 },
    { "FPGAVersion",            0x002C, 4 },
    { "EvCntPresc",             0x0040, 4 },
    { "EvCntControl",           0x0044, 4 },
    { "UsecDiv",                0x004C, 4 },
    { "ClockControl",           0x0050, 4 },
    { "SecondsShift",           0x005C, 4 },
    { "SecondsCounter",         0x0060, 4 },
    { "TimestampEventCounter",  0x0064, 4 },
    { "SecondsLatch",           0x0068, 4 },
    { "TimestampLatch",         0x006C, 4 },
    { "FIFOSeconds",            0x0070, 4 },
    { "FIFOTimestamp",          0x0074, 4 },
    { "FIFOEvt",                0x0078, 4 },
    { "FracDiv",                0x0080, 4 },
    { "RxInitPS",               0x0088, 4 },
    { "GPIODir",                0x0090, 4 },
    { "GPIOIn",                 0x0094, 4 },
    { "GPIOOut",                0x0098, 4 },
    { "PS0",                    0x0100, 4 },
    { "PS1",                    0x0104, 4 },
    { "PS2",                    0x0108, 4 },
    { "Pulse",                  0x0200, 0x200 },
    { "FPOutMap",               0x0400, 0x40 },
    { "UnivOutMap",             0x0440, 0x40 },
    { "TBOutMap",               0x0480, 0x80 },
    { "UO0",                    0x0440, 2 },
    { "UO1",                    0x0442, 2 },
    { "UO2",                    0x0444, 2 },
    { "UO3",                    0x0446, 2 },
    { "UO4",                    0x0448, 2 },
    { "UO5",                    0x044a, 2 },
    { "UO6",                    0x044c, 2 },
    { "UO7",                    0x044e, 2 },
    { "UO8",                    0x0450, 2 },
    { "UO9",                    0x0452, 2 },
    { "DiagIn",                 0x1000, 4 },
    { "DiagCE",                 0x1004, 4 },
    { "DiagReset",              0x1008, 4 },
#if 0
    { "DataBuf",                0x0800, 0x800 },
#endif
    { "Ram0",                   0x4000, 0x1000 },
    { "Ram1",                   0x5000, 0x1000 },
    { "ActiveRam",              0x40004000, 0x1000 },
    { NULL,                     -1,     0 }
};

struct regs *findname(char *name)
{
    struct regs *r;
    for (r = reglist; r->name && strcmp(r->name, name); r++);
    return r;
}

int fd;
struct MapRamItemStruct mr[EVR_MAX_EVENT_CODE+1];
struct PulseStruct pp[EVR_MAX_PULSES];
unsigned short map[0x40];

int main (int argc, char **argv )
{
    struct regs *r;
    int off;
    unsigned int val;
    void *mem;
    char *name = "/dev/era4";
    int i;

    if (argc != 2) {
        printf("Usage: evrDiag REG\n");
        return(1);
    }

    fd = EvrOpen(&mem, name);

    if (!fd) {
        printf("Could not open %s!\n", name);
        return(1);
    }

    r = findname(argv[1]);
    off = r->offset;
    if (off < 0)
        exit(1);
    if (r->size == 2) {
        val = __read_evr_register16(fd, off);
        printf("%s (%d) -> 0x%04x (%d)\n", argv[2], off, val, val);
    } else if (r->size == 4) {
        val = __read_evr_register(fd, off);
        printf("%s (%d) -> 0x%08x (%d)\n", argv[2], off, val, val);
    } else if (r->size == 0x40 || r->size == 0x80) {
        /* Must be an output map! */
        if (__read_evr_region16(fd, off, map, r->size * sizeof(unsigned short))) {
            printf("Cannot read %s?!?\n", r->name);
            exit(0);
        }
        for (i = 0; i < r->size / sizeof(unsigned short); i++)
            printf("%2d: %04x (%d)\n", i, (int) map[i], (int) map[i]);
    } else if (r->size == 0x1000) {
        /* Must be a MapRam! */
        if (r->offset & 0x40000000) {
            val = READ_EVR_REGISTER(fd, Control) & (1 << C_EVR_CTRL_MAP_RAM_SELECT);
            if (val) {
                if (READ_EVR_REGION32(fd, MapRam[1], mr, sizeof(mr))) {
                    printf("Cannot read MapRam[1]?!?\n");
                    exit(0);
                }
            } else {
                if (READ_EVR_REGION32(fd, MapRam[0], mr, sizeof(mr))) {
                    printf("Cannot read MapRam[0]?!?\n");
                    exit(0);
                }
            }
        } else {
            if (__read_evr_region32(fd, off, mr, sizeof(mr))) {
                printf("Cannot read MapRam?!?\n");
                exit(0);
            }
        }
        printf("              Func     Trig     Set     Clear\n");
        printf("            -------- -------- -------- --------\n");
        for (i = 0; i <= EVR_MAX_EVENT_CODE; i++)
            if (mr[i].IntEvent || mr[i].PulseTrigger || mr[i].PulseSet || mr[i].PulseClear)
                printf("%3d (0x%02x): %08x %08x %08x %08x\n", i, i, mr[i].IntEvent, 
                       mr[i].PulseTrigger, mr[i].PulseSet, mr[i].PulseClear);
    } else if (r->size == 0x200) {
        /* Must be Pulse! */
        if (__read_evr_region32(fd, off, pp, sizeof(pp))) {
            printf("Cannot read Pulse info?!?\n");
            exit(0);
        }
        printf("Pulse     Polarity     Prescaler     Delay     Width\n");
        printf("-----     --------     ---------     -----     -----\n");
        for (i = 0; i < 16; i++) {
            if (pp[i].Control & (1 << C_EVR_PULSE_ENA)) {
                printf(" %3d         %s       %6d       %5d     %5d\n",
                       i, (pp[i].Control & (1 << C_EVR_PULSE_POLARITY)) ? "NEG" : "POS",
                       pp[i].Prescaler, pp[i].Delay, pp[i].Width);
            }
        }
    }

    EvrClose(fd);
    return 0;
}

