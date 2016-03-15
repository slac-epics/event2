#ifndef EVRMEMMAP_H
#define EVRMEMMAP_H
/*
  erapi.h -- Definitions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006

*/

/*
  Note: Byte ordering is big-endian.
 */

#define EVR_MEM_WINDOW      ((sizeof(struct MrfErRegs) + PAGE_SIZE) & PAGE_MASK)
#define EVR_SH_MEM_WINDOW   ((sizeof(struct EvrQueues) + PAGE_SIZE) & PAGE_MASK)

#ifndef u16
#define u16 unsigned short
#endif
#ifndef u32
#define u32 unsigned int
#endif

#ifndef be16_to_cpu
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define be16_to_cpu(x) bswap_16(x)
#define be32_to_cpu(x) bswap_32(x)
#else
#define be16_to_cpu(x) ((u16)(x))
#define be32_to_cpu(x) ((u32)(x))
#endif
#endif

#define EVR_MAX_FPOUT_MAP   32
#define EVR_MAX_UNIVOUT_MAP 32
#define EVR_MAX_TBOUT_MAP   64
#define EVR_MAX_FPIN_MAP    16
#define EVR_MAX_UNIVIN_MAP  16
#define EVR_MAX_TBIN_MAP    64
#define EVR_MAX_BUFFER      2048
#define EVR_MAPRAMS         2
#define EVR_MAX_PRESCALERS  64
#define EVR_MAX_PULSES      32
#define EVR_MAX_CML_OUTPUTS 8
#define EVR_MAX_EVENT_CODE  255
#define EVR_DIAG_MAX_COUNTERS 32

struct PulseStruct {
  u32  Control;
  u32  Prescaler;
  u32  Delay;
  u32  Width;
};

struct EvrIoctlPulse {
  u32                Id;
  struct PulseStruct Pulse;
};

typedef enum { EvrTrigFree=0, EvrTrigAlloc=1, EvrTrigSteal=2, EvrTrigCheck=3 } EvrIoctlTrigOp;

struct EvrIoctlTrig {
    u32       Id;
    u32       Op;
};

struct CMLStruct {
  /* Bit patterns contain pattern bits in the 20 lowest bit locations */
  u32  Pattern00; /* bit pattern for low state */
  u32  Pattern01; /* bit pattern for rising edge */
  u32  Pattern10; /* bit pattern for falling edge */
  u32  Pattern11; /* bit pattern for high state */
  u32  Control;   /* CML Control Register */
  u32  Resv[3];
};

struct MapRamItemStruct {
  u32  IntEvent;
  u32  PulseTrigger;
  u32  PulseSet;
  u32  PulseClear;
};

struct FIFOEvent {
  u32 TimestampHigh;
  u32 TimestampLow;
  u32 EventCode;
};

struct DBufInfo {
  u32  data[512];
  u32  status;
};

/* These must be powers of two!!! */
#define MAX_EVR_EVTQ 1024
#define MAX_EVR_DBQ     4
struct EvrQueues {
  struct FIFOEvent evtq[MAX_EVR_EVTQ];     /* 12K */
  struct DBufInfo  dbq[MAX_EVR_DBQ];       /*  8K + 16 */
  long long ewp;                           /*  20 */
  long long dwp;
  int  fifofull;
};

struct MrfErRegs {
  u32  Status;                              /* 0000: Status Register */
  u32  Control;                             /* 0004: Main Control Register */
  u32  IrqFlag;                             /* 0008: Interrupt Flags */
  u32  IrqEnable;                           /* 000C: Interrupt Enable */
  u32  PulseIrqMap;                         /* 0010:  */
  u32  pcieInt;                             /* 0014: SLAC EVR - Interrupt Control */
  u32  Resv0x0018;                          /* 0018: Reserved */
  u32  Resv0x001C;                          /* 001C: Reserved */
  u32  DataBufControl;                      /* 0020: Data Buffer Control */
  u32  TxDataBufControl;                    /* 0024: TX Data Buffer Control */
  u32  Resv0x0028;                          /* 0028: Reserved */
  u32  FPGAVersion;                         /* 002C: FPGA version */
  u32  Resv0x0030[(0x040-0x030)/4];         /* 0030-003F: Reserved */
  u32  EvCntPresc;                          /* 0040: Event Counter Prescaler */
  u32  EvCntControl;                        /* 0044: Event Counter Control */
  u32  Resv0x0048;                          /* 0048: Reserved */
  u32  UsecDiv;                             /* 004C: round(Event clock/1 MHz) */
  u32  ClockControl;                        /* 0050: Clock Control */
  u32  Resv0x0054[(0x05C-0x054)/4];         /* 0054-005B: Reserved */
  u32  SecondsShift;                        /* 005C: Seconds Counter Shift Register */
  u32  SecondsCounter;                      /* 0060: Seconds Counter */
  u32  TimestampEventCounter;               /* 0064: Timestamp Event counter */
  u32  SecondsLatch;                        /* 0068: Seconds Latch Register */
  u32  TimestampLatch;                      /* 006C: Timestamp Latch Register */
  u32  FIFOSeconds;                         /* 0070: Event FIFO Seconds Register */
  u32  FIFOTimestamp;                       /* 0074: Event FIFO Timestamp Register */
  u32  FIFOEvent;                           /* 0078: Event FIFO Event Code Register */
  u32  Resv0x007C;                          /* 007C: Reserved */
  u32  FracDiv;                             /* 0080: Fractional Synthesizer SY87739L Control Word */
  u32  Resv0x0084;                          /* 0084: Reserved */
  u32  RxInitPS;                            /* 0088: Initial value for RF Recovery DCM phase */
  u32  Resv0x008C;
  u32  GPIODir;                             /* 0090: GPIO signal direction */
  u32  GPIOIn;                              /* 0094: GPIO input register */
  u32  GPIOOut;                             /* 0098: GPIO output register */
  u32  Resv0x009Cto0x00FC[(0x100-0x09C)/4]; /* 009C-00FF: Reserved */
  u32  Prescaler[EVR_MAX_PRESCALERS];       /* 0100-01FF: Prescaler Registers */
  struct PulseStruct Pulse[EVR_MAX_PULSES]; /* 0200-03FF: Pulse Output Registers */
  u16  FPOutMap[EVR_MAX_FPOUT_MAP];         /* 0400-043F: Front panel output mapping */
  u16  UnivOutMap[EVR_MAX_UNIVOUT_MAP];     /* 0440-047F: Universal I/O output mapping */
  u16  TBOutMap[EVR_MAX_TBOUT_MAP];         /* 0480-04FF: TB output mapping */
  u32  FPInMap[EVR_MAX_FPIN_MAP];           /* 0500-053F: Front panel input mapping */
  u32  UnivInMap[EVR_MAX_UNIVIN_MAP];       /* 0540-057F: Universal I/O input mapping */
  u32  Resv0x0580[(0x600-0x580)/4];         /* 0580-05FF: Reserved */
  struct CMLStruct CML[EVR_MAX_CML_OUTPUTS];/* 0600-06FF: CML Output Structures */
  u32  Resv0x0700[(0x800-0x700)/4];         /* 0700-07FF: Reserved */
  u32  Databuf[EVR_MAX_BUFFER/4];           /* 0800-0FFF: Data Buffer */
  u32  DiagIn;                              /* 1000:      Diagnostics input bits */
  u32  DiagCE;                              /* 1004:      Diagnostics count enable */
  u32  DiagReset;                           /* 1008:      Diagnostics count reset */
  u32  Resv0x100C[(0x1080-0x100C)/4];       /* 100C-1080: Reserved */
  u32  DiagCounter[EVR_DIAG_MAX_COUNTERS];  /* 1080-10FF: Diagnostics counters */
  u32  Resv0x1100[(0x1800-0x1100)/4];       /* 1100-17FF: Reserved */
  u32  TxDatabuf[EVR_MAX_BUFFER/4];         /* 1800-1FFF: TX Data Buffer */
  u32  Resv0x2000[(0x4000-0x2000)/4];       /* 2000-3FFF: Reserved */
  struct MapRamItemStruct MapRam[EVR_MAPRAMS][EVR_MAX_EVENT_CODE+1];
                                            /* 4000-4FFF: Map RAM 1 */
                                            /* 5000-5FFF: Map RAM 2 */
  char Resv0x6000[0x2000];                  /* 6000-7FFF: Reserved padding to 8K boundary */
};

/* -- Control Register bit mappings */
#define C_EVR_CTRL_MASTER_ENABLE    31
#define C_EVR_CTRL_EVENT_FWD_ENA    30
#define C_EVR_CTRL_TXLOOPBACK       29
#define C_EVR_CTRL_RXLOOPBACK       28
#define C_EVR_CTRL_TS_CLOCK_DBUS    14
#define C_EVR_CTRL_RESET_TIMESTAMP  13
#define C_EVR_CTRL_LATCH_TIMESTAMP  10
#define C_EVR_CTRL_MAP_RAM_ENABLE   9
#define C_EVR_CTRL_MAP_RAM_SELECT   8
#define C_EVR_CTRL_FIFO_ENABLE      6
#define C_EVR_CTRL_FIFO_DISABLE     5
#define C_EVR_CTRL_FIFO_STOP_EV_EN  4
#define C_EVR_CTRL_RESET_EVENTFIFO  3
/* -- Status Register bit mappings */
#define C_EVR_STATUS_DBUS_HIGH      31
#define C_EVR_STATUS_LEGACY_VIO     16
#define C_EVR_STATUS_FIFO_STOPPED   5
/* -- Interrupt Flag/Enable Register bit mappings */
#define C_EVR_IRQ_MASTER_ENABLE   31
#define C_EVR_NUM_IRQ             6
#define C_EVR_IRQFLAG_DATABUF     5
#define C_EVR_IRQFLAG_PULSE       4
#define C_EVR_IRQFLAG_EVENT       3
#define C_EVR_IRQFLAG_HEARTBEAT   2
#define C_EVR_IRQFLAG_FIFOFULL    1
#define C_EVR_IRQFLAG_VIOLATION   0
#define EVR_IRQ_MASTER_ENABLE     (1 << C_EVR_IRQ_MASTER_ENABLE)
#define EVR_IRQFLAG_DATABUF       (1 << C_EVR_IRQFLAG_DATABUF)
#define EVR_IRQFLAG_PULSE         (1 << C_EVR_IRQFLAG_PULSE)
#define EVR_IRQFLAG_EVENT         (1 << C_EVR_IRQFLAG_EVENT)
#define EVR_IRQFLAG_HEARTBEAT     (1 << C_EVR_IRQFLAG_HEARTBEAT)
#define EVR_IRQFLAG_FIFOFULL      (1 << C_EVR_IRQFLAG_FIFOFULL)
#define EVR_IRQFLAG_VIOLATION     (1 << C_EVR_IRQFLAG_VIOLATION)
/* -- Databuffer Control Register bit mappings */
#define C_EVR_DATABUF_LOAD        15
#define C_EVR_DATABUF_RECEIVING   15
#define C_EVR_DATABUF_STOP        14
#define C_EVR_DATABUF_RXREADY     14
#define C_EVR_DATABUF_CHECKSUM    13
#define C_EVR_DATABUF_MODE        12
#define C_EVR_DATABUF_SIZEHIGH    11
#define C_EVR_DATABUF_SIZELOW     2
/* -- Databuffer Control Register bit mappings */
#define C_EVR_TXDATABUF_COMPLETE   20
#define C_EVR_TXDATABUF_RUNNING    19
#define C_EVR_TXDATABUF_TRIGGER    18
#define C_EVR_TXDATABUF_ENA        17
#define C_EVR_TXDATABUF_MODE       16
#define C_EVR_TXDATABUF_SIZEHIGH   11
#define C_EVR_TXDATABUF_SIZELOW    2
/* -- Clock Control Register bit mapppings */
#define C_EVR_CLKCTRL_RECDCM_RUN     15
#define C_EVR_CLKCTRL_RECDCM_INITD   14
#define C_EVR_CLKCTRL_RECDCM_PSDONE  13
#define C_EVR_CLKCTRL_EVDCM_STOPPED  12
#define C_EVR_CLKCTRL_EVDCM_LOCKED  11
#define C_EVR_CLKCTRL_EVDCM_PSDONE  10
#define C_EVR_CLKCTRL_CGLOCK        9
#define C_EVR_CLKCTRL_RECDCM_PSDEC  8
#define C_EVR_CLKCTRL_RECDCM_PSINC  7
#define C_EVR_CLKCTRL_RECDCM_RESET  6
#define C_EVR_CLKCTRL_EVDCM_PSDEC   5
#define C_EVR_CLKCTRL_EVDCM_PSINC   4
#define C_EVR_CLKCTRL_EVDCM_SRUN    3
#define C_EVR_CLKCTRL_EVDCM_SRES    2
#define C_EVR_CLKCTRL_EVDCM_RES     1
#define C_EVR_CLKCTRL_USE_RXRECCLK  0
/* -- CML Control Register bit mappings */
#define C_EVR_CMLCTRL_REFCLKSEL     3
#define C_EVR_CMLCTRL_RESET         2
#define C_EVR_CMLCTRL_POWERDOWN     1
#define C_EVR_CMLCTRL_ENABLE        0
/* -- Pulse Control Register bit mappings */
#define C_EVR_PULSE_OUT             7
#define C_EVR_PULSE_SW_SET          6
#define C_EVR_PULSE_SW_RESET        5
#define C_EVR_PULSE_POLARITY        4
#define C_EVR_PULSE_MAP_RESET_ENA   3
#define C_EVR_PULSE_MAP_SET_ENA     2
#define C_EVR_PULSE_MAP_TRIG_ENA    1
#define C_EVR_PULSE_ENA             0
/* -- Map RAM Internal event mappings */
#define C_EVR_MAP_SAVE_EVENT        31
#define C_EVR_MAP_LATCH_TIMESTAMP   30
#define C_EVR_MAP_LED_EVENT         29
#define C_EVR_MAP_FORWARD_EVENT     28
#define C_EVR_MAP_STOP_FIFO         27
#define C_EVR_MAP_HEARTBEAT_EVENT   5
#define C_EVR_MAP_RESETPRESC_EVENT  4
#define C_EVR_MAP_TIMESTAMP_RESET   3
#define C_EVR_MAP_TIMESTAMP_CLK     2
#define C_EVR_MAP_SECONDS_1         1
#define C_EVR_MAP_SECONDS_0         0
/* -- Output Mappings */
#define C_EVR_SIGNAL_MAP_BITS       6
#define C_EVR_SIGNAL_MAP_PULSE      0
#define C_EVR_SIGNAL_MAP_DBUS       32
#define C_EVR_SIGNAL_MAP_PRESC      40
#define C_EVR_SIGNAL_MAP_HIGH       62
#define C_EVR_SIGNAL_MAP_LOW        63
/* GPIO mapping for delay module */
#define EVR_UNIV_DLY_DIN    0x01
#define EVR_UNIV_DLY_SCLK   0x02
#define EVR_UNIV_DLY_LCLK   0x04
#define EVR_UNIV_DLY_DIS    0x08
/* -- FP Input Mapping bits */
#define C_EVR_FPIN_EXTEVENT_BASE   0
#define C_EVR_FPIN_BACKEVENT_BASE  8
#define C_EVR_FPIN_BACKDBUS_BASE   16
#define C_EVR_FPIN_EXT_ENABLE      24
#define C_EVR_FPIN_BACKEV_ENABLE   25

#define EV_IOC_MAGIC 220
#define EVR_MAX_READ 0x1000
#define EV_IOCRESET   _IO(EV_IOC_MAGIC, 0)
#define EV_IOCIRQEN   _IO(EV_IOC_MAGIC, 1)
#define EV_IOCIRQDIS  _IO(EV_IOC_MAGIC, 2)
#define EV_IOCIRQMASK _IO(EV_IOC_MAGIC, 3)
#define EV_IOCEVTTAB  _IO(EV_IOC_MAGIC, 4)
#define EV_IOCPULSE   _IO(EV_IOC_MAGIC, 5)
#define EV_IOCREAD32  _IO(EV_IOC_MAGIC, 6)
#define EV_IOCREAD16  _IO(EV_IOC_MAGIC, 7)
#define EV_IOCTRIG    _IO(EV_IOC_MAGIC, 8)

#define EV_IOC_MAX   8

/* The bits 27-24 of the FPGAVersion register tell us the form factor. */
#define EVR_FORM_CPCI  0
#define EVR_FORM_PMC   1
#define EVR_FORM_VME   2
#define EVR_FORM_SLAC 15

/*
 * These are user space macros.  They are controlled by INLINE_READ_EVR 
 * (which should be "inline", blank, or undefined), and DEFINE_READ_EVR 
 * (which should be defined or undefined).
 */

#ifndef INLINE_READ_EVR
#define INLINE_READ_EVR
#endif
#ifdef DEFINE_READ_EVR
INLINE_READ_EVR int __read_evr_region32(int fd, int offset, void *buf, int size)
{
    ((u32 *)buf)[0] = offset;
    ((u32 *)buf)[1] = size;
    if (ioctl(fd, EV_IOCREAD32, buf) < 0) {
        char buf[128];
        sprintf(buf, "READ EVR at offset %d failed", offset);
        perror(buf);
        return -1;
    }
    return 0;
}

INLINE_READ_EVR int __read_evr_region16(int fd, int offset, void *buf, int size)
{
    ((u16 *)buf)[0] = offset;
    ((u16 *)buf)[1] = size;
    if (ioctl(fd, EV_IOCREAD16, buf) < 0) {
        char buf[128];
        sprintf(buf, "READ EVR at offset %d failed", offset);
        perror(buf);
        return -1;
    }
    return 0;
}

INLINE_READ_EVR u32 __read_evr_register(int fd, int offset)
{
    u32 args[2];
    if (__read_evr_region32(fd, offset, args, sizeof(u32))) {
        return 0; /* Exit? */
    } else {
        return args[0];
    }
}

INLINE_READ_EVR u16 __read_evr_register16(int fd, int offset)
{
    u16 args[2];
    if (__read_evr_region16(fd, offset, args, sizeof(u16))) {
        return 0; /* Exit? */
    } else {
        return args[0];
    }
}
#else
INLINE_READ_EVR int __read_evr_region32(int fd, int offset, void *buf, int size);
INLINE_READ_EVR int __read_evr_region16(int fd, int offset, void *buf, int size);
INLINE_READ_EVR u32 __read_evr_register(int fd, int offset);
INLINE_READ_EVR u16 __read_evr_register16(int fd, int offset);
#endif

#define READ_EVR_REGISTER(fd, rname)             \
    __read_evr_register(fd, offsetof(struct MrfErRegs, rname))
#define READ_EVR_REGISTER16(fd, rname)             \
    __read_evr_register16(fd, offsetof(struct MrfErRegs, rname))
#define READ_EVR_REGION32(fd, rname, buf, size)  \
    __read_evr_region32(fd, offsetof(struct MrfErRegs, rname), buf, size)
#define READ_EVR_REGION16(fd, rname, buf, size)  \
    __read_evr_region16(fd, offsetof(struct MrfErRegs, rname), buf, size)
#endif
