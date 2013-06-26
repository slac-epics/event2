/*
  erapi.h -- Definitions for Micro-Research Event Receiver
             Application Programming Interface

  Author: Jukka Pietarinen (MRF)
  Date:   08.12.2006

*/

/*
  Note: Byte ordering is big-endian.
 */

#define EVR_MEM_WINDOW      0x00008000

#ifndef u16
#define u16 unsigned short
#endif
#ifndef u32
#define u32 unsigned int
#endif

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define be16_to_cpu(x) bswap_16(x)
#define be32_to_cpu(x) bswap_32(x)
#else
#define be16_to_cpu(x) ((u16)(x))
#define be32_to_cpu(x) ((u32)(x))
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

struct MrfErRegs {
  u32  Status;                              /* 0000: Status Register */
  u32  Control;                             /* 0004: Main Control Register */
  u32  IrqFlag;                             /* 0008: Interrupt Flags */
  u32  IrqEnable;                           /* 000C: Interrupt Enable */
  u32  PulseIrqMap;                         /* 0010:  */
  u32  Resv0x0014;                          /* 0014: Reserved */
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

/* ioctl commands */
#define EV_IOC_MAGIC 220
#define EV_IOCRESET  _IO(EV_IOC_MAGIC, 0)
#define EV_IOCIRQEN  _IO(EV_IOC_MAGIC, 1)
#define EV_IOCIRQDIS _IO(EV_IOC_MAGIC, 2)

/* Function prototypes */
int EvrOpen(struct MrfErRegs **pEr, char *device_name);
int EvrClose(int fd);
int EvrEnable(volatile struct MrfErRegs *pEr, int state);
int EvrGetEnable(volatile struct MrfErRegs *pEr);
void EvrDumpStatus(volatile struct MrfErRegs *pEr);
int EvrGetViolation(volatile struct MrfErRegs *pEr, int clear);
int EvrDumpMapRam(volatile struct MrfErRegs *pEr, int ram);
int EvrMapRamEnable(volatile struct MrfErRegs *pEr, int ram, int enable);
int EvrSetForwardEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrEnableEventForwarding(volatile struct MrfErRegs *pEr, int enable);
int EvrGetEventForwarding(volatile struct MrfErRegs *pEr);
int EvrSetLedEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetFIFOEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetLatchEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrSetFIFOStopEvent(volatile struct MrfErRegs *pEr, int ram, int code, int enable);
int EvrClearFIFO(volatile struct MrfErRegs *pEr);
int EvrGetFIFOEvent(volatile struct MrfErRegs *pEr, struct FIFOEvent *fe);
int EvrEnableFIFOStopEvent(volatile struct MrfErRegs *pEr, int enable);
int EvrGetFIFOStopEvent(volatile struct MrfErRegs *pEr);
int EvrEnableFIFO(volatile struct MrfErRegs *pEr, int enable);
int EvrGetFIFOState(volatile struct MrfErRegs *pEr);
int EvrDumpFIFO(volatile struct MrfErRegs *pEr);
int EvrSetPulseMap(volatile struct MrfErRegs *pEr, int ram, int code, int trig,
		   int set, int clear);
int EvrClearPulseMap(volatile struct MrfErRegs *pEr, int ram, int code, int trig,
		   int set, int clear);
int EvrSetPulseParams(volatile struct MrfErRegs *pEr, int pulse, u32 presc,
		      u32 delay, u32 width);
void EvrDumpPulses(volatile struct MrfErRegs *pEr, int pulses);
int EvrSetPulseProperties(volatile struct MrfErRegs *pEr, int pulse, int polarity,
			  int map_reset_ena, int map_set_ena, int map_trigger_ena,
			  int enable);
int EvrSetUnivOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpUnivOutMap(volatile struct MrfErRegs *pEr, int outputs);
int EvrSetFPOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpFPOutMap(volatile struct MrfErRegs *pEr, int outputs);
int EvrSetTBOutMap(volatile struct MrfErRegs *pEr, int output, int map);
void EvrDumpTBOutMap(volatile struct MrfErRegs *pEr, int outputs);
void EvrIrqAssignHandler(volatile struct MrfErRegs *pEr, int fd, void (*handler)(int));
int EvrIrqEnable(volatile struct MrfErRegs *pEr, int mask);
int EvrGetIrqFlags(volatile struct MrfErRegs *pEr);
int EvrClearIrqFlags(volatile struct MrfErRegs *pEr, int mask);
void EvrIrqHandled(int fd);
int EvrSetPulseIrqMap(volatile struct MrfErRegs *pEr, int map);
void EvrClearDiagCounters(volatile struct MrfErRegs *pEr);
void EvrEnableDiagCounters(volatile struct MrfErRegs *pEr, int enable);
u32 EvrGetDiagCounter(volatile struct MrfErRegs *pEr, int idx);
int EvrUnivDlyEnable(volatile struct MrfErRegs *pEr, int dlymod, int enable);
int EvrUnivDlySetDelay(volatile struct MrfErRegs *pEr, int dlymod, u32 dly0, u32 dly1);
void EvrDumpHex(volatile struct MrfErRegs *pEr);
int EvrSetFracDiv(volatile struct MrfErRegs *pEr, int fracdiv);
int EvrGetFracDiv(volatile struct MrfErRegs *pEr);
int EvrSetDBufMode(volatile struct MrfErRegs *pEr, int enable);
int EvrGetDBufStatus(volatile struct MrfErRegs *pEr);
int EvrReceiveDBuf(volatile struct MrfErRegs *pEr, int enable);
int EvrGetDBuf(volatile struct MrfErRegs *pEr, char *dbuf, int size);
int EvrSetTimestampDivider(volatile struct MrfErRegs *pEr, int div);
int EvrGetTimestampCounter(volatile struct MrfErRegs *pEr);
int EvrGetSecondsCounter(volatile struct MrfErRegs *pEr);
int EvrGetTimestampLatch(volatile struct MrfErRegs *pEr);
int EvrGetSecondsLatch(volatile struct MrfErRegs *pEr);
int EvrSetTimestampDBus(volatile struct MrfErRegs *pEr, int enable);
int EvrSetPrescaler(volatile struct MrfErRegs *pEr, int presc, u32 div);
int EvrSetExtEvent(volatile struct MrfErRegs *pEr, int ttlin, int code, int enable);
int EvrSetBackEvent(volatile struct MrfErRegs *pEr, int ttlin, int code, int enable);
int EvrSetBackDBus(volatile struct MrfErRegs *pEr, int ttlin, int dbus);
int EvrSetTxDBufMode(volatile struct MrfErRegs *pEr, int enable);
int EvrGetTxDBufStatus(volatile struct MrfErRegs *pEr);
int EvrSendTxDBuf(volatile struct MrfErRegs *pEr, char *dbuf, int size);
int EvrGetFormFactor(volatile struct MrfErRegs *pEr);
