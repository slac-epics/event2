TOP=../..

include $(TOP)/configure/CONFIG

#=============================
# build a library

# for gdb/ddd debugger:
USR_CFLAGS_solaris-sparc-gnu += -g
USR_CFLAGS_linux-x86 += -g
# for debug print statements and slc-awareness
USR_CFLAGS += -DDEBUG_PRINT
# Use EVR driver routines
USR_CFLAGS_RTEMS += -DEVR_DRIVER_SUPPORT
ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
USR_CFLAGS_Linux += -DEVR_DRIVER_SUPPORT
endif
ifdef DIAG_TIMER
USR_CFLAGS += -DDIAG_TIMER
endif


SRC_DIRS += $(TOP)/mrfApp/src

INC+=evrTime.h
INC+=evrPattern.h
INC+=evrMessage.h
INC+=bsa.h

DBDINC += longSubRecord
DBDINC += bsaRecord
DBDINC += bsacompressRecord

DBD += event2Support.dbd
DBD += evrEvInvariantDelay.dbd
DBD += evrEvDesc.dbd
DBD += evrPattern.dbd
DBD += evrModifier5.dbd
DBD += evrTime.dbd
DBD += bsa.dbd
DBD += drvEvr.dbd

LIBRARY_IOC = event2Support

event2Support_SRCS += evrEvInvariantDelay.c
event2Support_SRCS += evrEvDesc.c
event2Support_SRCS += evrPattern.c
event2Support_SRCS += evrModifier5.c
event2Support_SRCS += evrTime.c
event2Support_SRCS += evrMessage.c
event2Support_SRCS += bsa.c
event2Support_SRCS += longSubRecord.c
event2Support_SRCS += bsaRecord.c
event2Support_SRCS += bsacompressRecord.c
event2Support_SRCS += drvEvr.c
event2Support_LIBS += $(EPICS_BASE_IOC_LIBS)
ifdef DIAG_TIMER
event2Support_LIBS += diagTimer
endif

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

