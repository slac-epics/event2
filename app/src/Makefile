TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build an ioc application
# Use PROD_IOC_RTEMS for RTEMS only
# Use PROD_IOC_Linux for Linux only
PROD_IOC_Linux = event2Ioc event2Test event2Diag event2Version

# ipimbIoc.dbd will be created and installed
DBD += event2Ioc.dbd

# event2Ioc.dbd will be made up from these files:
event2Ioc_DBD += base.dbd
event2Ioc_DBD += event2Support.dbd
event2Ioc_DBD += devEvent2Er.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
event2Ioc_SRCS += event2Ioc_registerRecordDeviceDriver.cpp
event2Ioc_SRCS_DEFAULT += event2Main.cpp

# Add locally compiled object code
#event2Ioc_SRCS += dbSubExample.c

event2Ioc_LIBS += event2Support
event2Ioc_LIBS += devEvent2Er
event2Ioc_LIBS += drvEvent2
ifdef DIAG_TIMER
event2Ioc_LIBS += diagTimer
endif
event2Ioc_LIBS += $(EPICS_BASE_IOC_LIBS)

event2Test_SRCS_DEFAULT += event2Test.c
event2Test_LIBS += drvEvent2
ifdef DIAG_TIMER
event2Test_LIBS += diagTimer
endif
event2Test_LIBS += $(EPICS_BASE_IOC_LIBS)

event2Diag_SRCS_DEFAULT += event2Diag.c
event2Diag_LIBS += drvEvent2
ifdef DIAG_TIMER
event2Diag_LIBS += diagTimer
endif
event2Diag_LIBS += $(EPICS_BASE_IOC_LIBS)

event2Version_SRCS_DEFAULT += event2Version.c
event2Version_LIBS += drvEvent2
ifdef DIAG_TIMER
event2Version_LIBS += diagTimer
endif
event2Version_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

