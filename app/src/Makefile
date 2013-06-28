TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build an ioc application
# Use PROD_IOC_RTEMS for RTEMS only
# Use PROD_IOC_Linux for Linux only
PROD_IOC_Linux = evrIoc evrTest

# ipimbIoc.dbd will be created and installed
DBD += evrIoc.dbd

# evrIoc.dbd will be made up from these files:
evrIoc_DBD += base.dbd
evrIoc_DBD += genSubRecord.dbd
evrIoc_DBD += evrSupport.dbd
evrIoc_DBD += devMrfEr.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
evrIoc_SRCS += evrIoc_registerRecordDeviceDriver.cpp
evrIoc_SRCS_DEFAULT += evrMain.cpp

# Add locally compiled object code
#evrIoc_SRCS += dbSubExample.c

evrIoc_LIBS += genSub
evrIoc_LIBS += generalTime
evrIoc_LIBS += evrSupport
evrIoc_LIBS += devMrfEr
evrIoc_LIBS += drvMrf
evrIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

evrTest_SRCS_DEFAULT += evrTest.c
evrTest_LIBS += drvMrf
evrTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

