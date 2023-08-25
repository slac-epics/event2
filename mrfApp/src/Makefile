#------------------------------------------------------------------------------
# $(TIMING)/mrfApp/src/Makefile
#
# Author:  Eric Bjorklund
# Date:    19 December 2005
#
#------------------------------------------------------------------------------
# This file contains the EPICS make rules and definitions for making and
# installing the driver support modules for the Micro-Research Finland
# Series 200 event receiver module.
#
#------------------------------------------------------------------------------
# Modification History:
#
# 19 Dec 2005   Bjo     Original Release
# 21 Aug 2006   Saa     Changes to build on RTEMS too.
#
#------------------------------------------------------------------------------

#---------------------
# Set up the Environment
#
TOP = ../..
SOURCE_DIR := $(patsubst %../.., %., $(TOP))

include $(TOP)/configure/CONFIG
include $(SOURCE_DIR)/MRF_CONFIG

# Don't use -O0 for RTEMS as it generates undef references to in_be16()
#USR_CFLAGS_Linux += -O0
USR_CFLAGS += -DDEBUG_PRINT
USR_CFLAGS += -fno-keep-inline-functions
ifeq ($(BSP_SUPPORTS_PCI), YES)
    USR_CFLAGS += -DPCI
endif

#---------------------
# Only build for "hardware" IOC's
#
#ifneq ($(findstring Host, $(VALID_BUILDS)),)
#    VALID_BUILDS = Host
#endif

#---------------------
# Build the record support modules
#
CUSTOM_RECORDS += erRecord
CUSTOM_RECORDS += ereventRecord

DBDINC += $(CUSTOM_RECORDS)

#---------------------
# Install DBD files
#
DBD += drvEvent2Er.dbd
DBD += devEvent2Er.dbd
#TODO: Delete these
##DBD += devEvent2ErLib.dbd
#devEvent2ErLib_DBD += base.dbd
#devEvent2ErLib_DBD += devEvent2Er.dbd

#---------------------
# Build the Event Receiver Library
#
ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
LIBRARY_IOC_Linux   += devEvent2Er
endif

# Source objects for Event Receiver Library
devEvent2Er_OBJS       += devMrfEr
devEvent2Er_OBJS       += erRecord
devEvent2Er_OBJS       += ereventRecord

# Linux source
ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
devEvent2Er_OBJS_Linux += drvLinuxEvr
endif

devEvent2ErLib_OBJS += $(devEvent2Er_OBJS)
devEvent2ErLib_SRCS += devEvent2ErLib_registerRecordDeviceDriver.cpp

#---------------------
# Build the Library for the MRF driver level access
#
ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
LIBRARY_IOC_Linux	+= drvEvent2
endif

# Source objects for MRF driver Library
ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
drvEvent2_OBJS_Linux += drvLinux
drvEvent2_OBJS_Linux += evrIrqHandlerThread
drvEvent2_OBJS_Linux += erapi.o
endif

drvEvent2Lib_OBJS	+= $(drvEvent2_OBJS)

OBJS_IOC_vxWorks 	+= $(drvEvent2_OBJS)

#---------------------
# Install include files
#
INC += mrfCommon.h
INC += fidmath.h
# INC += mrfVme64x.h

#INC += drvMrfEr.h
#INC += devMrfEr.h

INC += erapi.h
INC += evrIrqHandler.h

INC += debugPrint.h

#=======================================================================#
#                               RULES                                   #
#=======================================================================#

#---------------------
# Generic EPICS build rules
#
include $(TOP)/configure/RULES
