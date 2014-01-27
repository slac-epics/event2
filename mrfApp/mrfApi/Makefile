#------------------------------------------------------------------------------
# $(TIMING)/mrfApp/mrfApi/Makefile
#
# Author:  Bruce Hill
# Date:    Mar 15, 2010
#------------------------------------------------------------------------------
# EPICS make rules for Micro-Research Finland API library and diagnostic tools
#------------------------------------------------------------------------------

#---------------------
# Set up the Environment
#
TOP = ../..
SOURCE_DIR := $(patsubst %../.., %., $(TOP))

include $(TOP)/configure/CONFIG

USR_CFLAGS += -g
# Don't use -O0 for RTEMS as it generates undef references to in_be16()
#USR_CFLAGS_Linux += -O0
USR_CFLAGS += -fPIC
USR_CFLAGS += -std=gnu99


#---------------------
# Build the MRF APplication Interface library
#
LIBRARY_IOC_Linux       += mrfApi

# API lib source
mrfApi_SRCS                     +=      empty.c

#=======================================================================#
#                               RULES                                   #
#=======================================================================#

#---------------------
# Generic EPICS build rules
#
include $(TOP)/configure/RULES
