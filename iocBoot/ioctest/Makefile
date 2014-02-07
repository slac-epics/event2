# Makefile to install the st.cmd into the correctly location.
# Author: Jason Tiller

TOP = ../..
IOC = ioctest
include $(TOP)/configure/CONFIG
ARCH = linux-x86
TARGETS = envPaths

# Make sure these archive files are fully expanded
USR_ARCHIVEFLAGS += -V

include $(TOP)/configure/RULES
include $(TOP)/configure/RULES.ioc-targets
