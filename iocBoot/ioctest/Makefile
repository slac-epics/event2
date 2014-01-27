# Makefile to install the st.cmd into the correctly location.
# Author: Jason Tiller

TOP = ../..
include $(TOP)/configure/CONFIG
ARCH = linux-x86
TARGETS = envPaths
COMMON_DIR = .

# Make sure these archive files are fully expanded
USR_ARCHIVEFLAGS += -V

# See RULES.copy for more information on these variables.
COPY_ONLY_IF_NON_LOCAL=1
COPY_SUBDIR=iocBoot/ioc-mec-imb02
COPY_FILES=st.cmd

DB += evrTest.db evrSLACTest.db

include $(TOP)/configure/RULES
include $(TOP)/configure/RULES.ioc-targets

#include $(TOP)/configure/RULES.ioc
#-include $(TOP)/configure/RULES.archive
#-include $(TOP)/configure/RULES.autosave
#-include $(TOP)/configure/RULES.copy
