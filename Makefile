#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += mrfApp
DIRS += evrSupport
DIRS += app
#DIRS += iocBoot

include $(TOP)/configure/RULES_TOP
