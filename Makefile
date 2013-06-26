#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += driver
DIRS += mrfApp
DIRS += evrSupport
DIRS += app

include $(TOP)/configure/RULES_TOP
