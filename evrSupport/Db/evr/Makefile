TOP=../../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
#
# EVR databases
DB += evrEvent.db
DB += evrEventCtrl.db
DB += evrTrigger.db
DB += evrEventAll.db
# evrPattern support
DB += evrPattern.db
DB += evrPatternProc.db
DB += evrPatternAll.db
# LCLS templates
DB += fiducial.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

