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
DB += evrDevTrig.db
DB += evrTrigger.db
DB += evrEventAll.db
# evrPattern support
DB += evrPattern.db
DB += evrPatternProc.db
DB += evrPatternAll.db
DB += evrDevInfo.db
# fiducialTimestamp timestamp support (includes patterns)
DB += fiducialTimestamp.db

# Event Codes
DB += evrEventCode.template
DB += evrEventCodeName.template

# PCDS Event Code List
DB += pcdsEventCodes.db

# Complete DB files for each of our common EVR boards
DB += evrPmc230.db
DB += evrVme230.db
DB += evrVme230RF.db
DB += evrSLAC.db

REQ += autosave_evrDevInfo.tpl-req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

