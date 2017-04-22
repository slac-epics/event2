TOP=../../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
USR_VPATH += ../../bsa
USR_VPATH += ../../templates
USR_VPATH += $(COMMON_DIR)

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
#
# EVR databases
DB += evr.db
DB += evr-pmc.db
DB += evrWithDelays.db
DB += evrWithExtDelays.db
DB += evrPmc.template
DB += evrEvent.db
DB += evrEventCtrl.db
DB += evrDevTrig.db
DB += evrTrigger.db
# evrPattern support
DB += evrPattern.db
DB += evrPatternProc.db
DB += evrEventAll.db
DB += evrEventFACET.db
DB += evrEventXTA.db
DB += evrPatternAll.db
DB += evrDevInfo.db
DB += evrMod36.db
# fiducialTimestamp timestamp support (includes patterns)
DB += fiducialTimestamp.db

# Event Codes
DB += evrEventCode.template
DB += evrEventCodeName.template

# Event Code Delays
DB += eventCodeDelays.db

# LCLS Event Code List
DB += lclsEventCodes.substitutions

# PCDS Event Code List
# includes LCLS event codes plus event sequencer codes
DB += pcdsEventCodes.db
DB += pcdsEventCodes.substitutions

# Complete DB files for each of our common EVR boards
DB += evrPmc230.db
DB += evrSLAC.db

# A record to disable/enable a particular trigger.
DB += evrTrigCtrl.db

# Sample substitution files for each
# of our common EVR boards
DB += evrPmc230.substitutions
DB += evrSLAC.substitutions

# Autosave for EVG event code delays
REQ += eventCodeDelays.req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

