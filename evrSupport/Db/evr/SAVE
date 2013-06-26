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
DB += evr.db
DB += evr-pmc.db
DB += evrWithDelays.db
DB += evrWithExtDelays.db
DB += evr.template
DB += evrEvent.db
DB += evrEventCtrl.db
DB += evrDevTrig.db
DB += evrTrigger.db
# evrPattern support
DB += evrPattern.db
DB += evrPatternProc.db
DB += evrEventAll.db
DB += evrPatternAll.db
DB += evrPatternTest.db
DB += evrPatternTestBsa.db
DB += evrMod36.db
# LCLS templates
DB += fiducial.db
# Event Codes
DB += evrEventCode.template
DB += lclsEventCodes.db
DB += lclsEventCodes.substitutions
# Event Code Sequences
DB += eventCodeSequenceAll.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

