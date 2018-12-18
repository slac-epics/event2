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

# XTOD templates
DB += basic-evr.substitutions
DB += evrPmc230.template
DB += evrSLAC.template
DB += evrPmcEvent.template
DB += evrSlaveEvent.template
DB += evrEventCtrl.template
DB += evrEventRate.template
DB += evrFiducial.template
DB += evrLabel.template
DB += evrLabels.template
DB += evrTaxi.template
DB += evr.template
DB += evrWithExtDelays.template

#----------------------------------------------------
# Create and install (or just install)
# Archive request files, templates, substitutions like this
# ARCHIVE += <file>.archive
# ARCHIVE += <file>.substitutions
# ARCHIVE += <file>.template
#
ARCHIVE += archive_evr_PMC.archive
ARCHIVE += archive_evr_PMC.tpl-arch
ARCHIVE += archive_evr_SLAC.archive
ARCHIVE += archive_evr_SLAC.tpl-arch

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
# REQ += <file>.req
# REQ += <file>.substitutions
# REQ += <file>.template
#
REQ += autosave_evr_PMC.req
REQ += autosave_evr_PMC.tpl-req
REQ += autosave_evr_SLAC.req
REQ += autosave_evr_SLAC.tpl-req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

