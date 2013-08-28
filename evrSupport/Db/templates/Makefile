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
# XTOD templates
DB_INSTALLS += evrSLAC.template
DB_INSTALLS += evrPmc230.template
DB_INSTALLS += evrPmcEvent.template
DB_INSTALLS += evrEventCtrl.template

#----------------------------------------------------
# Create and install (or just install)
# Archive request files, templates, substitutions like this
# ARCHIVE += <file>.archive
# ARCHIVE += <file>.substitutions
# ARCHIVE += <file>.template
#
ARCHIVE_INSTALLS += archive_evr.tpl-arch
ARCHIVE_INSTALLS += archive_evr_SLAC.tpl-arch
ARCHIVE_INSTALLS += archive_pmc_evr.tpl-arch
ARCHIVE_INSTALLS += archive_vme_evr.tpl-arch

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
# REQ += <file>.req
# REQ += <file>.substitutions
# REQ += <file>.template
#
REQ += autosave_evr.tpl-req
REQ += autosave_evr_SLAC.tpl-req
REQ += autosave_pmc_evr.tpl-req
REQ += autosave_vme_evr.tpl-req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

