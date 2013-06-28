#!../../bin/linux-x86_64/evrIoc

# This is a sample st.cmd file for the EVR.

epicsEnvSet("IOCNAME",     "ioc-evr-tst1")
epicsEnvSet("EVRTYPE",     "1")
epicsEnvSet("EVRBASE",     "TST:1:EVR")
epicsEnvSet("IOCBASE",     "TST:1:IOC")
epicsEnvSet("EVRCHANNELS", "IP0E=Enabled,IP1E=Enabled")

#############################################################
epicsEnvSet( "ENGINEER", "Michael Browne (mcbrowne)" )
epicsEnvSet( "LOCATION", "$(IOCNAME)" )
epicsEnvSet( "IOCSH_PS1", "$(IOCNAME)> " )
< envPaths
epicsEnvSet("IOC",         "$(IOCNAME)")
cd( "../.." )

# Run common startup commands for linux soft IOC's
< /reg/d/iocCommon/All/pre_linux.cmd

# Register all support components
dbLoadDatabase("dbd/evrIoc.dbd")
evrIoc_registerRecordDeviceDriver(pdbbase)

ErDebugLevel( 0 )

# Initialize PMC EVR
ErConfigure( 0, 0, 0, 0, $(EVRTYPE) )

# Load EVR record instances
dbLoadRecords( "db/evrTest.db", "IOC=$(IOCBASE),EVR=$(EVRBASE),$(EVRCHANNELS)" )

# Initialize the IOC and start processing records
iocInit()

# All IOCs should dump some common info after initial startup.
< /reg/d/iocCommon/All/post_linux.cmd
