# Comment/uncomment to allow detection of unconfigured board
#DETECT_UNCONFIGURED_BOARD=y
# Comment/uncomment to hide/show debugging messages in system log
#DEBUG=y

USR_CFLAGS += -fno-keep-inline-functions
ifeq ($(DETECT_UNCONFIGURED_BOARD),y)
#  CFLAGS += -DDETECT_UNCONFIGURED_BOARD
endif
ifeq ($(DEBUG),y)
#  CFLAGS += -DDEBUG
endif

#
# This is sort of a schitzophrenic Makefile.  It is invoked in three different
# ways:
#     1. During part of a normal build, with target install, to install evrmemmap.h
#        so we have the correct memory map from userland.
#     2. To build the modules, with target modules.  This just calls make modules for
#        the kernel build directory with M pointing here.
#     3. From the kernel build directory, with KERNELRELEASE set, when building
#        the modules.
#

ifneq ($(KERNELRELEASE),)
#
# This should only be invoked if we are compiling the kernel modules.
#

evr_device-objs := jtag.o eeprom.o local_conf.o mrfcommon.o mrfevr.o evrram.o

obj-m := evr_device.o

else
#
# Start with the regular build stuff.
#
TOP = ..
include $(TOP)/configure/CONFIG

INC += evrmemmap.h

PROD_IOC_Linux = evrAlloc

evrAllocSRCS += evrAlloc.c

#=======================================================================#
#                               RULES                                   #
#=======================================================================#

#---------------------
# Generic EPICS build rules
#
include $(TOP)/configure/RULES

#
# Now, this is for compiling the modules.
#
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Build these RT kernel drivers
RT_KERNELS = 3.14.12-rt9 3.18.11-rt7

# XCROSS_HOME and KERNELDIR for linuxRT kernel 3.14.12-rt9
KERNELDIR_3.14.12-rt9	= /afs/slac/g/lcls/package/linuxRT/buildroot-2014.11/buildroot-glibc-x86_64/output/build/linux-3.14.12/
XCROSS_HOME_3.14.12-rt9	= /afs/slac/g/lcls/package/linuxRT/buildroot-2014.11/host/linux-x86_64/x86_64-glibc-linux/usr/bin/x86_64-linux-

# XCROSS_HOME and KERNELDIR for linuxRT kernel 3.18.11-rt7
KERNELDIR_3.18.11-rt7	= /afs/slac/package/linuxRT/buildroot-2015.02/buildroot-2015.02-x86_64/output/build/linux-3.18.11/
XCROSS_HOME_3.18.11-rt7	= /afs/slac/package/linuxRT/buildroot-2015.02/host/linux-x86_64/x86_64/usr/bin/x86_64-linux-

modules: modclean
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	@/bin/rm -f evr_device.ko.`uname -r`.`arch`
	cp evr_device.ko evr_device.ko.`uname -r`.`arch`
	@chmod 0644 evr_device.ko.`uname -r`.`arch`

ifeq '$(EPICS_HOST_ARCH)' 'rhel6-x86_64'
modules: linuxRT
endif

.PHONY: $(RT_KERNELS)
linuxRT: modclean $(RT_KERNELS)

$(RT_KERNELS):
	$(MAKE) ARCH=x86_64 CROSS_COMPILE=$(XCROSS_HOME_$(@)) -C $(KERNELDIR_$(@)) M=$(PWD) modules
	@/bin/rm -f evr_device.ko.$(@).x86_64
	cp evr_device.ko evr_device.ko.$(@).x86_64
	@chmod 0644 evr_device.ko.$(@).x86_64

realclean clean: modclean

modclean:
	-rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.* module*

.PHONY: modules modules_install clean

endif
