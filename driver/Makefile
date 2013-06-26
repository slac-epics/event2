# Comment/uncomment to allow detection of unconfigured board
#DETECT_UNCONFIGURED_BOARD=y
# Comment/uncomment to hide/show debugging messages in system log
#DEBUG=y

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

pci_mrfevr-objs := jtag.o eeprom.o local_conf.o mrfcommon.o mrfevr.o evrram.o

obj-m := pci_mrfevr.o

else
#
# Start with the regular build stuff.
#
TOP = ..
include $(TOP)/configure/CONFIG

INC += evrmemmap.h

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

realclean clean:: modclean

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modclean:
	-rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.*

.PHONY: modules modules_install clean

endif
