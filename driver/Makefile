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

# TARGET = pci_mrfevr

ifneq ($(KERNELRELEASE),)

pci_mrfevr-objs := jtag.o eeprom.o local_conf.o mrfcommon.o mrfevr.o erapi.o

obj-m := pci_mrfevr.o

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	-rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.*

.PHONY: modules modules_install clean

endif
