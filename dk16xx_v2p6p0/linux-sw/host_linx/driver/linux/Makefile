#
# Makefile for the CP1604, CP1616
#

# KERNELDIR can be speficied on the command line or environment
ifeq ($(KERNELDIR),)
    KERNELDIR := /lib/modules/$(shell uname -r)/build
    VERSION   := $(shell uname -r)
endif

TOPDIR      := $(shell pwd)
INCLUDEDIR  := $(KERNELDIR)/include
ifeq ($(VERSION),)
    VERSIONFILE := $(INCLUDEDIR)/linux/version.h
    VERSION     := $(shell awk -F\" '/REL/ {print $$2}' $(VERSIONFILE))
endif
MAJOR       := $(shell echo $(VERSION) | cut -d. -f1-2)

# We need the configuration file, for CONFIG_SMP and possibly other stuff
# (especiall for RISC platforms, where CFLAGS depends on the exact
# processor being used).
CONFIG := $(shell if test -f $(KERNELDIR)/.config; then echo y; fi)
ifeq ($(CONFIG),y)
    include $(KERNELDIR)/.config
endif

# ARCH can be speficed on the comdline or env. too, and defaults to this arch
# Unfortunately, we can't easily extract if from kernel configuration
# (well, we could look athe asm- symlink... don't know if worth the effort)
ifndef ARCH
  ARCH := $(shell uname -m | sed -e s/i.86/i386/ -e s/sun4u/sparc64/ \
    -e s/arm.*/arm/ -e s/sa110/arm/)
endif

# This is useful if cross-compiling. Taken from kernel Makefile (CC changed)
AS      =$(CROSS_COMPILE)as
LD      =$(CROSS_COMPILE)ld
CC      =$(CROSS_COMPILE)gcc
CPP     =$(CC) -E
AR      =$(CROSS_COMPILE)ar
NM      =$(CROSS_COMPILE)nm
STRIP   =$(CROSS_COMPILE)strip
OBJCOPY =$(CROSS_COMPILE)objcopy
OBJDUMP =$(CROSS_COMPILE)objdump

# The platform-specific Makefiles include portability nightmares.
# Some platforms, though, don't have one, so check for existence first
ARCHMAKEFILE = $(TOPDIR)/Makefile.$(ARCH)
ifeq ($(ARCHMAKEFILE),$(wildcard $(ARCHMAKEFILE)))
  include $(ARCHMAKEFILE)
endif

# CFLAGS: all assignments to CFLAGS are inclremental, so you can specify
# the initial flags on the command line or environment, if needed.

ifeq ($(DEBUG),y)
    EXTRA_CFLAGS += -O -g -DDEBUG -Wall
else
    EXTRA_CFLAGS += -O2
endif

ifeq ($(LATENCY),y)
    EXTRA_CFLAGS += -DLATENCY_MEASUREMENT
endif

ifeq ($(RTAI),y)
    rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
     $(shell rtai-config --prefix)/bin/rtai-config,\
     $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)

    RTAIMODULEDIR := $(shell $(rtaiconfigbin) --module-dir)
    INSTALLDIRRTAI := /lib/modules/$(shell uname -r)/rtai
    EXTRA_CFLAGS += -I$(shell $(rtaiconfigbin) --prefix)/include -DRTAI
endif
ifeq ($(XENO),y)
    xenoconfigbin := $(if $(@shell xeno-config --prefix),\
     $(shell xeno-config --prefix)/bin/xeno-config,\
     $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)

    EXTRA_CFLAGS += -I$(shell $(xenoconfigbin) --prefix)/include -DXENOMAI
endif
EXTRA_CFLAGS += -Wall -D__KERNEL__ -DMODULE -I$(INCLUDEDIR) -D_DPR_HOST_ -D_DPR_LINUX
ifeq ($(NET),y)
    EXTRA_CFLAGS += -DNETWORK_DRIVER -I$(src)/.. -I$(src)/../../layer2lib/inc
endif

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m := cp16xx.o
cp16xx-y := cp16xx_linux.o cp16xx_ioctl32.o
else
# normal Makefile
all:
ifeq ($(RTAI),y)
	@echo "" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "***************** Build the kernel module with RTAI support!!! **************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
endif
ifeq ($(XENO),y)
	@echo "" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "***************** Build the kernel module with Xenomai support!!!************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
endif
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(TOPDIR) modules

install: all
# notice RTAI-3.2 has a problem with circullar dependency and therefor not suitable for automatic loading
# since RTAI-3.3-test4 this problem is fixed
ifeq ($(RTAI),y)
	install -d $(INSTALLDIRRTAI)
	install -c $(RTAIMODULEDIR)/* $(INSTALLDIRRTAI)
endif
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(TOPDIR) modules_install
	/sbin/depmod -aF /lib/modules/$(VERSION)

deinstall:
	rm -f $(shell find /lib/modules/$(VERSION) -name 'cp16xx.*')
	rm -f $(shell find /lib/modules/$(VERSION) -name 'rtai_*.*')

clean:
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(TOPDIR) clean
	rm -f $(obj-m)
endif

