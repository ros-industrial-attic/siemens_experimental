INSTALLDIR =/usr

LBITS := $(shell uname -m)

#64-Bit System?
ifeq ($(LBITS),x86_64)
  INSTALLDIRLIB =$(INSTALLDIR)/lib64
else
#32-Bit System?
  INSTALLDIRLIB =$(INSTALLDIR)/lib
endif

INSTALLDIRINC =$(INSTALLDIR)/include
ETC =/etc
LD = ld
CC =g++
AR =ar

LIB =libpniousr.so
LIBA =libpniousr.a
LIBOBJ =iobase/lnk/iobase.lo \
    iodataupdate/lnk/iodataupdate.lo \
    kramiotlb/lnk/kramiotlb.lo \
        tracelib/lnk/tracelib.lo 

LDFLAGS =
ifeq ($(RTAI),y)
    rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
     $(shell rtai-config --prefix)/bin/rtai-config,\
     $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)

    #LDFLAGS += -L$(shell $(rtaiconfigbin) --library-dir) -llxrt
    LDFLAGS += $(shell $(rtaiconfigbin) --library-dir)/liblxrt.a
endif

ifeq ($(XENO),y)
    xenoconfigbin := $(if $(@shell xeno-config --prefix),\
     $(shell xeno-config --prefix)/bin/xeno-config,\
     $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)

    #LDFLAGS += -L$(shell $(xenoconfigbin) --library-dir) -lnative
    LDFLAGS += $(shell $(xenoconfigbin) --library-dir)/libnative.a
endif

all:
ifeq ($(RTAI),y)
	@echo "" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "***************** Build the PNIO library with RTAI support!!! ***************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
endif
ifeq ($(XENO),y)
	@echo "" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "***************** Build the PNIO library with Xenomai support!!! ************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
	@echo "*****************************************************************************************" 1>&2
endif
	make -C iobase/csd
	make -C iodataupdate/csd
	make -C kramiotlb/csd
	make -C tracelib/csd
	make -C ../servlib
	$(AR) rc $(LIBA) $(LIBOBJ)
	$(CC) -shared -o $(LIB) $(LIBOBJ) -lpthread -lrt $(LDFLAGS) -L../servlib -lservusr

clean:
	make -C iobase/csd clean
	make -C iodataupdate/csd clean
	make -C kramiotlb/csd clean
	make -C tracelib/csd clean
	rm -f $(LIB) $(LIBA)

deinstall:
	rm -f $(INSTALLDIRLIB)/$(LIB) \
	      $(INSTALLDIRLIB)/$(LIBA)
	rm -f $(INSTALLDIRINC)/pniobase.h \
	      $(INSTALLDIRINC)/pniousrd.h \
	      $(INSTALLDIRINC)/pniousrx.h \
	      $(INSTALLDIRINC)/pniousrt.h \
	      $(INSTALLDIRINC)/pnioerrx.h

install: all
	cp -p $(LIB) $(INSTALLDIRLIB)/
	cp -p $(LIBA) $(INSTALLDIRLIB)/
	cp -p iobase/inc/pniousrt.h $(INSTALLDIRINC)/
	cp -p iobase/inc/pniousrx.h $(INSTALLDIRINC)/
	cp -p iobase/inc/pniousrd.h $(INSTALLDIRINC)/
	cp -p iobase/inc/pnioerrx.h $(INSTALLDIRINC)/
	cp -p iobase/inc/pniobase.h $(INSTALLDIRINC)/
	cp -p tracelib/inc/pniotrace.conf $(ETC)/

lint:
	make -C iobase/csd lint
	make -C iodataupdate/csd lint
	make -C kramiotlb/csd lint
	make -C tracelib/csd lint

