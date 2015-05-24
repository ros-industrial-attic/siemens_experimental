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

LIB =libservusr.so
LIBA =libservusr.a
LIBOBJ = lnk/serv.lo \
	 ../pniolib/tracelib/lnk/tracelib.lo

all:
	make -C csd
	make -C ../pniolib/tracelib/csd
	$(AR) rc $(LIBA) $(LIBOBJ)
	$(CC) -shared -o $(LIB) $(LIBOBJ) -lpthread -lrt

clean:
	make -C csd clean
	make -C ../pniolib/tracelib/csd clean
	rm -f $(LIB) $(LIBA)

install: all
	cp -p $(LIB) $(INSTALLDIRLIB)/
	cp -p $(LIBA) $(INSTALLDIRLIB)/
	cp -p inc/servusrx.h $(INSTALLDIRINC)/
	cp -p ../pniolib/iobase/inc/pniobase.h $(INSTALLDIRINC)/
	cp -p ../pniolib/iobase/inc/pnioerrx.h $(INSTALLDIRINC)/
	cp -p ../pniolib/tracelib/inc/servtrace.conf $(ETC)/

deinstall:
	rm -f $(INSTALLDIRLIB)/$(LIB) \
	      $(INSTALLDIRLIB)/$(LIBA)
	rm -f $(INSTALLDIRINC)/servusrx.h

lint:
	make -C csd lint
	make -C ../pniolib/tracelib/csd lint
