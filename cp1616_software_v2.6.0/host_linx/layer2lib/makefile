INSTALLDIR =/usr
INSTALLDIRLIB =$(INSTALLDIR)/lib
INSTALLDIRINC =$(INSTALLDIR)/include
ETC =/etc
LD = ld
CC =g++
AR =ar

LIB =libl2interface.so
LIBA =libl2interface.a
LIBOBJ = lnk/l2interface.lo \
	 ../pniolib/tracelib/lnk/tracelib.lo \
	 ../servlib/lnk/serv.lo

all:
	make -C csd
	make -C ../pniolib/tracelib/csd
	make -C ../servlib/csd
	$(AR) rc $(LIBA) $(LIBOBJ)
	$(CC) -shared -o $(LIB) $(LIBOBJ) -lpthread -lrt

clean:
	make -C csd clean
	make -C ../pniolib/tracelib/csd clean
	rm -f $(LIB) $(LIBA)

install: all
	cp -p $(LIB) $(INSTALLDIRLIB)/
	cp -p $(LIBA) $(INSTALLDIRLIB)/
	cp -p inc/l2eth_defs.h $(INSTALLDIRINC)/
	cp -p inc/l2eth_user.h $(INSTALLDIRINC)/
	cp -p inc/l2eth_errs.h $(INSTALLDIRINC)/
	cp -p inc/l2eth_config.h $(INSTALLDIRINC)/
	cp -p inc/l2eth_trace.conf $(ETC)/

deinstall:
	rm -f $(INSTALLDIRLIB)/$(LIB) \
	      $(INSTALLDIRLIB)/$(LIBA)
	rm -f $(INSTALLDIRINC)/l2eth_defs.h
	rm -f $(INSTALLDIRINC)/l2eth_user.h
	rm -f $(INSTALLDIRINC)/l2eth_errs.h
	rm -f $(INSTALLDIRINC)/l2eth_config.h
	rm -f $(ETC)/l2eth_trace.conf

lint:
	make -C csd lint
	make -C ../pniolib/tracelib/csd lint

