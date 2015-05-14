# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

TOPDIR = $(shell pwd)

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS = -g -Wall -D_DPR_LINUX
else
# Options for release
  CFLAGS = -O -Wall -D_DPR_LINUX
endif

ifeq ($(RTAI),y)
    rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
     $(shell rtai-config --prefix)/bin/rtai-config,\
     $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)

    #CFLAGS += -I$(shell $(rtaiconfigbin) --prefix)/include -DRTAI -L$(shell $(rtaiconfigbin) --library-dir) -llxrt
    CFLAGS += -I$(shell $(rtaiconfigbin) --prefix)/include -DRTAI $(shell $(rtaiconfigbin) --library-dir)/liblxrt.a
endif
ifeq ($(XENO),y)
    xenoconfigbin := $(if $(@shell xeno-config --prefix),\
     $(shell xeno-config --prefix)/bin/xeno-config,\
     $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)

    #CFLAGS += -I$(shell $(xenoconfigbin) --prefix)/include -DXENOMAI -L$(shell $(xenoconfigbin) --library-dir) -lnative
    CFLAGS += -I$(shell $(xenoconfigbin) --prefix)/include -DXENOMAI $(shell $(xenoconfigbin) --library-dir)/libnative.a
endif

ifeq ($(LATENCY),y)
    CFLAGS += -DLATENCY_MEASUREMENT
endif

SRC1 = $(TOPDIR)/cp16xxtest.c
SRC2 = $(TOPDIR)/pniotest.c
SRC3 = $(TOPDIR)/pnioping.c
SRC_L2T = $(TOPDIR)/l2eth_test_prog.c
SRC_L2B = $(TOPDIR)/l2eth_basic.c

EXE = pniotest pnioping cp16xxtest

all: $(EXE)

cp16xxtest: $(SRC1)
	g++ $(CFLAGS) -I $(TOPDIR)/../driver -o cp16xxtest $(SRC1) -lpthread

pniotest: $(SRC2)
	g++ $(CFLAGS) -o pniotest $(SRC2) -lpniousr

pnioping: $(SRC3)
	g++ $(CFLAGS) -o pnioping $(SRC3) -lpniousr

#--- layer2 test programm: l2eth_test_prog
#
l2t: $(SRC_L2T)
	g++ $(CFLAGS) -I $(TOPDIR)/../layer2lib/inc -o l2eth_test $(SRC_L2T)  -ll2interface

l2b: $(SRC_L2B)
	g++ $(CFLAGS) -I $(TOPDIR)/../layer2lib/inc -o l2eth_basic $(SRC_L2B)  -ll2interface

clean:
	rm -f $(EXE)

cpreset:
	./cp16xxtest -y reset

install: $(EXE)

deinstall:

lint:
