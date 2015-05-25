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

SRC1 = $(TOPDIR)/cpio.c

EXE = cpio

all: $(EXE)

cpio: $(SRC1)
	g++ $(CFLAGS) -o cpio $(SRC1) -lpniousr

clean:
	rm -f $(EXE)

cpreset:
	./cp16xxtest -y reset

install: $(EXE)

deinstall:

lint:
