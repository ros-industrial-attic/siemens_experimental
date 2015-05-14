# Comment/uncomment the following line to disable/enable debugging
DEBUG = y
#NOTRACE = y

TOPDIR = $(shell cd ..; pwd)

# Which compiler
#CC = gcc
CC = g++

# Where are include files kept
INCLUDE = -I $(TOPDIR)/inc \
 -I $(TOPDIR)/../iodataupdate/inc \
 -I $(TOPDIR)/../kramiotlb/inc \
 -I $(TOPDIR)/../tracelib/inc \
 -I $(TOPDIR)/../../driver

OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC -fno-exceptions $(INCLUDE) -D_DPR_LINUX -DIO_ROUTER -DPROFI_ENERGY -DMANAGED_CP_EXCHANGE
# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS += -g
else
# Options for release
  CFLAGS += -O
endif

ifeq ($(NOTRACE),)
  CFLAGS +=  -DPC_TRACE_ON
endif


ifeq ($(RTAI),y)
  rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
   $(shell rtai-config --prefix)/bin/rtai-config,\
   $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)

  CFLAGS += -I$(shell $(rtaiconfigbin) --prefix)/include -DRTAI
endif
ifeq ($(XENO),y)
  xenoconfigbin := $(if $(@shell xeno-config --prefix),\
   $(shell xeno-config --prefix)/bin/xeno-config,\
   $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)

  CFLAGS += -I$(shell $(xenoconfigbin) --prefix)/include -DXENOMAI
endif


SRC = $(SRC_DIR)/traceint.cpp \
      $(SRC_DIR)/fct_common.cpp \
      $(SRC_DIR)/fct_contr.cpp \
      $(SRC_DIR)/fct_device.cpp \
      $(SRC_DIR)/ioconcentrator.cpp \
      $(SRC_DIR)/transferwd.cpp \
      $(SRC_DIR)/pniousrx.cpp \
      $(SRC_DIR)/pniousrd.cpp \
      $(SRC_DIR)/pniowd.cpp \
      $(SRC_DIR)/pnio_pe.cpp \
      $(SRC_DIR)/pnio_pe_util.cpp

OBJ = $(OBJ_DIR)/traceint.o \
      $(OBJ_DIR)/fct_common.o \
      $(OBJ_DIR)/fct_contr.o \
      $(OBJ_DIR)/fct_device.o \
      $(OBJ_DIR)/ioconcentrator.o \
      $(OBJ_DIR)/transferwd.o \
      $(OBJ_DIR)/pniousrx.o \
      $(OBJ_DIR)/pniousrd.o \
      $(OBJ_DIR)/pniowd.o \
      $(OBJ_DIR)/pnio_pe.o \
      $(OBJ_DIR)/pnio_pe_util.o

LIB = $(LIB_DIR)/iobase.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all:  $(OBJ) $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)

$(OBJ_DIR)/traceint.o: $(SRC_DIR)/traceint.cpp
	$(COMPILE)
$(OBJ_DIR)/fct_common.o: $(SRC_DIR)/fct_common.cpp
	$(COMPILE)
$(OBJ_DIR)/fct_contr.o: $(SRC_DIR)/fct_contr.cpp
	$(COMPILE)
$(OBJ_DIR)/ioconcentrator.o: $(SRC_DIR)/ioconcentrator.cpp
	$(COMPILE)
$(OBJ_DIR)/transferwd.o: $(SRC_DIR)/transferwd.cpp
	$(COMPILE)
$(OBJ_DIR)/fct_device.o: $(SRC_DIR)/fct_device.cpp
	$(COMPILE)
$(OBJ_DIR)/pniousrx.o: $(SRC_DIR)/pniousrx.cpp
	$(COMPILE)
$(OBJ_DIR)/pniousrd.o: $(SRC_DIR)/pniousrd.cpp
	$(COMPILE)
$(OBJ_DIR)/pniowd.o: $(SRC_DIR)/pniowd.cpp
	$(COMPILE)
$(OBJ_DIR)/pnio_pe.o: $(SRC_DIR)/pnio_pe.cpp
	$(COMPILE)
$(OBJ_DIR)/pnio_pe_util.o: $(SRC_DIR)/pnio_pe_util.cpp
	$(COMPILE)


lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)


