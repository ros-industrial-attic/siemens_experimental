# Comment/uncomment the following line to disable/enable debugging
DEBUG = y
#NOTRACE = y

TOPDIR = $(shell cd ..; pwd)

# Which compiler
#CC = gcc
CC = g++

# Where are include files kept
INCLUDE = -I $(TOPDIR)/inc \
 -I $(TOPDIR)/../pniolib/tracelib/inc \
 -I $(TOPDIR)/../driver \
 -I $(TOPDIR)/../pniolib/iobase/inc \
 -I $(TOPDIR)/../pniolib/iodataupdate/inc \
 -I $(TOPDIR)/../pniolib/kramiotlb/inc

OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC -fno-exceptions $(INCLUDE) -D_DPR_LINUX -DIO_ROUTER
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


#ifeq ($(RTAI),y)
#  rtaiconfigbin := $(if $(@shell rtai-config --prefix),\
#   $(shell rtai-config --prefix)/bin/rtai-config,\
#   $(shell /usr/realtime/bin/rtai-config --prefix)/bin/rtai-config)

#  CFLAGS += -I$(shell $(rtaiconfigbin) --prefix)/include -DRTAI
#endif
#ifeq ($(XENO),y)
#  xenoconfigbin := $(if $(@shell xeno-config --prefix),\
#   $(shell xeno-config --prefix)/bin/xeno-config,\
#   $(shell /usr/xenomai/bin/xeno-config --prefix)/bin/xeno-config)

#  CFLAGS += -I$(shell $(xenoconfigbin) --prefix)/include -DXENOMAI
#endif


SRC = $(SRC_DIR)/cp_info.cpp \
      $(SRC_DIR)/ldah_flash.cpp \
      $(SRC_DIR)/ldah_fwdown.cpp \
      $(SRC_DIR)/ldah_confdown.cpp \
      $(SRC_DIR)/ldah_check.cpp \
      $(SRC_DIR)/servusrx.cpp

OBJ = $(OBJ_DIR)/cp_info.o \
      $(OBJ_DIR)/ldah_flash.o \
      $(OBJ_DIR)/ldah_fwdown.o \
      $(OBJ_DIR)/ldah_confdown.o \
      $(OBJ_DIR)/ldah_check.o \
      $(OBJ_DIR)/servusrx.o

LIB = $(LIB_DIR)/serv.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all:  $(OBJ) $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)

$(OBJ_DIR)/cp_info.o: $(SRC_DIR)/cp_info.cpp
	$(COMPILE)
$(OBJ_DIR)/ldah_flash.o: $(SRC_DIR)/ldah_flash.cpp
	$(COMPILE)
$(OBJ_DIR)/ldah_fwdown.o: $(SRC_DIR)/ldah_fwdown.cpp
	$(COMPILE)
$(OBJ_DIR)/ldah_confdown.o: $(SRC_DIR)/ldah_confdown.cpp
	$(COMPILE)
$(OBJ_DIR)/ldah_check.o: $(SRC_DIR)/ldah_check.cpp
	$(COMPILE)
$(OBJ_DIR)/servusrx.o: $(SRC_DIR)/servusrx.cpp
	$(COMPILE)

lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)


