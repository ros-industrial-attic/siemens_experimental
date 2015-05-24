# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y
#NOTRACE = y

TOPDIR = $(shell cd ..; pwd)

# Which compiler
#CC = gcc
CC = g++

# Where are include files kept
INCLUDE = -I $(TOPDIR)/inc \
 -I $(TOPDIR)/../iobase/inc \
 -I $(TOPDIR)/../iodataupdate/inc \
 -I $(TOPDIR)/../../driver

OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC $(INCLUDE) -D_DPR_LINUX
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

SRC   =  $(SRC_DIR)/krtlb_common.c \
         $(SRC_DIR)/krtlb_host.c

OBJ   =  $(OBJ_DIR)/krtlb_common.o \
         $(OBJ_DIR)/krtlb_host.o

LIB  =  $(LIB_DIR)/kramiotlb.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all:  $(OBJ) $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)

$(TOPDIR)/obj/krtlb_common.o:  $(TOPDIR)/src/krtlb_common.c
	$(COMPILE)

$(OBJ_DIR)/krtlb_host.o:  $(SRC_DIR)/krtlb_host.c
	$(COMPILE)

$(OBJ_DIR)/krtlb_target.o:  $(SRC_DIR)/krtlb_target.c
	$(COMPILE)

lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)
