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
 -I $(TOPDIR)/../kramiotlb/inc \
 -I $(TOPDIR)/../../driver

OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC $(INCLUDE) -D_DPR_LINUX -DCNS_USED_OUTSIDE_EDD -DEDD_CFG_ERTEC_400 -DIO_ROUTER
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

SRC   =  $(SRC_DIR)/iodu_com.c \
         $(SRC_DIR)/iodu_con.c \
         $(SRC_DIR)/iodu_dev.c \
         $(SRC_DIR)/cns_e400.c

OBJ   =  $(OBJ_DIR)/iodu_com.o \
         $(OBJ_DIR)/iodu_con.o \
         $(OBJ_DIR)/iodu_dev.o \
         $(OBJ_DIR)/cns_e400.o

LIB  =  $(LIB_DIR)/iodataupdate.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all:  $(OBJ) $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)

$(OBJ_DIR)/iodu_com.o:  $(SRC_DIR)/iodu_com.c
	$(COMPILE)
$(OBJ_DIR)/iodu_con.o:  $(SRC_DIR)/iodu_con.c
	$(COMPILE)
$(OBJ_DIR)/iodu_dev.o:  $(SRC_DIR)/iodu_dev.c
	$(COMPILE)
$(OBJ_DIR)/cns_e400.o:  $(SRC_DIR)/cns_e400.c
	$(COMPILE)

lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)
