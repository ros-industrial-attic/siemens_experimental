# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

TOPDIR = $(shell cd ..; pwd)

# Which compiler
#CC = gcc
CC = g++

# Where are include files kept
INCLUDE = -I $(TOPDIR)/inc \
 -I $(TOPDIR)/../iobase/inc \
 -I $(TOPDIR)/../../driver

OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC $(INCLUDE) -D_DPR_LINUX -DPC_TRACE_ON
# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS += -g
else
# Options for release
  CFLAGS += -O
endif

SRC   =  $(SRC_DIR)/tracemcr.cpp \
         $(SRC_DIR)/traceout.cpp \
         $(SRC_DIR)/trace_os.cpp

OBJ   =  $(OBJ_DIR)/tracemcr.o \
         $(OBJ_DIR)/traceout.o \
         $(OBJ_DIR)/trace_os.o

LIB  =  $(LIB_DIR)/tracelib.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all: $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)

$(OBJ_DIR)/tracemcr.o: $(SRC_DIR)/tracemcr.cpp
	$(COMPILE)
$(OBJ_DIR)/traceout.o: $(SRC_DIR)/traceout.cpp
	$(COMPILE)
$(OBJ_DIR)/trace_os.o: $(SRC_DIR)/trace_os.cpp
	$(COMPILE)


lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)
