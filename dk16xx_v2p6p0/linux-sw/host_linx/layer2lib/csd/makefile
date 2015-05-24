# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

TOPDIR = $(shell cd ..; pwd)

# Which compiler
#CC = gcc
CC = g++

# Where are include files kept
INCLUDE = -I $(TOPDIR)/inc \
 -I $(TOPDIR)/../driver \
 -I $(TOPDIR)/../pniolib/tracelib/inc


OBJ_DIR = $(TOPDIR)/obj
SRC_DIR = $(TOPDIR)/src
LIB_DIR = $(TOPDIR)/lnk

CFLAGS = -Wall -fPIC $(INCLUDE) -D_DPR_LINUX -DPC_TRACE_ON -D_LINUX

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS += -g
else
# Options for release
  CFLAGS += -O
endif

SRC = $(SRC_DIR)/l2eth_base.c \
	$(SRC_DIR)/l2eth_user.c \
	$(SRC_DIR)/l2eth_buf_mgmt.c

OBJ = $(OBJ_DIR)/l2eth_base.o \
	$(OBJ_DIR)/l2eth_user.o \
	$(OBJ_DIR)/l2eth_buf_mgmt.o

LIB = $(LIB_DIR)/l2interface.lo

LINT = splint
LINTFLAGS = -sysdirs /usr/include -warnposix -preproc $(INCLUDE)

COMPILE = $(CC) -c -o $@ $< $(CFLAGS)

# created missed directories
$(shell mkdir -p $(OBJ_DIR))
$(shell mkdir -p $(LIB_DIR))

all:  $(OBJ) $(LIB)

$(LIB): $(OBJ)
	ld -r -o $(LIB) $(OBJ)


$(OBJ_DIR)/l2eth_base.o: $(SRC_DIR)/l2eth_base.c
	$(COMPILE)

$(OBJ_DIR)/l2eth_user.o: $(SRC_DIR)/l2eth_user.c
	$(COMPILE)

$(OBJ_DIR)/l2eth_buf_mgmt.o: $(SRC_DIR)/l2eth_buf_mgmt.c
	$(COMPILE)


LINTFLAGS = +fce  -A   -d__GNUC__  -D_DPR_LINUX -DPC_TRACE_ON -D_LINUX

LINT = $(TOPDIR)/../flint   $(TOPDIR)/../co-gnu3.lnt  \
	$(INCLUDE) \
	-i/usr/include \
	-i/usr/include/g++ \
	-i/usr/include/g++/i586-suse-linux \
	-i/usr/include/asm  \
	-i/usr/include/asm/mach-voyager \
	-i/usr/lib/gcc-lib/i586-suse-linux/3.3.3/include/ \
	-i$(TOPDIR)/../../flexelint/supp/ansi   \
	-i$(TOPDIR)/inc \
	-i$(TOPDIR)/ \
	-i$(TOPDIR)/../driver \
	-i$(TOPDIR)/../pniolib/tracelib/inc


lint: $(SRC)
	@for f in $(SRC);do if ! $(LINT) $(LINTFLAGS) $$f; then exit 2;fi;done

clean:
	rm -f $(OBJ) $(LIB)


