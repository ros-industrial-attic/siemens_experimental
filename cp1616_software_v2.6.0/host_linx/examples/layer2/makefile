# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

TOPDIR = $(shell pwd)

CFLAGS = -D_DPR_LINUX -Wall -I $(TOPDIR)/../../driver
# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS += -g
else
# Options for release
  CFLAGS += -O
endif

EXTRAFLAGS =

SRC1 = $(TOPDIR)/l2eth_ping.c

EXE = l2eth_ping

all: $(EXE)


l2eth_ping: $(SRC1)
	g++ $(CFLAGS) -o l2eth_ping $(SRC1) -ll2interface

clean:
	rm -f $(EXE)
