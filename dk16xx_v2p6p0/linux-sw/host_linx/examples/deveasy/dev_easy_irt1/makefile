# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

TOPDIR = $(shell pwd)
SRCDIR = $(TOPDIR)
INCDIR = $(TOPDIR)
BINDIR = $(TOPDIR)

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
# Options for development - debug
  CFLAGS = -g -Wall
else
# Options for release
  CFLAGS = -O -Wall
endif

SRC = $(SRCDIR)/easy_dev_irt1.c
EXE = $(BINDIR)/easy_dev_irt1
CFG = $(SRCDIR)/easy_dev_cfg.h

all: $(EXE)

$(EXE): $(SRC) $(CFG)
	gcc $(CFLAGS) -I $(INCDIR) -o $(EXE) -lpniousr $(SRC)

clean:
	rm -f $(EXE)
