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

SRC = $(SRCDIR)/easy_dev2.c
EXE = $(BINDIR)/easy_dev2

all: $(EXE)

$(EXE): $(SRC)
	gcc $(CFLAGS) -I $(INCDIR) -o $(EXE) -lpniousr $(SRC)

clean:
	rm -f $(EXE)
