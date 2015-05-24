
TOPDIR = $(shell pwd)



SRC4 = $(TOPDIR)/time_daemon.cpp


EXE = timedaemon

all: $(EXE)

timedaemon: $(SRC4)
	g++ $(CFLAGS) -o timedaemon $(SRC4) -lpniousr -lservusr

clean:
	rm -f $(EXE)