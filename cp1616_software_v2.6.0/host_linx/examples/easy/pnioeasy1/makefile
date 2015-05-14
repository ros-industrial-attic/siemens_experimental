CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy1.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy1

pnioeasy1: pnioeasy1.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy1.o: pnioeasy1.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy1 pnioeasy1.o
