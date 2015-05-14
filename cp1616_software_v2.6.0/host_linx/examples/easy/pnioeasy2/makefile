CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy2.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy2

pnioeasy2: pnioeasy2.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy2.o: pnioeasy2.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy2 pnioeasy2.o
