CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy3.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy3

pnioeasy3: pnioeasy3.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy3.o: pnioeasy3.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy3 pnioeasy3.o
