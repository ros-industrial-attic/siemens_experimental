CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy4.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy4

pnioeasy4: pnioeasy4.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy4.o: pnioeasy4.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy4 pnioeasy4.o
