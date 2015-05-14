CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy_irt1.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy_irt1

pnioeasy_irt: pnioeasy_irt1.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy_irt.o: pnioeasy_irt1.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy_irt1 pnioeasy_irt1.o
