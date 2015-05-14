CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy1_load_proj.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy1_load_proj

pnioeasy1: pnioeasy1_load_proj.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy1_load_proj.o: pnioeasy1_load_proj.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy1_load_proj pnioeasy1_load_proj.o
