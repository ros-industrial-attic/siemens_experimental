CC = g++
RM = rm
MAKE = make

OBJ = pnioeasy2_load_fw.o
CPPFLAGS = -O
LDFLAGS  = -lpniousr

all: pnioeasy2_load_fw

pnioeasy2: pnioeasy2_load_fw.o
	$(CC) -o $@ $< $(LDFLAGS)

pnioeasy2_load_fw.o: pnioeasy2_load_fw.cpp
	$(CC) $(CPPFLAGS) -c -o $@ $<

clean:
	rm -f pnioeasy2_load_fw pnioeasy2_load_fw.o
