BINDIR = ../bin
INCLUDE_EIGEN =../include
LIBDIR=../lib
CC=g++
CFLAGS=-I $(INCLUDE_EIGEN) -I ../src -g -O2

ODIR=../obj

all: uc2urdf

uc2urdf_SOURCES = $(wildcard *.cpp)
#substitute .cpp by .o to obtain object filenames
uc2urdf_OBJECTS = $(patsubst %, $(ODIR)/%,$(uc2urdf_SOURCES:.cpp=.o))

$(ODIR)/%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS)


uc2urdf: $(uc2urdf_OBJECTS)
	$(CC) -o $@ $(uc2urdf_OBJECTS) $(CFLAGS) $(LIBS) && mv uc2urdf ../bin

.PHONY: clean

clean:
	rm -rf $(ODIR)/*.o *~
