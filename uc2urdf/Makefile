include $(shell rospack find mk)/cmake.mk

PROG = uc2urdf
TRGTS = uc2urdf

$(PROG): $(TRGTS)
all: $(TRGTS)


uc2urdf:
	+$(MAKE) -C src
clean:
	rm -f *.o *~
	cd src; make clean
# End of the main math makefile