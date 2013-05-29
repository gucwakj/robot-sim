#
# chrobotsim
#
PACKAGE=chrobotsim
VERSION=0.0.1
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC=../crobotsim
HEADERS= $(SRC)/robotsim.h $(SRC)/mobotsim.h $(SRC)/linkbotsim.h $(SRC)/graphics.h $(SRC)/base.h
CPPFLAGS = -DdDOUBLE -fPIC
INCLUDES=-I../opende/sys/include/ -I../tinyxml2/
LIBS=-L../opende/sys/lib/ -L../tinyxml2/

target: librobotsim.dl

chrobotsim_chdl.o: chrobotsim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chrobotsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

robotsim.o: $(SRC)/robotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/robotsim.cpp $(CPPFLAGS) $(INCLUDES)

base.o: $(SRC)/base.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus $(SRC)/base.cpp $(CPPFLAGS) $(INCLUDES)

mobotsim.o: $(SRC)/mobotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/mobotsim.cpp $(CPPFLAGS) $(INCLUDES)

linkbotsim.o: $(SRC)/linkbotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/linkbotsim.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus $(SRC)/graphics.cpp $(CPPFLAGS) $(INCLUDES)

librobotsim.dl: robotsim.o mobotsim.o linkbotsim.o base.o graphics.o chrobotsim_chdl.o
	ch dllink librobotsim.dl cplusplus robotsim.o mobotsim.o linkbotsim.o base.o graphics.o chrobotsim_chdl.o $(LIBS) -lpthread -lrt -losg -losgViewer -losgSim -losgUtil -lOpenThreads -losgGA -losgDB -losgFX -lode -ltinyxml2

install:
	ch pkginstall.ch $(PACKAGE)
	sudo cp ../../opende/sys/lib/libode.so.1 /usr/local/ch/extern/lib/
	sudo cp ../../tinyxml2/libtinyxml2.so.1 /usr/local/ch/extern/lib/

uninstall:
	ch pkginstall.ch -u $(PACKAGE)

createpkg: librobotsim.dl
	echo Building $(PACKAGE)-$(VERSION).zip
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	mkdir $(PKGDIR)/data
	cp -R ../resources/* $(PKGDIR)/data
	mkdir $(PKGDIR)/demos
	cp -R ../../demos/chdemos/* $(PKGDIR)/demos
	mkdir $(PKGDIR)/dl
	cp librobotsim.dl $(PKGDIR)/dl
	#mkdir $(PKGDIR)/docs
	#cp ../docs/robotsim.pdf $(PKGDIR)/docs
	mkdir $(PKGDIR)/include
	cp $(HEADERS) $(PKGDIR)/include
	mkdir $(PKGDIR)/lib
	cp chrobotsim.chf $(PKGDIR)/lib
	cp Makefile $(PACKAGE)-$(VERSION)
	#cp README $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	echo Done Building $(PACKAGE)-$(VERSION)

clean:
	rm *.o *.dl

