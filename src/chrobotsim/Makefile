#
# ChRobotSim
#
PACKAGE=chrobotsim
VERSION=0.0.1
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC=../CRobotSim
HEADERS= $(SRC)/robotsim.h $(SRC)/mobotsim.h $(SRC)/robot.h $(SRC)/graphics.h
CPPFLAGS = -DdDOUBLE
INCLUDES=-I../opende/sys/include/ -I../tinyxml2/
LIBS=-L../opende/sys/lib/ -L../tinyxml2/

target: librobotsim.dl

chrobotsim_chdl.o: chrobotsim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chrobotsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

robotsim.o: $(SRC)/robotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus ../CRobotSim/robotsim.cpp $(CPPFLAGS) $(INCLUDES)

robot.o: $(SRC)/robot.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus ../CRobotSim/robot.cpp $(CPPFLAGS) $(INCLUDES)

mobotsim.o: $(SRC)/mobotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus ../CRobotSim/mobotsim.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus ../CRobotSim/graphics.cpp $(CPPFLAGS) $(INCLUDES)

librobotsim.dl: robotsim.o robot.o mobotsim.o graphics.o chrobotsim_chdl.o
	ch dllink librobotsim.dl cplusplus robotsim.o robot.o mobotsim.o graphics.o chrobotsim_chdl.o $(LIBS) -lpthread -lrt -losg -losgViewer -losgSim -losgUtil -lOpenThreads -losgGA -losgDB -losgFX ../opende/sys/lib/libode.a ../tinyxml2/libtinyxml2.a

install:
	ch pkginstall.ch $(PACKAGE)

uninstall:
	ch pkginstall.ch -u $(PACKAGE)

createpkg: librobotsim.dl
	echo Building $(PACKAGE)-$(VERSION).zip
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	mkdir $(PKGDIR)/data
	cp -R ../test/data/* $(PKGDIR)/data
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

