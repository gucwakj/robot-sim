#
# chrobotsim
#
PACKAGE=chrobotsim
VERSION=0.0.1
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC=../crobotsim
HEADERS= $(SRC)/robotsim.h $(SRC)/mobotsim.h $(SRC)/linkbotsim.h $(SRC)/graphics.h $(SRC)/base.h
CPPFLAGS = -DdDOUBLE -fPIC
INCLUDES=-I../opende/sys/include/ -I../openscenegraph/sys/include/ -I../tinyxml2/
LIBPATHS=-L../opende/sys/lib/ -L../openscenegraph/build/lib/ -L../tinyxml2/build/
LIBS = -losg -losgViewer -losgUtil -lOpenThreads -losgGA -losgDB -losgFX -lode -ltinyxml2

target: librobotsim.dl

OBJS =	robotsim.o \
		base.o \
		mobotsim.o \
		mobotgroup.o \
		linkbotsim.o \
		linkbotgroup.o \
		graphics.o \
		chrobotsim_chdl.o \
		chmobotsim_chdl.o \
		chlinkbotisim_chdl.o \
		chlinkbotlsim_chdl.o \
		chlinkbott_chdl.o

chrobotsim_chdl.o: chrobotsim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chrobotsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)
	
chmobotsim_chdl.o: chmobotsim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chmobotsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)
	
chlinkbotisim_chdl.o: chlinkbotisim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chlinkbotisim_chdl.cpp $(CPPFLAGS) $(INCLUDES)
	
chlinkbotlsim_chdl.o: chlinkbotlsim_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chlinkbotlsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbott_chdl.o: chlinkbott_chdl.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus chlinkbott_chdl.cpp $(CPPFLAGS) $(INCLUDES)

robotsim.o: $(SRC)/robotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/robotsim.cpp $(CPPFLAGS) $(INCLUDES)

base.o: $(SRC)/base.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus $(SRC)/base.cpp $(CPPFLAGS) $(INCLUDES)

mobotsim.o: $(SRC)/mobotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/mobotsim.cpp $(CPPFLAGS) $(INCLUDES)

mobotgroup.o: $(SRC)/mobotgroup.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/mobotgroup.cpp $(CPPFLAGS) $(INCLUDES)

linkbotsim.o: $(SRC)/linkbotsim.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/linkbotsim.cpp $(CPPFLAGS) $(INCLUDES)

linkbotgroup.o: $(SRC)/linkbotgroup.cpp $(HEADERS) 
	ch dlcomp librobotsim.dl cplusplus $(SRC)/linkbotgroup.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobotsim.dl cplusplus $(SRC)/graphics.cpp $(CPPFLAGS) $(INCLUDES)

librobotsim.dl: $(OBJS)
	ch dllink librobotsim.dl cplusplus $(OBJS) $(LIBPATHS) $(LIBS)

install:
	ch pkginstall.ch $(PACKAGE)

uninstall:
	ch pkginstall.ch -u $(PACKAGE)

createpkg: librobotsim.dl
	echo Building $(PACKAGE)-$(VERSION).zip
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	mkdir $(PKGDIR)/bin
	cp ../opende/sys/lib/libode.so $(PKGDIR)/bin
	cp ../tinyxml2/build/libtinyxml2.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosg.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgViewer.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgUtil.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgDB.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgGA.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgFX.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgText.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libOpenThreads.so $(PKGDIR)/bin
	mkdir $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_png.so $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_3ds.so $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_tga.so $(PKGDIR)/bin/osgPlugins-3.0.1
	mkdir $(PKGDIR)/data
	cp -R ../resources/ground $(PKGDIR)/data
	cp -R ../resources/mobot $(PKGDIR)/data
	cp -R ../resources/linkbot $(PKGDIR)/data
	mkdir $(PKGDIR)/demos
	cp -R ../../demos/chdemos/mobot $(PKGDIR)/demos
	cp -R ../../demos/chdemos/linkbot $(PKGDIR)/demos
	mkdir $(PKGDIR)/dl
	cp librobotsim.dl $(PKGDIR)/dl
	mkdir $(PKGDIR)/include
	cp $(HEADERS) $(PKGDIR)/include
	mkdir $(PKGDIR)/lib
	cp chrobotsim.chf $(PKGDIR)/lib
	cp chmobotsim.chf $(PKGDIR)/lib
	cp chlinkbotisim.chf $(PKGDIR)/lib
	cp chlinkbotlsim.chf $(PKGDIR)/lib
	cp chlinkbott.chf $(PKGDIR)/lib
	cp delay.chf $(PKGDIR)/lib
	cp Makefile $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	echo Done Building $(PACKAGE)-$(VERSION)

cleanpkg:
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip

clean:
	rm *.o *.dl

