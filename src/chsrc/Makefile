#
# chrobosim
#
PACKAGE=chrobosim
VERSION=linux-1.4.70
PKGDIR=$(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC=../librobosim
HEADERS= $(SRC)/robosim.h $(SRC)/mobotsim.h $(SRC)/linkbotsim.h $(SRC)/graphics.h $(SRC)/base.h
CPPFLAGS = -DdDOUBLE -fPIC
INCLUDES=-I../librobosim/build/ -I../opende/sys/include/ -I../openscenegraph/sys/include/ -I../tinyxml2/
LIBPATHS=-L../opende/sys/lib/ -L../openscenegraph/build/lib/ -L../tinyxml2/build/
LIBS = -losg -losgViewer -losgUtil -lOpenThreads -losgGA -losgDB -losgFX -losgShadow -lode -ltinyxml2

target: librobosim.dl

OBJS =	robosim.o \
		base.o \
		mobotsim.o \
		mobotgroupsim.o \
		linkbotsim.o \
		linkbotgroupsim.o \
		graphics.o \
		chmobotsim_chdl.o \
		chlinkbotisim_chdl.o \
		chlinkbotlsim_chdl.o \
		chlinkbottsim_chdl.o \
		rgbhashtable.o

chmobotsim_chdl.o: chmobotsim_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chmobotsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbotisim_chdl.o: chlinkbotisim_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkbotisim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbotlsim_chdl.o: chlinkbotlsim_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkbotlsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbottsim_chdl.o: chlinkbottsim_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkbottsim_chdl.cpp $(CPPFLAGS) $(INCLUDES)

robosim.o: $(SRC)/robosim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/robosim.cpp $(CPPFLAGS) $(INCLUDES)

base.o: $(SRC)/base.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/base.cpp $(CPPFLAGS) $(INCLUDES)

mobotsim.o: $(SRC)/mobotsim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobotsim.cpp $(CPPFLAGS) $(INCLUDES)

mobotgroupsim.o: $(SRC)/mobotgroupsim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobotgroupsim.cpp $(CPPFLAGS) $(INCLUDES)

linkbotsim.o: $(SRC)/linkbotsim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbotsim.cpp $(CPPFLAGS) $(INCLUDES)

linkbotgroupsim.o: $(SRC)/linkbotgroupsim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbotgroupsim.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/graphics.cpp $(CPPFLAGS) $(INCLUDES)

rgbhashtable.o: $(SRC)/rgbhashtable.c $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/rgbhashtable.c $(CPPFLAGS) $(INCLUDES)

librobosim.dl: $(OBJS)
	ch dllink librobosim.dl cplusplus $(OBJS) $(LIBPATHS) $(LIBS)

install:
	ch ./pkginstall.ch $(PACKAGE)

uninstall:
	ch ./pkginsall.ch -u $(PACKAGE)

createpkg: librobosim.dl
	echo Building $(PACKAGE)-$(VERSION).zip
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip
	mkdir -p $(PKGDIR)
	cp ../../README $(PKGDIR)
	mkdir $(PKGDIR)/bin
	cp ../opende/sys/lib/libode.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosg.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgDB.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgFX.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgGA.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgShadow.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgText.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgUtil.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libosgViewer.so $(PKGDIR)/bin
	cp ../openscenegraph/build/lib/libOpenThreads.so $(PKGDIR)/bin
	mkdir $(PKGDIR)/bin/gui
	cp ../configurator/build/robosim $(PKGDIR)/bin/gui
	cp ../configurator/interface3.glade $(PKGDIR)/bin/gui/interface.glade
	cp -R ../configurator/images $(PKGDIR)/bin/gui
	mkdir $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_png.so $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_3ds.so $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../openscenegraph/build/lib/osgPlugins-3.0.1/osgdb_tga.so $(PKGDIR)/bin/osgPlugins-3.0.1
	cp ../tinyxml2/build/libtinyxml2.so $(PKGDIR)/bin
	mkdir $(PKGDIR)/data
	cp -R ../resources/ground $(PKGDIR)/data
	cp -R ../resources/mobot $(PKGDIR)/data
	cp -R ../resources/linkbot $(PKGDIR)/data
	mkdir $(PKGDIR)/dl
	cp librobosim.dl $(PKGDIR)/dl
	mkdir $(PKGDIR)/docs
	cp ../../docs/robosim.pdf $(PKGDIR)/docs
	cp ../../docs/images/robosim.png $(PKGDIR)/docs
	mkdir $(PKGDIR)/include
	cp $(HEADERS) $(PKGDIR)/include
	mv $(PKGDIR)/include/mobotsim.h $(PKGDIR)/include/mobot.h
	mv $(PKGDIR)/include/linkbotsim.h $(PKGDIR)/include/linkbot.h
	mkdir $(PKGDIR)/lib
	cp chmobotsim.chf $(PKGDIR)/lib
	cp chlinkbotisim.chf $(PKGDIR)/lib
	cp chlinkbotlsim.chf $(PKGDIR)/lib
	cp chlinkbottsim.chf $(PKGDIR)/lib
	cp delay.chf $(PKGDIR)/lib
	cp systemTime.chf $(PKGDIR)/lib
	cp Makefile $(PACKAGE)-$(VERSION)
	cp pkginstall.ch $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	echo Done Building $(PACKAGE)-$(VERSION)

cleanpkg:
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip

clean:
	rm *.o *.dl

