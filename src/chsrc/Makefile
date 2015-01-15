#
# chrobosim
#

PACKAGE = chrobosim
VERSION = linux-1.7.3
PKGDIR = $(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC = ../librobosim
HEADERS = $(SRC)/robosim.h $(SRC)/mobot.h $(SRC)/linkbot.h $(SRC)/graphics.h $(SRC)/base.h $(SRC)/macros.h
CPPFLAGS = -DdDOUBLE -fPIC
INCLUDES = -I../librobosim/build/ -I../opende/sys/include/ -I../openscenegraph/include/ -I../openscenegraph/build/include/ -I../tinyxml2/
LIBPATHS = -L../opende/sys/lib/ -L../openscenegraph/build/lib/ -L../tinyxml2/build/
LIBS = -losg -losgViewer -losgUtil -lOpenThreads -losgGA -losgDB -losgFX -losgShadow -lode -ltinyxml2

target: librobosim.dl

OBJS =	robosim.o \
		base.o \
		graphics.o \
		linkbot.o \
		linkbotgroup.o \
		mobot.o \
		mobotgroup.o \
		rgbhashtable.o \
		chlinkboti_chdl.o \
		chlinkbotl_chdl.o \
		chlinkbott_chdl.o \
		chmobot_chdl.o

robosim.o: $(SRC)/robosim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/robosim.cpp $(CPPFLAGS) $(INCLUDES)

base.o: $(SRC)/base.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/base.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/graphics.cpp $(CPPFLAGS) $(INCLUDES)

linkbot.o: $(SRC)/linkbot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbot.cpp $(CPPFLAGS) $(INCLUDES)

linkbotgroup.o: $(SRC)/linkbotgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbotgroup.cpp $(CPPFLAGS) $(INCLUDES)

mobot.o: $(SRC)/mobot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobot.cpp $(CPPFLAGS) $(INCLUDES)

mobotgroup.o: $(SRC)/mobotgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobotgroup.cpp $(CPPFLAGS) $(INCLUDES)

rgbhashtable.o: $(SRC)/rgbhashtable.c $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/rgbhashtable.c $(CPPFLAGS) $(INCLUDES)

chlinkboti_chdl.o: chlinkboti_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkboti_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbotl_chdl.o: chlinkbotl_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkbotl_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chlinkbott_chdl.o: chlinkbott_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chlinkbott_chdl.cpp $(CPPFLAGS) $(INCLUDES)

chmobot_chdl.o: chmobot_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus chmobot_chdl.cpp $(CPPFLAGS) $(INCLUDES)

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
	cp ../../COPYRIGHT $(PKGDIR)
	cp ../../CHANGELOG $(PKGDIR)
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
	cp ../../CHANGELOG $(PKGDIR)/bin/gui
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
	cp mobot.h $(PKGDIR)/include
	cp linkbot.h $(PKGDIR)/include
	cp $(SRC)/macros.h $(PKGDIR)/include
	mkdir $(PKGDIR)/lib
	cp chmobot.chf $(PKGDIR)/lib
	cp chlinkboti.chf $(PKGDIR)/lib
	cp chlinkbotl.chf $(PKGDIR)/lib
	cp chlinkbott.chf $(PKGDIR)/lib
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

