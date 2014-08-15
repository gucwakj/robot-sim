#
# chrobosim
#

PACKAGE = chrobosim
VERSION = linux-1.7.70
PKGDIR = $(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC = ../librobosim
HEADERS = $(SRC)/macros.h $(SRC)/robosim.h $(SRC)/graphics.h $(SRC)/robot.h $(SRC)/modularrobot.h $(SRC)/mobot.h $(SRC)/linkbot.h $(SRC)/nxt.h
CPPFLAGS = -DdDOUBLE -fPIC
INCLUDES = -I../librobosim/build/ -I../opende/sys/include/ -I../osg/include/ -I../osg/build/include/ -I../tinyxml2/
LIBPATHS = -L../opende/sys/lib/ -L../osg/build/lib/ -L../tinyxml2/build/
LIBS = -losg -losgViewer -losgUtil -lOpenThreads -losgGA -losgDB -losgShadow -lode -ltinyxml2

target: librobosim.dl

OBJS =	robosim.o \
		graphics.o \
		robot.o \
		robotgroup.o \
		linkbot.o \
		linkbotgroup.o \
		mobot.o \
		mobotgroup.o \
		nxt.o \
		nxtgroup.o \
		rgbhashtable.o \
		robotgroup_chdl.o \
		clinkboti_chdl.o \
		clinkbotl_chdl.o \
		cmobot_chdl.o \
		cnxt_chdl.o

robosim.o: $(SRC)/robosim.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/robosim.cpp $(CPPFLAGS) $(INCLUDES)

graphics.o: $(SRC)/graphics.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/graphics.cpp $(CPPFLAGS) $(INCLUDES)

robot.o: $(SRC)/robot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/robot.cpp $(CPPFLAGS) $(INCLUDES)

robotgroup.o: $(SRC)/robotgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/robotgroup.cpp $(CPPFLAGS) $(INCLUDES)

linkbot.o: $(SRC)/linkbot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbot.cpp $(CPPFLAGS) $(INCLUDES)

linkbotgroup.o: $(SRC)/linkbotgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbotgroup.cpp $(CPPFLAGS) $(INCLUDES)

mobot.o: $(SRC)/mobot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobot.cpp $(CPPFLAGS) $(INCLUDES)

mobotgroup.o: $(SRC)/mobotgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobotgroup.cpp $(CPPFLAGS) $(INCLUDES)

nxt.o: $(SRC)/nxt.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/nxt.cpp $(CPPFLAGS) $(INCLUDES)

nxtgroup.o: $(SRC)/nxtgroup.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/nxtgroup.cpp $(CPPFLAGS) $(INCLUDES)

rgbhashtable.o: $(SRC)/rgbhashtable.c $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/rgbhashtable.c $(CPPFLAGS) $(INCLUDES)

robotgroup_chdl.o: robotgroup_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus robotgroup_chdl.cpp $(CPPFLAGS) $(INCLUDES)

clinkboti_chdl.o: clinkboti_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus clinkboti_chdl.cpp $(CPPFLAGS) $(INCLUDES)

clinkbotl_chdl.o: clinkbotl_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus clinkbotl_chdl.cpp $(CPPFLAGS) $(INCLUDES)

cmobot_chdl.o: cmobot_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus cmobot_chdl.cpp $(CPPFLAGS) $(INCLUDES)

cnxt_chdl.o: cnxt_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus cnxt_chdl.cpp $(CPPFLAGS) $(INCLUDES)

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
	cp ../configurator/build/robosim $(PKGDIR)/bin
	cp ../configurator/interface3.glade $(PKGDIR)/bin/interface.glade
	cp ../../CHANGELOG $(PKGDIR)/bin
	cp -R ../configurator/images $(PKGDIR)/bin
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
	cp robot.h $(PKGDIR)/include
	cp linkbot.h $(PKGDIR)/include
	cp mobot.h $(PKGDIR)/include
	cp nxt.h $(PKGDIR)/include
	cp $(SRC)/macros.h $(PKGDIR)/include
	mkdir $(PKGDIR)/lib
	cp robotgroup.chf $(PKGDIR)/lib
	cp clinkboti.chf $(PKGDIR)/lib
	cp clinkbotl.chf $(PKGDIR)/lib
	cp cmobot.chf $(PKGDIR)/lib
	cp cnxt.chf $(PKGDIR)/lib
	cp delay.chf $(PKGDIR)/lib
	cp systemTime.chf $(PKGDIR)/lib
	mkdir $(PKGDIR)/so
	cp ../opende/sys/lib/libode.so.1 $(PKGDIR)/so
	cp ../osg/build/lib/libosg.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgDB.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgGA.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgShadow.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgText.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgUtil.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libosgViewer.so.100 $(PKGDIR)/so
	cp ../osg/build/lib/libOpenThreads.so.13 $(PKGDIR)/so
	cp ../tinyxml2/build/libtinyxml2.so.1 $(PKGDIR)/so
	mkdir $(PKGDIR)/so/osgPlugins-3.2.0
	cp ../osg/build/lib/osgPlugins-3.2.0/osgdb_png.so $(PKGDIR)/so/osgPlugins-3.2.0
	cp ../osg/build/lib/osgPlugins-3.2.0/osgdb_3ds.so $(PKGDIR)/so/osgPlugins-3.2.0
	cp ../osg/build/lib/osgPlugins-3.2.0/osgdb_tga.so $(PKGDIR)/so/osgPlugins-3.2.0
	cp Makefile $(PACKAGE)-$(VERSION)
	cp pkginstall.ch $(PACKAGE)-$(VERSION)
	zip -rq $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION)
	echo Done Building $(PACKAGE)-$(VERSION)

cleanpkg:
	rm -rf $(PACKAGE)-$(VERSION)
	rm -rf $(PACKAGE)-$(VERSION).zip

clean:
	rm *.o *.dl

