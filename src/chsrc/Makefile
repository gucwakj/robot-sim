#
# chrobosim
#

PACKAGE = chrobosim
VERSION = linux-1.7.70
PKGDIR = $(PACKAGE)-$(VERSION)/$(PACKAGE)

SRC = ../librobosim/src
INC = ../librobosim/inc
CHDL = ./chdl
HEADERS = $(INC)/macros.hpp $(INC)/robosim.hpp $(INC)/graphics.hpp $(INC)/robot.hpp $(INC)/modularrobot.hpp $(INC)/linkbot.hpp $(INC)/mobot.hpp $(INC)/nxt.hpp
CPPFLAGS = -DdDOUBLE -fPIC -Wno-write-strings
INCLUDES = -I../librobosim/inc/ -I../librobosim/build/ -I../librobosim/tpl/ -I../opende/sys/include/ -I../osg/include/ -I../osg/build/include/ -I../tinyxml2/
LIBPATHS = -L../opende/sys/lib/ -L../osg/build/lib/ -L../tinyxml2/build/
LIBS = -losg -losgViewer -losgUtil -lOpenThreads -losgGA -losgDB -losgShadow -lode -ltinyxml2

target: librobosim.dl

OBJS =	robosim.o \
		graphics.o \
		robot.o \
		modularrobot.o \
		linkbot.o \
		mobot.o \
		nxt.o \
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

modularrobot.o: $(SRC)/modularrobot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/modularrobot.cpp $(CPPFLAGS) $(INCLUDES)

linkbot.o: $(SRC)/linkbot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/linkbot.cpp $(CPPFLAGS) $(INCLUDES)

mobot.o: $(SRC)/mobot.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/mobot.cpp $(CPPFLAGS) $(INCLUDES)

nxt.o: $(SRC)/nxt.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/nxt.cpp $(CPPFLAGS) $(INCLUDES)

rgbhashtable.o: $(SRC)/rgbhashtable.c $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(SRC)/rgbhashtable.c $(CPPFLAGS) $(INCLUDES)

robotgroup_chdl.o: $(CHDL)/robotgroup_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(CHDL)/robotgroup_chdl.cpp $(CPPFLAGS) $(INCLUDES)

clinkboti_chdl.o: $(CHDL)/clinkboti_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(CHDL)/clinkboti_chdl.cpp $(CPPFLAGS) $(INCLUDES)

clinkbotl_chdl.o: $(CHDL)/clinkbotl_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(CHDL)/clinkbotl_chdl.cpp $(CPPFLAGS) $(INCLUDES)

cmobot_chdl.o: $(CHDL)/cmobot_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(CHDL)/cmobot_chdl.cpp $(CPPFLAGS) $(INCLUDES)

cnxt_chdl.o: $(CHDL)/cnxt_chdl.cpp $(HEADERS)
	ch dlcomp librobosim.dl cplusplus $(CHDL)/cnxt_chdl.cpp $(CPPFLAGS) $(INCLUDES)

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
	cp ./include/robot.h $(PKGDIR)/include
	cp ./include/linkbot.h $(PKGDIR)/include
	cp ./include/mobot.h $(PKGDIR)/include
	cp ./include/nxt.h $(PKGDIR)/include
	cp $(INC)/macros.hpp $(PKGDIR)/include
	mkdir $(PKGDIR)/lib
	cp ./chf/robotgroup.chf $(PKGDIR)/lib
	cp ./chf/clinkboti.chf $(PKGDIR)/lib
	cp ./chf/clinkbotl.chf $(PKGDIR)/lib
	cp ./chf/cmobot.chf $(PKGDIR)/lib
	cp ./chf/cnxt.chf $(PKGDIR)/lib
	cp ./chf/delay.chf $(PKGDIR)/lib
	cp ./chf/systemTime.chf $(PKGDIR)/lib
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

