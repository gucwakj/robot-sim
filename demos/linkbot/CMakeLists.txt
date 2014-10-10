# build linkbot demos
project(linkbotdemos)

# set minimum cmake version
cmake_minimum_required(VERSION 2.8)

# supress developer warnings
cmake_policy(SET CMP0015 NEW)

# verbose ouptut
set(CMAKE_VERBOSE_MAKEFILE off)

# set build type
set(CMAKE_BUILD_TYPE "Release")

# set developer warnings
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(MSVC)
	set(CMAKE_CXX_FLAGS "-Wno-write-strings")
endif(MSVC)

# include directories
include_directories(
	"${linkbotdemos_SOURCE_DIR}"
	"${linkbotdemos_BINARY_DIR}"
)

# set up a configure file
configure_file(
	"${linkbotdemos_SOURCE_DIR}/config.h.in"
	"${linkbotdemos_BINARY_DIR}/config.h"
)

# double precision
add_definitions(-DdDOUBLE)

# include directories
include_directories("../../src/opende/include")
include_directories("../../src/tinyxml2")

option(ENABLE_GRAPHICS "Enable graphics for simulation." ON)
if(ENABLE_GRAPHICS)
	include_directories("../../src/osg/include")
	include_directories("../../src/osg/build/include")
endif(ENABLE_GRAPHICS)

# add librobosim library subdirectory
include_directories("../../src/librobosim/inc")
include_directories("../../src/librobosim/tpl")
if (MSVC)
	link_directories("../../src/librobosim/build/Debug")
	link_directories("../../src/librobosim/build/Release")
else (MSVC)
	link_directories("../../src/librobosim/build")
endif (MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized robosim debug robosimd)

# linkbot demos
add_executable(test test.cpp)
target_link_libraries(test ${EXTRA_LIBS})

