# build cubus demos
project(cubusdemos)

# set minimum cmake version
cmake_minimum_required(VERSION 2.8)

# set the version number
set(cubusdemos_VERSION_MAJOR 0)
set(cubusdemos_VERSION_MINOR 1)
set(cubusdemos_VERSION_PATCH 0)

# supress developer warnings
cmake_policy(SET CMP0015 NEW)

# verbose ouptut
set(CMAKE_VERBOSE_MAKEFILE off)

# set build type
set(CMAKE_BUILD_TYPE "Release")

# set developer warnings
set(CMAKE_CXX_FLAGS "-Wno-write-strings")

# include directories
include_directories(
	"${cubusdemos_SOURCE_DIR}"
	"${cubusdemos_BINARY_DIR}"
)

# set up a configure file
configure_file(
	"${cubusdemos_SOURCE_DIR}/config.h.in"
	"${cubusdemos_BINARY_DIR}/config.h"
)

# disable windows warnings
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)

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

# cubus demos
add_executable(test test.cpp)
target_link_libraries(test ${EXTRA_LIBS})

