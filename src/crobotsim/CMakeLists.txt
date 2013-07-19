# Build robotsim simulator
project(robosim)

# Set minimum cmake version
cmake_minimum_required(VERSION 2.8)

# Set the version number
set(ROBOSIM_VERSION_MAJOR 0)
set(ROBOSIM_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Verbose ouptut
set(CMAKE_VERBOSE_MAKEFILE off)

# Set build type
set(CMAKE_BUILD_TYPE "Release")

# Set up a configure file
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

# Disable windows warnings
if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif (MSVC)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add tinyxml2 library
include_directories("../tinyxml2")
if (MSVC)
	link_directories("../tinyxml2/build/Release")
elseif (MSVC)
	link_directories("../tinyxml2/build")
endif (MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# Add ODE library
include_directories("../opende/include")
if (MSVC)
	link_directories("../opende/lib/Release")
	link_directories("../opende/lib/Debug")
elseif (MSVC)
	link_directories("../opende/lib")
endif (MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} optimized ode debug oded)

# Option to enable double precision
option (ROBOTSIM_ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ROBOTSIM_ENABLE_DOUBLE)
	add_definitions(-DdDOUBLE)
else (ROBOTSIM_ENABLE_DOUBLE)
	add_definitions(-DdSINGLE)
endif (ROBOTSIM_ENABLE_DOUBLE)

# Option to enable graphics
option (ROBOTSIM_ENABLE_GRAPHICS "Enable graphics for simulation." ON)
if (ROBOTSIM_ENABLE_GRAPHICS)
	include_directories("../openscenegraph/build/include")
	include_directories("../openscenegraph/include")
	link_directories("../openscenegraph/build/lib")
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osg debug osgd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgViewer debug osgViewerd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgUtil debug osgUtild)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgGA debug osgGAd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgDB debug osgDBd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized osgText debug osgTextd)
	set(EXTRA_LIBS ${EXTRA_LIBS} optimized OpenThreads debug OpenThreadsd)
	# Add openGL
	find_package(OpenGL REQUIRED)
	if (OPENGL_FOUND)
		include_directories(${OPENGL_INCLUDE_DIR})
		message (status ${OPENGL_LIBRARIES})
		set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES})
	endif (OPENGL_FOUND)
endif (ROBOTSIM_ENABLE_GRAPHICS)

# Add pthreads
find_package(Threads)
set (EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# Set postfix for build types
#set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")
#set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
#set(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add a postfix, usually empty on windows")
#set(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a postfix, usually empty on windows")

# Set the build postfix extension according to what configuration is being built
if (CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_BUILD_POSTFIX "")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set(CMAKE_BUILD_POSTFIX "s")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set(CMAKE_BUILD_POSTFIX "rd")
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_BUILD_POSTFIX "d")
else ()
    set(CMAKE_BUILD_POSTFIX "")
endif ()

# Add the library
if (ROBOTSIM_ENABLE_GRAPHICS)
add_library( robotsim SHARED
	base.cpp
	graphics.cpp
	robotsim.cpp
	mobotsim.cpp
	linkbotsim.cpp
	#pid.cpp
)
else (ROBOTSIM_ENABLE_GRAPHICS)
add_library( robotsim SHARED
	base.cpp
	robotsim.cpp
	mobotsim.cpp
	linkbotsim.cpp
	#pid.cpp
)
endif (ROBOTSIM_ENABLE_GRAPHICS)

# Link library against dependencies
target_link_libraries(robotsim ${EXTRA_LIBS})
