cmake_minimum_required(VERSION 2.8)
project(CRobotSim)

# Set the version number
set(CRobotSim_VERSION_MAJOR 0)
set(CRobotSim_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Verbose ouptut
set(CMAKE_VERBOSE_MAKEFILE off)

# Set up a configure file
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add ODE library
include_directories("../opende/sys/include")
link_directories("../opende/sys/lib")

# Add tinyxml2 library
include_directories("../tinyxml2")
link_directories("../tinyxml2")

# Option to enable double precision
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
	add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
	add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Option to enable graphics
option (ENABLE_GRAPHICS "Enable graphics for simulation." ON)
if (ENABLE_GRAPHICS)
	# Add OSG
	find_package(OpenSceneGraph 2.8.0 REQUIRED osgViewer osgSim osgUtil OpenThreads osgGA osgDB osgFX)
	if (OPENSCENEGRAPH_FOUND)
		include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
		set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENSCENEGRAPH_LIBRARIES})
	endif (OPENSCENEGRAPH_FOUND)
	# Add openGL
	find_package(OpenGL REQUIRED)
	if (OPENGL_FOUND)
		include_directories(${OPENGL_INCLUDE_DIR})
		set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES})
	endif (OPENGL_FOUND)
endif (ENABLE_GRAPHICS)

# Add pthreads
find_package(Threads)
set (EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# Add the library
if (ENABLE_GRAPHICS)
add_library( robotsim SHARED
	robot.cpp
	robotsim.cpp
	graphics.cpp
	mobotsim.cpp
	pid.cpp
)
else (ENABLE_GRAPHICS)
add_library( robotsim SHARED
	robot.cpp
	robotsim.cpp
	mobotsim.cpp
	pid.cpp
)
endif (ENABLE_GRAPHICS)

target_link_libraries(robotsim ode ${EXTRA_LIBS} rt tinyxml2)
