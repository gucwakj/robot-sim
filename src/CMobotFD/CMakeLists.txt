cmake_minimum_required(VERSION 2.8)
project(CMobotFD)

# Set the version number
set(CMobotFD_VERSION_MAJOR 0)
set(CMobotFD_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

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

# Option to enable double precision
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
	add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
	add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add pthreads
find_package(Threads)
set (EXTRA_LIBS ${EXTRA_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# Add the library
add_library( mobotfd SHARED
	mobotfd.cpp
	mobot.cpp
	pid.cpp
	threads.cpp
)
target_link_libraries(mobotfd ode ${EXTRA_LIBS} rt)
