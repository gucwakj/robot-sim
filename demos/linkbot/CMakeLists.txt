cmake_minimum_required(VERSION 2.8)
project(CRobotSim_Linkbot_Demos)

# Set the version number
set (CRobotSim_Linkbot_Demos_VERSION_MAJOR 0)
set (CRobotSim_Linkbot_Demos_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set up a configure file
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add ODE headers
include_directories("../../src/opende/include")

# Add tinyxml2 headers
include_directories("../../src/tinyxml2")

# Add OSG headers
option (ENABLE_GRAPHICS "Enable graphics for simulation." ON)
if (ENABLE_GRAPHICS)
	include_directories("../../src/openscenegraph/include")
	include_directories("../../src/openscenegraph/build/include")
endif (ENABLE_GRAPHICS)

# Compilation options
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add crobotsim library subdirectory
include_directories("../../src/librobosim")
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
add_executable(recorddistances recorddistances.cpp)
target_link_libraries(recorddistances ${EXTRA_LIBS})
add_executable(group group.cpp)
target_link_libraries(group ${EXTRA_LIBS})
