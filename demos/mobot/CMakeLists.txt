cmake_minimum_required(VERSION 2.8)
project(CRobotSim_Mobot_Demos)

# Set the version number
set (CROBOTSIM_MOBOT_DEMOS_VERSION_MAJOR 0)
set (CROBOTSIM_MOBOT_DEMOS_VERSION_MINOR 1)

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
include_directories("../../src/openscenegraph/include")
include_directories("../../src/openscenegraph/build/include")

# Compilation options
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add crobotsim library subdirectory
include_directories("../../src/crobotsim")
link_directories("../../src/crobotsim/build/Debug")
link_directories("../../src/crobotsim/build/Release")
set(EXTRA_LIBS ${EXTRA_LIBS} optimized robotsim debug robotsimd)

# Add cmobotik library subdirectory
#include_directories("../../src/CMobotIK")
#link_directories("../../src/CMobotIK")

# crobotsim mobot demos
add_executable(mtest test.cpp)
target_link_libraries(mtest ${EXTRA_LIBS})
add_executable(dataAcquisition dataAcquisition.cpp)
target_link_libraries(dataAcquisition ${EXTRA_LIBS})
add_executable(liftRecorded liftRecorded.cpp)
target_link_libraries(liftRecorded ${EXTRA_LIBS})