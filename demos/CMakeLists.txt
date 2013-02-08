cmake_minimum_required(VERSION 2.8)
project(CRobotSim_Demos)

# Set the version number
set (CRobotSim_Demos_VERSION_MAJOR 0)
set (CRobotSim_Demos_VERSION_MINOR 1)

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
include_directories("../src/opende/sys/include")

# Compilation options
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add CRobotSim library subdirectory
include_directories("../src/CRobotSim")
link_directories("../src/CRobotSim")

# Add the executable
add_executable(dataAcquisition dataAcquisition.cpp)
target_link_libraries(dataAcquisition robotsim)
add_executable(liftRecorded liftRecorded.cpp)
target_link_libraries(liftRecorded robotsim)
