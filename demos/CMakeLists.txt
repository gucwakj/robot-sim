cmake_minimum_required(VERSION 2.8)
project(IRSE_Demos)

# Set the version number
set (IRSE_Demos_VERSION_MAJOR 0)
set (IRSE_Demos_VERSION_MINOR 1)

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

# Add IRSE library subdirectory
include_directories("../src/IRSE")
link_directories("../src/IRSE")

# Add the executable
add_executable(dataAcquisition dataAcquisition.cpp)
target_link_libraries(dataAcquisition irse)
add_executable(liftRecorded liftRecorded.cpp)
target_link_libraries(liftRecorded irse)
