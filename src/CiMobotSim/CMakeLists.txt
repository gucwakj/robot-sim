cmake_minimum_required(VERSION 2.6)
project(CiMobot_Sim)

# Set the version number
set(CiMobot_Sim_VERSION_MAJOR 0)
set(CiMobot_Sim_VERSION_MINOR 1)

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

option (ENABLE_DRAWSTUFF "Enable usage of the ODE drawstuff routines for visualization." ON)
if (ENABLE_DRAWSTUFF)
  include_directories("../opende/include")
endif (ENABLE_DRAWSTUFF)

option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
	add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
	add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add the library
add_library( imobotsim SHARED
	imobotsim.cpp
	pid.cpp
)
