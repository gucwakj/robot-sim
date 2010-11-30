cmake_minimum_required (VERSION 2.6)
project (iMobot_Sim)

# Set the version number
set (iMobot_Sim_VERSION_MAJOR 0)
set (iMobot_Sim_VERSION_MINOR 1)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "#{PROJECT_BINARY_DIR}/config.h"
  )

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add include directory for ODE
include_directories("../opende/include")

# Add the executable
add_executable(imobot_simulation imobot_simulation.cpp)
