cmake_minimum_required (VERSION 2.6)
project (iMobot_Sim)

# Set the version number
set (iMobot_Sim_VERSION_MAJOR 0)
set (iMobot_Sim_VERSION_MINOR 1)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# Set up defines
add_definitions(-DdSINGLE)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add ODE library
include_directories("opende/sys/include")
link_directories("opende/sys/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

# Add librobot subdirectory and stuff
include_directories("librobot")
add_subdirectory("librobot")
set (EXTRA_LIBS ${EXTRA_LIBS} robot)

# Add ODE library

# Add the executable
add_executable(imobot_simulation imobot_simulation.cpp)
target_link_libraries(imobot_simulation ${EXTRA_LIBS})
