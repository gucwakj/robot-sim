cmake_minimum_required (VERSION 2.6)
project (CiMobot_Sim)

# Set the version number
set (CiMobot_Sim_VERSION_MAJOR 0)
set (CiMobot_Sim_VERSION_MINOR 1)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# Compilation options
option (ENABLE_DRAWSTUFF 
        "Enable usage of the ODE drawstuff routines for visualization." OFF)

option (ENABLE_DOUBLE
        "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotSim library subdirectory and stuff
include_directories("../CiMobotSim")
link_directories("../CiMobotSim")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotsim)

# Add ODE library
include_directories(
  "../opende/sys/include"
  )
link_directories("../opende/sys/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

# Add librobot subdirectory and stuff
include_directories("../librobot")
link_directories("../librobot")
set (EXTRA_LIBS ${EXTRA_LIBS} robot)


# If drawstuff is enabled, include drawstuff library
if (ENABLE_DRAWSTUFF)
  include_directories("../opende/include")
  link_directories("../opende/drawstuff/src/.libs")
  find_package(GLUT)
  find_package(OpenGL)
  set (EXTRA_LIBS ${EXTRA_LIBS} drawstuff ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif (ENABLE_DRAWSTUFF)

message( ${EXTRA_LIBS} )

# Add the executable
add_executable(cimobotsim_test cimobotsim_test.cpp)
target_link_libraries(cimobotsim_test ${EXTRA_LIBS})
