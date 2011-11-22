cmake_minimum_required(VERSION 2.8)
project(CiMobot_FD_test)

# Set the version number
set(CiMobot_FD_test_VERSION_MAJOR 0)
set(CiMobot_FD_test_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotSim library subdirectory
include_directories("../CiMobotFD")
link_directories("../CiMobotFD")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotfd)

# Add ODE library
include_directories("../opende/sys/include")
link_directories("../opende/sys/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

# Compilation options
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

option (ENABLE_GPROF "Enable GNU Profiling capabilities for simulation." OFF)
if (ENABLE_GPROF)
  add_definitions(-Dpg)
  set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -pg)
endif (ENABLE_GPROF)

option (ENABLE_DRAWSTUFF "Enable usage of the ODE drawstuff routines for visualization." OFF)
if (ENABLE_DRAWSTUFF)
  include_directories("../opende/include")
  link_directories("../opende/drawstuff/src/.libs")
  find_package(GLUT)
  find_package(OpenGL)
  set (EXTRA_LIBS ${EXTRA_LIBS} drawstuff ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif (ENABLE_DRAWSTUFF)

# Add the executable
add_executable(cimobotfd_test cimobotfd_test.cpp)
target_link_libraries(cimobotfd_test ${EXTRA_LIBS})
