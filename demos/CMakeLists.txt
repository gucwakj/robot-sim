cmake_minimum_required (VERSION 2.8)
project (CiMobotSim_Demos)

# Set the version number
set (CiMobotSim_Demos_VERSION_MAJOR 0)
set (CiMobotSim_Demos_VERSION_MINOR 1)

# Set up a configure file
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
)

# Compilation options
option (ENABLE_DRAWSTUFF "Enable usage of the ODE drawstuff routines for visualization." ON)

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

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotSim library subdirectory and stuff
include_directories("../src/CiMobotSim")
link_directories("../src/CiMobotSim")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotsim)

# Add ODE library
include_directories("../src/opende/sys/include")
link_directories("../src/opende/sys/lib")
set (EXTRA_LIBS ${EXTRA_LIBS} ode)

if (ENABLE_DRAWSTUFF)
  include_directories("../src/opende/include")
  link_directories("../src/opende/drawstuff/src/.libs")
  find_package(GLUT)
  find_package(OpenGL)
  set (EXTRA_LIBS ${EXTRA_LIBS} drawstuff ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif (ENABLE_DRAWSTUFF)

# Add the executable
add_executable(inchworm inchworm.cpp)
target_link_libraries(inchworm ${EXTRA_LIBS})
add_executable(camera_stand camera_stand.cpp)
target_link_libraries(camera_stand ${EXTRA_LIBS})
