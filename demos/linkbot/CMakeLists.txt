cmake_minimum_required(VERSION 2.8)
project(robosim_linkbot_demos)

# Set the version number
set (ROBOSIM_LINKBOT_DEMOS_VERSION_MAJOR 0)
set (ROBOSIM_LINKBOT_DEMOS_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set developer warnings
set(CMAKE_CXX_FLAGS "-Wall")

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

# Double precision
add_definitions(-DdDOUBLE)

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
add_executable(inchwormConstraintDemo inchwormConstraintDemo.cpp)
target_link_libraries(inchwormConstraintDemo ${EXTRA_LIBS})
add_executable(arm arm.cpp)
target_link_libraries(arm ${EXTRA_LIBS})
