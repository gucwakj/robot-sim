cmake_minimum_required(VERSION 2.8)
project(IRSE_tests)

# Set the version number
set(CMobotIK_test_VERSION_MAJOR 0)
set(CMobotIK_test_VERSION_MINOR 1)
set(IRSE_test_VERSION_MAJOR 0)
set(IRSE_test_VERSION_MINOR 1)

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

# Option to enable double precision
option (ENABLE_DOUBLE "Enable double precision for simulation." ON)
if (ENABLE_DOUBLE)
	add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
	add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

# Add CMobotIK library subdirectory
include_directories("../CMobotIK")
link_directories("../CMobotIK")

# Add IRSE library subdirectory
include_directories("../IRSE")
link_directories("../IRSE")

# Add the executable
add_executable(ik ik.cpp)
target_link_libraries(ik mobotik)

# Add the executable
add_executable(fd fd.cpp)
target_link_libraries(fd irse)
