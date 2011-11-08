cmake_minimum_required(VERSION 2.6)
project(CiMobot_IK_test)

# Set the version number
set(CiMobot_IK_test_VERSION_MAJOR 0)
set(CiMobot_IK_test_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set up a configure file
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotIK library subdirectory
include_directories("../CiMobotIK")
link_directories("../CiMobotIK")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotik)

# Add the executable
add_executable(cimobotik_test cimobotik_test.cpp)
target_link_libraries(cimobotik_test ${EXTRA_LIBS})
