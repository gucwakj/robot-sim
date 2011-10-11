cmake_minimum_required(VERSION 2.6)
project(CiMobot_IK_test)

set(CiMobot_IK_test_VERSION_MAJOR 0)
set(CiMobot_IK_test_VERSION_MINOR 1)
cmake_policy(SET CMP0015 NEW)

configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

include_directories("../CiMobotIK")
link_directories("../CiMobotIK")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotik)

add_executable(cimobotik_test cimobotik_test.cpp)
target_link_libraries(cimobotik_test ${EXTRA_LIBS})