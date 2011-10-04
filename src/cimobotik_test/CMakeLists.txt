cmake_minimum_required(VERSION 2.6)
project(CiMobot_IK_test)

set(CiMobot_IK_test_VERSION_MAJOR 0)
set(CiMobot_IK_test_VERSION_MINOR 1)

configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

# Add CiMobotIK library subdirectory
include_directories("../CiMobotIK")
link_directories("../CiMobotIK")
set (EXTRA_LIBS ${EXTRA_LIBS} imobotik)

option(ENABLE_GRAPHICS "Enable OpenGL for visualization." OFF)
if (ENABLE_GRAPHICS)
	include_directories("/usr/include/")
	link_directories("/usr/lib/")
	find_package(GLUT)
	find_package(OpenGL)
	set (EXTRA_LIBS ${EXTRA_LIBS} glui)
	set (EXTRA_LIBS ${EXTRA_LIBS} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif (ENABLE_GRAPHICS)

message(${EXTRA_LIBS})

add_executable(cimobotik_test cimobotik_test.cpp)
target_link_libraries(cimobotik_test ${EXTRA_LIBS})