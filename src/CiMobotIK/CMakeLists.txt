cmake_minimum_required(VERSION 2.6)
project(CiMobot_ik)

set(CiMobot_ik_VERSION_MAJOR 0)
set(CiMobot_ik_VERSION_MINOR 1)

configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

option (ENABLE_GRAPHICS "Enable OpenGL for visualization." OFF)
if (ENABLE_GRAPHICS)
	include_directories("/usr/include/")
	link_directories("/usr/lib/")
	find_package(GLUT)
	find_package(OpenGL)
	set (EXTRA_LIBS glui)
	set (EXTRA_LIBS ${EXTRA_LIBS} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif (ENABLE_GRAPHICS)

add_library( imobotik STATIC
	imobotik.cpp
	jacobian.cpp
	linearR3.cpp
	matrixRmn.cpp
	node.cpp
	tree.cpp
	vectorRn.cpp
)
#target_link_libraries(ik_test ${EXTRA_LIBS})