cmake_minimum_required(VERSION 2.6)
project(CiMobot_IK)

set(CiMobot_IK_VERSION_MAJOR 0)
set(CiMobot_IK_VERSION_MINOR 1)

configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

include_directories("${PROJECT_BINARY_DIR}")

add_library( imobotik STATIC
	imobotik.cpp
	jacobian.cpp
	linearR3.cpp
	matrixRmn.cpp
	node.cpp
	tree.cpp
	vectorRn.cpp
)