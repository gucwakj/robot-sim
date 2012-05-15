cmake_minimum_required(VERSION 2.8)
project(CMobot_IK)

# Set the version number
set(CMobotIK_VERSION_MAJOR 0)
set(CMobotIK_VERSION_MINOR 1)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set up a configure file
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Option to enable imobot simulation
option (ENABLE_IMOBOT "Use iMobot properties for simulation." OFF)

# Option to enable imobot simulation
option (ENABLE_MOBOT "Use Mobot properties for simulation." ON)

# Add the library
add_library( mobotik SHARED
	mobotik.cpp
	jacobian.cpp
	matrixR33.cpp
	matrixRmn.cpp
	node.cpp
	tree.cpp
	vectorR3.cpp
	vectorRn.cpp
)
