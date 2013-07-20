# Set the name and the supported language of the project
project(robosimgui)

# Set the minimum version of cmake required to build this project
cmake_minimum_required(VERSION 2.8)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add tinyxml2 library
include_directories("../tinyxml2")
link_directories("../tinyxml2/build")
set(EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})
set(EXTRA_LIBS ${EXTRA_LIBS} ${GTK3_LIBRARIES})
add_definitions(${GTK3_CFLAGS_OTHER})

# Add an executable compiled from hello.c
add_executable(robosimconfigurator
	main.cpp
)

# Link the target to the GTK+ libraries
target_link_libraries(robosimconfigurator ${EXTRA_LIBS})
