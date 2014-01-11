# Set the name and the supported language of the project
project(robosim)

# Set the minimum version of cmake required to build this project
cmake_minimum_required(VERSION 2.8)

# Suppress developer warnings
cmake_policy(SET CMP0015 NEW)

# Set developer warnings
set(CMAKE_CXX_FLAGS "-Wall")

# Disable windows warnings
if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif (MSVC)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add tinyxml2 library
include_directories("../tinyxml2")
if (MSVC)
	link_directories("dll")
else (MSVC)
	link_directories("../tinyxml2/build")
endif (MSVC)
set(EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# Use the package PkgConfig to detect GTK+ headers/library files
if (MSVC)
	include_directories("C:/gtk/include/gtk-2.0" "C:/gtk/lib/gtk-2.0/include" "C:/gtk/include/atk-1.0" "C:/gtk/include/cairo" "C:/gtk/include/gdk-pixbuf-2.0" "C:/gtk/include/pango-1.0" "C:/gtk/include/glib-2.0" "C:/gtk/lib/glib-2.0/include" "C:/gtk/include/freetype2" "C:/gtk/include/libpng14")
	link_directories("C:/gtk/lib")
	set(EXTRA_LIBS ${EXTRA_LIBS} gtk-win32-2.0 gdk-win32-2.0 atk-1.0 pangowin32-1.0 gdi32 pangocairo-1.0 gdk_pixbuf-2.0 pango-1.0 cairo gobject-2.0 gmodule-2.0 gthread-2.0 glib-2.0 intl)
else (MSVC)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
	include_directories(${GTK3_INCLUDE_DIRS})
	link_directories(${GTK3_LIBRARY_DIRS})
	set(EXTRA_LIBS ${EXTRA_LIBS} ${GTK3_LIBRARIES})
	add_definitions(${GTK3_CFLAGS_OTHER})
endif (MSVC)

# Add an executable compiled from hello.c
if (MSVC)
add_executable(robosim WIN32
	main.cpp
	RoboSim.rc
)
else (MSVC)
add_executable(robosim
	main.cpp
)
endif (MSVC)

# Link the target to the GTK+ libraries
target_link_libraries(robosim ${EXTRA_LIBS})
