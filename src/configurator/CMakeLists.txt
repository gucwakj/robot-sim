# Set the name and the supported language of the project
project(robosimgui)

# Set the minimum version of cmake required to build this project
cmake_minimum_required(VERSION 2.8)

# Supress developer warnings
cmake_policy(SET CMP0015 NEW)

# Disable windows warnings
if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif (MSVC)

# Add the binary directory to include file search path
include_directories("${PROJECT_BINARY_DIR}")

# Add tinyxml2 library
include_directories("../tinyxml2")
link_directories("../tinyxml2/build")
set(EXTRA_LIBS ${EXTRA_LIBS} tinyxml2)

# Use the package PkgConfig to detect GTK+ headers/library files
if (MSVC)
	include_directories("C:/MinGW/include/gtk-2.0" "C:/MinGW/lib/gtk-2.0/include" "C:/MinGW/include/atk-1.0" "C:/MinGW/include/cairo" "C:/MinGW/include/gdk-pixbuf-2.0" "C:/MinGW/include/pango-1.0" "C:/MinGW/include/glib-2.0" "C:/MinGW/lib/glib-2.0/include" "C:/MinGW/include/freetype2" "C:/MinGW/include/libpng14")
	link_directories("C:/MinGW/lib")
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
add_executable(robosimconfigurator
	main.cpp
)

# Link the target to the GTK+ libraries
target_link_libraries(robosimconfigurator ${EXTRA_LIBS})
