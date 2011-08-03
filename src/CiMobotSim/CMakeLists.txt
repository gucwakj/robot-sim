# The ODE Library
cmake_minimum_required(VERSION 2.6)
include_directories(../opende/sys/include)

# Set up a configure file
configure_file (
  "config.h.in"
  "config.h"
  )

option (ENABLE_DRAWSTUFF 
        "Enable usage of the ODE drawstuff routines for visualization." OFF)
option (ENABLE_DOUBLE
        "Enable double precision for simulation." ON)

if (ENABLE_DRAWSTUFF)
  include_directories("../opende/include")
endif (ENABLE_DRAWSTUFF)

if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else (ENABLE_DOUBLE)
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

add_library(imobotsim
  imobotsim.cpp
  pid.cpp
  )
