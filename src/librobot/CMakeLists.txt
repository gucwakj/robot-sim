# The ODE Library
cmake_minimum_required(VERSION 2.6)
include_directories(../opende/sys/include)

option (ENABLE_DOUBLE
        "Enable double precision for simulation." ON)

if (ENABLE_DOUBLE)
  add_definitions(-DdDOUBLE)
else ()
  add_definitions(-DdSINGLE)
endif (ENABLE_DOUBLE)

add_library(robot
  librobot.cpp
  )
