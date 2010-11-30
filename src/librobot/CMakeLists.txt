# The ODE Library
cmake_minimum_required(VERSION 2.8)
include_directories(../opende/sys/include)
add_definitions(-DdSINGLE)


add_library(robot
  librobot.cpp
  )
