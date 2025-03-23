cmake_minimum_required(VERSION 3.5)

project(robotic_cpp_test)

#set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")

add_compile_options(
  -O3 -g -Wall -Wpedantic -std=c++14
  )

################################################################################

find_package(Python REQUIRED)

message(STATUS "[rai] python user site: " ${Python_SITELIB})

include_directories(
  /home/shiping/pyvenv/lib/python3.12/site-packages/robotic/include/rai
  $ENV{HOME}/.local/include
  )

link_directories(
  /home/shiping/pyvenv/lib/python3.12/site-packages/robotic
  $ENV{HOME}/.local/lib
  )

################################################################################

add_executable(a.out main.cpp)
target_link_libraries(a.out 
                      rai)