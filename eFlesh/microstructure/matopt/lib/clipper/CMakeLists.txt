cmake_minimum_required(VERSION 3.1)
project(polyclipping)

add_library(matopt_clipper matopt_clipper/clipper.cpp)
add_library(matopt::clipper ALIAS matopt_clipper)

target_include_directories(matopt_clipper PUBLIC .)

# Generate position independent code
set_target_properties(matopt_clipper PROPERTIES POSITION_INDEPENDENT_CODE ON)
