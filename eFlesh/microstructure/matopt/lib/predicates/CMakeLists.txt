cmake_minimum_required(VERSION 3.1)
project(predicates_psm)

add_library(geo_predicates
	STATIC
		predicates_psm.cpp
		predicates_psm.h
)
add_library(geo::predicates ALIAS geo_predicates)

target_include_directories(geo_predicates PUBLIC .)

# Use C++11
target_compile_features(geo_predicates PUBLIC cxx_std_11)

# Generate position independent code
set_target_properties(geo_predicates PROPERTIES POSITION_INDEPENDENT_CODE ON)
