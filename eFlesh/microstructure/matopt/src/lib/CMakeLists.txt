add_library(matopt_lib
	material_optimization.cpp
	material_optimization.h
	material_optimization_ipopt.cpp
	material_optimization_ipopt.h
	material_optimization_gd.cpp
	material_optimization_gd.h
	material_optimization_lbfgs.cpp
	material_optimization_lbfgs.h
	logger.cpp
	logger.h
	convert.cpp
	convert.h
	polygons.cpp
	polygons.h
	mesh_utils.cpp
	mesh_utils.h
	materials_ext.cpp
	materials_ext.h
)

add_library(matopt::lib ALIAS matopt_lib)

target_include_directories(matopt_lib PUBLIC .)

target_link_libraries(matopt_lib PRIVATE warnings::all ceres::ceres PUBLIC Eigen3::Eigen json::json matopt::libigl ipopt::ipopt spdlog::spdlog geo::predicates MeshFEM matopt::libigl matopt::clipper matopt_lbfgspp)

# Use C++14
target_compile_features(matopt_lib PUBLIC cxx_std_14)

# Generate position independent code
set_target_properties(matopt_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

