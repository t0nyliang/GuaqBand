#pragma once

#include <Eigen/Dense>
#include <json.hpp>
#include <MeshFEM/MaterialOptimization.hh>

///
/// Optimize material properties using a local/global method.
///
/// Input material parameters are given as a json dict. This function only supports
/// box constraints in the form:
///
/// {
///   "lower": [
///     [0, E_min],
///     [1, v_min]
///   ],
///   "upper": [
///     [0, E_max],
///     [1, v_max]
///   ]
/// }
///
/// @param[in]  V                        #V x (2|3) input mesh vertices.
/// @param[in]  F                        #F x 3 input mesh triangles.
/// @param[in]  cell_index               Index of the material assign to each element.
/// @param[in]  boundary_conditions      Boundary conditions with target displacements.
/// @param[in]  material_bounds          Material bounds.
/// @param[out] material_parameters      Optimized parameters for each element.
/// @param[in]  degree                   The degree
/// @param[in]  iterations               Number of outer iterations.
/// @param[in]  iterationsPerDirichlet   Number of local/global iterations to run before re-solving
///                                      the target Dirichlet problem.
/// @param[in]  regularizationWeight     Regularization weight.
/// @param[in]  anisotropyPenaltyWeight  Anisotropy penalty weight.
/// @param[in]  noRigidMotionDirichlet   Apply no rigid motion constraint in Dirichlet solve.
/// @param[in]  intermediate_results     Where to save intermediate results.
///
void material_optimization(
	const Eigen::MatrixXd &V,
	const Eigen::MatrixXi &F,
	const Eigen::VectorXi &cell_index,
	const nlohmann::json &boundary_conditions,
	const nlohmann::json &material_bounds,
	Eigen::MatrixXd &material_parameters,
	std::vector<double> regularization_multipliers,
	size_t degree = 2,
	size_t iterations = 8,
	size_t iterationsPerDirichlet = 1,
	double regularizationWeight = 0.0,
	double anisotropyPenaltyWeight = 0.0,
	bool noRigidMotionDirichlet = false,
	const std::string &intermediate_results = "",
	const std::string material_type = "isotropic",
	size_t shear_degree = 0,
	std::vector<double> shear_coeffs = std::vector<double>(),
	double angle = 0.0,
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg);