#pragma once

#include <Eigen/Dense>
#include <json.hpp>
#include <MeshFEM/MaterialOptimization.hh>

///
/// Optimize material properties using a local/global method.
///
/// Input material parameters are given as a json dict. There can be two variants:
///
/// 1) Simple box region.
///    {
///      "lower": [
///        [0, E_min],
///        [1, v_min]
///      ],
///      "upper": [
///        [0, E_max],
///        [1, v_max]
///      ]
///    }
///
/// 2) Simplex region defined by an intersection of half-planes:
///
///  {
///    "simplex": [
///      [a_0, b_0, c_0],
///      ...
///      [a_i, b_i, c_i],
///    ]
///  }
///
/// 3) One material domain per element:
///
/// {
///   "simplex_per_element": [
///     [
///       [a_0^0, b_0^0, c_0^0],
///       ...
///     ],
///     ...
///     [
///       ...
///       [a_i^k, b_i^k, c_i^k],
///       ...
///     ],
///   ]
/// }
///
///
///

///
/// In the second version, the half-planes defining the material region are defined by the equation
/// a_i * x + b_i * y + c_i >= 0. In the third version, "simplex_per_element" is a list of simplices
/// defining a different material region, one for every element of the mesh.
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
/// @param[in]  ipopt_config             Parameters to forward to Ipopt.
/// @param[in]  intermediate_results     Where to save intermediate results.
///
void material_optimization_ipopt(
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
	const nlohmann::json &ipoptConfig = {},
	const std::string &intermediateResults = "",
	const std::string materialType = "isotropic",
	size_t shear_degree = 0,
	std::vector<double> shear_coeffs = std::vector<double>(),
	double angle = 0,
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg);
