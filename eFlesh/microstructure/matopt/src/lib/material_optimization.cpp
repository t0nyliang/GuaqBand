////////////////////////////////////////////////////////////////////////////////
#include "material_optimization.h"
#include "logger.h"
#include "convert.h"
#include "materials_ext.h"
#include <MeshFEM/BoundaryConditions.hh>
#include <MeshFEM/ElasticityTensor.hh>
#include <MeshFEM/Flattening.hh>
#include <MeshFEM/MaterialField.hh>
#include <MeshFEM/MaterialOptimization.hh>
#include <MeshFEM/Materials.hh>
#include <MeshFEM/MeshIO.hh>
#include <MeshFEM/MSHFieldParser.hh>
#include <MeshFEM/MSHFieldWriter.hh>
#include <MeshFEM/Types.hh>
#include <ceres/ceres.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

namespace {

// Term for imposing a graph laplacian-based regularization.
// For material parameter smoothness regularization, a term should be generated
// for each edge (mi, mj) of the material graph. Then mi_x and mj_x in the
// residual computation correspond to the variables of mi and mj to regularize.
template<size_t _NVars>
struct GraphLaplacianTerm {
	// Positive weights ony!
	GraphLaplacianTerm(Real w) {
		if (w < 0)
			throw std::runtime_error("Laplacian term weight must be nonnegative.");
		weightSqrt = sqrt(w);
	}

	template<typename T>
	bool operator()(const T *mi_x, const T *mj_x, T *e) const {
		for (size_t v = 0; v < _NVars; ++v)
			e[v] = T(weightSqrt) * (mi_x[v] - mj_x[v]);
		return true;
	}

	Real weightSqrt;
};

// Term for imposing similarity of orthotropic Young's moduli.
template<size_t _NVars>
struct AnisotropyTerm {
	// Positive weights ony!
	AnisotropyTerm(Real w) {
		if (w < 0)
			throw std::runtime_error("Anisotropy penalty term weight must be nonnegative.");
		weightSqrt = sqrt(w);
	}

	template<typename T>
	bool operator()(const T *m, T *e) const {
		e[0] = T(weightSqrt) * (m[1] - m[0]);
		if (_NVars == 1) return true;
		e[1] = T(weightSqrt) * (m[2] - m[0]);
		e[2] = T(weightSqrt) * (m[2] - m[1]);
		return true;
	}

	Real weightSqrt;
};

} // anonymous namespace

namespace MaterialOptimization {

template<class _Simulator>
void Optimizer<_Simulator>::run(MSHFieldWriter &writer, std::vector<Real> regularization_multipliers, size_t iterations,
		size_t iterationsPerDirichletSolve, Real regularizationWeight,
		Real anisotropyPenaltyWeight, bool noRigidMotionDirichlet, RegularizationMultMode multMode)
{
	auto neumannLoad = m_sim.neumannLoad();
	m_sim.projectOutRigidComponent(neumannLoad);
	// writer.addField("Neumann load", m_sim.dofToNodeField(neumannLoad), DomainType::PER_NODE);

	// Get "material graph" adjacences for Laplacian (smoothness) regularization
	std::vector<std::set<size_t> > materialAdj;
	m_matField->materialAdjacencies(mesh(), materialAdj);

	VField u_dirichletTargets;
	SMField e_dirichletTargets_avg;

	constexpr size_t _NVar = Material::numVars;

	// Set initial field based on lower and upper bounds
	for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
        auto &mati = m_matField->material(mi);
        auto lowerBounds = mati.lowerBounds();
        auto upperBounds = mati.upperBounds();

        for (int i=0; i<_NVar; i++)  {
            mati.vars[i] = 0.5 * (lowerBounds[i].value + upperBounds[i].value);
        }
    }

	// Write initial material variable fields
	m_matField->writeVariableFields(writer, "0 ");
	//return;

	Real current_objective;
	Real best_objective_value;
	int iteration_without_improvement = 0;
	for (size_t iter = 1; iter <= iterations; ++iter) {
		if (((iter - 1) % iterationsPerDirichletSolve) == 0) {
			m_sim.addTargetsToDirichlet();

			if (noRigidMotionDirichlet) m_sim.applyNoRigidMotionConstraint();
			else                        m_sim.removeNoRigidMotionConstraint();

			// std::cout << "solving target dirichlet" << std::endl;
			// m_sim.dumpDirichlet();
			u_dirichletTargets = m_sim.solve(neumannLoad);
			e_dirichletTargets_avg = m_sim.averageStrainField(u_dirichletTargets);

			m_sim.removeTargetsFromDirichlet();

			// Apply a no rigid motion constraint if the user didn't specify
			// Dirichlet constraints. If Dirichlet constraints are present, they
			// must fully pin down the rigid degrees of freedom, since we don't
			// yet support partial no-rigid-motion constraints in this setting.
			ComponentMask needsTranslationConstraint, needsRotationConstraint;
			m_sim.analyzeDirichletPosedness(needsTranslationConstraint, needsRotationConstraint);
			if (needsTranslationConstraint.hasAny(N) || needsTranslationConstraint.hasAny(N)) {
				if (needsTranslationConstraint.hasAll(N) && needsTranslationConstraint.hasAll(N))
					m_sim.applyRigidMotionConstraint(u_dirichletTargets);
				else {
					throw std::runtime_error("Incomplete Dirichlet constraints are currently unsupported");
				}
			}
		}

		// std::cout << "solving user load" << std::endl;
		// m_sim.dumpDirichlet();
		auto u = m_sim.solve(neumannLoad);
		auto s_neumann_avg = m_sim.averageStressField(u);

		if (iter == 1) {
		    current_objective = objective(u);
		    best_objective_value = current_objective;

			// Write inital ("iteration 0") objective and gradient norm.
			std::vector<Real> g = objectiveGradient(u);
			Real gradNormSq = 0;
			for (size_t c = 0; c < g.size(); ++c) {
				gradNormSq += g[c] * g[c];
			}
			logger().debug("0 objective, gradient norm:\t{}\t{}", current_objective, sqrt(gradNormSq));
		}

		writer.addField(std::to_string(iter) + " u_neumann",          u,                  DomainType::PER_NODE);
		writer.addField(std::to_string(iter) + " u_dirichletTargets", u_dirichletTargets, DomainType::PER_NODE);

		ceres::Problem problem;

		typedef typename Material::template StressStrainFitCostFunction<typename SMField::ValueType> Fitter;
		for (size_t ei = 0; ei < mesh().numElements(); ++ei) {
			ceres::CostFunction *fitCost = new ceres::AutoDiffCostFunction<
				Fitter, flatLen(N), _NVar>(new Fitter(e_dirichletTargets_avg(ei), s_neumann_avg(ei),
										   mesh().element(ei)->volume()));
			problem.AddResidualBlock(fitCost, NULL,
									 &(m_matField->materialForElement(ei).vars[0]));
		}

		//ceres::CostFunction *regularizer = NULL;
		//if (regularizationWeight > 0.0) {
		//	regularizer = new ceres::AutoDiffCostFunction<
		//		GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
		//				new GraphLaplacianTerm<_NVar>(regularizationWeight));
		//}

		//logger().debug("Before anisotropy penalty");

		ceres::CostFunction *anisotropyPenalty = NULL;
		// Note: we have to always provide the same size for a given parameter
		// block, so even though we only reference numE variables, we must claim
		// the residual block is affected by all _NVar variables in each block.
		if ((_NVar == 4 || _NVar == 9) && anisotropyPenaltyWeight > 0.0) {
			constexpr size_t numE = (_NVar == 4) ? 1 : 3;
			anisotropyPenalty = new ceres::AutoDiffCostFunction<
				AnisotropyTerm<numE>, numE, _NVar>(
						new AnisotropyTerm<numE>(anisotropyPenaltyWeight));
		}

		// Add in variable bounds and regularization (if requested)
		for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
			auto &mati = m_matField->material(mi);
			for (const auto &bd : mati.upperBounds()) problem.SetParameterUpperBound(&(mati.vars[0]), bd.var, bd.value);
			for (const auto &bd : mati.lowerBounds()) problem.SetParameterLowerBound(&(mati.vars[0]), bd.var, bd.value);

			if (anisotropyPenalty)
				problem.AddResidualBlock(anisotropyPenalty, NULL, &(mati.vars[0]));

			if (regularizationWeight <= 0.0) continue;
			for (size_t mj : materialAdj.at(mi)) {
				// Make sure graph is undirected.
				assert(materialAdj.at(mj).find(mi) != materialAdj.at(mj).end());
				// Add one term per edge, not two
				if (mi < mj) continue;

				ceres::CostFunction *regularizer = new ceres::AutoDiffCostFunction<
				//GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
				//		new GraphLaplacianTerm<_NVar>(std::max(regularization_multipliers[mi],regularization_multipliers[mj]) * regularizationWeight));
				//GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
				//		new GraphLaplacianTerm<_NVar>((regularization_multipliers[mi] + regularization_multipliers[mj]) * regularizationWeight));
                GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
						new GraphLaplacianTerm<_NVar>(std::min(regularization_multipliers[mi], regularization_multipliers[mj]) * regularizationWeight));

				problem.AddResidualBlock(regularizer, NULL, &(mati.vars[0]),
										 &(m_matField->material(mj).vars[0]));
			}
		}

		ceres::Solver::Options options;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.BriefReport() << "\n";

		/*logger().debug("start test X");
		m_matField->writeVariableFields(writer, std::to_string(iter) + " ");
		m_sim.materialFieldUpdated();
		u = m_sim.solve(neumannLoad);
		logger().debug("end test X");
		*/

		// Write current material variable fields
		//m_matField->writeVariableFields(writer, std::to_string(iter) + " ");

		//logger().debug("Before post-iteration solution and print statistics");

		// Write the post-iteration solution and print statistics
		m_sim.materialFieldUpdated();
		m_matField->writeVariableFields(writer, std::to_string(iter) + " ");
		//logger().debug("After updating material field");
		u = m_sim.solve(neumannLoad);
		current_objective = objective(u);
		//logger().debug("After solving neumannLoad");

		std::vector<Real> g = objectiveGradient(u);
		writer.addField(std::to_string(iter) + " u", u, DomainType::PER_NODE);

		// Write gradient component fields
		m_matField->writeVariableFields(writer, std::to_string(iter) + " grad_", g);

		Real gradNormSq = 0;
		for (size_t c = 0; c < g.size(); ++c) {
			gradNormSq += g[c] * g[c];
		}
		logger().debug("{} objective, gradient norm:\t{}\t{}", iter, current_objective, sqrt(gradNormSq));
		std::cout << iter << " objective, gradient norm: " << current_objective << " " << sqrt(gradNormSq) << std::endl;

		if (iter > 1) {
            if (current_objective < (best_objective_value * 0.99)) {
                best_objective_value = current_objective;
                iteration_without_improvement = 0;
            } else {
                std::cout << "Best: " << best_objective_value << std::endl;
                std::cout << "Current: " << current_objective << std::endl;
                iteration_without_improvement++;
            }

            if (iteration_without_improvement >= 5) {
                std::cout << "Leaving optimization due to " << iteration_without_improvement
                          << " iterations without improvement!" << std::endl;
                std::cout << "Best: " << best_objective_value << std::endl;
                std::cout << "Current: " << current_objective << std::endl;
                break;
            }
        }
		else {
		    best_objective_value = current_objective;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// Explicit Instantiations
////////////////////////////////////////////////////////////////////////////////

//                                      Dim  Deg  Material
template class Optimizer<Simulator<Mesh<2,   1,   Materials::Isotropic  >>>;
template class Optimizer<Simulator<Mesh<2,   1,   Materials::Orthotropic>>>;
template class Optimizer<Simulator<Mesh<2,   2,   Materials::Isotropic  >>>;
template class Optimizer<Simulator<Mesh<2,   2,   Materials::Orthotropic>>>;
template class Optimizer<Simulator<Mesh<3,   1,   Materials::Isotropic  >>>;
template class Optimizer<Simulator<Mesh<3,   1,   Materials::Orthotropic>>>;
template class Optimizer<Simulator<Mesh<3,   2,   Materials::Isotropic  >>>;
template class Optimizer<Simulator<Mesh<3,   2,   Materials::Orthotropic>>>;

template class Optimizer<Simulator<Mesh<2,   1,   Materials::Cubic      >>>;
template class Optimizer<Simulator<Mesh<2,   2,   Materials::Cubic      >>>;
template class Optimizer<Simulator<Mesh<3,   1,   Materials::Cubic      >>>;
template class Optimizer<Simulator<Mesh<3,   2,   Materials::Cubic      >>>;


} // namespace MaterialOptimization

////////////////////////////////////////////////////////////////////////////////
// Run material optimization on a particular (mesh, bc) pair.
////////////////////////////////////////////////////////////////////////////////

namespace {

///
/// Optimize material properties using a local/global method.
///
/// @param[in]  inVertices               Mesh vertices.
/// @param[in]  inElements               Mesh elements.
/// @param[in]  matIdxForElement         Material index for each element.
/// @param[in]  boundaryConditions       Boundary conditions with target displacements.
/// @param[in]  materialBounds           Material bounds.
/// @param[out] optimizedParameters      Optimized parameters for each element.
/// @param[in]  iterations               Number of outer iterations.
/// @param[in]  iterationsPerDirichlet   Number of local/global iterations to run before re-solving
///                                      the target Dirichlet problem.
/// @param[in]  regularizationWeight     Regularization weight.
/// @param[in]  anisotropyPenaltyWeight  Anisotropy penalty weight.
/// @param[in]  noRigidMotionDirichlet   Apply no rigid motion constraint in Dirichlet solve.
/// @param[in]  intermediateResults      Where to save intermediate results.
///
/// @tparam     _N                       Dimension of the mesh.
/// @tparam     _FEMDegree               FEM degree.
/// @tparam     _Material                Material type.
///
template<size_t _N, size_t _FEMDegree, template<size_t> class _Material>
void execute(
	const std::vector<MeshIO::IOVertex> &inVertices,
	const std::vector<MeshIO::IOElement> &inElements,
	std::vector<size_t> matIdxForElement,
	const nlohmann::json &boundaryConditions,
	const nlohmann::json &materialBounds,
	Eigen::MatrixXd &optimizedParameters,
	std::vector<double> regularization_multipliers,
	size_t iterations = 8,
	size_t iterationsPerDirichlet = 1,
	Real regularizationWeight = 0.0,
	Real anisotropyPenaltyWeight = 0.0,
	bool noRigidMotionDirichlet = false,
	const std::string &intermediateResults = "",
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg)
{
	typedef MaterialOptimization::Mesh<_N, _FEMDegree, _Material> Mesh;
	typedef MaterialOptimization::Simulator<Mesh>           Simulator;
	typedef MaterialOptimization::Optimizer<Simulator>      Opt;

	typedef typename Opt::MField MField;

	// Material bounds are global variables
	_Material<_N>::setBoundsFromJson(materialBounds);

	// If available, use element->material associations.
	// Otherwise, we use one material per element.
	auto matField = std::make_shared<MField>(inElements.size(), matIdxForElement);

	// Read boundary conditions
	bool noRigidMotion;
	std::string bcs(boundaryConditions.dump());
	std::istringstream iss(bcs);
	auto bconds = readBoundaryConditions<_N>(iss,
		BBox<VectorND<_N>>(inVertices), noRigidMotion);

	Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion);

	MSHFieldWriter writer(intermediateResults, matOpt.mesh());

	logger().debug("Attempting optimization");
	matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
		regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet);

	matField->writeVariableFields(writer, "Final ");

	// Extract material parameters
	optimizedParameters.resize(inElements.size(), _Material<_N>::numVars);
	for (size_t f = 0; f < inElements.size(); ++f) {
		auto mat = matField->materialForElement(f);
		for (int c = 0; c < _Material<_N>::numVars; ++c) {
			optimizedParameters(f, c) = mat.vars[c];
		}
	}
}

///
/// Optimize material properties using a local/global method.
///
/// @param[in]  inVertices               Mesh vertices.
/// @param[in]  inElements               Mesh elements.
/// @param[in]  matIdxForElement         Material index for each element.
/// @param[in]  boundaryConditions       Boundary conditions with target displacements.
/// @param[in]  materialBounds           Material bounds.
/// @param[out] optimizedParameters      Optimized parameters for each element.
/// @param[in]  iterations               Number of outer iterations.
/// @param[in]  iterationsPerDirichlet   Number of local/global iterations to run before re-solving
///                                      the target Dirichlet problem.
/// @param[in]  regularizationWeight     Regularization weight.
/// @param[in]  anisotropyPenaltyWeight  Anisotropy penalty weight.
/// @param[in]  noRigidMotionDirichlet   Apply no rigid motion constraint in Dirichlet solve.
/// @param[in]  intermediateResults      Where to save intermediate results.
///
/// @tparam     _N                       Dimension of the mesh.
/// @tparam     _FEMDegree               FEM degree.
///
template<size_t _N, size_t _FEMDegree>
void executeConstrainedCubic(
	const std::vector<MeshIO::IOVertex> &inVertices,
	const std::vector<MeshIO::IOElement> &inElements,
	std::vector<size_t> matIdxForElement,
	const nlohmann::json &boundaryConditions,
	const nlohmann::json &materialBounds,
	Eigen::MatrixXd &optimizedParameters,
	std::vector<double> regularization_multipliers,
	size_t iterations = 8,
	size_t iterationsPerDirichlet = 1,
	Real regularizationWeight = 0.0,
	Real anisotropyPenaltyWeight = 0.0,
	bool noRigidMotionDirichlet = false,
	const std::string &intermediateResults = "",
	size_t shear_degree = 0,
	std::vector<double> shear_coeffs = std::vector<double>(),
	Real angle = 0.0,
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg)
{
	typedef MaterialOptimization::Mesh<_N, _FEMDegree, Materials::ConstrainedCubic> Mesh;
	typedef MaterialOptimization::Simulator<Mesh>           Simulator;
	typedef MaterialOptimization::Optimizer<Simulator>      Opt;

	typedef typename Opt::MField MField;

	// Material bounds are global variables
	Materials::ConstrainedCubic<_N>::setBoundsFromJson(materialBounds);
	Materials::ConstrainedCubic<_N>::setShearParameters(shear_degree, shear_coeffs);

	const Real PI = 3.14159265;
    const Real theta = angle*PI/180;
    Materials::ConstrainedCubic<_N>::setOrthotropicAngle(theta);

	// If available, use element->material associations.
	// Otherwise, we use one material per element.
	auto matField = std::make_shared<MField>(inElements.size(), matIdxForElement);

	// Read boundary conditions
	bool noRigidMotion;
	std::string bcs(boundaryConditions.dump());
	std::istringstream iss(bcs);
	auto bconds = readBoundaryConditions<_N>(iss,
		BBox<VectorND<_N>>(inVertices), noRigidMotion);

	Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion);

	MSHFieldWriter writer(intermediateResults, matOpt.mesh());

	logger().debug("Attempting optimization");
	matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
		regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet);

	matField->writeVariableFields(writer, "Final ");

	// Extract material parameters
	optimizedParameters.resize(inElements.size(), Materials::ConstrainedCubic<_N>::numVars);
	for (size_t f = 0; f < inElements.size(); ++f) {
		auto mat = matField->materialForElement(f);
		for (int c = 0; c < Materials::ConstrainedCubic<_N>::numVars; ++c) {
			optimizedParameters(f, c) = mat.vars[c];
		}
	}
}

} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////

void material_optimization(
	const Eigen::MatrixXd &V,
	const Eigen::MatrixXi &F,
	const Eigen::VectorXi &cell_index,
	const nlohmann::json &boundary_conditions,
	const nlohmann::json &material_bounds,
	Eigen::MatrixXd &material_parameters,
	std::vector<double> regularization_multipliers,
	size_t degree,
	size_t iterations,
	size_t iterationsPerDirichlet,
	double regularizationWeight,
	double anisotropyPenaltyWeight,
	bool noRigidMotionDirichlet,
	const std::string &intermediate_results,
	const std::string materialType,
	size_t shear_degree,
	std::vector<double> shear_coeffs,
	double angle,
	MaterialOptimization::RegularizationMultMode multMode)
{
	// Load input triangle mesh
	std::vector<MeshIO::IOVertex> VI;
	std::vector<MeshIO::IOElement> FI;
	to_meshfem(V, F, VI, FI);

	// Read cell index if available
	std::vector<size_t> matIdxForElement;
	if (cell_index.rows() == F.rows()) {
		matIdxForElement.reserve(F.rows());
		for (int i = 0; i < F.rows(); ++i) {
			matIdxForElement.push_back(cell_index[i]);
		}
	}

	if (materialType == "constrained_cubic") {

	    // Look up and run appropriate optimizer instantiation.
	    auto exec = (degree == 1)
	            ? executeConstrainedCubic<2, 1>
				: executeConstrainedCubic<2, 2>;

	    exec(VI,
            FI,
            matIdxForElement,
            boundary_conditions,
            material_bounds,
            material_parameters,
            regularization_multipliers,
            iterations,
            iterationsPerDirichlet,
            regularizationWeight,
            anisotropyPenaltyWeight,
            noRigidMotionDirichlet,
            intermediate_results,
            shear_degree,
            shear_coeffs,
            angle,
            multMode);
	}
	else {
        // Look up and run appropriate optimizer instantiation.
        auto exec = (degree == 1) ?
                    ((materialType == "orthotropic")
                     ? execute<2, 1, Materials::Orthotropic>
                     : ((materialType == "cubic")
                        ? execute<2, 1, Materials::Cubic>
                        : (execute<2, 1, Materials::Isotropic>)))
                                  : ((materialType == "orthotropic")
                                     ? execute<2, 2, Materials::Orthotropic>
                                     : ((materialType == "cubic")
                                        ? execute<2, 2, Materials::Cubic>
                                        : execute<2, 2, Materials::Isotropic>));

        exec(VI,
             FI,
             matIdxForElement,
             boundary_conditions,
             material_bounds,
             material_parameters,
             regularization_multipliers,
             iterations,
             iterationsPerDirichlet,
             regularizationWeight,
             anisotropyPenaltyWeight,
             noRigidMotionDirichlet,
             intermediate_results,
             multMode);
    }
}
