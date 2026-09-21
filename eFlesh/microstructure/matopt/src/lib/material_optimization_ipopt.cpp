#include "material_optimization_ipopt.h"
#include "logger.h"
#include "convert.h"
#include "polygons.h"
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
#include <igl/write_triangle_mesh.h>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpIpoptCalculatedQuantities.hpp>
#include <IpIpoptData.hpp>
#include <IpOrigIpoptNLP.hpp>
#include <IpTNLPAdapter.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

namespace {

template<class Optimizer>
class MaterialOptimizationProblem : public Ipopt::TNLP {

public:
	typedef Ipopt::Number Number;
	typedef Ipopt::Index Index;

	ceres::Problem * m_problem;
	std::vector<double> m_grad_f;
	Optimizer * m_opt;

	const Optimizer & opt() const { return *m_opt; }
	Optimizer & opt() { return *m_opt; }

	void set_variables(Index n, const Number *x);
	void get_variables(Index n, Number *x) const;

	virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
		Index& nnz_h_lag, IndexStyleEnum& index_style) override;
	virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
		Number* g_l, Number* g_u) override;
	virtual bool get_starting_point(Index n, bool init_x, Number* x,
		bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda,
		Number* lambda) override;
	virtual bool eval_f(
		Index n, const Number* x, bool new_x, Number& obj_value) override;
	virtual bool eval_grad_f(
		Index n, const Number* x, bool new_x, Number* grad_f) override;
	virtual bool eval_g(
		Index n, const Number* x, bool new_x, Index m, Number* g) override;
	virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
		Index nele_jac, Index* iRow, Index* jCol, Number* values) override;
	virtual void finalize_solution(Ipopt::SolverReturn status, Index n,
		const Number* x, const Number* z_L, const Number* z_U, Index m,
		const Number* g, const Number* lambda, Number obj_value,
		const Ipopt::IpoptData* ip_data,
		Ipopt::IpoptCalculatedQuantities* ip_cq) override;

	void solve();
};

} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////

namespace MaterialOptimization {

template<class _Simulator>
class IpoptOptimizer {
public:
	static constexpr size_t N = _Simulator::N;
	static constexpr size_t K = _Simulator::K;
	static constexpr size_t Degree = _Simulator::Degree;

	typedef typename _Simulator::SField  SField;
	typedef typename _Simulator::VField  VField;
	typedef typename _Simulator::SMField SMField;
	typedef typename _Simulator::SMatrix SMatrix;
	typedef typename _Simulator::ETensor ETensor;
	typedef typename _Simulator::_Point  _Point;
	typedef typename _Simulator::MField  MField;
	typedef typename MField::Material    Material;

	typedef MaterialOptimizationProblem<IpoptOptimizer<_Simulator>> IpoptProblem;

	template<typename Elems, typename Vertices>
	IpoptOptimizer(Elems inElems, Vertices inVertices,
			std::shared_ptr<MField> matField,
			const std::vector<CondPtr<N> > &boundaryConditions,
			bool noRigidMotion)
		: m_sim(inElems, inVertices, matField), m_matField(matField)
	{
		m_sim.applyBoundaryConditions(boundaryConditions);
		if (noRigidMotion)
			m_sim.applyNoRigidMotionConstraint();
	}

	VField currentDisplacement() const {
		return m_sim.solve();
	}

	// 1/2 int_bdry ||u - t||^2 dA = 1/2 int_bdry ||d||^2 dA
	// where d = componentMask * (u - t) is the component-masked
	// distance-to-target vector field (linearly/quadratically interpolated over
	// each boundary element).
	Real objective(const VField &u) const {
		Real obj = 0;
		Interpolant<_Point, K - 1, Degree> dist;
		for (size_t bei = 0; bei < m_sim.mesh().numBoundaryElements(); ++bei) {
			auto be = m_sim.mesh().boundaryElement(bei);
			for (size_t i = 0; i < be.numNodes(); ++i) {
				auto bn = be.node(i);
				dist[i] = bn->targetComponents.apply((u(bn.volumeNode().index())
							- bn->targetDisplacement).eval());
			}
			obj += Quadrature<K - 1, Degree * Degree>::integrate(
					[&](const EvalPt<K - 1> &p)
						{ return dist(p).dot(dist(p)); }, be->volume());
		}

		/*std::cout << "Checking variables: " << std::endl;
		for (size_t ei = 0; ei < mesh().numElements(); ++ei) {
             std::cout << materialField().materialForElement(ei).vars[0] << std::endl;
             std::cout << materialField().materialForElement(ei).vars[1] << std::endl;
        }*/

		return obj / 2;
	}

	// From adjoint method:
	// dJ/dp = int_omega strain(u) : dE/dp : strain(lambda) dv
	std::vector<Real> objectiveGradient(const VField &u) const {
		auto lambda = m_sim.solveAdjoint(u);
		std::vector<Real> g(m_matField->numVars(), 0);
		std::vector<size_t> elems;
		typename _Simulator::Strain e_u, e_lambda;
		for (size_t var = 0; var < m_matField->numVars(); ++var) {
			// Support of dE/dp on the mesh.
			m_matField->getInfluenceRegion(var, elems);
			ETensor dE;
			m_matField->getETensorDerivative(var, dE);
			for (size_t i = 0; i < elems.size(); ++i) {
				size_t ei = elems[i];
				auto e = m_sim.mesh().element(ei);
				m_sim.elementStrain(ei,      u,      e_u);
				m_sim.elementStrain(ei, lambda, e_lambda);
				g[var] += Quadrature<K, (Degree - 1) * (Degree - 1)>::integrate(
					[&](const EvalPt<K> &p)
						{ return dE.doubleContract(e_u(p))
								   .doubleContract(e_lambda(p)); },
					e->volume());
			}
		}

		return g;
	}

	void run(MSHFieldWriter &writer, std::vector<Real> regularization_multipliers,
	    size_t iterations = 15,
		size_t iterationsPerDirichletSolve = 1,
		Real regularizationWeight = 0.0,
		Real anisotropyPenaltyWeight = 0.0,
		bool noRigidMotionDirichlet = false,
		const nlohmann::json &ipoptConfig = {},
		RegularizationMultMode = avg);

	// Number of linear constraints for a given element
	size_t numConstraintsForElement(size_t elem) const {
		return m_constraints[elem].size();
	}

	// Evaluate a hyperplane constraint for an element and a local half-plane
	double evalConstraint(size_t elem, size_t index, const double *x) const {
	    const HyperPlane &plane = m_constraints[elem][index];
		double res = plane.back();
		for (size_t k = 0; k < Material::numVars; ++k) {
			res += plane[k] * x[k];
		}

		//std::cout << "Element " << elem << " constraint " << index << ": " << x[0] << ", " << x[1] << " - "  << res << std::endl;
		return res;
	}

	// Evaluate the gradient of a hyperplane constraint for an element and a local half-plane
	void evalConstraintGrad(size_t elem, size_t index, const double *x, double *g) const {
		const HyperPlane &plane = m_constraints[elem][index];
		for (size_t k = 0; k < Material::numVars; ++k) {
			g[k] = plane[k];
		}
	}

	void readMaterialConstraints(const nlohmann::json &bounds);

	const typename _Simulator::Mesh &mesh() const { return m_sim.mesh(); }
	const _Simulator &simulator() const { return m_sim; }

	const MField &materialField() const { return *m_matField; }
	MField &materialField() { return *m_matField; }

private:
	_Simulator m_sim;
	std::shared_ptr<typename _Simulator::MField> m_matField;

	// Linear constraints for each element of the mesh.
	// Each element has h_e linear constraints on its material properties.
	// Each linear constraint is a tuple {c_k}_k representing the hyperplane
	// \sum_{k=0}^{H-2} c_k * mat_k + c_{H-1} <= 0

	typedef std::array<double, Material::numVars + 1> HyperPlane;
	std::vector<std::vector<HyperPlane>> m_constraints;
};

} // namespace MaterialOptimization

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
		if (w < 0) {
			throw std::runtime_error("Laplacian term weight must be nonnegative.");
		}
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
		if (w < 0) {
			throw std::runtime_error("Anisotropy penalty term weight must be nonnegative.");
		}
		weightSqrt = sqrt(w);
	}

	template<typename T>
	bool operator()(const T *m, T *e) const {
		e[0] = T(weightSqrt) * (m[1] - m[0]);
		if (_NVars == 1) { return true; }
		e[1] = T(weightSqrt) * (m[2] - m[0]);
		e[2] = T(weightSqrt) * (m[2] - m[1]);
		return true;
	}

	Real weightSqrt;
};

template<size_t _NVars>
struct OrthotropicShearTerm {
	// Positive weights ony!
	OrthotropicShearTerm(Real w, std::vector<double> poly_weights, int degree) {
		if (w < 0) {
			throw std::runtime_error("orthotropic shear penalty term weight must be nonnegative.");
		}
		weightSqrt = sqrt(w);

		// save info about polynomial weights

	}

	template<typename T>
	bool operator()(const T *m, T *e) const {
		e[0] = T(weightSqrt) * (m[1] - m[0]);
		if (_NVars == 1) { return true; }
		e[1] = T(weightSqrt) * (m[2] - m[0]);
		e[2] = T(weightSqrt) * (m[2] - m[1]);
		return true;
	}

	Real weightSqrt;
};


} // anonymous namespace

////////////////////////////////////////////////////////////////////////////////

namespace MaterialOptimization {

template<class _Simulator>
void IpoptOptimizer<_Simulator>::run(
	MSHFieldWriter &writer, std::vector<double> regularization_multipliers, size_t iterations,
	size_t iterationsPerDirichletSolve, Real regularizationWeight,
	Real anisotropyPenaltyWeight, bool noRigidMotionDirichlet,
	const nlohmann::json &ipoptConfig, RegularizationMultMode multMode)
{
	auto neumannLoad = this->m_sim.neumannLoad();
	m_sim.projectOutRigidComponent(neumannLoad);
	writer.addField("Neumann load", m_sim.dofToNodeField(neumannLoad), DomainType::PER_NODE);

	// Get "material graph" adjacences for Laplacian (smoothness) regularization
	std::vector<std::set<size_t> > materialAdj;
	m_matField->materialAdjacencies(mesh(), materialAdj);

	VField u_dirichletTargets;
	SMField e_dirichletTargets_avg;

	// Write initial material variable fields
	m_matField->writeVariableFields(writer, "0 ");

	constexpr size_t _NVar = Material::numVars;
	Real current_objective;
	Real best_objective_value;
	int iteration_without_improvement = 0;
	MField best_material_field = *m_matField;
	for (size_t iter = 1; iter <= iterations; ++iter) {
	    std::cout << "[MATOPT IPOPT] iter " << iter << std::endl;
		if (((iter - 1) % iterationsPerDirichletSolve) == 0) {
		    m_sim.addTargetsToDirichlet();
            if (noRigidMotionDirichlet) m_sim.applyNoRigidMotionConstraint();
			else                        m_sim.removeNoRigidMotionConstraint();

			logger().trace("solving target dirichlet");
			//for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
			// 	auto &mati = m_matField->material(mi);
			 	//qfm::logger().trace("mat_{}, E={}, v={}", mi, mati.vars[0], mati.vars[1]);
			//}
			//m_sim.dumpDirichlet();
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
				if (needsTranslationConstraint.hasAll(N) && needsTranslationConstraint.hasAll(N)) {
					m_sim.applyRigidMotionConstraint(u_dirichletTargets);
				} else {
					throw std::runtime_error("Incomplete Dirichlet constraints are currently unsupported");
				}
			}
		}

		//qfm::logger().trace("solving user load");
		// for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
		// 	auto &mati = m_matField->material(mi);
		// 	qfm::logger().trace("mat_{}, E={}, v={}", mi, mati.vars[0], mati.vars[1]);
		// }
		// m_sim.dumpDirichlet();
		auto u = m_sim.solve(neumannLoad);
		auto s_neumann_avg = m_sim.averageStressField(u);

		if (iter == 1) {
		    current_objective = objective(u);
		    best_objective_value = current_objective;
		    best_material_field = *m_matField;

			// Write initial ("iteration 0") objective and gradient norm.
			std::vector<Real> g = objectiveGradient(u);
			Real gradNormSq = 0;
			for (size_t c = 0; c < g.size(); ++c) {
				gradNormSq += g[c] * g[c];
			}
			logger().debug("0 objective, gradient norm:\t{}\t{}", current_objective, sqrt(gradNormSq));
		}

		writer.addField(std::to_string(iter) + " u_neumann",          u,                  DomainType::PER_NODE);
		writer.addField(std::to_string(iter) + " u_dirichletTargets", u_dirichletTargets, DomainType::PER_NODE);

		///////////////////////////////////
		// Solve non-linear least-square //
		///////////////////////////////////

		ceres::Problem problem;

		typedef typename Material::template StressStrainFitCostFunction<typename SMField::ValueType> Fitter;
		for (size_t ei = 0; ei < mesh().numElements(); ++ei) {
			ceres::CostFunction *fitCost = new ceres::AutoDiffCostFunction<
				Fitter, flatLen(N), _NVar>(new Fitter(e_dirichletTargets_avg(ei), s_neumann_avg(ei),
										   mesh().element(ei)->volume()));
			problem.AddResidualBlock(fitCost, NULL,
									 &(m_matField->materialForElement(ei).vars[0]));
		}

		/*ceres::CostFunction *regularizer = NULL;
		if (regularizationWeight > 0.0) {
			regularizer = new ceres::AutoDiffCostFunction<
				GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
						new GraphLaplacianTerm<_NVar>(regularizationWeight));
		}*/

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
			if (anisotropyPenalty) {
				problem.AddResidualBlock(anisotropyPenalty, NULL, &(mati.vars[0]));
			}

			if (regularizationWeight <= 0.0) continue;
			for (size_t mj : materialAdj.at(mi)) {
				// Make sure graph is undirected.
				assert(materialAdj.at(mj).find(mi) != materialAdj.at(mj).end());
				// Add one term per edge, not two
				if (mi < mj) continue;

				//std::cout << mi << ", " << mj << std::endl;
				ceres::CostFunction *regularizer = NULL;
				switch (multMode) {
				    case min:
				        regularizer = new ceres::AutoDiffCostFunction<GraphLaplacianTerm<_NVar>, _NVar, _NVar, _NVar>(
                                new GraphLaplacianTerm<_NVar>(std::min(regularization_multipliers[mi],regularization_multipliers[mj]) * regularizationWeight));
				        break;
				    case avg:
				        regularizer = new ceres::AutoDiffCostFunction<GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
                                new GraphLaplacianTerm<_NVar>(0.5*(regularization_multipliers[mi] + regularization_multipliers[mj]) * regularizationWeight));
				        break;
				    case max:
				        regularizer = new ceres::AutoDiffCostFunction<GraphLaplacianTerm<_NVar>, _NVar,  _NVar, _NVar>(
                                new GraphLaplacianTerm<_NVar>(std::max(regularization_multipliers[mi], regularization_multipliers[mj]) * regularizationWeight));
				        break;
				    default:
				        break;
				}
				problem.AddResidualBlock(regularizer, NULL, &(mati.vars[0]),
										 &(m_matField->material(mj).vars[0]));
			}
		}

		ceres::Solver::Options options;

		Ipopt::SmartPtr<IpoptProblem> nlp = new IpoptProblem();
		nlp->m_problem = &problem;
		nlp->m_opt = this;

		// Solve with Ipopt
		{
			logger().trace("Reading Ipopt solver options:\n{}", ipoptConfig.dump(4));
			Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
			auto setDoubleValue = [&](const std::string &arg) {
				if (ipoptConfig.count(arg)) {
					app->Options()->SetNumericValue(arg, ipoptConfig[arg].template get<double>());
				}
			};
			auto setIntValue = [&](const std::string &arg) {
				if (ipoptConfig.count(arg)) {
					app->Options()->SetIntegerValue(arg, ipoptConfig[arg].template get<int>());
				}
			};
			auto setStringValue = [&](const std::string &arg) {
				if (ipoptConfig.count(arg)) {
					app->Options()->SetStringValue(arg, ipoptConfig[arg].template get<std::string>());
				}
			};
			app->Options()->SetStringValue("hessian_approximation", "limited-memory");
			setDoubleValue("tol");
			setIntValue("max_iter");
			setIntValue("print_level");
			setStringValue("mu_strategy");
			setStringValue("bound_mult_init_method");
			setStringValue("linear_solver");

			Ipopt::ApplicationReturnStatus app_status;
			app_status = app->Initialize();

			app_status = app->OptimizeTNLP(nlp);
			assert(app_status == Ipopt::Solve_Succeeded);
			(void) app_status;
		}

        /*for (size_t ei = 0; ei < mesh().numElements(); ++ei) {
             std::cout << materialField().materialForElement(ei).vars[0] << std::endl;
             std::cout << materialField().materialForElement(ei).vars[1] << std::endl;
        }*/

		///////////////////////////////
		// End of least-square solve //
		///////////////////////////////

		// Write current material variable fields
		m_matField->writeVariableFields(writer, std::to_string(iter) + " ");

		// Write the post-iteration solution and print statistics
		m_sim.materialFieldUpdated();
		u = m_sim.solve(neumannLoad);
		current_objective = objective(u);
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
                best_material_field = *m_matField;
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

	*m_matField = best_material_field;
	logger().debug("Best objective:\t{}", best_objective_value);
}

template<class _Simulator>
void IpoptOptimizer<_Simulator>::readMaterialConstraints(const nlohmann::json &bounds) {
	m_constraints.assign(mesh().numElements(), {});

	auto numVars = Material::numVars;
	if (bounds.is_array()) {
	    //throw std::runtime_error("Constraint not implemented yet: lower/upper per element");

	    for (size_t e = 0; e < mesh().numElements(); ++e) {
	        auto &mati = m_matField->materialForElement(e);
			for (size_t k = 0; k < numVars; ++k) {
				mati.vars[k] = 0.0;
			}

			auto ebounds = bounds[e];
			if (ebounds.count("simplex")) {
                auto cst = ebounds["simplex"];
                Eigen::MatrixXd planes(cst.size(), cst[0].size());
                for (int i = 0; i < planes.rows(); ++i) {
                    HyperPlane plane;
                    for (int j = 0; j < planes.cols(); ++j) {
                        plane[j] = planes(i, j) = cst[i][j];
                    }
                    m_constraints[e].push_back(plane);
                }
                Eigen::MatrixXd vertices;
                intersect_half_planes(planes, vertices);
                if (vertices.rows() == 0) {
                    //logger().error("Found empty constraint set:\n{}", planes);
                    //qfm_assert(false);
                }
                mati.vars[0] = vertices.col(0).mean();
                mati.vars[1] = vertices.col(1).mean();
			}
			else {
			    /*for (const auto &kv : bounds[e]["lower"]) {
                    HyperPlane plane;
                    std::fill(plane.begin(), plane.end(), 0.0);
                    for (size_t k = 0; k < numVars; ++k) {
                        if (k == (size_t) kv[0]) {
                            plane[k] = -1.0;
                        }
                    }
                    plane.back() = kv[1];
                    m_constraints[e].push_back(plane);
                    //std::cout << "lower: " << m_constraints[e].size();
                    mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
                }
                for (const auto &kv : bounds[e]["upper"]) {
                    HyperPlane plane;
                    std::fill(plane.begin(), plane.end(), 0.0);
                    for (size_t k = 0; k < numVars; ++k) {
                        if (k == (size_t) kv[0]) {
                            plane[k] = 1.0;
                        }
                    }
                    plane.back() = -1.0 * kv[1].get<double>();
                    m_constraints[e].push_back(plane);
                    //std::cout << "upper: " << m_constraints[e].size();
                    mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
                }*/
                for (const auto &kv : bounds[e]["lower"]) {
                    mati.lower_bounds[kv[0].get<int>()] = kv[1].get<double>();
                    mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
                }
                for (const auto &kv : bounds[e]["upper"]) {
                    mati.upper_bounds[kv[0].get<int>()] = kv[1].get<double>();
                    mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
                }
                mati.local_bounds = true;
			}
	    }
	}
	else if (bounds.count("lower") && bounds.count("upper")) {
		// Box constraints
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
		for (size_t e = 0; e < mesh().numElements(); ++e) {
			auto &mati = m_matField->materialForElement(e);
			for (size_t k = 0; k < numVars; ++k) {
				mati.vars[k] = 0.0;
			}

			for (const auto &kv : bounds["lower"]) {
				HyperPlane plane;
				std::fill(plane.begin(), plane.end(), 0.0);
				for (size_t k = 0; k < numVars; ++k) {
					if (k == (size_t) kv[0]) {
						plane[k] = -1.0;
					}
				}
				plane.back() = kv[1];
				m_constraints[e].push_back(plane);
				//std::cout << "lower: " << m_constraints[e].size();
				mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
			}
			for (const auto &kv : bounds["upper"]) {
				HyperPlane plane;
				std::fill(plane.begin(), plane.end(), 0.0);
				for (size_t k = 0; k < numVars; ++k) {
					if (k == (size_t) kv[0]) {
						plane[k] = 1.0;
					}
				}
				plane.back() = -1.0 * kv[1].get<double>();
				m_constraints[e].push_back(plane);
				//std::cout << "upper: " << m_constraints[e].size();
				mati.vars[kv[0].get<int>()] += 0.5 * kv[1].get<double>();
			}
		}
	} else if (bounds.count("simplex")) {
		// Same simplex for everyone
		///  {
		///    "simplex": [
		///      [a_0, b_0, c_0],
		///      ...
		///      [a_i, b_i, c_i],
		///    ]
		///  }

		for (size_t e = 0; e < mesh().numElements(); ++e) {
            auto &mati = m_matField->materialForElement(e);
            for (size_t k = 0; k < numVars; ++k) {
                mati.vars[k] = 0.0;
            }

            auto cst = bounds["simplex"];
			Eigen::MatrixXd planes(cst.size(), cst[0].size());
			for (int i = 0; i < planes.rows(); ++i) {
				HyperPlane plane;
				for (int j = 0; j < planes.cols(); ++j) {
					plane[j] = planes(i, j) = cst[i][j];
				}
				m_constraints[e].push_back(plane);
			}
			Eigen::MatrixXd vertices;
			intersect_half_planes(planes, vertices);
			if (vertices.rows() == 0) {
				//logger().error("Found empty constraint set:\n{}", planes);
				//qfm_assert(false);
			}
			mati.vars[0] = vertices.col(0).mean();
			mati.vars[1] = vertices.col(1).mean();
        }
	} else if (bounds.count("simplex_per_element")) {
		// Different simplex for each element
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
		if (bounds["simplex_per_element"].size() != mesh().numElements()) {
			logger().error("Invalid number of constraints (should be one simplex per element)");
			//qfm_assert(false);
		}
		logger().debug("Using different simplex constraints per element");
		for (size_t e = 0; e < mesh().numElements(); ++e) {
			auto cst = bounds["simplex_per_element"][e];
			//qfm_assert_msg(cst.size() > 0, "Needs at least 1 constraint per element");
			Eigen::MatrixXd planes(cst.size(), cst[0].size());
			for (int i = 0; i < planes.rows(); ++i) {
				HyperPlane plane;
				for (int j = 0; j < planes.cols(); ++j) {
					plane[j] = planes(i, j) = cst[i][j];
				}
				m_constraints[e].push_back(plane);
			}
			Eigen::MatrixXd vertices;
			intersect_half_planes(planes, vertices);
			if (vertices.rows() == 0) {
				//logger().error("Found empty constraint set:\n{}", planes);
				//qfm_assert(false);
			}
			// There may be different constraints acting on a given material (e.g.,
			// coming from different elements). Here we assume that each material
			// is constrained by the same simplex (which is the case in our application),
			// so that we can initialize the material parameters by simply averaging the
			// simplex vertices coordinates.
			auto &mati = m_matField->materialForElement(e);
			mati.vars[0] = vertices.col(0).mean();
			mati.vars[1] = vertices.col(1).mean();
		}
		// throw std::runtime_error("Constraint not implemented yet: simplex_per_element");
	} else {
		throw std::runtime_error("Unrecognized material bounds");
	}
}

////////////////////////////////////////////////////////////////////////////////
// Explicit Instantiations
////////////////////////////////////////////////////////////////////////////////

//                                           Dim  Deg  Material
//template class IpoptOptimizer<Simulator<Mesh<2,   1,   Materials::Isotropic>>>;
//template class IpoptOptimizer<Simulator<Mesh<2,   2,   Materials::Isotropic>>>;
//template class IpoptOptimizer<Simulator<Mesh<3,   1,   Materials::Isotropic>>>;
//template class IpoptOptimizer<Simulator<Mesh<3,   2,   Materials::Isotropic>>>;

//typedef IpoptOptimizer<Simulator<Mesh<2,   1,   Materials::Isotropic>>> IpoptOptimizer2d1p;
//typedef IpoptOptimizer<Simulator<Mesh<2,   2,   Materials::Isotropic>>> IpoptOptimizer2d2p;
//typedef IpoptOptimizer<Simulator<Mesh<3,   1,   Materials::Isotropic>>> IpoptOptimizer3d1p;
//typedef IpoptOptimizer<Simulator<Mesh<3,   2,   Materials::Isotropic>>> IpoptOptimizer3d2p;

} // namespace MaterialOptimization

////////////////////////////////////////////////////////////////////////////////

namespace {

template<class Optimizer>
void MaterialOptimizationProblem<Optimizer>::set_variables(Index n, const Number *x)
{
	// Copy data from x to the material field.
	for (size_t mi = 0, i = 0; mi < opt().materialField().numMaterials(); ++mi) {
		assert(i < n);
		auto &mati = opt().materialField().material(mi);
		std::copy_n(x + i, Optimizer::Material::numVars, &(mati.vars[0]));
		//std::cout << mati.vars[0] << ", " << mati.vars[1] << std::endl;
		i += Optimizer::Material::numVars;
	}

	/*for (int i=0; i<n; i++) {
	    std::cout << "x[" << i << "]: " << x[i] << std::endl;
	}*/
}

template<class Optimizer>
void MaterialOptimizationProblem<Optimizer>::get_variables(Index n, Number *x) const
{
	// Copy data from x to the material field.
	for (size_t mi = 0, i = 0; mi < opt().materialField().numMaterials(); ++mi) {
		assert(i < n);
		const auto &mati = opt().materialField().material(mi);
		std::copy_n(&(mati.vars[0]), Optimizer::Material::numVars, x + i);
		i += Optimizer::Material::numVars;
	}
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::get_nlp_info(
	Index& n, Index& m, Index& nnz_jac_g,
	Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	// std::cout << "get_nlp_info" << std::endl;
	size_t numElements = opt().mesh().numElements();
	n = m_problem->NumParameters();
	assert(n == opt().materialField().numMaterials() * Optimizer::Material::numVars);
	m = 0;
	for (size_t e = 0; e < numElements; ++e) {
		// Some constraints might be redundant (e.g., if two elements are
		// assigned the same material id). But this should not impact the
		// optimization.
		m += opt().numConstraintsForElement(e);
	}

	// Optimization/dirty hack: keep one constraint per original quad
	//m /= 4; // HACK FOR QUAD->TRIANGLES REDUNDANCY

	nnz_jac_g = m * Optimizer::Material::numVars;
	nnz_h_lag = 0; // approximated
	index_style = TNLP::C_STYLE;
	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::get_bounds_info(
	Index n, Number* x_l, Number* x_u, Index m,
	Number* g_l, Number* g_u)
{
	// std::cout << "get_bounds_info" << std::endl;
	for (Index i = 0; i < n; ++i) {
		x_l[i] = std::numeric_limits<double>::lowest();
		x_u[i] = std::numeric_limits<double>::max();
	}

	// get global upper and lower bounds:
	auto g_upper = opt().materialField().material(0).upperBounds();
	auto g_lower = opt().materialField().material(0).lowerBounds();
	for (size_t mi = 0, i = 0; mi < opt().materialField().numMaterials(); ++mi) {
	    assert(i < n);
		const auto &mati = opt().materialField().material(mi);

		if (mati.local_bounds) {
            std::copy_n(&(mati.lower_bounds[0]), Optimizer::Material::numVars, x_l + i);
            std::copy_n(&(mati.upper_bounds[0]), Optimizer::Material::numVars, x_u + i);
        }

		i += Optimizer::Material::numVars;
	}

	for (Index i = 0; i < m; ++i) {
		g_l[i] = std::numeric_limits<double>::lowest();
		g_u[i] = 0.0;
	}

	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::get_starting_point(
	Index n, bool init_x, Number* x,
	bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda,
	Number* lambda)
{
	// std::cout << "get_starting_point" << std::endl;

	// Here, we assume we only have starting values for x, if you code
	// your own NLP, you can provide starting values for the dual variables
	// if you wish to use a warm start option
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

	// Initialize to the given starting point
	get_variables(n, x);

	Eigen::VectorXd g(m);
	eval_g(n, x, true, m, g.data());

	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::eval_f(
	Index n, const Number* x, bool new_x, Number& obj_value)
{
	//std::cout << "eval_f" << std::endl;
	set_variables(n, x);
	m_problem->Evaluate(ceres::Problem::EvaluateOptions(), &obj_value, nullptr, nullptr, nullptr);
	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::eval_grad_f(
	Index n, const Number* x, bool new_x, Number* grad_f)
{
	// std::cout << "eval_grad_f" << std::endl;
	set_variables(n, x);
	m_grad_f.resize(n);
	m_problem->Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, &m_grad_f, nullptr);
	std::copy_n(m_grad_f.data(), n, grad_f);
	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::eval_g(
	Index n, const Number* x, bool new_x, Index m, Number* g)
{
	//std::cout << "eval_g" << std::endl;
	set_variables(n, x);
	size_t numElements = opt().mesh().numElements();
	for (size_t e = 0, i = 0; e < numElements; ++e) {
		//if (e % 4 == 0) // HACK FOR QUAD->TRIANGLES REDUNDANCY
		for (size_t lc = 0; lc < opt().numConstraintsForElement(e); ++lc) {
			g[i++] = opt().evalConstraint(e, lc, &(opt().materialField().materialForElement(e).vars[0]));
		}
	}
	return true;
}

template<class Optimizer>
bool MaterialOptimizationProblem<Optimizer>::eval_jac_g(
	Index n, const Number* x, bool new_x, Index m,
	Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
	// std::cout << "eval_jac_g" << std::endl;
	if (values == nullptr) {
		// sparse Jacobian
		size_t numElements = opt().mesh().numElements();
		for (size_t e = 0, current_row = 0; e < numElements; ++e) {
			//if (e % 4 == 0) // HACK FOR QUAD->TRIANGLES REDUNDANCY
			for (size_t lc = 0; lc < opt().numConstraintsForElement(e); ++lc) {
				auto mat_idx = opt().materialField().materialIndexForElement(e);
				for (size_t k = 0; k < Optimizer::Material::numVars; ++k) {
					*(iRow++) = current_row;
					*(jCol++) = mat_idx * Optimizer::Material::numVars + k;
				}
				++current_row;
			}
		}
	} else {
		// return the values of the Jacobian of the constraints
		set_variables(n, x);
		size_t numElements = opt().mesh().numElements();
		for (size_t e = 0; e < numElements; ++e) {
			//if (e % 4 == 0) // HACK FOR QUAD->TRIANGLES REDUNDANCY
			for (size_t lc = 0; lc < opt().numConstraintsForElement(e); ++lc) {
				opt().evalConstraintGrad(e, lc,
					&(opt().materialField().materialForElement(e).vars[0]), values);
				values += Optimizer::Material::numVars;
			}
		}
	}

	return true;
}

template<class Optimizer>
void MaterialOptimizationProblem<Optimizer>::finalize_solution(
	Ipopt::SolverReturn status, Index n,
	const Number* x, const Number* z_L, const Number* z_U, Index m,
	const Number* g, const Number* lambda, Number obj_value,
	const Ipopt::IpoptData* ip_data,
	Ipopt::IpoptCalculatedQuantities* ip_cq)
{
	// std::cout << "finalize_solution" << std::endl;
	set_variables(n, x);
}

//template class MaterialOptimizationProblem<MaterialOptimization::IpoptOptimizer2d1p>;
//template class MaterialOptimizationProblem<MaterialOptimization::IpoptOptimizer2d2p>;
//template class MaterialOptimizationProblem<MaterialOptimization::IpoptOptimizer3d1p>;
//template class MaterialOptimizationProblem<MaterialOptimization::IpoptOptimizer3d2p>;

} // anonymous namespace

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
	const nlohmann::json &ipoptConfig = {},
	const std::string &intermediateResults = "",
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg)
{
	typedef MaterialOptimization::Mesh<_N, _FEMDegree, _Material> Mesh;
	typedef MaterialOptimization::Simulator<Mesh>           Simulator;
	typedef MaterialOptimization::IpoptOptimizer<Simulator> Opt;

	typedef typename Opt::MField MField;

	// Material bounds are global variables
	// _Material<_N>::setBoundsFromJson(materialBounds);

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
	matOpt.readMaterialConstraints(materialBounds);

	std::cout << "Opening: " << intermediateResults << std::endl;
	MSHFieldWriter writer(intermediateResults, matOpt.mesh());

	logger().debug("Attempting optimization");
	matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
		regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet, ipoptConfig, multMode);

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
/// @tparam     _Material                Material type.
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
	const nlohmann::json &ipoptConfig = {},
	const std::string &intermediateResults = "",
	size_t shear_degree = 0,
	std::vector<double> shear_coeffs = std::vector<double>(),
	Real angle = 0.0,
	MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg)
{
	typedef MaterialOptimization::Mesh<_N, _FEMDegree, Materials::ConstrainedCubic> Mesh;
	typedef MaterialOptimization::Simulator<Mesh>           Simulator;
	typedef MaterialOptimization::IpoptOptimizer<Simulator> Opt;

	typedef typename Opt::MField MField;

	// Material bounds are global variables
	// Materials::ConstrainedCubic<_N>::setBoundsFromJson(materialBounds);
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
	matOpt.readMaterialConstraints(materialBounds);

	std::cout << "Opening: " << intermediateResults << std::endl;
	MSHFieldWriter writer(intermediateResults, matOpt.mesh());

	logger().debug("Attempting optimization");
	matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
		regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet, ipoptConfig);

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

void material_optimization_ipopt(
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
	const nlohmann::json &ipoptConfig,
	const std::string &intermediateResults,
	const std::string materialType,
	size_t shear_degree,
	std::vector<double> shear_coeffs,
	double angle,
	MaterialOptimization::RegularizationMultMode multMode)
{
    std::cout << "Material type: " << materialType << std::endl;

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
             ipoptConfig,
             intermediateResults,
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
             ipoptConfig,
             intermediateResults,
             multMode);
    }
}
