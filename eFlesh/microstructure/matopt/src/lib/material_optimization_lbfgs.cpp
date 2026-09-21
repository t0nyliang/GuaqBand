#include "material_optimization_lbfgs.h"
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
#include <Eigen/Sparse>
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
#include <LBFGSB.h>

using namespace LBFGSpp;

namespace MaterialOptimization {

    template<class _Simulator>
    class LBFGSOptimizer {
    public:
        static constexpr size_t N = _Simulator::N;
        static constexpr size_t K = _Simulator::K;
        static constexpr size_t Degree = _Simulator::Degree;

        typedef typename _Simulator::SField SField;
        typedef typename _Simulator::VField VField;
        typedef typename _Simulator::SMField SMField;
        typedef typename _Simulator::SMatrix SMatrix;
        typedef typename _Simulator::ETensor ETensor;
        typedef typename _Simulator::_Point _Point;
        typedef typename _Simulator::MField MField;
        typedef typename MField::Material Material;

        template<typename Elems, typename Vertices>
        LBFGSOptimizer(Elems inElems, Vertices inVertices,
                    std::shared_ptr <MField> matField,
                    const std::vector <CondPtr<N>> &boundaryConditions,
                    bool noRigidMotion, double weight_elasticity = 1.0, double weight_bounds = 1.0, double weight_regularization = 1.0,
                    double weight_smoothing = 1.0)
                : m_sim(inElems, inVertices, matField), m_matField(matField), m_boundaryConditions(boundaryConditions),
                m_weight_elasticity(weight_elasticity), m_weight_bounds(weight_bounds), m_weight_regularization(weight_regularization),
                m_weight_smoothing(weight_smoothing){
            m_sim.applyBoundaryConditions(boundaryConditions);
            if (noRigidMotion)
                m_sim.applyNoRigidMotionConstraint();

            for (unsigned i = 0; i < boundaryConditions.size(); i++) {
                CondPtr<N> cond = boundaryConditions[i];
                BoundaryCondition<N> *new_cond;
                if (const DirichletCondition<N> * dc = dynamic_cast<const DirichletCondition<N> *>(cond.get())) {
                    if (auto tc = dynamic_cast<const TargetCondition<N> *>(cond.get()))
                        continue;
                    if (auto tnc = dynamic_cast<const TargetNodesCondition<N> *>(cond.get()))
                        continue;
                    std::cout << "Adding dirichlet!" << std::endl;
                    VectorND<N> zero_vector;
                    zero_vector.setZero();
                    new_cond = new DirichletCondition<N>(dc->region, zero_vector, dc->componentMask);

                    m_adjointBoundaryConditions.push_back(CondPtr<N>(new_cond));
                }
            }
        }

        double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad)
        {
            auto neumannLoad = m_sim.neumannLoad();

            // Compute u
            m_matField->setVars(x);
            m_sim.materialFieldUpdated();
            auto u = m_sim.solve(neumannLoad);

            // Compute objective
            double fx = computeObjective(u, m_weight_elasticity, m_weight_bounds, m_weight_regularization, m_weight_smoothing);

            // compute gradient
            computeObjectiveGradient(u, grad, m_weight_elasticity, m_weight_bounds, m_weight_regularization, m_weight_smoothing);

            //Eigen::VectorXd agrad = Eigen::VectorXd::Zero(m_matField->numVars());
            //approximateObjectiveGradient(u, agrad, m_weight_elasticity, m_weight_bounds, m_weight_regularization, m_weight_smoothing);

            return fx;
        }

        VField currentDisplacement() const {
            return m_sim.solve();
        }

        Real computeObjective(const VField &u, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing = 1.0) {
            Real result = 0.0;
            Real result_elasticity = 0.0;
            Real result_bounds = 0.0;
            Real result_regularization = 0.0;
            Real result_smoothing = 0.0;

            if (weight_elasticity > 0.0) {
                result_elasticity = weight_elasticity * computeElasticityObjective(u);
            }

            result_bounds = weight_bounds * computeBoundsObjective();
            result_regularization = weight_regularization * computeRegularizationObjective();

            if (weight_smoothing > 0.0) {
                result_smoothing = weight_smoothing * computeSmoothingObjective(u);
            }

            result = result_elasticity + result_bounds + result_regularization + result_smoothing;

            logger().debug("objectives (total, elasticity, bounds, regularization, smoothing):\t{}\t{}\t{}\t{}\t{}", result, result_elasticity, result_bounds, result_regularization, result_smoothing);

            return result;
        }

        void computeObjectiveGradient(const VField &u, Eigen::VectorXd &g, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing = 1.0) {
            Eigen::VectorXd result_elasticity = Eigen::VectorXd::Zero(m_matField->numVars()) ;
            Eigen::VectorXd result_bounds = Eigen::VectorXd::Zero(m_matField->numVars()) ;
            Eigen::VectorXd result_regularization = Eigen::VectorXd::Zero(m_matField->numVars()) ;
            Eigen::VectorXd result_smoothing = Eigen::VectorXd::Zero(m_matField->numVars()) ;

            if (weight_elasticity > 0.0) {
                auto adjoint_rhs = m_sim.computeAdjointRhs(u);
                auto lambda = solveAdjoint(adjoint_rhs);
                computeElasticityObjectiveGradient(u, lambda, result_elasticity);
            }
            computeBoundsObjectiveGradient(result_bounds);
            computeRegularizationObjectiveGradient(result_regularization);
            if (weight_smoothing > 0.0) {
                auto adjoint_rhs = computeSmoothingAdjointRhs(u);
                auto lambda = solveAdjoint(adjoint_rhs);
                computeSmoothingObjectiveGradient(u, lambda, result_smoothing);
            }

            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                g(var) = weight_elasticity * result_elasticity(var) + weight_bounds * result_bounds(var) +
                        weight_regularization * result_regularization(var) + weight_smoothing * result_smoothing(var);
            }

            //logger().debug("gradient (total, elasticity, bounds, regularization):\t{}\t{}\t{}\t{}", g.norm(),
            //        weight_elasticity*result_elasticity.norm(), weight_bounds*result_bounds.norm(), weight_regularization*result_regularization.norm(),
            //        weight_smoothing*result_smoothing.norm());

            return;
        }

        VField solveAdjoint(const VField &adjoint_rhs) {
            // Prepare to solve adjoint
            m_sim.removeDirichletConditions();
            m_sim.applyBoundaryConditions(m_adjointBoundaryConditions);

            // Solve adjoint
            auto lambda = m_sim.solve(adjoint_rhs);

            // Reapply old conditions
            //m_sim.removeAllBoundaryConditions();
            m_sim.removeDirichletConditions();
            //m_sim.removeTargets();
            m_sim.applyBoundaryConditions(m_boundaryConditions);

            return lambda;
        }

        // 1/2 int_bdry ||u - t||^2 dA = 1/2 int_bdry ||d||^2 dA
        // where d = componentMask * (u - t) is the component-masked
        // distance-to-target vector field (linearly/quadratically interpolated over
        // each boundary element).
        Real computeElasticityObjective(const VField &u) const {
            Real obj = 0;
            Interpolant < _Point, K - 1, Degree > dist;
            for (size_t bei = 0; bei < m_sim.mesh().numBoundaryElements(); ++bei) {
                auto be = m_sim.mesh().boundaryElement(bei);
                for (size_t i = 0; i < be.numNodes(); ++i) {
                    auto bn = be.node(i);
                    dist[i] = bn->targetComponents.apply((u(bn.volumeNode().index())
                                                          - bn->targetDisplacement).eval());
                }
                obj += Quadrature<K - 1, 4>::integrate(
                        [&](const EvalPt<K - 1> &p) { return dist(p).dot(dist(p)); }, be->volume());
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
        void computeElasticityObjectiveGradient(const VField &u, const VField &lambda, Eigen::VectorXd &g) {
            std::vector <size_t> elems;
            typename _Simulator::Strain e_u, e_lambda;
            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                // Support of dE/dp on the mesh.
                m_matField->getInfluenceRegion(var, elems);
                ETensor dE;
                m_matField->getETensorDerivative(var, dE);
                for (size_t i = 0; i < elems.size(); ++i) {
                    size_t ei = elems[i];
                    auto e = m_sim.mesh().element(ei);
                    m_sim.elementStrain(ei, u, e_u);
                    m_sim.elementStrain(ei, lambda, e_lambda);
                    g(var) += Quadrature<K, 6>::integrate(
                            [&](const EvalPt <K> &p) {
                                return dE.doubleContract(e_u(p))
                                        .doubleContract(e_lambda(p));
                            },
                            e->volume());
                }
            }

            return;
        }


        void approximateObjectiveGradient(const VField &u, Eigen::VectorXd &g, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing=1.0) {
            double epsilon = 1e-6;

            double curValue = computeObjective(u, weight_elasticity, weight_bounds, weight_regularization, weight_smoothing);

            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                size_t param;
                size_t matIdx;
                variableRole(var, matIdx, param);

                // Compute perturbed value
                m_matField->material(matIdx).vars[param] += epsilon;
                VField perU;
                if (weight_elasticity > 0.0) {
                    m_sim.materialFieldUpdated();
                    perU = m_sim.solve(this->m_sim.neumannLoad());
                }
                double perValue = computeObjective(perU, weight_elasticity, weight_bounds, weight_regularization, weight_smoothing);

                // centered finite difference
                g(var) = (perValue - curValue) / (epsilon);

                // go back
                m_matField->material(matIdx).vars[param] -= epsilon;
            }

            logger().debug("approximate gradient :\t{}", g.norm());

            return;
        }

        // Number of linear constraints for a given element
        size_t numConstraintsForElement(size_t elem) const {
            return m_constraints[elem].size();
        }

        Real computeBoundsObjective(Real epsilon = 1e-4) {
            double result = 0.0;

            for (size_t m = 0; m < m_matField->numMaterials(); ++m) {
                auto mat = m_matField->material(m);
                std::vector<size_t> elems;
                m_matField->getInfluenceRegion(m*Material::numVars, elems);
                double mat_penalization = 0.0;
                for (size_t lc = 0; lc < numConstraintsForElement(elems[0]); ++lc) {
                    const HyperPlane &plane = m_constraints[elems[0]][lc];
                    double plane_value = plane.back();
                    for (size_t k = 0; k < Material::numVars; ++k) {
                        plane_value += plane[k] * mat.vars[k];
                    }

                    //std::cout << "plane value: " << plane_value << std::endl;

                    if (plane_value + epsilon > 0.0) {
                        Real corrected_value = (plane_value+epsilon);
                        mat_penalization += corrected_value * corrected_value;
                    }
                }
                result += mat_penalization;
            }

            return result;
        }

        void computeBoundsObjectiveGradient(Eigen::VectorXd &g, Real epsilon = 1e-4) {
            for (size_t m = 0; m < m_matField->numMaterials(); ++m) {
                auto mat = m_matField->material(m);
                std::vector<size_t> elems;
                m_matField->getInfluenceRegion(m*Material::numVars, elems);
                size_t e = elems[0]; // any element is ok, since they are suppose to have the same boundaries
                for (size_t lc = 0; lc < numConstraintsForElement(e); ++lc) {
                    const HyperPlane &plane = m_constraints[e][lc];
                    double plane_value = plane.back();
                    for (size_t k = 0; k < Material::numVars; ++k) {
                        plane_value += plane[k] * mat.vars[k];
                    }

                    if (plane_value + epsilon > 0.0) {
                        Real corrected_value = (plane_value+epsilon);
                        for (size_t k = 0; k < Material::numVars; ++k) {
                            g(m*Material::numVars + k) += 2 * corrected_value * plane[k];
                        }
                    }
                }
            }

            return;
        }

        Real computeRegularizationObjective() {
            Real result = 0.0;

            // Get "material graph" adjacences for Laplacian (smoothness) regularization
	        std::vector<std::set<size_t> > materialAdj;
	        m_matField->materialAdjacencies(mesh(), materialAdj);

	        // Add in variable bounds and regularization (if requested)
            for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
                auto &mati = m_matField->material(mi);

                for (size_t mj : materialAdj.at(mi)) {
                    // Make sure graph is undirected.
                    assert(materialAdj.at(mj).find(mi) != materialAdj.at(mj).end());
                    // Add one term per edge, not two
                    if (mi < mj) continue;

                    auto &matj = m_matField->material(mj);
                    for (size_t v = 0; v < Material::numVars; ++v)
			            result += (mati.vars[v] - matj.vars[v]) * (mati.vars[v] - matj.vars[v]);
                }
            }

            return result;
        }


        void computeRegularizationObjectiveGradient(Eigen::VectorXd &g) {
            // Get "material graph" adjacences for Laplacian (smoothness) regularization
	        std::vector<std::set<size_t> > materialAdj;
	        m_matField->materialAdjacencies(mesh(), materialAdj);

	        // Add in variable bounds and regularization (if requested)
            for (size_t mi = 0; mi < m_matField->numMaterials(); ++mi) {
                auto &mati = m_matField->material(mi);

                for (size_t mj : materialAdj.at(mi)) {
                    // Make sure graph is undirected.
                    assert(materialAdj.at(mj).find(mi) != materialAdj.at(mj).end());
                    // Add one term per edge, not two
                    if (mi < mj) continue;

                    auto &matj = m_matField->material(mj);
                    for (size_t v = 0; v < Material::numVars; ++v) {
                        g(mi*Material::numVars + v) += 2.0 * (mati.vars[v] - matj.vars[v]);
                        g(mj*Material::numVars + v) += -2.0 * (mati.vars[v] - matj.vars[v]);
                    }
                }
            }

            return;
        }

        Eigen::SparseMatrix<Real> computeLaplacianMatrix() {
            //std::cout << "Laplacian Matrix" << std::endl;
            std::vector<std::set<size_t>> adj(m_sim.mesh().numVertices());
            for (auto be : m_sim.mesh().boundaryElements()) {
                for (auto v1 : be.vertices()) {
                    size_t v1Index = v1.node().volumeNode().vertex().index();

                    for (auto v2 : be.vertices()) {
                        size_t v2Index = v2.node().volumeNode().vertex().index();

                        if (v2 == v1)
                            continue;
                        else if (!adj[v1Index].count(v2Index)) {
                            adj[v1Index].insert(v2Index);
                        }
                    }
                }
            }

            int numVertices = m_sim.mesh().numVertices();
            Eigen::SparseMatrix<Real> L(numVertices, numVertices);
            std::vector<Eigen::Triplet<Real>> triplets;
            for (auto ve : m_sim.mesh().boundaryVertices()) {
                size_t veIndex = ve.node().volumeNode().vertex().index();
                triplets.push_back(Eigen::Triplet<Real>(veIndex, veIndex, -1.0));

                for (auto neighbor : adj[veIndex]) {
                    triplets.push_back(Eigen::Triplet<Real>(veIndex, neighbor, 1.0 / adj[veIndex].size()));
                }
            }

            L.setFromTriplets(triplets.begin(), triplets.end());

            //std::cout << "Laplacian Matrix" << std::endl;
            return L;
        }

        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> computePositionMatrix(const VField &u) {
            //std::cout << "Position Matrix" << std::endl;
            int numVertices = m_sim.mesh().numVertices();
            Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> p(numVertices, N);
            p.fill(0.0);

            //std::cout << "p: " << p.rows() << ", " << p.cols() << std::endl;
            //std::cout << "u: " << u.domainSize() << ", " << u.dim() << std::endl;
            //std::cout << "v: " << numVertices;
            for (auto v : m_sim.mesh().vertices()) {
                size_t vNodeIndex = v.node().volumeNode().index();
                //p.row(v.index()) = v.node()->p + u(vNodeIndex);
                p.row(v.index()) = u(vNodeIndex);
            }

            //std::cout << "Position Matrix" << std::endl;
            return p;
        }

        Real computeSmoothingObjective(const VField &u) {
            Real result = 0.0;

            //std::cout << "Computing matrices" << std::endl;
            auto L = computeLaplacianMatrix();
            auto p = computePositionMatrix(u);

            //std::cout << "adding values to result" << std::endl;
            Eigen::MatrixXd resultingMatrix = L * p;
            for (unsigned i=0; i<resultingMatrix.rows(); i++) {
                result += 0.5 * resultingMatrix.row(i).squaredNorm();
            }

            //std::cout << "Smoothing result: " << result << std::endl;

            return result;
        }

        void computeSmoothingObjectiveGradient(const VField &u, const VField &lambda, Eigen::VectorXd &g) {
            std::vector <size_t> elems;
            typename _Simulator::Strain e_u, e_lambda;
            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                // Support of dE/dp on the mesh.
                m_matField->getInfluenceRegion(var, elems);
                ETensor dE;
                m_matField->getETensorDerivative(var, dE);
                for (size_t i = 0; i < elems.size(); ++i) {
                    size_t ei = elems[i];
                    auto e = m_sim.mesh().element(ei);
                    m_sim.elementStrain(ei, u, e_u);
                    m_sim.elementStrain(ei, lambda, e_lambda);
                    g(var) += Quadrature<K, 6>::integrate(
                            [&](const EvalPt <K> &p) {
                                return dE.doubleContract(e_u(p))
                                        .doubleContract(e_lambda(p));
                            },
                            e->volume());
                }
            }
        }

        VField computeSmoothingAdjointRhs(const VField &u) {
            VField dofLoad(m_sim.numDoFs());
            dofLoad.clear();

            auto L = computeLaplacianMatrix();
            auto p = computePositionMatrix(u);

            auto force = - p.transpose() * L.transpose() * L;

            for (auto v : m_sim.mesh().vertices()) {
                size_t vNodeIndex = v.node().volumeNode().index();
                dofLoad(m_sim.DoF(vNodeIndex)) = force.col(v.index());
            }

            return dofLoad;
        }

        SField computeSmoothingField(const VField &u) {
            // Computing adjacency list for each vertex
            std::vector<std::set<size_t>> adj(m_sim.mesh().numVertices());
            for (auto be : m_sim.mesh().boundaryElements()) {
                for (auto v1 : be.vertices()) {
                    size_t v1Index = v1.node().volumeNode().vertex().index();

                    for (auto v2 : be.vertices()) {
                        size_t v2Index = v2.node().volumeNode().vertex().index();

                        if (v2 == v1)
                            continue;
                        else if (!adj[v1Index].count(v2Index)) {
                            adj[v1Index].insert(v2Index);
                        }
                    }
                }
            }

            int numVertices = m_sim.mesh().numVertices();
            std::vector<Real> result(numVertices);
            for (auto ve : m_sim.mesh().boundaryVertices()) {
                size_t veIndex = ve.node().volumeNode().vertex().index();
                size_t veNodeIndex = ve.node().volumeNode().index();
                Eigen::Matrix<Real, N, 1> v = ve.node().volumeNode()->p;
                Eigen::Matrix<Real, N, 1> value = Eigen::MatrixXd::Zero(N, 1);

                for (auto neighbor : adj[veIndex]) {
                    size_t neiNodeIndex = m_sim.mesh().vertex(neighbor).node().index();
                    Eigen::Matrix<Real, N, 1> w = m_sim.mesh().vertex(neighbor).node()->p;
                    Eigen::Matrix<Real, N, 1> diff = w + u(veNodeIndex) - v - u(neiNodeIndex);
                    Eigen::Matrix<Real, N, 1> diffVertices = w - v;

                    value += diff / diffVertices.norm();
                }

                result[veIndex] = value.squaredNorm();
            }

            return result;
        }

        void readMaterialConstraints(const nlohmann::json &bounds);

        const typename _Simulator::Mesh &mesh() const { return m_sim.mesh(); }

        const _Simulator &simulator() const { return m_sim; }

        const MField &materialField() const { return *m_matField; }

        MField &materialField() { return *m_matField; }

        void variableRole(size_t var, size_t &matIdx, size_t &param) const {
            matIdx = var / Material::numVars;
            param  = var % Material::numVars;
        }

        void writeDisplacement(MSHFieldWriter &writer) {
            auto u = m_sim.solve(m_sim.neumannLoad());
            writer.addField("u", u, DomainType::PER_NODE);

            m_sim.addTargetsToDirichlet();
			auto ud = m_sim.solve(m_sim.neumannLoad());
			writer.addField("target", ud, DomainType::PER_NODE);
			m_sim.removeTargetsFromDirichlet();
        }

    private:
        _Simulator m_sim;
        std::shared_ptr<typename _Simulator::MField> m_matField;
        const std::vector <CondPtr<N>> m_boundaryConditions;
        std::vector <CondPtr<N>> m_adjointBoundaryConditions;

        // Linear constraints for each element of the mesh.
        // Each element has h_e linear constraints on its material properties.
        // Each linear constraint is a tuple {c_k}_k representing the hyperplane
        // \sum_{k=0}^{H-2} c_k * mat_k + c_{H-1} <= 0

        typedef std::array<double, Material::numVars + 1> HyperPlane;
        std::vector <std::vector<HyperPlane>> m_constraints;

        double m_weight_regularization;
        double m_weight_bounds;
        double m_weight_elasticity;
        double m_weight_smoothing;
    };


    template<class _Simulator>
    void LBFGSOptimizer<_Simulator>::readMaterialConstraints(const nlohmann::json &bounds) {
        m_constraints.assign(mesh().numElements(), {});

        auto numVars = Material::numVars;
        if (bounds.is_array()) {
            //throw std::runtime_error("Constraint not implemented yet: lower/upper per element");

            for (size_t e = 0; e < mesh().numElements(); ++e) {
                auto &mati = m_matField->materialForElement(e);
                for (size_t k = 0; k < numVars; ++k) {
                    mati.vars[k] = 0.0;
                    mati.lower_bounds[k] = std::numeric_limits<double>::lowest();
                    mati.upper_bounds[k] = std::numeric_limits<double>::max();
                }
                mati.lower_bounds[0] = 0.005;
                mati.upper_bounds[0] = 1.0;

                mati.lower_bounds[1] = -0.99;
                mati.upper_bounds[1] = 0.99;

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
                } else {
                    for (size_t k = 0; k < numVars; ++k) {
                        mati.vars[k] = 0.0;
                    }

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
                }

                //mati.vars[0] = 0.3 + 0.1*(double)rand() / RAND_MAX;
                //mati.vars[1] = 0.1 + 0.1*(double)rand() / RAND_MAX;
                //mati.vars[0] = 0.3;
                //mati.vars[1] = 0.1;
            }
        } else if (bounds.count("lower") && bounds.count("upper")) {
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
                    mati.lower_bounds[kv[0].get<int>()] = kv[1].get<double>();
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
                    mati.upper_bounds[kv[0].get<int>()] = kv[1].get<double>();
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
}

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
            const std::vector <MeshIO::IOVertex> &inVertices,
            const std::vector <MeshIO::IOElement> &inElements,
            std::vector <size_t> matIdxForElement,
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
            MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg,
            std::vector<double> initial_guess = std::vector<double>(),
            double elasticityWeight = 1.0,
            double boundsWeight = 1.0,
            double smoothingWeight = 1.0,
            std::string solution_info = "") {
        typedef MaterialOptimization::Mesh<_N, _FEMDegree, _Material> Mesh;
        typedef MaterialOptimization::Simulator<Mesh> Simulator;
        typedef MaterialOptimization::LBFGSOptimizer<Simulator> Opt;

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
        auto bconds = readBoundaryConditions<_N>(iss, BBox<VectorND<_N>>(inVertices), noRigidMotion);

        Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion, elasticityWeight, boundsWeight, regularizationWeight, smoothingWeight);
        matOpt.readMaterialConstraints(materialBounds);

        if (initial_guess.size() == matField->numVars()) {
            matField->setVars(initial_guess);
        }

        std::cout << "Opening: " << intermediateResults << std::endl;
        MSHFieldWriter writer(intermediateResults, matOpt.mesh());

        logger().debug("Attempting optimization");
        //matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet, regularizationWeight,
        //        anisotropyPenaltyWeight, noRigidMotionDirichlet, multMode, 1.0, 1e4);

        LBFGSBParam<double> param;
        param.epsilon = 1e-10; // tolerance for convergence test
        param.past = 0.0; // distance for delta-based convergence test (distance between consecutive objective functions)
        param.wolfe = 0.9999; // coefficient for the Wolfe condition
        param.max_linesearch = 100; // maximum number of iterations in linesearch
        param.max_iterations = iterations; // maximum number of iterations
        LBFGSBSolver<double> solver(param);

        // Variable bounds
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(matField->numVars(), -std::numeric_limits<double>::infinity());
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(matField->numVars(),  std::numeric_limits<double>::infinity());
        Eigen::VectorXd  x = Eigen::VectorXd::Zero(matField->numVars());
        for (int m=0; m<matField->numMaterials(); m++) {
            auto &mat = matField->material(m);
            for (size_t k = 0; k < _Material<_N>::numVars; ++k) {
                lb(m*_Material<_N>::numVars + k) = mat.lower_bounds[k];
                ub(m*_Material<_N>::numVars + k) = mat.upper_bounds[k];
                x(m*_Material<_N>::numVars + k)  = mat.vars[k];
            }
        }

        //std::cout << "lb: " << lb << std::endl;
        //std::cout << "ub: " << ub << std::endl;
        //
        // std::cout << "x: " << x << std::endl;
        double fx;
        int niter = solver.minimize(matOpt, x, fx, lb, ub);

        std::cout << niter << " iterations" << std::endl;
        //std::cout << "x = \n" << x.transpose() << std::endl;
        std::cout << "f(x) = " << fx << std::endl;
        //std::cout << "grad = " << solver.final_grad().transpose() << std::endl;
        std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;

        matField->setVars(x);
        matField->writeVariableFields(writer, "Final ");

        matOpt.writeDisplacement(writer);

        // Write solution info:
        if (solution_info != "") {
            nlohmann::json j;

            j["x"] = x;
            j["f(x)"] = fx;
            j["||grad||"] = solver.final_grad_norm();
            j["iterations"] = niter;

            std::ofstream o(solution_info);
            o << std::setw(4) << j << std::endl;
        }

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
        const std::string &intermediateResults = "",
        size_t shear_degree = 0,
        std::vector<double> shear_coeffs = std::vector<double>(),
        Real angle = 0.0,
        MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg,
        std::vector<double> initial_guess = std::vector<double>(),
        double elasticityWeight = 1.0,
        double boundsWeight = 1.0,
        double smoothingWeight = 1.0,
        std::string solution_info = "")
    {
        typedef MaterialOptimization::Mesh<_N, _FEMDegree, Materials::ConstrainedCubic> Mesh;
	    typedef MaterialOptimization::Simulator<Mesh>           Simulator;
        typedef MaterialOptimization::LBFGSOptimizer<Simulator> Opt;

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

        logger().debug("Attempting optimization");
        Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion, elasticityWeight, boundsWeight, regularizationWeight, smoothingWeight);
        matOpt.readMaterialConstraints(materialBounds);

        if (initial_guess.size() == matField->numVars()) {
            matField->setVars(initial_guess);
        }

        std::cout << "Opening: " << intermediateResults << std::endl;
        MSHFieldWriter writer(intermediateResults, matOpt.mesh());

        logger().debug("Attempting optimization");
        //matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet, regularizationWeight,
        //        anisotropyPenaltyWeight, noRigidMotionDirichlet, multMode, 1.0, 1e4);

        LBFGSBParam<double> param;
        param.epsilon = 1e-10; // tolerance for convergence test
        param.past = 0.0; // distance for delta-based convergence test (distance between consecutive objective functions)
        param.wolfe = 0.9999; // coefficient for the Wolfe condition
        param.max_linesearch = 100; // maximum number of iterations in linesearch
        param.max_iterations = iterations; // maximum number of iterations
        LBFGSBSolver<double> solver(param);

        // Variable bounds
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(matField->numVars(), -std::numeric_limits<double>::infinity());
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(matField->numVars(),  std::numeric_limits<double>::infinity());
        Eigen::VectorXd  x = Eigen::VectorXd::Zero(matField->numVars());
        size_t numv = Materials::ConstrainedCubic<_N>::numVars;
        for (int m=0; m<matField->numMaterials(); m++) {
            auto &mat = matField->material(m);
            for (size_t k = 0; k < Materials::ConstrainedCubic<_N>::numVars; ++k) {
                lb(m*numv + k) = mat.lower_bounds[k];
                ub(m*numv + k) = mat.upper_bounds[k];
                x(m*numv + k)  = mat.vars[k];
            }
        }

        //std::cout << "lb: " << lb << std::endl;
        //std::cout << "ub: " << ub << std::endl;
        //
        // std::cout << "x: " << x << std::endl;
        double fx;
        int niter = solver.minimize(matOpt, x, fx, lb, ub);

        std::cout << niter << " iterations" << std::endl;
        //std::cout << "x = \n" << x.transpose() << std::endl;
        std::cout << "f(x) = " << fx << std::endl;
        // std::cout << "grad = " << solver.final_grad().transpose() << std::endl;
        std::cout << "||grad|| = " << solver.final_grad_norm() << std::endl;

        matField->setVars(x);
        matField->writeVariableFields(writer, "Final ");

        matOpt.writeDisplacement(writer);

        // Write solution info:
        if (solution_info != "") {
            nlohmann::json j;

            j["x"] = x;
            j["f(x)"] = fx;
            j["||grad||"] = solver.final_grad_norm();
            j["iterations"] = niter;

            std::ofstream o(solution_info);
            o << std::setw(4) << j << std::endl;
        }

        // Extract material parameters
        optimizedParameters.resize(inElements.size(), numv);
        for (size_t f = 0; f < inElements.size(); ++f) {
            auto mat = matField->materialForElement(f);
            for (int c = 0; c < numv; ++c) {
                optimizedParameters(f, c) = mat.vars[c];
            }
        }
    }

}

////////////////////////////////////////////////////////////////////////////////

void material_optimization_lbfgs(
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
	const std::string &intermediateResults,
	const std::string materialType,
	size_t shear_degree,
	std::vector<double> shear_coeffs,
	double angle,
	MaterialOptimization::RegularizationMultMode multMode,
	std::vector<double> initial_guess,
	double elasticityWeight,
    double boundsWeight,
    double smoothingWeight,
    std::string solution_info)
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
             intermediateResults,
             shear_degree,
             shear_coeffs,
             angle,
             multMode,
             initial_guess,
             elasticityWeight,
             boundsWeight,
             smoothingWeight,
             solution_info);
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
             intermediateResults,
             multMode,
             initial_guess,
             elasticityWeight,
             boundsWeight,
             smoothingWeight,
             solution_info);
    }
}