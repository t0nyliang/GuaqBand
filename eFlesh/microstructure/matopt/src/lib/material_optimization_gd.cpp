#include "material_optimization_gd.h"
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
#include <Eigen/Sparse>
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


namespace MaterialOptimization {

    template<class _Simulator>
    class GDOptimizer {
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
        GDOptimizer(Elems inElems, Vertices inVertices,
                    std::shared_ptr <MField> matField,
                    const std::vector <CondPtr<N>> &boundaryConditions,
                    bool noRigidMotion)
                : m_sim(inElems, inVertices, matField), m_matField(matField), m_boundaryConditions(boundaryConditions) {
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

        VField currentDisplacement() const {
            return m_sim.solve();
        }


        Real computeObjective(const VField &u, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing=1.0) {
            Real result = 0.0;

            if (weight_elasticity > 0.0) {
                result = weight_elasticity * computeElasticityObjective(u);
            }
            else {
                //std::cout << "objective: ignoring elasticity" << std::endl;
            }
            result += weight_bounds * computeBoundsObjective();
            result += weight_regularization * computeRegularizationObjective();

            if (weight_smoothing > 0.0) {
                //std::cout << "objective: working on smoothing" << std::endl;
                result += weight_smoothing * computeSmoothingObjective(u);
                //std::cout << "objective: working on smoothing" << std::endl;
            }
            else {
                //std::cout << "objective: ignoring smoothing" << std::endl;
            }

            return result;
        }

        std::vector<Real> computeObjectiveGradient(const VField &u, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing=1.0) {
            std::vector<Real> result(m_matField->numVars(), 0.0);

            // compute adjoint right hand side and solve adjoint
            std::vector<Real> result_elasticity;
            if (weight_elasticity > 0.0) {
                auto adjoint_rhs = m_sim.computeAdjointRhs(u);
                auto lambda = solveAdjoint(adjoint_rhs);
                result_elasticity = computeElasticityObjectiveGradient(u, lambda);
            }
            else {
                //std::cout << "gradient: ignoring elasticity" << std::endl;
                result_elasticity = std::vector<Real>(m_matField->numVars(), 0.0);
            }
            std::vector<Real> result_bounds = computeBoundsObjectiveGradient();
            std::vector<Real> result_regularization = computeRegularizationObjectiveGradient();

            std::vector<Real> result_smoothing;
            if (weight_smoothing > 0.0) {
                //std::cout << "gradient: working on smoothing" << std::endl;
                auto adjoint_rhs = computeSmoothingAdjointRhs(u);
                auto lambda = solveAdjoint(adjoint_rhs);
                result_smoothing = computeSmoothingObjectiveGradient(u, lambda);
                //std::cout << "gradient: working on smoothing" << std::endl;
            }
            else {
                //std::cout << "gradient: ignoring smoothing" << std::endl;
                result_smoothing = std::vector<Real>(m_matField->numVars(), 0.0);
            }

            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                result[var] = weight_elasticity * result_elasticity[var] + weight_bounds * result_bounds[var] +
                        weight_regularization * result_regularization[var] + weight_smoothing * result_smoothing[var];
            }

            return result;
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

        // From adjoint method:
        // dJ/dp = int_omega strain(u) : dE/dp : strain(lambda) dv
        std::vector <Real> computeElasticityObjectiveGradient(const VField &u, const VField &lambda) {

            std::vector <Real> g(m_matField->numVars(), 0);
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
                    g[var] += Quadrature<K, 6>::integrate(
                            [&](const EvalPt <K> &p) {
                                return dE.doubleContract(e_u(p))
                                        .doubleContract(e_lambda(p));
                            },
                            e->volume());
                }
            }

            return g;
        }

        /*std::vector <Real> approximateObjectiveGradient(const VField &u) {
            std::vector <Real> g(m_matField->numVars(), 0);
            double epsilon = 1e-5;

            double curValue = objective(u);

            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                size_t param;
                size_t matIdx;
                variableRole(var, matIdx, param);

                // Compute perturbed value
                m_matField->material(matIdx).vars[param] += epsilon;
                m_sim.materialFieldUpdated();
                auto perU = m_sim.solve(this->m_sim.neumannLoad());
                double perValue = objective(perU);

                // Compute negative perturbed value
                m_matField->material(matIdx).vars[param] -= 2*epsilon;
                m_sim.materialFieldUpdated();
                auto negPerU = m_sim.solve(this->m_sim.neumannLoad());
                double negPerValue = objective(negPerU);

                // centered finite difference
                g[var] = (perValue - negPerValue) / (2*epsilon);

                // go back
                m_matField->material(matIdx).vars[param] += epsilon;
            }

            return g;
        }*/

        std::vector <Real> approximateObjectiveGradient(const VField &u, Real weight_elasticity=1.0, Real weight_bounds=1.0, Real weight_regularization=1.0, Real weight_smoothing=1.0) {
            std::vector <Real> g(m_matField->numVars(), 0);
            /*double epsilon = 1e-6;

            //std::cout << "Computing Approximation for Gradient" << std::endl;

            double curValue = computeObjective(u, weight_elasticity, weight_bounds, weight_regularization, weight_smoothing);

            for (size_t var = 0; var < m_matField->numVars(); ++var) {
                size_t param;
                size_t matIdx;
                variableRole(var, matIdx, param);

                // Compute perturbed value
                m_matField->material(matIdx).vars[param] += epsilon;
                VField perU;
                if (weight_elasticity > 0.0 || weight_smoothing > 0.0) {
                    m_sim.materialFieldUpdated();
                    perU = m_sim.solve(this->m_sim.neumannLoad());
                }
                double perValue = computeObjective(perU, weight_elasticity, weight_bounds, weight_regularization, weight_smoothing);

                // centered finite difference
                g[var] = (perValue - curValue) / (epsilon);

                // go back
                m_matField->material(matIdx).vars[param] -= epsilon;
            }*/

            //std::cout << "Ending computation for gradient approximation" << std::endl;

            return g;
        }

        void run(MSHFieldWriter &writer, std::vector <Real> regularization_multipliers,
                 size_t iterations = 15,
                 size_t iterationsPerDirichletSolve = 1,
                 Real regularizationWeight = 0.0,
                 Real anisotropyPenaltyWeight = 0.0,
                 bool noRigidMotionDirichlet = false,
                 RegularizationMultMode = avg,
                 Real weight_elasticity = 1.0,
                 Real weight_bounds=1.0,
                 Real weight_smoothing=1.0);

        // Number of linear constraints for a given element
        size_t numConstraintsForElement(size_t elem) const {
            return m_constraints[elem].size();
        }

        Real computeBoundsObjective(Real epsilon = 1e-3) {
            double result = 0.0;

            for (size_t m = 0; m < m_matField->numMaterials(); ++m) {
                auto mat = m_matField->material(m);
                std::vector<size_t> elems;
                m_matField->getInfluenceRegion(m, elems);
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

        std::vector<double> computeBoundsObjectiveGradient(Real epsilon = 1e-3) {
            std::vector <Real> g(m_matField->numVars(), 0);
            size_t num = 0;

            for (size_t m = 0; m < m_matField->numMaterials(); ++m) {
                auto mat = m_matField->material(m);
                std::vector<size_t> elems;
                m_matField->getInfluenceRegion(m, elems);
                size_t e = elems[0]; // any element is ok, since they are suppose to have the same boundaries
                for (size_t lc = 0; lc < numConstraintsForElement(e); ++lc) {
                    const HyperPlane &plane = m_constraints[e][lc];
                    double plane_value = plane.back();
                    for (size_t k = 0; k < Material::numVars; ++k) {
                        plane_value += plane[k] * mat.vars[k];
                    }

                    if (plane_value + epsilon > 0.0) {
                        num++;
                        Real corrected_value = (plane_value+epsilon);
                        for (size_t k = 0; k < Material::numVars; ++k) {
                            g[m*Material::numVars + k] += 2 * corrected_value * plane[k];
                        }
                    }
                }
            }

            std::cout << "Elements inside regions of penalization: " << num << std::endl;

            return g;
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

        std::vector<Real> computeRegularizationObjectiveGradient() {
            std::vector<Real> g(m_matField->numVars(), 0);

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
                        g[mi*Material::numVars + v] += 2.0 * (mati.vars[v] - matj.vars[v]);
                        g[mj*Material::numVars + v] += -2.0 * (mati.vars[v] - matj.vars[v]);
                    }
                }
            }

            return g;
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
                p.row(v.index()) = v.node()->p + u(vNodeIndex);
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

        std::vector <Real> computeSmoothingObjectiveGradient(const VField &u, const VField &lambda) {
            std::vector <Real> g(m_matField->numVars(), 0);
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
                    g[var] += Quadrature<K, 6>::integrate(
                            [&](const EvalPt <K> &p) {
                                return dE.doubleContract(e_u(p))
                                        .doubleContract(e_lambda(p));
                            },
                            e->volume());
                }
            }

            return g;
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


        void readMaterialConstraints(const nlohmann::json &bounds);

        const typename _Simulator::Mesh &mesh() const { return m_sim.mesh(); }

        const _Simulator &simulator() const { return m_sim; }

        const MField &materialField() const { return *m_matField; }

        MField &materialField() { return *m_matField; }

        void variableRole(size_t var, size_t &matIdx, size_t &param) const {
            matIdx = var / Material::numVars;
            param  = var % Material::numVars;
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
    };


    template<class _Simulator>
    void GDOptimizer<_Simulator>::readMaterialConstraints(const nlohmann::json &bounds) {
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
                //mati.vars[0] = 0.005 + 0.1*(double)rand() / RAND_MAX;
                //mati.vars[1] = 0.0 + 0.1*(double)rand() / RAND_MAX;;
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

    template<class _Simulator>
void GDOptimizer<_Simulator>::run(
	MSHFieldWriter &writer, std::vector<double> regularization_multipliers, size_t iterations,
	size_t iterationsPerDirichletSolve, Real regularizationWeight,
	Real anisotropyPenaltyWeight, bool noRigidMotionDirichlet,
	RegularizationMultMode multMode, Real weight_elasticity, Real weight_bounds, Real weight_smoothing)
{
	auto neumannLoad = this->m_sim.neumannLoad();
	m_sim.projectOutRigidComponent(neumannLoad);
	writer.addField("Neumann load", m_sim.dofToNodeField(neumannLoad), DomainType::PER_NODE);

	SMField e_dirichletTargets_avg;

	// Write initial material variable fields
	m_matField->writeVariableFields(writer, "0 ");

	double stepSize = 1.0;
	constexpr size_t _NVar = Material::numVars;
	Real current_objective;
	Real best_objective_value;
	std::vector<Real> bestg;
	int iteration_without_improvement = 0;
	MField best_material_field = *m_matField;

	auto u = m_sim.solve(neumannLoad);
    auto s_neumann_avg = m_sim.averageStressField(u);
	current_objective = computeObjective(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
	std::vector<Real> g = computeObjectiveGradient(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
	//auto lambda = m_sim.solveAdjoint(u);
	//std::vector<Real> ag = approximateObjectiveGradient(u);
	std::vector<Real> ag = approximateObjectiveGradient(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
    best_objective_value = current_objective;
	best_material_field = *m_matField;

	 // Write initial ("iteration 1") objective and gradient norm.
    Real gradNormSq = 0;
    Real approxGradNormSq = 0;
    //Real approxGradNormSqSimpler = 0;
    for (size_t c = 0; c < g.size(); ++c) {
        gradNormSq += g[c] * g[c];
        approxGradNormSq += ag[c] * ag[c];
        //approxGradNormSqSimpler += ags[c] * ags[c];
    }
    //logger().debug("0 objective, gradient norm:\t{}\t{}", current_objective, sqrt(gradNormSq));
    std::cout << "weight: " <<  weight_smoothing << std::endl;
    logger().debug("0 objective, gradient norm, approx grad norm:\t{}\t{}\t{}", current_objective, sqrt(gradNormSq), sqrt(approxGradNormSq));
    //logger().debug("0 objective, gradient norm, approx grad norm:\t{}\t{}\t{}\t{}", current_objective, sqrt(gradNormSq), sqrt(approxGradNormSq), sqrt(approxGradNormSqSimpler));

    m_matField->writeVariableFields(writer, "0 ");
    writer.addField("0 u", u, DomainType::PER_NODE);
    //writer.addField("0 lambda", lambda, DomainType::PER_NODE);
	m_matField->writeVariableFields(writer, "0 grad_", g);
	m_matField->writeVariableFields(writer, "0 approx_grad_", ag);
	//m_matField->writeVariableFields(writer, "0 approx_grad_simpler", ags);

	for (size_t iter = 1; iter <= iterations; ++iter) {
	    //std::cout << "[MATOPT GD] iter " << iter << std::endl;

		// Find gradient
		//std::cout << "Computing gradient" << std::endl;
        g = computeObjectiveGradient(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
        std::vector<Real> newVars(g.size());
        //m_matField->writeVariableFields(writer, std::to_string(iter) + " m");
        //m_matField->writeVariableFields(writer, std::to_string(iter) + " igrad_", g);

		// Modify material according to gradient
		//std::cout << "Updating variables according to gradient" << std::endl;
		m_matField->getVars(newVars);
		Real gradNormSq = 0;
		Real correctedGradNormSq = 0;
		std::cout << "Step size: " << stepSize << std::endl;
        for (size_t var = 0; var < m_matField->numVars(); ++var) {
            size_t param;
            size_t matIdx;
            variableRole(var, matIdx, param);

            double newValue = newVars[var] - stepSize * g[var];
            double lb = m_matField->material(matIdx).lower_bounds[param];
            double ub = m_matField->material(matIdx).upper_bounds[param];

            double correctedG = g[var];
            if (newValue <= lb) {
                newValue = lb;
            }
            if (newValue >= ub) {
                newValue = ub;
            }

            if (newVars[var] <= lb || newVars[var] >= ub) {
                correctedG = 0;
            }

            //std::cout << "Old Value, new Value: " << newVars[var] << ", " << newValue << std::endl;
            //double correctedG = (newVars[var] - newValue) / stepSize;
            correctedGradNormSq += correctedG*correctedG;
            gradNormSq += g[var] * g[var];
            newVars[var] = newValue;
        }
        std::cout << "Norm, corrected: " << sqrt(gradNormSq) << ", " << sqrt(correctedGradNormSq) << std::endl;


        //std::cout << "Setting variables" << std::endl;
        m_matField->setVars(newVars);

		// Write current material variable fields
		//std::cout << "Writting variable fields" << std::endl;
		m_matField->writeVariableFields(writer, std::to_string(iter) + " ");

		// Write the post-iteration solution and print statistics
		bool ran_ok = true;
		try {
            m_sim.materialFieldUpdated();
            u = m_sim.solve(neumannLoad);
            current_objective = computeObjective(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
            //lambda = m_sim.solveAdjoint(u);
            g = computeObjectiveGradient(u, weight_elasticity, weight_bounds, regularizationWeight, weight_smoothing);
            std::vector <Real> ag = approximateObjectiveGradient(u, weight_elasticity, weight_bounds,
                                                                 regularizationWeight, weight_smoothing);
            writer.addField(std::to_string(iter) + " u", u, DomainType::PER_NODE);
            //writer.addField(std::to_string(iter) + " lambda", lambda, DomainType::PER_NODE);
            // Write gradient component fields
            m_matField->writeVariableFields(writer, std::to_string(iter) + " grad_", g);
            m_matField->writeVariableFields(writer, std::to_string(iter) + " approx_grad_", ag);

            gradNormSq = 0;
            approxGradNormSq = 0;
            //approxGradNormSqSimpler = 0;
            for (size_t c = 0; c < g.size(); ++c) {
                gradNormSq += g[c] * g[c];
                approxGradNormSq += ag[c] * ag[c];
                //approxGradNormSqSimpler += ags[c] * ags[c];
            }
            //logger().debug("0 objective, gradient norm, approx grad norm:\t{}\t{}\t{}\t{}", current_objective, sqrt(gradNormSq), sqrt(approxGradNormSq), sqrt(approxGradNormSqSimpler));
            logger().debug("{} objective, gradient norm:\t{}\t{}\t{}", iter, current_objective, sqrt(gradNormSq), sqrt(approxGradNormSq));
            //logger().debug("{} objective, gradient norm:\t{}\t{}", iter, current_objective, sqrt(gradNormSq));
            }
		catch(...) {
            ran_ok = false;
		}


        if (current_objective < (best_objective_value)) {
            std::cout << "improved solution" << std::endl;
            best_objective_value = current_objective;
            best_material_field = *m_matField;
            //bestg = ag;
            //bestg = g;
            iteration_without_improvement = 0;
            stepSize *= 2.0;
        } else {
            std::cout << "worse solution" << std::endl;
            iteration_without_improvement++;
            stepSize /= 10.0;
            *m_matField = best_material_field;
            //ag = bestg;
            //g = bestg;
            //m_sim.materialFieldUpdated();
        }

        if (iteration_without_improvement >= 10) {
            std::cout << "Leaving optimization due to " << iteration_without_improvement
                      << " iterations without improvement!" << std::endl;
            std::cout << "Best: " << best_objective_value << std::endl;
            std::cout << "Current: " << current_objective << std::endl;
            std::cout << "Step size: " << stepSize << std::endl;
            break;
        }

	}

	*m_matField = best_material_field;
	logger().debug("Best objective:\t{}", best_objective_value);
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
            Real elasticityWeight = 1.0,
            Real boundsWeight = 1.0,
            Real smoothingWeight = 1.0) {
        typedef MaterialOptimization::Mesh<_N, _FEMDegree, _Material> Mesh;
        typedef MaterialOptimization::Simulator<Mesh> Simulator;
        typedef MaterialOptimization::GDOptimizer<Simulator> Opt;

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
                                                 BBox < VectorND < _N >> (inVertices), noRigidMotion);

        Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion);
        matOpt.readMaterialConstraints(materialBounds);

        std::cout << "Opening: " << intermediateResults << std::endl;
        MSHFieldWriter writer(intermediateResults, matOpt.mesh());

        logger().debug("Attempting optimization");
        matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
                   regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet, multMode, elasticityWeight,
                   boundsWeight, smoothingWeight);

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
            size_t shear_degree = 0,
	        std::vector<double> shear_coeffs = std::vector<double>(),
	        Real angle = 0.0,
            MaterialOptimization::RegularizationMultMode multMode = MaterialOptimization::avg,
            Real elasticityWeight = 1.0,
            Real boundsWeight = 1.0,
            Real smoothingWeight = 1.0) {
        typedef MaterialOptimization::Mesh<_N, _FEMDegree, Materials::ConstrainedCubic> Mesh;
	typedef MaterialOptimization::Simulator<Mesh>           Simulator;
        typedef MaterialOptimization::GDOptimizer<Simulator> Opt;

        typedef typename Opt::MField MField;

        // Material bounds are global variables
	    // Materials::ConstrainedCubic<_N>::setBoundsFromJson(materialBounds);
	    Materials::ConstrainedCubic<_N>::setShearParameters(shear_degree, shear_coeffs);

	    const Real PI = 3.14159265;
        const Real theta = angle*PI/180;
        Materials::ConstrainedCubic<_N>::setOrthotropicAngle(theta);

        // Material bounds are global variables
        // Materials::ConstrainedCubic><_N>::setBoundsFromJson(materialBounds);

        // If available, use element->material associations.
        // Otherwise, we use one material per element.
        auto matField = std::make_shared<MField>(inElements.size(), matIdxForElement);

        // Read boundary conditions
        bool noRigidMotion;
        std::string bcs(boundaryConditions.dump());
        std::istringstream iss(bcs);
        auto bconds = readBoundaryConditions<_N>(iss,
                                                 BBox < VectorND < _N >> (inVertices), noRigidMotion);

        Opt matOpt(inElements, inVertices, matField, bconds, noRigidMotion);
        matOpt.readMaterialConstraints(materialBounds);

        std::cout << "Opening: " << intermediateResults << std::endl;
        MSHFieldWriter writer(intermediateResults, matOpt.mesh());

        logger().debug("Attempting optimization");
        matOpt.run(writer, regularization_multipliers, iterations, iterationsPerDirichlet,
                   regularizationWeight, anisotropyPenaltyWeight, noRigidMotionDirichlet, multMode, elasticityWeight,
                   boundsWeight, smoothingWeight);

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

}
////////////////////////////////////////////////////////////////////////////////

void material_optimization_gd(
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
    double smoothingWeight)
{
    std::cout << "Material type: " << materialType << std::endl;
    std::cout << "weight: " << smoothingWeight << std::endl;

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
             elasticityWeight,
             boundsWeight,
             smoothingWeight);
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
             elasticityWeight,
             boundsWeight,
             smoothingWeight);
    }
}