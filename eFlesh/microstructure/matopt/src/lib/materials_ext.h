#ifndef MATOPTIMIZATION_MATERIALS_EXT_H
#define MATOPTIMIZATION_MATERIALS_EXT_H

#include <MeshFEM/Materials.hh>

namespace Materials {

    // Axis-aligned cubic material.
    // 3 variables
    // Vars    0: Young's moduli,
    // Var     1: Poisson ratio
    // Var     2: Shear modulus
    template<size_t _N>
    struct Cubic : public Materials::VariableMaterial<_N, Cubic, 3> {
        static constexpr size_t N = _N;
        typedef ElasticityTensor <Real, _N> ETensor;
        typedef Eigen::Matrix<Real, flatLen(_N), 1> FlattenedSymmetricMatrix;

        typedef Materials::VariableMaterial<N, Cubic, 3> Base;
        using Base::vars;

        // WARNING: bounds are shared by all cubic materials! (static)
        struct CubicBounds : Materials::Bounds {
            CubicBounds() {
                // Default Bounds
                // Upper: Upper bounds should be based on base material's moduli.
                //        Poisson ratios can't be greater than 0.5
                //        (at 0.5, 3D isotropic lambda becomes Inf, so we avoid it
                //        here too)
                // Lower: Young's and shear moduli must be positive and are hard to make
                //        small--this minimum should be set based on homogenization results
                //        Poisson ratios can't be less than -1, and for robustness we
                //        limit them to -0.75
                Base::setUpperBounds({Bound(0, 300), Bound(1, 0.45), Bound(2, 0.45)});
                Base::setLowerBounds({Bound(0, 0.01), Bound(1, -0.75), Bound(2, 0.01)});
            }
        };

        Cubic() {
            // Default Parameters
            vars[0] = 1.0;
            vars[1] = 0.3;
            vars[2] = 1.0 / (2.0 * (1 + 0.3));
        }

        static const std::string &variableName(size_t i) {

            static const std::vector <std::string> namesXD = {
                    "E",
                    "nu",
                    "mu"};
            return namesXD.at(i);
        }

        // Used for adjoint method gradient-based optimization
        void getETensorDerivative(size_t p, ETensor &d) const;

        void getTensor(ETensor &tensor) const {
            if (_N == 3) {
                tensor.setOrthotropic3D(vars[0], vars[0], vars[0],
                                        vars[1], vars[1], vars[1],
                                        vars[2], vars[2], vars[2]);
            } else {
                tensor.setOrthotropic2D(vars[0], vars[0], vars[1], vars[2]);
            }
        }

        // Ceres-compatible cost function to fit orthotropic material parameters to
        // best achieve:
        //      e ~= E^(-1)(Y_x, Y_y, ...) : s
        template<class SMatrix>
        struct StressStrainFitCostFunction {
            StressStrainFitCostFunction(const SMatrix &e, const SMatrix &s, Real vol)
                    : strain(e), stress(s) {
                if (vol <= 0) throw std::runtime_error("Volume must be positive");
                volSqrt = sqrt(vol);
            }

            template<typename T>
            bool operator()(const T *x, T *e) const {
                if (_N == 3) {
                    T D01 = -x[1] / x[0], // -nu_yx / E_y
                            D02 = -x[1] / x[0], // -nu_zx / E_z
                            D12 = -x[1] / x[0]; // -nu_zy / E_z
                    e[0] = T(stress[0]) / x[0] + T(stress[1]) * D01 + T(stress[2]) * D02;
                    e[1] = T(stress[0]) * D01 + T(stress[1]) / x[0] + T(stress[2]) * D12;
                    e[2] = T(stress[0]) * D02 + T(stress[1]) * D12 + T(stress[2]) / x[0];
                    e[3] = T(0.5 * stress[3]) / x[2];
                    e[4] = T(0.5 * stress[4]) / x[2];
                    e[5] = T(0.5 * stress[5]) / x[2];
                } else {
                    T D01 = -x[1] / x[0]; // -nu_yx / E_y
                    e[0] = T(stress[0]) / x[0] + T(stress[1]) * D01;
                    e[1] = T(stress[0]) * D01 + T(stress[1]) / x[0];
                    e[2] = T(0.5 * stress[2]) / x[2];
                }

                for (size_t i = 0; i < flatLen(_N); ++i) {
                    e[i] -= T(strain[i]);
                    if (i >= _N) e[i] *= T(sqrt(2.0));
                    e[i] *= T(volSqrt);
                }

                return true;
            }

            Real volSqrt;
            SMatrix strain, stress;
        };

    private:
        friend Base;
        static CubicBounds g_bounds;
    };

    // Static variable needs to be explicitly defined...
    template<size_t _N>
    typename Cubic<_N>::CubicBounds Cubic<_N>::g_bounds;

    // Axis-aligned cubic material with constrained shear modulus.
    // 3 variables
    // Vars    0: Young's moduli,
    // Var     1: Poisson ratio
    template<size_t _N>
    struct ConstrainedCubic : public Materials::VariableMaterial<_N, ConstrainedCubic, 2> {
        static constexpr size_t N = _N;
        typedef ElasticityTensor <Real, _N> ETensor;
        typedef Eigen::Matrix<Real, flatLen(_N), 1> FlattenedSymmetricMatrix;

        typedef Materials::VariableMaterial<N, ConstrainedCubic, 2> Base;
        using Base::vars;

        // WARNING: bounds are shared by all cubic materials! (static)
        struct ConstrainedCubicBounds : Materials::Bounds {
            ConstrainedCubicBounds() {
                // Default Bounds
                // Upper: Upper bounds should be based on base material's moduli.
                //        Poisson ratios can't be greater than 0.5
                //        (at 0.5, 3D isotropic lambda becomes Inf, so we avoid it
                //        here too)
                // Lower: Young's and shear moduli must be positive and are hard to make
                //        small--this minimum should be set based on homogenization results
                //        Poisson ratios can't be less than -1, and for robustness we
                //        limit them to -0.75
                Base::setUpperBounds({Bound(0, 300), Bound(1, 0.45)});
                Base::setLowerBounds({Bound(0, 0.01), Bound(1, -0.75)});
            }
        };

        ConstrainedCubic() {
            // Default Parameters
            vars[0] = 1.0;
            vars[1] = 0.3;
        }

        static void setShearParameters(size_t shear_degree, std::vector<Real> shear_coeffs) {
            degree = shear_degree;
            for (auto c : shear_coeffs) {
                param.push_back(c);
            }
        }

        static void setOrthotropicAngle(Real a) {
            angle = a;
        }

        template<class T>
        static T shearFormula(T E, T nu) {
            T result;
            if (degree) {
                result = T(0.0);
                //std::cout << "partial: " << result << std::endl;
                size_t coeff_idx = 0;

                std::vector<T> nu_powers(1, T(1.0));
                std::vector<T> E_powers(1, T(1.0));
                for (int i=1; i<=degree; i++) {
                    T new_nu_power = nu_powers[i-1] * nu;
                    T new_E_power = E_powers[i-1] * E;

                    nu_powers.push_back(new_nu_power);
                    E_powers.push_back(new_E_power);
                }

                for (int i=0; i<=degree; i++) {
                    for (int j=0; j<=degree; j++) {
                        if ((i + j) > degree)
                            continue;

                        result += param[coeff_idx] * nu_powers[i] * E_powers[j];

                        coeff_idx += 1;
                    }
                }
            }
            else {
                result = E / (2.0 * (1.0 + nu));
            }

            //std::cout << "result: " << result << std::endl;

            return result;
        }

        template<class T>
        static T shearFormulaDerivative(size_t p, T E, T nu) {
            T result;
            if (degree) {
                result = T(0.0);
                size_t coeff_idx = 0;

                std::vector<T> nu_powers(1, T(1.0));
                std::vector<T> E_powers(1, T(1.0));
                for (int i=1; i<=degree; i++) {
                    T new_nu_power = nu_powers[i-1] * nu;
                    T new_E_power = E_powers[i-1] * E;

                    nu_powers.push_back(new_nu_power);
                    E_powers.push_back(new_E_power);
                }

                for (int i=0; i<=degree; i++) {
                    for (int j=0; j<=degree; j++) {
                        if ((i + j) > degree)
                            continue;

                        if (p == 0) {
                            if (j == 0) {
                                coeff_idx += 1;
                                continue;
                            }

                            double multiplier = (double) j;
                            result += multiplier * param[coeff_idx] * nu_powers[i] * E_powers[j-1];
                        }
                        else {
                            if (i == 0) {
                                coeff_idx += 1;
                                continue;
                            }

                            double multiplier = (double) i;
                            result += multiplier * param[coeff_idx] * nu_powers[i-1] * E_powers[j];
                        }

                        coeff_idx += 1;
                    }
                }
            }
            else {
                result = (p == 0) ? 1 / (2 * (1 + nu)) : -E / (2 * (1 + nu) * (1 + nu));
            }

            return result;
        }

        static const std::string &variableName(size_t i) {
            static const std::vector <std::string> namesXD = {
                    "E",
                    "nu"};
            return namesXD.at(i);
        }

        // Used for adjoint method gradient-based optimization
        void getETensorDerivative(size_t p, ETensor &d) const;

        void getTensor(ETensor &tensor) const {
            auto shear = shearFormula(vars[0], vars[1]);

            if (_N == 3) {
                tensor.setOrthotropic3D(vars[0], vars[0], vars[0],
                                        vars[1], vars[1], vars[1],
                                        shear, shear, shear);
            } else {
                //std::cout << "setting isotropic" << std::endl;
                tensor.setOrthotropic2D(vars[0], vars[0], vars[1], shear);
                //tensor.setIsotropic(vars[0], vars[1]);
            }

            Eigen::Matrix<Real, _N, _N> rot;
            rot << cos(angle), -sin(angle),
                   sin(angle),  cos(angle);
            //std::cout << "tensor before: " << tensor << std::endl;
            tensor = tensor.transform(rot);
            //std::cout << "tensor after: " << tensor << std::endl;
        }

        // Ceres-compatible cost function to fit orthotropic material parameters to
        // best achieve:
        //      e ~= E^(-1)(Y_x, Y_y, ...) : s
        template<class SMatrix>
        struct StressStrainFitCostFunction {
            StressStrainFitCostFunction(const SMatrix &e, const SMatrix &s, Real vol)
                    : strain(e), stress(s) {
                if (vol <= 0) throw std::runtime_error("Volume must be positive");
                volSqrt = sqrt(vol);
            }

            template<typename T>
            bool operator()(const T *x, T *e) const {
                auto shear = shearFormula(x[0], x[1]);

                /*if (_N == 3) {
                    T D01 = -x[1] / x[0], // -nu_yx / E_y
                            D02 = -x[1] / x[0], // -nu_zx / E_z
                            D12 = -x[1] / x[0]; // -nu_zy / E_z
                    e[0] = T(stress[0]) / x[0] + T(stress[1]) * D01 + T(stress[2]) * D02;
                    e[1] = T(stress[0]) * D01 + T(stress[1]) / x[0] + T(stress[2]) * D12;
                    e[2] = T(stress[0]) * D02 + T(stress[1]) * D12 + T(stress[2]) / x[0];
                    e[3] = T(0.5 * stress[3]) / shear;
                    e[4] = T(0.5 * stress[4]) / shear;
                    e[5] = T(0.5 * stress[5]) / shear;
                } else {
                    T D01 = -x[1] / x[0]; // -nu_yx / E_y
                    e[0] = T(stress[0]) / x[0] + T(stress[1]) * D01;
                    e[1] = T(stress[0]) * D01 + T(stress[1]) / x[0];
                    e[2] = T(0.5 * stress[2]) / shear;
                }

                for (size_t i = 0; i < flatLen(_N); ++i) {
                    e[i] -= T(strain[i]);
                    if (i >= _N) e[i] *= T(sqrt(2.0));
                    e[i] *= T(volSqrt);
                }

                std::cout << "e[0]" << e[0] << std::endl;
                std::cout << "e[1]" << e[1] << std::endl;
                std::cout << "e[2]" << e[2] << std::endl;*/

                ElasticityTensor<T, _N> complianceTensor;
                if (_N == 3) {
                    //complianceTensor.setOrthotropicCompliance3D(x[0], x[0], x[0],
                    //                        x[1], x[1], x[1],
                    //                        shear, shear, shear);
                } else {
                    //std::cout << "x[0]: " << x[0] << std::endl;
                    //std::cout << "x[1]: " << x[1] << std::endl;
                    //std::cout << "shear: " << shear << std::endl;
                    complianceTensor.setOrthotropicCompliance2D(x[0], x[0], x[1], shear);

                    /*T D00 = 1.0 / x[0];
                    T D11 = 1.0 / x[0];
                    T D01 = -1.0 * x[1] / D11;
                    T D22 = 1.0 * shear;
                    Eigen::Matrix<T, 3, 3> test;
                    test << D00, D01, 0.0,
                            D01, D11, 0.0,
                            0.0, 0.0, D22;
                    std::cout << "test: " << test << std::endl;
                    test(0,0) = D00;
                    test(0,1) = D01;
                    test(1,0) = D01;
                    test(1,1) = D11;
                    test(2,2) = D22;
                    std::cout << "test: " << test << std::endl;*/
                }

                //std::cout << "Tensor: " << complianceTensor << std::endl;

                Eigen::Matrix<T, _N, _N> rot;
                rot << T(cos(angle)), T(-sin(angle)),
                       T(sin(angle)),  T(cos(angle));
                complianceTensor = complianceTensor.transform(rot);

                //auto complianceTensor = tensor.inverse();

                SymmetricMatrixValue<T, N> stressT;
                for (size_t i = 0; i < flatLen(_N); ++i) {
                    stressT[i] = T(stress[i]);
                }

                auto fakeStrain = complianceTensor.doubleContract(stressT);
                //std::cout << "fakeStrain[0]: " << fakeStrain[0] << std::endl;
                //std::cout << "fakeStrain[1]: " << fakeStrain[1] << std::endl;
                //std::cout << "fakeStrain[2]: " << fakeStrain[2] << std::endl;
                for (size_t i = 0; i < flatLen(_N); ++i) {
                    e[i] = fakeStrain[i];
                    e[i] -= T(strain[i]);
                    if (i >= _N) e[i] *= T(sqrt(2.0));
                    e[i] *= T(volSqrt);
                }

                //std::cout << "ne[0]" << e[0] << std::endl;
                //std::cout << "ne[1]" << e[1] << std::endl;
                //std::cout << "ne[2]" << e[2] << std::endl;

                return true;
            }

            Real volSqrt;
            SMatrix strain, stress;
        };

        static size_t degree;
        static std::vector<Real> param;
        static Real angle;

    private:
        friend Base;
        static ConstrainedCubicBounds g_bounds;
    };

    // Static variable needs to be explicitly defined...
    template<size_t _N>
    typename ConstrainedCubic<_N>::ConstrainedCubicBounds ConstrainedCubic<_N>::g_bounds;

    template<size_t _N>
    std::vector<Real> ConstrainedCubic<_N>::param = std::vector<Real>();

    template<size_t _N>
    size_t ConstrainedCubic<_N>::degree = 0;

    template<size_t _N>
    Real ConstrainedCubic<_N>::angle = 0.0;

}
#endif //MATOPTIMIZATION_MATERIALS_EXT_H
