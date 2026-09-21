//
// Created by Davi Colli Tozoni on 7/29/22.
//

#include "materials_ext.h"

namespace Materials {

// Derivatives of the elasticity tensor with respect to the cubic material properties.
// Modified from
// Vars    0: Young's moduli,
// Var     2: Poisson ratio
// Var     3: Shear modulus
    template<size_t _N>
    void Cubic<_N>::getETensorDerivative(size_t p, Cubic<_N>::ETensor &d) const {
        d.clear();
        if (_N == 3) {
            Real Ex = vars[0], Ey = vars[0], Ez = vars[0];
            Real vyx = vars[1], vzx = vars[1], vzy = vars[1];
            if (p == 0) {
                d.D(0, 0) = pow(Ey, 2) * pow(Ez - Ey * pow(vzy, 2), 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 1) = -((Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) * (-Ez + Ey * pow(vzy, 2)) *
                              pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                  pow(Ey, 2) * pow(vzy, 2), -2));
                d.D(0, 2) = -(Ez * (vzx + vyx * vzy) * pow(Ey, 2) * (-Ez + Ey * pow(vzy, 2)) *
                              pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                  pow(Ey, 2) * pow(vzy, 2), -2));
                d.D(1, 1) = pow(Ey, 2) * pow(Ez * vyx + Ey * vzx * vzy, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 2) = Ez * (vzx + vyx * vzy) * (Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(2, 2) = pow(Ey, 2) * pow(Ez, 2) * pow(vzx + vyx * vzy, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
            };
            if (p == 1) {
                d.D(0, 0) = -2 * Ey * (Ez * vyx + Ey * vzx * vzy) * pow(Ex, 2) * (-Ez + Ey * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 1) = Ex * Ey *
                            (Ey * Ez * (Ez - Ex * vzx * (vzx - 2 * vyx * vzy)) + Ex * pow(Ez, 2) * pow(vyx, 2) -
                             pow(Ey, 2) * (Ez - 2 * Ex * pow(vzx, 2)) * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 2) = Ex * Ey * Ez * (Ex * (Ez * vyx * (2 * vzx + vyx * vzy) + Ey * vzy * pow(vzx, 2)) +
                                            Ey * vzy * (Ez - Ey * pow(vzy, 2))) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 1) = 2 * Ex * (Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) * (Ez - Ex * pow(vzx, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 2) = Ex * Ey * Ez *
                            (Ex * Ez * vzx * pow(vyx, 2) + Ey * (Ez * (vzx + 2 * vyx * vzy) - Ex * pow(vzx, 3)) +
                             vzx * pow(Ey, 2) * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(2, 2) = 2 * Ex * Ey * (Ex * vyx * vzx + Ey * vzy) * (vzx + vyx * vzy) * pow(Ez, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
            };
            if (p == 2) {
                d.D(3, 3) = 1;
                d.D(4, 4) = 1;
                d.D(5, 5) = 1;
            };
        } else if (_N == 2) {
            Real Ex = vars[0], Ey = vars[0];
            Real vyx = vars[1];
            if (p == 0) {
                d.D(0, 0) = pow(Ey, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(0, 1) = vyx * pow(Ey, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(1, 1) = pow(Ey, 2) * pow(vyx, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
            };
            if (p == 1) {
                d.D(0, 0) = 2 * Ey * vyx * pow(Ex, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(0, 1) = Ex * Ey * (Ey + Ex * pow(vyx, 2)) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(1, 1) = 2 * Ex * vyx * pow(Ey, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
            };
            if (p == 2) { d.D(2, 2) = 1; };
        }
    }

    // Derivatives of the elasticity tensor with respect to the constrained cubic material properties.
    // Modified from
    // Vars    0: Young's moduli,
    // Var     1: Poisson ratio
    template<size_t _N>
    void ConstrainedCubic<_N>::getETensorDerivative(size_t p, ConstrainedCubic<_N>::ETensor &d) const {
        d.clear();
        if (_N == 3) {
            Real Ex = vars[0], Ey = vars[0], Ez = vars[0];
            Real vyx = vars[1], vzx = vars[1], vzy = vars[1];
            if (p == 0) {
                d.D(0, 0) = pow(Ey, 2) * pow(Ez - Ey * pow(vzy, 2), 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 1) = -((Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) * (-Ez + Ey * pow(vzy, 2)) *
                              pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                  pow(Ey, 2) * pow(vzy, 2), -2));
                d.D(0, 2) = -(Ez * (vzx + vyx * vzy) * pow(Ey, 2) * (-Ez + Ey * pow(vzy, 2)) *
                              pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                  pow(Ey, 2) * pow(vzy, 2), -2));
                d.D(1, 1) = pow(Ey, 2) * pow(Ez * vyx + Ey * vzx * vzy, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 2) = Ez * (vzx + vyx * vzy) * (Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(2, 2) = pow(Ey, 2) * pow(Ez, 2) * pow(vzx + vyx * vzy, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
            };
            if (p == 1) {
                d.D(0, 0) = -2 * Ey * (Ez * vyx + Ey * vzx * vzy) * pow(Ex, 2) * (-Ez + Ey * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 1) = Ex * Ey *
                            (Ey * Ez * (Ez - Ex * vzx * (vzx - 2 * vyx * vzy)) + Ex * pow(Ez, 2) * pow(vyx, 2) -
                             pow(Ey, 2) * (Ez - 2 * Ex * pow(vzx, 2)) * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(0, 2) = Ex * Ey * Ez * (Ex * (Ez * vyx * (2 * vzx + vyx * vzy) + Ey * vzy * pow(vzx, 2)) +
                                            Ey * vzy * (Ez - Ey * pow(vzy, 2))) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 1) = 2 * Ex * (Ez * vyx + Ey * vzx * vzy) * pow(Ey, 2) * (Ez - Ex * pow(vzx, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(1, 2) = Ex * Ey * Ez *
                            (Ex * Ez * vzx * pow(vyx, 2) + Ey * (Ez * (vzx + 2 * vyx * vzy) - Ex * pow(vzx, 3)) +
                             vzx * pow(Ey, 2) * pow(vzy, 2)) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
                d.D(2, 2) = 2 * Ex * Ey * (Ex * vyx * vzx + Ey * vzy) * (vzx + vyx * vzy) * pow(Ez, 2) *
                            pow(Ey * (-Ez + Ex * vzx * (vzx + 2 * vyx * vzy)) + Ex * Ez * pow(vyx, 2) +
                                pow(Ey, 2) * pow(vzy, 2), -2);
            };
        } else if (_N == 2) {
            Real E = vars[0];
            Real nu = vars[1];

            // 2D Lambda = (nu * E) / (1.0 - nu * nu);
            //    mu = E / (2.0 + 2.0 * nu);
            auto dL = (p == 0) ? nu / (1 - nu * nu)
                          : E * (1 + nu * nu) / ((1 - nu * nu) * (1 - nu * nu));

            auto dmu_iso = (p == 0) ? 1 / (2 * (1 + nu))
                       : -E / (2 * (1 + nu) * (1 + nu));

            d.D(0, 0) = dL + 2*dmu_iso;
            d.D(0, 1) = dL;
            d.D(1, 1) = dL + 2*dmu_iso;

            /*if (p == 0) {
                d.D(0, 0) = pow(Ey, 2) * pow(0, -2);
                d.D(0, 1) = vyx * pow(Ey, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(1, 1) = pow(Ey, 2) * pow(vyx, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
            };
            if (p == 1) {
                d.D(0, 0) = 2 * Ey * vyx * pow(Ex, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(0, 1) = Ex * Ey * (Ey + Ex * pow(vyx, 2)) * pow(Ey - Ex * pow(vyx, 2), -2);
                d.D(1, 1) = 2 * Ex * vyx * pow(Ey, 2) * pow(Ey - Ex * pow(vyx, 2), -2);
            };*/
        }

        // 2D and 3D mu: E / (2 (1 + nu))
        Real E = vars[0];
        Real nu = vars[1];
        Real dmu = shearFormulaDerivative(p, E, nu);
        for (size_t i = _N; i < flatLen(_N); ++i) {
            d.D(i, i) = dmu;
        }

        Eigen::Matrix<Real, _N, _N> rot;
        rot << cos(angle), -sin(angle),
               sin(angle),  cos(angle);

        //std::cout << "before: " << d;
        d = d.transform(rot);
        //std::cout << "after: " << d;

        /*assert(p == 0 || p == 1);
        d.clear();
        Real E = vars[0], nu = vars[1];
        Real dL, dmu;
        if (_N == 2) {
            // 2D Lambda = (nu * E) / (1.0 - nu * nu);
            //    mu = E / (2.0 + 2.0 * nu);
            dL = (p == 0) ? nu / (1 - nu * nu)
                          : E * (1 + nu * nu) / ((1 - nu * nu) * (1 - nu * nu));
        }
        if (_N == 3) {
            // 3D Lambda = (nu * E) / ((1.0 + nu) * (1.0 - 2.0 * nu));
            Real denSqrt = 1 - nu - 2 * nu * nu;
            dL = (p == 0) ? nu / ((1.0 + nu) * (1.0 - 2 * nu)) : E * (1 + 2 * nu * nu) / (denSqrt * denSqrt);
        }

        // 2D and 3D mu: E / (2 (1 + nu))
        dmu = (p == 0) ? 1 / (2 * (1 + nu))
                       : -E / (2 * (1 + nu) * (1 + nu));
        for (size_t i = 0; i < flatLen(_N); ++i) {
            for (size_t j = i; j < _N; ++j)
                d.D(i, j) = dL;
            d.D(i, i) += (i < _N) ? 2 * dmu : dmu;
        }*/
    }

template struct Cubic<2>;
template struct Cubic<3>;
template struct ConstrainedCubic<2>;
template struct ConstrainedCubic<3>;

}