#pragma once

////////////////////////////////////////////////////////////////////////////////
//#include <quadfoam/common.h>
#include <Eigen/Dense>
////////////////////////////////////////////////////////////////////////////////

///
/// Intersects negative half-planes and compute the convex polygon formed by the intersection. The
/// algorithm works as follows: 1) compute each pairwise intersection between half-planes, and
/// compute their bounding box; 2) iteratively clip the bounding box by each half-plane. This will
/// not work if the intersected region is unbounded, but at least the resulting polygon is inside
/// the valid region.
///
/// @param[in]  P     #P x 3 matrix of half-plane equations.
/// @param[out] V     #V x 2 matrix of output vertices positions.
///
void intersect_half_planes(const Eigen::MatrixXd &P, Eigen::MatrixXd &V);

///
/// Compute the total area of a given polygon.
///
/// @param[in]  V     #V x (2|3) matrix of vertices along the polygon.
///
/// @return     Total area of the polygon.
///
double polygon_area(const Eigen::MatrixXd &V);

