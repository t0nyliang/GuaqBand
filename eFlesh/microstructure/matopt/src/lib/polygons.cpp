////////////////////////////////////////////////////////////////////////////////
#include "polygons.h"
#include <predicates_psm.h>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

inline GEO::Sign point_is_in_half_plane(
	const Eigen::RowVector2d &p, const Eigen::RowVector2d &q1, const Eigen::RowVector2d &q2)
{
	return GEO::PCK::orient_2d(q1.data(), q2.data(), p.data());
}

inline bool intersect_segments(
	const Eigen::RowVector2d & p1, const Eigen::RowVector2d & p2,
	const Eigen::RowVector2d & q1, const Eigen::RowVector2d & q2,
	Eigen::RowVector2d & result)
{

	Eigen::RowVector2d Vp = p2 - p1;
	Eigen::RowVector2d Vq = q2 - q1;
	Eigen::RowVector2d pq = q1 - p1;

	double a =  Vp(0);
	double b = -Vq(0);
	double c =  Vp(1);
	double d = -Vq(1);

	double delta = a*d-b*c;
	if (delta == 0.0) {
		return false;
	}

	double tp = (d * pq(0) -b * pq(1)) / delta;

	result <<
		(1.0 - tp) * p1(0) + tp * p2(0),
		(1.0 - tp) * p1(1) + tp * p2(1);

	return true;
}

// -----------------------------------------------------------------------------

void clip_polygon_by_half_plane(const Eigen::MatrixXd &P_in,
	const Eigen::RowVector2d &q1, const Eigen::RowVector2d &q2, Eigen::MatrixXd &P_out)
{
	using namespace GEO;
	assert(P_in.cols() == 2);
	std::vector<Eigen::RowVector2d> result;

	if (P_in.rows() == 0) {
		P_out.resize(0, 2);
		return ;
	}

	if (P_in.rows() == 1) {
		if (point_is_in_half_plane(P_in.row(0), q1, q2)) {
			P_out.resize(1, 2);
			P_out << P_in.row(0);
		} else {
			P_out.resize(0, 2);
		}
		return;
	}

	Eigen::RowVector2d prev_p = P_in.row(P_in.rows() - 1);
	Sign prev_status = point_is_in_half_plane(prev_p, q1, q2);

	for (unsigned int i = 0; i < P_in.rows(); ++i) {
		Eigen::RowVector2d p = P_in.row(i);
		Sign status = point_is_in_half_plane(p, q1, q2);
		if (status != prev_status && status != ZERO && prev_status != ZERO) {
			Eigen::RowVector2d intersect;
			if (intersect_segments(prev_p, p, q1, q2, intersect)) {
				result.push_back(intersect) ;
			}
		}

		switch(status) {
		case NEGATIVE:
			break ;
		case ZERO:
			result.push_back(p) ;
			break ;
		case POSITIVE:
			result.push_back(p) ;
			break ;
		default:
			break;
		}

		prev_p = p ;
		prev_status = status ;
	}

	P_out.resize((int) result.size(), 2);
	for (size_t i = 0; i < result.size(); ++i) {
		P_out.row((int) i) = result[i];
	}
}


////////////////////////////////////////////////////////////////////////////////

void intersect_half_planes(const Eigen::MatrixXd &P_in, Eigen::MatrixXd &V) {
	GEO::PCK::initialize();

	// std::cout << P << std::endl;

	// 0. Normalize plane equations
	Eigen::MatrixXd P = P_in;
	for (int i = 0; i < P.rows(); ++i) {
		P.row(i) /= P.row(i).head<2>().norm();
	}

	// 1. Compute pairwise intersections and their bounding box
	typedef Eigen::Hyperplane<double, 2> Hyperplane2d;
	typedef Eigen::ParametrizedLine<double, 2> Line2d;
	assert(P.cols() == 3);
	Eigen::AlignedBox2d box;
	for (int i = 0; i < P.rows(); ++i) {
		Hyperplane2d pi(P.row(i).head<2>(), P(i, 2));
		for (int j = 0; j < i; ++j) {
			Hyperplane2d pj(P.row(j).head<2>(), P(j, 2));
			// If the lines are parallel, this functions returns a points on the first line,
			// which should still works for what we want.
			Eigen::Vector2d m = pi.intersection(pj);
			box.extend(m);
		}
	}
	// 2. Clip polygon by each half-plane and return result
	Eigen::MatrixXd poly(4, 2);
	poly <<
		box.min()[0], box.min()[1],
		box.max()[0], box.min()[1],
		box.max()[0], box.max()[1],
		box.min()[0], box.max()[1];
	Eigen::MatrixXd tmp;
	// std::ofstream oss("/home/jdumas/public/tmp/foo.obj");
	// std::vector<std::pair<int, int>> edges;
	for (int i = 0; i < P.rows(); ++i) {
		Line2d line(Hyperplane2d(P.row(i).head<2>(), P(i, 2)));
		Eigen::RowVector2d a = line.pointAt(0).transpose();
		Eigen::RowVector2d b = line.pointAt(1).transpose();
		clip_polygon_by_half_plane(poly, a, b, tmp);
		std::swap(poly, tmp);
		// std::cout << a(0) * P(i, 0) + a(1) * P(i, 1) + P(i, 2) << std::endl;
		// std::cout << b(0) * P(i, 0) + b(1) * P(i, 1) + P(i, 2) << std::endl;
		// std::cout << "[(" << a(0) << ", " << a(1) << "),";
		// std::cout << "(" << b(0) << ", " << b(1) << ")]" << std::endl;
		// edges.emplace_back(2*i+1, 2*i+2);
	}
	V = poly;
	// if (V.col(0).mean() < 0) {
	// 	qfm::logger().error("Negative Young's modulus\n{}", P);
	// }
	// for (auto e : edges) {
	// 	oss << "l " << e.first << " " << e.second << std::endl;
	// }
	// {
	// 	static int cnt = 0;
	// 	std::ofstream oss("/home/jdumas/public/tmp/cst_" + std::to_string(cnt++) + ".obj");
	// 	for (int i = 0; i < V.rows(); ++i) {
	// 		oss << "v " << V(i, 0) << " " << V(i, 1) << " 0\n";
	// 	}
	// 	oss << "f";
	// 	for (int i = 0; i < V.rows(); ++i) {
	// 		oss << " " << i+1;
	// 	}
	// 	oss << std::endl;
	// }
	// exit(0);
}

// -----------------------------------------------------------------------------

double polygon_area(const Eigen::MatrixXd &V) {
	double sum = 0;
	for (int i = 1; i < V.rows() - 1; ++i) {
		Eigen::Matrix2d M;
		M.row(0) = (V.row(i) - V.row(0)).head<2>();
		M.row(1) = (V.row(i + 1) - V.row(0)).head<2>();
		sum += M.determinant();
	}
	return std::abs(sum / 2.0);
}
