#include "mesh_utils.h"
#include "eigen_utils.h"
#include <matopt_clipper/clipper.hpp>
#include <Eigen/Dense>
#include <igl/boundary_loop.h>
#include <igl/triangle/triangulate.h>
#include <igl/doublearea.h>
#include <igl/barycenter.h>
#include <igl/bounding_box_diagonal.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/ramer_douglas_peucker.h>
#include <stack>

typedef std::vector<std::complex<double>> Polygon;
typedef std::vector<Polygon>              Polygons;

constexpr int FACTOR = 1<<23;

class Point {
public:
	static ClipperLib::IntPoint toClipper(const std::complex<double>& p) {
		ClipperLib::IntPoint r;
		r.X = (ClipperLib::cInt)std::round(real(p) * FACTOR);
		r.Y = (ClipperLib::cInt)std::round(imag(p) * FACTOR);
		return r;
	}

	static ClipperLib::IntPoint toClipper(const Eigen::Vector2d & p) {
		ClipperLib::IntPoint r;
		r.X = (ClipperLib::cInt)std::round(p.x() * FACTOR);
		r.Y = (ClipperLib::cInt)std::round(p.y() * FACTOR);
		return r;
	}

	static Eigen::Vector2d fromClipper(const ClipperLib::IntPoint& p) {
		return Eigen::Vector2d(p.X, p.Y) / FACTOR;
	}
};

void append_path(const ClipperLib::Path& path,
	std::vector<Eigen::Vector2d>& vertices,
	std::vector<std::pair<int, int>> &edges,
	bool closed = true)
{
	const size_t base_index = vertices.size();
	const size_t num_segments = path.size();
	for (auto p : path) {
		Eigen::Vector2d v = Point::fromClipper(p);
		vertices.push_back(v);
	}

	for (size_t i = 0; i < num_segments; i++) {
		if (closed || i + 1 < num_segments) {
			edges.emplace_back(base_index + i, base_index + ((i+1)%num_segments));
		}
	}
}

Eigen::Vector2d largest_triangle_barycenter(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F) {
	Eigen::VectorXd A;
	Eigen::MatrixXd B;
	igl::doublearea(V, F, A);
	igl::barycenter(V, F, B);
	double max_area = A.maxCoeff();
	for (int f = 0; f < F.rows(); ++f) {
		if (A(f) == max_area) {
			return B.row(f).transpose();
		}
	}
	return Eigen::Vector2d();
}

void triangulate(const Eigen::MatrixXd &V, const Eigen::MatrixXi &E,
	const Eigen::MatrixXd &H, Eigen::MatrixXd &VO, Eigen::MatrixXi &FO)
{
	igl::triangle::triangulate(V, E, H, "zqjQ", VO, FO);
}

Eigen::Vector2d get_point_inside_simple(const ClipperLib::Path &contour) {
	Eigen::MatrixXd V, H, TV;
	Eigen::MatrixXi E, TF;
	V.resize(contour.size(), 2);
	E.resize(contour.size(), 2);
	for (size_t i = 0; i < contour.size(); ++i) {
		V.row(i) = Point::fromClipper(contour[i]).transpose();
		E.row(i) << i, (i + 1) % contour.size();
	}
	triangulate(V, E, H, TV, TF);
	return largest_triangle_barycenter(TV, TF);
}

Eigen::Vector2d get_point_inside_nested(const ClipperLib::PolyNode &node) {
	Eigen::MatrixXd V, H, TV;
	Eigen::MatrixXi E, TF;
	size_t num_vertices = node.Contour.size();
	for (const ClipperLib::PolyNode * child : node.Childs) {
		num_vertices += child->Contour.size();
	}
	V.resize(num_vertices, 2);
	E.resize(num_vertices, 2);
	H.resize(node.ChildCount(), 2);
	// Main hole
	for (size_t i = 0; i < node.Contour.size(); ++i) {
		V.row(i) = Point::fromClipper(node.Contour[i]).transpose();
		E.row(i) << i, (i + 1) % node.Contour.size();
	}
	// Inner blobs
	size_t k = 0;
	size_t offset = node.Contour.size();
	for (const ClipperLib::PolyNode * child : node.Childs) {
		for (size_t i = 0; i < child->Contour.size(); ++i) {
			V.row(offset + i) = Point::fromClipper(child->Contour[i]).transpose();
			E.row(offset + i) << i, (i + 1) % child->Contour.size();
		}
		offset += child->Contour.size();
		H.row(k++) = get_point_inside_simple(child->Contour).transpose();
	}
	triangulate(V, E, H, TV, TF);
	return largest_triangle_barycenter(TV, TF);
}

void loops_to_segments_internal(const ClipperLib::PolyTree &polytree,
	Eigen::MatrixXd &V, Eigen::MatrixXi &E, Eigen::MatrixXd &H)
{
	// Extract resulting contours
	typedef std::pair<int, int> Edge;
	std::vector<Eigen::Vector2d> vertices;
	std::vector<Edge> edges;
	std::vector<Eigen::Vector2d> holes;
	std::stack<const ClipperLib::PolyNode *> pending;
	if (polytree.GetFirst()) {
		pending.push(polytree.GetFirst());
	}
	while (!pending.empty()) {
		const ClipperLib::PolyNode & outer = *pending.top();
		pending.pop();
		append_path(outer.Contour, vertices, edges);
		for (const ClipperLib::PolyNode * innerPtr: outer.Childs) {
			const ClipperLib::PolyNode &inner = *innerPtr;
			if (inner.IsOpen()) {
				throw std::runtime_error("An open polyline was found nested in the tree.");
			}
			append_path(inner.Contour, vertices, edges, !outer.IsOpen());
			if (inner.IsHole()) {
				if (inner.ChildCount() > 0) {
					// Hole has nested component, we need to deal with this...
					holes.push_back(get_point_inside_nested(inner));
					for (const ClipperLib::PolyNode * child : inner.Childs) {
						pending.push(child);
					}
				} else {
					// Hole is a simple guy.
					holes.push_back(get_point_inside_simple(inner.Contour));
				}
			} else {
				if (inner.ChildCount() > 0) {
					throw std::runtime_error("Logical error: a non-hole polygon has children.");
				}
			}
		}
		if (outer.GetNext()) {
			// Add next sibling node to visit (horizontal traversal)
			pending.push(outer.GetNext());
		}
	}

	// Convert std::vector into matrices
	V.resize(vertices.size(), 2);
	E.resize(edges.size(), 2);
	H.resize(holes.size(), 2);
	for (int i = 0; i < V.rows(); ++i) {
		V.row(i) << vertices[i].transpose();
	}
	for (int i = 0; i < E.rows(); ++i) {
		E.row(i) << edges[i].first, edges[i].second;
	}
	for (int i = 0; i < H.rows(); ++i) {
		H.row(i) << holes[i].transpose();
	}
}

void polygons_to_clipper(const Polygons &polygons, ClipperLib::PolyTree &polytree) {
	// Convert loops to clipper
	ClipperLib::Paths loops_in;
	for (const auto & loop : polygons) {
		loops_in.emplace_back();
		for (auto p : loop) {
			loops_in.back().push_back(Point::toClipper(p));
		}
	}

	ClipperLib::CleanPolygons(loops_in, loops_in);

	// Extract clean polytree with Clipper
	polytree.Clear();
	ClipperLib::Clipper clipper(
		ClipperLib::ioStrictlySimple | ClipperLib::ioPreserveCollinear);
	clipper.AddPaths(loops_in, ClipperLib::ptSubject, true);
	clipper.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
}

void remove_duplicated_vertices(const Eigen::MatrixXd& VI,
                                const Eigen::MatrixXi& SI,
                                Eigen::MatrixXd& VO,
                                Eigen::MatrixXi& SO,
                                double tolerance)
{
    Eigen::MatrixXd V(VI.rows(), 3);
    V.setZero();
    V.leftCols<2>() = VI.leftCols<2>();
    Eigen::VectorXi SVI, SVJ;
    igl::remove_duplicate_vertices(V, SI, tolerance, VO, SVI, SVJ, SO);
}

void remove_duplicated_and_degenerate_edges(const Eigen::MatrixXi& EI, Eigen::MatrixXi& EO)
{
    std::vector<std::array<int, 2>> edges;
    eigen_to_vector(EI, edges);

    auto is_degenerate = [](auto& table, size_t k) {
        const auto& elem = table[k];
        for (size_t i = 0; i < elem.size(); ++i) {
            for (size_t j = 0; j < i; ++j) {
                if (elem[i] == elem[j]) {
                    return true;
                }
            }
        }
        return false;
    };

    auto pop_element = [](auto& table, size_t k) {
        std::swap(table[k], table[table.size() - 1]);
        table.pop_back();
    };

    for (size_t i = 0; i < edges.size();) {
        // Remove edge if it contains duplicate vertices
        if (is_degenerate(edges, i)) {
            pop_element(edges, i);
            continue;
        }

        // Sort edge indices
        auto& e = edges[i];
        if (e[1] > e[0]) {
            std::swap(e[0], e[1]);
        }

        ++i;
    }

    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    edges.shrink_to_fit();

    vector_to_eigen(edges, EO);
}

Polygons boundary_polygons(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, double tolerance=0.0) {
	// Extract boundary loops
	std::vector<std::vector<int>> L;
	//if (F.cols() == 4) {
	//	Eigen::MatrixXi TF, TE;
	//	quad_to_tri(F, TF, TE);
	//	igl::boundary_loop(TF, L);
	//} else {
		igl::boundary_loop(F, L);
	//}

	// Convert to polygons
	Polygons polygons;
	for (auto &loop : L) {
		Eigen::MatrixXd P(loop.size() + 1, 2);
		for (int i = 0; i < (int) loop.size(); ++i) {
			P.row(i) << V.row(loop[i]).head<2>();
		}
		P.bottomRows<1>() << V.row(loop[0]).head<2>();

		Eigen::MatrixXd S;
		Eigen::VectorXi J;
		if (tolerance != 0.0) {
			igl::ramer_douglas_peucker(P, tolerance, S, J);
		} else {
			S = P;
		}

		Polygon poly;
		poly.reserve(S.rows() - 1);
		for (int i = 0; i < S.rows() - 1; ++i) {
			poly.emplace_back(S(i, 0), S(i, 1));
		}
		polygons.emplace_back(std::move(poly));
	}

	return polygons;
}

void edges_to_clipper(const Eigen::MatrixXd &V, const Eigen::MatrixXi &E, ClipperLib::Paths &paths)
{
	// Convert edges to clipper open paths
	paths.clear();
	for (int e = 0; e < E.rows(); ++e) {
		paths.push_back({
			Point::toClipper(V.row(E(e, 0)).transpose().head<2>()),
			Point::toClipper(V.row(E(e, 1)).transpose().head<2>())
		});
	}
}

void combine_polytrees(
	const ClipperLib::PolyTree &polytreeA,
	const ClipperLib::PolyTree &polytreeB,
	const ClipperLib::Paths &openLines,
	ClipperLib::ClipType operation,
	ClipperLib::PolyTree &result)
{
	// Extract closed loops
	ClipperLib::Paths pathsA, pathsB;
	ClipperLib::ClosedPathsFromPolyTree(polytreeA, pathsA);
	ClipperLib::ClosedPathsFromPolyTree(polytreeB, pathsB);

	// Perform intersection with Clipper
	result.Clear();
	ClipperLib::Clipper clipper(ClipperLib::ioStrictlySimple | ClipperLib::ioPreserveCollinear);
	clipper.AddPaths(pathsA, ClipperLib::ptSubject, true);
	if (!openLines.empty()) {
		clipper.AddPaths(openLines, ClipperLib::ptSubject, false);
	}
	clipper.AddPaths(pathsB, ClipperLib::ptClip, true);
	clipper.Execute(operation, result, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
}

void offset_polytree(const ClipperLib::PolyTree &polytree, double offset, ClipperLib::PolyTree &result) {
	// Perform offsetting
	ClipperLib::Paths paths;
	ClipperLib::ClosedPathsFromPolyTree(polytree, paths);

	result.Clear();
	ClipperLib::ClipperOffset co;
	co.AddPaths(paths, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
	co.Execute(result, offset * FACTOR);
}

void triangulate_internal(
	const Polygons &polygonsA,
	const Polygons &polygonsB,
	const Eigen::MatrixXd &LV,
	const Eigen::MatrixXi &LE,
	const Eigen::MatrixXd &P,
	double offset,
	double thres_rel,
	const std::string &flags,
	Eigen::MatrixXd &VO,
	Eigen::MatrixXi &FO,
	ClipperLib::ClipType operation = ClipperLib::ctIntersection)
{
	// Get Clipper polytree
	ClipperLib::PolyTree polytree;
	polygons_to_clipper(polygonsA, polytree);
	if (offset != 0.0) {
		offset_polytree(polytree, offset, polytree);
	}
	if (!polygonsB.empty()) {
		ClipperLib::PolyTree polytreeB;
		polygons_to_clipper(polygonsB, polytreeB);
		ClipperLib::Paths lines;
		if (LV.rows() > 0 && LE.rows() > 0) {
			edges_to_clipper(LV, LE, lines);
		}
		combine_polytrees(polytree, polytreeB, lines, operation, polytree);
	}

	// Convert to segments
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi segments;
	Eigen::MatrixXd H;
	loops_to_segments_internal(polytree, vertices, segments, H);

	// Append additional vertices
	vertices.conservativeResize(vertices.rows() + P.rows(), 2);
	if (P.rows() > 0) {
		vertices.bottomRows(P.rows()) = P.leftCols<2>();
	}

	if (vertices.rows() < 3) {
		// Empty result
		VO.resize(0, 3);
		FO.resize(0, 3);
		return;
	}

	{
		double thres = thres_rel * igl::bounding_box_diagonal(vertices);
		remove_duplicated_vertices(vertices, segments, vertices, segments, thres);
		remove_duplicated_and_degenerate_edges(segments, segments);
		vertices.conservativeResize(vertices.rows(), 2);
	}

	/////////////////////////////////////////////////////////////////////
	//
	// Triangulate boundary polygon
	//
	/////////////////////////////////////////////////////////////////////
	// q20.0: quality mesh generation (no angle less than 20.0 degree) //
	// Y    : no steiner point on boundary                             //
	// S0   : maximum number of added Steiner points                   //
	// a0.1 : maximum area constraint                                  //
	/////////////////////////////////////////////////////////////////////
	std::string default_flags = "zq20pQ" + flags;

	Eigen::MatrixXd VV;
	igl::triangle::triangulate(vertices, segments, H, default_flags, VV, FO);
	VO.resize(VV.rows(), 3);
	VO.leftCols<2>() = VV;
	VO.rightCols<1>().setZero();
}

void triangulate_union(const Polygons &polygonsA, const Polygons &polygonsB, const std::string &flags, Eigen::MatrixXd &VO, Eigen::MatrixXi &FO)
{
	Eigen::MatrixXd P;
	Eigen::MatrixXd LV;
	Eigen::MatrixXi LE;
	triangulate_internal(polygonsA, polygonsB, LV, LE, P, 0.0, 0.0, flags, VO, FO, ClipperLib::ctUnion);
}

void triangulate_difference(const Polygons &polygonsA, const Polygons &polygonsB, const std::string &flags, Eigen::MatrixXd &VO, Eigen::MatrixXi &FO)
{
	Eigen::MatrixXd P;
	Eigen::MatrixXd LV;
	Eigen::MatrixXi LE;
	triangulate_internal(polygonsA, polygonsB, LV, LE, P, 0.0, 0.0, flags, VO, FO, ClipperLib::ctDifference);
}

void triangulate_with_offset(const Polygons &polygons, double offset, const std::string &flags, Eigen::MatrixXd &VO, Eigen::MatrixXi &FO)
{
	Eigen::MatrixXd P, LV;
	Eigen::MatrixXi LE;
	triangulate_internal(polygons, {}, LV, LE, P, offset, 0.0, flags, VO, FO);
}

void mesh_union(
	const Eigen::MatrixXd &V1, const Eigen::MatrixXi &F1,
	const Eigen::MatrixXd &V2, const Eigen::MatrixXi &F2,
	const std::string &flags,
	Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
	// Extract boundary loops
	Polygons loops_1 = boundary_polygons(V1, F1);
	Polygons loops_2 = boundary_polygons(V2, F2);

	// Triangulate union
	triangulate_union(loops_1, loops_2, flags, V, F);
}

void mesh_difference(
	const Eigen::MatrixXd &V1, const Eigen::MatrixXi &F1,
	const Eigen::MatrixXd &V2, const Eigen::MatrixXi &F2,
	const std::string &flags,
	Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
	// Extract boundary loops
	Polygons loops_1 = boundary_polygons(V1, F1);
	Polygons loops_2 = boundary_polygons(V2, F2);

	// Triangulate difference
	triangulate_difference(loops_1, loops_2, flags, V, F);
}

void offset(const Eigen::MatrixXd &V,
	const Eigen::MatrixXi &F,
	Eigen::MatrixXd &OV,
	Eigen::MatrixXi &OF,
	double offset)
{
	// Triangulate the offset contours
	triangulate_with_offset(boundary_polygons(V, F), offset, "", OV, OF);
	OV.conservativeResize(OV.rows(), V.cols());
	if (V.cols() == 3) { OV.col(2).setZero(); }
}

