#include "convert.h"

// From Eigen to MeshIO
void to_meshfem(
	const Eigen::MatrixXd &VI,
	const Eigen::MatrixXi &FI,
	std::vector<MeshIO::IOVertex> &VO,
	std::vector<MeshIO::IOElement> &FO)
{
	VO.resize(VI.rows());
	FO.resize(FI.rows());
	for (int v = 0; v < (int) VO.size(); ++v) {
		for (int c = 0; c < VI.cols(); ++c) {
			VO[v][c] = (c == 2 ? 0 : VI(v, c));
		}
	}
	for (int f = 0; f < (int) FO.size(); ++f) {
		FO[f] = MeshIO::IOElement(FI.cols());
		for (int lv = 0; lv < FI.cols(); ++lv) {
			FO[f][lv] = FI(f, lv);
		}
	}
}

// From MeshIO to Eigen (triangle mesh)
void from_meshfem(
	const std::vector<MeshIO::IOVertex> &VI,
	const std::vector<MeshIO::IOElement> &FI,
	Eigen::MatrixXd &VO,
	Eigen::MatrixXi &FO)
{
	VO.resize(VI.size(), 3);
	FO.resize(FI.size(), 3);
	for (int v = 0; v < (int) VI.size(); ++v) {
		for (int c = 0; c < 3; ++c) {
			VO(v, c) = VI[v][c];
		}
	}
	for (int f = 0; f < (int) FI.size(); ++f) {
		assert(FI[f].size() == 3);
		for (int lv = 0; lv < 3; ++lv) {
			FO(f, lv) = FI[f][lv];
		}
	}
}
