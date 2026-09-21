#ifndef MATOPTIMIZATION_MESH_UTILS_H
#define MATOPTIMIZATION_MESH_UTILS_H

#include <Eigen/Dense>

void mesh_union(
	const Eigen::MatrixXd &V1, const Eigen::MatrixXi &F1,
	const Eigen::MatrixXd &V2, const Eigen::MatrixXi &F2,
	const std::string &flags,
	Eigen::MatrixXd &V, Eigen::MatrixXi &F);

void mesh_difference(
	const Eigen::MatrixXd &V1, const Eigen::MatrixXi &F1,
	const Eigen::MatrixXd &V2, const Eigen::MatrixXi &F2,
	const std::string &flags,
	Eigen::MatrixXd &V, Eigen::MatrixXi &F);

void offset(const Eigen::MatrixXd &V,
	const Eigen::MatrixXi &F,
	Eigen::MatrixXd &OV,
	Eigen::MatrixXi &OF,
	double offset);

#endif //MATOPTIMIZATION_MESH_UTILS_H
