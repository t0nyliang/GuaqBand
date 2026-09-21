//
// Created by Davi Colli Tozoni on 3/12/22.
//

#ifndef MATOPTIMIZATION_CONVERT_H
#define MATOPTIMIZATION_CONVERT_H

#include <MeshFEM/MeshIO.hh>
#include <Eigen/Dense>

// From Eigen to MeshIO (edges, triangles or quads)
void to_meshfem(
	const Eigen::MatrixXd &VI,
	const Eigen::MatrixXi &FI,
	std::vector<MeshIO::IOVertex> &VO,
	std::vector<MeshIO::IOElement> &FO);

// From MeshIO to Eigen (triangle mesh)
void from_meshfem(
	const std::vector<MeshIO::IOVertex> &VI,
	const std::vector<MeshIO::IOElement> &FI,
	Eigen::MatrixXd &VO,
	Eigen::MatrixXi &FO);

#endif //MATOPTIMIZATION_CONVERT_H
