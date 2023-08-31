#include "ObjIO.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "spdlog/spdlog.h"
#include "Pattern.h"
#include <igl/readOBJ.h>

void ObjIO::LoadFromFile(const std::string &filename, VectorXd &vertices, MatrixXi &topo) const {
	MatrixXd vertices_mat;
	std::cerr << filename << std::endl;
	bool success = igl::readOBJ(filename, vertices_mat, topo);
	if (!success) {
		spdlog::error("Invalid input file {}", filename);
	}

	vertices.resize(vertices_mat.rows() * 3);

	Vector3d max_coord = vertices_mat.row(0), min_coord = vertices_mat.row(0);
	const int num_points = vertices_mat.rows();
	for (int i = 0; i < num_points; i++) {
		max_coord = max_coord.cwiseMax(vertices_mat.row(i).transpose());
		min_coord = min_coord.cwiseMin(vertices_mat.row(i).transpose());
	}

	double scale_ratio = (max_coord - min_coord).maxCoeff() / 2;
	Vector3d center = (max_coord + min_coord) / 2;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		vertices.segment<3>(i3) = (vertices_mat.row(i).transpose() - center) / scale_ratio;
	}
}

void ObjIO::SaveToFile(const std::string &filename, const VectorXd &vertices, const MatrixXi &topo) const {
	// TODO: 
	throw std::logic_error("unimplemented method");
}