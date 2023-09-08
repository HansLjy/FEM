#include "ObjIO.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "spdlog/spdlog.h"
#include "Pattern.h"
#include <igl/readOBJ.h>

void ObjIO::RealLoadFromFile(const std::string &filename, VectorXd &vertices, MatrixXi &topo) const {
	MatrixXd vertices_mat;
	std::cerr << filename << std::endl;
	bool success = igl::readOBJ(filename, vertices_mat, topo);
	if (!success) {
		spdlog::error("Invalid input file {}", filename);
	}

	vertices.resize(vertices_mat.rows() * 3);

	const int num_points = vertices_mat.rows();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		vertices.segment<3>(i3) = vertices_mat.row(i).transpose();
	}
}

void ObjIO::RealSaveToFile(const std::string &filename, const VectorXd &vertices, const MatrixXi &topo) const {
	// TODO: 
	throw std::logic_error("unimplemented method");
}