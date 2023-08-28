#include "ObjIO.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "spdlog/spdlog.h"
#include "Pattern.h"

void ObjIO::LoadFromFile(const std::string &filename, VectorXd &vertices, MatrixXi &topo) const {
	std::ifstream obj_file(filename);
	std::string line;
	std::getline(obj_file, line);

	int vertex_cnt = 0, face_cnt = 0;
	while(obj_file.good()) {
		std::stringstream line_stream(line);
		std::string first_word;
		line_stream >> first_word;
		if (first_word == "v") {
			vertex_cnt++;
		} else if (first_word == "f") {
			face_cnt++;
		} else if (first_word != "#") {
			std::cerr << "unexpected instruction " << first_word << " detected, neglect it and continue..." << std::endl; 
		}
		std::getline(obj_file, line);
	}
	vertices.resize(vertex_cnt * 3);
	topo.resize(face_cnt, 3);

	spdlog::info("Mesh loaded, #vertices = {}, #faces = {}", vertex_cnt, face_cnt);

	obj_file.clear();
	obj_file.seekg(0);
	
	face_cnt = 0;
	int vertex_cnt3 = 0;
	std::getline(obj_file, line);
	while(obj_file.good()) {
		std::stringstream line_stream(line);
		std::string first_word;
		double x, y, z;
		int face_id[3];
		line_stream >> first_word;
		if (first_word == "v") {
			line_stream >> x >> y >> z;
			vertices.segment<3>(vertex_cnt3) << x, y, z;
			vertex_cnt3 += 3;
		} else if (first_word == "f") {
			line_stream >> face_id[0] >> face_id[1] >> face_id[2];
			for (int i = 0; i < 3; i++) {
				if (face_id[i] > 0) {
					topo(face_cnt, i) = face_id[i] - 1;
				} else {
					topo(face_cnt, i) = face_cnt - face_id[i];
				}
			}
			face_cnt++;
		}
		std::getline(obj_file, line);
	}
}

void ObjIO::SaveToFile(const std::string &filename, const VectorXd &vertices, const MatrixXi &topo) const {
	// TODO: 
	throw std::logic_error("unimplemented method");
}