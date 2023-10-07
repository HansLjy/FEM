#include "FileIO.hpp"
#include "Pattern.h"
#include "ObjIO.hpp"

template<>
Factory<FileIO>* Factory<FileIO>::_the_factory = nullptr;

namespace {
	const bool obj_io_registered = Factory<FileIO>::GetInstance()->Register("obj", []() {
		return new ObjIO;
	});
}


bool FileIOHelper::_already_read = false;
VectorXd FileIOHelper::_the_vertices;
MatrixXi FileIOHelper::_the_topo;

void FileIOHelper::ReadMesh(const std::string& filename, bool centered) {
	if (!_already_read) {
		auto suffix = filename.substr(filename.find_last_of('.') + 1);
		auto io = Factory<FileIO>::GetInstance()->GetProduct(suffix);
		io->LoadFromFile(MODEL_PATH + filename, centered, _the_vertices, _the_topo);
		_already_read = true;
		delete io;
	}
}

void FileIO::LoadFromFile(const std::string &filename, bool centered, VectorXd &vertices, MatrixXi &topo) const {
	RealLoadFromFile(filename, vertices, topo);
	if (centered) {
		Vector3d max_coord = vertices.segment<3>(0);
		Vector3d min_coord = vertices.segment<3>(0);
		const int num_points = vertices.size() / 3;
		for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
			max_coord = max_coord.cwiseMax(vertices.segment<3>(i3));
			min_coord = min_coord.cwiseMin(vertices.segment<3>(i3));
		}

		double scale_ratio = (max_coord - min_coord).maxCoeff() / 2;
		Vector3d center = (max_coord + min_coord) / 2;
		for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
			vertices.segment<3>(i3) -= center;
			vertices.segment<3>(i3) /= scale_ratio;
		}
	}
}

void FileIO::SaveToFile(const std::string &filename, bool centered, const VectorXd &vertices, const MatrixXi &topo) const {
	throw std::logic_error("unimplemented method");
}
