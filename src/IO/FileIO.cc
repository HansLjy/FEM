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

void FileIOUtils::ReadMesh(
	const std::string &filename,
	VectorXd &vertices,
	MatrixXi &topo,
	bool centered,
	const Matrix3d& rotation,
	const Vector3d& translation
) {
	auto suffix = filename.substr(filename.find_last_of('.') + 1);
	auto io = Factory<FileIO>::GetInstance()->GetProduct(suffix);
	io->LoadFromFile(MODEL_PATH + filename, vertices, topo, centered, rotation, translation);
	delete io;
}

void FileIOUtils::WriteMesh(
	const std::string &filename,
	const MatrixXd &vertices, const MatrixXi &topo,
	const Matrix3d& rotation, const Vector3d& translation
) {
	auto suffix = filename.substr(filename.find_last_of('.') + 1);
	auto io = Factory<FileIO>::GetInstance()->GetProduct(suffix);
	io->SaveToFile(MODEL_PATH + filename, vertices, topo, rotation, translation);
	delete io;
}

void FileIO::LoadFromFile(
	const std::string &filename,
	VectorXd &vertices,
	MatrixXi &topo,
	bool centered,
	const Matrix3d &rotation,
	const Vector3d &translation
) const {
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
	const int num_points = vertices.size() / 3;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		vertices.segment<3>(i3) = rotation * vertices.segment<3>(i3) + translation;
	}
}

void FileIO::SaveToFile(
	const std::string &filename,
	const MatrixXd &vertices, const MatrixXi &topo,
	const Matrix3d &rotation, const Vector3d &translation
) const {
	const int num_points = vertices.rows();
	MatrixXd translated_vertices(vertices.rows(), vertices.cols());
	for (int i = 0; i < num_points; i++) {
		translated_vertices.row(i) = vertices.row(i) * rotation + translation.transpose();
	}
	RealSaveToFile(filename, translated_vertices, topo);
}
