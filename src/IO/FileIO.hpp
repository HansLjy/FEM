#pragma once

#include <string>
#include "EigenAll.h"

inline std::string GetSuffix(const std::string& filename) {
	return filename.substr(filename.find_last_of('.') + 1);
}

class FileIO {
public:
	void LoadFromFile(
		const std::string& filename,
		VectorXd& vertices, MatrixXi& topo,
		bool centered,
		const Matrix3d& rotation,
		const Vector3d& translation
	) const;

	virtual void RealLoadFromFile(const std::string& filename, VectorXd& vertices, MatrixXi& topo) const = 0;
	
	void SaveToFile(
		const std::string& filename,
		const MatrixXd& vertices, const MatrixXi& topo,
		const Matrix3d& rotation, const Vector3d& translation
	) const;

	virtual void RealSaveToFile(const std::string& filename, const MatrixXd& vertices, const MatrixXi& topo) const = 0;

	virtual ~FileIO() = default;
};

namespace FileIOUtils {
	// file name is rel to model directory
	void ReadMesh(
		const std::string& filename,
		VectorXd& vertices, MatrixXi& topo,
		bool centered = false,
		const Matrix3d& rotation = Matrix3d::Identity(),
		const Vector3d& translation = Vector3d::Zero()
	);

	// file name is rel to model directory
	void WriteMesh(
		const std::string& filename,
		const MatrixXd& vertices, const MatrixXi& topo,
		const Matrix3d& rotation = Matrix3d::Identity(),
		const Vector3d& translation = Vector3d::Zero()
	);
}

namespace DebugUtils {
	void DumpVertexList(const std::string& name, const std::vector<Vector3d> vertex_list);
}