#pragma once

#include <string>
#include "EigenAll.h"

inline std::string GetSuffix(const std::string& filename) {
	return filename.substr(filename.find_last_of('.') + 1);
}

class FileIO {
public:
	void LoadFromFile(const std::string& filename, bool centered, VectorXd& vertices, MatrixXi& topo) const;
	virtual void RealLoadFromFile(const std::string& filename, VectorXd& vertices, MatrixXi& topo) const = 0;
	void SaveToFile(const std::string& filename, bool centered, const VectorXd& vertices, const MatrixXi& topo) const;
	virtual void RealSaveToFile(const std::string& filename, const VectorXd& vertices, const MatrixXi& topo) const = 0;

	virtual ~FileIO() = default;
};