#pragma once

#include <string>
#include "EigenAll.h"

class FileIO {
public:
	virtual void LoadFromFile(const std::string& filename, VectorXd& vertices, MatrixXi& topo) const = 0;
	virtual void SaveToFile(const std::string& filename, const VectorXd& vertices, const MatrixXi& topo) const = 0;
};