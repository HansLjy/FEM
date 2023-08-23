//
// Created by hansljy on 10/14/22.
//

#ifndef FEM_JSONUTIL_H
#define FEM_JSONUTIL_H

#include "EigenAll.h"
#include "nlohmann/json.hpp"
#include "glm/glm.hpp"

using nlohmann::json;

template<int dim=3>
Vector<double, dim> Json2Vec(const json& vec) {
    Vector<double, dim> vector;
    for (int i = 0; i < dim; i++) {
        vector(i) = vec[i];
    }
    return vector;
}
VectorXd Json2VecX(const json& vec);

template<int cols>
MatrixXi Json2MatXi(const json& vec) {
	int rows = vec.size() / cols;
	MatrixXi mat(rows, cols);
	int index = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			mat(i, j) = vec[index++];
		}
	}
	return mat;
}

Matrix3d Json2Matrix3d(const json& mat);

glm::vec3 Json2GlmVec3(const json& vec);

#endif //FEM_JSONUTIL_H
