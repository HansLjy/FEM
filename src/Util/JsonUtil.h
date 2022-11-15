//
// Created by hansljy on 10/14/22.
//

#ifndef FEM_JSONUTIL_H
#define FEM_JSONUTIL_H

#include "EigenAll.h"
#include "nlohmann/json.hpp"

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
Matrix3d Json2Matrix3d(const json& mat);

#endif //FEM_JSONUTIL_H
