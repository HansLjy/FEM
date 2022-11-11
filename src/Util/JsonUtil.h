//
// Created by hansljy on 10/14/22.
//

#ifndef FEM_JSONUTIL_H
#define FEM_JSONUTIL_H

#include "EigenAll.h"
#include "nlohmann/json.hpp"

using nlohmann::json;

Vector3d Json2Vec(const json& vec);
VectorXd Json2VecX(const json& vec);
Matrix3d Json2Matrix3d(const json& mat);

#endif //FEM_JSONUTIL_H
