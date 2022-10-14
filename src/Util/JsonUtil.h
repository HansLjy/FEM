//
// Created by hansljy on 10/14/22.
//

#ifndef FEM_JSONUTIL_H
#define FEM_JSONUTIL_H

#include "EigenAll.h"
#include "nlohmann/json.hpp"

using nlohmann::json;

Vector3d Json2Vec(const json& vec);

#endif //FEM_JSONUTIL_H
