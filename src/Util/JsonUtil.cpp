//
// Created by hansljy on 10/14/22.
//

#include "JsonUtil.h"

Vector3d Json2Vec(const json& vec) {
    Vector3d result;
    result << vec[0], vec[1], vec[2];
    return result;
}