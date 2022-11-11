//
// Created by hansljy on 10/14/22.
//

#include "JsonUtil.h"

Vector3d Json2Vec(const json& vec) {
    Vector3d result;
    result << vec[0], vec[1], vec[2];
    return result;
}

VectorXd Json2VecX(const json& vec) {
    VectorXd result(vec.size());
    for (int i = 0; i < vec.size(); i++) {
        result(i) = vec[i];
    }
    return result;
}

Matrix3d Json2Matrix3d(const json& mat) {
    return (Matrix3d() <<
        mat[0], mat[1], mat[2],
        mat[3], mat[4], mat[5],
        mat[6], mat[7], mat[8]
    ).finished();
}