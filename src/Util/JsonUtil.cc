//
// Created by hansljy on 10/14/22.
//

#include "JsonUtil.h"

Matrix3d Json2Matrix3d(const json& mat) {
    return (Matrix3d() <<
        mat[0], mat[1], mat[2],
        mat[3], mat[4], mat[5],
        mat[6], mat[7], mat[8]
    ).finished();
}

glm::vec3 Json2GlmVec3(const json& vec) {
    return {vec[0], vec[1], vec[2]};
}