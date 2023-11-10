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

Matrix3d JsonUtils::ReadJsonRotationList(const json &rotation_list) {
	Matrix3d rotation = Matrix3d::Identity();
	for (const auto& rotation_config : rotation_list) {
		const Vector3d axis = Json2Vec(rotation_config["axis"]).normalized();
		const double angle = static_cast<double>(rotation_config["angle"]) / 180 * M_PI;
		rotation = AngleAxisd(angle, axis) * rotation;
	}
	return rotation;
}