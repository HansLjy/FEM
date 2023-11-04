#include "ECCD.hpp"
#include "CTCD.h"

ECCD* ECCD::CreateFromConfig(const json &config) {
	return new ECCD(static_cast<double>(config["tolerance"]));
}

double ECCD::VertexFaceCollision(
	const Vector3d &x, const Vector3d &x1, const Vector3d &x2, const Vector3d &x3,
	const Vector3d &v, const Vector3d &v1, const Vector3d &v2, const Vector3d &v3
) {
	double hit_time;
	bool hit = CTCD::vertexFaceCTCD(x, x1, x2, x3, x + v, x1 + v1, x2 + v2, x3 + v3, _tolerance, hit_time);
	if (hit) {
		return hit_time;
	} else {
		return 2;
	}
}

double ECCD::EdgeEdgeCollision(
	const Vector3d &x11, const Vector3d &x12, const Vector3d &x21, const Vector3d &x22,
	const Vector3d &v11, const Vector3d &v12, const Vector3d &v21, const Vector3d &v22
) {
	double hit_time;
	bool hit = CTCD::edgeEdgeCTCD(x11, x12, x21, x22, x11 + v11, x12 + v12, x21 + v21, x22 + v22, _tolerance, hit_time);
	if (hit) {
		return hit_time;
	} else {
		return 2;
	}
}

