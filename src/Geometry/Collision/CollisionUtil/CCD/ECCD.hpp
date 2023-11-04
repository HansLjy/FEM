#pragma once
#include "CCD.h"

// wrapper of https://github.com/evouga/collisiondetection
class ECCD : public CCD {
public:
	static ECCD* CreateFromConfig(const json& config);
	ECCD(double tolerance) : _tolerance(tolerance) {}
	double VertexFaceCollision(const Vector3d &x, const Vector3d &x1, const Vector3d &x2, const Vector3d &x3, const Vector3d &v, const Vector3d &v1, const Vector3d &v2, const Vector3d &v3) override;

	double EdgeEdgeCollision(const Vector3d &x11, const Vector3d &x12, const Vector3d &x21, const Vector3d &x22, const Vector3d &v11, const Vector3d &v12, const Vector3d &v21, const Vector3d &v22) override;

protected:
	double _tolerance;
};