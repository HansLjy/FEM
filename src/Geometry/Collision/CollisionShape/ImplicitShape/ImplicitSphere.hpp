#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

class ImplicitSphere {
public:
	static ImplicitSphere CreateFromConfig(const json& config);
	bool Intersect(const Vector3d& x) const;
	bool Intersect(const Vector3d& x1, const Vector3d& x2, double& ratio) const;
	bool Intersect(const Vector3d& x1, const Vector3d& x2, const Vector3d& x3, Vector2d& ratio) const;

	//<- project to closest point on the surface
	Vector3d SurfaceProject(const Vector3d& point) const;

protected:
	Vector3d _center;
	double _radius;

	ImplicitSphere(const Vector3d& center, double radius)
		: _center(center), _radius(radius) {}
};