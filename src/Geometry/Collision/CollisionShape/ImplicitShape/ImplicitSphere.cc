#include "ImplicitSphere.hpp"
#include "GeometryUtil.hpp"

ImplicitSphere ImplicitSphere::CreateFromConfig(const json &config) {
	return {
		Json2Vec(config["center"]),
		config["radius"]
	};
}


bool ImplicitSphere::Intersect(const Vector3d& x) const {
	return (x - _center).norm() < _radius;
}

bool ImplicitSphere::Intersect(const Vector3d& x1, const Vector3d& x2, double& ratio) const {
	ratio = GeometryUtil::GetClosestPoint(x1, x2, _center);
	return ratio >= 0 && ratio <= 1;
}

bool ImplicitSphere::Intersect(const Vector3d& x1, const Vector3d& x2, const Vector3d& x3, Vector2d& ratio) const {
	ratio = GeometryUtil::GetPointPlaneClosestPoint(_center, x1, x2, x3);
	return ratio[0] >= 0 && ratio[1] >= 0 && ratio[0] + ratio[1] <= 1;
}

Vector3d ImplicitSphere::SurfaceProject(const Vector3d &point) const {
	return _center + (point - _center).normalized() * _radius;
}