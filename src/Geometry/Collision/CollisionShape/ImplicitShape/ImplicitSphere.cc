#include "ImplicitSphere.hpp"
#include "GeometryUtil.hpp"

bool ImplicitSphere::Intersect(const Vector3d& x) const {
	return (x - _center).norm() < _radius;
}

bool ImplicitSphere::Intersect(const Vector3d& x1, const Vector3d& x2, double& ratio) const {
	ratio = GeometryUtil::GetClosestPoint(x1, x2, _center);
	return ratio >= 0 && ratio <= 1;
}

bool ImplicitSphere::Intersect(const Vector3d& x1, const Vector3d& x2, const Vector3d& x3, Vector2d& ratio) const {
	ratio = GeometryUtil::GetClosestPoint(x1, x2, x3, _center);
	return ratio[0] >= 0 && ratio[1] >= 0 && ratio[0] + ratio[1] <= 1;
}
