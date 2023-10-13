#include "GeometryUtil.hpp"

double GeometryUtil::GetCot(const Vector3d &a, const Vector3d &b) {
    const Vector3d a_normalized = a.normalized();
    const Vector3d b_normalized = b.normalized();
    const double c = a_normalized.dot(b_normalized);
    const double s = a_normalized.cross(b_normalized).norm();
    return c / s;
}

double GeometryUtil::GetClosestPoint(const Vector3d& x1, const Vector3d& x2, const Vector3d& c) {
	return (c - x1).dot(x2 - x1) / (x2 - x1).squaredNorm();
}

Vector2d GeometryUtil::GetClosestPoint(const Vector3d& x1, const Vector3d& x2, const Vector3d& x3, const Vector3d& c) {
	Vector3d x21 = x2 - x1;
	Vector3d x31 = x3 - x1;
	Vector3d cx1 = c - x1;
	Matrix2d A;
	A << x21.dot(x21), x31.dot(x21),
		 x21.dot(x31), x31.dot(x31);
	
	Vector2d b;
	b << cx1.dot(x21),
		 cx1.dot(x31);
	
	return A.inverse() * b;
}
