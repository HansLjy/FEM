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

Vector2d GeometryUtil::GetLineLineClosestPoint(const Vector3d& x11, const Vector3d& x12, const Vector3d& x21, const Vector3d& x22) {
	Vector3d e1 = x12 - x11;
	Vector3d e2 = x22 - x21;
	Vector3d e12 = x21 - x11;
	Matrix2d A;
	A << e1.dot(e1), -e1.dot(e2),
		 -e1.dot(e2), e2.dot(e2);

	Vector2d b;
	b << e12.dot(e1),
		 -e12.dot(e2);
		
	return A.inverse() * b;
}
