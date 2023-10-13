#include "GeometryUtil.hpp"

double GeometryUtil::GetCot(const Vector3d &a, const Vector3d &b) {
    const Vector3d a_normalized = a.normalized();
    const Vector3d b_normalized = b.normalized();
    const double c = a_normalized.dot(b_normalized);
    const double s = a_normalized.cross(b_normalized).norm();
    return c / s;
}