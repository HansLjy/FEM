#pragma once

#include "EigenAll.h"

namespace GeometryUtil {
    //<- cotangent of the smaller angle between a and b
    double GetCot(const Vector3d& a, const Vector3d& b);

	//<- return alpha where x1 + alpha(x2 - x1) is the closest point to c on x1->x2
	double GetClosestPoint(const Vector3d& x1, const Vector3d& x2, const Vector3d& c);

	//<- return alpha, beta where x1 + alpha(x2 - x1) + beta(x3 - x1) is the closest point to c on the plane(x1, x2, x3)
	Vector2d GetPointPlaneClosestPoint(const Vector3d& x, const Vector3d& x1, const Vector3d& x2, const Vector3d& x3);
	double GetPointPlaneDistance(const Vector3d& x, const Vector3d& x1, const Vector3d& x2, const Vector3d& x3);

	//<- return alpha, beta where x11 + alpha(x12 - x11), x21 + beta(x22 - x21) are the closest point on the two line x11 -> x12, x21 -> x22
	Vector2d GetLineLineClosestPoint(const Vector3d& x11, const Vector3d& x12, const Vector3d& x21, const Vector3d& x22);
	double GetLineLineDistance(const Vector3d& x11, const Vector3d& x12, const Vector3d& x21, const Vector3d& x22);
}