#pragma once

#include "EigenAll.h"

namespace GeometryUtil {
    //<- cotangent of the smaller angle between a and b
    double GetCot(const Vector3d& a, const Vector3d& b);

	//<- return alpha where x1 + alpha(x2 - x1) is the closest point to c on x1->x2
	double GetClosestPoint(const Vector3d& x1, const Vector3d& x2, const Vector3d& c);

	//<- return alpha, beta where x1 + alpha(x2 - x1) + beta(x3 - x1) is the closest point to c on the plane(x1, x2, x3)
	Vector2d GetClosestPoint(const Vector3d& x1, const Vector3d& x2, const Vector3d& x3, const Vector3d& c);
}