#pragma once

#include "EigenAll.h"

namespace GeometryUtil {
    //<- cotangent of the smaller angle between a and b
    double GetCot(const Vector3d& a, const Vector3d& b);
}