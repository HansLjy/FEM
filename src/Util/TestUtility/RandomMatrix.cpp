//
// Created by hansljy on 10/28/22.
//

#include "RandomMatrix.h"
#include <random>

Matrix3d GetRandomRotationMatrix() {
    rand(); // in case this is the initial random number
    Vector3d axis = Vector3d::Random().normalized();
    double angle = rand();
    Matrix3d R;
    R = AngleAxisd(angle, axis);
    return R;
}