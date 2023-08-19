//
// Created by hansljy on 10/8/22.
//

#pragma once

#include "Pattern.h"
#include "EigenAll.h"


template <class Data>
class ExternalForce {
public:
    // <- external force in **local** coordinate system
    virtual VectorXd GetExternalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const = 0;
    
    // <- sum / integration of current force in **local** coordinate system
    virtual Vector3d GetTotalForce(const Data* data, const Matrix3d& rotation, const Vector3d& position) const = 0;

    // <- sum / integration of f * x^T in **local** coordinate system
    virtual Matrix3d GetTotalForceAffineTorque(const Data* data, const Matrix3d& rotation, const Vector3d& position) const = 0;

    virtual ~ExternalForce() = default;
};