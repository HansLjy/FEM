//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_RIGIDBODY_H
#define FEM_RIGIDBODY_H

#include "Object.h"
#include "Shape.h"

// Quaternion-based rigid body implementation
// See the lecture note https://graphics.pixar.com/pbm2001/pdf/notesg.pdf
class RigidBody : public Object {
public:
    RigidBody(const Shape& shape, double mass, const Matrix3d& inertia, const VectorXd& x, const VectorXd& v): Object() {

    }

    void GetMass(SparseMatrixXd &mass) const override;
    void GetMass(COO &coo, int x_offset, int y_offset) const override;

protected:
    Shape* shape;       // shape

    double _mass;       // mass
    Matrix3d _I;        // inertial tensor
    Matrix3d _Iinv;     // inverse of inertial tensor
};

#endif //FEM_RIGIDBODY_H
