//
// Created by hansljy on 10/7/22.
//

#include "RigidBody.h"


void RigidBody::GetMass(SparseMatrixXd &mass) const {
    COO coo;
    GetMass(coo, 0, 0);
    mass.setFromTriplets(coo.begin(), coo.end());
}

void RigidBody::GetMass(COO &coo, int x_offset, int y_offset) const {
    Quaterniond q(_x.block<4, 1>(3, 0));
    Matrix3d Ic = q.matrix() * _I * q.matrix().transpose();
    for (int i = 0; i < 3; i++) {
        coo.push_back(Tripletd(x_offset + i, y_offset + i, _mass));
    }
    x_offset += 3;
    y_offset += 3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            coo.push_back(Tripletd(x_offset + i, y_offset + j, Ic(i, j)));
        }
    }
}

