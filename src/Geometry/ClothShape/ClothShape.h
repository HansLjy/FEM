//
// Created by hansljy on 10/27/22.
//

#ifndef FEM_CLOTHSHAPE_H
#define FEM_CLOTHSHAPE_H

#include "Shape.h"

class ClothShape : public Shape {
public:
    void GetSurface(const Object &object, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const override;
    ~ClothShape() override = default;

    DERIVED_DECLARE_CLONE(Shape)
};

#endif //FEM_CLOTHSHAPE_H
