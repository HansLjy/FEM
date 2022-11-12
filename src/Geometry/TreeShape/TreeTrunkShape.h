//
// Created by hansljy on 11/11/22.
//

#ifndef FEM_TREETRUNKSHAPE_H
#define FEM_TREETRUNKSHAPE_H

#include "Shape.h"

class TreeTrunkShape : public Shape {
public:
    TreeTrunkShape(double radius_max, double radius_min);
    void GetSurface(const Object &object, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const override;

    DERIVED_DECLARE_CLONE(Shape)

protected:
    const double _radius_max;
    const double _radius_min;

};

#endif //FEM_TREETRUNKSHAPE_H
