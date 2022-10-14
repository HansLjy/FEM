//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_SHAPE_H
#define FEM_SHAPE_H

#include "EigenAll.h"
#include "Object.h"

class Shape {
public:
    virtual void GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const = 0;
    virtual ~Shape() = default;
    BASE_DECLARE_CLONE(Shape)
};

#endif //FEM_SHAPE_H
