//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_CURVESHAPE_H
#define FEM_CURVESHAPE_H

#include "RenderShape/RenderShape.h"
#include "Object/Curve.h"

class CurveShape : public RenderShape {
public:
    CurveShape(double radius = 0.1);
    void GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const override;

    DERIVED_DECLARE_CLONE(RenderShape)

public:
    const double _radius;
};

#endif //FEM_CURVESHAPE_H
