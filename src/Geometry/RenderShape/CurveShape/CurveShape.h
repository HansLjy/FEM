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
	void Bind(const Object &obj) override;
    void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override;

public:
	const Curve* _curve;
    const double _radius;
};

#endif //FEM_CURVESHAPE_H
