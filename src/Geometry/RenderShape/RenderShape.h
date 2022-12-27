//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_RENDERSHAPE_H
#define FEM_RENDERSHAPE_H

#include "EigenAll.h"
#include "Object.h"

class RenderShape {
public:
    virtual void GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const = 0;
    virtual ~RenderShape() = default;
};

class ReducedRenderShape : public RenderShape {
public:
	ReducedRenderShape() {}
	void GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const override;
};

#endif //FEM_RENDERSHAPE_H
