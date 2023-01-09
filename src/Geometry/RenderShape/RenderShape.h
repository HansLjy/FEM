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

class FixedRenderShape : public RenderShape {
public:
    FixedRenderShape(const MatrixXd& vertices, const MatrixXi& topos) : _vertices(vertices), _topos(topos) {}
    void GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const override {
        vertices = _vertices;
        topos = _topos;
    }

protected:
    MatrixXd _vertices;
    MatrixXi _topos;
};

DECLARE_XXX_FACTORY(FixedRenderShape)

#endif //FEM_RENDERSHAPE_H
