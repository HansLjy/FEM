//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_RENDERSHAPE_H
#define FEM_RENDERSHAPE_H

#include "EigenAll.h"
#include "Object.h"

class RenderShape {
public:
	virtual void Bind(const Object& obj) = 0;
    virtual void GetSurface(MatrixXd &vertices, MatrixXi &topos) const = 0;
    virtual ~RenderShape() = default;
};

class SampledRenderShape : public RenderShape {
public:
	void Bind(const Object &obj) override;
	void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override;

protected:
	const SampledObject* _sampled_object;
};

class ReducedRenderShape : public RenderShape {
public:
	ReducedRenderShape() {}
	void Bind(const Object &obj) override;
	void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override;

protected:
	const RenderShape* _proxy_render_shape;
};

class DecomposedRenderShape : public RenderShape {
public:
	void Bind(const Object &obj) override;
	void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override;

protected:
	const RenderShape* _proxy_render_shape;
};

class FixedRenderShape : public RenderShape {
public:
    FixedRenderShape(const MatrixXd& vertices, const MatrixXi& topos) : _vertices(vertices), _topos(topos) {}
	void Bind(const Object &obj) override {}
    void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override {
        vertices = _vertices;
        topos = _topos;
    }

protected:
    MatrixXd _vertices;
    MatrixXi _topos;
};

DECLARE_XXX_FACTORY(FixedRenderShape)

#endif //FEM_RENDERSHAPE_H
