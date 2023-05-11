//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_RENDERSHAPE_H
#define FEM_RENDERSHAPE_H

#include "EigenAll.h"
#include "JsonUtil.h"
#include <functional>

/* Render Shape Policy */
class RenderShape {
public:
	template<class Object> void GetSurface(const Object* obj, MatrixXd &vertices, MatrixXi &topos) const;
};

class SampledRenderShape : public RenderShape {
public:
	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const;
};

class ProxyRenderShape : public RenderShape {
public:
	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const;
};

#include "FixedShape/FixedShape.hpp"
class FixedRenderShape : public RenderShape {
public:
	FixedRenderShape(const json& config) : FixedRenderShape(
		FixedShapeFactory::Instance()->GetVertices(config["type"], config),
		FixedShapeFactory::Instance()->GetFaceTopo(config["type"], config)
	) {}
    FixedRenderShape(const MatrixXd& vertices, const MatrixXi& topos) : _vertices(vertices), _topos(topos) {}
	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {
        vertices = _vertices;
        topos = _topos;
    }

protected:
    MatrixXd _vertices;
    MatrixXi _topos;
};

template<class Object>
void SampledRenderShape::GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {
	vertices = StackVector<double, 3>(obj->_x);
	topos = obj->_face_topo;
}

template<class Object>
void ProxyRenderShape::GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {
	obj->_proxy->GetSurface(vertices, topos);
}

#define PROXY_RENDER_SHAPE(RenderShape) \
    void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override {\
		return RenderShape::GetSurface(this, vertices, topos);\
	} \
	friend RenderShape;

#endif //FEM_RENDERSHAPE_H
