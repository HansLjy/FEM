#include "RenderShape.h"
#include "DecomposedObject.h"

void ReducedRenderShape::Bind(const Object &obj) {
	auto reduced_obj = dynamic_cast<const ReducedObject*>(&obj);
	_proxy_render_shape = reduced_obj->_proxy->_render_shape;
}

void ReducedRenderShape::GetSurface(MatrixXd &vertices, MatrixXi &topos) const {
	_proxy_render_shape->GetSurface(vertices, topos);
}

void DecomposedRenderShape::Bind(const Object &obj) {
	auto decomposed_obj = dynamic_cast<const RigidDecomposedObject*>(&obj);
	_proxy_render_shape = decomposed_obj->_proxy->_render_shape;
}

void DecomposedRenderShape::GetSurface(MatrixXd &vertices, MatrixXi &topos) const {
	_proxy_render_shape->GetSurface(vertices, topos);
}

#include "FixedShape/Rectangle.h"

BEGIN_DEFINE_XXX_FACTORY(FixedRenderShape)
	ADD_PRODUCT("rectangle", RectangleRenderShape)
END_DEFINE_XXX_FACTORY