#include "RenderShape.h"

void ReducedRenderShape::GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const {
	const auto& reduced_obj = dynamic_cast<const ReducedObject&>(object);
	reduced_obj._proxy->GetRenderShape(vertices, topos);
}

#include "FixedShape/Rectangle.h"

BEGIN_DEFINE_XXX_FACTORY(FixedRenderShape)
	ADD_PRODUCT("rectangle", RectangleRenderShape)
END_DEFINE_XXX_FACTORY